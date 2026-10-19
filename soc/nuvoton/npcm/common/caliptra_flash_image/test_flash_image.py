"""Tests for flash_image.py packing, manifest synthesis, and verification."""
import hashlib
from pathlib import Path

import flash_image as fi

LAYOUTS = Path(__file__).parent / "layouts"


def _slot(image: bytes, offset: int, identifier: int) -> bytes:
    """Return the payload bytes of one slot in a packed flash image."""
    for ident, _ioff, size, _fname, _icsum, _ihcsum, data in fi.iter_images(image, offset):
        if ident == identifier:
            assert len(data) == size
            return data
    raise AssertionError(f"slot {identifier:#010x} not present")


def _build_bypass(tmp_path, soc_fw: bytes, format: str = "v3") -> tuple[bytes, int]:
    """Build a bypass image from `soc_fw`; return (raw image, slot offset)."""
    soc = tmp_path / "soc.bin"
    soc.write_bytes(soc_fw)
    mcu = tmp_path / "mcu.bin"
    mcu.write_bytes(b"\x5a" * 64)
    out = tmp_path / "flash.bin"
    fi.main(["--soc-fw", str(soc), "--mcu-fw", str(mcu),
             "--layout", str(LAYOUTS / "bypass.json"),
             "--format", format, "--output", str(out)])
    return out.read_bytes(), fi.load_layout(LAYOUTS / "bypass.json")["slot_base_offset"]


def _build_caliptra(tmp_path, soc_fw: bytes, mcu_fw: bytes = b"\x5a" * 64,
              format: str = "v3") -> tuple[bytes, int]:
    """Build a caliptra image; return (raw image, slot offset)."""
    soc = tmp_path / "soc.bin"
    soc.write_bytes(soc_fw)
    mcu = tmp_path / "mcu.bin"
    mcu.write_bytes(mcu_fw)
    cal = tmp_path / "caliptra.bin"
    cal.write_bytes(b"\xc1" * 128)
    out = tmp_path / "flash.bin"
    fi.main(["--soc-fw", str(soc), "--mcu-fw", str(mcu), "--caliptra-fw", str(cal),
             "--layout", str(LAYOUTS / "caliptra.json"), "--format", format, "--output", str(out)])
    return out.read_bytes(), fi.load_layout(LAYOUTS / "caliptra.json")["slot_base_offset"]


def test_caliptra_manifest_digest_matches_padded_soc_fw(tmp_path):
    """The fw_id=3 digest covers the padded slot bytes, not the raw file.

    Caliptra DMA-reads `size` bytes -- the padded length from the FLSH
    ImageHeader -- and compares SHA-384 against this entry. Hashing the
    unpadded file passes every build-time check and then holds the M7 in
    reset on hardware, so this is the regression test for that trap.
    """
    import soc_manifest as sm

    raw, off = _build_caliptra(tmp_path, b"\xa5" * 5)
    manifest = _slot(raw, off, fi.SOC_MANIFEST_IDENTIFIER)
    packed_soc = _slot(raw, off, fi.SOC_IMAGES_BASE_IDENTIFIER)

    assert packed_soc == fi.pad_to_page(b"\xa5" * 5)
    entries = {e["fw_id"]: e for e in sm.parse_manifest(manifest)["entries"]}
    assert entries[fi.SOC_FW_FW_ID]["digest"] == hashlib.sha384(packed_soc).digest()


def test_caliptra_manifest_binds_mcu_runtime_and_signs(tmp_path):
    """The fw_id=2 entry binds the MCU runtime and all 8 signatures verify."""
    import soc_manifest as sm

    raw, off = _build_caliptra(tmp_path, b"\xa5" * 16, mcu_fw=b"\x5a" * 64)
    manifest = _slot(raw, off, fi.SOC_MANIFEST_IDENTIFIER)

    # the slot is page-padded, so the manifest is a prefix of it, not all of it
    assert len(manifest) == len(fi.pad_to_page(bytes(sm.MANIFEST_SIZE)))
    entries = {e["fw_id"]: e for e in sm.parse_manifest(manifest)["entries"]}
    assert entries[fi.MCU_RT_FW_ID]["digest"] == \
        hashlib.sha384(fi.pad_to_page(b"\x5a" * 64)).digest()
    assert entries[fi.MCU_RT_FW_ID]["staging"] == fi.MCU_RT_STAGING
    assert entries[fi.SOC_FW_FW_ID]["load"] == fi.SOC_FW_LOAD
    assert all(sm.verify_manifest(manifest).values())


def test_caliptra_manifest_digest_matches_padded_mcu_fw(tmp_path):
    """The fw_id=2 digest covers the padded slot bytes, not the raw file.

    mcu-fw is read raw at generation time unless it is padded before hashing.
    With a 9-byte runtime (not a multiple of a page), hashing the raw file
    would sign a digest the packed 256-byte slot never matches, so a
    freshly-built image would fail its own verify.
    """
    import soc_manifest as sm

    raw, off = _build_caliptra(tmp_path, b"\xa5" * 5, mcu_fw=b"\x5a" * 9)
    manifest = _slot(raw, off, fi.SOC_MANIFEST_IDENTIFIER)
    packed_mcu = _slot(raw, off, fi.MCU_RT_IDENTIFIER)

    assert packed_mcu == fi.pad_to_page(b"\x5a" * 9)
    entries = {e["fw_id"]: e for e in sm.parse_manifest(manifest)["entries"]}
    assert entries[fi.MCU_RT_FW_ID]["digest"] == hashlib.sha384(packed_mcu).digest()


def test_bypass_mcu_build_does_not_generate_a_manifest(tmp_path):
    """bypass-mcu has no manifest slot and must not pay for LMS signing."""
    soc, mcu = tmp_path / "soc.bin", tmp_path / "mcu.bin"
    soc.write_bytes(b"\xa5" * 16)
    mcu.write_bytes(b"\x5a" * 16)
    out = tmp_path / "flash.bin"
    fi.main(["--soc-fw", str(soc), "--mcu-fw", str(mcu), "--format", "v3",
             "--layout", str(LAYOUTS / "bypass-mcu.json"), "--output", str(out)])
    idents = [ident for ident, *_ in fi.iter_images(out.read_bytes(), 0x10000)]
    assert fi.SOC_MANIFEST_IDENTIFIER not in idents


def test_layouts_are_chip_neutral():
    """The chip, and with it the container format, comes from the npcm5 build's
    CONFIG_NPCM500_REV_*, so a layout names only its boot flow."""
    import json
    for path in LAYOUTS.glob("*.json"):
        layout = json.loads(path.read_text())
        assert layout["version"] == path.stem
        assert not {"chip", "format"} & layout.keys(), path.name


def test_bypass_soc_builds_from_soc_fw_alone(tmp_path):
    """bypass-soc is the fallback when caliptra-mcu-sw is absent, so it must pack
    with no MCU firmware and ship only the SoC slots."""
    soc = tmp_path / "soc.bin"
    soc.write_bytes(b"\xa5" * 16)
    out = tmp_path / "flash.bin"
    fi.main(["--soc-fw", str(soc), "--format", "v1",
             "--layout", str(LAYOUTS / "bypass-soc.json"), "--output", str(out)])
    idents = [ident for ident, *_ in fi.iter_images(out.read_bytes(), 0x10000)]
    assert idents == [fi.SOC_IMAGES_BASE_IDENTIFIER + i for i in range(3)]


def test_v1_packs_on_4_byte_boundaries_and_binds_only_mcu(tmp_path):
    """v1 is flash_image_tool.py compatible: FLSH header, no page alignment, and a
    manifest that authorizes just the MCU runtime."""
    import soc_manifest as sm
    raw, off = _build_bypass(tmp_path, b"\xa5" * 5, format="v1")
    assert raw[off:off + 4] == b"FLSH"
    for ident, ioff, size, *_ in fi.iter_images(raw, off):
        assert size % 4 == 0
    assert _slot(raw, off, fi.SOC_IMAGES_BASE_IDENTIFIER) == b"\xa5" * 5 + b"\x00" * 3
    entries = sm.parse_manifest(_slot(raw, off, fi.SOC_MANIFEST_IDENTIFIER))["entries"]
    assert [e["fw_id"] for e in entries] == [fi.MCU_RT_FW_ID]


def test_v1_caliptra_manifest_binds_soc_fw(tmp_path):
    """caliptra in v1 must still bind fw_id=3: the MCU runtime asks Caliptra to
    authorize the SoC firmware, and an unbound one holds the M7 in reset."""
    import soc_manifest as sm
    raw, off = _build_caliptra(tmp_path, b"\xa5" * 5, format="v1")
    entries = sm.parse_manifest(_slot(raw, off, fi.SOC_MANIFEST_IDENTIFIER))["entries"]
    assert [e["fw_id"] for e in entries] == [fi.MCU_RT_FW_ID, fi.SOC_FW_FW_ID]


def test_aligned_soc_fw_is_stored_unchanged(tmp_path):
    """pad_to_page() is idempotent: aligned input is byte-for-byte in the slot."""
    aligned = b"\xa5" * fi.FLASH_PAGE_SIZE
    raw, off = _build_bypass(tmp_path, aligned)
    assert _slot(raw, off, fi.SOC_IMAGES_BASE_IDENTIFIER) == aligned


def test_every_image_starts_on_a_page_boundary(tmp_path):
    """The flash DMA only fires on a full page, so an image that starts
    mid-page is read with the wrong bytes at its front."""
    raw, off = _build_caliptra(tmp_path, b"\xa5" * 5, mcu_fw=b"\x5a" * 9)
    assert off % fi.FLASH_PAGE_SIZE == 0
    for ident, ioff, size, *_ in fi.iter_images(raw, off):
        assert ioff % fi.FLASH_PAGE_SIZE == 0, f"{ident:#010x} off={ioff:#x}"
        assert size % fi.FLASH_PAGE_SIZE == 0, f"{ident:#010x} size={size}"


def test_format_is_required(tmp_path):
    """No safe default: 08ba397 in the upstream tool exists because a v1 image
    silently passed as valid and booted an NPCM500 FPGA into a Caliptra
    zero-length DMA. --format must be spelled out every time."""
    soc = tmp_path / "soc.bin"
    soc.write_bytes(b"\xa5" * 8)
    try:
        fi.main(["--soc-fw", str(soc), "--layout", str(LAYOUTS / "bypass.json"),
                 "--output", str(tmp_path / "flash.bin")])
        raise AssertionError("expected SystemExit for a missing --format")
    except SystemExit as exc:
        assert exc.code != 0


def test_v1_and_v3_round_trip(tmp_path):
    """Both formats pack and parse back to the same slot contents."""
    for fmt, magic in (("v1", b"FLSH"), ("v3", b"")):
        raw, off = _build_bypass(tmp_path, b"\xa5" * 8, format=fmt)
        assert raw[off:off + len(magic)] == magic if magic else True
        version, count, *_ = fi.parse_header(raw, off)
        assert version == fi.FORMAT_VERSIONS[fmt]
        assert count == 5  # manifest, mcu_runtime, soc_fw, npcm_header, flash_otp
        assert _slot(raw, off, fi.SOC_IMAGES_BASE_IDENTIFIER) == fi.pad_to(b"\xa5" * 8, fi.FORMAT_ALIGN[fmt] or fi.FLASH_PAGE_SIZE)


def test_v1_flsh_bytes_parse_as_v1_not_a_mangled_version_word(tmp_path):
    """Regression for the 08ba397 incident: a real "FLSH"-magic v1 header must
    not be mistaken for a v3 header whose version word happens to start with
    the same bytes ("FL" as u16 LE == 0x4c46, which is not 1 or 3, but the
    detector must key off the magic, not luck into the right branch)."""
    raw, off = _build_bypass(tmp_path, b"\xa5" * 8, format="v1")
    assert raw[off:off + 4] == b"FLSH"
    version, *_ = fi.parse_header(raw, off)
    assert version == 1


import subprocess
import sys

HERE = Path(__file__).parent


def _squeeze(text: str) -> str:
    """Collapse every whitespace run to one space.

    verify pads labels to a fixed column, so a status line carries a long
    space run that a plain two-space replace would not fully collapse.
    """
    return " ".join(text.split())


def _run_verify(image: Path, layout: str) -> subprocess.CompletedProcess:
    return subprocess.run(
        [sys.executable, str(HERE / "flash_image.py"), "verify", str(image),
         "--layout", str(LAYOUTS / layout)],
        capture_output=True, text=True, cwd=HERE)


def _write_caliptra(tmp_path, soc_fw: bytes) -> Path:
    raw, _off = _build_caliptra(tmp_path, soc_fw)
    out = tmp_path / "caliptra.bin"
    out.write_bytes(raw)
    return out


def _corrupt_slot_byte(image: Path, identifier: int, delta: int) -> None:
    """Flip one byte `delta` into the payload of `identifier`, in place."""
    raw = bytearray(image.read_bytes())
    off = fi.load_layout(LAYOUTS / "caliptra.json")["slot_base_offset"]
    for ident, ioff, _size, *_ in fi.iter_images(bytes(raw), off):
        if ident == identifier:
            raw[off + ioff + delta] ^= 0xFF
            image.write_bytes(bytes(raw))
            return
    raise AssertionError(f"slot {identifier:#010x} not present")


def test_verify_accepts_a_good_caliptra_image(tmp_path):
    result = _run_verify(_write_caliptra(tmp_path, b"\xa5" * 16), "caliptra.json")
    assert result.returncode == 0, result.stdout + result.stderr
    assert "manifest vendor_imc_ecc" in result.stdout
    assert "manifest digest soc_fw" in result.stdout
    assert "Image is valid!" in result.stdout


def test_verify_rejects_a_tampered_manifest_signature(tmp_path):
    """Flipping a signature byte must fail the build, not just the boot.

    Offset 2800 lands in the vendor signature block (O_VENDOR_SIGS == 2708),
    which the FLSH per-image checksum also covers -- so assert on the manifest
    row specifically, not merely on the exit status.
    """
    image = _write_caliptra(tmp_path, b"\xa5" * 16)
    _corrupt_slot_byte(image, fi.SOC_MANIFEST_IDENTIFIER, 2800)
    result = _run_verify(image, "caliptra.json")
    assert result.returncode != 0
    assert "manifest vendor_pub_keys_ecc BAD" in _squeeze(result.stdout)
    assert "Image has errors." in result.stdout


def test_verify_rejects_a_soc_fw_digest_mismatch(tmp_path):
    """The manifest must still describe the firmware actually packed."""
    image = _write_caliptra(tmp_path, b"\xa5" * 16)
    _corrupt_slot_byte(image, fi.SOC_IMAGES_BASE_IDENTIFIER, 0)
    result = _run_verify(image, "caliptra.json")
    assert result.returncode != 0
    assert "manifest digest soc_fw (fw_id=3) BAD" in _squeeze(result.stdout)
