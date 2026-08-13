"""Tests for flash_image.py packing, manifest synthesis, and verification."""
import hashlib
from pathlib import Path

import flash_image as fi

LAYOUTS = Path(__file__).parent / "layouts"


def _slot(image: bytes, offset: int, identifier: int) -> bytes:
    """Return the payload bytes of one slot in a packed flash image."""
    for ident, _ioff, size, _icsum, _ihcsum, data in fi.iter_images(image, offset):
        if ident == identifier:
            assert len(data) == size
            return data
    raise AssertionError(f"slot {identifier:#010x} not present")


def _build_a1(tmp_path, soc_fw: bytes) -> tuple[bytes, int]:
    """Build an a1 image from `soc_fw`; return (raw image, slot offset)."""
    soc = tmp_path / "soc.bin"
    soc.write_bytes(soc_fw)
    out = tmp_path / "flash.bin"
    fi.main(["--soc-fw", str(soc), "--layout", str(LAYOUTS / "a1.json"),
             "--output", str(out)])
    return out.read_bytes(), fi.load_layout(LAYOUTS / "a1.json")["slot_base_offset"]


def _build_a2(tmp_path, soc_fw: bytes, mcu_fw: bytes = b"\x5a" * 64) -> tuple[bytes, int]:
    """Build an a2 image; return (raw image, slot offset)."""
    soc = tmp_path / "soc.bin"
    soc.write_bytes(soc_fw)
    mcu = tmp_path / "mcu.bin"
    mcu.write_bytes(mcu_fw)
    cal = tmp_path / "caliptra.bin"
    cal.write_bytes(b"\xc1" * 128)
    out = tmp_path / "flash.bin"
    fi.main(["--soc-fw", str(soc), "--mcu-fw", str(mcu), "--caliptra-fw", str(cal),
             "--layout", str(LAYOUTS / "a2.json"), "--output", str(out)])
    return out.read_bytes(), fi.load_layout(LAYOUTS / "a2.json")["slot_base_offset"]


def test_a2_manifest_digest_matches_padded_soc_fw(tmp_path):
    """The fw_id=3 digest covers the padded slot bytes, not the raw file.

    Caliptra DMA-reads `size` bytes -- the padded length from the FLSH
    ImageHeader -- and compares SHA-384 against this entry. Hashing the
    unpadded file passes every build-time check and then holds the M7 in
    reset on hardware, so this is the regression test for that trap.
    """
    import soc_manifest as sm

    raw, off = _build_a2(tmp_path, b"\xa5" * 5)
    manifest = _slot(raw, off, fi.SOC_MANIFEST_IDENTIFIER)
    packed_soc = _slot(raw, off, fi.SOC_IMAGES_BASE_IDENTIFIER)

    assert packed_soc == b"\xa5" * 5 + b"\x00" * 3
    entries = {e["fw_id"]: e for e in sm.parse_manifest(manifest)["entries"]}
    assert entries[fi.SOC_FW_FW_ID]["digest"] == hashlib.sha384(packed_soc).digest()


def test_a2_manifest_binds_mcu_runtime_and_signs(tmp_path):
    """The fw_id=2 entry binds the MCU runtime and all 8 signatures verify."""
    import soc_manifest as sm

    raw, off = _build_a2(tmp_path, b"\xa5" * 16, mcu_fw=b"\x5a" * 64)
    manifest = _slot(raw, off, fi.SOC_MANIFEST_IDENTIFIER)

    assert len(manifest) == sm.MANIFEST_SIZE
    entries = {e["fw_id"]: e for e in sm.parse_manifest(manifest)["entries"]}
    assert entries[fi.MCU_RT_FW_ID]["digest"] == hashlib.sha384(b"\x5a" * 64).digest()
    assert entries[fi.MCU_RT_FW_ID]["staging"] == fi.MCU_RT_STAGING
    assert entries[fi.SOC_FW_FW_ID]["load"] == fi.SOC_FW_LOAD
    assert all(sm.verify_manifest(manifest).values())


def test_a2_manifest_digest_matches_padded_mcu_fw(tmp_path):
    """The fw_id=2 digest covers the padded slot bytes, not the raw file.

    mcu-fw is read raw at generation time unless it is padded before hashing.
    With a 9-byte runtime (not a multiple of 4), hashing the raw file would
    sign a digest the packed 12-byte slot never matches, so a freshly-built
    image would fail its own verify.
    """
    import soc_manifest as sm

    raw, off = _build_a2(tmp_path, b"\xa5" * 5, mcu_fw=b"\x5a" * 9)
    manifest = _slot(raw, off, fi.SOC_MANIFEST_IDENTIFIER)
    packed_mcu = _slot(raw, off, fi.MCU_RT_IDENTIFIER)

    assert packed_mcu == b"\x5a" * 9 + b"\x00" * 3
    entries = {e["fw_id"]: e for e in sm.parse_manifest(manifest)["entries"]}
    assert entries[fi.MCU_RT_FW_ID]["digest"] == hashlib.sha384(packed_mcu).digest()


def test_a1_build_does_not_generate_a_manifest(tmp_path):
    """A1 layouts have no manifest slot and must not pay for LMS signing."""
    raw, off = _build_a1(tmp_path, b"\xa5" * 16)
    idents = [ident for ident, *_ in fi.iter_images(raw, off)]
    assert fi.SOC_MANIFEST_IDENTIFIER not in idents


def test_aligned_soc_fw_is_stored_unchanged(tmp_path):
    """pad4() is idempotent: aligned input is byte-for-byte in the slot."""
    raw, off = _build_a1(tmp_path, b"\xa5" * 8)
    assert _slot(raw, off, fi.SOC_IMAGES_BASE_IDENTIFIER) == b"\xa5" * 8


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


def _write_a2(tmp_path, soc_fw: bytes) -> Path:
    raw, _off = _build_a2(tmp_path, soc_fw)
    out = tmp_path / "a2.bin"
    out.write_bytes(raw)
    return out


def _corrupt_slot_byte(image: Path, identifier: int, delta: int) -> None:
    """Flip one byte `delta` into the payload of `identifier`, in place."""
    raw = bytearray(image.read_bytes())
    off = fi.load_layout(LAYOUTS / "a2.json")["slot_base_offset"]
    for ident, ioff, _size, *_ in fi.iter_images(bytes(raw), off):
        if ident == identifier:
            raw[off + ioff + delta] ^= 0xFF
            image.write_bytes(bytes(raw))
            return
    raise AssertionError(f"slot {identifier:#010x} not present")


def test_verify_accepts_a_good_a2_image(tmp_path):
    result = _run_verify(_write_a2(tmp_path, b"\xa5" * 16), "a2.json")
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
    image = _write_a2(tmp_path, b"\xa5" * 16)
    _corrupt_slot_byte(image, fi.SOC_MANIFEST_IDENTIFIER, 2800)
    result = _run_verify(image, "a2.json")
    assert result.returncode != 0
    assert "manifest vendor_pub_keys_ecc BAD" in _squeeze(result.stdout)
    assert "Image has errors." in result.stdout


def test_verify_rejects_a_soc_fw_digest_mismatch(tmp_path):
    """The manifest must still describe the firmware actually packed."""
    image = _write_a2(tmp_path, b"\xa5" * 16)
    _corrupt_slot_byte(image, fi.SOC_IMAGES_BASE_IDENTIFIER, 0)
    result = _run_verify(image, "a2.json")
    assert result.returncode != 0
    assert "manifest digest soc_fw (fw_id=3) BAD" in _squeeze(result.stdout)
