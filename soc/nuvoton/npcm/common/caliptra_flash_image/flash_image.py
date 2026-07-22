#!/usr/bin/env python3
# Licensed under the Apache-2.0 license
"""Build (and verify) an NPCM flash image from a layout table.

The slot table lives in a layout JSON (layouts/, default a1.json): identifier
and source per slot, plus the FLSH container offset and the partition-table
state. The layout file alone decides which slots a variant ships:

  a1      SoC firmware only; MCU ROM boots it directly
  a1-mcu  adds MCU firmware; MCU ROM loads and jumps to it first
  a2      full Caliptra boot; fills the reserved Caliptra FMC+RT bundle and
          SoC manifest slots from --caliptra-fw and --soc-manifest

The FLSH binary format is inlined from flash_image_tool.py (only the pieces this
tool needs to pack and verify an image); source of truth for the format is
common/flash-image/src/lib.rs, common/config/src/{flash,boot}.rs.

Usage:
  flash_image.py --soc-fw fw.bin --output flash.bin [--mcu-fw mcu.bin] ...
  flash_image.py verify flash.bin [--offset 0x10000 | --layout a1.json]
"""

import argparse
import json
import struct
import sys
from pathlib import Path
from typing import Optional

# --- FLSH format (common/flash-image/src/lib.rs) ---
MAGIC = b"FLSH"  # U32<BigEndian>, on-disk bytes are literal "FLSH"
HEADER_VERSION = 1
FLASH_HEADER_SIZE = 16
IMAGE_HEADER_SIZE = 20

CALIPTRA_FMC_RT_IDENTIFIER = 0x00000000
SOC_MANIFEST_IDENTIFIER = 0x00000001
MCU_RT_IDENTIFIER = 0x00000002
SOC_IMAGES_BASE_IDENTIFIER = 0x00001000

# common/config/src/boot.rs
PARTITION_ID = {"none": 0, "a": 1, "b": 2, "staging": 3}
PARTITION_STATUS = {"invalid": 0, "valid": 1, "bootfailed": 2, "bootsuccessful": 3}
ROLLBACK = {"disable": 0x00000000, "enable": 0x00010000}

DEFAULT_LAYOUT = Path(__file__).resolve().parent / "layouts" / "a1.json"

# --- npcm500 SOC header + flash OTP (common/npcm-image-header, npcm-otp-image) ---
NPCM_IMAGE_TAG = 0x6D63704E      # "Npcm" little-endian
NPCM_IMAGE_VERSION = 1
SOC_IMAGE_HEADER_SIZE = 128      # NpcmImageHeader, repr(C, packed)

OTP_TAG = 0x4F54504E             # "NPT0" little-endian
OTP_VERSION = 1
OTP_RNG0_SIZE = 8
OTP_RNG0_OFFSET = 0
HW_CFG_FIELD_SIZE = 18
HW_CFG_FIELD_OFFSET = 320

FLASH_CONNECTION_BITS = {"internal": 0, "external-ab": 2, "external": 3}

# verify hint: the three soc_image_* entries and what each holds, so a reader
# seeing the auto-generated header/OTP populated is not surprised.
SOC_IMAGE_NOTE = {
    SOC_IMAGES_BASE_IDENTIFIER: "SoC firmware",
    SOC_IMAGES_BASE_IDENTIFIER + 1: "NpcmImageHeader, auto-generated",
    SOC_IMAGES_BASE_IDENTIFIER + 2: "flash OTP, auto-generated",
}


def soc_header_bytes() -> bytes:
    """npcm500 SOC_IMAGE_HEADER: NpcmImageHeader default (tag + version, rest
    zero), 128 bytes packed."""
    return struct.pack("<IH", NPCM_IMAGE_TAG, NPCM_IMAGE_VERSION) \
        + b"\x00" * (SOC_IMAGE_HEADER_SIZE - 6)


def soc_otp_bytes(flash_connection: str, dis_mcp: bool = False) -> bytes:
    """npcm500 soc_get_otp_binary: 24-byte header + 8-byte otp_rng0 (u64 LE,
    byte0 = flash_connection[1:0] | dis_mcp<<4) + 18-byte hw_cfg_field
    (dev_id=0xA, val_dev_id at index 2). 50 bytes (as_bytes_array)."""
    header = struct.pack("<IIIIII", OTP_TAG, OTP_VERSION, OTP_RNG0_SIZE,
                         OTP_RNG0_OFFSET, HW_CFG_FIELD_SIZE, HW_CFG_FIELD_OFFSET)
    rng0 = FLASH_CONNECTION_BITS[flash_connection] | (int(dis_mcp) << 4)
    hw = bytearray(HW_CFG_FIELD_SIZE)
    hw[2] = 0x0A | (1 << 7)  # dev_id=0xA, val_dev_id=true
    return header + struct.pack("<Q", rng0) + bytes(hw)


def parse_size(s: str) -> int:
    """Parse 16M / 1024K / 0x800000 / decimal."""
    s = s.strip()
    if s[:2].lower() == "0x":
        n = int(s, 16)
    else:
        mult = {"k": 1024, "m": 1024 * 1024}.get(s[-1:].lower())
        n = int(s[:-1]) * mult if mult else int(s)
    if n < 0:
        raise ValueError("size must be non-negative")
    return n


def checksum(data: bytes) -> int:
    """Two's-complement of the byte sum, u32. == builder calculate_checksum."""
    return (-sum(data)) & 0xFFFFFFFF


def pad4(data: bytes) -> bytes:
    """Pad to a multiple of 4 with zeros, like builder::flash_image::load_file."""
    return data + b"\x00" * ((-len(data)) % 4)


def image_name(identifier: int) -> str:
    if identifier == CALIPTRA_FMC_RT_IDENTIFIER:
        return "caliptra_fw"
    if identifier == SOC_MANIFEST_IDENTIFIER:
        return "soc_manifest"
    if identifier == MCU_RT_IDENTIFIER:
        return "mcu_runtime"
    if identifier >= SOC_IMAGES_BASE_IDENTIFIER:
        return f"soc_image_{identifier - SOC_IMAGES_BASE_IDENTIFIER}"
    return f"image_0x{identifier:08x}"


def build_flash_image(images: list[tuple[int, bytes]]) -> bytes:
    """Serialize (identifier, raw_data) pairs into a flash image blob.

    Mirrors generate_image_info + FlashImage::to_bytes: header, then one
    ImageHeader per image, then the (4-byte-padded) image payloads.
    """
    n = len(images)
    if n == 0:
        raise ValueError("no images to pack")
    padded = [(ident, pad4(data)) for ident, data in images]

    offset = FLASH_HEADER_SIZE + IMAGE_HEADER_SIZE * n
    infos = bytearray()
    for ident, data in padded:
        body = struct.pack("<IIII", ident, offset, len(data), checksum(data))
        infos += body + struct.pack("<I", checksum(body))
        offset += len(data)

    hdr_wo = MAGIC + struct.pack("<HHI", HEADER_VERSION, n, FLASH_HEADER_SIZE)
    out = bytearray(hdr_wo + struct.pack("<I", checksum(hdr_wo)))
    out += infos
    for _, data in padded:
        out += data
    return bytes(out)


def build_partition_table(active, a_status, b_status, rollback) -> bytes:
    wo = struct.pack(
        "<IHHHHII",
        PARTITION_ID[active],
        0,  # partition_a_boot_count
        PARTITION_STATUS[a_status],
        0,  # partition_b_boot_count
        PARTITION_STATUS[b_status],
        ROLLBACK[rollback],
        0,  # reserved
    )
    return wo + struct.pack("<I", checksum(wo))


def write_at(path: Path, offset: int, blob: bytes):
    """Write blob at offset without truncating the rest of the file (== Rust
    write_to_file: create, truncate(false), seek, write)."""
    path.parent.mkdir(parents=True, exist_ok=True)
    if not path.exists():
        path.write_bytes(b"")
    with open(path, "r+b") as f:
        f.seek(offset)
        f.write(blob)


def prep_output(path: Path, prealloc):
    """0xFF-fill to `prealloc` bytes when set (erased flash), else start empty
    and let slot writes grow it."""
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(b"\xff" * prealloc if prealloc is not None else b"")


# ------------------------------------------------------------------- parse side

def parse_header(raw: bytes, off: int):
    if len(raw) < off + FLASH_HEADER_SIZE:
        raise ValueError("file too small for flash header")
    magic = raw[off:off + 4]
    version, count, hdr_off, hcsum = struct.unpack(
        "<HHI", raw[off + 4:off + 12]
    ) + struct.unpack("<I", raw[off + 12:off + 16])
    if magic != MAGIC:
        raise ValueError(f"bad magic {magic!r} at offset {off:#x} (want {MAGIC!r})")
    if version != HEADER_VERSION:
        raise ValueError(f"unsupported version {version}")
    if checksum(raw[off:off + 12]) != hcsum:
        raise ValueError("header checksum mismatch")
    return count, hdr_off, hcsum


def find_flash_image(raw: bytes):
    """Scan 4-byte-aligned offsets for a valid header (magic + version)."""
    for o in range(0, max(0, len(raw) - FLASH_HEADER_SIZE) + 1, 4):
        if raw[o:o + 4] == MAGIC and raw[o + 4:o + 6] == struct.pack("<H", HEADER_VERSION):
            return o
    return None


def iter_images(raw: bytes, off: int):
    count, hdr_off, _ = parse_header(raw, off)
    for i in range(count):
        p = off + hdr_off + IMAGE_HEADER_SIZE * i
        ident, ioff, size, icsum, ihcsum = struct.unpack("<IIIII", raw[p:p + IMAGE_HEADER_SIZE])
        start = off + ioff
        yield ident, ioff, size, icsum, ihcsum, raw[start:start + size]


def resolve_offset(raw: bytes, offset):
    if offset is not None:
        return offset
    found = find_flash_image(raw)
    if found is None:
        raise ValueError("no flash image magic found")
    return found


# ---------------------------------------------------------------- layout build

def load_layout(path: Path) -> dict:
    layout = json.loads(path.read_text())
    layout["slot_base_offset"] = int(layout["slot_base_offset"], 0)
    return layout


def build_layout_images(layout: dict,
                        sources: dict[str, Optional[bytes]]) -> list[tuple[int, bytes]]:
    """Emit (identifier, data) rows in layout-table order.

    A row with no "source" is a reserved slot and never emitted; every sourced
    row is required. Which variant ships which slots is decided entirely by the
    layout file (a1 / a1-mcu / a2), never by flags. In the A1 layouts the three
    soc_image_* rows are independent
    peers, not nested: soc_fw is the firmware payload; npcm_header is the NPCM
    boot-config header (UART baud, FIU flash clocks, board type -- NOT a
    descriptor of the firmware); flash_otp is the flash OTP. The caller
    synthesizes the last two (soc_header_bytes/soc_otp_bytes) when
    --soc-header/--soc-otp are not given, so `verify` showing them populated
    with no user-supplied file is expected -- the boot ROM requires both.
    """
    images = []
    for entry in layout["images"]:
        src = entry.get("source")
        if src is None:
            continue
        data = sources.get(src)
        if data is None:
            raise ValueError(f"layout slot {entry['name']} needs source '{src}'")
        images.append((int(entry["id"], 0), data))
    return images


def cmd_build(argv):
    parser = argparse.ArgumentParser(prog="flash_image.py", description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--soc-fw", required=True,
                        help="SoC firmware loaded by MCU ROM or MCU firmware")
    parser.add_argument("--mcu-fw",
                        help="MCU firmware (a1-mcu and a2 layouts)")
    parser.add_argument("--caliptra-fw",
                        help="signed Caliptra FMC+RT bundle (a2 layout)")
    parser.add_argument("--soc-manifest",
                        help="SoC manifest (a2 layout)")
    parser.add_argument("--flash-connection",
                        choices=tuple(FLASH_CONNECTION_BITS), default="internal")
    parser.add_argument("--soc-header", help="override generated 128-byte NPCM image header")
    parser.add_argument("--soc-otp", help="override generated 50-byte flash OTP")
    parser.add_argument("--dis-mcp", action="store_true", help="set dis_mcp in generated OTP")
    parser.add_argument("--flash-size", type=parse_size,
                        help="pad the image to this flash size (default: size to content)")
    parser.add_argument("--layout", type=Path, default=DEFAULT_LAYOUT,
                        help="layout JSON describing the slot table (default: layouts/a1.json)")
    parser.add_argument("--output", required=True)
    args = parser.parse_args(argv)

    layout = load_layout(args.layout)
    sources = {
        "soc-fw": Path(args.soc_fw).read_bytes(),
        "mcu-fw": Path(args.mcu_fw).read_bytes() if args.mcu_fw else None,
        "caliptra-fw": Path(args.caliptra_fw).read_bytes() if args.caliptra_fw else None,
        "soc-manifest": Path(args.soc_manifest).read_bytes() if args.soc_manifest else None,
        "soc-header": Path(args.soc_header).read_bytes() if args.soc_header
        else soc_header_bytes(),
        "soc-otp": Path(args.soc_otp).read_bytes() if args.soc_otp
        else soc_otp_bytes(args.flash_connection, args.dis_mcp),
    }
    images = build_layout_images(layout, sources)
    part = layout["partition"]
    output = Path(args.output)
    prep_output(output, args.flash_size)
    write_at(output, layout["slot_base_offset"], build_flash_image(images))
    write_at(output, 0, build_partition_table(
        part["active"], part["a"], part["b"], part["rollback"]))
    print(f"Wrote {layout['version']} flash image ({len(images)} image(s), "
          f"{output.stat().st_size} bytes) -> {output}")


def cmd_verify(argv):
    parser = argparse.ArgumentParser(prog="flash_image.py verify",
                                     description="validate magic/version/checksums")
    parser.add_argument("file")
    parser.add_argument("--offset", type=lambda x: int(x, 0), default=None,
                        help="flash-image offset; omit to locate by magic scan")
    parser.add_argument("--layout", type=Path,
                        help="layout JSON; its slot_base_offset is the expected offset")
    args = parser.parse_args(argv)

    raw = Path(args.file).read_bytes()
    offset = args.offset
    if offset is None and args.layout:
        offset = load_layout(args.layout)["slot_base_offset"]
    off = resolve_offset(raw, offset)
    count, _, _ = parse_header(raw, off)
    print(f"Verify flash image @ {off:#x}: {count} image(s)")
    ok = True
    for ident, ioff, size, icsum, _ihcsum, data in iter_images(raw, off):
        good = len(data) == size and checksum(data) == icsum
        ok = ok and good
        note = SOC_IMAGE_NOTE.get(ident, "")
        print(f"  {image_name(ident):<12} id={ident:#010x} off={ioff:#x} size={size} "
              f"{'OK' if good else 'BAD'}" + (f"  ({note})" if note else ""))
    print("Image is valid!" if ok else "Image has errors.")
    sys.exit(0 if ok else 1)


def main(argv=None):
    argv = sys.argv[1:] if argv is None else list(argv)
    if argv and argv[0] == "verify":
        cmd_verify(argv[1:])
    else:
        cmd_build(argv)


if __name__ == "__main__":
    main()
