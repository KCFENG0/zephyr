#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Program the NPCM500 MCP flash with a SEGGER J-Link on the Caliptra MCU (VeeR EL2).

J-Link's native RISC-V support requires a hardware V11 or later probe. To support any
J-Link probe, this script drives the JTAG DTM and RISC-V debug module directly over the
J-Link DLL's raw JTAG API (JLINKARM_JTAG_StoreGetRaw) and invokes the SEGGER flash loader
(jlink/NPCM500_MCP.FLM):

  - jlink/NPCM500_MCP.FLM (PrgCode, PrgData) is loaded to its work RAM and the image to MCU
    SRAM by system bus access (SBA), many DMI scans per USB transfer;
  - SEGGER_FL_Prepare/Erase/Program/Verify/Restore get their arguments in a0..a3, sp and ra by
    abstract register access and return to an ebreak; per 64KB, Erase then Program, which
    reads back through the DRA window and re-reads any mismatch through FIU2 UMA (returns 2);
  - each 64KB in SRAM is checked against the file by the loader's mcp_crc32 (not a SEGGER
    entry), so SBA writes need no read back; check runs SEGGER_FL_Verify over the image.

The MCU is halted and write and check overwrite its SRAM. With --reset, a command that succeeds
then resets the Caliptra subsystem, whose ROM boots the image in flash; otherwise power-cycle.

Normally run by `west flash --tool jlink` (npcm500_mcp runner). By hand:
  mcp_jlink.py id
  mcp_jlink.py [--reset] write build/zephyr/NPCM5_flash.bin [offset]
  mcp_jlink.py check build/zephyr/NPCM5_flash.bin [offset]
"""

import argparse
import contextlib
import ctypes
import os
import shutil
import struct
import sys
import time
import zlib
from pathlib import Path

HERE = Path(__file__).resolve().parent

MCP_DRA = 0xB0000000
MCP_SIZE = 0x200000
MCP_SECTOR = 0x1000
IMG_CHUNK = 0x10000

# Work RAM as in jlink/NPCM500_MCP.xml: SRAM the firmware never executes from. The VeeR
# I-cache caches SRAM and the debugger cannot execute fence.i (progbufsize=0), so a new loader
# build would run stale code: FLM_FENCEI, alone in its 64-byte I-cache line and never holding
# anything else, runs fence.i first.
FLM_FILE = HERE / "jlink" / "NPCM500_MCP.FLM"
FLM_ADDR, FLM_RAM = 0x30090000, 0x8000
FLM_STACK = FLM_ADDR + FLM_RAM - 0x100
FLM_FENCEI = FLM_ADDR + FLM_RAM - 0x80
FLM_RET = FLM_ADDR + FLM_RAM - 4
FLM_PAGE = 256
IMG_BUF = 0x30000000
EBREAK, FENCE_I = 0x00100073, 0x0000100F

# MCI watchdog enables. Only the MCU (not the debug system bus) may write them,
# so these go through the core.
MCI_WDT_EN = (0x200000B0, 0x200000C0)
# MCI GENERIC_OUTPUT_WIRES[0] bit 1 resets the whole Caliptra subsystem, MCU included.
# Also MCU-only.
MCI_GENERIC_OUT0, MCI_SS_RESET = 0x20000408, 1 << 1

FIU2_BASE = 0x40023000
FIU_SPI_FL_CFG, FIU_UMA_CODE, FIU_UMA_DB0 = 0x14, 0x16, 0x1A
FIU_UMA_CTS, FIU_UMA_ECTS, FIU_MSR_IE_CFG, FIU_EXT_DB_CFG = 0x1E, 0x1F, 0x43, 0x48
SPI_RDID = 0x9F

# RISC-V debug spec 0.13
IR_DTMCS, IR_DMI = 0x10, 0x11
DMI_DATA0, DMI_DATA1 = 0x04, 0x05
DMI_DMCONTROL, DMI_DMSTATUS, DMI_ABSTRACTCS, DMI_COMMAND = 0x10, 0x11, 0x16, 0x17
DMI_SBCS, DMI_SBADDRESS0, DMI_SBDATA0 = 0x38, 0x39, 0x3C
DMCONTROL_ACTIVE, DMCONTROL_HALTREQ, DMCONTROL_RESUMEREQ = 1, 1 << 31, 1 << 30
DMSTATUS_ALLHALTED, DMSTATUS_ALLRUNNING = 1 << 9, 1 << 11
DMSTATUS_ALLRESUMEACK, DMSTATUS_ALLHAVERESET = 1 << 17, 1 << 19
SBCS_BUSYERROR, SBCS_READONADDR, SBCS_READONDATA = 1 << 22, 1 << 20, 1 << 15
SBCS_ACCESS32, SBCS_AUTOINC, SBCS_ERROR = 2 << 17, 1 << 16, 7 << 12
REG_RA, REG_SP, REG_A0 = 0x1001, 0x1002, 0x100A
REG_MEPC, REG_MCAUSE, REG_DCSR, REG_DPC = 0x341, 0x342, 0x7B0, 0x7B1
DCSR_EBREAKM = 1 << 15


class McpError(Exception):
    pass


def find_jlink_lib(lib=None):
    if lib:
        return lib
    names = ["JLink_x64.dll", "JLinkARM.dll"] if os.name == "nt" else ["libjlinkarm.so"]
    dirs = []
    exe = shutil.which("JLink" if os.name == "nt" else "JLinkExe")
    if exe:
        dirs.append(Path(os.path.realpath(exe)).parent)
    dirs += [Path("/opt/SEGGER/JLink"), Path("C:/Program Files/SEGGER/JLink")]
    for d in dirs:
        for n in names:
            if (d / n).is_file():
                return str(d / n)
    raise McpError(
        "J-Link library not found: install the J-Link Software Pack or pass --jlink-lib"
    )


class Jtag:
    """Raw JTAG through the J-Link DLL. All scans start and end in Run-Test/Idle."""

    def __init__(self, lib, speed, serial=None):
        self.dll = ctypes.CDLL(lib)
        self.dll.JLINKARM_OpenEx.restype = ctypes.c_char_p
        if serial and self.dll.JLINKARM_EMU_SelectByUSBSN(int(serial)) < 0:
            raise McpError(f"J-Link S/N {serial} not found")
        err = self.dll.JLINKARM_OpenEx(None, None)
        if err:
            raise McpError(f"J-Link open failed: {err.decode(errors='replace')}")
        self.dll.JLINKARM_TIF_Select(0)
        self.dll.JLINKARM_SetSpeed(speed)
        self.shift(0x3F, 0, 7)  # Test-Logic-Reset, then Run-Test/Idle

    def close(self):
        self.dll.JLINKARM_Close()

    def shift(self, tms, tdi, n):
        nb = (n + 7) // 8
        t = (ctypes.c_uint8 * nb).from_buffer_copy(tms.to_bytes(nb, "little"))
        d = (ctypes.c_uint8 * nb).from_buffer_copy(tdi.to_bytes(nb, "little"))
        o = (ctypes.c_uint8 * nb)()
        self.dll.JLINKARM_JTAG_StoreGetRaw(d, o, t, n)
        self.dll.JLINKARM_JTAG_SyncBits()
        return int.from_bytes(bytes(o), "little")

    @staticmethod
    def dr_seg(n, idle):
        """TMS for one DR scan of n bits from/to Run-Test/Idle; data at bit 3."""
        length = 3 + n + 2 + idle
        return 1 | 1 << (2 + n) | 1 << (3 + n), length

    def ir(self, v, irlen=5):
        tms = 0b11 | 1 << (3 + irlen) | 1 << (4 + irlen)
        self.shift(tms, v << 4, 6 + irlen)

    def dr(self, v, n, idle=5):
        tms, length = self.dr_seg(n, idle)
        return (self.shift(tms, v << 3, length) >> 3) & ((1 << n) - 1)


class Dmi:
    """RISC-V debug module over a JTAG DTM (abits from dtmcs)."""

    BATCH = 1024

    def __init__(self, jtag, idle=4):
        self.jtag = jtag
        self.idle = idle
        jtag.ir(IR_DTMCS)
        dtmcs = jtag.dr(0, 32)
        if dtmcs & 0xF != 1:
            raise McpError(
                f"no RISC-V 0.13 DTM on the JTAG chain (dtmcs={dtmcs:#010x}); "
                "is JTAG connected to the Caliptra MCU?"
            )
        self.abits = (dtmcs >> 4) & 0x3F
        self.idle = max(idle, (dtmcs >> 12) & 7)
        jtag.ir(IR_DMI)
        self.nbits = self.abits + 34

    def dmireset(self):
        self.jtag.ir(IR_DTMCS)
        self.jtag.dr(1 << 16, 32)
        self.jtag.ir(IR_DMI)

    def reattach(self):
        """Select DMI and set dmactive again, as after a target reset; returns dmstatus."""
        self.dmireset()
        self.write(DMI_DMCONTROL, DMCONTROL_ACTIVE)
        return self.read(DMI_DMSTATUS)

    def batch(self, ops):
        """ops: [(op, addr, data)], op 1 read / 2 write. Returns the read data, one per op."""
        out = []
        for i in range(0, len(ops), self.BATCH):
            out += self._batch(ops[i : i + self.BATCH])
        return out

    def _batch(self, ops):
        n = self.nbits
        seg_tms, seg_len = Jtag.dr_seg(n, self.idle)
        scans = ops + [(0, 0, 0)]
        tms = tdi = 0
        for i, (op, addr, data) in enumerate(scans):
            pos = i * seg_len
            tms |= seg_tms << pos
            tdi |= (op | data << 2 | addr << 34) << (pos + 3)
        tdo = self.jtag.shift(tms, tdi, seg_len * len(scans))
        res = []
        for i in range(1, len(scans)):
            r = (tdo >> (i * seg_len + 3)) & ((1 << n) - 1)
            if r & 3:
                self.dmireset()
                # Busy: the DM needs more Run-Test/Idle clocks between scans; caller retries
                self.idle = min(self.idle * 2, 64)
                what = "busy" if r & 3 == 3 else "failed"
                raise McpError(
                    f"DMI scan {i}/{len(ops)} {what}, now {self.idle} idle clocks"
                )
            res.append((r >> 2) & 0xFFFFFFFF)
        return res

    def read(self, addr):
        return self.batch([(1, addr, 0)])[0]

    def write(self, addr, data):
        self.batch([(2, addr, data)])

    # -- hart control

    def wait(self, addr, mask, timeout, what):
        end = time.monotonic() + timeout
        while True:
            v = self.read(addr)
            if v & mask:
                return v
            if time.monotonic() > end:
                raise McpError(
                    f"timeout waiting for {what} (reg {addr:#x} = {v:#010x})"
                )

    def halt(self):
        self.write(DMI_DMCONTROL, DMCONTROL_ACTIVE)
        self.write(DMI_DMCONTROL, DMCONTROL_ACTIVE | DMCONTROL_HALTREQ)
        self.wait(DMI_DMSTATUS, DMSTATUS_ALLHALTED, 2, "halt")
        self.write(DMI_DMCONTROL, DMCONTROL_ACTIVE)

    def resume(self):
        self.write(DMI_DMCONTROL, DMCONTROL_ACTIVE | DMCONTROL_RESUMEREQ)
        self.wait(DMI_DMSTATUS, DMSTATUS_ALLRESUMEACK, 2, "resume")
        self.write(DMI_DMCONTROL, DMCONTROL_ACTIVE)

    def abstract(self, command):
        self.write(DMI_COMMAND, command)
        end = time.monotonic() + 1
        while (cs := self.read(DMI_ABSTRACTCS)) & 1 << 12:
            if time.monotonic() > end:
                raise McpError("abstract command stuck busy")
        if cs & 0x700:
            self.write(DMI_ABSTRACTCS, 0x700)
            raise McpError(f"abstract command {command:#010x}: cmderr {(cs >> 8) & 7}")

    def reg_write(self, regno, val):
        self.write(DMI_DATA0, val)
        self.abstract(2 << 20 | 1 << 17 | 1 << 16 | regno)

    def reg_read(self, regno):
        self.abstract(2 << 20 | 1 << 17 | regno)
        return self.read(DMI_DATA0)

    def core_write32(self, addr, val):
        """Memory write by abstract access: goes out through the hart, as an MCU access."""
        self.batch([(2, DMI_DATA0, val), (2, DMI_DATA1, addr)])
        self.abstract(2 << 24 | 2 << 20 | 1 << 16)

    # -- system bus

    def sb_check(self):
        cs = self.read(DMI_SBCS)
        if cs & (SBCS_BUSYERROR | SBCS_ERROR):
            self.write(DMI_SBCS, SBCS_BUSYERROR | SBCS_ERROR)
            raise McpError(f"system bus error, sbcs={cs:#010x}")

    def sb_write32(self, addr, words):
        ops = [
            (2, DMI_SBCS, SBCS_BUSYERROR | SBCS_ERROR | SBCS_ACCESS32 | SBCS_AUTOINC),
            (2, DMI_SBADDRESS0, addr),
        ]
        ops += [(2, DMI_SBDATA0, w) for w in words]
        self.batch(ops)
        self.sb_check()

    def sb_read32(self, addr, n):
        ops = [
            (
                2,
                DMI_SBCS,
                SBCS_BUSYERROR
                | SBCS_ERROR
                | SBCS_ACCESS32
                | SBCS_AUTOINC
                | SBCS_READONADDR
                | SBCS_READONDATA,
            ),
            (2, DMI_SBADDRESS0, addr),
        ]
        ops += [(1, DMI_SBDATA0, 0)] * n
        data = self.batch(ops)[2:]
        self.sb_check()
        return data

    def sb_write8(self, addr, val):
        self.batch(
            [
                (2, DMI_SBCS, SBCS_BUSYERROR | SBCS_ERROR),
                (2, DMI_SBADDRESS0, addr),
                (2, DMI_SBDATA0, val),
            ]
        )
        self.sb_check()

    def sb_read8(self, addr):
        v = self.batch(
            [
                (2, DMI_SBCS, SBCS_BUSYERROR | SBCS_ERROR | SBCS_READONADDR),
                (2, DMI_SBADDRESS0, addr),
                (1, DMI_SBDATA0, 0),
            ]
        )[2]
        self.sb_check()
        return v & 0xFF


def read_flm(path):
    """Return the PrgCode+PrgData image and the symbols of a SEGGER flash loader (ELF32)."""
    elf = Path(path).read_bytes()
    if elf[:6] != b"\x7fELF\x01\x01":
        raise McpError(f"{path}: not a little-endian ELF32 file")
    (shoff,) = struct.unpack_from("<I", elf, 0x20)
    shentsize, shnum, shstrndx = struct.unpack_from("<HHH", elf, 0x2E)
    # name, type, flags, addr, offset, size, link, info, addralign, entsize
    secs = [
        struct.unpack_from("<10I", elf, shoff + i * shentsize) for i in range(shnum)
    ]

    def cstr(off):
        return elf[off : elf.index(b"\0", off)].decode()

    byname = {cstr(secs[shstrndx][4] + sh[0]): sh for sh in secs}
    try:
        img = bytearray()
        for name in ("PrgCode", "PrgData"):
            sh = byname[name]
            img += bytes(sh[3] - len(img))
            nobits = sh[1] == 8
            img += bytes(sh[5]) if nobits else elf[sh[4] : sh[4] + sh[5]]
        sym, names = byname[".symtab"], secs[byname[".symtab"][6]][4]
    except KeyError as e:
        raise McpError(f"{path}: no {e} section") from None
    syms = {}
    for off in range(sym[4], sym[4] + sym[5], 16):
        name, value = struct.unpack_from("<II", elf, off)
        if name:
            syms[cstr(names + name)] = value
    return bytes(img), syms


def retry(tries, fn, *args):
    for t in range(1, tries + 1):
        try:
            return fn(*args)
        except McpError as e:
            if t == tries:
                raise
            print(f"  retry {t}/{tries - 1}: {e}", flush=True)


class Mcp:
    def __init__(self, dmi):
        self.dmi = dmi

    # -- FIU2 UMA, only for the JEDEC ID check (the flash loader does the programming)

    def fiu_rd(self, reg):
        return self.dmi.sb_read8(FIU2_BASE + reg)

    def fiu_wr(self, reg, val):
        self.dmi.sb_write8(FIU2_BASE + reg, val)

    def uma_exec(self, cts):
        self.fiu_wr(FIU_UMA_CTS, cts & 0x7F)
        self.fiu_wr(FIU_UMA_CTS, cts)
        for _ in range(100):
            if not self.fiu_rd(FIU_UMA_CTS) & 0x80:
                return
        raise McpError("UMA EXEC_DONE stuck (FIU2 clock gated?)")

    def jedec_id(self):
        fl_cfg, ie_cfg = self.fiu_rd(FIU_SPI_FL_CFG), self.fiu_rd(FIU_MSR_IE_CFG)
        self.fiu_wr(FIU_SPI_FL_CFG, fl_cfg & 0x3F)
        self.fiu_wr(FIU_MSR_IE_CFG, ie_cfg | 0x08)
        try:
            self.fiu_wr(FIU_UMA_ECTS, 0x06)
            try:
                self.fiu_wr(FIU_UMA_CODE, SPI_RDID)
                self.uma_exec(0xA0)
                out = []
                for _ in range(3):
                    self.uma_exec(0x91)
                    out.append(self.fiu_rd(FIU_UMA_DB0))
            finally:
                self.fiu_wr(FIU_UMA_ECTS, 0x07)
        finally:
            self.fiu_wr(FIU_EXT_DB_CFG, 0)
            self.fiu_wr(FIU_MSR_IE_CFG, ie_cfg)
            self.fiu_wr(FIU_SPI_FL_CFG, fl_cfg)
        print(
            "MCP JEDEC ID = {:#04x} {:#04x} {:#04x} (W25Q16PW: 0xef 0x60 0x15)".format(
                *out
            ),
            flush=True,
        )
        if out[0] in (0x00, 0xFF):
            raise McpError("no MCP flash response on FIU2 CS0")

    # -- flash loader, called the way J-Link calls a SEGGER flash loader

    def flm_load(self):
        img, self.flm_syms = read_flm(FLM_FILE)
        if len(img) > FLM_RAM - 0x1000:
            raise McpError(f"{FLM_FILE.name}: {len(img)} bytes do not fit the work RAM")
        for addr in MCI_WDT_EN:
            self.dmi.core_write32(addr, 0)
        self.dmi.reg_write(REG_DCSR, self.dmi.reg_read(REG_DCSR) | DCSR_EBREAKM)
        retry(3, self.load, FLM_ADDR, img, True)
        self.dmi.sb_write32(FLM_FENCEI, [FENCE_I, EBREAK])
        self.dmi.sb_write32(FLM_RET, [EBREAK])
        self.flm_run(FLM_FENCEI, FLM_FENCEI + 4, "fence.i")

    def load(self, addr, data, readback=False):
        data += b"\xff" * (-len(data) % 4)
        words = [
            int.from_bytes(data[i : i + 4], "little") for i in range(0, len(data), 4)
        ]
        self.dmi.sb_write32(addr, words)
        if readback and self.dmi.sb_read32(addr, len(words)) != words:
            raise McpError(f"SRAM {addr:#010x}: readback differs")

    def flm_call(self, name, *args, timeout=60):
        d = self.dmi
        for i, v in enumerate(args):
            d.reg_write(REG_A0 + i, v & 0xFFFFFFFF)
        d.reg_write(REG_SP, FLM_STACK)
        d.reg_write(REG_RA, FLM_RET)
        self.flm_run(FLM_ADDR + self.flm_syms[name], FLM_RET, name, timeout)
        return d.reg_read(REG_A0)

    def flm_run(self, pc, end, name, timeout=60):
        d = self.dmi
        d.reg_write(REG_DPC, pc)
        d.resume()
        try:
            d.wait(DMI_DMSTATUS, DMSTATUS_ALLHALTED, timeout, name)
        except McpError:
            d.halt()
            raise
        if d.reg_read(REG_DPC) != end:
            raise McpError(
                f"{name} trapped: mcause={d.reg_read(REG_MCAUSE):#x} "
                f"mepc={d.reg_read(REG_MEPC):#010x}"
            )

    def load_chunk(self, image, i, pad):
        chunk = image[i : i + IMG_CHUNK]
        chunk += b"\xff" * (-len(chunk) % pad)
        retry(3, self.load_checked, chunk)
        return chunk

    def load_checked(self, chunk):
        self.load(IMG_BUF, chunk)
        crc = self.flm_call("mcp_crc32", IMG_BUF, len(chunk))
        if crc != zlib.crc32(chunk):
            raise McpError(f"SRAM {IMG_BUF:#010x}: CRC-32 {crc:#010x} != image")

    def flash_chunk(self, addr, n):
        rc = self.flm_call("SEGGER_FL_Erase", addr, 0, -(-n // MCP_SECTOR))
        if rc:
            raise McpError(f"SEGGER_FL_Erase {addr:#010x}: {rc:#x}")
        rc = self.flm_call("SEGGER_FL_Program", addr, n, IMG_BUF)
        if rc != 2:
            bad = self.flm_call("SEGGER_FL_Verify", addr, n, IMG_BUF)
            raise McpError(
                f"SEGGER_FL_Program {addr:#010x}: {rc}, first bad {bad:#010x}"
            )

    @contextlib.contextmanager
    def flm_session(self):
        self.flm_load()
        self.flm_call("SEGGER_FL_Prepare", 0, 0, 0)
        try:
            yield
        except BaseException:
            # Restore talks to the same target that just failed: report the first error
            with contextlib.suppress(McpError):
                self.flm_call("SEGGER_FL_Restore", 0, 0, 0)
            raise
        self.flm_call("SEGGER_FL_Restore", 0, 0, 0)

    def reset(self):
        # The reset ends the abstract write before it completes, so do not wait for the
        # command. It can also leave the TAP off the DMI register (dmstatus reads 0) and the
        # debug module inactive, so select DMI and set dmactive again on every poll. The hart
        # was halted, so havereset or running both prove the reset.
        d = self.dmi
        d.batch(
            [
                (2, DMI_DATA0, MCI_SS_RESET),
                (2, DMI_DATA1, MCI_GENERIC_OUT0),
                (2, DMI_COMMAND, 2 << 24 | 2 << 20 | 1 << 16),
            ]
        )
        end, st = time.monotonic() + 2, 0
        while time.monotonic() < end:
            with contextlib.suppress(McpError):
                st = d.reattach()
            if st & (DMSTATUS_ALLHAVERESET | DMSTATUS_ALLRUNNING):
                print(f"MCU reset (dmstatus {st:#010x}): the new image boots", flush=True)
                return
        raise McpError(f"MCU did not reset (dmstatus {st:#010x})")

    def write(self, image, offset):
        print(f"image: {len(image)} bytes -> MCP flash {offset:#08x}", flush=True)
        self.jedec_id()
        start = time.monotonic()
        with self.flm_session():
            for i in range(0, len(image), IMG_CHUNK):
                chunk = self.load_chunk(image, i, FLM_PAGE)
                retry(3, self.flash_chunk, MCP_DRA + offset + i, len(chunk))
                done = min(i + IMG_CHUNK, len(image))
                print(
                    f"  write {done * 100 // len(image):3d}% ({done:#08x} / {len(image):#08x})",
                    flush=True,
                )
        print(
            f"MCP flash {offset:#08x}+{len(image):#x}: written and verified "
            f"in {time.monotonic() - start:.1f}s",
            flush=True,
        )

    def check(self, image, offset):
        bad = 0
        with self.flm_session():
            for i in range(0, len(image), IMG_CHUNK):
                n = len(self.load_chunk(image, i, 4)[: len(image) - i])
                addr = MCP_DRA + offset + i
                end = self.flm_call("SEGGER_FL_Verify", addr, n, IMG_BUF)
                if end != addr + n:
                    print(
                        f"  {addr - MCP_DRA:#08x}+{n:#x}: first difference at "
                        f"{end - MCP_DRA:#08x}",
                        flush=True,
                    )
                    bad += 1
        if bad:
            raise McpError(f"{bad} 64KB blocks differ")
        print(f"MCP flash {offset:#08x}+{len(image):#x} matches")


def main():
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0], allow_abbrev=False)
    p.add_argument("--speed", type=int, default=4000, help="JTAG kHz (default 4000)")
    p.add_argument("--serial", help="J-Link USB serial number")
    p.add_argument("--jlink-lib", help="path to libjlinkarm.so / JLink_x64.dll")
    p.add_argument(
        "--reset", action="store_true", help="reset the Caliptra subsystem when the command succeeds"
    )
    sub = p.add_subparsers(dest="cmd", required=True)
    sub.add_parser("id", help="read the MCP flash JEDEC ID", allow_abbrev=False)
    for name in ("write", "check"):
        s = sub.add_parser(name, help=f"{name} an image (read back on the MCU)", allow_abbrev=False)
        s.add_argument("file", type=Path)
        s.add_argument("offset", nargs="?", default=0, type=lambda v: int(v, 0))
    args = p.parse_args()

    try:
        image = None
        if args.cmd != "id":
            image = args.file.read_bytes()
            if args.offset % MCP_SECTOR:
                raise McpError(f"offset {args.offset:#x} is not 4KB aligned")
            if not image or args.offset + len(image) > MCP_SIZE:
                raise McpError(
                    f"{len(image):#x} bytes at {args.offset:#x} "
                    f"do not fit the {MCP_SIZE:#x} byte MCP flash"
                )
        jtag = Jtag(find_jlink_lib(args.jlink_lib), args.speed, args.serial)
        try:
            dmi = Dmi(jtag)
            dmi.halt()
            mcp = Mcp(dmi)
            if args.cmd == "id":
                mcp.jedec_id()
            elif args.cmd == "write":
                mcp.write(image, args.offset)
            else:
                mcp.check(image, args.offset)
            if args.reset:
                mcp.reset()
        finally:
            jtag.close()
    except (McpError, OSError) as e:
        print(f"MCP FAILED: {e}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
