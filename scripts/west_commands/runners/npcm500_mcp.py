# Copyright (c) 2026 Nuvoton Technology Corporation.
#
# SPDX-License-Identifier: Apache-2.0

'''Program the NPCM500 MCP SPI flash.

The flash sits behind FIU2 and no debug tool ships a driver for it, so
both paths run a board programmer on the Caliptra MCU, which drives FIU2
UMA, and load the image into MCU SRAM 64 KB at a time:

- openocd: the board's mcp_flash_program.tcl with mcp_stub.bin.
- jlink: the board's SEGGER flash loader jlink/NPCM500_MCP.FLM, called
  by mcp_jlink.py as J-Link would. J-Link's own RISC-V support needs a
  V11 or later probe, so mcp_jlink.py drives the debug module over the
  J-Link library's raw JTAG API, which works with any J-Link.

Both halt the MCU and overwrite its SRAM, then reset the Caliptra
subsystem so its ROM boots the new image (west flash passes --reset by
default). With --no-reset the MCU stays halted until a power cycle.
'''

import os
import shutil
import sys
from pathlib import Path

from runners.core import RunnerCaps, ZephyrBinaryRunner
from runners.jlink import DEFAULT_JLINK_EXE

MCP_SECTOR = 0x1000
JLINK_DEFAULT_KHZ = 4000


class Npcm500McpBinaryRunner(ZephyrBinaryRunner):
    '''Runner front-end for NPCM500 MCP flash programming.'''

    def __init__(self, cfg, tool='openocd', config=None, speed=None,
                 offset=0, commander=DEFAULT_JLINK_EXE, reset=False):
        super().__init__(cfg)
        self.support = Path(cfg.board_dir) / 'support'
        self.file = cfg.file
        self.tool = tool
        self.config = config or ['interface/jlink.cfg']
        self.speed = speed
        self.offset = offset
        self.openocd = cfg.openocd or 'openocd'
        self.openocd_search = cfg.openocd_search or []
        self.commander = commander
        self.reset = reset

    @classmethod
    def name(cls):
        return 'npcm500_mcp'

    @classmethod
    def capabilities(cls):
        return RunnerCaps(commands={'flash'}, file=True, reset=True)

    @classmethod
    def do_add_parser(cls, parser):
        parser.add_argument('--tool', choices=['openocd', 'jlink'],
                            default='openocd',
                            help='programming tool (default openocd)')
        parser.add_argument('--config', action='append',
                            help='''OpenOCD adapter config, default
                            interface/jlink.cfg; may be given multiple
                            times''')
        parser.add_argument('--speed', type=int,
                            help=f'''JTAG kHz (default: caliptra_mcu.cfg's
                            for openocd, {JLINK_DEFAULT_KHZ} for jlink)''')
        parser.add_argument('--offset', default=0,
                            type=lambda s: int(s, 0),
                            help='MCP flash offset, 4 KB aligned (default 0)')
        parser.add_argument('--commander', default=DEFAULT_JLINK_EXE,
                            help=f'J-Link Commander, default {DEFAULT_JLINK_EXE}')

    @classmethod
    def do_create(cls, cfg, args):
        if args.offset % MCP_SECTOR:
            raise ValueError(f'--offset {args.offset:#x} is not 4 KB aligned')
        return Npcm500McpBinaryRunner(cfg, args.tool, args.config, args.speed,
                                      args.offset, args.commander,
                                      bool(args.reset))

    def do_run(self, command, **kwargs):
        if self.file is None:
            raise ValueError('no image: pass --file (board.cmake sets the '
                             '<KERNEL_BIN_NAME>_flash.bin image)')
        image = Path(self.file)
        if not image.is_file():
            raise ValueError(f'image {image} not found')

        self.logger.info(f'Flashing {image} to MCP flash @ {self.offset:#x} '
                         f'with {self.tool} via the Caliptra MCU')
        flash = self.flash_openocd if self.tool == 'openocd' else self.flash_jlink
        flash(image.resolve().as_posix())

    def flash_openocd(self, image):
        self.require(self.openocd)
        cmd = [self.openocd, '-s', str(self.support)]
        for path in self.openocd_search:
            cmd += ['-s', path]
        for c in self.config + ['caliptra_mcu.cfg']:
            cmd += ['-f', c]
        if self.speed:
            cmd += ['-c', f'adapter speed {self.speed}']
        # Flash-only session: no servers, so a running debug session or a
        # stale OpenOCD holding the default ports does not get in the way.
        for port in ('gdb_port', 'tcl_port', 'telnet_port'):
            cmd += ['-c', f'{port} disabled']
        # background polling causes DMI timing errors on the VeeR TAP
        cmd += ['-f', 'mcp_flash_program.tcl', '-c', 'init', '-c', 'poll off',
                '-c', 'halt',
                '-c', f'npcm500_mcp_write_fast {image} {self.offset:#x}']
        if self.reset:
            cmd += ['-c', 'npcm500_mcp_reset']
        cmd += ['-c', 'shutdown']
        self.check_call(cmd)

    def flash_jlink(self, image):
        cmd = [sys.executable, str(self.support / 'mcp_jlink.py'),
               '--speed', str(self.speed or JLINK_DEFAULT_KHZ)]
        # Use the J-Link library installed with --commander
        commander = shutil.which(self.commander)
        if commander:
            lib_dir = Path(os.path.realpath(commander)).parent
            for name in ('libjlinkarm.so', 'JLink_x64.dll'):
                if (lib_dir / name).is_file():
                    cmd += ['--jlink-lib', str(lib_dir / name)]
                    break
        if self.reset:
            cmd += ['--reset']
        cmd += ['write', image, f'{self.offset:#x}']
        self.check_call(cmd)
