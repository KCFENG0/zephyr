# Copyright (c) 2026 Nuvoton Technology Corporation.
#
# SPDX-License-Identifier: Apache-2.0

'''Program the NPCM500 MCP SPI flash.

The flash sits behind FIU2 and no debug tool ships a driver for it, so
OpenOCD runs the board's mcp_flash_program.tcl, which loads mcp_stub.bin
onto the Caliptra MCU to drive FIU2 UMA and feeds the image through MCU
SRAM 64 KB at a time.

It halts the MCU and overwrites its SRAM, then resets the Caliptra
subsystem so its ROM boots the new image (west flash passes --reset by
default). With --no-reset the MCU stays halted until a power cycle.
'''

from pathlib import Path

from runners.core import RunnerCaps, ZephyrBinaryRunner

MCP_SECTOR = 0x1000


class Npcm500McpBinaryRunner(ZephyrBinaryRunner):
    '''Runner front-end for NPCM500 MCP flash programming.'''

    def __init__(self, cfg, config=None, speed=None, offset=0,
                 reset=False):
        super().__init__(cfg)
        self.support = Path(cfg.board_dir) / 'support'
        self.file = cfg.file
        self.config = config or ['interface/jlink.cfg']
        self.speed = speed
        self.offset = offset
        self.openocd = cfg.openocd or 'openocd'
        self.openocd_search = cfg.openocd_search or []
        self.reset = reset

    @classmethod
    def name(cls):
        return 'npcm500_mcp'

    @classmethod
    def capabilities(cls):
        return RunnerCaps(commands={'flash'}, file=True, reset=True)

    @classmethod
    def do_add_parser(cls, parser):
        parser.add_argument('--config', action='append',
                            help='''OpenOCD adapter config, default
                            interface/jlink.cfg; may be given multiple
                            times''')
        parser.add_argument('--speed', type=int,
                            help="JTAG kHz (default: caliptra_mcu.cfg's)")
        parser.add_argument('--offset', default=0,
                            type=lambda s: int(s, 0),
                            help='MCP flash offset, 4 KB aligned (default 0)')

    @classmethod
    def do_create(cls, cfg, args):
        if args.offset % MCP_SECTOR:
            raise ValueError(f'--offset {args.offset:#x} is not 4 KB aligned')
        return Npcm500McpBinaryRunner(cfg, args.config, args.speed,
                                      args.offset, bool(args.reset))

    def do_run(self, command, **kwargs):
        if self.file is None:
            raise ValueError('no image: pass --file (board.cmake sets the '
                             '<KERNEL_BIN_NAME>_flash.bin image)')
        image = Path(self.file)
        if not image.is_file():
            raise ValueError(f'image {image} not found')

        self.logger.info(f'Flashing {image} to MCP flash @ {self.offset:#x} '
                         'via the Caliptra MCU')
        self.flash_openocd(image.resolve().as_posix())

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

