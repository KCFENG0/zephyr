# NPCM500: program the MCP flash over JTAG by driving FIU2 UMA directly from OpenOCD
#
# MCP flash = FIU2 SW_CS0 (PVT role, DEV_NUM=0), W25Q16PW 2MB, reg base 0x40023000, DRA 0xB000_0000.
# Sequence follows zephyr/drivers/flash/flash_npcm_fiu_qspi.c (UMA lock + manual SW_CS +
# opcode/addr/data). Page program uses EXT_DB (datasheet §4.1 EXT_DB_CFG/EXT_DB3_0..F_C): each UMA
# write sends 16 bytes, loaded with four 32-bit writes. The first transfer after CS goes low carries
# code + addr + 16B; after that C_SIZE=1 skips the code and sends 16B of data only
# (C_SIZE semantics: A_SIZE=0 & UMA_ADDR_SIZE=0 & EXT_DB_EN=1).
#
# Normally run by `west flash` (npcm500_mcp runner). By hand, an adapter config comes first, then
# the target config, then this script; the CPU must be halted and no other master may drive FIU2:
#   JTAG -> Caliptra MCU (run npcm500_mcp_id first to confirm the MCU can see FIU2), with any
#   adapter config for the host's JTAG master:
#     openocd -f interface/jlink.cfg -f caliptra_mcu.cfg -f mcp_flash_program.tcl -c init \
#         -c "poll off" -c halt -c "npcm500_mcp_write_fast ./NPCM5_flash.bin 0" -c shutdown
#
#   Program (flash_offset must be 4KB aligned; every sector covered is erased first):
#   ... -c "npcm500_mcp_write_fast ./NPCM5_flash.bin 0" ...                 ;# on-target programmer
#   ... -c "npcm500_mcp_write ./NPCM5_flash.bin 0" ...                      ;# host-fed, slow
#   ... -c "npcm500_mcp_erase_all" ...                                      ;# full chip erase
#   Compare and read:
#   ... -c "npcm500_mcp_check ./NPCM5_flash.bin 0" ...                      ;# fast compare (CRC, locate on mismatch)
#   ... -c "npcm500_mcp_verify ./NPCM5_flash.bin 0 0x1000" ...              ;# compare first 0x1000 bytes of file
#   ... -c "npcm500_mcp_read 0 0x100" ...                                   ;# hexdump to the console
#   ... -c "npcm500_mcp_dump mcp_backup.bin 0 0x200000" ...                 ;# read back to a file (whole 2MB)
#
# npcm500_mcp_write_fast <file> <flash_offset>  (what west flash runs)
#   The stub (mcp_stub.bin, built from src/mcp_stub.c and found next to this script) is loaded at
#   the top of MCU SRAM; the image is loaded 64KB at a time to the bottom of SRAM and the core
#   erases, programs and verifies each 4KB sector. Finally the flash CRC-32 is compared with the
#   file's (taken from the gzip trailer). Image size is limited only by the 2MB flash.
#   This overwrites the firmware in MCU SRAM: add -c npcm500_mcp_reset to boot the new image
#   (west flash does, unless --no-reset), or power-cycle.
#
# npcm500_mcp_reset
#   Reset the whole Caliptra subsystem, MCU included; its ROM boots the image in MCP flash.
#
# npcm500_mcp_write <file> <flash_offset>
#   The OpenOCD host reads the image and feeds it straight into the FIU2 UMA registers, not via
#   target SRAM. Needs exec in OpenOCD's Jim Tcl and od on the host (busybox is enough).

set FIU2_BASE      0x40023000
set MCP_DRA        0xB0000000
set MCP_SIZE       0x200000
set MCP_SECTOR     0x1000
set MCP_PAGE       0x100

set MCP_DIR        [file dirname [info script]]
set IMG_CHUNK      0x10000

# On-target stub at the top 4KB of MCU SRAM (0x3000_0000..0x300A_0000), 64KB chunk buffer at its
# bottom, and the argument/result registers.
set STUB_ADDR      0x3009F000
set IMG_BUF        0x30000000
set STUB_REGS      {a0 a1 a2}

set FIU_SPI_FL_CFG 0x14
set FIU_UMA_CODE   0x16
set FIU_UMA_AB0    0x17
set FIU_UMA_DB0    0x1A
set FIU_UMA_CTS    0x1E
set FIU_UMA_ECTS   0x1F
set FIU_MSR_IE_CFG 0x43
set FIU_EXT_DB_CFG 0x48
set FIU_EXT_DB     0x50

# UMA_CTS: EXEC_DONE(7) DEV_NUM(6)=0 -> PVT/CS0, RD_WR(5), C_SIZE(4), A_SIZE(3), D_SIZE[2:0]
set CTS_EXEC       0x80
set CTS_WR_CODE    0xA0
set CTS_WR_ADDR    0xA8
set CTS_WR_DATA    0xB0
set CTS_RD_1B      0x91
# UMA_ECTS: SW_CS0..2 = 1 (deassert), DEV_NUM_BACK=0, UMA_ADDR_SIZE=0
set ECTS_IDLE      0x07
set ECTS_CS0_LOW   0x06
set IE_UMA_BLOCK   0x08
# EXT_DB_CFG: EXT_DB_EN(5) | D_SIZE_DB=16
set EXT_DB_16B     0x30

set SPI_WREN       0x06
set SPI_RDSR1      0x05
set SPI_RDID       0x9F
set SPI_PP         0x02
set SPI_SE_4K      0x20
set SPI_CE         0xC7
set SR1_BUSY       0x01
set SR1_WEL        0x02

proc fiu_rd {reg}     { return [lindex [[target current] read_memory [expr {$::FIU2_BASE + $reg}] 8 1] 0] }
proc fiu_wr {reg val} { [target current] write_memory [expr {$::FIU2_BASE + $reg}] 8 [list $val] }

# JTAG is far slower than SPI, so the first read normally sees done; the limit only guards a hung FIU
# Datasheet 4.1 EXT_DB example step 8 -> 9: write the transaction fields first, then EXEC_DONE separately
proc uma_exec {cts} {
    fiu_wr $::FIU_UMA_CTS [expr {$cts & ~$::CTS_EXEC}]
    fiu_wr $::FIU_UMA_CTS $cts
    for {set i 0} {$i < 100} {incr i} {
        if {!([fiu_rd $::FIU_UMA_CTS] & $::CTS_EXEC)} { return }
    }
    error "UMA EXEC_DONE stuck (FIU2 clock gated?)"
}

# UMA_AB0..AB2 = addr[7:0], addr[15:8], addr[23:16]
proc uma_set_addr {addr} {
    [target current] write_memory [expr {$::FIU2_BASE + $::FIU_UMA_AB0}] 8 \
        [list [expr {$addr & 0xFF}] [expr {($addr >> 8) & 0xFF}] [expr {($addr >> 16) & 0xFF}]]
}

# Run body in the caller scope with SW_CS0 driven low; CS is released on success and on error
proc with_cs {body} {
    fiu_wr $::FIU_UMA_ECTS $::ECTS_CS0_LOW
    set rc [catch {uplevel 1 $body} out]
    fiu_wr $::FIU_UMA_ECTS $::ECTS_IDLE
    if {$rc} { error $out }
    return $out
}

# Send opcode, then read nread bytes
proc spi_cmd {op {nread 0}} {
    with_cs {
        fiu_wr $::FIU_UMA_CODE $op
        uma_exec $::CTS_WR_CODE
        set out {}
        for {set i 0} {$i < $nread} {incr i} {
            uma_exec $::CTS_RD_1B
            lappend out [fiu_rd $::FIU_UMA_DB0]
        }
        set out
    }
}

proc mcp_sr1 {} { return [lindex [spi_cmd $::SPI_RDSR1 1] 0] }

proc mcp_wren {} {
    spi_cmd $::SPI_WREN
    if {!([mcp_sr1] & $::SR1_WEL)} { error [format "WREN failed, SR1=0x%02x (status-register protected?)" [mcp_sr1]] }
}

proc mcp_wait_ready {tries what} {
    for {set i 0} {$i < $tries} {incr i} {
        if {!([mcp_sr1] & $::SR1_BUSY)} { return }
        sleep 1
    }
    error "MCP flash busy timeout: $what"
}

proc mcp_sector_erase {addr} {
    mcp_wren
    with_cs {
        fiu_wr $::FIU_UMA_CODE $::SPI_SE_4K
        uma_set_addr $addr
        uma_exec $::CTS_WR_ADDR
    }
    mcp_wait_ready 500 [format "erase 0x%06x" $addr]
}

# W25Q16JV tCE typ 5s / max 25s; with sleep 1 per try this waits at least 30s
proc mcp_chip_erase {} {
    mcp_wren
    spi_cmd $::SPI_CE
    mcp_wait_ready 30000 "chip erase"
}

# 16 bytes -> EXT_DB0..F, little-endian 32-bit word x4
proc ext_db_load {b} {
    set w {}
    foreach {b0 b1 b2 b3} $b { lappend w [expr {$b0 | ($b1 << 8) | ($b2 << 16) | ($b3 << 24)}] }
    [target current] write_memory [expr {$::FIU2_BASE + $::FIU_EXT_DB}] 32 $w
}

# data: byte list, length a multiple of 16 and within one page; the first 16B go with code+addr
# On error mcp_session clears EXT_DB_EN; otherwise later code-only writes would send 16 extra bytes
proc mcp_page_program {addr data} {
    set n [llength $data]
    mcp_wren
    with_cs {
        fiu_wr $::FIU_UMA_CODE $::SPI_PP
        uma_set_addr $addr
        fiu_wr $::FIU_EXT_DB_CFG $::EXT_DB_16B
        set cts $::CTS_WR_ADDR
        for {set i 0} {$i < $n} {incr i 16} {
            ext_db_load [lrange $data $i [expr {$i + 15}]]
            uma_exec $cts
            set cts $::CTS_WR_DATA
        }
        fiu_wr $::FIU_EXT_DB_CFG 0
    }
    mcp_wait_ready 100 [format "program 0x%06x" $addr]
}

# UMA needs RD_MODE Normal/Fast (datasheet §4.1) and UMA_BLOCK set (as the driver's uma_lock); restored on exit
proc mcp_session {body} {
    set fl_cfg [fiu_rd $::FIU_SPI_FL_CFG]
    set ie_cfg [fiu_rd $::FIU_MSR_IE_CFG]
    fiu_wr $::FIU_SPI_FL_CFG [expr {$fl_cfg & 0x3F}]
    fiu_wr $::FIU_MSR_IE_CFG [expr {$ie_cfg | $::IE_UMA_BLOCK}]
    fiu_wr $::FIU_UMA_ECTS $::ECTS_IDLE
    set rc [catch {uplevel 1 $body} out]
    fiu_wr $::FIU_EXT_DB_CFG 0
    fiu_wr $::FIU_MSR_IE_CFG $ie_cfg
    fiu_wr $::FIU_SPI_FL_CFG $fl_cfg
    if {$rc} { error $out }
    return $out
}

proc mcp_check_id {} {
    set id [spi_cmd $::SPI_RDID 3]
    echo [format "MCP JEDEC ID = 0x%02x 0x%02x 0x%02x (W25Q16PW: 0xef 0x60 0x15)" {*}$id]
    if {[lindex $id 0] == 0x00 || [lindex $id 0] == 0xFF} { error "no MCP flash response on FIU2 CS0" }
}

proc npcm500_mcp_id {} {
    mcp_run { mcp_session { mcp_check_id } }
}

proc mcp_check_range {offset len} {
    if {$offset < 0 || $len <= 0 || $offset + $len > $::MCP_SIZE} {
        error [format "range 0x%x+0x%x outside MCP flash (0..0x%x)" $offset $len $::MCP_SIZE]
    }
}

# Print every 16KB and at the end
proc mcp_progress {what done total} {
    if {$done % 0x4000 && $done < $total} { return }
    echo [format "  %-7s %3d%% (0x%06x / 0x%06x)" $what [expr {$done * 100 / $total}] $done $total]
}

# Reads go through DRA (0xB000_0000), not UMA; each line: flash offset, 16 bytes hex, ASCII
proc mcp_hexdump {offset len} {
    mcp_check_range $offset $len
    set bytes [mcp_read_bytes $offset $len]
    for {set i 0} {$i < $len} {incr i 16} {
        set hex ""
        set asc ""
        foreach b [lrange $bytes $i [expr {$i + 15}]] {
            append hex [format "%02x " $b]
            append asc [expr {$b >= 0x20 && $b < 0x7F ? [format %c $b] : "."}]
        }
        echo [format "%06x: %-48s %s" [expr {$offset + $i}] $hex $asc]
    }
}

proc npcm500_mcp_erase_all {} {
    mcp_run {
        mcp_session {
            mcp_check_id
            echo "chip erase (2MB, typ 5s / max 25s) ..."
            mcp_chip_erase
        }
    }
    echo "MCP flash erased (all 0xFF)"
}

proc npcm500_mcp_read {offset len} {
    mcp_run { mcp_hexdump $offset $len }
}

proc npcm500_mcp_dump {file offset len} {
    mcp_check_range $offset $len
    dump_image $file [expr {$::MCP_DRA + $offset}] $len
}

# This OpenOCD build does not always print why a -c command failed, so print it here
proc mcp_run {body} {
    if {[catch {uplevel 1 $body} e]} {
        echo "MCP FAILED: $e"
        error $e
    }
}

# ---- on-target programmer (mcp_stub.c) ----

proc stub_reg {r} { return [expr {[lindex [capture "reg $r"] end] & 0xFFFFFFFF}] }

# Arguments go in a0..a2, results come back in the first two.
# entry +0: a0 = mcp_flash_sector(a0, a1, a2), a1 = byte offset of the first differing word
# entry +4: a0 = mcp_crc32(a0, a1); returns {a0 a1}
proc stub_call {entry a0 a1 a2} {
    foreach r $::STUB_REGS v [list $a0 $a1 $a2] { capture "reg $r $v" }
    resume [expr {$::STUB_ADDR + $entry}]
    if {[catch {wait_halt 30000} e]} {
        halt
        error "stub did not reach its breakpoint: $e"
    }
    return [list [stub_reg [lindex $::STUB_REGS 0]] [stub_reg [lindex $::STUB_REGS 1]]]
}

proc stub_load {} {
    set type [[target current] cget -type]
    if {$type ne "riscv"} { error "the on-target stub runs on the Caliptra MCU, not a $type core" }
    mcu_wdt_off
    load_image [file join $::MCP_DIR mcp_stub.bin] $::STUB_ADDR bin
}

# Disable the MCI watchdogs while the core is halted.
# Only the MCU may write these registers, so access them through the hart (abstract access).
proc mcu_wdt_off {} {
    riscv set_mem_access abstract
    mww 0x200000B0 0
    mww 0x200000C0 0
    riscv set_mem_access sysbus abstract
}

# MCI GENERIC_OUTPUT_WIRES[0] bit 1 resets the whole Caliptra subsystem, MCU included; the debug
# module stays up. Only the MCU may write MCI, so this is an abstract memory write through the
# hart. The reset ends the command before it completes, which write_memory reports as a failure,
# so it goes over raw DMI (data0, data1, command) and dmstatus.allhavereset confirms it.
proc mcu_reset {} {
    riscv dmi_write 0x04 2
    riscv dmi_write 0x05 0x20000408
    riscv dmi_write 0x17 0x02210000
    for {set i 0} {$i < 100} {incr i} {
        if {[riscv dmi_read 0x11] & (1 << 19)} {
            echo "MCU reset: the Caliptra subsystem boots the new image"
            return
        }
    }
    error [format "MCU did not reset (dmstatus 0x%08x)" [riscv dmi_read 0x11]]
}

proc rd32 {addr} { return [lindex [[target current] read_memory $addr 32 1] 0] }

proc stub_error {rc a1 dst src} {
    set codes {0xffffffff "UMA EXEC_DONE stuck" 0xfffffffe "flash busy timeout" 0xfffffffd "WREN failed (WEL=0)"}
    set key [format 0x%08x $rc]
    if {[dict exists $codes $key]} { return "stub: [dict get $codes $key]" }
    if {$rc & 0x80000000} { return [format "stub trap: cause=0x%x pc=0x%08x" [expr {$rc & 0xFF}] $a1] }
    return [format "stub verify: %d words differ, first 0x%06x got 0x%08x want 0x%08x" $rc \
        [expr {$dst + $a1}] [rd32 [expr {$::MCP_DRA + $dst + $a1}]] [rd32 [expr {$src + $a1}]]]
}

proc stub_flash_sector {dst src len} {
    lassign [stub_call 0 $dst $src $len] rc a1
    if {$rc} { error [stub_error $rc $a1 $dst $src] }
}

# Load file bytes [i, i+n) into IMG_BUF: shift the base back by i, then min_addr/max_length keep only the IMG_BUF window
proc mcp_load_chunk {file i n} {
    load_image $file [expr {$::IMG_BUF - $i}] bin $::IMG_BUF $::IMG_CHUNK
    set pn [expr {($n + 15) & ~15}]
    if {$pn > $n} { mwb [expr {$::IMG_BUF + $n}] 0xFF [expr {$pn - $n}] }
    return $pn
}

# gzip trailer = CRC-32 (LE) + ISIZE; busybox has no cksum/crc32 but does have gzip/tail/od
proc file_crc32 {file} {
    set b [lrange [lindex [split [exec gzip -c $file | tail -c 8 | od -b] \n] 0] 1 4]
    set crc 0
    foreach o [lreverse $b] { set crc [expr {($crc << 8) | [scan $o %o]}] }
    return $crc
}

proc mcp_write_fast {file offset} {
    if {$offset % $::MCP_SECTOR} { error "flash offset must be 4KB aligned" }
    set len [lindex [exec wc -c $file] 0]
    mcp_check_range $offset $len
    echo [format "image %s: %d bytes" $file $len]

    mcp_session { mcp_check_id }
    stub_load
    echo "load 64KB chunk -> target SRAM, then erase + program + verify on target per 4KB sector ..."
    for {set i 0} {$i < $len} {incr i $::IMG_CHUNK} {
        set pn [mcp_load_chunk $file $i [expr {$len - $i < $::IMG_CHUNK ? $len - $i : $::IMG_CHUNK}]]
        for {set j 0} {$j < $pn} {incr j $::MCP_SECTOR} {
            set n [expr {$pn - $j < $::MCP_SECTOR ? $pn - $j : $::MCP_SECTOR}]
            mcp_retry 3 { stub_flash_sector [expr {$offset + $i + $j}] [expr {$::IMG_BUF + $j}] $n }
        }
        mcp_progress write [expr {$i + $pn < $len ? $i + $pn : $len}] $len
    }

    # The stub only compares against the SRAM copy; finally compare the flash CRC-32
    # with the file CRC-32 to catch load_image transfer errors.
    set want [file_crc32 $file]
    set got [lindex [stub_call 4 $offset $len 0] 0]
    if {$got != $want} { error [format "CRC-32: flash 0x%08x != file 0x%08x (SRAM copy corrupted? rerun)" $got $want] }
    echo [format "MCP flash 0x%06x: %d bytes programmed + verified (CRC-32 0x%08x)" $offset $len $got]
}

# Compare the whole image CRC-32 first (stub on the flash side, gzip on the file side; seconds). Only on a
# mismatch locate it: like a write, load 64KB at a time into target SRAM, stub compares each 4KB sector read-only.
proc mcp_check {file offset} {
    if {$offset % $::MCP_SECTOR} { error "flash offset must be 4KB aligned" }
    set len [lindex [exec wc -c $file] 0]
    mcp_check_range $offset $len
    stub_load

    set want [file_crc32 $file]
    set got [lindex [stub_call 4 $offset $len 0] 0]
    if {$got == $want} {
        echo [format "MCP flash 0x%06x+0x%x matches %s (CRC-32 0x%08x)" $offset $len $file $got]
        return
    }
    echo [format "CRC-32 flash 0x%08x != file 0x%08x, locating ..." $got $want]
    set bad 0
    for {set i 0} {$i < $len} {incr i $::IMG_CHUNK} {
        set pn [mcp_load_chunk $file $i [expr {$len - $i < $::IMG_CHUNK ? $len - $i : $::IMG_CHUNK}]]
        for {set j 0} {$j < $pn} {incr j $::MCP_SECTOR} {
            set n [expr {$pn - $j < $::MCP_SECTOR ? $pn - $j : $::MCP_SECTOR}]
            set dst [expr {$offset + $i + $j}]
            set src [expr {$::IMG_BUF + $j}]
            lassign [stub_call 8 $dst $src $n] rc a1
            if {$rc} { echo "  [stub_error $rc $a1 $dst $src]"; incr bad }
        }
        mcp_progress check [expr {$i + $pn < $len ? $i + $pn : $len}] $len
    }
    error [format "%d 4KB sectors differ (0 = only the SRAM copy/CRC disagreed: rerun)" $bad]
}

proc npcm500_mcp_check {file offset} {
    mcp_run { mcp_check $file $offset }
}

proc npcm500_mcp_write_fast {file offset} {
    mcp_run { mcp_write_fast $file $offset }
}

proc npcm500_mcp_reset {} {
    mcp_run { mcu_reset }
}

proc npcm500_mcp_write {file offset} {
    mcp_run { mcp_write $file $offset }
}

# OpenOCD Jim on a minimal host (e.g. a BMC) may have no binary command and busybox od has no -A/-t; convert od -b -v output
# (each line: address + up to 16 three-digit octal bytes; the last line is only the end address) to a byte list
proc mcp_read_file {file} {
    set bytes {}
    foreach line [split [exec od -b -v $file] \n] {
        foreach tok [lrange $line 1 end] { lappend bytes [scan $tok %o] }
    }
    return $bytes
}

# Read flash via DRA with 32-bit reads to cut JTAG accesses; unaligned head/tail read whole words and trim
proc mcp_read_bytes {offset len} {
    set a0 [expr {$offset & ~3}]
    set nw [expr {($offset + $len - $a0 + 3) / 4}]
    set out {}
    foreach w [[target current] read_memory [expr {$::MCP_DRA + $a0}] 32 $nw] {
        lappend out [expr {$w & 0xFF}] [expr {($w >> 8) & 0xFF}] \
                    [expr {($w >> 16) & 0xFF}] [expr {($w >> 24) & 0xFF}]
    }
    set skip [expr {$offset - $a0}]
    return [lrange $out $skip [expr {$skip + $len - 1}]]
}

# Print only the first 8, count the rest
proc mcp_mismatch {badvar addr want got} {
    upvar 1 $badvar bad
    incr bad
    if {$bad <= 8} { echo [format "  0x%06x: want 0x%02x got 0x%02x" $addr $want $got] }
}

proc mcp_quiet {args} {}

# want: byte list compared byte by byte against flash from offset; report = progress callback every 4KB
proc mcp_compare {want offset {report mcp_quiet}} {
    set len [llength $want]
    mcp_check_range $offset $len
    set bad 0
    for {set i 0} {$i < $len} {incr i $::MCP_SECTOR} {
        set n [expr {$len - $i < $::MCP_SECTOR ? $len - $i : $::MCP_SECTOR}]
        set addr [expr {$offset + $i}]
        foreach g [mcp_read_bytes $addr $n] e [lrange $want $i [expr {$i + $n - 1}]] {
            if {$g != $e} { mcp_mismatch bad $addr $e $g }
            incr addr
        }
        {*}$report [expr {$i + $n}] $len
    }
    if {$bad} { error [format "verify: %d bytes differ in 0x%06x+0x%x" $bad $offset $len] }
}

proc mcp_verify {file offset len} {
    set want [mcp_read_file $file]
    if {$len < 0} { set len [llength $want] }
    if {$len > [llength $want]} { error [format "len 0x%x > file size 0x%x" $len [llength $want]] }
    mcp_compare [lrange $want 0 [expr {$len - 1}]] $offset {mcp_progress verify}
    echo [format "MCP flash 0x%06x+0x%x matches %s" $offset $len $file]
}

# Compare the first len bytes of file with flash from offset; omitted len = whole file
proc npcm500_mcp_verify {file offset {len -1}} {
    mcp_run { mcp_verify $file $offset $len }
}

# One sector: erase + program (inside a UMA session), restore FIU settings, then read back via DRA and compare.
# data <= 4KB and a multiple of 16; the last page may be shorter than 256B, lrange truncates naturally.
proc mcp_write_sector {addr data} {
    mcp_session {
        mcp_sector_erase $addr
        for {set i 0} {$i < [llength $data]} {incr i $::MCP_PAGE} {
            mcp_page_program [expr {$addr + $i}] [lrange $data $i [expr {$i + $::MCP_PAGE - 1}]]
        }
    }
    mcp_compare $data $addr
}

# 100kHz JTAG occasionally corrupts a single access (seen: one EXT_DB word written as 0 in 410K bytes);
# re-erase and reprogram only the failed sector instead of the whole chip
proc mcp_retry {tries body} {
    for {set t 1} {1} {incr t} {
        if {![catch {uplevel 1 $body} e]} { return }
        if {$t >= $tries} { error $e }
        echo "  retry $t/[expr {$tries - 1}]: $e"
    }
}

proc mcp_write {file offset} {
    if {$offset % $::MCP_SECTOR} { error "flash offset must be 4KB aligned" }
    set bytes [mcp_read_file $file]
    set len [llength $bytes]
    mcp_check_range $offset $len
    echo [format "image %s: %d bytes" $file $len]

    # Pad the tail with 0xFF to 16-byte alignment (0xFF is a no-op for NOR)
    while {[llength $bytes] % 16} { lappend bytes 0xFF }
    set plen [llength $bytes]

    mcp_session { mcp_check_id }
    echo "erase + program + verify, per 4KB sector ..."
    for {set i 0} {$i < $plen} {incr i $::MCP_SECTOR} {
        set data [lrange $bytes $i [expr {$i + $::MCP_SECTOR - 1}]]
        mcp_retry 3 { mcp_write_sector [expr {$offset + $i}] $data }
        mcp_progress write [expr {$i + [llength $data]}] $plen
    }
    echo [format "MCP flash 0x%06x: %d bytes programmed + verified" $offset $len]
}
