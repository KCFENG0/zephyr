# SPDX-License-Identifier: Apache-2.0

# Local divergence from upstream: this tree does not auto-discover syscall
# headers, so the class header is registered explicitly as the other driver
# classes here do. Drop this line when the tree catches up.
zephyr_syscall_header(${ZEPHYR_BASE}/include/zephyr/drivers/otp.h)

zephyr_library()

# zephyr-keep-sorted-start
zephyr_library_sources_ifdef(CONFIG_OTP_NPCM otp_npcm.c)
# zephyr-keep-sorted-stop
