# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_C2H_SHELL		c2h_shell.c)
zephyr_library_sources_ifdef(CONFIG_C2H_NPCM		c2h_npcm.c)
