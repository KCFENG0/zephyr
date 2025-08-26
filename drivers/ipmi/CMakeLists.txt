# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_IPMI_KCS_NPCM	kcs_npcm.c)
