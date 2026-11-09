# SPDX-License-Identifier: Apache-2.0

# The revision names the NPCM500 chip revision, not the board: a1 (taped out,
# default) or a2. It selects npcm500_evb_<rev>.overlay and CONFIG_NPCM500_REV_*.
if(NOT DEFINED BOARD_REVISION)
  set(BOARD_REVISION a1)
elseif(NOT BOARD_REVISION MATCHES "^a[12]$")
  message(FATAL_ERROR "Invalid board revision: ${BOARD_REVISION}\n"
                      "Valid revisions for ${BOARD} are: a1, a2")
endif()
