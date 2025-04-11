# SPDX-License-Identifier: Apache-2.0

add_subdirectory(common)
add_subdirectory(${SOC_SERIES})

set(SOC_LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/linker.ld CACHE INTERNAL "")
