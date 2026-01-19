# SPDX-License-Identifier: Apache-2.0

add_subdirectory(common)
add_subdirectory(${SOC_SERIES})

set(SOC_LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/linker.ld CACHE INTERNAL "")

# Create default empty snippets-app-sections.ld
# Projects can override by generating this file with content after find_package(Zephyr)
set(SNIPPETS_APP_SECTIONS_DIR ${PROJECT_BINARY_DIR}/include/generated)
file(MAKE_DIRECTORY ${SNIPPETS_APP_SECTIONS_DIR})
file(WRITE ${SNIPPETS_APP_SECTIONS_DIR}/snippets-app-sections.ld
"/* Default empty - override in project CMakeLists.txt after find_package(Zephyr) */\n")
