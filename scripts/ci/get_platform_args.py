#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0

"""
Generate platform arguments for Twister based on included_platforms.txt

With all in config file:
PLATFORM_ARGS=$(python3 ./scripts/ci/get_platform_args.py)  # Returns: ""
./scripts/twister --subset 1/10 ${TWISTER_COMMON} ${PUSH_OPTIONS}   # Tests all 500+ platforms

With specific platforms:
PLATFORM_ARGS=$(python3 ./scripts/ci/get_platform_args.py)  # Returns: "-p npcm400_evb -p
npcm500_evb"
./scripts/twister --subset 1/10 ${TWISTER_COMMON} ${PUSH_OPTIONS} -p npcm400_evb -p npcm500_evb
 # Tests only those 2 platforms
"""

import os
import sys
from pathlib import Path

def load_included_platforms():
    """Load list of platforms to include in CI testing."""
    zephyr_base = Path(os.environ.get('ZEPHYR_BASE', '.'))
    included_file = zephyr_base / 'scripts' / 'ci' / 'included_platforms.txt'
    included = []
    include_all = False

    if included_file.exists():
        try:
            with open(included_file, 'r') as f:
                for line in f:
                    line = line.strip()
                    # Skip empty lines and comments
                    if line and not line.startswith('#'):
                        if line.lower() == 'all':
                            include_all = True
                        else:
                            included.append(line)

            if include_all:
                return None  # None means include all
            else:
                return included
        except Exception as e:
            print(f"Error reading included platforms file: {e}", file=sys.stderr)
            return None  # Default to include all on error
    else:
        return None  # Default to include all

def main():
    """Generate platform arguments for Twister."""
    platforms = load_included_platforms()

    if platforms is None:
        # Include all platforms - no platform arguments needed
        print("")
    else:
        # Generate -p arguments for each included platform
        args = []
        for platform in platforms:
            args.extend(["-p", platform])
        print(" ".join(args))

if __name__ == "__main__":
    main()
