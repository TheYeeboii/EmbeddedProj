#!/bin/sh
# Format our C code to BARR-C:2018 layout: clang-format, then the fixes it
# cannot express (constant-left ==, break aligned with case, EOF marker...).
# Usage: tools/barr_c_format.sh [files...]   (default: app_program + tests)
set -eu
cd "$(dirname "$0")/.."
CLANG_FORMAT=${CLANG_FORMAT:-clang-format}
if [ "$#" -eq 0 ]; then
    set -- firmware/app_program/*.c firmware/app_program/*.h tests/*.c
fi
"$CLANG_FORMAT" -i --style=file:firmware/app_program/.clang-format "$@"
python3 tools/barr_c_fix.py "$@" >/dev/null
