#!/bin/sh
# Format our C code to BARR-C:2018 layout: clang-format, then the fixes it
# cannot express (constant-left ==, break aligned with case, EOF marker...).
# Usage: tools/barr_c_format.sh [files...]   (default: app_program + testing)
set -eu
cd "$(dirname "$0")/.."
CLANG_FORMAT=${CLANG_FORMAT:-clang-format}
if [ "$#" -eq 0 ]; then
    set -- app_program/*.c app_program/*.h app_program/subsystem*/*.c \
        app_program/subsystem*/*.h testing/*/*.c
fi
"$CLANG_FORMAT" -i --style=file:app_program/.clang-format "$@"
python3 tools/barr_c_fix.py "$@" >/dev/null
