# Coding standard: BARR-C:2018

Our code in `firmware/app_program/` and the host tests in `tests/` follow the
[Barr Group Embedded C Coding Standard (BARR-C:2018)](https://barrgroup.com/embedded-systems/books/embedded-c-coding-standard).
The micro T-Kernel, the Pico SDK and the other course-template folders under
`firmware/` keep their own style and are not covered.

## Tools

| Tool | What it does |
|---|---|
| `firmware/app_program/.clang-format` | Layout rules: 80 columns, 4-space indent, no tabs, braces on their own lines around every block, aligned declarations, `char * p_x` pointer spacing, `name (args)` in function definitions. |
| `tools/barr_c_fix.py` | What clang-format cannot do: constant on the left of `==`, `for (;;)`, `f` suffix on float constants, `break` aligned with its `case`, the end-of-file marker. |
| `tools/barr_c_format.sh` | Runs both of the above. **Use this instead of plain clang-format**, which would undo the `break` alignment and the blank line after the end-of-file marker. |
| `tools/barr_c_check.py` | Line-based report of rules that need a human: `goto`/`continue`, missing `default`/final `else`, comma declarations, naming prefixes, function length, float suffixes. It is a heuristic and reports some false positives (multi-line parameter lists, enum constants). |

```sh
CLANG_FORMAT=clang-format tools/barr_c_format.sh   # format everything
python3 tools/barr_c_check.py                      # list what is left
```

## What the code follows

- Layout (1.2, 1.3, 3.1-3.6): 80 columns, Allman braces on every block, one
  statement per line, no tabs, LF line endings (`.gitattributes`), end-of-file
  marker.
- Parentheses around every `&&` / `||` operand (1.4.b), constants on the left
  of `==` (8.6.a).
- No `goto` or `continue` (1.7); every `switch` has a `default` (8.3.b); every
  `else if` chain ends with `else` (8.2.d); infinite loops are `for (;;)`.
- Functions under 100 lines (6.2.a): the large state machines were split into
  named helpers (`buddy3_update`, `buddy5_avoidance_task`,
  `robot_controller_handle_command`, `buddy2_update`, the hump detector).
- Single exit point (6.2.c) in our functions.
- Naming (6.1, 7.1): file-scope data `g_`, pointers `p_`/`pp_`, booleans `b_`,
  no names shorter than 3 characters, tasks end in `_task`, interrupt and
  cyclic handlers end in `_isr`.
- Types (5.1, 5.2, 5.4): every enum and struct is a `typedef` ending in `_t`,
  fixed-width integers, `float32_t` from `robot_types.h`, `f` on float
  constants, no `==` / `!=` on floats, NaN handled without self-comparison.
- One header per source file (4.2.a), with an include guard and Doxygen
  comments on every public function (2.2.h).
- Signed and unsigned values are not mixed (5.3.c): the firmware builds
  without warnings under `-Wsign-compare -Wsign-conversion`, and the portable
  modules also under `-Wconversion -Wfloat-equal`.
- The host tests follow the same rules: descriptive names, `g_`/`b_`/`p_`
  prefixes, and test cases split into functions under 100 lines.
- No commented-out code (2.1.c): the old encoder-angle line search is now the
  `BUDDY3_SEARCH_USE_ENCODER_ANGLE` switch.

## Deliberate deviations

| Rule | Where | Why |
|---|---|---|
| 1.1.a C99 | `_Static_assert` in `robot_hal.c`, `buddy3_ir.c` | The firmware is built as GNU C11 by the Pico SDK; compile-time checks are worth more than strict C99. |
| 1.6.a comment every cast | `(UB *)`, `(UINT)`, `(INT)` in `tm_printf` / `tm_sprintf` calls | Hundreds of identical casts required by the kernel's printf API. Each module states once that values are cast to the type their `%u`/`%d` expects; narrowing casts elsewhere carry their own comment. |
| 4.2.a one header per source | `pin_config.h`, `robot_types.h`, `robot_debug.h`, `usb_console_compat.h` | Configuration and type headers with no code behind them. |
| 5.1.c, 6.1.i module prefixes on public names | e.g. `ir_sensor_reading_t`, `robot_diagnostics_init()` in `robot_steering_trace.c` | Public function and type names were kept so the other buddies' code, the tests and the docs did not change. |
| 6.3.a function-like macros | `ROBOT_ROUTINE_LOG(...)`, `APP_TASK(fn, pri)`, kernel `DI()`/`EI()` | Variadic forwarding to `tm_printf` (no `va_list` variant exists), and static task initialisers a function cannot produce. |
| 7.1.m `b_` on Boolean members | Members of public structs (e.g. `buddy3_ir_state_t.line_seen`, `buddy4_imu_state_t`) | Public structure members are part of the interface other modules and tests use. Private structs were renamed. |
| 5.2.a fixed-width types | `UINT`, `INT`, `ID`, `ER`, `TMO` | Required by the micro T-Kernel API they are passed to. |
| 8.5.b no `setjmp`/`longjmp` | `tests/test_buddy5_avoidance.c` | The test harness has to leave the avoidance task's `for (;;)` loop; it runs on the PC only, never on the robot. |
| `usb_console_compat.h` | | Course-template file (different copyright holder); only whitespace was changed. |

## Project documents the standard asks for

- Abbreviations table (1.5.b): [ABBREVIATIONS.md](ABBREVIATIONS.md).
- File templates (4.4.a): [templates/module.h](templates/module.h) and
  [templates/module.c](templates/module.c).

## Known test status

- `tests/test_buddy3_ir_junction.c` fails at line 102 because it expects the
  barcode reader to start unconfigured, but `BUDDY3_BARCODE_MIN_ELEMENT_MM`
  now defaults to 3 mm (decode by default). It passes completely when built
  with `-DBUDDY3_BARCODE_MIN_ELEMENT_MM=0.0f`. The test expectation needs
  updating to the new default.
- `tests/test_buddy5_avoidance.c` is not part of the test scripts and already
  failed before the standards work (its expected arc PWM values are out of
  date).
