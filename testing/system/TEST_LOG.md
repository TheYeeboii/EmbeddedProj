# System tests

The finished rover against the demo requirements.

| ID | What is tested | Requirement | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| SYS-01 | barcode read forwards | A–Z, both directions | manual | line follow over a barcode | dashboard shows the letter |
| SYS-02 | barcode read in reverse | both directions | manual | drive over the same barcode the other way | same letter, "read in reverse" |
| SYS-03 | mapped movement at a cross-junction | mapping via MQTT | manual | apply a mapping on the dashboard, run a mapped barcode into a junction | forward / U-turn / left / right as mapped |
| SYS-04 | end of the line | safe stop | manual | line follow off the end of a line | stops within 300 mm, mode Idle |
| SYS-05 | obstacle width | width 10–20 cm reported | manual | 10 cm and 20 cm box on the line | dashboard width close to the real width; rover goes round and rejoins the line |
| SYS-06 | lifted rover stops | safety | manual | lift the rover while line following | motors stop, stay stopped when put down |

*Method:* **host** = automatic test on the laptop, run by
`sh testing/run_barcode_tests.sh` and `sh testing/run_ir_diagnostics_tests.sh`
(each prints one `PASS:` line per test program); **manual** = a person observes
the result on the rover or the dashboard.

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
