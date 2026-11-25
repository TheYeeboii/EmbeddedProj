# System tests

The finished rover against the demo requirements.

| ID | What is tested | Requirement | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| SYS-01 | barcode read forwards | A–Z, both directions | manual | line follow over a barcode | dashboard shows the letter |
| SYS-02 | barcode read in reverse | both directions | manual | drive over the same barcode the other way | same letter, "read in reverse" |
| SYS-03 | mapped movement at a cross-junction | mapping via MQTT | manual | apply a mapping on the dashboard, run a mapped barcode into a junction | forward / U-turn / left / right as mapped |
| SYS-04 | end of the line | safe stop | manual | line follow off the end of a line | stops within 600 mm (`BUDDY3_NO_LINE_STOP_MM`, raised from 300 mm on 24 Nov after track measurements), mode Idle |
| SYS-05 | obstacle width | width 10–20 cm reported | manual | 10 cm and 20 cm box on the line | dashboard width close to the real width; rover goes round and rejoins the line |
| SYS-06 | lifted rover stops | safety | manual | lift the rover while line following | motors stop, stay stopped when put down |
| SYS-07 | Wi-Fi lost while line following | safety (Tribunal 2: safe, predictable response to a failed part) | manual | turn the hotspot off while the rover line follows; turn it back on after 20 s | line following stops 2 s after the link drops; rover rejoins Wi-Fi and MQTT by itself and stays stopped until restarted |
| SYS-08 | full demo run | all demo requirements | manual | run the whole demo track: barcodes, junctions, hump, obstacle | every task done in one run without help |

*Method:* **manual** = a person observes the result on the rover or the
dashboard.

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Commit author | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|---|
