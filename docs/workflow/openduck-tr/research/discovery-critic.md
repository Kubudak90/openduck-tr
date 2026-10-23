# Completeness critic

# OpenDuck-TR spec readiness: completeness critique of the four reports

The four reports leave seven things unsettled before a concrete spec can be written:
- The servo speed. There are three published values: 0.12, 0.14 and 0.18 s/60°.
- The get-up strategy. The current one cannot work on this hardware.
- A combined mass budget, which comes to about 1.75–1.89 kg, not 1.65 kg.
- Firmware and command input. No report covers either.
- Acceptance criteria. None are defined.
- How the walk task stays bit-identical (rl/TASK.md:13) once the 35 cm constants change.
- Eight places where the reports give conflicting values (section D).

My calculations are in /private/tmp/claude-501/-Users-huseyinarslan-Desktop-ads-z-klas-r-2/8217cd75-2c54-4fa4-b47f-57c38ee96c2b/scratchpad/s8/critic/tools/calc.py. The fetched shop pages are in /private/tmp/claude-501/-Users-huseyinarslan-Desktop-ads-z-klas-r-2/8217cd75-2c54-4fa4-b47f-57c38ee96c2b/scratchpad/s8/critic/html/. The joint ranges below come from the 27 cm recordings in /private/tmp/claude-501/-Users-huseyinarslan-Desktop-ads-z-klas-r-2/8217cd75-2c54-4fa4-b47f-57c38ee96c2b/scratchpad/s6/torque/*.npz. The repo was not modified.

## A. Gaps I resolved

**R1. What "35 cm" means.**
- torque-needs.md uses s = H/273 mm, where 273 mm is the current robot's height at HOME. The RL report measured the head top at HOME as 0.2737 m.
- So H is the head-top height in the "stand" keyframe, and s = 1.28. The reports use 1.282 and 1.2835; the difference does not matter.
- Spec: 350 ± 5 mm head-top height at "stand", measured on the exported MJCF. It should be written to scale.json.

**R2. "1.8 Hz" is a 27 cm-equivalent number, not the 35 cm GAIT_FREQ.**
- In the u formula, ωe = ω0·√s, so κ = 1 is the Froude-similar gait: 2.5/√s = 2.21 Hz on the 35 cm robot.
- The "1.8 Hz" in parts-research and the servo-data report is κ = 0.72. On the real robot that is **1.59 Hz**.
- Setting GAIT_FREQ = 1.8 on the 35 cm robot means κ = 0.815, which gives u = 1.13 with the DSSERVO Blue speed. That fails.

Knee u by κ (my calculation):

| κ | GAIT_FREQ at 35 cm | DS3230 Blue (2.88 N·m, 5.72 rad/s) | Red | Direnc text | DS3225 |
|---|---|---|---|---|---|
| 0.8 | 1.77 Hz | 1.11 | 1.05 | 0.78 | 0.89 |
| 0.72 | 1.59 Hz | 1.00 | 0.95 | 0.72 | 0.82 |
| 0.6 | 1.32 Hz | 0.85 | 0.80 | 0.62 | 0.71 |
| 0.5 | 1.10 Hz | 0.74 | 0.69 | 0.54 | 0.62 |

Proposed Step-0 speed gate (u ≤ 0.85, stall 2.88 N·m, mass 1.80 kg), from the bench-measured ω0 at 6.5 V:

| Measured ω0 | 60° time | Use κ | GAIT_FREQ |
|---|---|---|---|
| ≥ 7.82 rad/s | ≤ 0.134 s | 0.8 | 1.77 Hz |
| ≥ 7.04 rad/s | ≤ 0.149 s | 0.72 | 1.59 Hz |
| ≥ 5.87 rad/s | ≤ 0.178 s | 0.6 | 1.32 Hz |
| ≥ 4.89 rad/s | — | 0.5 | 1.10 Hz |

**R3. Only a 180° DS3230MG is confirmed in Turkey, so the hip_pitch horn needs an offset.**
- Motorobit's DSSERVO listing says "180°": https://www.motorobit.com/ds3230mg-30kg-su-gecirmez-dijital-servo-motor (48 in stock, 1,939.20 TL incl. KDV).
- Direnc's page does not state the travel, the box contents or a stock count, and gives 4.8–7.2 V: https://www.direnc.net/ds3230mg-su-gecirmez-servo-motor
- I found no 270° unit in stock, so the spec should assume 180°.

Joint ranges the 27 cm policies actually used (degrees, converted to left-leg sign):

| Joint | Walk with DR | Sit/stand |
|---|---|---|
| hip_pitch | −52 … +16 | −31 … +90 |
| knee | −37 … +40 | −3 … +62 |
| ankle | −26 … +57 | −45 … +49 |
| hip_yaw | −21 … +4 | — |
| neck | 1 … 40 | — |
| head_yaw | −34 … +34 | — |
| head_pitch | 1 … 38 | — |

- hip_pitch needs −52 … +90, a span of 142°. Set the hip_pitch horn one 25T tooth off (+14.4°). The servo then covers −75.6 … +104.4, so set the limits to −75/+90.
- Knee and ankle fit with a neutral near 0. For neck_pitch, use −25° on the ES3054, as the servo-data report says.

**R4. The current get-up strategy cannot work on B2 hardware.**
- docs/workflow/sit-stand-getup/research/phys-getup.md:91–116: the roll stages swing head_yaw to ±2.97 rad (±170°). The front get-up uses hip_pitch ±1.57 and neck −1.57.
- An MG90S head_yaw can do only about ±80–90°. With the hip_pitch offset from R3, hip_pitch cannot reach −90°.
- So no scripted or learned get-up from that strategy family will transfer. A v1 get-up must be a new strategy, or it is out of scope (question U1).

**R5. 25T horns in Turkish stock.**
- Motorobit "25T Servo Motor Kolu": black, 193 in stock (https://www.motorobit.com/25t-servo-motor-kolu-siyah), and grey, 199 in stock (https://www.motorobit.com/25t-servo-motor-kolu-gri). 75.75 TL incl. KDV. Holes 20 and 24 mm from the centre, screws included. Red, yellow and green are out of stock.
- This is the same hole pattern as the DSSERVO aluminium arm. So the servo-data report's option 2 (capture the arm in the printed link) can be built from Turkish stock. The page does not state the material.
- I found no 25T disc horn at Motorobit or Direnc. The Robotistan and Robocombo search pages render in JavaScript, so those are inconclusive.
- It is also unknown whether the Direnc DS3230 comes with an aluminium arm. Budget 8 + 2 arms (about 760 TL).

**R6. Heat-set inserts.**
- Direnc "İnsert Somun M3" (https://www.direnc.net/insert-somun-m3) is out of stock. It is also a 6 × 5 mm stainless part, not a brass heat-set insert.
- Keep the M3 nut-trap design.

**R7. Bearings.**
- Motorobit F623ZZ, flanged, 3 × 11 × 4: in stock (1193), 75.75 TL (https://www.motorobit.com/f623zz-flansli-rulman-3mm). The flange locates the bearing axially in a printed pocket.
- Rulmanal plain 623ZZ: 40.69 TL, 345 in stock.
- Quantity: 10 (8 DS joints + 2 hip_yaw) + 2 spares. parts-research listed 8 × 693ZZ.

**R8. ES3054 dimensions.** The CAD report listed these as open questions (27 vs 31.1 mm height, hole pitch). The EMAX drawing cited by the servo-data report answers them: case 26.7 mm, spline top 31.1 mm, 2 × Ø4.1 holes at 35.0 mm pitch, tabs 17.8–19.7 mm above the bottom.

**R9. DS3230 shaft offset and cable exit.** The CAD report's question is answered by the DSSERVO STEP: the shaft is 10.0 mm from the shaft-side end and centred in width, and the cable gland is centred in width. So the right leg can keep using mirror_y'd servo shapes.

**R10. PCA9685 prescale with the 23–27 MHz oscillator spread.** prescale = round(osc/(4096·f)) − 1 (NXP datasheet).

| Prescale | Frequency range | Verdict |
|---|---|---|
| 18 | 295–347 Hz | Can exceed the 330 Hz DS limit |
| 19 | 281–330 Hz | Acceptable |
| 20 | 267–314 Hz | Most margin |

Use 19 only after the ch15 capture shows the oscillator is at or below 27 MHz; otherwise use 20. One tick is 0.80 or 0.84 µs.

**R11. Head mass is lower without head electronics.**
- The torque-needs head model (0.370 kg) includes 29 g of head electronics: board, hat, camera, speaker, ToF (cad/openduck.py:75–79). B2-ESP32 has none of these.
- Servos (ES3054 + 3 × MG90S + SG90) 69 g, horns 3 g, printed 129 g × s³ = 272 g. Total 0.344 kg, so k_head = 1.78 instead of 1.91.
- If the cosmetic head shells keep a fixed wall thickness (printed mass grows with s² instead of s³: 212 g), the head is 0.284 kg and k_head = 1.47, 23% less load on the neck and head servos.

**R12. Bottom-up mass and the knee thermal check.**

| Item | Mass |
|---|---|
| Servos (8 × 65 + 3 × 20 + 3 × 13.4 + 9) | 629 g |
| Printed, walls grow with scale (s³) / fixed walls (s²) | 630 g / 491 g |
| Electronics and battery (power report) | 420 g |
| Idlers (my estimate: about 7 g × 10 joints) | ~70 g |
| Metal horns (my estimate) | ~40 g |
| M3 fasteners (my estimate) | ~100 g |
| **Total** | **1.75–1.89 kg** |

- No report and no MASSES entry counts fasteners or horns.
- At 1.80 kg, k_legs = 3.41. Knee walk-DR RMS is then 0.30 × 3.41 = 1.02 N·m.
- Against the "RMS ≤ stall/3" rule:

| Servo | stall/3 | Result at 1.80 kg |
|---|---|---|
| DS3230 Blue | 0.96 N·m | Fails |
| DS3230 Red | 1.10 N·m | Passes |
| DS3230, Direnc text | 1.02 N·m | Borderline |
| DS3225 | 0.90 N·m | Fails, even at 1.65 kg (0.94 N·m) |

- u barely changes with mass because the knee is speed-limited. So mass is a thermal issue, not a speed issue.

**R13. hip_yaw (ES3054) under the old policy.**
- Walk-DR hip_yaw torque scaled to 35 cm (× 3.13): peak 1.50, p95 0.44, RMS 0.19 N·m.
- The ES3054 stall is 0.343 N·m, so stall/3 is 0.114. p95 is above stall, and RMS is 1.6 times the thermal limit.
- The group sweep shows walking needs only 0.02 N·m at hip_yaw. So the stall-normalised torque penalty in retraining is required, not optional.

**R14. Combined mandatory BOM for B2-ESP32.**
- 15,308.15 TL (B1) − 1,179.72 TL (B1 power lines) + 4,523.81 TL (power report) = **18,652.24 TL**, or 16,855.15 TL without the charger.
- Not yet included: bearings (407–758 TL), spare 25T arms (~760 TL), spare servos (1,943.62 TL).

**R15. Where the policy weights live.**
- ESP-IDF places `const` data in flash by default, read through the MMU cache (https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-guides/memory-types.html).
- The RL report's "214 KB float32 in internal SRAM" therefore needs an explicit copy at boot (heap_caps_malloc with MALLOC_CAP_INTERNAL) or DRAM_ATTR. Otherwise reads behave like the PSRAM case.
- Check the largest free internal block with Wi-Fi or BLE running. int8 via ESP-NN (54 KB) is the fallback.

**R16. Gamepad constraint.** The ESP32-S3 supports only BLE gamepads through Bluepad32. That rules out DualShock, DualSense, Switch Pro and Joy-Con; Xbox controllers work only with firmware 5 or later (https://bluepad32.readthedocs.io/en/latest/FAQ/).

**R17. The walk task's bit-identical contract.**
- rl/TASK.md:13 requires task="walk" to stay bit-identical. The RL report's D1 edits the constants in place, which would break that and the 27 cm baselines.
- Add a robot profile (for example `OPENDUCK_ROBOT = xl330_27 | tr35_b2`) that selects the constant set, matching the CAD configs (configs/xl330_27.json, tr35_b2.json).

**R18. PWM direction.**
- DSSERVO says "CCW for increasing pulse" and EMAX says CCW for 1500 → 1900 µs. The CAD report's sign is +1 on all 15 joints, with positive meaning CCW as seen facing the horn.
- So pwm_dir = +1 on every DS and ES3054 joint, provided the vendor's CCW is also viewed from the output side. Confirm on the bench. MG90S and SG90 are unknown.

**R19. A third published DS3230MG speed.** Motorobit lists 0.16/0.14 s/60° at 5/6.8 V, 26/30 kg·cm, 4.8–6.8 V, 67 g, 333 Hz. The page also says "Modeli: DS3235MG", so the text looks copy-pasted. That makes 0.12, 0.14 and 0.18 s/60° across Direnc, Motorobit and DSSERVO.

## B. Missing from every report (the spec must add these; they are not user decisions)

- **G1. Firmware architecture.** Needed:
  - task layout (200 Hz sensing, 50 Hz policy, I2C writes);
  - a policy supervisor (main/getup handoff, fall detection);
  - safe start: read the pots, then command the measured pose, then enable OE (only B2 can do this);
  - soft E-stop;
  - storage format for the per-joint calibration data;
  - telemetry and logging for sysID;
  - a timing budget with a jitter limit.
- **G2. Attitude estimator.** The policy observes projected gravity; the sim uses the true quaternion plus noise. The firmware needs a filter (Mahony or a complementary filter), and the sim's sensor model must run the same filter.
- **G3. hip_yaw structure.** The whole leg hangs off a 17 g servo with a single output bearing. Only the servo-data report mentions an idler plus thrust ring, and the CAD plan does not cover it. It needs its own bearing arrangement and a load check.
- **G4. Re-run the fit study.** The CAD fit used the case plus a 6 mm stub (about 52 mm). The servo-data U-link stack is 60.4 mm (−9.9 … +50.5), so the thighs sit about 8 mm further out than the fit study shows.
- **G5. Cables across joints.** Each DS servo carries 4–5 wires (3 + wiper + optional pot-low). The spec needs paths across hip_yaw, hip_pitch and knee, bend radius, strain relief and slack for the full travel.
- **G6. Thermal protection.** No servo has a temperature sensor. Either add duty or rest limits in firmware, or put an NTC on the knee servos. Note that ADC1 is fully used.
- **G7. Fall survival.** At 1.8 kg, with get-up probably deferred, the spec needs a target for how many falls without breakage. MG90S inner gears are plastic.
- **G8. Step-0 pass/fail numbers.** Use the R2 speed gate. Add pass/fail limits for the pot (wiper span at least 1 V, monotonic over the travel), stall current, 3.3 V pulse acceptance, and ES3054 at 300 Hz.
- **G9. Acceptance criteria. None exist.** A draft list (the targets need the user, see U8):
  - mass at most 1.85 kg;
  - H = 350 ± 5 mm;
  - static position error at most 1° after calibration, at most 2° at 1 A;
  - inference at most 2 ms, loop jitter under 1 ms;
  - stand 60 s;
  - 10 sit/stand cycles;
  - walk at least vx_target for 2 min without falling;
  - knee case under 60 °C after 10 min of walking;
  - runtime at least 20 min;
  - N falls without breakage.

## C. Open questions only the user can answer

- **U1.** Get-up in v1? The options are a new leg-only or front-only strategy, a DS3230 on neck_pitch, or manual recovery (see R4).
- **U2.** How is the robot commanded: BLE gamepad (which model, see R16), Wi-Fi from a phone or PC, or USB? Is Wi-Fi on while the policy runs? With Wi-Fi off, the optional ADC2 ground-sense becomes possible.
- **U3.** Which printer: bed size and TPU capability? The CAD report assumed 220 × 220 × 250; docs.py:325 checks 180.
- **U4.** Battery: is a spot-weld service available, or do you accept the Profuse 4S 1500 mAh LiPo fallback (22 Wh)?
- **U5.** Budget ceiling. The mandatory total is now 18.65k TL plus bearings, arms and spares.
- **U6.** Stance width: accept the thighs about 14–22 mm further out per side, or re-lay-out the hip and trunk?
- **U7.** Supplier for the 8 + 2 DS3230:
  - Direnc: 1,046 TL, "YUNIQUE" brand, variant and travel unknown, 15 in stock.
  - Motorobit: 1,939 TL, DSSERVO, 180° stated, 48 in stock.
  
  Should you buy one of each for Step 0?
- **U8.** Acceptance targets: walking speed (expect about 0.15–0.25 m/s), runtime, number of falls.
- **U9.** If hip_yaw or the knee fails thermally in sim or on the bench, are you willing to move hip_yaw to DS-class, or switch to the Red DS3230? This affects stock, mass and width.
- **U10.** You own the lead-owned RL files (rl/TASK.md:16). Do you approve the edits and the profile approach in R17?
- **U11.** Should the head stay electronics-free (no LED eyes or speaker)? And may the cosmetic shells keep a fixed wall thickness to save neck torque (R11)?

## D. Contradictions between reports, and recommended resolution

1. **DS3230 speed and damping.**
   - The values used: 8.3 rad/s / 0.37 (parts-research, CAD, power) against 5.72 rad/s / 0.50 (servo-data), with the RL report using both.
   - Resolution: until Step 0, the sim uses DSSERVO Blue: 2.88 N·m, 5.72 rad/s, damping 0.50. Training starts at κ = 0.6 (1.32 Hz), and R2's gate sets the final κ.
2. **GAIT_FREQ.** "1.8 Hz" (parts-research, servo-data), "1.6–1.8 Hz" (CAD) and "1.6 Hz" (RL). Resolution: write the spec as GAIT_FREQ = 2.21·κ Hz on the 35 cm robot. Never use 1.8 Hz at 35 cm (see R2).
3. **Leg power rails.** XL4015 per two servos (parts-research, servo-data, CAD MASSES, RL risk list) against one 300 W / 20 A buck (power). Resolution: use the power design, with 2 × XL4016 as the fallback, and update CAD MASSES and the RL risk list.
4. **Battery.** Spring holders + 2 × 2S BMS (parts-research, CAD) against a spot-welded 4S1P + 4S BMS (power). Resolution: use the power design. parts-research itself flagged the series 2S BMS pair as unapproved.
5. **Idler bearing.** 693ZZ (CAD, parts-research) against 623ZZ (servo-data). Resolution: 623ZZ or F623ZZ. They are cheaper and in deeper stock (R7), and the quantity is 10 + 2.
6. **DS3230 kp.** 3.5 N·m/rad (CAD, from the XL330 0.87 rad rule) against 58 (RL, 3° P band). Resolution: use the RL value with DR over a 1–8° band. The CAD rule recreates the soft-servo sag that the RL report removes. Bench sysID sets the final value.
7. **ADC attenuation.** 12 dB with 100k/47k (power) against ATTEN2 (RL). Resolution: keep 12 dB with 100k/47k. RL's ±10 mV figure is eFuse absolute error, and that disappears with per-joint raw-code calibration; 12 dB also tolerates an unknown pot supply. Change only if the Step-0 noise test favours ATTEN2.
8. **Position and velocity filtering, and delays.**
   - RL: q at 200 Hz with 1 substep of latency, qd from a first-order IIR (α = 0.47).
   - Power: a 500 Hz stream, a 25 Hz Butterworth on q, and 8–15 ms of delay. Leg action delay: RL {0, 1} substeps against power 3–12 ms.
   - Resolution: rl/sensors.py is the single definition and the firmware copies it exactly. Average the ADC over each 5 ms window and use the RL IIR. Include the RC filter (τ = 3.2 ms). Randomise leg action delay over U{0..2} substeps until the bench measures command-to-motion latency.
9. **PCA9685 prescale.** 18 (servo-data) against 19 (power) against 330 Hz (RL). Resolution: 19 or 20, chosen after calibration (R10).
10. **Leg proportions.** The RL report says the DS bodies change the thigh/shin ratio; the CAD report scales the joint points uniformly. Resolution: keep uniform scaling (thigh 54 mm, shin 63 mm). The CAD fit study found only ear clashes on the same link, so the crouch table angles can be reused as a starting point.
11. **hip_yaw margin.** RL says 5.5× margin; power says a 55% RMS thermal risk. Both are true. The old policy used far more hip_yaw torque than it needs (R13). Resolution: add a stall-normalised torque penalty and a sim acceptance check that hip_yaw RMS ≤ 0.11 N·m.
12. **Robot mass.** 1.65 (brief), 1.69 (parts-research), 1.70 (power) against 1.75–1.89 kg bottom-up (R12). Resolution: design to 1.85 kg, do torque and thermal checks with k = 3.5, and set acceptance at a measured mass of 1.85 kg or less.
13. **Joint stack width.** About 52 mm (CAD fit) against 60.4 mm (servo-data U-link). Resolution: the servo-data stack, and re-run the fit (G4).
14. **Walk constants.** RL edits rl/config.py in place; rl/TASK.md:13 requires walk to stay bit-identical, and the CAD report requires a byte-identical XL330 snapshot. Resolution: the profile mechanism in R17.
15. **Channel and rail map.** CAD hw_map ("two rails per leg", PCA 0x40) against the power pin and channel map. Resolution: use the power map. PCA#1 is the leg board at 0x40, ch0–7 on DS. PCA#2 at 0x41 runs at 50 Hz with hip_yaw on ch0–1. Wipers are on GPIO1–10, and the IMU on SPI GPIO39–42.