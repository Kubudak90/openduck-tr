# Servo tork/hiz ihtiyaci (mevcut 27 cm modelden)

## Ozet
I measured torque and speed demand on the current OpenDuck (27.3 cm HOME, 0.676 kg, 15 position servos: kp 1.1 N.m/rad, ±0.96 N.m, back-EMF damping 0.053 N.m·s/rad, which means a no-load speed of about 18 rad/s). Every 5 ms physics substep was logged; nothing in the repo was edited. The tasks were walking (walk-mjx-v2: the demo, the command-range extremes, and DR+push with 16 envs × 20 s), crouch/sit/stand (zero policy on 'main', the A3 schedule, 32 DR envs from HOME and 32 from the seated bank), the three keyframe get-ups (front/back/left on the repo getup model, 20 rest states each), and static holds (HOME, h 0.090/0.070/0.065, seat).

On top of the raw demand I ran three checks:
- **Minimum torque per joint group:** lower one group's force limit at a time (others at 0.96) until a task fails. With all groups at their minimum at once, every task still passes.
- **Minimum no-load speed:** sweep the servo speed down until a task fails.
- **Candidate servos at each height:** using Froude scaling (tau ~ m·g·L, time ~ sqrt(L)), a servo on a robot of height H behaves like a servo of stall tau_s/k and speed w0·sqrt(s) on today's robot. I checked each candidate two ways: (a) replaying the recorded low-torque ("lean") trajectories through its torque-speed line, and (b) running the real policy and scripts with its equivalent parameters.

**Key numbers at the current scale:**
- **Knee is the bottleneck.** Walking needs at least **0.50 N.m** at the knee (fails at 0.45), with knee speeds of 10–15 rad/s peak and about 11.7 rad/s at the 99th percentile. The knee also carries a high continuous load: RMS 0.28–0.31 N.m.
- **Other groups (minimum, max over walk / sit-stand / get-up):** hip_pitch 0.45 (walking alone 0.25), ankle 0.30, hip_yaw 0.15, hip_roll 0.10, neck 0.35, head 0.35. Neck and head need almost nothing for walking (0.02) and 0.10/0.06 for sit/stand; their 0.35 comes only from the get-up scripts.
- **Sit/stand** needs about 0.25 N.m (hip_pitch; 0.20 fails). Measured peaks are only 0.28 N.m.
- **Get-up** needs a uniform limit of 0.45 N.m (fails at 0.40). The back and left get-ups hit the 0.96 clip on knee, ankle, neck and head_yaw: per rollout, head_yaw spends 0.45 s saturated, neck_pitch 0.16 s, knee 0.13 s, ankle 0.11 s. The scripts also need fast servos: they fail as soon as damping rises above 0.053.
- **Static holds are small:** at most 0.13 N.m (knee at h 0.090), 0.12 N.m (hip_pitch / neck at h 0.065), and ≤ 0.07 N.m in the seat pose.

**What this means for the servos:**
- The sim's 0.96 N.m servo gives the current robot about **1.9× margin** over the knee minimum.
- A real XL330-M288 at 5 V (0.52 N.m, 103 rpm) runs everything in the sim but has **no margin** (1× only). It would also overheat on the knee: RMS is about 52% of its stall.
- The 0.96 N.m figure comes from a BAM fit with the XL330 driven from a 2S pack. That is above its rated 3.7–6 V.

**Sizing result:** with today's gait (2.5 Hz) and 2× margin, **no candidate class passes at 22, 27, 33, 40 or 47 cm**. Two things cause this:
- **Mass doesn't drop with size:** 15 servos plus 39 g of electronics plus the battery stay in the robot, so required torque falls more slowly than s^4 when you go smaller.
- **The 55–60 g servos are too slow and too big:** STS3215 manages 4.7 rad/s, MG996R and DS3218 7.5 rad/s, and they fit only at 32–36 cm and up. Above about 33 cm the s^3 body mass outgrows their torque.

**Combinations that work at 1× only:**
- **MG92B at 22 cm:** u 0.81; the sim passes walk, sit/stand and get-up.
- **XL330 at 5 V at 27 cm** (the current design).
- **STS3032 at 22 cm:** walk and sit/stand pass, get-up fails.

**Combinations just over 1×:**
- **DS3218 at 33–40 cm:** u 1.10–1.12; the sim walk passes at 33 cm.
- **ST3020 at 12 V at 33–40 cm:** sim walk passes, u 1.22–1.32.

**The micro 9 g class is far too weak for a 27 cm robot** (0.4–0.5× of the 1× need). Reaching 2× with cheap servos needs a slower, retrained gait and a lighter, less heavily actuated design (see recommendations).

## Eklem gruplari
Current robot (27.3 cm, 0.676 kg, sim servo ±0.96 N.m). Each cell: |torque| **peak / p95 / RMS** in N.m, then **w** = peak joint speed in rad/s, then **sat** = seconds per run per joint at |tau| ≥ 0.912 N.m (if any). Left and right legs are pooled. "head" = head_pitch + head_yaw + head_roll pooled; mouth is excluded.

| task | hip_yaw | hip_roll | hip_pitch | knee | ankle | neck | head |
|---|---|---|---|---|---|---|---|
| walk demo, nominal, 19 s | 0.35/0.09/0.03 w3.9 | 0.35/0.24/0.10 w3.0 | 0.61/0.35/0.17 w6.9 | 0.96/0.52/0.25 w9.6 sat0.01 | 0.50/0.29/0.14 w5.0 | 0.06/0.05/0.04 w0.6 | 0.05/0.01/0.01 w0.8 |
| walk max command (0.30 fwd / 1.0 turn / 0.15 side / -0.15 back), 17 s | 0.40/0.13/0.06 w4.7 | 0.47/0.26/0.12 w4.0 | 0.68/0.42/0.21 w8.2 | 0.96/0.69/0.31 w14.6 sat0.13 | 0.87/0.47/0.23 w8.5 | 0.10/0.04/0.03 w0.9 | 0.05/0.01/0.01 w0.6 |
| walk DR + push + random head, 16 × 20 s, 0 falls | 0.48/0.14/0.06 w5.3 | 0.63/0.28/0.13 w6.1 | 0.96/0.40/0.20 w12.1 | 0.96/0.61/0.30 w15.2 sat0.21 | 0.96/0.43/0.20 w9.4 sat0.01 | 0.16/0.08/0.04 w1.5 | 0.17/0.03/0.01 w2.3 |
| crouch hold h 0.070, DR, 16 envs | 0.02/0.01/0.01 w0.3 | 0.04/0.01/0.01 w0.5 | 0.13/0.10/0.07 w0.8 | 0.10/0.08/0.04 w0.4 | 0.08/0.06/0.02 w0.8 | 0.16/0.14/0.10 w0.8 | 0.10/0.04/0.02 w1.4 |
| crouch + sit + stand from HOME, A3, DR, 32 × 11 s | 0.06/0.03/0.01 w0.6 | 0.06/0.03/0.01 w0.8 | 0.28/0.18/0.07 w2.9 | 0.23/0.14/0.06 w2.0 | 0.18/0.11/0.05 w1.9 | 0.14/0.08/0.03 w1.1 | 0.17/0.04/0.02 w1.8 |
| stand from seat, A3, DR, 32 × 5.5 s | 0.22/0.02/0.01 w2.1 | 0.17/0.03/0.02 w1.9 | 0.28/0.21/0.08 w2.8 | 0.23/0.15/0.08 w1.8 | 0.18/0.14/0.06 w1.6 | 0.14/0.10/0.04 w0.8 | 0.18/0.05/0.02 w2.5 |
| get-up front (best_front_x_slow, 3.2 s) | 0.23/0.14/0.06 w2.3 | 0.33/0.16/0.09 w3.7 | 0.64/0.47/0.20 w9.2 | 0.66/0.40/0.20 w9.1 | 0.48/0.37/0.14 w7.4 | 0.65/0.43/0.18 w8.0 | 0.57/0.16/0.08 w6.4 |
| get-up back (combo_back, 4.9 s) | 0.37/0.20/0.08 w5.7 | 0.33/0.18/0.09 w5.5 | 0.70/0.47/0.20 w12.0 | 0.96/0.47/0.22 w16.8 sat0.06 | 0.96/0.47/0.22 w17.1 sat0.10 | 0.96/0.60/0.26 w15.5 sat0.16 | 0.96/0.37/0.18 w17.7 sat0.15 |
| get-up left (combo_left, 4.7 s) | 0.41/0.16/0.07 w6.2 | 0.46/0.19/0.09 w5.9 | 0.72/0.47/0.21 w12.4 | 0.96/0.52/0.24 w16.8 sat0.09 | 0.93/0.42/0.18 w15.8 sat0.01 | 0.65/0.37/0.16 w8.0 | 0.96/0.40/0.16 w16.8 sat0.05 |

**Get-up success:** 100% nominal for all three; with DR 85% (front), 85% (back), 75% (left).

**Get-up time at saturation per rollout (s):**
- back: head_yaw 0.45, neck_pitch 0.16, right knee 0.13, right ankle 0.11, left ankle 0.08
- left: head_yaw 0.15, both knees 0.09, right ankle 0.03

**Get-up with the force limit removed** (keyframe PD only): back would ask for knee 1.35, ankle 1.17, neck 1.10, head_yaw 2.09 N.m. These are PD step demands, not real needs: the limit sweep succeeds at ≥ 0.45 N.m and fails at 0.35. The phys-getup model gives the same numbers within ±0.05 N.m.

**Static holds** (kp 1.1, settled 3 s, |tau| N.m):

| pose | hip_yaw | hip_roll | hip_pitch | knee | ankle | neck | head |
|---|---|---|---|---|---|---|---|
| HOME (h 0.115) | 0.00 | 0.01 | 0.01 | 0.07 | 0.02 | 0.02 | 0.02 |
| crouch h 0.090 | 0.00 | 0.01 | 0.03 | 0.13 | 0.06 | 0.02 | 0.03 |
| crouch h 0.070 | 0.00 | 0.00 | 0.06 | 0.04 | 0.01 | 0.10 | 0.02 |
| deep crouch h 0.065, head HOME | 0.00 | 0.00 | 0.09 | 0.01 | 0.00 | 0.12 | 0.03 |
| deep crouch h 0.065, head SIT | 0.00 | 0.00 | 0.12 | 0.02 | 0.03 | 0.00 | 0.06 |
| seat | 0.00 | 0.00 | 0.01 | 0.00 | 0.01 | 0.00 | 0.02 |

With a stiff servo (kp 30) the same poses are within ±0.05 N.m. The trunk sags 3.6–6.9° at kp 1.1.

**Minimum force limit per group** (only that group lowered, others at 0.96; grid result, and the next grid step down failed):

| group | walk (DR, 32 envs) | sit/stand (DR) | get-up |
|---|---|---|---|
| hip_yaw | 0.02 | 0.02 | 0.15 |
| hip_roll | 0.06 | 0.02 | 0.10 |
| hip_pitch | 0.25 | 0.25 | 0.45 |
| knee | **0.50** | 0.20 | 0.45 |
| ankle | 0.20 | 0.15 | 0.30 |
| neck | 0.02 | 0.10 | 0.35 |
| head | 0.02 | 0.06 | 0.35 |
| all groups together | 0.50 | 0.25 | 0.45 |

All groups at their own minimum simultaneously: still passes (factor 1.0).

**Minimum no-load speed** (all joints at stall 0.96, damping b = 0.96/w0):
- Walking needs w0 ≥ 8 rad/s (fails at 7). The policy tolerates b up to about 0.12, i.e. 2.3× the trained value.
- Sit/stand passes down to 3 rad/s.
- Get-up needs w0 = 18 (b = 0.053): the keyframe timing is speed-critical.
- At stall 1.92, walking needs w0 ≥ 15 with kp 1.1, or ≥ 12 if kp scales with stall torque.

**Lean walk** (every group at its minimum limit): knee RMS stays at 0.28 N.m, so the continuous load does not shrink with the limit.

## Olcekleme tablosu
**Method**
- s = H / 273 mm.
- Mass model: 15 servos + printed parts 296 g × s^3 (same body density as now) + 39 g electronics + battery (60 g micro, 72 g 20–25 g class, 150 g 55–60 g classes). Head (neck subtree): 5 servos + 129 g × s^3 + 29 g.
- Torque scale factor: k_legs = (m/0.676) × s; k_head = (m_head/0.248) × s.
- Required stall (2× margin) = 2 × group minimum × k.
- "walk+sit" uses the walk / sit-stand minimums: yaw 0.02, roll 0.06, pitch 0.25, knee 0.50, ankle 0.20, neck 0.10, head 0.06.
- "+get-up" uses yaw 0.15, roll 0.10, pitch 0.45, knee 0.50, ankle 0.30, neck 0.35, head 0.35.
- For reference, pure geometric scaling (k = s^4) gives a 2× knee requirement of 0.42 / 0.96 / 2.14 / 4.61 / 8.78 N.m at 22 / 27 / 33 / 40 / 47 cm.

**Required stall torque, 2× margin (N.m)**

Groups in order: hip_yaw / hip_roll / hip_pitch / knee / ankle / neck / head.

| H | class (mass) | robot kg | k_legs | walk + sit/stand | + get-up |
|---|---|---|---|---|---|
| 22 cm | micro (12 g) | 0.43 | 0.52 | .02/.06/.26/.52/.21/.10/.06 | .16/.10/.47/.52/.31/.36/.36 |
| 22 cm | 20–25 g | 0.57 | 0.67 | .03/.08/.34/.67/.27/.13/.08 | .20/.13/.61/.67/.40/.45/.45 |
| 22 cm | STS3215 (55 g) | 1.17 | 1.39 | .06/.17/.70/1.39/.56/.24/.14 | .42/.28/1.25/1.39/.84/.85/.85 |
| 22 cm | MG996R (60 g) | 1.24 | 1.48 | .06/.18/.74/1.48/.59/.26/.15 | .44/.30/1.33/1.48/.89/.90/.90 |
| 27 cm | micro | 0.57 | 0.83 | .03/.10/.41/.83/.33/.17/.10 | .25/.17/.74/.83/.50/.60/.60 |
| 27 cm | 20–25 g | 0.70 | 1.02 | .04/.12/.51/1.02/.41/.20/.12 | .31/.20/.92/1.02/.61/.71/.71 |
| 27 cm | STS3215 | 1.30 | 1.90 | .08/.23/.95/1.90/.76/.34/.21 | .57/.38/1.71/1.90/1.14/1.20/1.20 |
| 27 cm | MG996R | 1.38 | 2.01 | .08/.24/1.01/2.01/.80/.36/.22 | .60/.40/1.81/2.01/1.21/1.27/1.27 |
| 33 cm | micro | 0.80 | 1.43 | .06/.17/.72/1.43/.57/.31/.19 | .43/.29/1.29/1.43/.86/1.08/1.08 |
| 33 cm | 20–25 g | 0.93 | 1.67 | .07/.20/.83/1.67/.67/.35/.21 | .50/.33/1.50/1.67/1.00/1.22/1.22 |
| 33 cm | STS3215 | 1.54 | 2.75 | .11/.33/1.37/2.75/1.10/.52/.31 | .82/.55/2.47/2.75/1.65/1.81/1.81 |
| 33 cm | MG996R | 1.61 | 2.88 | .12/.35/1.44/2.88/1.15/.54/.33 | .86/.58/2.59/2.88/1.73/1.90/1.90 |
| 40 cm | micro | 1.21 | 2.62 | .10/.31/1.31/2.62/1.05/.58/.35 | .79/.52/2.36/2.62/1.57/2.05/2.05 |
| 40 cm | 20–25 g | 1.34 | 2.91 | .12/.35/1.45/2.91/1.16/.63/.38 | .87/.58/2.62/2.91/1.75/2.21/2.21 |
| 40 cm | STS3215 | 1.95 | 4.22 | .17/.51/2.11/4.22/1.69/.84/.50 | 1.26/.84/3.79/4.22/2.53/2.94/2.94 |
| 40 cm | MG996R | 2.02 | 4.38 | .18/.53/2.19/4.38/1.75/.87/.52 | 1.31/.88/3.94/4.38/2.63/3.04/3.04 |
| 47 cm | micro | 1.79 | 4.56 | .18/.55/2.28/4.56/1.82/1.04/.62 | 1.37/.91/4.10/4.56/2.73/3.63/3.63 |
| 47 cm | 20–25 g | 1.92 | 4.89 | .20/.59/2.45/4.89/1.96/1.09/.66 | 1.47/.98/4.40/4.89/2.94/3.83/3.83 |
| 47 cm | STS3215 | 2.52 | 6.43 | .26/.77/3.21/6.43/2.57/1.34/.80 | 1.93/1.29/5.79/6.43/3.86/4.68/4.68 |
| 47 cm | MG996R | 2.60 | 6.62 | .26/.79/3.31/6.62/2.65/1.37/.82 | 1.99/1.32/5.96/6.62/3.97/4.80/4.80 |

**Servos checked** (datasheet stall @ voltage, no-load speed w0, mass):

| class | servo | stall | w0 | mass | notes |
|---|---|---|---|---|---|
| micro | SG90 | 0.18 N.m @4.8 V | 10.5 rad/s | 9 g | |
| micro | ES08MA II | 0.20 @6 V | 10.5 | 12 g | not 20–25 g as the task listed |
| micro | MG90S | 0.22 @6 V | 13.1 | 13.4 g | |
| micro | SCS0009 | 0.23 @6 V | 10.5 | 13.2 g | serial bus |
| micro | MG92B | 0.34 @6 V | 13.1 | 13.8 g | |
| 20–25 g | STS3032 | 0.44 @6 V | 11.6 | 20 g | serial bus |
| 20–25 g | XL330-M288 | 0.52 @5 V | 10.8 | 18 g | 0.60 @6 V; rated max 6 V |
| 55–60 g | STS3215 | 1.90 @7.4 V | 4.7 | 55 g | |
| 55–60 g | ST3020 | 2.45 @12 V | 6.3 | ~65 g? | listing says 97 g |
| 55–65 g | MG996R | 1.08 @6 V | 7.5 | 55 g | |
| 55–65 g | DS3218 | 2.0 @6.8 V | 7.5 | 60 g | some listings 29.8 kg.cm |

**Verdict per servo**
- **u** = 99.9th percentile of (replayed motor torque / equivalent stall) over the lean walk + sit/stand trajectories, worst joint (knee in every case). u ≤ 0.5 is a 2× pass; u ≤ 1 means it works with no margin.
- **sim** = equivalent servo run on the real policy and scripts, in the order walk / sit-stand / get-up (+ = pass).
- **Thermal**: knee RMS ≤ stall/3. This fails for every combination, including XL330 at 5 V at 27 cm (RMS = 0.52 of stall).
- **Fit**: a crude check that the CAD scales with the servo body length; the current CAD is built around the XL330's 34 mm.

| H | servo | u | sim | verdict |
|---|---|---|---|---|
| 22 cm | SG90 | 1.31 | -++ | FAIL |
| 22 cm | ES08MA II | 1.31 | -++ | FAIL |
| 22 cm | MG90S | 1.26 | -+- | FAIL |
| 22 cm | SCS0009 | 1.18 | -++ | FAIL |
| 22 cm | MG92B | **0.81** | **+++** | 1× only |
| 22 cm | STS3032 | 0.89 | ++- | 1× only; fit needs ≥ 23 cm |
| 22 cm | XL330 @5 V | 0.95 | ++- | 1× only; fit needs ≥ 27 cm |
| 22 cm | MG996R / DS3218 / STS3215 / ST3020 | 1.35 / 1.34 / 2.11 / 1.59 | — | FAIL; also too big (need ≥ 32–36 cm) |
| 27 cm | micro class | 1.32–2.22 | --- | FAIL |
| 27 cm | STS3032 | 1.18 | -++ | FAIL |
| 27 cm | XL330 @5 V | **0.94** | **+++** | 1× only (the current design) |
| 27 cm | MG996R / DS3218 | 1.24 / 1.22 | ++- | FAIL, does not fit |
| 27 cm | STS3215 / ST3020 | 1.91 / 1.44 | -+- | FAIL, does not fit |
| 33 cm | micro / 20–25 g | 1.6–4.1 | --- | FAIL |
| 33 cm | DS3218 | 1.12 | ++- | FAIL, closest |
| 33 cm | ST3020 | 1.32 | ++- | FAIL |
| 33 cm | MG996R | 1.28 | -+- | FAIL |
| 33 cm | STS3215 | 1.74 | -+- | FAIL; fit needs ≥ 36 cm |
| 40 cm | DS3218 | 1.10 | -++ | FAIL |
| 40 cm | ST3020 | 1.22 | ++- | FAIL |
| 40 cm | STS3215 | 1.61 | -+- | FAIL |
| 40 cm | MG996R | 2.03 | — | FAIL |
| 40 cm | micro / 20–25 g | 3–7.8 | — | FAIL |
| 47 cm | all | ≥ 1.41 | — | FAIL (DS3218 1.71, ST3020 1.41, STS3215 1.70) |

**References and sensitivity**
- The current sim servo on the current robot gives u 0.52, i.e. about 2×.
- If printed mass scales as s^2 (fixed wall thickness): DS3218 at 40 cm comes to 1.72 kg with u 1.03; ST3020 at 40 cm u 1.21. Nothing reaches u ≤ 0.5.

## Oneriler
1. **Size the knee first**, then hip_pitch and ankle. On today's robot the knee needs ≥ 0.5 N.m (1×) delivered at 8–12 rad/s, and carries about 0.28–0.30 N.m RMS. With a 2× margin that means roughly 1 N.m stall; the stall ≈ 3 × continuous rule gives about 0.9 N.m. Scale these by k (table) for other heights.
   - hip_yaw and hip_roll need at most 0.15 / 0.10 N.m (1×).
   - Neck and head need under 0.1 N.m for walking and sit/stand. They only matter for the scripted get-up (0.35).
   - So mixed classes make sense: cheap micro bus servos on hip_yaw, hip_roll and the head, and the strongest affordable servo on hip_pitch, knee and ankle.

2. **Choose servos with position feedback** (serial bus: SCS0009, STS3032, STS3215, ST3020, XL330). The RL policy observes joint position and velocity (q − q0 and qd are in the 49-dim observation). PWM servos (SG90, MG90S, MG92B, ES08MA, MG996R, DS3218) give no feedback, which adds a large sim-to-real gap on top of the torque problem.

3. **Cheap 9 g micro servos (SG90 / MG90S / ES08MA / SCS0009) cannot carry a 27 cm OpenDuck**: they reach only 0.4–0.5× of the 1× knee need, and every equivalent sim fails. The least risky cheap options:
   - **About 22 cm with micro/20 g bus servos.** STS3032 (0.44 N.m @6 V; u 0.89; sim walk and sit/stand pass) or MG92B (u 0.81; sim passes all three, but PWM). Both are 1× only and will run hot. Make the leg servos the strongest available. Cut mass where possible: drop the mouth and head_roll servos, use a lighter battery.
   - **Keep the current 27 cm design** with XL330 on legs at 5–6 V. It works at 1× in sim; 2× margin needs the 0.96 N.m that the repo assumes from 2S (7.4 V) drive, which is above the XL330's rated maximum of 6 V.
   - **A 33–40 cm build with DS3218 or ST3020.** Closest of the big servos (u 1.10–1.32). It needs a light body (≤ 1.7 kg), a 6.8 V / 12 V supply rated for about 15 × 2–3 A peak, and a slower gait. STS3215 is too slow for this gait at any height (4.7 rad/s).

4. **Retrain for the servo you actually buy.** The 2× margin cannot be met with the current 2.5 Hz walk-mjx-v2 gait. Put the chosen servo's scaled parameters in the MJCF `openduck` default class: forcerange = tau_stall/k, damping = forcerange/(w0·sqrt(s)), kp to match the servo's P gain. Then retrain walking with a lower GAIT_FREQ (about 1.5–2 Hz × 1/sqrt(s)), a stronger torque penalty and a lower command range. Re-run these scripts to check that knee p99.9 and RMS fit the servo. Two findings matter here:
   - **The minimums are upper bounds.** They come from a policy trained at 0.96 N.m, so a policy retrained for a weaker servo may need less.
   - **Damping limit.** The current policy only tolerates damping b ≤ about 0.12 (2.3× the trained value), so a slow servo needs a policy trained for it.

5. **Get-up scripts.** They need a uniform ≥ 0.45 N.m and fast servos (fail at w0 < 18 rad/s equivalent). For any slower or bigger servo, stretch the keyframe durations (≥ ×sqrt(s), more for slow servos) and re-optimise them with that servo's limits. The back and left scripts already sit at the 0.96 clip for 0.1–0.45 s (head_yaw, neck, knee, ankle).

6. **Before trusting the table, check these assumptions:**
   - Datasheet stall torques of cheap PWM servos are optimistic (clones often reach 60–80%).
   - The DS3218 rating varies between listings, and the ST3020 mass is uncertain.
   - The geometric fit check is crude.
   - Mass follows s^3 for printed parts, as specified; the s^2 sensitivity is in `scaling.json`.
   - The sim servo is a P controller with a torque clip and linear back-EMF. It does not model deadband or voltage sag.
   - The sit/stand numbers come from the zero policy. Its DR stand-up already fails 6–13% of the time at full torque, which is a balance problem, not a torque one.

7. **Repo side effect.** Nothing in the repo was edited. One gitignored cache file, `/Users/huseyinarslan/Desktop/adsız klasör 2/openduck/rl/__pycache__/config.cpython-312.pyc`, was regenerated by my analysis runs (`config.py` had changed at 10:03 and its cached copy was stale).

All results are under `/private/tmp/claude-501/-Users-huseyinarslan-Desktop-ads-z-klas-r-2/8217cd75-2c54-4fa4-b47f-57c38ee96c2b/scratchpad/s6/torque/`:
- **Per-task results:** `walk_torque.json`, `sitstand_torque.json`, `getup_torque.json`, `static_hold.json`
- **Sweeps:** `group_sweep.json`, `omega_sweep.json`
- **Equivalent-servo runs:** `servo_equiv.json`, `servo_equiv_kpscaled.json`
- **Lean trajectories:** `lean_traj.json`
- **Sizing:** `scaling.json` (every height × servo row, including the s^2 sensitivity) and `scaling.log` (all tables)
- **Raw logs:** `.npz` time series of torque, speed, position and ctrl at 200 Hz
