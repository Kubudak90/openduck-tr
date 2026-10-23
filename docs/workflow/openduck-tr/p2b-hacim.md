# P2b: TR-35 trunk and electronics, internal volume budget

Input: `spec.md` v3.1 (3.1 Mekanik / Güç / Elektronik, A1), `p2-interface.md` (frozen K_T, hip and neck boundaries,
harness contract, decisions in sec. 12), `research/power-electronics.md`, `research/parts-research.md`,
`research/listings.json`. Code: `cad/trunk.py` (`build_tr35_b2`, `volume_report`), `cad/electronics.py`,
`configs/tr35_b2.json` `params.trunk`. The tables in sections 7 and 8 are generated from the built model by
`trunk.volume_report(m)` (FreeCAD); everything else in this file explains them.

All coordinates: mm, trunk frame (x forward, y left, z up), zero pose, as in `p2-interface.md`.

## 1. Result

| Done-when item | Result |
|---|---|
| Same-link overlaps in the trunk (electronics, wiring and the legs/head in-tubes included) | **0** (`check.py` section 3, `test_trunk_fc`) |
| Internal volume budget report | this file (sections 7, 8) |
| Trunk mass and COM against the interface budget | **654.0 g** bottom-up (budget 733.6 g); trunk COM (-26.2, 0.4, 23.7) |
| Every trunk print part fits the 256 mm cube | yes; largest is `trunk_base` 138.0 x 76.0 x 65.9 |
| Scans with the legs/head stubs | **0 counted collisions**: single-joint 0, pairs 0, leg-leg 0, envelopes 0 / 0 / 0 (14 modelled contacts listed, unchanged from P2-0) |
| Golden (xl330_27) | 6/6 byte-identical (full run with `check.py`) |

Also: all P2b parts lie inside K_T (section 6 (a) = 0); both heat sources sit on PETG with >= 5 mm air to PLA (buck
5.50 mm, XL4015 5.60 mm); the robot bottom-up mass with the P2a/P2c stubs is 1768.9 g (x 1.05 = 1857 g, limit 1950 g);
HOME COM x = -10.2 mm (P2-0 stub -10.9; target [-8, 12]).

## 2. Layout

The frozen K_T leaves exactly one place for the 78 x 40 x 30 battery: lying in the core behind the hip-yaw stacks,
front face 0.75 mm behind the frozen hip harness entry tubes (x -13.25 at y +-8, z <= 12) and back face on the K_T back
plane (x -92). Above it is the only place for the 60 x 53 x 27 buck. Everything else is packed around these two.

| Zone (K_T region) | Contents |
|---|---|
| Core rear, low (x -92..-14, \|y\| <= 22, z -3..31) | battery in a PETG tunnel (floor, 1.2 mm walls, 1.5 mm roof = buck tray), TPU strap at x -47.5..-40.5 |
| Core/back rear, top (x -92..-25, z 31..62) | 20 A buck on the tunnel roof, PETG hood with top grille, back-box side walls with intake slots, buck window in the rear wall |
| Back-box side slots (x -90..-60, \|y\| 21.4..28.2, z -1.5..31) | left: XT30 (main disconnect, reached through the rear port) + ADS1115 #2; right: ADS1115 #1 + power-latch reserve |
| Side bays (x -60..-14, \|y\| 21.4..38/40) | PCA9685 #1 (left) / #2 (right) standing in floor pockets at x -39.7..-14.3; left rear: fuse on a pedestal, MP1584 above it; right rear: VBAT cap on the floor, overshoot clamp on a shelf; inner channels: hip signal branches, neck signal branch, battery main lead |
| Core front of the buck (x -30..-21, z 31..55) | MPU6500 flat on the tunnel roof (1.8 mm air to the buck), pot-divider board on two posts above it |
| Between the ES3054s (\|y\| < 5.5, z 39.7..57.7) | LiPo alarm (buzzer type), on the cage plate |
| Under the neck servo (x 21.5..43.5, y -3.2..20.8, z -1.5..10.5) | servo hub (star ground, LEG/AUX bars) with the INA226 on top |
| Front-left pocket (x 25.5..46, y 21.5..36.5) | main switch, bezel in the front face, recessed panel down to z -3 |
| Low sides beside the yaw housings (x -14..21, \|y\| 24.5..36.2) | hip power branches to the hub (floor level), LEG 2200 uF caps above them, neck signal branch (right, under the shoulder) |
| Belly (x -66..-40, z -24..-3) | XL4015 hanging from the floor on two PETG posts, AUX 2200 uF cap; open-ended PETG duct |
| Deck (x -42..-16, z 62..72) | ESP32-S3-DevKitC-1 on the hood/top plate, USB port in the left end wall |

## 3. Structure (PETG where loaded or near heat, PLA shells at constant wall)

* **`trunk_base`** (PETG, one print): 1.5 mm floor over the whole footprint (mating face z = -3); the turntable block
  (two housings r 12.5 fused at y = 0, 6800ZZ seats d 19 at z -3..2 and 19..24, 1 mm shoulder web r 8.5 between them,
  rotor zones of `params.iface` cut out); the cage support bracket (z 20..24) and the **ES3054 cage** (4 mm plate under
  the tabs at z 35.7..39.7 with the case openings and 4 x M3 nut traps on its underside, rear wall x -13.8..-10.5 behind
  the coupler/arm rotor box); the **neck DS cradle** (4 mm frame on the horn side of the tabs, y -13.9..-9.9, with the
  body window, 4 x M3 clearance holes and nut traps on the -y face, side walls x 21.3..23.2 / 43.5..45.3, web into the
  right housing, two wire holes for the hip and neck power branches); the battery tunnel and front stops (centre stop
  and right stop; the left side is the battery-lead exit); the fuse pedestal, clamp shelf, divider posts, XL4015
  posts; floor pockets for the two vertical PCAs and the switch bezel.
* **`buck_hood`** (PETG): top plate with 9 x 3 mm grille slots over the buck (1188 mm2), rear wall with the battery port,
  the XT30 port and the buck window (the buck's back end sits in the rear plane), back-box side walls with 2 x 4 intake
  slots, two front posts on the tunnel roof.
* **`belly_duct`** (PETG): bottom and side walls under the floor, both x ends open (XL4015 cooling duct).
* **`torso_shell_left/right`, `torso_top`, `deck_cover`** (PLA, constant 1.8 mm wall = `params.wall`): outer skin made of
  flat panels on the K_T boundary (low side walls, front faces and chin, shoulders, core side walls over the cages with
  the rotor zones and yaw housings cut out, mid side walls, step, top, back faces). The left shell carries the recessed
  switch panel. Vent slots in both low side walls (side-bay air). The back faces start 32 mm off the centre line and 3 mm
  above the floor so no PLA comes within 5 mm of the buck or the XL4015.
* **`battery_strap`** (TPU 95A): loop around the battery and the tunnel walls through floor slots; it closes under the
  floor at the front opening of the belly duct.
* **Fasteners**: the 8 servo M3 screws are modelled with nuts in their traps (ES3054 tabs 4 x M3 x 6, neck DS tabs
  4 x M3 x 8); the 20 g trunk fastener budget (`struct.fastener_mass_g`, shells/hood clips included) is carried by these
  items.
* **Neck idler seat**: the frozen K_T gives the F623ZZ seat box (x 26.2..40.5, y 21..26.65, z 34.4..48.7) no connection
  to the rest of the trunk except through the 0.4 mm `seat_gap`. The seat disc is therefore bonded to the DS rear face with
  a 0.4 mm VHB pad (`neck_seat_vhb`), aligned in place with the U-link pin fitted. A screwed tie to the cage plate needs
  ICR-1 (section 10).

## 4. Electronics at real envelopes

All 23 placed parts come from `cad/electronics.py` (size, mass, kind, heat, source, estimate flags). Decisions taken
there (P2b):

* PCA9685 x 2: servo headers, screw terminal and V+ capacitor removed, signal wires soldered (V+ is unused,
  power-electronics 2; mass ladder 3) -> 62.5 x 25.4 x 7 mm, standing on edge in floor pockets.
* ESP32-S3-DevKitC-1: headers removed, wires soldered -> 63 x 25.4 x 8 mm, flat on the deck, USB end at +y.
* MPU6500 (GY-6500): flat on the PETG tunnel roof at (-22.5, 0, 32.3), board edges parallel to the trunk axes (the MJCF
  `imu` site needs no rotation), 9.3 mm from the trunk COM.
* LiPo alarm: buzzer type 30 x 18 x 9 (estimate). The common 1-8S checker with display (40 x 25 x 11) fits no free
  volume of the frozen K_T (checked with a voxel search of the free space); P3 must source a compact unit or accept the
  alarm outside the shell.
* VBAT: one 1000 uF 35 V at the buck input ("1000 uF on the battery bus"); the second BOM unit is a spare. LEG bus: two
  2200 uF (one per leg branch, next to the hub feed); AUX bus: one 2200 uF next to the XL4015.
* Overshoot clamp (zener + MOSFET + dump resistor) and power-cut MOSFET latch: reserved volumes with estimated envelopes.
* Main switch: KCD1 envelope (bezel 21 x 15, body 19 x 13 x 18) in the front-left pocket. The KCD1 is rated DC 12 V
  only; per spec 3.1 a 16.8 V-rated switch is confirmed in the stock plan, otherwise the XT30 alone is the disconnect and
  the panel stays blank.

## 5. Power chain, wiring channels, star ground, access

* Chain (spec 3.1): battery leads leave the battery front face low on the left, run back outside the left tunnel wall
  (`battery_lead_wiring`, d 6) to the **XT30** in the left back-box slot -> **15 A fuse** (left side bay) -> **switch**
  (front left) -> VBAT bus with the 1000 uF cap at the buck input, INA226 (VBUS sense, on the hub), XL4015 (belly) and
  MP1584 -> 5 V to the DevKitC.
* **Star ground** is the servo hub's ground bar (under the neck servo): all buck/XL4015 returns and all 15 servo returns
  land there; one 22 AWG wire to the ESP32 GND next to the divider (power-electronics 1).
* **Wire channels** (Items of kind `harness` in the trunk link, all in `check.py` scans):
  * hip (left/right): `*_hip_trunk_wiring` d 7.5 continues the frozen in-tube from (-9.5, +-8, 12) up to z 16 and out to
    |y| 25 under the cage bracket; it splits into `*_hip_power_wiring` (d 4.4, down to the floor, forward through the low
    side bay to the hub; the right one passes the cradle frame through a hole and under the tab screws) and
    `*_hip_signal_wiring` (d 3.0, back along the inner side-bay channel to the PCA and divider).
  * neck: `neck_trunk_wiring` d 5 continues the frozen head in-tube from (33.33, -34.4, 18) down; `neck_power_wiring`
    (d 3) goes under the neck servo to the hub, `neck_signal_wiring` (d 2) runs back under the right shoulder to PCA #2 and
    ADS #1.
  * modelled wiring mass is scaled to the 25 g trunk wiring budget (`electronics.WIRING_G`).
* **Access**: battery slides out through the rear port after the strap is opened at the belly-duct front opening; the
  XT30 is reached through the port next to it; the switch bezel is on the chest; USB through the deck end wall; fuse, alarm
  and the ES3054 screws under `torso_top`; buck CV/CC pots through the grille slots; the side shells come off for the PCA,
  hub and divider wiring.

## 6. Thermal and airflow

* Heat sources (`electronics.HEAT_SOURCE_W` = 1 W, power-electronics 1 Thermal): 20 A buck (4-5 W at 6 A) and XL4015
  (< 2 W). ESP32 (0.5 W) and MP1584 (0.4 W) are warm parts, not heat sources.
* Buck: sits on the PETG tunnel roof, under the PETG hood; nearest PLA 5.50 mm (shell back face). Air enters through the
  rear window (the module's open back face, est. 50 % open of 1480 mm2) and 2 x 4 side slots (528 mm2), crosses the module
  and leaves through the top grille (1188 mm2; estimate for 5 W natural convection ~1000 mm2 each way). The hood's front
  is open to the core.
* XL4015: hangs from the PETG floor on PETG posts inside the open-ended PETG belly duct; nearest PLA 5.60 mm.
* The battery is separated from the buck by the 1.5 mm PETG roof and a 0.8 mm strap gap. The IMU is 1.8 mm in front of
  the buck's front end (no PLA involved; see open issues).

## 7. Volume budget and clearances

### Volume budget

| Group | Items | Volume in K_T (cm3) | Mass (g) |
|---|---|---|---|
| fixed | 10 | 58.6 | 129.0 |
| structure PETG | 3 | 62.8 | 62.8 |
| shell PLA | 4 | 35.7 | 42.9 |
| strap TPU | 1 | 0.8 | 0.7 |
| electronics | 23 | 279.3 | 370.0 |
| wiring | 10 | 7.0 | 25.0 |
| fasteners | 1 | 1.5 | 20.0 |
| foreign harness | 3 | 1.4 | 3.6 |
| **K_T** | | **625.6** | |
| **free (K_T - all parts)** | | **178.6** (29 %) | |

Free volume = K_T minus the in-K_T volume of every trunk item (the items do not overlap). It is fragmented: most
nearest-neighbour gaps below are 0-0.5 mm face contacts by design (boards resting on PETG supports, parts seated
against walls), and the voxel search found no remaining slot for any part larger than those placed (e.g. the
40 x 25 x 11 alarm).

### Electronics envelopes and clearances

| Part | Envelope (mm, placed x y z) | Vol (cm3) | Mass (g) | Heat (W) | Box x / y / z (mm) | Nearest neighbour (gap mm) | Est. |
|---|---|---|---|---|---|---|---|
| battery_4s_lipo | 78.0 x 40.0 x 30.0 | 93.6 | 200.0 | 0.0 | -92.0..-14.0 / -20.0..20.0 / -1.5..28.5 | trunk_base (0.0) | - |
| buck_20a | 60.0 x 53.0 x 27.0 | 85.9 | 70.0 | 5.0 | -91.8..-31.8 / -26.5..26.5 / 30.8..57.8 | trunk_base (0.0) | mass_g |
| xl4015 | 24.0 x 54.0 x 16.0 | 20.7 | 16.0 | 2.0 | -65.0..-41.0 / -32.5..21.5 / -21.7..-5.7 | trunk_base (0.2) | - |
| aux_bus_cap | 20.0 x 10.0 x 10.0 | 1.6 | 3.0 | 0.0 | -63.0..-43.0 / 22.0..32.0 / -22.2..-12.2 | belly_duct (0.0) | mass_g |
| esp32_s3_devkitc | 25.4 x 63.0 x 8.0 | 12.8 | 10.0 | 0.5 | -41.7..-16.3 / -31.5..31.5 / 62.0..70.0 | buck_hood (0.0) | size, mass_g |
| pca9685_1 | 25.4 x 7.0 x 62.5 | 11.1 | 6.5 | 0.0 | -39.7..-14.3 / 28.5..35.5 / -2.5..60.0 | trunk_base (0.2) | size, mass_g |
| pca9685_2 | 25.4 x 7.0 x 62.5 | 11.1 | 6.5 | 0.0 | -39.7..-14.3 / -35.5..-28.5 / -2.5..60.0 | neck_signal_wiring (0.1) | size, mass_g |
| imu_mpu6500 | 15.0 x 25.0 x 3.0 | 1.1 | 2.0 | 0.0 | -30.0..-15.0 / -12.5..12.5 / 30.8..33.8 | trunk_base (0.0) | mass_g |
| divider_board | 6.0 x 40.0 x 20.0 | 4.8 | 4.0 | 0.0 | -29.5..-23.5 / -20.0..20.0 / 35.0..55.0 | trunk_base (0.0) | size, mass_g |
| main_switch | 20.5 x 15.0 x 21.0 | 5.2 | 4.0 | 0.0 | 25.5..46.0 / 21.5..36.5 / -2.0..19.0 | torso_shell_left (0.0) | size |
| servo_hub | 22.0 x 24.0 x 8.0 | 4.2 | 12.0 | 0.0 | 21.5..43.5 / -3.2..20.8 / -1.5..6.5 | trunk_base (0.0) | size, mass_g |
| ina226 | 16.0 x 23.0 x 4.0 | 1.5 | 2.0 | 0.0 | 21.5..37.5 / -2.5..20.5 / 6.5..10.5 | servo_hub (0.0) | size, mass_g |
| leg_bus_cap_1 | 20.0 x 10.0 x 10.0 | 1.6 | 3.0 | 0.0 | -2.0..18.0 / 25.0..35.0 / 4.0..14.0 | left_hip_power_wiring (0.5) | mass_g |
| leg_bus_cap_2 | 20.0 x 10.0 x 10.0 | 1.6 | 3.0 | 0.0 | -2.0..18.0 / -35.0..-25.0 / 4.0..14.0 | right_hip_power_wiring (0.5) | mass_g |
| xt30_pair | 22.0 x 5.5 x 10.5 | 1.3 | 3.0 | 0.0 | -88.0..-66.0 / 22.0..27.5 / -1.5..9.0 | trunk_base (0.0) | size |
| fuse_15a | 10.0 x 14.0 x 25.0 | 3.5 | 5.0 | 0.0 | -58.0..-48.0 / 22.1..36.1 / 5.0..30.0 | trunk_base (0.0) | size |
| vbat_cap | 13.0 x 13.0 x 20.0 | 2.7 | 3.0 | 0.0 | -56.0..-43.0 / -35.5..-22.5 / -1.5..18.5 | trunk_base (0.0) | mass_g |
| overshoot_clamp | 12.0 x 10.0 x 20.0 | 2.4 | 3.0 | 0.0 | -55.5..-43.5 / -37.5..-27.5 / 26.0..46.0 | trunk_base (0.0) | size, mass_g |
| lipo_alarm | 30.0 x 9.0 x 18.0 | 4.9 | 5.0 | 0.0 | -8.2..21.8 / -4.5..4.5 / 39.7..57.7 | trunk_base (0.0) | size, mass_g |
| power_latch | 20.0 x 6.0 x 12.0 | 1.4 | 3.0 | 0.0 | -85.0..-65.0 / -27.8..-21.8 / 16.0..28.0 | ads1115_1 (0.0) | size, mass_g |
| ads1115_1 | 28.0 x 5.0 x 17.5 | 2.5 | 2.0 | 0.0 | -89.0..-61.0 / -27.3..-22.3 / -1.5..16.0 | trunk_base (0.0) | size, mass_g |
| ads1115_2 | 28.0 x 5.0 x 17.5 | 2.5 | 2.0 | 0.0 | -89.0..-61.0 / 22.3..27.3 / 9.0..26.5 | xt30_pair (0.0) | size, mass_g |
| mp1584 | 17.0 x 4.0 x 22.0 | 1.5 | 2.0 | 0.4 | -58.0..-41.0 / 32.0..36.0 / 30.0..52.0 | fuse_15a (0.0) | size, mass_g |

### Heat sources (spec 3.1: PETG at the source, >= 5 mm air to PLA)

| Heat source | Loss (W) | Min gap to PLA (mm) | Nearest PLA part | Touching parts (gap < 0.5 mm) |
|---|---|---|---|---|
| buck_20a | 5.0 | 5.50 | torso_shell_left | buck_hood (bracket), trunk_base (bracket) |
| xl4015 | 2.0 | 5.60 | torso_shell_right | aux_bus_cap (chip), trunk_base (bracket) |

### Trunk print parts

| Print part | Material | Volume (cm3) | Mass (g) | Extents (mm) | Fits 256 mm cube | Orientation / support |
|---|---|---|---|---|---|---|
| neck_pitch_idler_seat | PETG | 0.3 | 0.3 | 14.0 x 14.0 x 4.0 | yes | flat disc on the bed |
| trunk_base | PETG | 44.8 | 44.8 | 138.0 x 76.0 x 65.9 | yes | floor on the bed; battery-tunnel roof is a 40 mm bridge; supports under the ES3054 cage plate (z 35.7) and over the neck-frame window |
| buck_hood | PETG | 12.8 | 12.8 | 67.0 x 67.0 x 63.5 | yes | grille face on the bed (upside down), no supports |
| belly_duct | PETG | 5.2 | 5.2 | 72.0 x 26.0 x 21.0 | yes | bottom plate on the bed, no supports |
| torso_shell_left | PLA | 12.7 | 15.3 | 106.0 x 65.0 x 40.0 | yes | outer side face on the bed; shoulder/top panels bridge < 20 mm; switch panel vertical |
| torso_shell_right | PLA | 12.9 | 15.5 | 106.0 x 63.5 x 40.0 | yes | outer side face on the bed; shoulder/top panels bridge < 20 mm |
| torso_top | PLA | 5.7 | 6.9 | 69.2 x 67.0 x 1.8 | yes | flat plate on the bed |
| deck_cover | PLA | 4.4 | 5.2 | 80.0 x 26.0 x 10.0 | yes | top face on the bed (upside down), no supports |
| battery_strap | TPU 95A | 0.8 | 0.7 | 44.4 x 33.1 x 7.0 | yes | TPU, printed flat as a strip and wrapped |

## 8. Mass and COM

### Mass and COM

| Quantity | Value |
|---|---|
| Trunk link mass (bottom-up) | 654.0 g (budget 733.6 g) |
| Trunk COM (trunk frame) | (-26.2, 0.4, 23.7) mm |
| Robot mass (bottom-up, with P2a/P2c stubs) | 1768.9 g; x 1.05 = 1857.3 g (limit 1950 g) |
| HOME COM (trunk frame) | (-10.2, 1.0, -10.6) mm |
| Feet x span at HOME / centre | -26.4..44.3 / 9.0 mm; COM - centre -19.1 mm |

Mass against the interface budget (`p2-interface.md` sec. 8, trunk 733.6 g):

| Line | Budget (g) | P2b (g) | Note |
|---|---|---|---|
| Servos, bearings, idler (2 x ES3054, neck DS, 4 x 6800ZZ, F623ZZ, seat, VHB) | 128.6 | 129.0 | seat 0.3 g + VHB 0.1 g |
| Electronics | 385.0 | 370.0 | `electronics.total_mass_g()`; lighter PCAs (headers removed), smaller hub, 1 VBAT cap, compact alarm |
| Printed (PETG 62.8, PLA 42.9, TPU 0.7) | 175.0 | 106.4 | density_eff per kind (`params.print`) |
| Fasteners | 20.0 | 20.0 | modelled servo screws carry the budget |
| Wiring | 25.0 | 25.0 | modelled channels scaled to the budget |
| legs/head in-tubes in the trunk link (P2a/P2c items) | - | 3.6 | |
| **Total** | **733.6** | **654.0** | 79.6 g under budget |

HOME COM: x -10.2 mm is 19.1 mm behind the foot centre (target: COM x in [-8, 12], i.e. foot centre -7 +- 10). P2b
moved what it could: the battery front is on the hip-harness tubes, the buck is as far forward as the ES3054 cage
allows (x -31.8; the IMU and the divider take the 9.5 mm in front of it), the servo hub, caps, switch, INA226 and wiring
sit in front of the yaw axes, and the trunk came out 79 g lighter than budgeted (mostly from the rear). Closing the
remaining 2.2 mm needs ~3.9 kg mm of forward moment (the battery 20 mm forward), which the frozen K_T does not allow;
per decision 5 (`p2-interface.md` sec. 12) the residual goes to the HOME re-derivation in P5.

## 9. Checks run

* `cad/check.py` (full, tr35_b2 with the P2a/P2c stubs): single-joint 0 (107 poses), adjacent pairs 0 (300 poses, module scope), leg-leg 0 (32 poses), same link 0, envelopes 0 / 0 / 0 (138 poses); 14 pose-pairs listed as modelled contacts (as at P2-0: 10 knee+65/ankle-45 fold, 4 leg-leg feet); 34 harness items in all scans; 48 module-external pose-pairs listed for information; random 28 / 40 collision-free (information); 358 s with other FreeCAD jobs running
* Module-external contacts listed for information in section 4 (MJCF self-collision pairs, spec 3.2.2) are knee
  harness/horn/servo and foot parts against the trunk skin, PCAs, fuse, MP1584, clamp, buck and hood at combined
  hip_pitch/knee extremes, and the head shell against the hood/shells at combined neck/head_pitch extremes.
* `pytest tests/cad`: 83 passed (new: `test_trunk_data.py` 8, `test_trunk_fc.py` 9).
* `pytest tests/golden`: 6/6 (full, with `check.py`).
* Development aid (not in the repo): a 0.5 mm voxel model of K_T minus the fixed contents and the structure, used to
  search placements; it showed that no slot of the frozen K_T takes the 40 x 25 x 11 alarm or a back wall behind the
  battery.

## 10. Interface change requests

Both verified with the `check.py` section 6 logic on a config copy with the extra K_T box: containment / intrusion /
envelope-envelope 0 / 0 / 0 over the 138 single-joint (limits) and leg-leg poses.

| ICR | Extra K_T box [x0, x1, y0, y1, z0, z1] | Why | Min distance to other modules |
|---|---|---|---|
| ICR-1 neck seat tie | [19.9, 26.2, 21.0, 26.65, 34.4, 39.7] | lets a 4 mm PETG tie join the F623ZZ seat plate to the ES3054 cage plate (screwed), replacing the VHB-bonded seat; the frozen seat box touches the trunk only through the 0.4 mm seat gap | 0.10 mm (neck envelope leg B at y 26.75, same as the frozen seat box) |
| ICR-2 battery door | [-95.0, -92.0, -24.0, 24.0, -3.0, 31.0] | a 2 mm PETG door over the rear battery port: protects the LiPo end in back falls and retains it without the strap | 24.95 mm (right ankle idler pin at right hip_pitch -63) |

## 11. Open issues

* HOME COM 19.1 mm behind the foot centre (x -10.2 vs >= -8): P5 HOME re-derivation (decision 5).
* LiPo alarm: only a compact buzzer type fits (30 x 18 x 9 estimate); P3 to source one, or accept the 1-8S checker on a
  short lead outside the shell.
* Main switch: KCD1 is rated DC 12 V; keep the reserved front-left location only if a 16.8 V-rated switch is in stock
  (spec 3.1), else XT30 only.
* IMU sits 1.8 mm in front of the buck's front end (the only flat spot near the trunk COM): expect a few degC warm-up;
  calibrate the gyro bias at operating temperature (sensor model bias DR covers it).
* Neck idler seat is VHB-bonded until ICR-1; the battery's back end is exposed until ICR-2.
* `trunk_base` needs supports under the ES3054 plate and the neck-frame window; everything else prints without.
* Board fastening is modelled at envelope level as PETG supports, floor pockets, shelves and posts (no screw bosses or
  clips for the small boards, no shell/hood fasteners); P3 drawings should add clips or VHB notes.
* Estimates to confirm in Step 0 / P3: buck mass (70 g) and terminal side, alarm, INA226, MP1584, hub, divider, clamp
  and latch sizes, PCA/DevKitC heights after header removal, the buck module's open-face fraction for the air path.
