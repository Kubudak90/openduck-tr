# Servo mechanical and electrical data (DS3230MG + DS3225MG fallback, EMAX ES3054, TowerPro MG90S, SG90), 623/683/693ZZ bearings, rear-idler bracket pattern

## Summary
I took the CAD envelopes from manufacturer sources: DSSERVO's installation drawings and STEP model (I measured the STEP in FreeCAD), EMAX's dimensioned ES3054 sheet, TowerPro's A–F tables and datasheet copies. DSSERVO uses one 40x20x40.4 mm case for DS3218, DS3225 and DS3230. Its drawings agree: 4 x Ø4.5 holes on a 49.5 x 10 mm pattern, tab span 54.5 mm, tab underside 27.7 mm above the bottom. The STEP adds what the drawings leave out: output axis 10.0 mm from the shaft-side end, 25T spline Ø6.0 with its top 45.0 mm up, tab 3.2 mm thick, cable gland on the shaft-side end 8.55 mm above the bottom, flat bottom with no rear boss. So any rear idler has to come from the bracket.

KEY FINDING: DSSERVO's own DS3230MG table gives a no-load speed of 0.18 s/60° (blue) or 0.17 s/60° (red) at 6.8 V, i.e. 5.8–6.2 rad/s. The earlier analysis used 0.12 s/60° (8.7 rad/s) from the Direnc listing. Put through parts-research's own u formula, the official speed gives u ≈ 1.31–1.38 at the 2.5 Hz gait, which fails. At 1.8 Hz it gives about 0.95–1.00, so no margin. DSSERVO's DS3225MG table (0.14 s/60°, 2.75 N·m at 6.8 V) actually comes out better: u ≈ 1.08 at 2.5 Hz and 0.82 at 1.8 Hz. The Direnc product photo also shows an HV-labelled DS3230 ("30 kg @7.4 V, DC 6~8.4 V"), and the listing does not say whether it is the 180° or 270° version.

Before ordering 8 units: bench-measure no-load speed, stall current, the label and the travel. No DS32xx teardown turned up anywhere, so the potentiometer is still unconfirmed.

Recommended idler for leg joints: a printed cradle plus a U-link, with a 623ZZ (3x10x4; Rulmanal 40.69 TL, 345 in stock) on an M3 axle, coaxial with the output, under the servo bottom.

## Facts
## 1. DSSERVO DS3230MG (DS3225MG / DS3218MG share the case)
Sources: DSSERVO store (aifitlab.com) spec tables, fetched as Shopify JSON: https://aifitlab.com/products/dsservo-ds3230-mg-servo-motor , https://aifitlab.com/products/dsservo-ds3225-mg-servo-motor , https://aifitlab.com/products/dsservo-ds3218-mg-servo-motor , https://aifitlab.com/products/dsservo-ds3230-pro-servo-motor , https://aifitlab.com/products/dsservo-rds3230-servo-motor
Drawings: https://cdn.shopify.com/s/files/1/0673/6848/5000/files/DS3230MG_Blue_-Drawing.pdf , .../DS3230MG_Red_-Drawing.pdf , .../DS3225MG-Drawing.pdf , .../DS3218MG-Drawing.pdf
STEP model: https://cdn.shopify.com/s/files/1/0673/6848/5000/files/DS3218MG.stp. It is byte-identical (same md5) to DSSERVO-DS3230.stp, which the DS3230 PRO page links. Local copy: /private/tmp/claude-501/-Users-huseyinarslan-Desktop-ads-z-klas-r-2/8217cd75-2c54-4fa4-b47f-57c38ee96c2b/scratchpad/s8/servo-data/dl/step/DS3218MG.stp
TR listing: https://www.direnc.net/ds3230mg-su-gecirmez-servo-motor (brand field "YUNIQUE"). Photo: https://www.direnc.net/ds3230mg-su-gecirmez-servo-motor-servo-motor-yunique-35942-12-B.jpg

STEP frame: x = length (output axis at x=+10.0, body x ±19.97), y = height (bottom plane y=−20.0, small features down to −20.28), z = width (±10.02).

| Item | Value | Source |
|---|---|---|
| Body L x W x H | 40 x 20 x 40.4 (Blue drawing and spec table: 40.5) | drawings; STEP: bottom y=−20.0 to case top y=20.4 |
| Output boss | Ø11.8, top 41.5 above bottom | STEP (measured) |
| Shaft position | 10.0 from the shaft-side end face, 30.0 from the far end; centred in width; 10.0 off the centre of the hole pattern | STEP (x=+10.00, z=0); drawing pixel check 9.6–10.1 |
| Spline | 25T; modelled as Ø6.0 cylinder from y 17.8 to 25.0 → top 45.0 above bottom, 3.5 exposed above boss; M3 tapped centre hole (Ø2.5 model, ~8.6 deep) | STEP. 25T confirmed by "25T servo arm" in listings (e.g. https://probots.co.in/ds3230-30kg-180-degrees-high-torque-servo-metal-gear-ip65.html) |
| Tabs | span 54.5 (STEP 54.4); full 20 mm width; thickness 3.2 (y 7.60–10.84); underside 27.7 above bottom (STEP 27.6–27.75), top 30.85 | drawing + STEP |
| Mounting holes | 4 x Ø4.5, pitch 49.5 (length) x 10.0 (width); hole centres 2.45 from tab end; slots open to the tab end | drawing (49.5/10); STEP (Ø4.5 at x ±24.75, z ±5.0) |
| Bottom | flat; 4 case screws (Ø2.2, ~M2) at (±17.5, ±7.5) from body centre with recessed Ø4.5 counterbores; motor Ø17.4 under the far half, axis 20 mm from the output axis; no rear shaft or boss | STEP |
| Cable exit | shaft-side end face; gland centre 8.55 above bottom, centred in width; gland 6.8 W x 4.5 H (h 6.3–10.8), protrudes 5.4 mm; 3 wires Ø1.5 at 1.5 mm pitch; lead 300 ± 5 mm | STEP; spec table |
| Supplied horns and hardware | 25T aluminium clamp single arm: hub Ø15 (h 42.2–47.9), holes Ø2.6 at r = 20 and 24 mm (drawing "24"), M2 clamp screw, M3 centre screw. Plastic: round disc, 4-point cross, 2 double arms (hole circles not published). 4 rubber grommets, 4 brass eyelets, 4 self-tapping screws | STEP solid 5; product photos DSSERVO-DS3230MGT-main-1/3 |
| Weight | 60 g (DSSERVO); 65 g (Direnc); 58 ± 2 g (Probots) | as cited |
| Bearings | "Double bearing"; IP66; aluminium middle case | spec table |

Electrical (1 kg·cm = 0.0980665 N·m; ω0 = (π/3)/t60):

| Version (source) | 5 V | 6.8 V | 6.5 V (my linear interp.) |
|---|---|---|---|
| DS3230MG Blue, steel gears, ratio 221 (DSSERVO) | 26 kg·cm = 2.55 N·m; 0.20 s/60° = 5.24 rad/s; stall 3.5 A | 30 = 2.94 N·m; 0.18 = 5.82 rad/s; 3.8 A | 2.88 N·m, 5.72 rad/s |
| DS3230MG Red, brass gears, ratio 397 (DSSERVO) | 29.5 = 2.89; 0.20 = 5.24; 2.1 A | 34.5 = 3.38; 0.17 = 6.16; 2.9 A | 3.30 N·m, 6.01 rad/s |
| DS3230MG (Direnc text; used by prior research) | 27 = 2.65; 0.16 = 6.54 | 32 = 3.14; 0.12 = 8.73; range 4.8–7.2 V; 50–330 Hz | 3.06, 8.36 |
| DS3225MG (DSSERVO) | 24.5 = 2.40; 0.16 = 6.54; 2.1 A | 28 = 2.75; 0.14 = 7.48; 2.9 A; ratio 275 | 2.69, 7.32 |
| DS3225 (Robocombo/Direnc text) | 21 = 2.06; 0.15 = 6.98 | 24.5 = 2.40; 0.13 = 8.06 | 2.35, 7.88 |
| DS3218MG (DSSERVO) | 18 = 1.77; 0.16; 1.8 A | 21.5 = 2.11; 0.14; 2.6 A | – |

All DS32xx (DSSERVO): 4.8–6.8 V; idle 4 mA @5 V and 5 mA @6.8 V; PWM 500–2500 µs, neutral 1500 µs; 180° or 270° SKUs over 500–2500 µs; dead band 3 µs; 50–330 Hz; counter-clockwise for increasing pulse; 3-pole motor; operating −25 to 70 °C.

Probots gives yet another set: 0.2/0.17 s/60°, "34.5 kg·cm @6.8 V", 3 µs, 1520 µs/333 Hz.

Prior-research u formula (parts-research §0, s = 1.282, m = 1.69 kg, k = 3.205): Blue u = 1.38 (2.5 Hz), 1.00 (1.8 Hz); Red 1.31 / 0.95; Direnc text 0.95 / 0.72; DS3225 DSSERVO 1.08 / 0.82. My computation: /private/tmp/claude-501/-Users-huseyinarslan-Desktop-ads-z-klas-r-2/8217cd75-2c54-4fa4-b47f-57c38ee96c2b/scratchpad/s8/servo-data/tools/calc.py

## 2. EMAX ES3054 (official dimensioned sheet)
Source: EMAX spec image https://cdn.shopify.com/s/files/1/0371/2814/2907/files/ES3054.webp , from https://shop.emax-usa.com/products/emx-sv-0280-es3154-bearing-metai-digital-servo . Same numbers on https://emaxmodel.com/products/emax-es3054-17g-3-5kg-0-13sec-23t-metal-gear-digital-servo-for-rc-airplane-es3154-upgrade and on the Robocombo TR listing (https://robocombo.com/emax-es3054-3-5kg-metal-disli-dijital-mini-servo-4300: 23T, 145°±10° (700–2300 µs), 17 g, 28.45x13x31.10).

| Item | Value |
|---|---|
| Body | 28.45 L x 13.00 W x 26.70 H (case); spline top 31.10 above bottom |
| Spline / horn screw | 23T, Ø5.90 +0/−0.05; M3 |
| Shaft position | 9.38 from the near hole centre → 6.1 from the shaft-side end face (derived; drawing pixels give 5.7–6.0); centred in width |
| Tabs | span 38.35; 13.0 wide; underside 17.80 and top 19.70 above bottom → 1.9 thick |
| Holes | 2 x Ø4.1 (one per tab, on the long centreline), pitch 35.00, open slot to tab end |
| Cable | exits the shaft-side end face, low (EMAX photo, not dimensioned); ~240 mm; JR/FUT plug |
| Weight | 17 g (EMAX); 20.1 g actual (https://www.readymaderc.com/products/details/emax-es3054-17g-metal-gear-digital-servo) |
| Electrical | 4.8–6.0 V; 3.0 kg·cm = 0.294 N·m and 0.15 s/60° = 6.98 rad/s @4.8 V; 3.5 = 0.343 N·m and 0.13 s/60° = 8.06 rad/s @6.0 V |
| PWM | standard direction CCW for 1500 → 1900 µs; max travel 145° ± 10° over 700–2300 µs → 0.091°/µs (0.084–0.097) |
| Not published | dead band, stall current, max frame rate |
| Other | 1 output bearing, metal gears, digital; 3 plastic horns plus mounting hardware (ReadyMadeRC) |

## 3. TowerPro MG90S and SG90
Sources:
- TowerPro: https://www.towerpro.com.tw/product/mg90s-3/ and https://www.towerpro.com.tw/product/sg90-7/
- TowerPro A–F key image: https://towerpro.com.tw/wp-content/uploads/2014/07/小馬達尺寸標示圖B.jpg (A = bottom→spline top, B = body length, C = bottom→case/gear-tower top, D = width, E = tab span, F = bottom→tab underside)
- MG90S datasheet: https://www.electronicoscaldas.com/datasheet/MG90S_Tower-Pro.pdf
- SG90 datasheet: https://tams.informatik.uni-hamburg.de/lehre/2023ss/vorlesung/es/doc/sg90.pdf
- Rapid SG90 sheet: https://static.rapidonline.com/pdf/37-1330t.pdf
- SG90 clone spec sheet: https://www.baud-dance.com/assets/files/舵机SG90规格书-a814e35ca5db3b0d4da2ff7ae3dc3d01.pdf
- DSSERVO MG90S drawing: https://cdn.shopify.com/s/files/1/0673/6848/5000/files/DSSERVO-MG90S-Drawing_Not_all-metal.pdf

| Item | MG90S | SG90 |
|---|---|---|
| A spline top | 32.5 | 30.3 (clone sheet: boss top 26.5 + spline) |
| B body length | 22.8 | 22.7 (22.2 TAMS, 22.5 clone, 23 Rapid) |
| C case / tower top | 28.4 | 27.0 (clone: main case top 22.1, boss top 26.5) |
| D width | 12.4 (A–F table) / 12.2 (dimension line) | 12.2 (11.8 TAMS) |
| E tab span | 32.1 | 32.3 (32.2 TAMS) |
| F tab underside | 18.5 | 17.0 (16 TAMS; 15.6 clone) |
| Tab thickness | not published (1.5 on DSSERVO's own MG90S) | 2.4 (clone) |
| Hole pitch | not published; 27.7 measured from DSSERVO MG90S drawing (scale 28.5 px/mm) | 27.85 (clone) |
| Hole | ~Ø2.0, open slot (DSSERVO drawing) | ~Ø2, slot |
| Shaft from shaft-side end | ~5.9 (measured on DSSERVO MG90S drawing) | 5.9 (measured on clone drawing) |
| Spline | 21T Ø4.80 (MakerWorld table via search; Tinytronics https://www.tinytronics.nl/shop/nl/robotica/motoren/motor/mg90s-mini-servo); DSSERVO MG90S is 25T Ø4.86 | 20T Ø4.9 (clone sheet); 21T in other sources |
| Horns | single, double, cross + 2 mount self-tappers + horn screw (TowerPro set 6) | same 3 types (set 7) |
| Weight | 13.4 g | 9 g |
| Cable | 250 mm, exits shaft-side end low | 250 mm, exits shaft-side end low |
| Torque | 1.8 kg·cm (0.177 N·m) @4.8 V; 2.2 (0.216) @6 V | 1.8 (0.177) @4.8 V; clone sheet 1.3 (0.127) |
| Speed | 0.10 s/60° (10.5 rad/s) @4.8 V; 0.08 (13.1) @6 V | 0.10 (10.5) @4.8 V |
| Dead band | 1 µs (TowerPro site) / 5 µs (datasheet) | 1 µs (site) / 10 µs (datasheets) / 5 µs (clone) |
| Current | stall ~0.7 A @4.8 V, ~0.9 A @6 V; running 220/250 mA (generic MG-90 listing, https://www.newegg.com/p/338-006X-000G4) | stall 0.75 A ±10%, running 90 mA (clone sheet) |
| Pulse | 1 / 1.5 / 2 ms = −90 / 0 / +90° (datasheet); shops quote 500–2400 µs for 180° | same |
| Frame rate | analog, 20 ms (50 Hz) | 50 Hz |

## 4. Bearings

| Bearing | d x D x B (mm) | Notes / source |
|---|---|---|
| 623ZZ | 3 x 10 x 4 | https://olimex.com/Products/Robot-CNC-Parts/Bearings/623ZZ . TR: Rulmanal KWC 40.69 TL incl. KDV, "345 STOKTA" (fetched 2026-10-23): https://www.rulmanal.com.tr/urun/623-zz-kwc-3-10-4-rulman |
| 693ZZ | 3 x 8 x 4 | https://kramp.com/shop-de/en/p/deep-groove-ball-bearing-3x8x4mm-ina-fag--693ZZ . F693ZZ flange Ø9.5 x 0.9: https://www.nsk.com/engineering/f693zzmc5p6-apn.html . TR: Rulmanal 89.33 TL, 55 in stock: https://www.rulmanal.com.tr/urun/693-zz-kwc-3-8-4-rulman |
| 683ZZ | 3 x 7 x 3 | https://www.kugellager-express.de/miniature-deep-groove-ball-bearing-683-zz-3x7x3-mm ; NMB C = 384 N, C0 = 129 N: https://www.nmbtc.com/wp-content/uploads/parts/spec-sheets/L-730ZZRA5P58LG.pdf . Not found at Rulmanal by URL guess |

## 5. How hobby robots provide a rear idler

| System | How the idler works | Source |
|---|---|---|
| Standard-servo bracket kits (MG995/MG996R/DS-size, 40x20x36) | multi-function bracket + short/long U bracket + ball bearing + metal 25T horns. The U leg bolts to the horn; the other leg pivots on a bearing in the bracket under the servo bottom, coaxial with the output | https://vxb.com/fr/products/two-sets-pan-tilt-servo-mount-bracket-for-40x20x36 ; https://scccltd.com/products/2-dof-servo-bracket-kit-mg996r |
| 25T aluminium disc horns used with these kits | Ø20, 4 x M3 tapped on 14 mm PCD, ~4.5 mm thick. The DSSERVO STEP also models this disc (Ø19.95, 4 x Ø2.5 at r 7) | https://probots.co.in/25t-circular-metal-servo-horn-mg996r-mg995.html ; https://st.robu.in/product/aluminum-servo-hornarm-25t-round-type-disc-mg995-mg996 |
| Lynxmotion SES | "C" bracket pivots on the back of the multi-purpose bracket (button-head screw, nylon washer, nylon acorn nut); ASB-09B C bracket comes with ball bearings; 3x8x4 flanged bearing for enlarging the older C-bracket hole | https://ca.robotshop.com/products/lynxmotion-ses-8x3x4mm-flange-bearing-10pk ; Lynx B pan/tilt wiki, seen in search results only (fetch returned 403) |
| Kondo KRS | the servo case itself has a bottom shaft that takes a "free horn"; bearing free horns for KRS-2300 (No.01025) and B3M/KRS-6000 | https://kondo-robot.com/w/wp-content/uploads/KRS_Servo-Manual.pdf ; https://kondo-robot.com/w/wp-content/uploads/KRS_Servo_Catalog_WEB_2023.pdf (search snippets) |
| Plain PWM servos (all four here) | no rear shaft, so the bracket must supply it | STEP + drawings |

## 6. Potentiometer, feedback wire, backlash
- **DS32xx**: no teardown found. Vendor copy claims "precision potentiometers" for DS3218 (https://thinkrobotics.com/products/ds3218-servo, via search). The 180°/270° SKUs on one case are consistent with a pot, but this is inference.
- **Sealing and access**: IP66, opened through 4 bottom screws (STEP).
- **Factory feedback servos**: they bring out the pot wiper as a 4th wire (https://learn.adafruit.com/analog-feedback-servos/about-servos-and-feedback).
- **Backlash**: not published by DSSERVO, EMAX or TowerPro. TR shops state the MG90S inner gears are plastic (listings.json).

## Design inputs
## A. Servo-local CAD frame
Origin = output axis ∩ case-bottom plane. +Y along the shaft toward the spline. +Z toward the FAR end (away from the shaft-side end face). X = Y × Z, i.e. width.

Mapping to the existing PARAMS in cad/openduck.py:
- servo_w = body_w
- servo_l = body_l
- servo_d = case_top_h (horn excluded)
- servo_near = shaft_near
- canonical origin: put it at h = (stack_top)/2

To import the DSSERVO STEP directly: translate by (−10, +20, 0) to put its origin in this frame, then rotate so that STEP +x (toward the shaft-side end) = −Z.

All values in mm and g. "est" means derived or measured from a drawing image; confirm with calipers in the Step-0 bench test.

```python
DS3230MG = dict(  # DS3225MG/DS3218MG identical case
  body_l=40.0, body_w=20.0, case_top_h=40.4, bottom_protrusion=0.3,
  shaft_near=10.0, shaft_far=30.0, shaft_x=0.0,
  boss_d=11.8, boss_top_h=41.5,
  spline='25T', spline_od=6.0, spline_top_h=45.0, horn_screw='M3', screw_depth=8.0,
  tab_span=54.5, tab_w=20.0, tab_t=3.2, tab_under_h=27.7, tab_top_h=30.85,
  holes=4, hole_pitch_l=49.5, hole_pitch_w=10.0, hole_d=4.5, hole_from_tab_end=2.45, hole_open_slot=True,
  case_screws=[(±7.5 across, z=10±17.5 along)], motor_axis_z=20.0, motor_d=17.4,   # bottom flat, no rear boss
  cable_face='shaft_end', cable_h=8.55, gland_w=6.8, gland_h=4.5, gland_out=5.4, lead=300,
  horn_al_arm=dict(hub_d=15.0, h=(42.2, 47.9), holes_d=2.6, holes_r=(20.0, 24.0), clamp='M2'),
  horn_al_disc_25T=dict(d=20.0, hub_d=9.0, h=(42.0, 46.5), holes='4xM3 tapped', pcd=14.0),   # buy separately
  mass=65.0)

ES3054 = dict(
  body_l=28.45, body_w=13.0, case_top_h=26.7,
  shaft_near=6.0, shaft_x=0.0,   # 6.1 derived, 5.7-6.0 from drawing pixels
  spline='23T', spline_od=5.90, spline_top_h=31.1, horn_screw='M3',
  tab_span=38.35, tab_w=13.0, tab_t=1.9, tab_under_h=17.8, tab_top_h=19.7,
  holes=2, hole_pitch_l=35.0, hole_d=4.1, hole_from_tab_end=1.675, hole_open_slot=True,
  cable_face='shaft_end', cable_h=None,  # est ~8, measure
  lead=240, mass=20.0)

MG90S = dict(  # TowerPro A-F
  body_l=22.8, body_w=12.2, case_top_h=28.4,  # C, gear-tower top
  spline_top_h=32.5, tab_span=32.1, tab_under_h=18.5, tab_t=2.4,  # est
  holes=2, hole_pitch_l=27.8, hole_d=2.0,  # est
  shaft_near=5.9,  # est
  spline='21T', spline_od=4.8,  # 20T/25T also reported
  horn_screw='~M2 self-tapping', cable_face='shaft_end', cable_h=3.0,  # est
  mass=13.4)

SG90 = dict(
  body_l=22.7, body_w=12.2, case_main_top_h=22.1, case_top_h=27.0, spline_top_h=30.3,
  tab_span=32.3, tab_under_h=17.0,  # 15.6-17.0 across sources
  tab_t=2.4, holes=2, hole_pitch_l=27.85, hole_d=2.0, shaft_near=5.9,
  spline='20T', spline_od=4.9, cable_face='shaft_end', cable_h=3.0,  # est
  mass=9.0)
```

## B. Actuator values for the MJCF, until the bench test
Values below are at 6.5 V for DS and 6.0 V for the micro servos; damping = stall/ω0.

| Servo | Range of stall torque | Range of ω0 | Damping | Sim default |
|---|---|---|---|---|
| DS3230MG | 2.88–3.30 N·m (DSSERVO Blue/Red) | 5.7–6.0 rad/s (DSSERVO) vs 8.36 (Direnc text) | 0.36–0.55 | worst case: 2.88 N·m, 5.72 rad/s, damping 0.50 |
| DS3225MG | 2.69 N·m | 7.32 rad/s | 0.37 | – |
| ES3054 | 0.343 N·m | 8.06 rad/s | 0.043 | – |
| MG90S | 0.216 N·m | 13.1 rad/s | 0.016 | – |
| SG90 | 0.177 N·m @4.8 V | 10.5 rad/s | 0.017 | – |

DR (my estimates, no published data): backlash ±1.0° on DS and ES3054, ±2° on MG90S/SG90; dead band 0.27° on DS (3 µs), 0.5–1° on the micro servos.

## C. PWM and channel plan (from the specs above)
- **Shared frequency per chip**: PCA9685 frequency is per chip, so analog and fast digital servos cannot share a board.
- **Chip #1, 8 x DS3230 legs**:
  - prescale = 18 → 321 Hz (period 3.11 ms, 0.76 µs/count = 0.068° on the 180° SKU).
  - Do not use prescale 17: that gives 339 Hz, above the 330 Hz spec.
  - Use 200 Hz (prescale 30, 0.11°/count) if the bench test shows heating.
- **Chip #2, 50 Hz** (4.88 µs/count = 0.44°): ES3054 x 3, MG90S x 3, SG90.
  - The ES3054 frame rate is unpublished. Bench-test hip_yaw ES3054 at 200 Hz; if it is OK, move them to chip #1.
- **Pulse calibration**:
  - DS 180° SKU: 500–2500 µs = 180°, 0.090°/µs, neutral 1500.
  - DS 270° SKU: 0.135°/µs.
  - ES3054: 700–2300 µs = 145° ± 10° → 0.091°/µs.
  - MG90S/SG90: calibrate per unit (nominal 1000–2000 µs = ±90°).
- **Spline clocking steps**: 25T 14.4°, 23T 15.65°, 21T 17.1°, 20T 18°. Clock the horn to the nearest tooth, then trim the remainder in software.
- **Joint-range consequences** (cad/openduck.py JOINT_LIMITS, lines 90–95):
  - hip_pitch ±90 uses the whole 180° travel. Use the 270° SKU on hip_pitch, or limit it to ±85.
  - head_yaw ±170 is impossible on an MG90S (~180° total). Limit it to about ±80.
  - neck_pitch −90…40 needs 130° of the ES3054's 135–155°. Put the servo neutral at −25°.
  - knee −90…70: put neutral at −10°.

## D. Recommended bracket and idler pattern for DS3230 leg joints (cradle + U-link)
1. **Cradle (stator)**:
   - Body pocket 40.3 x 20.3 (0.15 mm per side) up to the tab seat at h = 27.7.
   - 4 x M3 through the Ø4.5 tab holes into M3 nut traps (no heat-set inserts in stock).
   - Printed Ø4.3 x 3 locating bosses in the tab holes. Without them, M3 in Ø4.5 gives ±0.75 mm play, which breaks idler coaxiality.
   - Reaction-torque couple at 49.5 mm pitch is ~68 N per side at 3.38 N·m.
2. **Bottom plate** (h −5.4 … −0.4; 0.4 mm gap to the bottom plane):
   - 623ZZ pocket Ø10.1 x 4.0, coaxial with the output axis (10.0 from the shaft-side end, centred in width), with a 1.0 mm lip toward the servo.
   - Use 693ZZ (Ø8.1 pocket) only where radial space is short.
   - Print a press-fit test coupon first.
3. **Shaft-side end wall**: cable slot 8 W x 6 H (h 5.5–11.5), open toward the bottom. Keep 7 mm plus the cable bend radius clear outside the face.
4. **U-link (rotor)**:
   - Leg A, 4 mm, has two options. Option 1 is preferred: bolt it to the 25T Al disc horn (4 x M3 on 14 PCD, h 42–46.5); TR stock is unverified. Option 2 uses the supplied Al clamp arm: capture it in a 0.2 mm-clearance pocket and fasten with 2 x M2.5 through its Ø2.6 holes at r 20/24 plus the M3 axial screw.
   - Do not use the plastic horns on 3 N·m joints.
   - Leg B, 4 mm: M3 nut trap; M3 x 16 SHCS axle through the 623ZZ inner race.
   - Printed spacer OD 4.8 / ID 3.2 so only the inner race is clamped; 0.5 mm running gap.
5. **Axial stack**:
   - Leg B spans h −9.9 … −5.9. Leg A spans 46.5 … 50.5 (disc) or 47.9 … 51.9 (arm).
   - U inner span is ~52.4 mm; overall joint width is ~60 mm.
   - Without an idler the stack is 46.5–47.9 mm.
   - For comparison, the current XL330 stack is (26 + 3) x 1.282 = 37.2 mm at 35 cm.
6. **Where idlers are needed**:
   - Mandatory: knee and hip_pitch.
   - Recommended: hip_roll and ankle.
   - hip_yaw (ES3054, single output bearing, leg hanging below): 623ZZ idler coaxial at 6.0 from the shaft-side end, centred, plus a printed thrust ring around the horn.
   - Head: none, or optionally a 683ZZ on head_pitch.
   - CAD masses: add ~6–8 g per idler joint (bearing 2–3 g + screws + printed spacer; my estimate).

## E. Pot feedback wiring (B2)
- Open the DS3230 through its 4 bottom screws (M2). Solder to the wiper pad AND the pot-low pad, and optionally pot-high for ratiometric reading. Exit through the gland end or a Ø1.5 hole, and re-seal with silicone.
- **Why two wires**: single-ended reading against system GND shifts by I x R_ground. My estimate: 20–60 mΩ x 0.5–3.8 A = 10–230 mV, which is 1–20° at an assumed 6–11 mV/°.
- **Options**: read pot-low on a second ADC (ESP32-S3 ADC2 with Wi-Fi off; verify) or use a differential external ADC. At minimum, run short, heavy separate ground returns.
- **Filter at each pin**: RC (e.g. 1 kΩ + 100 nF).
- **Divider**: add one if the wiper span exceeds ~3.1 V. Measure the span at 6.5 V first.

## F. Rail sizing
- 2 x DS3230 per XL4015 rail: stall 7.6 A (Blue) or 5.8 A (Red), above the 5 A module limit. Use a current limit, one buck per servo, or accept sag.
- Bulk capacitance: ≥ 1000 µF per rail.
- ES3054 stall current is unpublished. MG90S is ~0.7–0.9 A.

## G. Step-0 bench checklist (one DS3230MG and one DS3225 if possible)
- Label: HV or not; 180° or 270°.
- No-load 60° time at 6.5 V.
- Stall current at 6.5 V.
- Calipers: tab_under_h, tab_t, shaft_near, spline OD and tooth count.
- Backlash with a 100 mm arm: 1° = 1.75 mm.
- Wiper span (V) and pot resistance.
- Then decide between DS3230MG and DS3225MG.

## Open questions
- Which DS3230MG does Direnc actually ship? The text says 27/32 kg·cm, 0.16/0.12 s/60° and 4.8–7.2 V; the photo shows an HV unit labelled '30 kg @7.4 V, DC 6~8.4 V'; DSSERVO's own table says 0.18 s/60° (Blue) or 0.17 (Red) at 6.8 V. Is it the Blue or Red variant, and the 180° or 270° SKU? It needs a physical unit or a question to the seller.
- Is the DS3225MG (DSSERVO: 2.75 N·m and 7.48 rad/s at 6.8 V) a better leg servo than the DS3230MG once speed is bench-measured? Robocombo has 99 in stock.
- Does the DS32xx really use a resistive pot with an accessible wiper pad, and what is its span in V at 6.5 V supply? No teardown was found.
- Are 25T aluminium disc horns (Ø20, 4 x M3 on 14 mm PCD) in stock in Turkey? Searches of Robocombo, Robotistan and Motorobit were inconclusive. If not, the supplied clamp arm has to be captured in the printed link.
- ES3054: maximum frame rate (is 200–330 Hz OK?), dead band and stall current are unpublished; they need a bench test.
- Exact MG90S/SG90 clone numbers (tab underside 15.6–18.5, hole pitch ~27.7–27.85, spline 20T/21T/25T) for the Robocombo batch: measure before printing the head brackets.
- Should hip_pitch use the 270° DS SKU (0.135°/µs, 0.4° dead band) to get margin beyond ±90°?
- How should the ~60 mm DS3230 joint stack with U-idler fit into the 35 cm leg layout? The scaled XL330 stack is 37 mm, so hip width and thigh/shin plate spacing need a re-layout.

## Risks
- SPEED: DSSERVO's official DS3230MG no-load speed (5.8–6.2 rad/s at 6.8 V) is 30–33% below the Direnc figure the prior torque analysis used (8.7 rad/s). With the parts-research u formula, B2 goes from u≈0.95 to 1.31–1.38 at 2.5 Hz, and to about 0.95–1.00 at 1.8 Hz, i.e. no margin.
- Direnc may ship an HV DS3230 (6–8.4 V rated, per the product photo). Run at 6.5 V it would be near the bottom of its range, with lower torque and speed than modelled.
- The Direnc DS3230MG listing does not state 180° vs 270° travel. With the 180° SKU, hip_pitch ±90 uses all of its travel.
- Pot-wiper feedback on the DS32xx is unverified, and opening the servo breaks the IP66 seal. A single-ended ADC reading is corrupted by ground-lead IR drop (estimated 1–20° at 0.5–3.8 A) unless the pot-low is also wired out.
- Rail sizing: 2 x DS3230 stall is 5.8–7.6 A per rail, above the 5 A XL4015 limit.
- Joint limits in cad/openduck.py exceed what the servos can do: head_yaw ±170° on an MG90S (~180° total), and neck_pitch needs 130° of the ES3054's 135–155°.
- Micro-servo clone dimensions vary by up to ~3 mm in tab height across sources, and spline tooth counts conflict. Head brackets printed from datasheet numbers may not fit.
- Plastic horns, and printed parts meshing directly with the 25T spline, will creep or strip at 3 N·m. Metal horns are needed, and disc-horn stock in Turkey is unverified.
- Tab-hole clearance (Ø4.5 holes with M3 screws) without locating bosses misaligns the rear idler and binds the joint.
- The joint axial stack grows to ~47.9 mm without an idler and ~60 mm with the U idler, versus 37 mm for the scaled XL330 design. The leg kinematics and collision model need a re-layout, not a param swap.
