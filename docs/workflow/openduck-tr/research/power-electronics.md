# Power and electronics design for B2-ESP32 (battery, servo rails, pot-feedback wiring, PCA9685/MPU6500 interface, electronics mass), using only parts verified in stock in Turkey on 2026-10-23

## Summary
Recommendation: a 4S1P Aspilsan pack with one 20 A buck for all eight DS3230MG servos. Everything below is in stock in Turkey (checked 2026-10-23). Prices include KDV.

**Battery**
- 4 × Aspilsan INR18650A28 as 4S1P: 14.6 V nominal, 16.8 V full, 40.9 Wh. The cells weigh 178 g (datasheet).
- Protection is a balanced 4S 30A BMS (komponentci, 97.47 TL). Firmware cuts off at 13.2 V, measured by an INA226 on I2C.
- 2S2P is rejected. A 2S pack falls from 8.4 V to 6.0 V. The XL4015 needs at least Vout + 1.5 V = 8.0 V in, so only the top ~10% of charge would be usable.
- 3S would work. 4S is chosen because it uses all four cells and halves the current.
- The pack must be spot-welded. Spring holders are rejected because contacts chatter during falls and can brown out the robot.
- Spot welders in stock cost at least 7.6k TL, so either use a welding service or the ready-made fallback: Profuse 4S 1500 mAh 95C LiPo (200 g, 2,528 TL).
- Charger: iMAX B6AC clone (robotistan, 1,797 TL). Budget option: 4S Type-C charge module at 1.12 A (215 TL).

**Servo power**
- One 300 W / 20 A buck set to 6.50 V feeds all 8 DS3230MG (robocombo, 329 TL, 62 in stock). Rated 15 A continuous.
- Estimated leg load while walking: 6.2 A RMS (40 W), with peaks around 18 A.
- I rejected 4 × XL4015 (two servos per rail). Each rail would carry 1.2–1.9 A RMS, but peaks of about 7.3 A exceed the XL4015's 5 A rating.
- One XL4015 at 6.0 V feeds 3 × ES3054, 3 × MG90S and the SG90.
- An MP1584 makes 5 V for the logic. It feeds the DevKitC 5V pin through a 1N5819 diode.
- Protection and distribution: 15 A mini blade fuse, KCD1 rocker as main switch and E-stop, bulk capacitors (2200 µF on each rail, 1000 µF on the battery bus), 16 and 20 AWG silicone wire, XT30 connectors.

**Position feedback**
- The same divider on all 10 wipers: R1 100 kΩ, R2 47 kΩ, 100 nF capacitor at the pin. Ratio 0.320, source impedance 32 kΩ, corner 50 Hz.
- Inputs are GPIO1–10 (ADC1_CH0–9) at 12 dB attenuation. The datasheet's usable range there is 0–2.9 V with ±50 mV error.
- 6.5 V at the wiper reads 2.08 V; 7.2 V reads 2.30 V. The same divider still works if the pot turns out to be fed from 3.3 V or 5 V inside the servo.
- The biggest error under load is not ADC noise. It is the voltage drop on each servo's ground lead. On stock leads this is about 0.9°/A (up to 2.7°/A if the pot runs from an internal regulator).
- Fix: re-lead the servos with 20 AWG, use a star ground, and model the error in sim as a load-dependent bias.

**PCA9685 and IMU**
- PCA9685 VCC is 3.3 V; the V+ terminal is not used.
- Leg board runs at about 305 Hz: 0.80 µs per step, about 0.11° per step. Head board runs at 50 Hz: 4.88 µs per step, about 0.66°.
- The internal oscillator (23–27 MHz in practice) must be calibrated with a capture pin.
- MPU6500 connects over SPI (GPIO39–42): 1 MHz for configuration, up to 20 MHz for sensor reads, with I2C_IF_DIS set.

**Mass**
- Pack about 199 g plus other electronics and wiring about 221 g, about 420 g total. The synthesis assumed 365 g, so this is 55 g more.
- The 300 W module's mass is not published. I estimated 70 g; weigh one before the CAD is finalised.

**Cost**
- New power and feedback lines: 4,523.81 TL, or 2,726.72 TL without the charger. These replace 1,179.72 TL of B1 power lines.
- ESP32, 2 × PCA9685 and MPU6500 (1,032.97 TL) are unchanged.

**Files**
- Calculations: /private/tmp/claude-501/-Users-huseyinarslan-Desktop-ads-z-klas-r-2/8217cd75-2c54-4fa4-b47f-57c38ee96c2b/scratchpad/s8/power-electronics/scripts/calc.py
- Fetched pages: /private/tmp/claude-501/-Users-huseyinarslan-Desktop-ads-z-klas-r-2/8217cd75-2c54-4fa4-b47f-57c38ee96c2b/scratchpad/s8/power-electronics/html/
- Datasheets: /private/tmp/claude-501/-Users-huseyinarslan-Desktop-ads-z-klas-r-2/8217cd75-2c54-4fa4-b47f-57c38ee96c2b/scratchpad/s8/power-electronics/dl/
- The repo was not modified.

## Facts
All prices TRY incl. KDV, checked 2026-10-23 by curl and parsing of schema.org offer, availability and visible stock text. Pricing rules: motorobit and direnc pages list the price before VAT, so I added 20%. Robotistan, komponentci, robocombo, pilpaketi and robolink list prices with KDV included.

## F1. ESP32-S3 / DevKitC-1 facts

| Fact | Value | Source |
|---|---|---|
| ADC1 pins | GPIO1–10 = ADC1_CH0–9 | https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/gpio.html |
| ADC2 pins | GPIO11–20 = ADC2_CH0–9 | same |
| Strapping pins | GPIO0, 3, 45, 46 | same |
| Flash/PSRAM pins | GPIO26–32 for SPI flash/PSRAM; GPIO33–37 also taken with octal flash/PSRAM | same |
| USB-JTAG pins | GPIO19/20 | same |
| GPIO3 strap | Selects JTAG source. With default eFuses (DIS_PAD_JTAG = 0, DIS_USB_JTAG = 0, STRAP_JTAG_SEL = 0) GPIO3 is ignored (Table 3-5) | ESP32-S3 datasheet v2.2, https://www.espressif.com/sites/default/files/documentation/esp32-s3_datasheet_en.pdf |
| ADC2 and Wi-Fi | "ADC2_CH… analog functions cannot be used with Wi-Fi simultaneously" (§4.2.2.1). The IDF oneshot read may return ESP_ERR_TIMEOUT | datasheet; https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/adc/adc_oneshot.html |
| ADC linearity | DNL ±4 LSB, INL ±8 LSB, ≤100 kSPS, measured with an external 100 nF capacitor and Wi-Fi off | datasheet Table 5-5 |
| ADC calibrated error | ATTEN0 0–850 mV ±5 mV; ATTEN1 0–1100 ±6; ATTEN2 0–1600 ±10; ATTEN3 (12 dB) 0–2900 mV ±50 mV | datasheet Table 5-6 (older IDF 4.4 docs say 0–3100 mV) |
| Wi-Fi TX peak current | 340 mA (802.11b, 21 dBm) | datasheet Table 5-7 |
| SPI2 IO_MUX pins | CS0 10, SCLK 12, MISO 13, MOSI 11, WP 14, HD 9. "At 40 MHz or lower, GPIO-matrix routing behaves the same" | https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/spi_master.html |
| DevKitC-1 power | Three mutually exclusive ways to power the board: USB, 5V pin, 3V3 pin | https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32s3/esp32-s3-devkitc-1/user_guide_v1.1.html |
| DevKitC-1 RGB LED | GPIO38 (v1.1), GPIO48 (v1.0) | same |
| Octal-PSRAM pins | GPIO35/36/37 not available on octal-PSRAM modules | same |
| Header J1 | 3V3, 3V3, RST, 4, 5, 6, 7, 15, 16, 17, 18, 8, 3, 46, 9, 10, 11, 12, 13, 14, 5V, G | same |
| Header J3 | G, TX, RX, 1, 2, 42, 41, 40, 39, 38, 37, 36, 35, 0, 45, 48, 47, 21, 20, 19, G, G | same |

## F2. PCA9685

Source unless noted: NXP datasheet Rev. 4 (2015), https://www.nxp.com/docs/en/data-sheet/PCA9685.pdf

| Fact | Value |
|---|---|
| Supply and I/O | VDD 2.3–5.5 V; inputs and outputs are 5.5 V tolerant |
| I2C | Fast-mode Plus, up to 1 MHz |
| PWM frequency | About 24–1526 Hz; prescale = round(osc / (4096·f)) − 1, minimum 3 |
| Clock | 25 MHz typical internal oscillator; external clock (EXTCLK) up to 50 MHz |
| Outputs | Totem-pole by default (25 mA sink, 10 mA source at 5 V); logic 0 at power-up |
| Update timing | "Updating the changes at the end of the LOW cycle", so an update can wait up to one PWM period |
| OE pin | Active low; with OE high, outputs go to 0 by default |
| Oscillator in practice | About 23–27 MHz (Adafruit library example servo.ino, https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library/blob/master/examples/servo/servo.ino) |
| Adafruit 815 board | 9 g with headers and terminal (5.5 g bare), 62.5 × 25.4 × 3 mm (https://www.adafruit.com/product/815) |

## F3. MPU-6500

Source unless noted: product specification rev 1.0, https://ocw.cs.pub.ro/courses/_media/pm/prj2017/amocanu/mpu_6500_rev1.0_1_.pdf

| Fact | Value |
|---|---|
| Supply | VDD and VDDIO 1.71–3.45 V |
| SPI speed | 1 MHz for all registers; 20 MHz for reading sensor and interrupt registers |
| SPI format | 4-wire; data latched on rising SCLK edge; MSB first |
| SPI mode note | Set I2C_IF_DIS to prevent switching into I2C mode |
| Noise | Gyro 0.01 °/s/√Hz; 0.1 °/s RMS at DLPF 92 Hz; accelerometer 300 µg/√Hz |
| GY-6500 module | 3–5 V supply with built-in regulator, I2C/SPI, 15 × 25 mm (https://www.motorobit.com/mpu6500-6-axis-acceleration-and-gyro-sensor-gy-6500) |

## F4. Battery, servo and power-part facts

| Part | Facts | Source |
|---|---|---|
| Aspilsan INR18650A28 | 2800 mAh nominal (2700 min); 3.65 V; charge end 4.2 V; standard charge 1.4 A, max 4.0 A, cut-off 140 mA; 14 A continuous, 25 A conditional (SOC > 70%); end voltage 2.5 V; AC impedance ≤ 20 mΩ; Ø18.3 × 65 mm; 44.5 ± 0.7 g; charge temperature 0–60 °C | Datasheet linked from pilpaketi: https://cdn.qukasoft.com/f/125011/bzR6WWFta24rcTQ4TmkwZ0g0TnI/p/aspilsan-inr18650a28-2900-mah-37v-25a-li-ion-sarjli-pil-6657861d84de4.pdf |
| DS3230MG | 4.8–7.2 V operating; 27 / 32 kg·cm at 5 / 6.8 V; 0.16 / 0.12 s per 60°; 50–330 Hz; 65 g; 40 × 20 × 40.5 mm | https://www.direnc.net/ds3230mg-su-gecirmez-servo-motor |
| DS3230 current | Stall 3.5 A at 5 V, 3.8 A at 6.8 V (red MG variant 2.1 / 2.9 A); idle 4–5 mA; dead band 3 µs. **Taken from search snippets only; aifitlab returned HTTP 429 to my direct fetch** | https://aifitlab.com/products/dsservo-ds3230-mg-servo-motor ; https://servodatabase.com/servo/dsservo/ds3230-mg |
| ES3054 | 4.8–6.0 V; 0.13 s per 60° at 6 V; about 17 g; frame rate and dead band not published | https://www.readymaderc.com/products/details/emax-es3054-17g-metal-gear-digital-servo (search) |
| 300 W / 20 A buck | Input 6–40 V (10–40 V recommended); output 1.2–36 V; 20 A max, 15 A recommended; ripple ≤ 50 mV; 60 × 53 × 27 mm; CV and CC potentiometers; fan needed for sustained high power. No mass published | https://www.motorobit.com/dc-dc-300w-20a-adjustable-voltage-step-down-module ; https://www.komponentci.net/300w-20a-dc-dc-buck-donusturucu-gerilim-voltaj-dusurucu-pmu6642 |
| 300 W / 20 A buck cooling | Passive heatsinks enough below 10–15 A; fan needed above 15 A continuous | https://probots.co.in/dc-dc-buck-converter-step-down-module-constant-current-led-driver-300w-20a.html |
| XL4015 module | 4–38 V in; 1.25–36 V out; needs Vin ≥ Vout + 1.5 V; 0–5 A, 75 W; 180 kHz; short-circuit limit 8 A; no reverse-polarity protection | https://www.komponentci.net/xl4015-5a-ayarlanabilir-dc-dc-voltaj-dusurucu-pmu3119 |
| XL4015 cooling and size | "Actively cooled ... when powering larger loads"; 54 × 24 × 16 mm | https://electrokit.com/en/dc-dc-omvandlare-step-down-1.25-35v-5a |
| XL4015 mass | About 16 g with heatsink (search snippet) | https://www.phippselectronics.com/product/xl4015-5a-dc-dc-step-down-adjustable-power-supply-module-with-heatsink/ |
| XL4016 | 8 A max, under 5 A long-term without a fan, 200 W | https://www.robocombo.com/xl4016-8a-dc-dc-step-down-voltaj-regulatoru-2991 |
| 4S 30A BMS | Overcharge 4.25 ± 0.025 V; overdischarge 2.50 ± 0.08 V; overcurrent 20–30 A; 10 A free-air, 15 A with heatsink, 30 A peak; 50 × 22 × 4 mm; balancing | komponentci / motorobit pages in the BOM |
| OLT 4S 40A balanced BMS | 58 × 43 × 3.4 mm; 4.3 g; overcharge 4.25 V; overdischarge 2.7 V; overcurrent 60 A | https://www.pilpaketi.com/olt-lityum-iyon-bms-4s-40a-balansli-mor |
| Fuse holder (direnc, KLS) | Takes 3.81 × 10.9 mm (mini) blades; 32 V, 30 A; 18 AWG leads | https://www.direnc.net/sigota-yuvasi-kablolu-30cm |
| 15 A mini low-profile fuse | 10.9 × 8.73 mm | https://www.motorobit.com/15a-mini-low-profile-car-fuse-bicak-sigorta |
| KCD1 rocker switch | "Withstands up to DC 12V 20A" | https://www.motorobit.com/kcd1-yellow-dot-illuminated-on-off-switch-12v |
| iMAX B6AC (not original iMAX) | 220 V AC or 11–18 V DC in; Li-ion/LiPo/LiFe 1–6 cells; 0.1–5 A | https://www.robotistan.com/imax-b6ac-professionel-li-po-with-adaptor-ni-mh-charger-balancer-50w |
| Profuse 4S LiPo (fallback) | 1500 mAh 95C: 200 g, 78 × 40 × 30 mm, XT60. 2200 mAh 40C: 243 g, 105 × 35 × 32 mm | motorobit pages in the BOM |

## F5. My calculations

Script: /private/tmp/claude-501/-Users-huseyinarslan-Desktop-ads-z-klas-r-2/8217cd75-2c54-4fa4-b47f-57c38ee96c2b/scratchpad/s8/power-electronics/scripts/calc.py

**Assumptions**
- Torque scale from the 27 cm sim to 35 cm: k = (1.65 / 0.676) × (350 / 273) = 3.13.
- DS3230MG at 6.5 V: stall torque 3.06 N·m; stall current 3.8 A × 6.5 / 6.8 = 3.63 A.
- Supply current is at most I_stall × τ / τ_stall. This is an upper bound.
- RMS torques come from the walk DR + push row of torque-needs.md.

**Per-joint load (upper bound, per servo)**
- hip_roll: 0.41 N·m RMS, 0.48 A
- hip_pitch: 0.63 N·m RMS, 0.74 A
- knee: 0.94 N·m RMS, 1.11 A
- ankle: 0.63 N·m RMS, 0.74 A

**Leg rail**
- All 8 servos: at most 6.2 A RMS, 40 W at 6.5 V. Peaks reach stall (about 3 N·m) on hip_pitch, knee and ankle.
- If split into two-servo rails: knee + ankle 1.86 A RMS, hip_roll + hip_pitch 1.23 A RMS. Either rail can hit about 7.3 A when both servos are near stall.

**hip_yaw (ES3054)**
- 0.19 N·m RMS, which is 55% of its 0.34 N·m stall. This is a thermal risk.

**Power and runtime**
- Output power: legs 40 W, aux rail about 11 W, logic about 0.8 W.
- Input power: about 58 W, assuming 90% / 88% / 85% converter efficiency.
- Pack: 40.9 Wh, of which 34.7 Wh is usable (85%). About 36 min of continuous walking.
- Battery current: about 4.0 A average, about 12.8 A peak (legs 18 A plus aux 5 A).

**Divider (100 kΩ / 47 kΩ, 100 nF)**
- Ratio 0.3197; source impedance 32.0 kΩ; time constant 3.2 ms; corner 49.8 Hz.

| Wiper voltage | 3.3 V | 5.0 V | 6.0 V | 6.5 V | 7.2 V | 8.0 V |
|---|---|---|---|---|---|---|
| ADC voltage | 1.06 V | 1.60 V | 1.92 V | 2.08 V | 2.30 V | 2.56 V |

- Fault case: 16.8 V on a wiper pushes only 132 µA into the pin clamp.
- Loading on the servo's own pot (worst case, mid-travel): 0.84% for a 5 kΩ pot, 1.67% for 10 kΩ.

**Resolution** (assuming 80% of the pot span covers 270° and about 0.76 mV per LSB)
- Pot fed at 6.5 V: 6.2 mV/°, 0.12° per LSB.
- Pot fed at 3.3 V: 3.1 mV/°, 0.24° per LSB.

**Ground-lead error** (error = IR drop on the servo's ground lead)
- Stock lead (22 AWG, 30 cm, plus JR connector, about 26 mΩ):
  - Pot fed from the servo rail: 0.9° at 1 A, 3.4° at stall.
  - Pot fed from an internal regulator: 2.7° at 1 A, 9.7° at stall.
- 20 AWG, 25 cm, soldered (about 10 mΩ):
  - Pot fed from the servo rail: 0.4° at 1 A, 1.3° at stall.
  - Pot fed from an internal regulator: 1.0° at 1 A, 3.7° at stall.

**PCA9685 timing**

| Target | Prescale | Actual | Tick | Per tick (270° servo) | Per tick (180° servo) |
|---|---|---|---|---|---|
| 50 Hz | 121 | 50.0 Hz | 4.88 µs | 0.66° | 0.44° |
| 300 Hz | 19 | 305 Hz | 0.80 µs | 0.108° | 0.072° |

- Oscillator spread of 23–27 MHz means up to ±8% pulse error (±120 µs at 1500 µs).
- I2C write at 400 kHz: 0.79 ms for 8 channels.

**Battery voltage choice**
- 2S runs 8.4 → 6.0 V, but the XL4015 needs at least 8.0 V in: only the top ~10% of charge is usable.
- 3S (12.6 → 9.0 V) and 4S (16.8 → 12 V) both work.

## F6. Out of stock or rejected (checked 2026-10-23)

**Out of stock**
- Chargers: pilpaketi 16.8 V 2 A (614.88) and 3 A (409.92) adapters; robocombo B6AC (Stok : 0); robolink SkyRC B6 V2 (3276); direnc ToolkitRC M7 (3148)
- Connectors and leads: komponentci XT60 pair; pilpaketi 4S balance lead; robotistan JST-XH 5-pin
- Wire: robotistan 16, 18 and 20 AWG pairs; motorobit 22 AWG black and 20 AWG red; robocombo 26 AWG; direnc 30 AWG silicone set
- Other: robolink XL4016 10 A; robotistan 30 cm servo extension; pilpaketi FNIRSI SWM-20 spot welder (3976.21); pilpaketi GOLD single 18650 holder

**Not found in stock**
- Transient-voltage-suppressor diode 1.5KE8.2A

**In stock but rejected**
- pilpaketi 4S1P spring holder (29.20 TL)
- FEIBEN 4 × 18650 PCB holder (69.44 TL, 27 g, 78 × 78 × 21 mm)

## Design inputs
## 1. Power architecture

```
            4S1P Aspilsan A28 (16.8 V full / 14.6 V nominal / 13.2 V firmware cutoff / 10.0 V BMS cutoff)
            cells spot-welded with 0.20 x 6 mm nickel strip; 4S 30A balanced BMS (B-,B1,B2,B3,B+ -> P+/P-)
            JST-XH 5-pin balance lead (charging only)          XT30 female on the pack (16 AWG, 10 cm)
                                                                   |
 robot side: XT30 male -> 15 A mini blade fuse (direnc holder, 18 AWG) -> KCD1 rocker (main switch + E-stop)
             -> VBAT bus (16 AWG) + 1000 uF 35 V at the buck inputs + INA226 VBUS sense (I2C 0x45)
                 |                                 |                                   |
     16 AWG      v                    18-20 AWG    v                      22 AWG       v
   300 W / 20 A buck                  XL4015 5 A                          MP1584EN 5 V 3 A
   CV 6.50 V, CC about 16 A           CV 6.00 V                           -> 1N5819 -> DevKitC "5V" pin
   = LEG bus (16 AWG, under 15 cm)    = AUX bus                           -> board LDO 3V3 -> ESP32-S3,
   + 2 x 2200 uF 16 V at the hub      + 1 x 2200 uF 16 V                     PCA9685 VCC x2, MPU6500, INA226
   -> 8 x DS3230MG                    -> 2 x ES3054 hip_yaw, ES3054 neck,
      (20 AWG +/- per servo)             3 x MG90S head, SG90 mouth
 GROUND: star bar on the servo hub. All buck OUT- terminals and all servo returns go to the bar.
         One 22 AWG wire runs from the bar to ESP32 GND, attached next to the divider R2 bottoms.
         PCA9685 GND reaches the hub only through the signal ribbon.
         Servo current never passes through a PCA9685 board.
```

### Rail table

| Rail | Converter | Set point | Loads | Expected current | Converter rating |
|---|---|---|---|---|---|
| VBAT | 4S pack | 12.0–16.8 V | everything | about 4 A average, about 13 A peak | Cells 14 A continuous / 25 A conditional; XT30; 15 A fuse |
| LEG | 300 W / 20 A buck | 6.50 V ± 0.05 V, set with a meter under about 1 A load | 8 × DS3230MG | at most 6.2 A RMS, about 18 A peak | 15 A recommended, 20 A max |
| AUX | XL4015 | 6.00 V | ES3054 × 3, MG90S × 3, SG90 | about 1.8 A RMS (estimate), 4–6 A during get-up | 5 A max, 75 W; my estimate about 3 A continuous without airflow |
| LOGIC | MP1584EN | 5.0 V | ESP32-S3 (340 mA Wi-Fi TX peak) and logic | under 0.5 A | 3 A |

### Why 4S1P
- 2S2P cannot feed a 6.5 V buck for most of its discharge (see F5).
- 4S gives the most energy from the 4 cells and the lowest current.
- 3S1P (3 cells, 30.7 Wh, 133.5 g) is a valid lighter option. It would need a 3S BMS (in stock at pilpaketi, 37 listings, not checked individually).

### Why one 20 A buck instead of 4 × XL4015
- With 4 × XL4015, each two-servo rail's peak (about 7.3 A) is above the 5 A rating. The rail would sag exactly when the speed-limited knee needs full voltage (u ≈ 0.95).
- One buck lets the peaks of 8 servos share capacity, and the CC potentiometer acts as a current limit.
- Mass is similar: 4 × 16 g = 64 g for the XL4015s against an estimated 70 g for the 300 W module.
- Fallbacks:
  - 2 × XL4016 (robocombo, 120 TL, 340 in stock), one per leg, with a fan.
  - 4 × XL4015 if their peaks prove acceptable on the bench.

### Bulk capacitance
- About 4400 µF on the LEG bus keeps a 10 A step to about 0.1 V of capacitive droop over 50 µs, plus the ESR drop.
- Measure overshoot during hard servo braking in Step 0. If it exceeds 7.0 V (DS3230MG maximum is 7.2 V per direnc), add capacitance or a transient-voltage-suppressor diode with a standoff voltage of at least 6.8 V. None was found in stock.

### Battery management in firmware
- Read the INA226 VBUS at 10 Hz.
- Below 13.2 V (3.3 V per cell) under load for more than 2 s: stop the policy, crouch or sit, then set PCA OE high.
- Below 12.8 V: buzzer or LED alarm.
- The BMS 2.5 V per cell cutoff is only a last resort.
- Optional current sensing: use about 20 cm of 16 AWG ground return as the shunt across the INA226 inputs (about 2.6 mΩ, ±10% drift from the copper temperature coefficient). Or fit a 2 mΩ 2512 shunt (not verified in stock).

### Charging
- iMAX B6AC in LiPo 4.20 V per cell balance mode at 1.4 A (0.5C, the datasheet standard charge), or LiIo mode (4.1 V per cell) for longer cell life.
- Connect the main lead to P+/P− through the XT30 and the balance lead to the JST-XH 5-pin.
- Charge between 0 and 60 °C, with the robot switched off.
- Budget option: motorobit 4S 16.8 V Type-C module, 5 V / 4 A in, 1.12 A out. It relies on the BMS for balancing.

### Thermal
- The 300 W module dissipates about 4–5 W at 6 A. Give it a vent or airflow path in the printed trunk; the XL4015 dissipates under 2 W.

## 2. ESP32-S3-DevKitC-1 N16R8 pin map

| GPIO | Function | Notes |
|---|---|---|
| 1 | ADC1_CH0, L_hip_yaw wiper (ES3054) | 12 dB attenuation |
| 2 | ADC1_CH1, L_hip_roll wiper | |
| 3 | ADC1_CH2, L_hip_pitch wiper | Strapping pin, but ignored with default eFuses (datasheet Table 3-5). Do not burn STRAP_JTAG_SEL |
| 4 | ADC1_CH3, L_knee wiper | |
| 5 | ADC1_CH4, L_ankle wiper | |
| 6 | ADC1_CH5, R_hip_yaw wiper (ES3054) | |
| 7 | ADC1_CH6, R_hip_roll wiper | |
| 8 | ADC1_CH7, R_hip_pitch wiper | |
| 9 | ADC1_CH8, R_knee wiper | |
| 10 | ADC1_CH9, R_ankle wiper | |
| 11–14 | Optional ADC2 ground-sense for L/R knee and ankle (thin wire from each servo's pot-low end, 10 kΩ + 100 nF, 0 dB attenuation) | Usable only with Wi-Fi off (datasheet). Otherwise spare |
| 15 | Input ← PCA#2 ch15 (via 1 kΩ) | Oscillator calibration, measure the period with MCPWM capture |
| 16 | Input ← PCA#1 ch15 (via 1 kΩ) | Oscillator calibration |
| 17 | I2C SDA | 400 kHz. Devices: PCA#1 0x40, PCA#2 0x41 (A0 bridged), INA226 0x45 (A0 = A1 = VS). Board pull-ups are 10 kΩ each, so about 3.3 kΩ in parallel |
| 18 | I2C SCL | |
| 21 | Output → PCA9685 OE on both boards | High = all PWM outputs off (soft E-stop). The boards pull OE low |
| 39 | SPI2 CS → MPU6500 NCS | GPIO matrix; at 40 MHz or below this behaves the same as IO_MUX |
| 40 | SPI2 SCLK → MPU6500 SCL/SCLK | |
| 41 | SPI2 MOSI → MPU6500 SDA/SDI | |
| 42 | SPI2 MISO ← MPU6500 AD0/SDO | |
| 47 | Input ← MPU6500 INT | Data-ready, rising edge; optional, polling also works |
| 38 / 48 | On-board RGB LED (v1.1 uses 38, v1.0 and likely clones use 48) | Blink-test which one, and use the other only after that |
| 0 | BOOT button, used as start/stop | Strapping pin; read it only after boot |
| 19 / 20 | USB CDC/JTAG | Telemetry and logging |
| 43 / 44 | UART0 console | |

Do not use:
- GPIO35–37: octal PSRAM.
- GPIO26–32: flash and PSRAM, not on the header.
- GPIO45/46: strapping pins, leave unconnected.

Using GPIO39–42 disables pad JTAG; USB-JTAG keeps working.

### MPU6500 (GY-6500)
- Wiring: VCC to 3V3, GND, FSYNC to GND if it has a pad, EDA and ECL not connected.
- At startup: wait for the start-up time, then set USER_CTRL (0x6A) bit 4 I2C_IF_DIS.
- SPI mode 3 (data latched on the rising edge). Use 1 MHz for register writes and 10–20 MHz for burst reads of ACCEL/GYRO (0x3B–0x48, 14 bytes).
- Gyro ±2000 dps (falls), accelerometer ±8 g, DLPF_CFG = 2 (92 Hz), A_DLPF about 99 Hz, sample rate 1 kHz.
- Mount it rigidly at the MJCF imu site.

### PCA9685
- VCC from the 3V3 pin. Leave the V+ terminal open and plug no servo into the PCA headers. The hub supplies servo + and −, and the PCA supplies only the signal.
- PCA#1 (0x40), leg board, prescale 19 (305 Hz nominal):
  - ch0 L_hip_roll, ch1 L_hip_pitch, ch2 L_knee, ch3 L_ankle
  - ch4 R_hip_roll, ch5 R_hip_pitch, ch6 R_knee, ch7 R_ankle
  - ch8–9 reserved for hip_yaw if the ES3054 passes a 300 Hz bench test
  - ch15 → GPIO16
- PCA#2 (0x41), 50 Hz (prescale 121):
  - ch0 L_hip_yaw, ch1 R_hip_yaw, ch2 neck_pitch, ch3 head_pitch, ch4 head_yaw, ch5 head_roll, ch6 mouth
  - ch15 → GPIO15
- The MG90S and SG90 are analog and stay at 50 Hz. The DS3230MG is rated 50–330 Hz (direnc).

### PCA9685 oscillator calibration
- At boot, and every 10 s, output a known pulse on ch15 and measure its period with MCPWM capture.
- Compute the actual oscillator frequency, then re-scale all pulse counts.
- This matters because the oscillator spreads over 23–27 MHz, which is ±8% of pulse width.

### Why 300 Hz on the leg board
- Output-update latency drops from up to 20 ms to up to 3.3 ms, because the PCA applies changes at the end of the LOW cycle.
- Steps get finer: 0.80 µs instead of 4.88 µs.
- The DS3230MG dead band (3 µs, from DSSERVO) then becomes the resolution limit.

### Signal levels
- PCA outputs swing 3.3 V. Bench-check that the DS3230MG and ES3054 accept 3.3 V pulses.
- Do not raise PCA VCC to 5 V without an I2C level shifter: at 5 V the input-high threshold is 0.7 × VDD = 3.5 V, above the ESP32's 3.3 V output.

## 3. Position-feedback divider and filter (all 10 leg joints)

### Circuit, per joint
```
servo pot wiper (30 AWG wire-wrap, twisted with that servo's own lead)
  -> R1 100 kΩ -> node -> R2 47 kΩ -> GND bar
                 node -> C 100 nF MLCC (X7R, 50 V) to GND, right at the ESP32 pin -> ADC1_CHx
```
- Place R1, R2 and C on the logic perfboard next to the ESP32, so the high-impedance node stays short. The wiper itself is low impedance (≤ 1.25 kΩ for a 5 kΩ pot).
- Ratio 0.3197; source impedance 32.0 kΩ; time constant 3.2 ms; corner 49.8 Hz.
- 6.5 V → 2.08 V; 7.2 V → 2.30 V. Both are well inside the 0–2.9 V effective range at 12 dB, which avoids the nonlinear top end.
- A fault such as 16.8 V on the wiper injects only 132 µA into the pin clamp, so no clamp diode is needed.
- The 100 nF capacitor matches Espressif's ADC test condition and keeps the sampling-capacitor kickback negligible.

### If the Step-0 bench test shows a lower pot supply
- About 3.3 V: change R2 to 220 kΩ (ratio 0.69, 3.3 V → 2.27 V; same resistor kit).
- About 5 V: keep R2 at 47 kΩ or change it to 100 kΩ (ratio 0.5, 5 V → 2.5 V).

The ES3054 hip_yaw (6.0 V rail) uses the same divider: 6.0 V → 1.92 V.

### Servo-side work
- Open each servo, solder 30 AWG wire-wrap to the pot wiper, and pass it through the grommet. This voids the waterproofing (see the synthesis).
- While the servo is open, replace the + and − leads with 20 AWG silicone, as short as possible. Solder them to the hub or use one JR connector.
- This cuts the ground-lead error by about 2.6× (F5).

### Error budget per joint (estimates; measure in Step 0)
| Source | Estimate |
|---|---|
| ADC raw noise | Assumed 2–6 LSB RMS. After 16× averaging and a 25 Hz low-pass, about 0.06–0.1° (pot at 6.5 V) or 0.12–0.25° (pot at 3.3 V) |
| INL residual after a 5-point per-joint fit | About 0.1–0.5° |
| Wiper loading by the divider | 0.8–1.7% of span, bow-shaped. Static: it shifts the servo's own set point, and the fit absorbs it |
| Ground-lead drop | 0.4°/A (20 AWG, pot on rail) to 1.0°/A (20 AWG, pot on internal regulator); stock leads 0.9–2.7°/A. Up to ±1.3–3.7° at stall. This is the dominant error under load |
| Gear backlash | Inside the servo; not seen as error because the pot sits on the output shaft. Horn and link compliance are not measured |

Compare with the training noise: 0.01 rad (0.57°) on q and 0.5 rad/s on qd.

### Firmware
- ADC1 continuous (DMA) mode, pattern of 10 channels at 20 kSPS total (2 kSPS per channel).
- Average 4 samples, giving a 500 Hz stream.
- Low-pass q with a 2nd-order Butterworth at 25 Hz. For qd, take a filtered derivative (Savitzky-Golay or a 20 Hz Butterworth on the difference).
- Sample both at the 50 Hz policy tick.
- Total q delay about 8–10 ms; qd about 12–15 ms.

### Per-joint calibration, stored in NVS
1. Servo on its real rail (6.5 V or 6.0 V), link on a printed protractor jig with detents at −90, −45, 0, 45 and 90°.
2. At each detent, average 256 samples of raw ADC.
3. Fit angle = cubic polynomial of the raw value (or use a 5-point LUT).
4. Sweep PCA commands in both directions to record the command-to-raw map, dead band and hysteresis for sysID.
5. Hang known loads (for example 0.5 / 1 / 2 N·m) to measure the load-dependent ADC bias directly.
6. Recheck the HOME pose on the assembled robot.

### Optional upgrade
- Ground-sense on ADC2 (GPIO11–14) for the four highest-current joints. Firmware subtracts the servo-local ground. Exact if the pot is fed from an internal regulator. Wi-Fi must be off.
- Or a differential external ADC (not costed).

## 4. Inputs for sim and RL
- Observation delay for q: 10 ± 5 ms; for qd: 15 ± 5 ms.
- q noise σ about 0.3°, plus a torque-proportional bias with slope in [0, 0.45]°/N·m (20 AWG, pot on rail) or up to 1.2°/N·m (pot on internal regulator). Refine with Step-0 data.
- Action latency for leg joints: up to 3.3 ms (PCA period) + 0.8 ms (I2C) + unknown servo internal delay. Randomise 3–12 ms.
- Head joints at 50 Hz: up to 20 ms.
- The rails are regulated, so do not randomise servo speed with battery voltage. Randomise servo torque and speed by ±5% to cover rail droop (about 0.1–0.3 V at peaks).

## 5. Mass budget, to replace MASSES in cad/openduck.py

| Group | Item | Mass (g) | Source |
|---|---|---|---|
| **Battery pack** | | **≈ 199** | |
| | 4 cells | 178.0 | Aspilsan datasheet (44.5 g each) |
| | BMS | 5 | estimate; OLT 40A is listed at 4.3 g |
| | Nickel strip | 3 | estimate |
| | Insulation (fish paper, heat-shrink) | 5 | estimate |
| | Leads, XT30, balance lead | 8 | estimate |
| **Power board** | | **≈ 104** | |
| | 300 W buck | 70 | estimate, not published; weigh one |
| | XL4015 | 16 | Phipps listing |
| | MP1584 | 2 | estimate |
| | INA226 | 2 | estimate |
| | Fuse and holder | 5 | estimate |
| | KCD1 rocker | 4 | estimate |
| | XT30 male | 1.5 | estimate |
| | 1000 µF capacitor | 3 | estimate |
| **Logic** | | **≈ 32** | |
| | DevKitC-1 | 10 | estimate; 63 × 25.4 mm per Espressif PCB drawing |
| | 2 × PCA9685 | 18 | Adafruit 815 (9 g each) |
| | GY-6500 | 2 | estimate |
| | Jumpers | 2 | estimate |
| **Servo hub** | | **≈ 30** | |
| | 7 × 9 perfboard | 15 | estimate |
| | Headers | 4 | estimate |
| | 3 × 2200 µF capacitors | 9 | estimate |
| | Passives | 1 | estimate |
| **Wiring** | | **≈ 55** | |
| | 16 AWG, 0.6 m | 10 | estimate |
| | 20 AWG, 2 m | 12 | estimate |
| | 8 × servo extensions | 32 | estimate |
| | 30 AWG, about 5 m | 1 | estimate |
| **Total** | | **≈ 420** | |

- The synthesis assumed 150 + 215 = 365 g, so this is about 55 g more and the robot comes to about 1.70 kg.
- Saving options:
  - Solder 20 AWG leads directly instead of using extensions (about −20 g).
  - Use a 5 × 7 hub (about −6 g).
  - Use 3S1P (−44.5 g of cells).
- MJCF/CAD items:
  - battery 0.199 kg. 2×2 pack envelope about 40 × 44 × 70 mm with the BMS on one side; or 1×4 flat, about 76 × 20 × 70 mm. Cell Ø18.3 × 65 mm.
  - power 0.104
  - board 0.030
  - imu 0.002
  - servo hub 0.030
  - wiring 0.055, spread over trunk and legs: about 25 g trunk, about 7.5 g per leg segment pair
  - Delete camera, speaker, tof and hat; B2-ESP32 has none.

## 6. BOM, verified 2026-10-23 (TRY incl. KDV)

| # | Item | Qty | Unit | Line | Shop / URL | Stock text |
|---|---|---|---|---|---|---|
| 1 | Aspilsan INR18650A28 2800 mAh | 4 | 124.26 | 497.04 | pilpaketi https://www.pilpaketi.com/aspilsan-pil | schema InStock |
| 2 | 4S 30A BMS, balanced, 50 × 22 × 4 mm | 1 | 97.47 | 97.47 | komponentci https://www.komponentci.net/4s-30a-bms-18650-lityum-pil-sarj-koruma-devresi-pmu6852 | "Stok Durumu Var" |
| 3 | Nickel strip 1P 0.20 × 6 mm (unit length not stated) | 1 | 25.62 | 25.62 | pilpaketi https://www.pilpaketi.com/nikel-serit-1p-0.20-mm-kalinlik-6-mm-genislik | schema InStock |
| 4 | 4S JST-XH balance extension, 22 AWG, 20 cm | 1 | 59.52 | 59.52 | komponentci https://www.komponentci.net/4s-lipo-pil-sarj-dengeleyici-uzatma-kablosu-jst-xh-pmu13788 | Var |
| 5 | XT30 male/female pair | 2 | 17.33 | 34.66 | komponentci https://www.komponentci.net/xt30-li-po-pil-erkek-disi-soket-takim-konnektor-pmu6847 | InStock / Var |
| 6 | iMAX B6AC charger (clone), 220 V AC or 11–18 V DC, 0.1–5 A | 1 | 1797.09 | 1797.09 | robotistan https://www.robotistan.com/imax-b6ac-professionel-li-po-with-adaptor-ni-mh-charger-balancer-50w | schema InStock (Tükendi box hidden) |
| 7 | 300 W / 20 A DC-DC buck, CV/CC | 1 | 329.00 | 329.00 | robocombo https://www.robocombo.com/300w-20a-dcdc-ayarlanabilir-voltaj-dusurucu-modul-2987 | "Stok : 62" |
| 8 | XL4015 5 A buck (AUX 6.0 V) | 1 | 83.39 | 83.39 | komponentci https://www.komponentci.net/xl4015-5a-ayarlanabilir-dc-dc-voltaj-dusurucu-pmu3119 | Var |
| 9 | MP1584EN 5 V 3 A | 1 | 51.87 | 51.87 | komponentci https://www.komponentci.net/mp1584en-5v-3a-ultra-kucuk-boyutlu-dc-dc-step-down-voltaj-regulator-modulu-pmu61596 | Var |
| 10 | INA226 module (VBAT monitor) | 1 | 104.61 | 104.61 | direnc https://www.direnc.net/ina226-iki-yonlu-akim-guc-izleme-sensor-modulu-en | InStock, data-stock-status 1 |
| 11 | Mini blade fuse holder, 18 AWG, 30 cm | 1 | 30.92 | 30.92 | direnc https://www.direnc.net/sigota-yuvasi-kablolu-30cm | InStock |
| 12 | 15 A mini low-profile blade fuse | 3 | 7.27 | 21.81 | motorobit https://www.motorobit.com/15a-mini-low-profile-car-fuse-bicak-sigorta | "100+ Product" (STOCK 360) |
| 13 | KCD1 rocker "DC 12V 20A" (main switch) | 1 | 18.18 | 18.18 | motorobit https://www.motorobit.com/kcd1-yellow-dot-illuminated-on-off-switch-12v | 100+ (STOCK 700) |
| 14 | 1000 µF 35 V electrolytic | 2 | 5.45 | 10.90 | motorobit https://www.motorobit.com/1000uf-35v-electrolytic-capacitor-13x20mm | 1000+ (STOCK 4402) |
| 15 | 2200 µF 16 V electrolytic | 4 | 4.24 | 16.96 | motorobit https://www.motorobit.com/2200uf-16v-electrolytic-capacitor-10x20mm | 1000+ (STOCK 1658) |
| 16 | 1N5819 Schottky diode | 2 | 1.40 | 2.80 | komponentci https://www.komponentci.net/1n5819-1a-40v-schottky-diyot-pmu6322 | InStock |
| 17 | 16 AWG silicone 1 m (red + black) | 2 | 60.00 | 120.00 | robocombo https://www.robocombo.com/16awg-kirmizi-silikon-kablo-1m and https://www.robocombo.com/16awg-siyah-silikon-kablo-1m | Stok : 617 / Stok : 80 |
| 18 | 20 AWG silicone 1 m (2 red + 2 black) | 4 | 24.00 | 96.00 | robocombo https://www.robocombo.com/20wh-od1.8-kirmizi-silikon-kablo-1m and https://www.robocombo.com/20awg-siyah-silikon-kablo-1m | Stok : 790 / 709 |
| 19 | Servo extension 30 cm F-M | 8 | 36.97 | 295.76 | motorobit https://www.motorobit.com/servo-extension-cable-30-cm-female-male | 100+ (STOCK 290) |
| 20 | 30 AWG wire-wrap, 150 m (wiper wires) | 1 | 435.88 | 435.88 | direnc https://www.direnc.net/30awg-250-metre-yesil-kablo-sarma-teli | InStock |
| 21 | 1/4 W metal-film resistor kit, 600 pcs (100K, 47K, 220K × 20 each) | 1 | 145.29 | 145.29 | direnc https://www.direnc.net/1/4w-metal-film-direnc-paketi-600-adet | InStock |
| 22 | 100 nF 50 V MLCC | 20 | 1.21 | 24.20 | motorobit https://www.motorobit.com/100nf-50v-5mm-multilayer-capacitor | 1000+ (STOCK 5186) |
| 23 | 7 × 9 cm double-sided epoxy perfboard | 2 | 45.45 | 90.90 | motorobit https://www.motorobit.com/7x9cm-epoxy-double-sided-perforated-plaque | 100+ (STOCK 224) |
| 24 | 1 × 40 male header | 4 | 4.08 | 16.32 | komponentci https://www.komponentci.net/erkek-pin-header-40-pin-12mm-254mm-1x40-180-tek-sira-pmu6341 | InStock |
| 25 | Heat-shrink set, 530 pcs | 1 | 117.62 | 117.62 | komponentci https://www.komponentci.net/isi-ile-daralan-makaron-kablo-yalitim-seti-shrink-boru-530-parca-pmu10825 | Var |
| | **Power + feedback subtotal** | | | **4,523.81** | (2,726.72 without the charger; replaces 1,179.72 TL of B1 power lines) | |
| L1 | ESP32-S3-DevKitC-1 N16R8 | 1 | 537.62 | 537.62 | komponentci https://www.komponentci.net/esp32-s3-devkitc-1-n16r8-wifi-ve-bluetooth-gelistirme-karti-pmu12351 | Var (alt: direnc 610.23, InStock) |
| L2 | PCA9685 16-channel | 2 | 179.50 | 359.00 | robocombo https://robocombo.com/pca9685-16-kanal-12-bit-pwm-servo-motor-surucusu | Stok : 99 |
| L3 | MPU6500 GY-6500 | 1 | 136.35 | 136.35 | motorobit https://www.motorobit.com/mpu6500-6-axis-acceleration-and-gyro-sensor-gy-6500 | 10+ (STOCK 75) |

### Verified alternatives

**300 W / 20 A buck**
- komponentci 299.10 (Var): https://www.komponentci.net/300w-20a-dc-dc-buck-donusturucu-gerilim-voltaj-dusurucu-pmu6642
- motorobit 545.40 (STOCK 3)
- robolink 573.00 (InStock)

**XL4016 8 A buck**
- robocombo 120.00 (Stok 340)
- direnc 162.73 (InStock)
- robotistan 197.06 (InStock)

**BMS**
- motorobit 4S 30A, 136.35 (STOCK 17)
- robolink 4S 30A, 114.00 (InStock; 15 A rated / 30 A peak)
- robolink 4S 40A, 118.50 (20 A charge / 40 A discharge, 45 × 60 mm)
- motorobit 4S 20A, 151.50 (STOCK 202; 10 A continuous charge, 20 A peak)
- komponentci 4S 20A, 97.47 (Var)
- pilpaketi OLT 4S 40A balanced, 129.00 (InStock, 4.3 g)
- pilpaketi OLT 4S 20A, 118.34 (InStock; overcurrent 30 ± 5 A, 30 mΩ)

**Charger**
- motorobit B6AC, 2727.00 (STOCK 53)
- robotistan iMAX B6 80 W without adapter, 1274.83 (InStock; needs an 11–18 V DC supply)
- motorobit 4S 16.8 V 4 A Type-C module, 215.13 (STOCK 157; 1.12 A charge): https://www.motorobit.com/4s-168v-4a-li-ion-lipo-pil-sarj-devresi
- komponentci 4S 2A Type-C module, 50.20 (Var)

**INA226**
- robotistan 149.91 (InStock)
- motorobit 193.92 (STOCK 217)

**No-weld battery fallback**
- motorobit Profuse 4S 1500 mAh 95C LiPo, 2528.23 (STOCK 99), 200 g, 22.2 Wh, XT60: https://www.motorobit.com/148v-4s-1500mah-95c-lipo-battery
- motorobit 4S 2200 mAh 40C, 2891.83 (STOCK 23), 243 g
- XT60 male to XT30 female adapter: robocombo 54.00 (Stok 494), https://www.robocombo.com/xt60-erkek-xt30-disi-konnektor-4188 . Check connector gender.

**Spot-welding**
- Spot-weld fixtures at pilpaketi cost 180–455 TL (shown in stock).
- The cheapest welder pen without an out-of-stock mark is 7,624.48 TL. Prefer a local pack-welding service (not verified).

## 7. Step-0 bench additions (power and feedback)
Use 1 DS3230MG, 1 ES3054, the 300 W buck, one divider and the ESP32. Record:
- (a) Pot supply voltage, pot resistance and wiper range over the full travel. This decides R2.
- (b) ADC raw noise at 0 / 1 / 2 A load, with and without Wi-Fi.
- (c) Angle error versus hanging load. This gives the ground-drop slope in °/A.
- (d) Rail overshoot during fast reversals.
- (e) Whether the ES3054 accepts 300 Hz.
- (f) Whether 3.3 V PCA pulses are accepted.
- (g) What a digital servo does with no PWM at power-up.
- (h) The 300 W module's mass and efficiency at 14.8 → 6.5 V, 6 A.

## Open questions
- Is the DS3230MG pot fed from the 6.5 V servo rail, or from an internal 3.3 V or 5 V regulator? This decides R2 (47 kΩ by default; 220 kΩ if the pot is about 3.3 V) and how big the ground-lead error is (0.4 vs 1.0°/A with 20 AWG leads). Open one servo in Step 0 to find out.
- Which DS3230MG variant does Direnc ship, 180° or 270°? This sets the angle per PCA step and per ADC LSB.
- Does the ES3054 accept a 300 Hz frame rate? If yes, move hip_yaw to PCA#1 ch8-9. Can its pot wiper be reached, and what is its resistance?
- What do the 300 W / 20 A module's mass, efficiency at 14.8 to 6.5 V and 6 A, and synchronous behaviour (can it sink regenerated current?) turn out to be? No published mass; I estimated 70 g.
- Who will spot-weld the 4S1P pack? Options are a local battery shop (not verified) or buying a welder for at least 7.6k TL. Otherwise accept the Profuse 4S 1500 mAh LiPo fallback (200 g, 22 Wh against 41 Wh).
- Is Wi-Fi needed while the policy runs? With Wi-Fi off, ADC2 (GPIO11-14) can be used for ground-sense on the knee and ankle joints, and for spare analog channels.
- Is the onboard RGB LED of the komponentci/direnc clone DevKitC on GPIO38 or GPIO48?
- What is the ESP32-S3 ADC's real noise on this board next to the 150-180 kHz bucks? No published figure; I assumed 2-6 LSB RMS.
- Do the DS3230MG and ES3054 respond reliably to 3.3 V PCA9685 pulses? And does a digital servo stay limp or hold position when PWM is off (OE high)? This matters for the soft E-stop.
- Exact DS3230MG stall current at 6.5 V: is it about 3.6 A (3.8 A at 6.8 V from the DSSERVO table) or about 2.8 A (red variant)? My only source is search snippets; aifitlab rate-limited direct fetches.

## Risks
- The servo ground-lead voltage drop biases the pot reading under load. With stock 22 AWG leads it is about 0.9°/A, up to 3.4° at stall (pot on the rail), or 2.7°/A, up to 9.7° (pot on an internal regulator). This is larger than the 0.57° training noise. Mitigate with 20 AWG re-leading, a star ground, sim randomisation of a torque-proportional bias, and optionally ADC2 ground-sense with Wi-Fi off.
- The divider loads the servo's own pot by 0.8-1.7% at mid-travel. This shifts the servo's position for a given command slightly; it is static and the per-joint calibration absorbs it.
- The PCA9685 internal oscillator spreads over 23-27 MHz, so pulse widths can be off by up to ±8% (about ±120 µs). Calibrate with ch15 and a capture pin at every boot. Uncalibrated, joint offsets could reach several degrees.
- XL4015 aux rail: peaks during get-up (neck and head saturating) could reach 5 A or more and trip the current limit or overheat. The ES3054 hip_yaw runs at an estimated 55% of stall torque RMS in the DR walk, which is a thermal risk for that servo too.
- If the 300 W buck's CC potentiometer is set too low, or the module sits without airflow in a closed printed trunk, the leg rail will sag at peak demand. The DS3230MG is speed-limited (u ≈ 0.95), so sag costs performance directly.
- Regenerated energy from braking servos can lift the 6.5 V bus. The DS3230MG maximum is 7.2 V per direnc (6.8 V in the task brief). No suitable transient-voltage-suppressor diode was found in stock, so this depends on bulk capacitance; measure it in Step 0.
- Spring 18650 holders were rejected: their contacts chatter in falls, causing brownouts, and their resistance is high. The welded pack depends on finding a welding service. Hand-soldering cells is not recommended.
- The BMS overdischarge threshold (2.5 ± 0.08 V per cell) is too deep for cell life. Protection relies on the INA226 firmware cutoff at 13.2 V, so the firmware has to work.
- The KCD1 rocker is rated DC 12 V 20 A but runs at up to 16.8 V. Derate it to 10 A or less and inspect it for arcing. An XT30 can serve as a backup disconnect.
- The ESP32-S3 ADC's 12 dB range is 0-2.9 V with ±50 mV calibrated error. The real linearity and temperature drift of this clone board are unknown. Use per-joint multi-point calibration on raw codes, not Espressif's mV calibration.
- The charger is an iMAX B6AC clone (robotistan says it is not original iMAX). Never leave charging unattended; charge only between 0 and 60 °C, with the pack out of the robot or the robot switched off.
- Electronics plus battery come to about 420 g against the 365 g the synthesis assumed (+55 g). The 300 W module's 70 g is an estimate.
- Opening each DS3230MG to solder the wiper and re-lead it voids the waterproofing, and the work can damage the pot (already flagged in the synthesis).
- DevKitC power inputs are mutually exclusive. The 1N5819 on the 5 V feed is required so USB and the battery can be connected at the same time without back-feeding.
