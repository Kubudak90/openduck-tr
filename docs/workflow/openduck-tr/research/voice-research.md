# OpenDuck-TR voice module research: DFRobot DF2301Q (SEN0539-EN)

Research date: 2026-10-24. Every fact has a source URL. Anything I could not confirm is marked **unverified**. Items marked "(measured)" were measured by me from DFRobot's own CAD/render files; items marked "(inference)" are my engineering reasoning, not a sourced fact.

Raw files I downloaded for this report (library source, DFRobot CAD PDF, board render, shop HTML) are in `/private/tmp/claude-501/-Users-huseyinarslan-Desktop-ads-z-klas-r-2/8217cd75-2c54-4fa4-b47f-57c38ee96c2b/scratchpad/s16/raw/`.

---

## 0. Summary

1. **Buy locally.** SAMM Market lists the module at **1.171,86 TL incl. KDV, "Stok : 6"** (checked 2026-10-24). Direnc.net, a DFRobot-listed Turkish distributor, has it at 1.817,05 TL incl. KDV. Importing is not worth it.
2. **Use UART, not the shared I2C bus.**
   - A DFRobot forum user reports the module "sometimes pulls the SDA and SCL lines low and doesn't release them", and this is "unrecoverable without a DFR power-reset". That would freeze your PCA9685/ADS1115/INA226 bus.
   - UART is event-driven (9600 8N1) and avoids the library's blocking delays.
   - Source: https://www.dfrobot.com/forum/topic/345770
3. **Turkish learning is plausible but not proven.**
   - DFRobot staff say the built-in system language is fixed but self-learned commands "can be set in different languages" (https://www.dfrobot.com/forum/topic/373279).
   - I found no report of anyone training Turkish words.
   - Only **17 custom commands + 1 custom wake word**. Each word needs **3 utterances**.
   - Learned commands are less accurate than built-ins (https://www.makerguides.com/getting-started-with-gravity-voice-recognition-module/).
4. **No official always-listening mode.**
   - The wake timeout is a 0-255 s setting.
   - There are only anecdotal workarounds, and other users report they fail on I2C.
5. **No servo-noise data exists.** I found no robot-project report and no mic-placement or foam guidance from DFRobot. You must test it yourself.
6. **Revision risk.** Direnc's product photo shows the older V1.0 board, not V1.1. V1.1 added four mounting holes and a plug-in speaker connector. See section 1.
7. **Top alternative sold in Turkey.** The Elechouse-V3-type "Ses Tanıma Modülü V3" is speaker-dependent and trains on any sound/language.
   - Robocombo has it for 1.434,50 TL (stock 4).
   - Motorobit has it for 2.181,60 TL (stock 10+).
   - Weaknesses: max 7 active commands at a time, 5 V TTL UART only, no wake word documented, no I2C.
   - The Hi-Link HLK-V20/V21 and ASRPRO are fixed-vocabulary and configured on a vendor platform, not trained by your voice.

---

## 1. DF2301Q specifications

| Item | Value | Status / source |
|---|---|---|
| Product names | DFRobot SKU SEN0539-EN. Silkscreen "Voice Recognition DF2301QG V1.1". PCB file name "SEN0539 [V1.2.0]", dated 2024-09-05. | CAD PDF https://dfimg.dfrobot.com/wiki/21151/SEN0539-EN_gravity-df2301q-offline-voice-recognition-sensor_cad_V1.2.pdf |
| Hardware revisions | V1.1 (2025-05-08): "Mounting holes have been added" and the speaker became a plug-and-play connector (V1.0 had it soldered). | https://www.dfrobot.com/product-2665.html |
| Board size, spec sheet | 49 x 32 mm | https://wiki.dfrobot.com/sen0539-en/ and https://www.dfrobot.com/product-2665.html |
| Board size, CAD | **50.0 x 32.0 mm**, PCB thickness 1.6 mm, rounded corners | CAD PDF above |
| Mounting holes (V1.1) | **4 holes at a 35.0 mm x 25.0 mm rectangle** (dimensioned in the CAD). | CAD PDF above |
| Hole positions (measured) | Left pair about 11.5 mm from the Gravity-connector edge, right pair about 46.5 mm (about 3.5 mm from the opposite edge). Top and bottom pairs about 3.5 mm from the long edges. | Measured from the CAD and the DFRobot board render in the SVG at https://dfimg.dfrobot.com/wiki/21151/SEN0539-EN_gravity-df2301q-offline-voice-recognition-sensor_svg_v1.2.svg |
| Hole diameter (measured) | About 3.1 mm clear hole inside a 6.0 mm round pad ring, so M3 clearance is likely. **Not dimensioned by DFRobot, so unverified.** Confirm with calipers. | Measured from the same render |
| V1.0 holes | The older V1.0 photo shows holes only on the right-hand side, 2 holes. | Direnc photo https://www.direnc.net/cevrimdisi-ses-algilama-sensoru-diger-sensor-ve-moduller-dfrobot-64734-21-O.jpg |
| Height with components | **Unverified.** No source gives it. Tallest parts are the flat speaker on top and the PH2.0 connectors. | - |
| Mass | **Unverified.** DFRobot, The Pi Hut and TME give no weight. | - |
| Supply voltage | 3.3 - 5 V (Direnc prints 3.3 - 5.5 V) | https://wiki.dfrobot.com/sen0539-en/ |
| Current | **Max <=370 mA at 5 V.** Idle, listening and speaker-playing figures are **unverified** (no source). | https://www.dfrobot.com/product-2665.html |
| Operating temperature | 0 - 70 C | https://www.dfrobot.com/product-2665.html |
| I/O level | Said to be "Compatible with 3.3V/5V" controllers. Whether the data pins follow VCC is **unverified**. | https://www.dfrobot.com/product-2665.html |
| I2C address | **0x64** (fixed). Registers: 0x02 CMD ID, 0x03 play-by-ID, 0x04 mute, 0x05 volume, 0x06 wake time. | https://github.com/DFRobot/DFRobot_DF2301Q (DFRobot_DF2301Q.h) |
| I2C address vs your bus | 0x64 is not a default address of PCA9685 (0x40+), ADS1115 (0x48-0x4B) or INA226 (0x40-0x4F). Datasheet knowledge, not fetched here. | (inference) |
| UART | **9600 baud, 8N1** (`hardSerial->begin(9600, SERIAL_8N1, rx, tx)` for ESP32). Binary frames F4 F5 ... FB with a checksum. | https://raw.githubusercontent.com/DFRobot/DFRobot_DF2301Q/master/DFRobot_DF2301Q.cpp |
| Interface switch | A slide switch selects I2C or UART. A forum user fixed a "getCMDID always 0" problem by re-seating the switch. | https://www.dfrobot.com/forum/topic/402520 |
| Connector | Gravity 4-pin, labelled **D/T, C/R, GND, VCC**. The package includes one Gravity-4P I2C/UART cable. The Robotistan cable listing describes a "PH2.0-4p" connector. The D/T and C/R mapping to SDA/SCL or TX/RX is not in the wiki text (it is an image): **unverified**. | https://www.dfrobot.com/product-2665.html, https://www.robotistan.com/gravity-iic-i2c-uart-4-pin-sensor-kablosu-10lu-paket |
| Microphones | **2 microphones**, sensitivity -28 dB. In the CAD they are two 6.0 mm round parts at the **left end**, one per corner, about 3.4 mm from the left and long edges, **about 25 mm apart**, on the component side. Capsule type and port direction are **unverified**. | CAD PDF + https://wiki.dfrobot.com/sen0539-en/ |
| Speaker | On-board flat speaker with a **25 x 20 mm** footprint, silkscreen "WS-2520-TR", on the top of the board. Impedance, power and thickness are **unverified**. | CAD PDF + render |
| External speaker | Second connector "SPK2" (2-pin) and a slide switch "SPK2 / SPK1" that selects the external or on-board speaker. A DFRobot forum user confirms "The switch is set to external". | CAD PDF, https://www.dfrobot.com/forum/topic/334549 |
| Mute / volume | `setMuteMode(1)` mutes replies. `setVolume(1..7)`. Whether mute persists across power cycles is **unverified**. | Library header, https://www.dfrobot.com/forum/topic/403164 |
| Indicators | Power LED (red) and recognition-status LED (blue). | http://www.tme.eu/Document/064226f2d0b3baf20e9949b0a9b6a625/DF-SEN0539-EN-TME.pdf (copy of the DFRobot wiki, 2023) |
| Recognition distance | **No official figure.** Vendor text only says "longer recognition distance". Anecdotes (4 m / 6 m in snippets) are **unverified**; I could not open the source pages. | https://github.com/DFRobot/DFRobot_DF2301Q |
| Built-in vocabulary | 121 fixed English command words plus 1 fixed wake word ("Hello robot"). DFRobot corrected the ID table on 2026-04-21. The Arduino library page still says "150". | https://wiki.dfrobot.com/sen0539-en/docs/21332, https://www.dfrobot.com/forum/topic/401847 |
| Latency | **Nobody measured it.** Library polling delays are not latency. | - |

### 1.1 Custom words: learning, wake word, wake-free, Turkish

| Question | Answer | Source |
|---|---|---|
| Custom command slots | **17**, IDs **5-21** in the order learned (first = 5, ... seventeenth = 21) | https://wiki.dfrobot.com/sen0539-en/docs/21332 |
| Custom wake words | **1** learned wake word, learned-wake ID = **1**. The default "Hello robot" is ID **2**. Both can wake the module. | Same page, TME copy |
| Wake-word learning | Wake with the default word, then say **"Learning wake word"**. Prompts: "Learning now, be quiet, please say the wake word to be learned!", then say it 3 times (the prompt "Learning successful, please say it again!" appears twice), then "Ok, learning completed!". The TME copy says "then say 'Hello robot'", which looks like a doc typo. | Same page, TME copy |
| Command learning | Wake, say **"Learning command word"**. For each command: say it 3 times in a row; the module then asks for the next. Say **"Exit learning"** to leave. | Same page |
| Deleting | Wake, say "I want to delete", then choose "Delete command word", "Delete wake word", "Delete all" or "Exit deleting". Re-learn ID is 202. | Same page |
| Caveat | The docs say "before learning a new command word, delete the previous one". Because the same flow also continues with "the second command", I read this as applying to re-learning. **Unverified.** Run "Delete all" first, then learn all 12. | TME copy |
| Voice prompts | Fixed English prompts in the wiki. Some units were reported speaking **Chinese** or another foreign language. Check your unit on arrival. | https://dfrobot.com/forum/topic/346509, https://www.dfrobot.com/forum/topic/334185 |
| Can fixed commands be overwritten? | No (forum answer by a user, not DFRobot staff). | https://www.dfrobot.com/forum/topic/335494 |
| Non-English words | DFRobot staff: "The system language of the voice recognition module is fixed, but the self-learning voice commands can be set in different languages." Another DFRobot post says commands can be recorded "in any language or even non-verbal sounds". **Turkish specifically: unverified.** | https://www.dfrobot.com/forum/topic/373279, https://community.dfrobot.com/makelog-318396.html |
| Replay of learned word | Learning stores only a recognition template. Replies are fixed "ok/done" style sounds (random pick). | https://www.dfrobot.com/forum/topic/403164 |
| Wake timeout | `setWakeTime(0-255)` seconds. The default is **unverified** (examples use 15 or 20). A user's code comment says "wake time max is 4 minutes" (255 s). One user says a new value only takes effect the next time it wakes. | Library header, https://www.dfrobot.com/forum/topic/335967, https://www.dfrobot.com/forum/topic/357160 |
| Wake-free / always listening | **No official support.** Anecdotes: |
| | - One user says that on UART, after waking, calling `setWakeTime(255)` immediately and again every 200 s keeps it awake. | https://www.dfrobot.com/forum/topic/335967 |
| | - Another says the enter-wake command (`DF2301Q_UART_MSG_CMD_SET_ENTERWAKEUP`) did nothing with values 0-3, and renewing the time in I2C did not work. | https://www.dfrobot.com/forum/topic/335967, https://www.dfrobot.com/forum/topic/357160 |
| | - Another says the wake period only extends on a valid command. | https://www.dfrobot.com/forum/topic/345770 |
| | - `setWakeTime(0)` "pauses listening" and one user could not resume. | https://www.dfrobot.com/forum/topic/335077 |
| | - One user abandoned always-on because "the commands would execute without intent". | https://www.dfrobot.com/forum/topic/357160 |

### 1.2 Library API (DFRobot_DF2301Q v1.0.1, MIT)

Sources: https://github.com/DFRobot/DFRobot_DF2301Q, DFRobot_DF2301Q.h and .cpp. README history lists 1.0.1 dated 2026/10/22 (a `#pragma pack()` fix).

```
DFRobot_DF2301Q_I2C(TwoWire* = &Wire, uint8_t addr = 0x64);   bool begin();
uint8_t getCMDID();           // 0 = nothing recognised. Blocks 50 ms (delay(50)).
void    playByCMDID(uint8_t); // blocks 1000 ms. "ID 1 enters wake state in I2C" (users report this failing).
uint8_t getWakeTime();  void setWakeTime(uint8_t 0-255);
void    setVolume(uint8_t 1-7);  void setMuteMode(uint8_t 0/1);

DFRobot_DF2301Q_UART(HardwareSerial*, rx, tx)   // ESP32 variant
bool begin();  uint8_t getCMDID();   // non-blocking: parses bytes already received
void playByCMDID(uint32_t);          // 1000 ms delay
void settingCMD(uint8_t type, uint32_t value);  // 100 ms delay
   types: SET_VOLUME(0x80) 1-7, SET_ENTERWAKEUP(0x81) 0, SET_MUTE(0x83) 0/1, SET_WAKE_TIME(0x84) 0-255
void resetModule();                  // 3000 ms delay
```

- The UART frame also carries notifications: `NOTIFY_WAKEUPENTER` (0xB1), `NOTIFY_WAKEUPEXIT` (0xB2), `PLAYSTART` and `PLAYEND`. These constants are defined in the header; whether the firmware actually sends them is **unverified**.
- Because the library blocks, write your own small non-blocking reader on a FreeRTOS task. The I2C registers and UART frame format are in the header.
- `begin()` on UART returns true even with nothing connected (https://www.dfrobot.com/forum/topic/402520).
- Whether `defined(ESP32)` is true on ESP32-S3 Arduino core builds is **unverified**.

### 1.3 Command-ID map

Learned commands are IDs 5-21. Learned wake word is ID 1. Default wake word is ID 2. Learning/management commands are IDs 200-208. The fixed-command table (IDs 22-142) is at https://wiki.dfrobot.com/sen0539-en/docs/21332 (updated 2026-04).

Examples from the current table: 22 Go forward, 23 Retreat, 24 Park a car, 25-30 Turn left/right 90/45/30 degrees, 31 Shift down a gear, 32 Turn on car light, 33 Turn off car light, 103 Turn on the light, 104 Turn off the light.

Older TME copy (2023): 32 "Line tracking mode" and 33 "Light tracking mode", i.e. the table changed. A user found the old table did not match his device ("Turn on the car light" sent 32); DFRobot said on 2026-04-21 that the list was corrected: https://www.dfrobot.com/forum/topic/401847. Verify IDs on your own unit by printing `getCMDID()` for each word.

**Mapping for 12 Turkish commands** (my suggestion): command 1 -> ID 5 ... command 12 -> ID 16, leaving IDs 17-21 spare.

---

## 2. Known problems

| Problem | Evidence |
|---|---|
| **I2C bus lock-up**: module "sometimes pulls the SDA and SCL lines low and doesn't release them", unrecoverable without a power reset. Another user: "UART doesn't make it freeze" and recommends UART. Long I2C cables caused lag or missed detections. | https://www.dfrobot.com/forum/topic/345770 |
| Module "stops working after a few minutes", needs reset. No confirmed root cause. One reply suggests `setWakeTime(255)`. | Same thread |
| **getCMDID() always 0**: the real cause was the I2C/UART slide switch not making contact. | https://www.dfrobot.com/forum/topic/402520 |
| **Learned commands less accurate than built-ins**; similar phrases confuse ("Turn on the red light" often heard as "Turn on the light"). Deleting/editing by voice is cumbersome. | https://www.makerguides.com/getting-started-with-gravity-voice-recognition-module/ |
| Replies cannot be customised; no documented way to change the response voice. Mute is the only option. | https://www.dfrobot.com/forum/topic/334558, https://www.dfrobot.com/forum/topic/403164 |
| Wake-up / enter-wake from the host is unreliable. | https://www.dfrobot.com/forum/topic/335967, https://www.dfrobot.com/forum/topic/357160 |
| Some units arrived speaking Chinese (one user), or "a foreign language not English" (another). | https://dfrobot.com/forum/topic/346509, https://www.dfrobot.com/forum/topic/334185 |
| No custom playback of learned words; needs a separate DFPlayer. | https://www.dfrobot.com/forum/topic/403164 |
| **Servo/motor noise and false triggers**: no robot-project reports found. Vendor claims "relatively accurate and reliable even in noisy environments" with no data. | https://community.dfrobot.com/makelog-313323.html |
| Mic placement or foam: no DFRobot guidance found (**unverified**). A search snippet attributed generic "motor/servo noise" advice to a Parallax forum post and a robot-vacuum patent; I could not open the Parallax page, so treat it as **unverified**. | - |

**Practical advice (inference, not sourced):**
- Keep the module on its own 5 V supply branch with local decoupling, away from the 6.5 V servo buck and its wiring.
- Mount the board on soft standoffs or silicone grommets so servo vibration does not reach the mic capsules, and keep the mic side facing a clear opening.
- Do not put foam directly over the capsule openings.
- Train your 12 words and the wake word in the same acoustic setup and with servos energised, so the templates include the noise.
- Choose phonetically distinct words so the learned templates do not collide.

---

## 3. Turkey stock and price for the DF2301Q (checked 2026-10-24)

Prices include the shop's own KDV figure where the page shows it. I read the pages by script (JSON-LD plus visible text). The HTTP 403 shops could not be read.

| Shop | Listing | Price (TL) | Stock as shown | URL |
|---|---|---|---|---|
| **SAMM Market** (market.samm.com, SAMM Teknoloji, Gebze) | "Gravity: Çevrimdışı Ses Tanıma Sensörü - I2C ve UART", SKU MP03951, supplier code SEN0539-EN | **1.171,86 incl. KDV** (976,55 + KDV; page also shows EUR 21.00) | "Stok : 6", Sepete Ekle, JSON-LD InStock | https://market.samm.com/gravity-cevrimdisi-ses-tanima-sensoru-i2c-ve-uart |
| **Direnc.net** | "Çevrimdışı Ses Algılama Sensörü Modülü", Stok Kodu SEN0539-EN, SKU T21829, brand DFRobot | 1.514,20 + KDV = **1.817,05 incl. KDV** | Sepete Ekle shown, JSON-LD InStock, **no quantity shown**. Product photo is the V1.0 board. | https://www.direnc.net/cevrimdisi-ses-algilama-sensoru |
| Özdisan | "SEN0539-EN" (DFRobot) | price on request ("Fiyat ve Stok Talep Et") | "Stok: 0", "Gelecek Stok Yok" | https://www.ozdisan.com/p/arduino-sensorleri-ve-modulleri-613/dfrobot-sen0539-en-1203428 |
| Domirobot | Site search for DF2301Q / SEN0539 / "ses tanıma" returned no matching product | - | not found | https://www.domirobot.com/arama?tip=1&word=DF2301Q |
| Robotistan | Site search for DF2301Q / SEN0539 returned no matching product | - | not found | https://www.robotistan.com/arama?q=DF2301Q |
| Motorobit | Site search for DF2301Q / SEN0539 returned no matching product | - | not found | https://www.motorobit.com/arama?q=DF2301Q |
| Empa | Search for DF2301Q / SEN0539 says "Arama kriterlerinize uygun ürün bulunamadı" | - | not found | https://empa.com/arama?q=SEN0539 |
| Robocombo | Storefront search is client-side rendered; I could not query it. Web searches with robocombo.com showed no DF2301Q listing. | - | **unverified** | https://robocombo.com/ |
| Hepsiburada, Trendyol, n11, Akakçe | Automated fetches got HTTP 403; web searches restricted to these domains returned no DF2301Q listing | - | **unverified** | e.g. https://www.hepsiburada.com/ara?q=DF2301Q |
| Farnell Turkey | A search result lists SEN0539-EN on tr.farnell.com; the page returned 403 and the price in the snippet looked like a data error | - | **unverified** | https://tr.farnell.com/dfrobot/sen0539-en/gravity-voice-recognition-module/dp/4829832 |

Notes:
- DFRobot's distributor page lists three Turkey distributors: **Direnc.net, Domirobot, Özdisan** (names read from logo file names and link URLs, not text). https://www.dfrobot.com/distributor
- SAMM Market shows free shipping from 1500 TL; a single module is below that, so a shipping fee applies (amount not shown). Same-day shipping for orders before 15:00 on weekdays (same SAMM page). Direnc also shows "1500₺ ve üzeri kargo ücretsiz".
- Revision risk: the Direnc photo shows the V1.0 board ("DF2301QG V1.0"). I could not determine which revision SAMM ships (no photo checked). If you need all four mounting holes, ask the shop or look at the board on arrival.
- Neither Turkish listing shows weight or height.

**Import route (not recommended):**
- DFRobot direct is **US$16.90** (3+ pieces $16.50; 5+ $16.00; 10+ $15.50; stock not stated): https://www.dfrobot.com/product-2665.html
- Shipping: DHL/FedEx 2-7 business days to Europe/Asia (no price on the page); registered mail to Turkey 20-40 days, "not guaranteed and is unstable". Terms are FOB Shanghai, the buyer pays duties and taxes, and no Turkey duty rate is published: https://www.dfrobot.com/shipping
- Free shipping needs an order over $150. At the current Turkish price, a single local unit is cheaper once courier and duty are added.
- **AliExpress: no listing found, unverified.**
- UK: The Pi Hut shows £16.30 incl. VAT, "Only 5 units left" (https://thepihut.com/products/gravity-offline-language-learning-voice-recognition-sensor-for-arduino-raspberry-pi-python-esp32). Shipping to Turkey is unverified.

---

## 4. Alternatives sold in Turkey

Facts below are from the shop listings checked 2026-10-24 and the vendor pages named.

| Module | Turkish shop, price, stock | Interface | Trainable by your voice? | Notes |
|---|---|---|---|---|
| **Elechouse-style "Voice Recognition Module V3"** | **Robocombo** "Ses Komutu Tanıma Modülü" (description says "Ses Tanıma Modülü V3"): **1.434,50 TL incl. KDV**, "Stok : 4", shows "Tükeniyor" (https://robocombo.com/ses-komutu-tanima-modulu-insan-sesi-algilayici-m-3934). **Motorobit** "Arduino Ses Tanıma Modülü" SKU SNS.SES.01.000006: 1.818,00 + KDV = **2.181,60 TL**, "Stokta 10+ ürün" (https://www.motorobit.com/arduino-ses-tanima-modulu). **n11** "Arduino Ses Tanıma Modülü": 2.330,32 TL, JSON-LD InStock, third-party seller (https://www.n11.com/urun/arduino-ses-tanima-modulu-15582901). | UART + GPIO, **5 V TTL**, 4.5-5.5 V, <40 mA, 31 x 50 mm, 3.5 mm mono mic jack. Default baud **unverified**. | **Yes, speaker-dependent.** "Any sound could be trained as command." Up to **80** commands stored, **only 7 active at a time**, each up to **1500 ms**. Accuracy "99% (under ideal environment)". | Elechouse page https://www.elechouse.com/product/speak-recognition-voice-recognition-module-v3/. **No wake word is documented** (unverified). The 7-active limit means your 12 commands need group switching. 5 V logic needs a level shifter on the ESP32-S3 RX line (inference). |
| Geetech / Elechouse V2-style "Ses Tanıma Seti" | **Robotistan** SKU 18966: **1.986,12 TL incl. KDV**, in stock (page shows the Sepete Ekle button; the "Tükendi" box is hidden) (https://www.robotistan.com/arduino-ile-uyumlu-ses-tanima-seti). **Direnc** Geetech T259: 1.743,53 TL, **out of stock** (https://www.direnc.net/arduino-ses-tanima-modulu-arduino-voice-kit). | 5 V TTL UART, 4.5-5.5 V, <40 mA, about 30 x 47.5 mm | Listing says "15 voice commands, 3 groups of 5" and never mentions training. **Trainable: unverified** (the older Elechouse design is, inference). | Only 5 active per group. Worse fit than V3 for 12 commands. |
| SAMM "Ses Komutu Tanımlama Modülü" | **SAMM Market** MP00967: 1.730,62 TL, **OutOfStock** (https://market.samm.com/en/voice-command-recognition-module-en) | not shown | unverified | Out of stock. |
| EasyVR 3 Plus shield | **Robotistan** SKU 13950: price shown as **0,00 TL**, no stock info, so treat as unavailable (https://www.robotistan.com/easyvr-shield-3-arduino-icin). SparkFun lists it as retired. | UART, 3.3-5 V | **Yes**: up to 256 user-defined speaker-dependent commands in 16 groups (up to 32 SD each), "trained in ANY language". Built-in speaker-independent sets: US English, French, German, Italian, Japanese, Spanish (no Turkish). | https://sparkfun.com/products/15453. Original price about $53 at SparkFun. A good technical fit but I found no live Turkish supply. |
| **ASRPRO-01** | **Motorobit** SKU SNS.SES.01.000047: 540,35 + KDV = **648,42 TL**, **out of stock** ("Gelince Haber Ver") (https://www.motorobit.com/asrpro-01-ses-tanima-modulu). Also seen as a Motorobit listing on Hepsiburada (403, not opened). | UART/I2C/PWM/SPI/GPIO, 9600 bps, 3.6-5 V, >500 mA | Listing: "kullanıcı tanımlı girişleri tanımlayabilmektedir", up to 300 entries, 10 m open space, 0.1 s, 98%. **How words are defined, whether it is voice-trained, and whether Turkish works: unverified.** | The Tianwen toolchain for ASR-PRO is firmware-based (per one Spanish retailer page; unverified). Not an on-device voice trainer as far as I can confirm. |
| **HLK-V20** / **HLK-V21** (Hi-Link) | **Motorobit**: V20 277,75 + KDV = **333,30 TL**, "Stokta 6 ürün" (https://www.motorobit.com/hlk-v20-ses-tanima-modulu). V21 732,25 + KDV = **878,70 TL**, "Stokta 10+ ürün", dual mic (https://www.motorobit.com/hlk-v21-offline-ses-tanima-modulu). Test kits with speaker+mic also listed. | UART, I2C, SPI, PWM, GPIO | **No.** Fixed set of "150 local commands"; wake and command words are "configured for free on the backend". Turkish unverified. | Vendor-platform configuration, not learned from your voice. |
| **VC-02** (Ai-Thinker) | **Not found** in Robotistan, Direnc, Motorobit, Domirobot or Empa searches (https://www.robotistan.com/arama?q=VC-02, same pattern on other shops). Web searches limited to Hepsiburada/Trendyol/n11 also found none. | UART, 3.6-5 V, US516P6 chip | **No.** Custom wake/command words are defined in a browser tool (voice.ai-thinker.com), "Chinese and English", up to 150 offline commands. | https://techiesms.com/product/ai-thinker-vc-02-kit-best-offline-voice-recognition-board/, datasheet https://download.mikroe.com/documents/datasheets/VC-02_datasheet.pdf. Not Turkish. |
| **LD3320** | **Not found** in the five Turkish shops searched. | SPI/parallel | Keyword list in Chinese pinyin, about 50 entries, speaker-independent. | https://www.toughdev.com/content/2014/08/ld3320-chinese-speech-recognition-and-mp3-player-module/ (from search summary, not fully verified). Chinese-oriented. |
| M5Stack Module ASR / CI-03T class | Not found in Turkish shops (unverified). | UART | No (firmware regeneration, Chinese/English/Japanese/Korean words, up to 300 commands). | https://docs.m5stack.com/en/module/Module_ASR |
| On-board option: ESP32-S3 + I2S mic + own Turkish keyword model | INMP441 I2S mic at Motorobit: 164,13 TL + KDV, shown "Tükendi" (out of stock) on the page that listed it. | I2S | You would train it yourself | A DFRobot staff reply points to ESP-SR (online training tool) and Edge Impulse as alternatives (https://www.dfrobot.com/forum/topic/403370). Not tested here. |

Other facts that matter:
- "ses tanıma modülü" searches at Robotistan, Direnc and Motorobit gave the items above and nothing else relevant.
- The Hi-Link and Ai-Thinker modules are fixed-vocabulary and not trainable from your voice.

---

## 5. Unverified list

- DF2301Q: board height with components, mass, speaker impedance/power/thickness, idle/listening/speaker-playing current, official recognition distance, latency, default wake timeout, I/O logic level at 5 V VCC, D/T and C/R mapping, capsule port direction.
- Turkish word learning (no Turkish test found).
- Whether SAMM or Direnc stock is revision V1.1 (four holes) or V1.0.
- Hepsiburada, Trendyol, n11, Akakçe, Robocombo and Farnell Turkey listings (not readable by script).
- AliExpress price/availability.
- Whether the Elechouse V3 has any wake word and its default baud (the Elechouse page does not say).
- How ASRPRO-01 defines words and whether it supports Turkish.
- Parallax/robot-vacuum noise advice and the 4 m / 6 m distance anecdotes (source pages not retrievable).

---

## 6. Source list

- DFRobot wiki: https://wiki.dfrobot.com/sen0539-en/ and https://wiki.dfrobot.com/sen0539-en/docs/21332 (also /docs/21330, /docs/21331)
- DFRobot product page: https://www.dfrobot.com/product-2665.html
- DFRobot CAD/SVG: https://dfimg.dfrobot.com/wiki/21151/SEN0539-EN_gravity-df2301q-offline-voice-recognition-sensor_cad_V1.2.pdf
- DFRobot library: https://github.com/DFRobot/DFRobot_DF2301Q
- TME copy of the wiki (2023): http://www.tme.eu/Document/064226f2d0b3baf20e9949b0a9b6a625/DF-SEN0539-EN-TME.pdf
- DFRobot forum: topics 335077, 335494, 335967, 345770, 357160, 373279, 346509, 334185, 334549, 334558, 401847, 402520, 403164, 403370 at https://www.dfrobot.com/forum/topic/<number>
- Makerguides: https://www.makerguides.com/getting-started-with-gravity-voice-recognition-module/
- CNX Software: https://www.cnx-software.com/2023/04/21/offline-voice-recognition-module-arduino-programming/
- DFRobot distributors: https://www.dfrobot.com/distributor
- DFRobot shipping: https://www.dfrobot.com/shipping
- Elechouse V3: https://www.elechouse.com/product/speak-recognition-voice-recognition-module-v3/
- SparkFun EasyVR 3 Plus: https://sparkfun.com/products/15453
- M5Stack Module ASR: https://docs.m5stack.com/en/module/Module_ASR
- Turkish shop pages as linked in sections 3 and 4.
