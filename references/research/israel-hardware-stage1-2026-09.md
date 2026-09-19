# Stage-1 robot hardware: buying it in Israel (research as of 2026-09-16)

Scope: a first small differential-drive robot (Pi 5 + microcontroller + 2 encoder gear motors) that later grows into a ROS 2 robot with LiDAR, camera and IMU. Every price below was read on the named page on **2026-09-16** unless marked otherwise. Prices include 18% Israeli VAT where the store is Israeli.

Exchange rate used: **1 USD = 3.033 ILS** (Bank of Israel representative rate, 2026-09-16, from `https://boi.org.il/PublicApi/GetExchangeRates`).

Confidence labels used in this file:
- **[V]** verified: I opened the page and read the price, stock or spec myself.
- **[S]** search snippet only: a search engine summary or listing, not a page I opened.
- **[E]** my estimate or general engineering knowledge, not checked against a source in this session.

Many Israeli sites (4project, DigiKey IL, Mouser IL, TME, raspberrypi.com docs) sit behind Cloudflare. 4project was read through a real browser session. DigiKey IL was partly readable and Mouser IL was not readable at all.

---

## (a) Israeli suppliers, checked

| Supplier | URL | What it's good for | Notes (2026-09-16) |
|---|---|---|---|
| **Piitel (פייטל)** | https://piitel.co.il/ | **Raspberry Pi (official)**, Pico, Pi accessories, Waveshare and Yahboom robot parts and kits | [V] The only Israel entry on the Raspberry Pi Approved Reseller list (https://www.raspberrypi.com/resellers/?q=&country=1). Pi 5: 2GB ₪320 (out of stock), **4GB ₪500**, **8GB ₪800**, 16GB ₪1,390 (in stock). 27W PSU ₪70, Active Cooler ₪30, Pico 2 with header ₪35, Yahboom 520 encoder motor ₪65. Ships by UPS in up to 5 business days. Pickup only by appointment after an online order; you can't browse in person. |
| **4Project** | https://www.4project.co.il/ | **Pololu, SparkFun, goBILDA and Makeblock distributor**: quality motor drivers, regulators, ToF sensors, wheels, hubs, casters, XT60, breadboards | [V] Warehouse at Derech HaAtzmaut 43, Yehud. Pololu lists 4Project as its Israeli distributor (https://www.pololu.com/distributors/0J118). Shows live stock ("מלאי שלנו" / "חסר במלאי"). Pi 5 4GB ₪504.90 (out of stock), 8GB ₪784. Encoder motors here are mostly premium Pololu/ServoCity (₪300–413 each). Cheap exception: Makeblock 25D 9V optical-encoder motor ₪100.90 on sale. Blocks bots, so browse it normally. |
| **Hackstore / אלקטרוניכאן** | https://hackstore.co.il/ | Chinese modules and motors (JGA25/JGB37), TB6612 boards, BMS boards, Li-ion pack chargers, buck converters, INA219/INA226, US-100, fuses, switches, solder, multimeters | [V] Rehovot. The footer says HaYetzira 19; a search snippet gives HaMinof 4. MCU boards cost more here (Pico 2 ₪80 vs ₪35 at Piitel). Good for power parts. |
| **Sollan (סולן)** | https://www.sollan.co.il/ | Li-ion cells (Samsung 35E), LiPo packs, buck converters | [V] Samsung INR18650-35E pair ₪98 (sale), 3S 2200mAh 35C LiPo with XT60 ₪342, 5V/5A buck (9–35V in) ₪198. Pickup in Rishon LeZion. Same-day delivery in Gush Dan for an extra fee [S]. |
| **KSP** | https://ksp.co.il/ | microSD cards, general tools | [V] (via KSP's search API) Kingston Canvas Select Plus 64GB ₪57, SanDisk High Endurance 64GB ₪79, SanDisk Extreme Pro A2 128GB ₪140. Arctic precision screwdriver kit ₪59, Hunter 78W hot-glue gun ₪35, Bolt 30W iron ₪45. **No Raspberry Pi boards found.** |
| **RCZone** (RC hobby) | https://rczone.co.il/ | LiPo balance chargers | [V] RCToolkit C3 2–3S LiPo/LiHV charger ₪250, on sale ₪220. |
| **C-Hobby** (RC hobby) | https://c-hobby.co.il/ | LiPo safety bag | [V] 20×10 cm fire-resistant bag, about ₪35. The price came from structured data on the page and may be a list price. |
| Dominator (RC) | https://www.dominator.co.il/ | LiPo packs and chargers | [S] Gens Ace 3S 2200mAh ₪350. The Spektrum charger page returned 404. |
| Tisaney Dor (טיסני דור) | https://dorhm.co.il/ | RC batteries and chargers | [V] The site is up, but product URLs from search (AC-3S10 charger ₪90 [S], IMAX B6AC) now return 404. |
| Skyline (קו האופק) | https://skylineairline.co.il/ | RC LiPo chargers and bags | [S] Listings exist; the product page returned 404 when fetched. |
| Cell-Tec | https://www.cell-tec.co.il/ | 18650 cells | [S] Samsung 35E ₪29 each in the snippet. The product URL redirected to the homepage, so this is unverified. |
| LaBatteria | https://www.labatteria.co.il/ | e-bike packs; could build a custom 3S pack | [V] The site exists. I did not check whether they build small packs or what it costs. |
| Belline / Belshop | https://www.belshop.co.il/ | General components, Raspberry Pi | [S] Lists a Pi 5 8GB; price not read. |
| DigiKey Israel | https://www.digikey.co.il/ | Anything catalogued, billed in ILS | [V] Delivery page: ILS orders arrive in 7–10 business days and DigiKey handles customs clearance. USD orders reach Israeli customs in about 3 days and you release them yourself. Free-shipping threshold not verified (see b). |
| Mouser Israel | https://www.mouser.co.il/ | Same | [S] "Free shipping on most orders over ₪400". The site blocked my fetch. |
| Farnell Israel | https://il.farnell.com/buy-raspberry-pi | Raspberry Pi, components | [V] The page exists. Terms not checked. |
| TME | https://www.tme.com/il/en/ | Components; stocks Pololu | [S] Blocked by Cloudflare. |
| Ivory, Bug | https://www.ivory.co.il/ , https://www.bug.co.il/ | Consumer electronics (SD cards, USB-C chargers) | [V] Both sites exist. Search found no Raspberry Pi listings for either. |
| Zap (price comparison) | https://www.zap.co.il/ | Consumer electronics only | [V] "raspberry pi 5" returns **no results**, so Zap is not useful for maker parts. |
| micro:bit Israel / LightHouse | https://www.microbit.co.il/ | Education, micro:bit | [V] The site exists; stock not examined. |
| Alltech, Robokit | https://www.alltech.co.il/ , https://robokit.co.il/shop/ | Educational kits, DJI drones | [V] The sites exist. Not component sources. |

Candidate names from the brief that did **not** check out:
- **Kitsat**, **Elkotec**, **Nisko**, **Chip Center**, **Anodit**: no connection from my fetch, and no matching store in search results. **Unverified; don't rely on them.**
- **Dash Electronics** (Petah Tikva): listed as a Pololu distributor (https://www.pololu.com/distributors/0J119), but the domain I guessed didn't resolve.
- **arduino.co.il**: redirects to a Google Sites login, so it's effectively not a store.
- **rav-bariach**: a door and lock company, not relevant.
- Sites like pinum.co.il, asif-bag.co.il and drivers.org.il show up when searching for TS101 or Pinecil. They look like machine-translated AliExpress mirrors. **Avoid.**

---

## (b) International options and import rules

### Israeli personal-import VAT exemption (it changed three times in 2025–26)

- **Current: USD 75.** Since **2 June 2026**, goods worth up to $75 enter free of all tax. From $75 to $500, 18% VAT only. Above $500, customs duty and purchase tax may also apply.
- History:
  - It was raised from $75 to $150 (Finance Ministry announcement 26 Nov 2025, amendment published 23 Dec 2025).
  - The Knesset revoked that.
  - The minister signed a $130 order effective 25 Feb 2026.
  - The Knesset revoked that on 1 June 2026 (59–23), so the limit went back to $75 on 2 June 2026.
- Sources:
  - [V] Kol-Zchut: https://www.kolzchut.org.il/he/זכותון_בנושא_יבוא_אישי_(חבילות_מחו"ל) ("עד 75 דולר פטורים... בין 25.02.2026 ל-01.06.2026 הפטור ניתן עד 130 דולר")
  - [V] ICL Global, 02.06.2026: https://www.iclglobal.com/news_update/vat-exemption-threshold-for-personal-imports-to-israel-updated/
  - [S] Times of Israel: https://www.timesofisrael.com/liveblog_entry/knesset-revokes-smotrichs-order-expanding-personal-import-tax-exemption-to-130/
  - [S] Ministry of Finance press release, Nov 2025: https://www.gov.il/en/pages/press_26112025 (blocked when fetched)
- **Does shipping count toward the threshold? Sources disagree.**
  - A snippet from israelaliexpress.co.il and the Dec 2025 amendment wording say the threshold is measured on the goods alone when shipping is listed separately, but once tax is due, VAT is charged on goods + shipping + insurance.
  - ShemeshPhone (May 2026) says the threshold includes shipping.
  - **Practical rule: keep each order's goods value under $75, and don't split one order to dodge tax.** This is political and may change again, so re-check before ordering.

### Where to buy abroad

| Source | Terms relevant to Israel | Source / confidence |
|---|---|---|
| **Amazon.com** | Free shipping to Israel restored for orders **over $49** of eligible items (not everything qualifies). Delivery about 1 month instead of 2 weeks. | [V] ynetnews, published 2026-04-17: https://www.ynetnews.com/business/article/s1r6pkytwx |
| **AliExpress** | Standard delivery about 15–30 days; express 5–10 days. **Items containing lithium batteries often fail to ship to Israel**: sellers ship, but AliExpress's hub returns the parcel. | [S] https://wowcoupons.co.il/guides/shipping-times-israel/ (low-quality source). [V] https://zuzu.deals/262548/aliexpress-failed-shipping/, **but that article is dated 2024-06-17**, so the current state is unverified. **Buy cells and packs locally.** |
| **DigiKey (digikey.co.il)** | ILS: 7–10 business days, customs cleared by DigiKey. USD: you clear customs yourself. Free-shipping threshold: a snippet says ₪800, but it traces to a **2012** press release. Incoterms (CPT vs DDP) not confirmed. | [V] https://www.digikey.co.il/en/help-support/delivery-information/delivery-time-and-cost ; [S] threshold |
| **Mouser (mouser.co.il)** | "Free shipping on most orders over ₪400". | [S] only |
| **Pololu (direct)** | FedEx International Connect Plus or Priority. Local distributors: 4Project and Dash Electronics. Romi Chassis Kit $39.95, Romi Encoder Pair $9.95. | [V] https://www.pololu.com/product/3500 , /3542 ; [S] https://www.pololu.com/ordering |
| **RobotShop** | Ships internationally. Duties, taxes and fees are paid by the buyer on delivery. Stocks Cytron MDD3A and Yahboom 520 motors. | [S] https://www.robotshop.com/policies/shipping-policy |
| **Waveshare (direct)** | Worldwide shipping. "All additional local taxes, duties… are customers' responsibilities." UGV01 tracked chassis $169.99. | [V] https://www.waveshare.com/shipping , https://www.waveshare.com/ugv01.htm |
| **Yahboom (direct)** | 520 encoder motor: $7.90 bare, $8.40 with bracket, $8.90 with coupling, **$9.90 with bracket + coupling + tire**. Shipping cost to Israel not checked. | [V] https://category.yahboom.net/products/md520 (Shopify JSON) |
| **PINE64 store** | Pinecil V2 $25.99. Shipping to Israel not checked. | [V] https://pine64.com/product/pinecil-smart-mini-portable-soldering-iron/ |

### Batteries and mains

- **Buy lithium cells and packs locally** (Sollan, 4Project, Hackstore, RC shops). Chargers, BMS boards and holders contain no cells, so they can be imported normally.
- [E] Israel is 230 V / 50 Hz with Type H sockets, which also accept 2-pin Type C Europlugs. Chargers and PSUs rated 100–240 V are fine with a plug adapter. **Avoid 110 V-only US tools** such as some soldering stations and glue guns. Israeli retail tools carry Israeli or EU plugs.

---

## (c) Component tables

### Architecture review: keep it, with these changes

1. **Pi 5 + Pico 2 over USB serial: keep.** [E]
   - The Pico 2's PIO decodes quadrature in hardware. It runs at 3.3 V logic and costs ₪35 at Piitel.
   - An ESP32 is only worth it if you want Wi-Fi micro-ROS without the Pi.
   - Start with a simple serial protocol plus a ROS 2 node, or ros2_control, on the Pi. micro-ROS on the RP2350 is possible but is not a Stage-1 need; I didn't verify its RP2350 support.
2. **Use 12 V motors on a 3S (11.1 V nominal, 12.6 V full) Li-ion pack.** [E]
   - Choose the **~200–330 RPM** versions. The 1:19 / 550 RPM versions are too fast for indoor navigation.
   - At 90 mm wheels, 205 RPM is about 0.97 m/s top speed and 333 RPM about 1.57 m/s. Both are plenty.
3. **Don't choose the motor driver by rated current. The 520-class motors stall at 3–4 A** ([V] Yahboom spec). [V]/[E]
   - **DRV8833 is ruled out:** its motor supply maximum is 10.8 V [E, TI datasheet], below a full 3S pack.
   - **TB6612FNG** (15 V abs max, 1.2 A continuous / 3.2 A peak [E, datasheet]) works for a light robot, but a stall can trip its thermal shutdown.
   - **Cytron MDD3A** (4–16 V, 3 A continuous / 5 A peak [S, cytron.io]) is the best-value dual driver. **I didn't find it in Israel.**
   - The local robust option is **2× Pololu DRV8874 carriers** (4.5–37 V, 2.1 A continuous per 4Project's title, current-sense output).
4. **Why L298N is a poor choice** [E; from ST's L298 datasheet, not re-fetched this session]:
   - Its bipolar (Darlington-style) H-bridge drops roughly **1.8–3.2 V at 1 A, and up to about 4.9 V at 2 A**. On an 11.1 V pack, the motors see only about 8–9 V.
   - That lost voltage becomes heat (big heatsink, wasted battery), so low-speed PWM control is poorer.
   - The typical red board's onboard 78M05 regulator and 5 V-oriented design are clumsy with a 3.3 V Pico.
   - It's large and has no current sensing.
   - MOSFET drivers (TB6612, DRV887x, MDD3A) drop tenths of a volt.
5. **Powering the Pi 5 from a buck converter** [S, summarising Raspberry Pi docs and forums: https://github.com/raspberrypi/documentation/blob/master/documentation/asciidoc/computers/raspberry-pi/power-supplies.adoc]:
   - A buck feeding 5 V has no USB-PD negotiation, so the Pi 5 limits downstream USB current to **600 mA**.
   - Set `usb_max_current_enable=1` in config.txt (or `PSU_MAX_CURRENT=5000` in EEPROM) to allow 1.6 A for LiDAR and USB cameras later.
   - Use a ≥5 A regulator, short thick leads, and a common ground.
   - Feeding the GPIO 5 V pins bypasses the Pi's input protection, so add your own fuse and polarity protection.
6. **Pack voltage and current: an INA219 or INA226 module on I²C** [E].
   - Common modules use a 0.1 Ω shunt: the INA219 then reads up to about 3.2 A and the INA226 only up to about 0.8 A.
   - The bus-voltage reading is fine either way. For whole-robot current, swap the shunt (e.g. 0.01 Ω) or use a Pololu ACS724/ACS723-type sensor.
   - The cheapest option is a 100 k / 22 k divider into a Pico ADC pin.
7. **HC-SR04's 5 V echo pin needs a 1 k / 2 k divider** into the 3.3 V Pico [E]. The **US-100 runs at 3.3 V** [E], so no level shifting.

### C1. Compute

| Part | Recommended (where, price, date) | Alternative | Compatibility / connector notes |
|---|---|---|---|
| Main computer | **Raspberry Pi 5 4GB**: Piitel ₪500 [V] https://piitel.co.il/shop/raspberry-pi-5/ | **8GB ₪800** at Piitel (in stock) or 4Project ₪784 [V] https://www.4project.co.il/product/raspberry-pi5-8g | 4GB is enough for headless ROS 2 + Nav2/SLAM Toolbox at Stage 1 [E]. Take 8GB if on-board rviz or vision models are planned. |
| Cooling | **Official Active Cooler**: Piitel ₪30 [V] https://piitel.co.il/shop/active-cooler/ | — | Required for sustained ROS 2 load [E]. |
| microSD | **Kingston Canvas Select Plus 64GB**: KSP ₪57 [V] https://ksp.co.il/web/item/399645 | SanDisk Extreme Pro A2 128GB ₪140 (KSP item 214036). Piitel official A2 32GB ₪26, **out of stock** [V]. | An NVMe SSD via M.2 HAT+ is a later upgrade (Piitel M.2 HAT+ ₪70, out of stock) [V]. |
| Bench PSU | **Official 27W USB-C PSU (EU)**: Piitel ₪70 [V] | — | For desk work while the robot is on a stand. The EU plug fits Israeli sockets [E]. |
| Microcontroller | **Raspberry Pi Pico 2 with header**: Piitel ₪35 [V] https://piitel.co.il/shop/raspberry-pi-pico-2-with-headers/ | 4Project Pico 2 without headers ₪33.20 [V]. Hackstore Pico 2 ₪80. ESP32-S3 at Hackstore ₪105–115 [V]. | USB serial to the Pi. 3.3 V I/O. Four PIO-decoded encoder inputs. |

### C2. Drivetrain

| Part | Recommended | Alternative | Notes |
|---|---|---|---|
| Encoder gear motors ×2 | **Yahboom 520 DC gear motor with Hall encoder**: Piitel ₪65 each [V] https://piitel.co.il/shop/520-dc-gear-motor-with-encoder-550rpm/ . **Ask for the 205 RPM (1:56) or 333 RPM (1:30) version.** The page title lists all three but shows one price and no variant selector. | Hackstore **JGA25-371 12V 400 RPM with encoder** ₪105 [V]: 1:9.8, 11 pulses/rev, 4 mm shaft, too fast. Pololu 25D 280 RPM encoder motor at 4Project ₪346.90 [V]. **Makeblock 25D 185 RPM 9V optical encoder** ₪100.90 sale [V]. Yahboom direct $9.90 incl. bracket, coupling and tire [V]. | [V] Yahboom spec (https://www.yahboom.net/public/upload/upload-html/1740736196/0.%20Motor%20introduction%20and%20usage.html): 12 V rated (11–16 V ok), rated 0.3 A, **stall 3 A (1:30) / 4 A (1:56)**, 6 mm D eccentric shaft, 11-line magnetic Hall AB encoder, **3.3–5 V encoder supply with built-in pull-ups**, **PH2.0 6-pin** connector. **1:30 gives 330 pulses per wheel revolution per channel, 1,320 counts with 4× decoding. 1:56 gives 616 and 2,464.** Whether Piitel includes the PH2.0 cable is unverified. |
| Motor brackets ×2 | **JGB37-520 metal bracket**: Hackstore ₪20 each [V] https://hackstore.co.il/product/מחזיק-מנוע-jgb37-520-מתכת/ | Yahboom bracket bundled if bought direct | [E] Fits the 37 mm gearbox face (M3). The Yahboom 520 is JGB37-style, but check the hole pattern. |
| Wheel hubs | **Pololu universal aluminum hub, 6 mm, M3, 2-pack**: 4Project ₪60.90 [V] https://www.4project.co.il/product/aluminum-mounting-hub-6mm-shaft-m3-holes-2pack | Coupling from a Yahboom or AliExpress wheel kit | Set screws onto the 6 mm D shaft. |
| Wheels | **Pololu 90×10 mm wheel pair (black)**: 4Project ₪50 [V] https://www.4project.co.il/product/500 | 70×8 mm pair ₪39.30 [V]. 65 mm rubber wheels from a JGB37 kit on AliExpress [S]. | [V] Pololu says these wheels have M3 holes at 12.7 and 19.1 mm spacing, **compatible with the universal hubs** (https://www.pololu.com/product/1439). The 10 mm width is fine indoors. |
| Caster | **Pololu 3/4" metal ball caster**: 4Project ₪21.10 [V] https://www.4project.co.il/product/986 | 1/2" metal ₪13.90 [V] | [E] Match heights: 45 mm axle height minus the motor-bracket offset. You'll likely need M3 spacers under the caster. |
| Chassis plate | **Piitel "Smart Robot Car Chassis Kit / 2 wheels"** ₪50 [S: price from Piitel search listing; product page didn't show it] https://piitel.co.il/shop/smart-robot-car-chassis-kit-2-wheels/ . Use the acrylic plates, drill for the brackets, keep the TT motors as spares. | Piitel "Robot Car Chassis 2WD" ₪122 [V]. **DIY 3–4 mm plywood, acrylic or aluminium plate** [E], cut or 3D-printed; a two-deck plate helps later for LiDAR. | TT-motor plates aren't drilled for 37 mm motors, so expect to drill. A two-level plate with 30–40 mm standoffs leaves room for the Pi, LiDAR and battery. |
| Motor driver | **2× Pololu DRV8874 single brushed DC motor driver carrier**: 4Project ₪60.60 each [V] https://www.4project.co.il/product/drv8874-single-brushed-dc-motor-driver-carrier | Budget: **Pololu TB6612FNG dual carrier** 4Project ₪30.30 [V] https://www.4project.co.il/product/1145 (Hackstore ₪35–50). 2× Pololu TB9051FTG ₪65.70 each [V]. Dual TB9051FTG Pi HAT ₪182.34 [V] (HAT form factor; less useful with a Pico). **Cytron MDD3A** via RobotShop [S]. **Avoid DRV8833** (≤10.8 V) and **L298N**. | Wire PWM/PH/EN lines from the Pico. DRV8874 CS output goes to a Pico ADC for stall detection. Add a 100–470 µF bulk capacitor near the drivers [E]. |

### C3. Power

| Part | Recommended | Alternative | Notes |
|---|---|---|---|
| Cells | **Samsung INR18650-35E ×4** (3 plus 1 spare): Sollan, 2 pairs at ₪98 = ₪196 [V] https://www.sollan.co.il/product/זוג-סוללות-ליתיום-samsung-inr18650-35e-מקצועיות-בהספק-א/ | 4Project LG HJ2 18650 ₪39.90 each [V] (discharge rating not checked). Cell-Tec 35E ₪29 [S]. | Sollan's page says "20A"; Samsung's datasheet rates the 35E around **8 A continuous** [E]. Budget about 3 A typical and up to 8 A for motor stalls. **Buy matched, new cells, never loose "flat-top" cells of unknown origin.** |
| Holder | **3×18650 holder**: Hackstore ₪16 [V] https://hackstore.co.il/product/מחזיק-3-סוללות-18650/ | Spot-welded 3S pack from a local pack builder (LaBatteria?) [unverified] | Spring holders add resistance. Fine at a few amps; check them for heat [E]. |
| BMS | **3S 12.6V 40A charge/balance board**: Hackstore ₪45 [V] https://hackstore.co.il/product/בקר-טעינה-ואיזון-עד-3s-12-6v-40a/ | Hackstore 3S 25A (out of stock) | Gives over-discharge and short-circuit protection. Charge through the BMS P+/P− [E]. |
| Charger | **12.6V 2A charger for 3-cell Li-ion packs**: Hackstore ₪80 [V] (search listing) https://hackstore.co.il/product/ספק-כוח-מטען-מארז-סוללות-ליתיום-12-6v-2a/ | Hackstore 12.6V 3A "medical standard" ₪220 [V]. Hackstore **B3 PLUS 2S/3S LiPo balance charger** ₪115 [V] (for LiPo). RCZone C3 ₪220 [V]. | 230 V input. Check the output barrel size and fit a matching jack on the robot (one listing says 3.5×1.35 mm) [S]. Charge on a non-flammable surface or in a LiPo bag. |
| 5 V regulator for the Pi 5 | **Pololu D42V55F5 (5 V, 5.5 A)**: Hackstore ₪170 [V] https://hackstore.co.il/product/מוריד-מתח-d42v55f5-מיצב-5v-עד-6a/ | Pololu D36V50F5 (5 V / 5.5 A) at 4Project ₪202.60 [V]. Budget: **Hackstore UBEC 5A** ₪75 [V]. Sollan 5V/5A buck (9–35 V in) ₪198 [V]; its 9 V minimum input is too close to a discharged 3S pack. | See the Pi 5 notes above (`usb_max_current_enable=1`). Keep 5 V leads short. |
| Fuse | **5×20 mm panel fuse holder ₪20 + 10 A fuse ₪5**: Hackstore [V] https://hackstore.co.il/product/בית-נתיך-5x20-ממ-לשקע-פאנל/ | Inline automotive blade-fuse holder from a car-parts store [E] | Put it right after the battery, before the switch [E]. |
| Power switch | **DPST rocker 29×21 mm**: Hackstore ₪25 [V] | Hackstore XT60 inline switch ₪65 [V] | **Current rating not shown on the listing.** Pick one rated ≥10 A DC. |
| Connectors | **XT60 male ₪5.10 + female ₪4.80** (×2 pairs): 4Project [V] https://www.4project.co.il/product/xt60-male-connector | Hackstore XT60 pair ₪18 [V] | XT60 for the battery-to-robot link. JST-PH 2.0 6-pin for the motors. Screw terminals on the driver side. |
| Voltage/current sensing | **INA219 module**: Hackstore ₪40 [V] https://hackstore.co.il/product/מודול-חיישן-מתח-וזרם-ina219/ | INA226 module ₪55 [V]. Resistor divider into Pico ADC about ₪5 [E]. Pololu/SparkFun sensors at 4Project (INA169 ₪91.60 [V]). | Shunt-range caveat above. |

### C4. Sensors (Stage 1)

| Part | Recommended | Alternative | Notes |
|---|---|---|---|
| Ultrasonic | **US-100**: Hackstore ₪40 [V] https://hackstore.co.il/product/חיישן-מרחק-אולטראסוני-us-100/ | **HC-SR04**: 4Project ₪11.90 [V]. Needs a 5 V supply and an echo divider to 3.3 V. | The US-100 runs at 3.3 V and also has a UART mode [E]. |
| ToF | **Pololu VL53L1X carrier (with regulator)**: 4Project ₪107.80 [V] https://www.4project.co.il/product/vl53l1x-distance-sensor-regulated-pololu | Pololu VL53L0X ₪93.70 [V]. Hackstore GY-53-L1X ₪95 [V] (UART/PWM module). AliExpress bare VL53L0X boards [S, price not read]. | I²C to the Pico or Pi. VL53L1X reaches about 4 m vs about 2 m for VL53L0X [E]. |
| IMU (Stage 2) | — | Piitel 10-DOF ICM20948 + LPS22HB module ₪81 [V] | For later. |

### C5. Prototyping

| Part | Recommended | Notes |
|---|---|---|
| Breadboard | 4Project 830-point ₪10.90 [V] https://www.4project.co.il/product/breadboard-830-points | Hackstore 830 ₪15 [V] |
| Jumpers | Piitel "Premium Jumper Wires 40×150mm" ₪16 [V, search listing] + Piitel 40× female-female 200 mm ₪23 [V] | Buy a set of male-female too (type not verified on the premium set). |
| Standoffs | M2.5 nylon or brass kit for the Pi and Pico: about ₪30 [E] | Not found at Piitel or Hackstore in quick searches. Hackstore stocks M3/M4 nylon spacers at ₪1–4 each [V]. |
| Wire, terminals, heat shrink | 18 AWG silicone red/black for power, 24 AWG for signals, screw terminals: about ₪60 [E] | Hackstore heat shrink ₪6–14 per size [V]. |

---

## (d) Recommended Stage-1 BOM with totals

All local (Israel) purchases, prices read 2026-09-16, ILS including VAT. USD at 3.033.

| # | Item | Store | ₪ |
|---|---|---|---|
| 1 | Raspberry Pi 5 **4GB** | Piitel | 500.00 |
| 2 | Active Cooler | Piitel | 30.00 |
| 3 | microSD 64GB (Kingston Canvas Select Plus) | KSP | 57.00 |
| 4 | Official 27W USB-C PSU | Piitel | 70.00 |
| 5 | Pico 2 with header | Piitel | 35.00 |
| 6 | 2× Yahboom 520 encoder motor (205 or 333 RPM) | Piitel | 130.00 |
| 7 | 2× JGB37-520 metal bracket | Hackstore | 40.00 |
| 8 | Pololu 6 mm hub (2-pack) | 4Project | 60.90 |
| 9 | Pololu 90×10 mm wheels (pair) | 4Project | 50.00 |
| 10 | Pololu 3/4" metal ball caster | 4Project | 21.10 |
| 11 | 2WD acrylic chassis plate kit | Piitel | 50.00 |
| 12 | 2× Pololu DRV8874 carrier | 4Project | 121.20 |
| 13 | 4× Samsung 35E (2 pairs) | Sollan | 196.00 |
| 14 | 3×18650 holder | Hackstore | 16.00 |
| 15 | 3S 40A BMS | Hackstore | 45.00 |
| 16 | 12.6V 2A pack charger | Hackstore | 80.00 |
| 17 | Pololu D42V55F5 5V/5.5A regulator | Hackstore | 170.00 |
| 18 | Fuse holder + 10A fuse | Hackstore | 25.00 |
| 19 | DPST rocker switch | Hackstore | 25.00 |
| 20 | 2× XT60 pairs | 4Project | 19.80 |
| 21 | INA219 module | Hackstore | 40.00 |
| 22 | US-100 ultrasonic | Hackstore | 40.00 |
| 23 | Pololu VL53L1X | 4Project | 107.80 |
| 24 | Breadboard 830 | 4Project | 10.90 |
| 25 | Jumper wire sets | Piitel | 39.00 |
| 26 | M2.5 standoffs/screws | (est.) | 30.00 |
| 27 | Wire, terminals, heat shrink, bulk caps | (est.) | 60.00 |
| | **Total parts (Pi 5 4GB)** | | **₪2,069.70 ≈ $682** |
| | Shipping, 5 stores (not verified, about ₪25–40 each) [E] | | ≈ ₪150 |
| | **All-in (4GB)** | | **≈ ₪2,220 ≈ $730** |
| | **All-in with Pi 5 8GB** (+₪300) | | **≈ ₪2,520 ≈ $830** |

**Budget variant (≈ ₪1,595 parts ≈ $526, plus shipping):**

| Change | Saving |
|---|---|
| Pololu TB6612FNG ₪30.30 instead of 2× DRV8874 | −₪90.90 |
| Hackstore UBEC 5A ₪75 instead of D42V55F5 | −₪95 |
| Drop VL53L1X to Stage 2 | −₪107.80 |
| Skip the 27W PSU | −₪70 |
| 3× 4Project LG HJ2 cells ₪119.70 instead of 4× 35E | −₪76.30 |
| Resistor divider instead of INA219 | −₪35 |

Raspberry Pi 5 pricing in Israel (₪500 for 4GB, ₪800 for 8GB) is the single biggest line. Ordering the Pi and small modules from abroad would cut cost only if each order stays under the $75 VAT line, and delivery is slow.

---

## (e) Tools BOM

| Tool | Recommended | ₪ | Alternative |
|---|---|---|---|
| Multimeter | **UNI-T UT33A+**: Hackstore [V] https://hackstore.co.il/product/רב-מודד-דיגיטאלי-ut33a/ | 250 | DT-9205M Hackstore ₪105 [V]. 4Project digital multimeter ₪116.37 sale [V]. KSP Omega ₪38 [V] (toy-grade). |
| Soldering iron | **908S 80W adjustable pencil iron (230 V)**: Hackstore [S: search listing ₪120 in stock; product URL returned 404 when fetched] | 120 | **Pinecil V2** $25.99 at pine64.com [V], plus a USB-C PD ≥45W charger (not priced). 4Project Bakon SBK936B 65W station ₪399.90 [V]. Avoid 30 W unregulated irons (KSP Bolt ₪45, 4Project ₪43.40). |
| Solder | **63/37 0.8 mm 100 g**: Hackstore [V] | 65 | 4Project leaded 0.8 mm 100 g ₪156.90 [V] |
| Flux pen or paste | — (not checked) | ~30 [E] | |
| Wire strippers | — (not found at KSP, Hackstore or 4Project in quick searches) | ~60 [E] | Any hardware store; self-adjusting type recommended |
| Flush cutters | — | ~30 [E] | |
| Heat shrink | Hackstore 1.5 / 3 / 5 mm | 24 | |
| Helping hands | **2-arm workstation with iron stand**: Hackstore [V] | 85 | Hackstore 2-arm clamp ₪45 (out of stock) [V] |
| Precision screwdrivers | **Arctic Precision Screwdriver Toolkit**: KSP [V] https://ksp.co.il/web/item/398521 | 59 | Bolt SE101 10-pc ₪89 [V] |
| Hot glue gun | **Hunter 78W**: KSP [V] https://ksp.co.il/web/item/207722 | 35 | |
| LiPo / Li-ion safety bag | **C-Hobby 20×10 cm**: [V, structured-data price] https://c-hobby.co.il/en/product/lipo-safe-bag/ | 35 | Skyline and Amazon bags [S] |
| Tip cleaner (brass wool) | Hackstore [V] | 15 | |
| **Tools total** | | **≈ ₪808 ≈ $266** | Budget (DT-9205M, skip helping hands) ≈ ₪578 |

---

## Ready-made kit alternatives

| Kit | Price / availability (2026-09-16) | Pros | Cons |
|---|---|---|---|
| **Waveshare WAVE ROVER** (4WD metal, ESP32 driver board, 3S 18650 UPS) | $81–112 at resellers [S] (sunsky-online, RobotShop, PiShop). **Not found at Piitel** [V]. | Metal body. ESP32 lower controller with JSON serial commands for Pi or Jetson. Built-in 3S UPS (cells not included; buy locally). Documented ROS path. | [V] The wiki says **N20 reduction motors, 12 V 200 RPM**. I found **no wheel encoders** in the spec (verify), so odometry would come from the IMU or open-loop commands only. Skid-steer 4WD is worse for odometry than 2WD with a caster. Import VAT applies above $75. |
| **Waveshare UGV01** (tracked) | $169.99 at waveshare.com [V] | Rugged, ESP32 slave, expandable | Tracks slip, which makes odometry poor. Over $75, so 18% VAT. |
| **Waveshare UGV Rover** (6-wheel, ROS 2) | Piitel ₪2,200, **out of stock** [V] https://piitel.co.il/shop/waveshare-ugv-rover-ros-2-6-wheel-4wd-ai-robot-kit/ | Full ROS 2 platform, pan-tilt | Expensive. Jetson/Pi not included. Out of stock. |
| **Yahboom MicroROS-Pi5** | Piitel ₪1,416, **out of stock** [V] https://piitel.co.il/shop/microros‑pi5-ros2-robot-car-kit-for-raspberry-pi-5/ | ESP32 micro-ROS board, **4 encoder motors**, **MS200 LiDAR**, 2 MP camera gimbal, 7.4 V battery, ROS 2 Humble course material | Out of stock. Whether the Pi 5 is included is unverified. Mecanum/4WD odometry. You learn less about wiring and power. |
| **Pololu Romi** chassis + encoder pair | $39.95 + $9.95 at pololu.com [V]. Not found in 4Project search [V]. | Excellent 2WD differential geometry. 12 CPR encoders on 120:1 motors (about 1,440 counts per wheel revolution) [E]. Under $75 means VAT-free if shipping is cheap. | 6× AA (NiMH) battery holder. Plastic gear motors. Needs a Romi 32U4 board or your own driver. Small deck for a Pi 5 + LiDAR. Pololu shipping cost to Israel not checked. |
| Hiwonder, DFRobot kits | Not researched this session | — | — |

**Recommendation:**
- Build the **custom 2WD** from the BOM. It teaches power, encoders and motor control, and it's the right geometry for ROS 2 differential-drive odometry.
- If the student wants a pre-built base instead, the **Yahboom MicroROS-Pi5** is the closest Israel-sourced option, once it's back in stock at Piitel.
- The **WAVE ROVER** is cheaper but likely lacks wheel encoders.

---

## (f) What I could not verify, or where the uncertainty is

1. **VAT exemption threshold:** $75 as of 2026-06-02 per Kol-Zchut and ICL. It has changed three times in 10 months and is politically contested, so re-check before ordering. Sources also disagree on whether shipping counts toward the $75.
2. **DigiKey IL free-shipping threshold** (₪800 per a 2012-era snippet) and Incoterms (CPT vs DDP). **Mouser IL ₪400** threshold (snippet only; site blocked).
3. **AliExpress lithium-battery ban to Israel:** the evidence is a June 2024 article. Current behaviour is unverified; assume batteries won't arrive.
4. **Amazon $49 free shipping:** from an April 2026 article. Amazon toggles this, so check at checkout.
5. **Piitel's Yahboom 520 motor:** which gear ratio ships for ₪65 (the page shows no variant selector), and whether the PH2.0 cable is included. Ask Piitel.
6. **Yahboom 520 fit:** that it fits the Hackstore JGB37-520 bracket hole pattern [E].
7. **Chassis plate:** the Piitel 2WD chassis ₪50 price came from a search listing; the product page didn't show it. The TT plates need drilling for 37 mm motors.
8. **Hackstore 908S iron:** in stock at ₪120 per the search listing; the product URL returned 404.
9. **Current ratings** of the Hackstore rocker switches and fuse holder aren't on the listings.
10. **Samsung 35E rating:** Sollan says 20 A; the datasheet figure (about 8 A continuous) is from memory. The LG HJ2 discharge rating at 4Project wasn't checked.
11. **Shipping costs and free-shipping thresholds** for Piitel, 4Project, Hackstore, Sollan and KSP (my ₪150 total is an estimate).
12. **Stores I couldn't find:** Kitsat, Elkotec, Nisko, Chip Center, Anodit. Dash Electronics exists per Pololu, but I couldn't reach its site.
13. **Waveshare WAVE ROVER:** its exact price on waveshare.com (JavaScript-rendered), and whether it has wheel encoders.
14. **Cytron MDD3A:** no Israeli stockist found. The RobotShop or Cytron shipping cost to Israel wasn't checked.
15. **Pi 5 power details:** the `usb_max_current_enable` and `PSU_MAX_CURRENT` behaviour is summarised from Raspberry Pi docs and forum search results. raspberrypi.com docs blocked my direct fetch.
16. **Datasheet figures from memory [E]:** L298N voltage drop, DRV8833 10.8 V limit, TB6612 ratings, US-100 3.3 V operation.
17. **Prices and stock** are a 2026-09-16 snapshot. 4Project and Piitel stock changes daily.
