# Importing hardware to Israel

Back to [HARDWARE.md](../HARDWARE.md) · [suppliers-israel.md](suppliers-israel.md)

> [!IMPORTANT]
> **Version-sensitive, dated 2026-09-16.** Israel's personal-import tax threshold changed
> **three times in ten months** and is politically contested. Courier service to Israel was
> suspended as recently as March 2026. **Re-check everything on this page before you order.**
> Evidence: `references/research/israel-hardware-stage1-2026-09.md` §(b),
> `israel-hardware-stages2-5-2026-09.md` §0. This is practical guidance, not legal or tax advice.

## 1. The VAT-free threshold for personal imports

**Current rule (since 2 June 2026):**

| Goods value per shipment | Tax |
|---|---|
| Up to **$75** | No tax |
| $75 – $500 | **18% VAT** |
| Over $500 | 18% VAT, and customs duty and purchase tax **may** also apply |

**Recent history:**

| Date | Change |
|---|---|
| 26 Nov 2025 | Finance Ministry announced raising the threshold from $75 to $150 (amendment published 23 Dec 2025) |
| Early 2026 | The Knesset revoked the $150 order |
| 25 Feb 2026 | The minister signed a **$130** order, effective that day |
| 1 June 2026 | The Knesset revoked the $130 order (59–23) |
| **2 June 2026** | **Back to $75** |

One secondary source (VATupdate) summarizes this period as "$150 from Dec 2025 to Jun 2026".
Kol-Zchut gives $130 for 25.02.2026–01.06.2026. Either way, **$75 applies now**.

Sources:
- Kol-Zchut (Hebrew): https://www.kolzchut.org.il/he/%D7%96%D7%9B%D7%95%D7%AA%D7%95%D7%9F_%D7%91%D7%A0%D7%95%D7%A9%D7%90_%D7%99%D7%91%D7%95%D7%90_%D7%90%D7%99%D7%A9%D7%99_%28%D7%97%D7%91%D7%99%D7%9C%D7%95%D7%AA_%D7%9E%D7%97%D7%95%22%D7%9C%29
- ICL Global, 2 June 2026: https://www.iclglobal.com/news_update/vat-exemption-threshold-for-personal-imports-to-israel-updated/
- VATupdate, 10 June 2026: https://www.vatupdate.com/2026/06/10/israel-restores-75-vat-exemption-threshold-for-personal-imports/
- Times of Israel liveblog (snippet): https://www.timesofisrael.com/liveblog_entry/knesset-revokes-smotrichs-order-expanding-personal-import-tax-exemption-to-130/
- Ministry of Finance press release, Nov 2025 (blocked when fetched): https://www.gov.il/en/pages/press_26112025

**Does shipping count toward $75? Sources disagree.** One reading (the Dec 2025 amendment wording,
AliExpress-Israel guides) says the threshold is measured on the goods alone when shipping is
listed separately, but once tax is due, VAT is charged on goods + shipping + insurance. Another
source (ShemeshPhone, May 2026) says the threshold includes shipping.

**Practical rules:**
1. Keep each order's goods value **under $75** when you can (for example a $69 RPLIDAR C1 alone).
2. **Don't split one order into several** to dodge tax.
3. For expensive items (depth camera, arm kit), budget **+18%**, and keep orders **under $500** where that's natural (different stores, different weeks).
4. Compare with the Israeli price **including** VAT and shipping. Local stock also means an easier warranty.

## 2. Shipping realities by vendor

| Vendor | What to expect (research as of 2026-09-16) | Source |
|---|---|---|
| **AliExpress** | Standard delivery ≈ 15–30 days, express 5–10 days (low-quality source). **Items containing lithium batteries often fail to reach Israel**: the seller ships, but AliExpress's hub returns the parcel. The evidence is a **June 2024** article; the current state is unverified. **Assume cells, packs and battery-containing kits won't arrive.** | https://wowcoupons.co.il/guides/shipping-times-israel/ · https://zuzu.deals/262548/aliexpress-failed-shipping/ |
| **Amazon.com** | Free shipping to Israel restored for eligible orders **over $49** (not every item qualifies); delivery about **1 month** instead of 2 weeks. Amazon toggles this, so check at checkout. | ynetnews, 2026-04-17: https://www.ynetnews.com/business/article/s1r6pkytwx |
| **DigiKey** (digikey.co.il) | Billed in **₪**: 7–10 business days, **DigiKey handles customs clearance**. Billed in **USD**: reaches Israeli customs in ≈ 3 days, and **you** release it. Free-shipping threshold: a snippet says ₪800, but it traces to a 2012 press release (unverified). | https://www.digikey.co.il/en/help-support/delivery-information/delivery-time-and-cost |
| **Mouser** (mouser.co.il) | "Free shipping on most orders over ₪400" (snippet only; site blocked fetching) | https://www.mouser.co.il/ |
| **Pololu** (direct) | FedEx International Connect Plus or Priority. Usually better to buy Pololu parts at **4Project** (official distributor). | https://www.pololu.com/distributors/0J118 |
| **RobotShop** | Ships internationally; **duties, taxes and fees are paid by the buyer on delivery** | https://www.robotshop.com/policies/shipping-policy |
| **Waveshare** (direct) | Worldwide shipping; "all additional local taxes, duties… are customers' responsibilities" | https://www.waveshare.com/shipping |
| **Seeed Studio** | Posted on **2026-03-02** that DHL/FedEx/UPS shipments to Israel were **suspended** (Middle East airspace closure). Status for Sept 2026 is unconfirmed: **email order@seeed.io before ordering**. | https://www.cep-research.com/2026/03/03/middle-east-conflict-continues-to-disrupt-global-logistics/ |
| **Couriers in general** | FedEx said on 2026-04-09 that Middle East service was back to normal. A FreightWaves report at the time said UPS flights to Israel were still suspended while FedEx and DHL operated. | https://www.freightwaves.com/news/ups-flights-to-israel-still-suspended-fedex-and-dhl-operate |
| WowRobo, PartaBot, Luxonis, RealSense, DFRobot | **Shipping to Israel not verified.** WowRobo's shipping-policy URL returned 404. Ask before paying. | — |

## 3. Batteries: always buy locally

- Buy **lithium cells and packs in Israel**: Sollan, 4Project, Hackstore, RC shops ([suppliers-israel.md](suppliers-israel.md)).
- Chargers, BMS boards, battery holders and cell-less kits **can** be imported normally.
- A kit that bundles a battery (for example some LeKiwi or robot-car kits) may be returned in transit. Choose the battery-less version.
- Safety: [SAFETY.md](../SAFETY.md) rule 5.

## 4. Mains power, plugs and chargers

| Topic | Guidance |
|---|---|
| Mains | Israel is **230 V, 50 Hz**. Sockets are **Type H**, which also accept 2-pin **Type C Europlugs**. |
| Imported PSUs and chargers | Anything rated **100–240 V** is fine with a plug adapter (check the label). **Avoid 110 V-only US tools** (some soldering stations and glue guns). Israeli retail tools carry Israeli or EU plugs. |
| USB-C PD | The Raspberry Pi 5 needs its own 5 V 5 A profile (official 27 W PSU); generic PD chargers make it limit USB current. A **Pinecil** iron needs a PD charger of **≥ 45 W**. |
| Mains work | Never open or modify a mains-powered charger or PSU ([SAFETY.md](../SAFETY.md)). |

## 5. Warranty and returns

This wasn't researched store by store, so treat these as general habits:

- **Prefer Israeli official resellers and importers** for expensive items (Pi 5 at Piitel, Bambu Lab at 3DbotX, which warns that grey-import units may be region-locked). A return abroad costs international shipping, and sometimes you pay the VAT again.
- Read each Israeli store's terms of sale (תקנון) for cancellation and return rules before ordering. They weren't checked for these stores.
- Photograph parts on arrival. Test motors, sensors and boards **within the first week**: early failures are the easy warranty claims.
- For AliExpress, open a dispute before the buyer-protection timer runs out.

## 6. Customs tips

- Keep invoices. Customs uses the **declared value**, so check that it matches what you paid.
- With **DigiKey in ₪**, clearance is handled for you. With USD orders, or couriers that hand you
  the clearance, expect paperwork. Couriers may charge a handling or clearance fee (amount not checked).
- Batch small parts into **one local order** (Piitel, 4Project, Hackstore) instead of many small imports; the shipping savings are often larger than the price gap.
- Check the **courier** before ordering from Seeed, WowRobo or any vendor that ships by UPS.
- Things that change: the $75 threshold, Amazon free shipping, courier status, AliExpress lithium
  policy. Re-verify all four before a big order.
