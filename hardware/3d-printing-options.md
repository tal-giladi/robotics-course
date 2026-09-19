# 3D printing options in Israel

Back to [HARDWARE.md](../HARDWARE.md). Background and the buy-or-not decision:
[F3D.01 Why 3D printing matters](../optional-foundations/3d-printing-and-cad/F3D.01-why-3d-printing.md);
ordering prints: [F3D.07](../optional-foundations/3d-printing-and-cad/F3D.07-materials-chassis-ordering.md).

> [!IMPORTANT]
> **Prices checked 2026-09-16, approximate, including VAT.** Evidence:
> `references/research/israel-hardware-stages2-5-2026-09.md` §3D printing.

**You don't need a printer to finish this course.** Brackets can be cut, bought or printed by a
service, and SO-101 kits can be bought with printed parts included.

## Services

| Service | Typical price | URL |
|---|---|---|
| 3D Print Orel | Small simple parts from ₪30–50; medium ₪100–300 | https://3dprintorel.co.il/3d-printing-price/ |
| 3D3 | Simple ₪10–20, small figure ≈ ₪30, large prototype > ₪500; modelling ₪100–400/h | https://www.3d3.co.il/post/discover-cost |
| 3D Factory (Jaffa), AMP Jerusalem, Center3D, Indus3design | Not checked | https://www.3dfactory.co.il/ · https://amp3d.co.il/ · https://www.centerd3.co.il/ · https://www.indus3design.com/3d-printing-cost/ |

A full SO-101 part set at a service is roughly **a few hundred ₪ (unverified estimate)**. Seeed's
printed set ($29.90) or a WowRobo package with parts is cheaper ([stage-5-robotic-arm.md](stage-5-robotic-arm.md)).

## Makerspaces

| Place | Details | URL |
|---|---|---|
| **MakeLab Israel, Yehud** | Free community lab: 3D printers, laser cutter, CNC, PCB. 28 Kdoshei Mitsrayim St., Sun–Thu 09:00–21:00, operator approval needed. | https://www.makelab.org.il/ |
| Makerz network | Membership makerspaces, mostly in the north and periphery; paid manufacturing service; no prices online | https://makerspace.co.il/makerspace/ |
| Impact Labs, Tel Aviv | Many printers; public access **not verified** | https://www.impactlabs.tech/?lang=en |
| FabLab IL Holon, MadaTech Haifa, Tel Aviv Makers | **Current status unverified** (only 2014-era articles) | https://www.fablabs.io/labs/fablabil |

## Printers with local prices

| Printer | ≈ ₪ (2026-09-16) | Where | Notes |
|---|---|---|---|
| **Bambu Lab A1 mini** | **₪990** (Combo with AMS lite ₪1,890) | 3DbotX (official importer): https://www.3dbotx.co.il/product-page/bambulab-a1-mini-3d-printer | 180 mm build cube; big enough for SO-101 parts and robot brackets. **The course's pick.** |
| Bambu Lab A1 mini (Zap) | ₪890 at Bug (₪755 in-store in Eilat) | https://www.zap.co.il/model.aspx?modelid=1243156 | Importer not stated. 3DbotX warns grey-import units may be region-locked, so check first. |
| Bambu Lab A1 | ₪1,550 | https://www.3dbotx.co.il/product-page/bambulab-a1-3d-printer | 256 mm cube; fits a full chassis plate |
| Creality Ender 3 V3 / V3 KE | ₪1,790 / ₪1,990 (search snippet only; the 3DbotX product page returned 404 on 2026-09-16) | 3DbotX https://www.3dbotx.co.il/product-page/bambulab-a1-3d-printer (browse the site) · Spider3D https://www.spider3d.co.il/product-tag/creality/ | More tinkering than Bambu |
| Prusa CORE One | ≈ ₪6,460 (verify) | https://shop.caliber.co.il/items/7537329-Prusa-CORE-One | Premium; not needed for this course |
| PLA filament | ₪100–200 per kg; spools from ₪69 | https://www.spider3d.co.il/product-category/%D7%A4%D7%99%D7%9C%D7%9E%D7%A0%D7%98%D7%99%D7%9D/ | PETG for parts near motors or in a hot car |

## When buying a printer becomes worthwhile

A rough rule using the prices above: an A1 mini (₪990) plus a spool (≈ ₪100) costs about the
same as **5–10 medium service jobs** (₪100–300 each). Buy one when at least one of these is true:

- You'll print the SO-101 parts yourself and expect to reprint grippers and mounts.
- You iterate on brackets: camera mount, LiDAR riser, e-stop box, cable clips (Stage 6). Each design takes 2–4 attempts.
- MakeLab Yehud or another makerspace is too far for repeated visits.

Otherwise use a service or a makerspace. Safety: printers run hot; keep them ventilated and
don't leave them printing unattended for long jobs.
