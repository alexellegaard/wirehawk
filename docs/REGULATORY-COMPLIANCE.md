# Wirehawk — Danish & EU Regulatory Map

> Scope: a **permanently installed, camera-guided 4-cable suspended CDPR** ("wire gantry") that moves a mass fast across a rooftop to scare birds away **without contact**. Developed by a Danish company to be **sold to customers** as a product.
>
> This document maps the rules that apply to (a) the machine as a *product* (CE marking), (b) the *installation* on a customer's roof, and (c) the *operation* of it (bird protection, data, worker safety). It is a map, not legal advice — the two load-bearing unknowns that need a lawyer/notified-body once you're close to market are flagged inline.
>
> Authoritative Danish sources cited inline; full list at the bottom. Last updated 23 Oct 2026.

---

## 0. What Wirehawk is, in regulatory terms

Wirehawk is **machinery**, not a drone. It is an assembly of linked parts, at least one of which moves, fitted with a drive system other than human/animal effort, joined together for a specific application — the definition in Article 3 of the Machinery Regulation[1]. The fact that it is a cable-suspended mass driven by four roof-corner winches (not an airborne vehicle) means the **drone/UAS regime (EASA, EU 2019/947, Danish Trafikstyrelsen rules) does not apply** to the machine itself — it is a fixed installation anchored to a building. This is the single most important framing: the product falls under **machinery law + electrical law + radio law**, and the installation falls under **building law**, while the *use* raises **wildlife-protection** and **data-protection** questions.

The system also contains: electrical/electronic equipment (drives, PSU, Raspberry Pi) and a **radio** (Wi-Fi) — so it is simultaneously "machinery", "electrical equipment" and "radio equipment" for CE-marking purposes[9].

---

## 1. Product law — CE marking (the core obligation)

### 1.1 The timing problem: you will ship under the *new* Machinery Regulation, not the Directive

EU machinery law is changing exactly in your delivery window:

- **Machinery Directive 2006/42/EC** applies to machinery placed on the market **up to and including 19 January 2027**[1].
- **Machinery Regulation (EU) 2023/1230** replaces it and applies from **20 January 2027**; the Directive is repealed the same day[1][2]. (A corrigendum moved the date from 14 Jan to 20 Jan 2027[1].)

Your full-scale demo is targeted for **~1 March 2027** — *after* the cut-over. So whatever you put on a customer's roof from 20 Jan 2027 onward must be assessed against the **Regulation**, not the Directive. That matters concretely because the Regulation introduces new essential requirements the Directive lacks — notably **cybersecurity** (protection of control systems against corruption and reasonably foreseeable malicious attempts) and **self-evolving/ML behaviour** (see §1.4)[2].

Machinery placed on the market under the old Directive before 20 Jan 2027 may continue to be made available afterward (no forced re-certification of stock)[1][2]. But there is no sell-off period for newly built machines: each unit is judged by the law in force the day it is first placed on the market or put into service[1].

### 1.2 Is Wirehawk "machinery" (or a "related product")?

Yes. A motor-driven CDPR is squarely within the machinery definition[1]. Two secondary categories to keep in mind:

- **Partly completed machinery** — if you ever ship the winch/mast assembly *without* the control software for the integrator to complete. You cannot CE-mark partly completed machinery; you issue a **declaration of incorporation** instead and the integrator does the final assessment[4].
- **"Related products" / safety components** — if you were to sell a *standalone* safety component (e.g. a certified safety controller) it has its own path[6].

For the product as you intend to sell it (a complete, working deterrence system), you are the **manufacturer** and you CE-mark the whole assembly[3][4].

### 1.3 The Danish enforcement layer

Denmark does not have a separate national "machine approval" — the EU machinery rules are implemented via the Danish **bekendtgørelse om maskiner** (machinery order)[4], and market surveillance (checking that machinery sold in Denmark is correctly CE-marked and documented) is the job of **Sikkerhedsstyrelsen** (the Danish Safety Technology Authority), which has held that role for machinery since **1 December 2018**[3]. Sikkerhedsstyrelsen is your Danish point of contact for CE questions and the authority that can pull a non-compliant machine off the market[3][4].

### 1.4 The AI/ML question — the single most consequential decision in this map

Wirehawk's bird detection is a **machine-learning** model (YOLOv8 on a Hailo accelerator), and its output drives the machine's motion. This is exactly the territory the new Machinery Regulation targets.

Under **Annex I, Part A** of the Regulation, two categories **lose the right to self-declare** and **must go through a Notified Body** (EU type-examination, full QA, or unit verification — no self-certification route)[6][7]:

> 5. Safety components with fully or partially self-evolving behaviour using machine learning approaches **ensuring safety functions**.
> 6. Machinery that has embedded systems with fully or partially self-evolving behaviour using machine learning approaches **ensuring safety functions** that have not been placed independently on the market.[6][7]

The operative phrase is **"ensuring safety functions."** The Regulation's Recital 55 narrows the Notified-Body track to systems that *learn through machine learning* **while ensuring a safety function**; rule-based software that executes fixed automated functions is excluded[6][7]. DTI (the Danish Technological Institute, whose certification body Dancert is Denmark's Notified Body for this) works with a deliberately broad definition of "self-evolving behaviour" — software/algorithms using machine learning and generalising from data, whether the training happened before or during operation[5]. Pre-trained, frozen models are treated as "self-evolving" on a capability-based reading (they *could* be retrained)[7].

**What this means for Wirehawk — the design rule:**

- If your **ML (bird detection) is kept out of the safety chain** — i.e. it only ever produces *targets*, and every safety function (E-stop, soft workspace limits, tension/force limits, height clamp, watchdog) is **deterministic, rule-based code** — then the ML is **not** "ensuring a safety function," and Wirehawk does **not** fall into Annex I Part A items 5/6. You keep the right to self-declare (module A) and no Notified Body is mandatory[6][7].
- If the ML ever participates in a safety function — e.g. an AI-based "person on roof" detector that triggers a stop, or ML-based tension prediction used as a protective limit — then it **flips into Annex I Part A** and a **Notified Body is mandatory** (module B+C, H, or G), with the body's number on the CE plate[6][7].

This is already Wirehawk's architecture. The handover doc states the principle verbatim: "Vision kommandere targets → safety verificerer, begrænser og har sidste ord" (*vision commands targets → safety verifies, limits, and has the last word*), and "Safety-laget må aldrig kunne omgås af vision." That separation is not just good engineering — it is the difference between a self-declared CE marking and a mandatory, costly Notified-Body type-examination. **Keep the ML out of the safety chain, and document that boundary explicitly in the risk assessment.**

Two caveats that apply **even if you stay out of Part A**:

1. **Annex III, 1.2.1's self-evolving-logic block applies without the "safety function" qualifier.** Any control system with "fully or partially self-evolving behaviour or logic … designed to operate with varying levels of autonomy" must (a) not cause actions beyond its defined task and movement space, (b) enable recording of the safety-related decision-making process (retained **1 year**), and (c) be correctable at all times to maintain safety[7]. So even a module-A machine with ML-driven autonomy owes these three things.
2. **Annex III generally** requires you to assess, at the design stage, the *intended evolution* of self-evolving behaviour and where it must stop[6].

**Action:** decide and write down, now, where the ML/safety boundary sits, and put it in the technical file. If you are ever tempted to add an ML-based safety feature (person detection, ML tension monitoring), expect to budget for a Notified Body.

### 1.5 Electrical and radio law riding alongside the machinery assessment

A machine containing electronics must also satisfy, in parallel:

- **Low Voltage Directive 2014/35/EU (LVD)** — for the non-radio electrical parts (drives, PSU, wiring) in the 50–1000 V AC range.
- **EMC Directive 2014/30/EU** — electromagnetic compatibility of the whole assembly.
- **Radio Equipment Directive 2014/53/EU (RED)** — for the Wi-Fi radio. RED is special: for *radio equipment* it *incorporates* the safety objectives of LVD and the protection requirements of EMC, so **LVD and EMC do not apply separately** to the radio part[9]. The Raspberry Pi with its Wi-Fi module is radio equipment under RED; the module you integrate should already carry a RED declaration, but **you** as manufacturer of the finished product still carry the RED (and machinery) obligations for the assembly[9].

Practical implication: the CE technical file for Wirehawk will reference **Machinery Regulation + LVD + EMC + RED** (and RoHS, §1.6), and the declaration of conformity lists all of them.

### 1.6 RoHS / WEEE — the electronics you buy become the electronics you sell

RoHS 2 (2011/65/EU) restricts lead, mercury, cadmium and other substances in electrical/electronic equipment, and the CE marking is an overt claim of RoHS compliance[12][13]. Machinery is in scope wherever it contains covered electrical/electronic equipment (drives, boards, cabling)[13]. WEEE (2012/19/EU) adds end-of-life take-back/recycling obligations for that electronic waste — worth one line in your agreement with customers about who handles decommissioning.

### 1.7 What the CE path actually produces (the deliverable checklist)

To CE-mark Wirehawk you will need, in order: a **risk assessment** (hazard identification across the machine's lifecycle), a **technical file** (design drawings, calculations, standards applied, test results, the ML-boundary documentation from §1.4), a **declaration of conformity** (or declaration of incorporation if partly completed), the **CE marking** on the machine, and a **user/installation manual** in the language of the destination country (Danish for Denmark)[4]. The Regulation also permits digital instructions under conditions[1][2]. The conformity assessment procedure is **module A (internal production control)** — unless §1.4 pushes you into a Notified Body[7].

---

## 2. Wildlife / bird-protection law (Danish + EU)

The Wirehawk method — **sudden movement, no contact, no projectiles, no sound** — sits on the permissive side of Danish wildlife law, but there are three rules you must respect.

### 2.1 Deterrence is lawful; "regulation" is not (without a permit)

Danish practice distinguishes **deterrence** (*afværgelse*/scaring) from **regulation** (*regulering*: killing, egg-oiling, nest removal during breeding). **Most scaring methods require no permit; only sound-based scaring *may* require one**[16]. Because Wirehawk uses **movement only** (no acoustic deterrent), it falls under the no-permit category[16].

### 2.2 Nests and eggs are absolutely protected

Under the Danish hunting & wildlife management act (*Jagt- og vildtforvaltningsloven* § 6 a, stk. 2): **"Fugles reder må ikke forsætligt ødelægges, beskadiges eller fjernes. Æg må ikke forsætligt ødelægges eller beskadiges"** — nests may not be deliberately destroyed, damaged or removed; eggs may not be deliberately destroyed or damaged. This applies **everywhere**, including rooftops and buildings, and to **all** bird nests[14]. Practical rule: the moving mass must never be able to strike a nest, and you should not run nest-destroying behaviour. (This also has a *design* consequence: the motion planner should treat the roof surface, edges and any structures as obstacles, which it already does for skylights/vents.)

### 2.3 Disturbance is only prohibited when it harms the species/population

The EU Birds Directive (2009/147/EC), implemented in Denmark via the *Artsfredningsbekendtgørelsen* and *Naturbeskyttelsesloven*, prohibits **deliberately** disturbing birds *in a way that has a harmful effect on the species or the population*[15][17]. Scaring individual seagulls off a single roof does not reach that threshold. The residual caution is the **breeding season**: the general rule is to avoid disturbing nesting birds while the nest is in use, and to remove old nest material only **before** the gulls begin building new nests in spring (the municipalities' guidance points to removal before ~1 April, when the season begins)[16]. Wirehawk's non-contact, non-lethal scaring is compatible with this, but the *timing* of installation/de-commissioning around an active nest is the one point to keep clean.

### 2.4 Net effect

**No permit is expected for operating Wirehawk as designed** (non-contact movement scaring, no sound, no nest interaction)[16]. The two obligations are: (a) never destroy/disturb nests or eggs[14], and (b) treat any future addition of an acoustic deterrent as a potential permit trigger[16]. Document the deterrence-vs-regulation distinction in your product material so customers understand they are buying deterrence, not pest control.

---

## 3. Building / installation law (per customer roof)

The masts, anchors and cable loads are attached to the customer's **building**, which brings it under the Danish **building regulation (BR18)**.

- Building work must not be taken into use without the municipality's **ibrugtagningstilladelse** (permission to put into use) where a permit is required[18][19].
- For building work in **construction class 2–4**, a **certified structural engineer (certificeret statiker)** must be attached to the project to document/verify the load-bearing construction (BR18 chapter 15)[18].
- The loads here are non-trivial: four corner masts each carrying dynamic cable tension, wind load on the mast and on the moving mass, and snow/ice. That is exactly the "bærende konstruktion" documentation BR18 wants[18].

**Action:** treat the roof attachment as building work in your customer offer — per-installation you (or the customer, agreed up front) obtain the municipal permit and a structural calculation (wind load per Eurocodes, anchor pull-out, mast moment). This is also where the **70×70 m / 60×60 m** scale and **~5 kg payload at 5 m/s** numbers feed a real structural spec. Whether a *given* installation needs a full permit depends on the specific municipality and the size of the structure — flag it as a per-site question, not a fixed answer.

---

## 4. Electrical installation law

The **fixed 230 V supply** to the roof winches/enclosure is a **fixed electrical installation**, which in Denmark:

- Must be carried out by an **authorized electrical contractor (autoriseret elinstallatør)** — the *elsikkerhedsloven* + *autorisationsloven* require authorization for fixed installation work, including control/regulation systems driving power functions[20][21].
- Must follow the Danish low-voltage installation standard series **HD 60364** (compliance with which is deemed to meet the safety requirement)[20].
- Outdoor circuits need **RCD (HPFI) protection ≤ 30 mA** at connection points[20].

**Action:** the 230 V roof feed, the enclosure and the protection devices are an authorized-electrician job, done to HD 60364. Your own engineers can do the low-voltage/signal side (EtherCAT, 24 V), but the mains installation is out of scope for them unless you hold (or subcontract) the authorization. Build this into the installation budget and the customer contract.

---

## 5. Work at height — Arbejdstilsynet (installation & maintenance)

This applies to **your own technicians** installing and servicing masts on rooftops — an operational/workplace obligation, not a product one, but a hard one.

Arbejdstilsynet's rules for work at height: do a **risk assessment** (height, duration, surface, weather, tools); prefer **collective protection** (guardrails, scaffolding) over individual fall protection; use individual fall-arrest equipment only where collective measures are not reasonably possible; and ensure people are **trained** in the equipment and in an emergency plan[22]. Rooftop work on an exposed, possibly sloped or slippery surface with wind is squarely the scenario AT flags as high-risk[22].

**Action:** before the first roof install, write the fall-protection method statement and train the install crew. This is also an insurance matter (§7).

---

## 6. Data protection & CCTV (GDPR + TV-overvågningsloven)

The camera on the roof records images continuously to detect birds. If those images can identify **people** — roof workers, maintenance staff, or neighbours visible over the parapet — then:

- **GDPR** (Databeskyttelsesforordningen + Databeskyttelsesloven) governs the processing of those personal data[23].
- **TV-overvågningsloven** (the Danish CCTV act) applies where the recording covers places of public access or workplaces, and requires **signage** (skiltning) informing people of the surveillance[23].
- CCTV recordings made for crime-prevention purposes must be **deleted within 30 days** as a main rule[23].

The cleanest mitigation — and it is a *design* decision — is **privacy by design**: run detection **on-device** (the Hailo accelerator), and **do not retain raw video**; store only detections/metadata (bounding box, timestamp) or nothing. If no identifiable imagery is ever stored, the GDPR/CCTV exposure shrinks to near zero, and you can honestly document "no personal data retained." If you *do* want to keep footage (e.g. to prove deterrence efficacy), then the 30-day deletion, signage and a lawful-basis analysis all kick in[23].

---

## 7. Product liability & insurance

**Produktansvarsloven** (implementing the EU Product Liability Directive) imposes **strict (objective) liability** on the producer for **personal injury** caused by a **defective** product — the injured party does not have to prove negligence[24]. A moving 5 kg mass at 5 m/s that could fall on someone, or a mast that could fail, is a real liability surface.

Two points worth internalising:

- The producer has a **development-risk defence**: not liable if the defect was not discoverable given scientific/technical knowledge at the time the product was placed on the market (§ 7, nr. 4)[24]. That defence is only as good as the technical file documenting that you applied the state of the art.
- **Product-liability insurance** is a prerequisite for selling this, and the CE technical file + risk assessment from §1.7 is exactly what an insurer will want to see before quoting.

**Action:** pair the CE file with a product-liability insurance policy before the first commercial unit ships; treat the risk assessment as an insurance document, not just a regulatory form.

---

## 8. Cybersecurity — RED delegated act now, Cyber Resilience Act later

If the Pi's Wi-Fi is **internet-connected** (telemetry, remote access, OTA updates), then the **RED Delegated Regulation (EU) 2022/30** has applied since **1 August 2025** to "internet-connected radio equipment," activating the RED essential requirements on network protection, personal-data protection and fraud protection (Art. 3(3)(d)–(f))[10][11]. In practice this means the device's network interfaces must not allow straightforward compromise, weak default passwords, etc. (the harmonised standard is EN 18031)[10].

Timeline note: the Commission has adopted a repeal of the RED delegated act effective **11 December 2027**, when the horizontal **Cyber Resilience Act (CRA, EU 2024/2847)** takes over cybersecurity for all digital products[10]. So you will touch this twice: RED-cybersecurity now, CRA for units placed after late 2027.

The new Machinery Regulation separately demands, in Annex III §1.1.9/§1.2.1, that control logic withstand "reasonably foreseeable malicious attempts" against communication corruption[2]. That is a machinery-file requirement regardless of the radio angle.

---

## 9. AI Act interaction (brief, because it defers to machinery law)

The **AI Act (EU 2024/1689)** would classify Wirehawk's vision system as **high-risk** only if it is (a) intended as a **safety component** and (b) the product must undergo third-party conformity assessment[8]. By the 2026 "Digital Omnibus" (Regulation 2026/1744), machinery was moved to **Annex I, Section B** of the AI Act, so the high-risk AI requirements get **folded into the Machinery Regulation** via delegated acts due by **2 August 2028** rather than applied as a parallel regime[7]. Net: for a machine like Wirehawk, the Machinery Regulation is the single lens for AI-related safety requirements; the AI Act only bites if §1.4 pushes you into the Notified-Body/self-evolving-safety-function territory[7][8].

---

## 10. Priority actions (what to do first)

1. **Write down the ML/safety boundary** (§1.4). This one decision determines whether CE is self-declared or requires a Notified Body. Wirehawk's existing "safety has the last word" architecture already points the right way — codify it in the risk assessment.
2. **Plan for the Machinery Regulation, not the Directive** (§1.1). Your 1 March 2027 demo lands after the 20 Jan 2027 cut-over. Build the technical file against 2023/1230 from day one.
3. **Lock the electrical path** (§4): authorized electrician + HD 60364 for the 230 V roof feed, priced into the installation.
4. **Lock the building path** (§3): municipal permit + certified structural engineer (wind load, anchoring) per customer roof — make it a named line in the offer.
5. **Design for privacy by default** (§6): on-device detection, no raw-video retention — decide this before the camera integration is frozen.
6. **CE file as an insurance document** (§7): risk assessment + technical file + DoC + product-liability insurance before first commercial unit.
7. **Wildlife = deterrence, no sound, no nest contact** (§2): document it in the user manual; revisit only if an acoustic deterrent is ever added.
8. **Cybersecurity now and later** (§8): RED 2022/30 for the internet-connected radio today, CRA from late 2027.

## Open questions (for a lawyer / Notified Body when you're near market)

- The precise **"safety function"** characterisation of the ML vision in Wirehawk's specific architecture (borderline judgment call — get a written opinion from DTI/Dancert or a machinery-compliance consultant).
- Whether a **rooftop mast/winch installation** triggers a full building permit in the target municipalities (per-site, not general).
- Whether the **demo unit on a customer's roof in March 2027** counts as "placed on the market / put into service" and therefore must already be fully compliant (almost certainly yes — plan for it).

## Sources

[1] https://eu-manuals.eu/machinery-regulation-2023-1230 — Machinery Regulation 2023/1230: dates and transition
[2] https://cecheck.com/machinery-regulation — Machinery Regulation (EU) 2023/1230 explained
[3] https://www.sik.dk/erhverv/produkter/maskiner/vejledninger/vejledning-maskindirektivet — Sikkerhedsstyrelsen: vejledning til maskindirektivet
[4] https://www.retsinformation.dk/api/pdf/223619 — Bekendtgørelse om maskiner (Retsinformation)
[5] https://dti.dk/services/type-examination-of-machines-that-use-ai-for-safety-functions/47581 — DTI: type examination of machines using AI for safety functions
[6] https://lawandtechnology.eu/en/self-learning-machines-regulation-2023-1230-third-party-assessment — Self-learning machines under Regulation 2023/1230
[7] https://particula.tech/blog/machinery-regulation-ml-safety-component-notified-body — Does your ML need a notified body (Machinery Regulation)
[8] https://ai-act-service-desk.ec.europa.eu/en/high-risk-ai-regulated-products — AI Act Service Desk: high-risk AI in regulated products
[9] https://ib-lenhardt.com/kb/faq/radio-equipment-directive-red — Radio Equipment Directive (RED) 2014/53/EU
[10] https://single-market-economy.ec.europa.eu/sectors/electrical-and-electronic-engineering-industries-eei/radio-equipment-directive-red_en — EC: Radio Equipment Directive (RED) overview
[11] https://eur-lex.europa.eu/eli/reg_del/2022/30/2023-10-27/eng/pdf — RED Delegated Regulation (EU) 2022/30 (cybersecurity)
[12] https://environment.ec.europa.eu/topics/waste-and-recycling/rohs-directive_en — RoHS Directive 2011/65/EU
[13] https://f2labs.com/technotes/rohs-2011-65-eu-and-machinery — Does RoHS apply to machinery
[14] https://sgavmst.dk/natur-og-jagt/natur-og-biodiversitet/vis-hensyn-til-dyr-i-naturen/pas-paa-fuglenes-reder-og-levesteder — SGAVMST: pas på fuglenes reder og levesteder
[15] https://sgavmst.dk/arter/artsforvaltning/beskyttede-arter — SGAVMST: beskyttede arter
[16] https://www.vejle.dk/da/service-og-selvbetjening/borger/bolig-byggeri-og-forsyning/skadelige-dyr-og-planter/soelvmaager — Vejle Kommune: sølvmåger (deterrence vs regulation)
[17] https://www.retsinformation.dk/eli/lta/2018/1466 — Artsfredningsbekendtgørelsen
[18] https://www.retsinformation.dk/eli/lta/2019/1399 — Bygningsreglement 2018 (BR18)
[19] https://www.bygningsreglementet.dk/administrative-bestemmelser/brv/vejledning-om-byggesagsbehandling-efter-br18/7_2_ibrugtagningstilladelse/?Layout=ShowAll — BR18: ibrugtagningstilladelse vejledning
[20] https://www.retsinformation.dk/eli/lta/2016/1082 — Bekendtgørelse om sikkerhed for elektriske installationer
[21] https://www.lovguiden.dk/loven/lov-om-autorisation-af-virksomheder-p%C3%A5-el-vvs-og-kloakinstallationsomr%C3%A5det/2?stk=1 — Autorisationsloven §2
[22] https://at.dk/faa-viden/fysisk-arbejde/forebyg-ulykker/forebyg-fald-fra-hoejden — Arbejdstilsynet: forebyg fald fra højden
[23] https://www.datatilsynet.dk/regler-og-vejledning/optagelser-og-overvaagning — Datatilsynet: optagelser og overvågning
[24] https://www.retsinformation.dk/eli/accn/A19890037130 — Produktansvarsloven
