# Wirehawk — Servicemanual

> **Dato:** 2026-10-29 · **Sprog:** Dansk — engelsk udgave: `SERVICE-MANUAL.md`. Hold de to i sync. Spejler `Wirehawk Service Manual.docx` (nuværende byg, 40×40×3 m).

## 1. Systemoversigt

Wirehawk er en 4-kabel ophængt CDPR med fire 3 m høje master i tagets hjørner, som hver har en winch (motor + servodrev + spole + ankertrisse). Hver mast er monteret med et kabel til sin winch, som er ført ud til en fælles masse på 3 kg, der kan styres rundt i robottens workspace. Et visionsystem detekterer fugle, og robotten bevæger massen hurtigt hen mod dem for at skræmme dem væk.

### 1.1 Komponenter pr. mast (×4)

- Servomotor
- Servodrev
- Strømskab
- Elskab
- Multiturn enkoderkabel
- Winch

### 1.2 Styring (sidder på mast 1)

- Raspberry Pi 5 (EtherCAT-master + vision)
- Ethernet-switch
- Strømtavle
- Nødstop — TODO
- Sikringer — TODO

## 2. Sikkerhed

- Afbryd altid strømmen før rengøring eller service inde i elskabe eller nogen anden elektrisk del. Hver mast kører på 230 V enfaset, og hele systemet er forsynet med 400 V trefaset.
- Jord (PE) er obligatorisk — kør derfor aldrig systemet ujordet.
- Servomotorerne er udstyret med hvilebremse: når 24 V fjernes, er akslen låst. En spændingsløs motor har derfor ikke mulighed for friløb og kan derfor ikke drejes med hånden.
- Servodrevets eget vedligeholdskapitel advarer: afbryd strømmen før rengøring. Brug kun trykluftpistol eller tør klud — aldrig benzin, fortynder, alkohol eller sure/alkaliske rengøringsmidler (A6-manual §14.1, s.270).
- Drevet er DC-sync-only: når EtherCAT-masteren stopper, låser drevet en sync-tabsfejl (ErC1.1). Det er forventet adfærd, ikke en defekt, og den kan nulstilles med fejlreset eller genstart (A6-manual §8.2.3 s.162).
- Arbejde i højden er omfattet af regler fra Arbejdstilsynet (faldsikring, risikovurdering, uddannet mandskab osv.). Dette er uden for denne manuals omfang.

## 3. Komponentoversigt

| # | Komponent | Navn | Antal |
|---|---|---|---|
| 1 | Servomotor | A6 1000 W EtherCAT, 3000 rpm, 3,18 Nm nominelt, bremse (24 V / 3,2 Nm), 17-bit absolut encoder, IP67 | 4 |
| 2 | Servodrev | A6-EC1000H2B1-M17 | 4 |
| 3 | 24 V strømforsyning | Mean Well LRS-100-24 (24 V / 4,5 A) | 4 |
| 4 | Multiturn enkoder-kabel | AS7-C-ENC076-BAT-3.0 (3 m) | 4 |
| 5 | Ophængsline | Dynamica DM20, 2 mm, splejset, 65 m | 4 |
| 6 | Elskab | IP66 | 4 |
| 7 | Netværkskabel | CAT6 U/UTP udendørs, 50 m | 4 |
| 8 | Ankertrisse | Biltema drejeligt hjul, dobbelt, 50 mm, med egen trisse indsat | 4 |
| 9 | Spole | 3D-printet spole og spolebase | 4 |
| 10 | Nødstop | TODO | 1 |
| 11 | Sikringer | TODO | — |

## 4. Serviceintervaller

Komponenterne i systemet nedarver deres serviceintervaller fra fabrikanten. Foreløbigt er A6-motoren samt servodrevet det eneste komponent med en fuld vedligeholdelsestabel opgivet af producenten. Intervaller markeret "foreløbig" har endnu intet producentopgivet serviceinterval og er indtil videre eftersynsanbefalinger, indtil leverandørens data er tilgængeligt.

### 4.1 Servomotor

Nedarvet fra A6-EC servo-manual: https://files.omc-stepperonline.com/A6-EC_series_servo_drive_manual.pdf

| Del | Serviceinterval | Servicebeskrivelse |
|---|---|---|
| Leje | 3–5 år (20.000 til 30.000 timer) | Udskiftes |
| Olietætning | 5000 timer | Udskiftes |
| Singelturn encoder | 3–5 år (20.000 til 30.000 timer) | Udskiftes |
| Multiturn encoder | Afhænger af de specifikke driftsforhold | TODO |

### 4.2 Servodrev

Nedarvet fra A6-EC servo-manual: https://files.omc-stepperonline.com/A6-EC_series_servo_drive_manual.pdf

| Del | Serviceinterval | Servicebeskrivelse |
|---|---|---|
| Busfilterkondensator | ca. 5 år | Udskiftes |
| Køleventilator | 2–3 år (10.000 til 30.000 timer) | Udskiftes |
| Elektrolytkondensator på kredsløbskort | ca. 5 år | Udskiftes |
| Foropladningsrelæ | 100.000 operationer | Udskiftes |
| Foropladningsmodstand | 20.000 operationer | Udskiftes |

### 4.3 Foreløbigt

Resten af komponenterne. TODO

## 5. Dagligt eftersyn (drev) — A6-manual §14.1, s.270

Standarddriftsforhold, som drevet er dimensioneret til: **30 °C** årlig gennemsnitlig omgivelsestemperatur, **< 80 %** gennemsnitsbelastning, **< 20 t** daglig drift.

**Tjek dagligt:**
- Omgivelsestemperatur/-fugtighed normal; ingen støv eller fremmedlegemer i/på drevet.
- Ingen unormal vibration eller støj fra enheden.
- Forsyningsspændingen er normal.
- Ingen underlig lugt.
- Ingen fibre på luftindtaget.
- Ingen fremmedlegemer ved belastningsenden.

**Rengøring:**
- Fjern støv (især *metalstøv*) fra drevets overflade, så det ikke kan trænge ind i drevet.
- Hold drevet godt ventileret; hold frontens konnektorer rene.

> Afbryd strømmen før rengøring. Kun trykluftpistol eller tør klud — aldrig benzin, fortynder, alkohol eller sure/alkaliske rengøringsmidler (§14.1, s.270).

## 6. Periodisk vedligehold — procedurenoter

- **Drev-/motorsliddel** (§14.2, s.271): intervaller i §4; bekræft med leverandøren inden udskiftning (§14.2-instruktion: "kontakt os eller agenten inden udskiftning").
- **Udskiftning af pasfeder** (§14.3.1, s.271–272): B-type pasfjedre på ASMK2-motoren; kiludtræksskruer pr. rammestørrelse; unbrakonøgle.
- **Udskiftning af olietætning** (§14.3.2 / §15.3.2, s.272–273): spidstang, skridsikre handsker, bomuldsklud; lir forsigtigt tætningen ud for at beskytte endeskjoldet.

**Parameter-advarsel ved service:** drevparametre (forstærkninger, inertiforhold osv.) er **retentive** — de overlever spændingscyklusser og er gemt i drevets EEPROM, ikke i masteren. Efter vedligehold, der kan forstyrre et drev, skal du verificere, at hele objektordbogen er intakt, inden drevet aktiveres igen (repo'ets `a6_gains`-værktøj sammenligner drev). Et drev, der indsættes som reservedel, leveres med fabriksindstillinger og opfører sig ikke som sit søsterdrev, før det er tunet til samme last.

## 7. Reservedele & udskiftningsliste

| Reservedel | Del | Interval | Bemærkninger |
|---|---|---|---|
| Køleventilator | A6-drevblæser | 2–3 år / 10–30k t | §14.2 |
| Elektrolytkondensatorer | drevkort + bus | ca. 5 år | §14.2 |
| Motorleje | ASMK2-motor | 3–5 år / 20–30k t | §14.2 |
| Motorolietætning | ASMK2-motor | 5.000 t | §14.2 |
| Encoder | 17-bit multiturn | 3–5 år / 20–30k t | §14.2 |
| Encoder-batteri | 3,6 V / 2500 mAh (batteriboks) | ca. 2 år | §13.5 — skift kabel/batteriboks |
| Ophængsline | Dynamica DM20 2 mm, splejset, 65 m | ved skade | afventer leverandørspec |
| Foropladningsrelæ/-modstand | drev | 100k / 20k operationer | §14.2 |

## 8. Vedligeholdelseslog (skabelon)

| Dato | Komponent | Serie/ID | Handling | Interval / årsag | Udført af | Næste frist |
|---|---|---|---|---|---|---|
| | | | | | | |

## 9. Åbne punkter

1. **Ophængsline-service-data** — indhent Dynamica Ropes' eftersyns-/kasseringskriterier for DM20 2 mm-linen (Salgsordre 28529).
2. **Drevbeskyttelse** — A6-manualen §11 kræver automatsikring + EMI-filter + kontaktor pr. drev; aktuelt ikke installeret.
3. **Nødstop** — planlagt (TODO i §3); testinterval (funktionstest) skal tilføjes.
4. **Visionshardware** — Hailo/kamera (kollegaens ansvar); tilføj, hvis det skal serviceres.
5. **Mast/forankringsstruktur** — strukturelt eftersyn af master/forankring skal tilføjes.
6. **Serviceintervaller for øvrige komponenter** — se §4.3 (under udredning).
