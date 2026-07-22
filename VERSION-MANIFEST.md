# CatFountain — Reorganization Manifest
Generated: 2026-07-17

This documents the new clean, sequential versioning applied to all 3 categories.
Nothing was deleted — every original file/folder was preserved and moved into
`OLD-UNSORTED-BACKUP/` (mirrored by category) exactly as it was, in case anything
here needs to be double-checked.

## Version V (Infrared) — 18 versions (was 19, 1 merged duplicate)

| New | Old | Note |
|---|---|---|
| V01-Origin | Test/V01-Origin | |
| V02-Base | Test/V02-Base | |
| V03-Iter1 | Test/V03-Iter1 | |
| V04-Iter2 | Test/V04-Iter2 | |
| V05-Iter3 | Test/V05-Iter3 | |
| V06-Iter4 | Test/V06-Iter4 | |
| V07-WithPIC | Test/V07-WithPIC | |
| V08-Iter5 | Test/V08-Iter5 | |
| V09-WithPicLowHDD | Test/V09-WithPicLowHDD | |
| V10-FullWithTransistor | Test/V10-FullWithTransistor | |
| V11-FullNoHangEmoji | Test/V11-FullNoHangEmoji | |
| V12-NewPicNotFit | Test/V12-NewPicNotFit | |
| V13-PicAppOK-IconNO | Test/V13-PicAppOK-IconNO | |
| V14-ESP32-PirRelay | Test/V14-ESP32-PirRelay | |
| V15-ESP12EEsp8266-AfterBurn | Test/V15-ESP12E-Esp8266 + Test/V16-AfterBurn | **merged** — both folders had byte-identical .ino content (confirmed via md5), kept as one version |
| V16-AllGood | Test/V17-AllGood | |
| V17-TriggerProblemFix | Test/V18-TriggerProblemFix | |
| V18-CorrectUseFirstTime-LATEST | V-1-CorrectUseFirstTime/V19-CorrectUseFirstTime | |
| Notes-MustDo.txt | V-2/MustDo.txt | loose note, not a version — kept at category root |

## Version VI (UltraSound) — 3 versions

| New | Old |
|---|---|
| V01-UltraSound | V20-UltraSound |
| V02-SolvedBug | V21-SolvedBug |
| V03-RestartButton-LATEST | V22-RestartButton-LATEST |

## Version X (LaserSensor) — 23 versions + Assets

Renumbered in true chronological order (by file modified time), merging the
Alfa/Beta/Test branches into one continuous line as requested.

| New | Old | Note |
|---|---|---|
| V01-Base | Beta/PooKooli-Fountain-BETA-V0 | |
| V02-V1.0 | Beta/PooKooli-Fountain-BETA-V1.0 | |
| V03-VL53L0X-SensorTest | Test/VL53L0X_ESP32_Test-1Sensor | |
| V04-V1.1-WorkGood | Beta/PooKooli-Fountain-BETA-V1.1-WorkGood | |
| V05-V1.1-MoreSenseSpeed | Beta/PooKooli-Fountain-BETA-V1.1-MoreSenceSpeed | |
| V06-V1.1.0 | Beta/PooKooli-Fountain-BETA-V1.1.0 | |
| V07-DualSensor-Test | Test/Dual_VL53L0X_ESP32_2Sensor | |
| V08-OledTest | Test/oled_test | |
| V09-OledTestRecovery | Test/oled_test_recovery | |
| V10-V1.2-Sensor2LCD | Beta/PooKooli-Fountain-BETA-V1.2-Sensor2-LCD | |
| V11-V1.3.1 | Beta/PooKooli-Fountain-BETA-V1.3.1 | |
| V12-V1.3.2 | Beta/PooKooli-Fountain-BETA-V1.3.2 | |
| V13-V1.4 | Beta/PooKooli-Fountain-BETA-V1.4 | |
| V14-V1.5.0 | Beta/PooKooli-Fountain-BETA-V1.5.0 | |
| V15-RfPt2272-Test | Test/rf_pt2272_test | |
| V16-V1.5.1 | Beta/PooKooli-Fountain-BETA-V1.5.1 | .ino file inside was mis-named "...V1.5.ino" — fixed to match folder |
| V17-V1.5.2 | Beta/PooKooli-Fountain-BETA-V1.5.2 | |
| V18-V1.5.3 | Beta/PooKooli-Fountain-BETA-V1.5.3 | |
| V19-V1.5.4 | Beta/PooKooli-Fountain-BETA-V1.5.4 | .ino filename typo fixed (was "...V1.5.ino") |
| V20-V1.5.5 | Beta/PooKooli-Fountain-BETA-V1.5.5 | .ino filename typo fixed (was "...V1.5.ino") |
| V21-V1.5-Final | Beta/PooKooli-Fountain-BETA-V1.5 | last of the 1.5.x line, saved after 1.5.5 by timestamp |
| V22-V1.6 | Beta/PooKooli-Fountain-BETA-V1.6 | duplicate loose copy "PooKooli-Fountain-BETA-V1.6.0" (byte-identical, confirmed via md5) was dropped |
| V23-AlfaX-V1.0-LATEST | Alfa/PooKooli-Fountain-X-V1.0 | most recent build, 2026-07-17 |

### Assets/
| New | Old | Note |
|---|---|---|
| Assets/schematic.svg | pookooli_schematic.svg | `pookooli_schematic2.svg` and `schema.svg` were byte-identical duplicates (confirmed via md5) — dropped |
| Assets/Image001.jpg | Image001.jpg | |
| Assets/Screenshot-2026-07-13-145037.png | Screenshot 2026-07-13 145037.png | renamed, no spaces |

`PooKooli Fountain - Shortcut.lnk` was left in place at the category root, untouched
(it's a Windows shortcut file, not source/asset — out of scope).

## Important note on the backup

All bash/file operations run in a sandboxed mount that is **not permitted to
delete files** on your real computer (a deliberate safety guardrail — nothing
can be destroyed by accident). Because of this, instead of deleting the old
messy folders, they were moved intact into:

```
0-CatFountain/OLD-UNSORTED-BACKUP/
  Version V (Infrared)/    <- old Test/, V-1-CorrectUseFirstTime/, V-2/
  Version VI (UltraSound)/ <- old V20/V21/V22 folders
  Version X (LaserSensor)/ <- old Alfa/, Beta/, Test/, loose svgs/images, duplicate V1.6.0
```

Once you've confirmed the new structure looks right, you can delete
`OLD-UNSORTED-BACKUP/` yourself from Windows Explorer (or File Explorer's
Recycle Bin) whenever you're comfortable — it is a full, untouched copy of
everything that was there before.

---

# Addendum — GitHub publication (2026-07-22)

Prepared for publication at <https://github.com/devhimoco/CatFountain>.
**No sketch source was modified.** Changes were limited to folder/file naming,
asset location, and added documentation.

## Top-level folders renamed

Spaces and parentheses become percent-encoded in GitHub URLs
(`Version%20V%20(Infrared)`), so the four top-level folders were renamed.
Version folders *inside* them keep their original names exactly.

| Before | After |
|---|---|
| `Version V (Infrared)` | `1-infrared/` |
| `Version VI (UltraSound)` | `2-ultrasonic/` |
| `Version X (LaserSensor)` | `3-laser/` |
| `StableVersion` | `stable/` |

## Assets moved to the repository root

`Version X (LaserSensor)/Assets/` → `assets/`, since the schematic and photos
describe the project as a whole rather than one generation.
`assets/evolution.svg` was newly created for the main README.

## Two `.ino` filenames corrected

Arduino requires the sketch filename to match its folder name; these two did
not, so the IDE would not open them cleanly. Same class of fix as the typo
corrections recorded above. Contents untouched.

| Folder | Was | Now |
|---|---|---|
| `3-laser/Alfa-X/V29-AlfaX-V1.1-FullResponsiveDesign` | `…-FullResponsiveDesign-LATEST.ino` | `…-FullResponsiveDesign.ino` |
| `3-laser/Alfa-X/V32AlfaX-V2.2-Debug-RestartOverNight` | `V30AlfaX-V2.2-…ino` (copy-paste typo — folder is V32) | `V32AlfaX-V2.2-…ino` |

All 57 sketches now satisfy the folder-name/filename rule.

## Excluded from the repository (still present on local disk)

Listed in `.gitignore`, not deleted:

- `OLD-UNSORTED-BACKUP.zip` — 1.5 MB, and what it contained is already
  documented above.
- `3-laser/Beta/V12-V1.3.2/{V01-Base, V02-V1.0, V04-…, V05-…, V06-…, V10-…, V11-…}`
  — accidentally nested folders whose contents are **byte-identical** to the
  siblings one level up at `Beta/` (verified with `cmp`). Only
  `V12-V1.3.2.ino` itself is tracked.

## Documentation added

`README.md` · `CHANGELOG.md` · `LICENSE` (MIT) · `.gitignore` ·
`docs/SETUP.md` · `docs/HARDWARE.md` · and a README in each of
`1-infrared/`, `2-ultrasonic/`, `3-laser/`, `3-laser/ModuleTest/`,
`3-laser/Beta/`, `3-laser/Alfa-X/`, `stable/`.
