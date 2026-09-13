# LC 3-Axis

An accurate three-axis force sensor assembled from off-the-shelf load cells,
with no custom compliant element.

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Paper](https://img.shields.io/badge/Paper-PDF-red.svg)](paper/lc-3-axis-iros2026.pdf)
[![Project Page](https://img.shields.io/badge/Project%20Page-lucasfeng7.github.io-blue.svg)](https://lucasfeng7.github.io/lc-3-axis/)

Lucas Feng, Ariel Slepyan, Krishna Murthy, Nitish Thakor
Johns Hopkins University

Presented at "Sensors and Actuators for Dexterous Manipulation: Bridging
Hardware and Application Needs," a workshop at IROS 2026 — Pittsburgh, PA.

---

## Overview

Affordable multi-axis force sensors are usually designed with a custom
compliant part, and the deformation of that part is measured after a load is
applied. Any creep, hysteresis, or thermal sensitivity in that part then
becomes a property of the finished sensor, so its accuracy depends on
operating temperature and load history in ways a single calibration cannot
capture. We instead developed a sensor using four commercial load cells that
are manufacturer-characterized to carry the full load. A single fitted
matrix maps their readings to force and accounts for assembly tolerances,
gain differences, and wiring polarity. The resulting sensor costs
\$116.55 in parts and has a 2.6&ndash;3.5 mN noise floor. Its output agrees
with a commercial reference to 1.0&ndash;2.5 % of full scale.

The full write-up, measurements, and comparison to prior work are in the
[paper](paper/lc-3-axis-iros2026.pdf) and on the
[project page](https://lucasfeng7.github.io/lc-3-axis/).

## Repository structure

```
.
├── cad/          SolidWorks source for the printed parts and top-level assembly
├── firmware/     Teensy 4.0 firmware (calibration, streaming, EEPROM storage)
├── paper/        Workshop paper (PDF)
├── media/        Demonstration video
├── docs/img/     Figures used by the project page
├── index.html    Project page (GitHub Pages)
├── CITATION.cff
└── LICENSE
```

## Bill of materials

Total **\$116.55** for the electromechanical parts below. Fasteners, cabling
and filament are excluded.

| Qty | Part | Role |
|---|---|---|
| 4 | TAL221 500 g bar load cell | the structure and the transducer, both |
| 4 | NAU7802 24-bit ADC breakout | one per cell, I²C address `0x2A` |
| 1 | TCA9548A I²C multiplexer | address `0x70`, A2:A0 to GND |
| 1 | Teensy 4.0 | reads the cells, applies the matrix, streams CSV |

Three part types are custom, and all three are FDM 3D printed.

| Part | Process | Notes |
|---|---|---|
| Upper plate | FDM, PLA | carries the tip &mdash; [`Top Plate.SLDPRT`](cad/Top%20Plate.SLDPRT) |
| Lower plate | FDM, PLA | bolts to the robot flange &mdash; [`Bottom Plate.SLDPRT`](cad/Bottom%20Plate.SLDPRT) |
| Interchangeable tip | FDM, PLA | sets contact height `h`, 38.6 mm here &mdash; [`Calibration Tip.SLDPRT`](cad/Calibration%20Tip.SLDPRT) |

No custom PCB. No machining. No cast or molded elastomer.

## CAD

SolidWorks source for the printed parts is in [`cad/`](cad/):

| File | Part |
|---|---|
| [`LC 3-Axis.SLDASM`](cad/LC%203-Axis.SLDASM) | top-level assembly |
| [`Top Plate.SLDPRT`](cad/Top%20Plate.SLDPRT) | upper plate |
| [`Bottom Plate.SLDPRT`](cad/Bottom%20Plate.SLDPRT) | lower plate |
| [`Calibration Tip.SLDPRT`](cad/Calibration%20Tip.SLDPRT) | interchangeable tip used for calibration, `h` = 38.6 mm |

The assembly references the load cells, converters and fasteners as
toolbox/purchased components, which are not included as separate files.

## How it works

Four vertical load cells connect the two 3D-printed plates in a square. Each
senses compression along the vertical axis, and together they respond to
`Fz`, `Mx`, and `My`. A lateral force is recovered from its moment about the
cell plane. For a contact at `(xc, yc)` and height `h`,

```
My = -Fx·h + Fz·xc
Mx =  Fy·h - Fz·yc
```

The contact location must therefore be known. This suits a probe with fixed
tip geometry rather than a general-purpose contact plate.

We tare each channel before loading to remove the assembly preload and the
weight of the parts above the cells. A fitted 3&times;4 matrix with a bias
converts the four tared counts into `(Fx, Fy, Fz)`. The same fit absorbs
placement error, cell-to-cell gain variation, and wiring polarity. Firmware
identifies the multiplexer channel used by each converter, so the physical
cell order does not matter.

## Firmware

[`firmware/lc_3_axis.ino`](firmware/lc_3_axis.ino) runs on the Teensy 4.0. It
discovers the multiplexer channels at boot, stores the calibration matrix and
the discovered channel map together in EEPROM, and refuses to load a
calibration whose map does not match the hardware present.

Libraries are Adafruit NAU7802 and Adafruit BusIO. I²C runs at 400 kHz on pins
18 and 19, with each ADC at gain 128 and 320 SPS.

Serial at 115200, newline endings. Output is plain CSV in newtons.

```
0.0142,-0.0031,1.9847
```

| key | does |
|---|---|
| `t` | re-tare, since zero drifts with temperature and the matrix does not |
| `r` | stream Fx, Fy, Fz |
| `p` | pause |
| `d` | also show the four raw cell counts |
| `u` | switch newtons and gram-force |

Lines beginning `#` are messages rather than data, so filter them when parsing.

## Calibrating

We calibrate against known masses rather than against another force sensor.
For each lateral axis, the sensor is clamped axis-horizontal at a bench edge
and known masses are hung from the tip, so gravity loads the sensing axis
perpendicular by construction. For the axial axis, a known mass is stacked
directly on the tip. The 3&times;4 calibration matrix is then fit against
the tared cell counts recorded at each load.

Validation against an independent reference is optional. Ours used an
OptoForce carried in series with the sensor on a UR5e, so both instruments
saw the same contact force.

## Citation

```bibtex
@inproceedings{feng2026lc3axis,
  title     = {An Accurate Three-Axis Force Sensor Assembled with Readily Available Off-the-Shelf Components with No Designed Compliant Element},
  author    = {Feng, Lucas and Slepyan, Ariel and Murthy, Krishna and Thakor, Nitish},
  booktitle = {IROS 2026 Workshop on Sensors and Actuators for Dexterous Manipulation},
  year      = {2026}
}
```

## License

MIT. See [LICENSE](LICENSE).
