# **Trinity Trader Triangulator**
*Automated trader satellite triangulation system using 4-step matrix math & polynomial correction*

**11 IC10 scripts across 9 chip housings** for precision trader location tracking

---

## **The Scripts**

**Core Control**
• **TrinitySearch** - Main controller (spiral scan + triangulation)
• **TrinityOS** - OS with scrolling LED display (15 status messages)

**4-Step Matrix Computer** (converts angles → X,Y,Z coords)
• **TrinityOne** - Angles to direction vectors (trig calcs)
• **TrinityTwo** - Builds triangulation matrix
• **TrinityThree** - Matrix inverse (cofactor method)
• **TrinityFour** - Final coordinate solver

**Angle Correction**
• **TrinityAngleSolver** - Piecewise interpolation for dish non-linearity
• **TrinityPolynomial** - Forward lookup (setting → true angle)
• **TrinityPolynomial2** - Inverse lookup (angle → setting)

**Initialization** (run once)
• **TrinityOS Stack Loader** - Preloads 15 display messages
• **TrinityPolynomial Stack Loader** - Loads 91-point correction table

---

## **Quick Setup**

**1.** Create 9 chip housings: `TrinitySearch`, `TrinityOS`, `TrinityAngleSolver`, `TrinityPolynomial`, `TrinityPolynomial2`, `TrinityOne`, `TrinityTwo`, `TrinityThree`, `TrinityFour`

**2.** Run both Stack Loader scripts once to initialize data

**3.** Load the 9 operational scripts into their respective chip housings

**4.** Connect hardware: satellite dish(es), Logic Dial, Enter/Reset buttons, 1×3 Console LED (label "TrinityOS")

**5.** Power on, press Enter to start

---

## **Usage**
• **Enter button** - Start search / Interrogate / Call down
• **Reset button** - Restart system
• **Logic Dial** - Select trader to triangulate

---

## **Technical Specs**
```
Scan: 0.9°H × 0.055°V spiral
Triangulation: 20°H × 5°V triangle
  Shot1: H+1°
  Shot2: H+21°, V+2.5°
  Shot3: H+11°, V-2.5°
Correction: 91-point polynomial table
Display: 6-char scrolling (bit-shift)
Comms: db register data bus
Math: Full floating-point
```

**Version:** Trinity OS 1.0 | **Author:** FlorpyDorp

*Find me on Stationeers Discord, or pray*
