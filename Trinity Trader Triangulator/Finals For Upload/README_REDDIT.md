# Trinity Trader Triangulator - IC10 Scripts

The Trinity Trader Triangulator (Triple T system) is a sophisticated multi-chip computer system for Stationeers that locates and triangulates trader satellite positions with precision. This system uses a 4-step Dot Matrix Math computer architecture along with advanced polynomial correction algorithms to accurately calculate three-dimensional trader coordinates from satellite dish readings.

---

## System Overview

The Triple T system consists of **11 IC10 scripts** distributed across **9 chip housings**. The system performs spiral scanning, three-shot triangulation, polynomial angle correction, matrix mathematics, and displays real-time status information to the user.

---

## Core Control Scripts

### TrinitySearch.ic10
**Chip Housing:** Load into the main IC10 chip housing labeled "TrinitySearch"

The primary control script that orchestrates the entire Trinity Trader Triangulator system. This script manages the satellite dish scanning operations, executes the spiral scan pattern (0.9° horizontal × 0.055° vertical increments), and coordinates the three-shot triangulation procedure. When a trader signal is detected, it performs three precise angle measurements using an optimized pattern (Shot1 at H+1°, Shot2 at H+21° V+2.5°, Shot3 at H+11° V-2.5°) and delegates the mathematical processing to the Polynomial and AngleSolver chips. The script also handles user input through Enter and Reset buttons, manages system state, and provides status updates to TrinityOS for display.

### TrinityOS.ic10
**Chip Housing:** Load into the IC10 chip housed in the "TrinityOS" chip housing

The operating system and user interface for the Trinity Trader Triangulator. This script manages a scrolling LED display that shows 15 different status messages (Booting, Resetting, Trader Approaching, Searching, Triangulating, Error states, etc.) by reading 6-character string chunks from its stack and scrolling them left using bit-shift operations. TrinityOS monitors the health of all chip housings in the system, provides visual feedback during all operations, handles interrogation and call-down procedures for trader satellites, and serves as the communication hub between TrinitySearch and the user. Message selection is controlled via db register 511.

---

## Mathematical Processing Chips (4-Step Dot Matrix Computer)

The Trinity system uses a four-chip architecture to perform complex matrix mathematics for 3D position triangulation. These chips work in sequence to convert three angle measurements into final X, Y, Z coordinates.

### TrinityOne.ic10
**Chip Housing:** Load into the IC10 chip housed in the "TrinityOne" Chip Housing

The first stage of the 4-step Dot Matrix Math computer. TrinityOne receives three sets of angle measurements (horizontal, vertical, and signal strength) from the satellite dish readings and converts them into directional vectors using trigonometric calculations (cosine and sine operations). It applies coordinate system transformations, handles compass offset corrections (stored in db registers 3, 7, 11 for multiple dish orientations), and outputs nine vector components plus three scaling factors to TrinityTwo for matrix assembly. This chip processes raw angles into normalized direction vectors that represent the lines-of-sight to the trader from different dish positions.

### TrinityTwo.ic10
**Chip Housing:** Load into the IC10 chip housed in the "TrinityTwo" Chip Housing

The second stage Converter chip in the 4-step Dot Matrix Math computer. TrinityTwo receives the vector components from TrinityOne and constructs the mathematical matrix needed for least-squares triangulation. It performs systematic multiplication operations to build the matrix elements (computing products like r1×r1, r1×r2, r2×r3, etc.) and accumulates these products into matrix coefficients stored in registers r8 through r15. The output is a structured matrix representation that encodes the geometric relationships between the three observation lines, preparing the data for matrix inversion in TrinityThree.

### TrinityThree.ic10
**Chip Housing:** Load into the IC10 chip housed in the "TrinityThree" Chip Housing

The third stage Inverter chip in the 4-step Dot Matrix Math computer. TrinityThree takes the matrix coefficients from TrinityTwo and computes the matrix inverse using cofactor calculations. It performs determinant computations, calculates the nine cofactors (r10 through r15 and additional temporary registers), and determines the matrix determinant (stored in r0). These inverse matrix elements are critical for solving the linear system of equations that defines the trader's 3D position. The inverted matrix is then passed to TrinityFour along with the right-hand side vector for final solution.

### TrinityFour.ic10
**Chip Housing:** Load into the IC10 chip housed in the "TrinityFour" Chip Housing

The final stage Solution chip in the 4-step Dot Matrix Math computer. TrinityFour receives the inverted matrix from TrinityThree and performs the final matrix-vector multiplication to solve for the trader's X, Y, Z coordinates. It computes the dot products (r11, r12, r13 representing X, Y, Z components), divides by the determinant (r7), calculates the distance magnitude using the Pythagorean theorem (sqrt(x² + y² + z²)), and stores the final position results in db registers for TrinitySearch and other system components to use. This chip outputs the actual 3D location of the trader satellite in space.

---

## Angle Correction & Polynomial Processing

### TrinityAngleSolver.ic10
**Chip Housing:** Load into the IC10 chip housed in the "TrinityAngleSolver" chip housing

A specialized angle correction chip that compensates for non-linear behavior in satellite dish mechanics. Real-world satellite dishes don't move in perfectly linear increments - the relationship between the commanded setting and the actual angle varies across the dish's range of motion. TrinityAngleSolver uses a piecewise linear interpolation algorithm with multiple breakpoints (at ratios 0.01, 0.05, 0.1, 0.9, 1.0) to map signal strength ratios to corrected angle values. It processes the ratio of signal strength to maximum signal (r3 = r2/r1) and applies conditional corrections that adjust for dish behavior in different regions of its movement range, providing accurate angle corrections that are critical for precise triangulation.

### TrinityPolynomial.ic10
**Chip Housing:** Load into the IC10 chip housed in the "TrinityPolynomial" chip housing

The forward polynomial angle correction system. This script uses a lookup table (stored in db registers 128-218) containing 91 pre-calculated true angle values corresponding to dish settings from 0° to 90°. It accepts a dish setting S from db[0], clamps it to [0, 90], and computes the true corrected angle R using either linear interpolation (default mode) or step lookup (mode >= 0.5). The interpolation formula R = v0×(1-α) + v1×α provides smooth angle transitions between table entries, where α is the fractional part of S. This corrects for systematic errors in dish positioning, ensuring the triangulation math receives accurate angle inputs. Results are written to db[1] (true angle) and db[2] (alpha diagnostic).

### TrinityPolynomial2.ic10
**Chip Housing:** Load into the IC10 chip housed in the "TrinityPolynomial2" chip housing

The inverse polynomial calculator that performs reverse lookup operations. While TrinityPolynomial converts dish settings to true angles (S → R), TrinityPolynomial2 solves the opposite problem: given a desired true angle R (stored in db[0]), it finds the dish setting S that will produce that angle. Using the same polynomial table (db[128-218]), it performs a binary-search-like algorithm to find the smallest index n where table[n+1] >= R_target, then either returns n directly (step mode) or performs linear interpolation to find the fractional setting S = n + α, where α = (R - table[n]) / (table[n+1] - table[n]). This is essential for commanding the dish to point at calculated angles accurately.

---

## Stack Loader Scripts

### TrinityOS Stack Loader.ic10
**Chip Housing:** Run this once as a standalone initialization script (can use any available IC10 chip)

Preloads all 15 scrolling status messages into the IC10 stack memory for TrinityOS to display. Each message is stored as a part count followed by multiple 6-character string chunks (e.g., "Trader Approaching" is stored as: 3, STR("Trader"), STR(" Appro"), STR("aching")). The script also configures db register pointers (db[1] through db[15]) that point to the stack positions where each message begins, allowing TrinityOS to quickly locate and load any message by setting db[511] to the desired message ID (1-15). Messages include: Resetting, Trader Approaching, Call down, Interrogating, Interrogate, Locking Trader, Triangulating, Trader Found, Searching, Begin Search, Choose Trader, OS Nominal, Error Chip Housing, Booting, and Trinity OS 1.0. Run this script before activating TrinityOS.

### TrinityPolynomial Stack Loader.ic10
**Chip Housing:** Run this once as a standalone initialization script (can use any available IC10 chip)

Initializes the polynomial correction lookup table used by both TrinityPolynomial and TrinityPolynomial2 chips. This script pushes 91 pre-calculated angle values (representing the true dish angles for settings 0° through 90°) into stack memory starting at position 128. The table data represents empirically measured or mathematically derived correction values that account for non-linearities in the satellite dish's mechanical response. Values range from 0.01° at setting 0 to 89.99° at setting 90, with progressively larger gaps between entries at higher angles (reflecting the dish's non-linear behavior). Both forward and inverse polynomial lookup operations depend on this data. Execute this script once during system setup before running any polynomial correction operations.

---

## Installation Instructions

1. **Build the chip housing structure:**
   - Create chip housings labeled: TrinitySearch, TrinityOS, TrinityAngleSolver, TrinityPolynomial, TrinityPolynomial2, TrinityOne, TrinityTwo, TrinityThree, TrinityFour
   - Ensure all chip housings are networked and powered

2. **Initialize lookup tables:**
   - Load and run **TrinityPolynomial Stack Loader.ic10** first (this only needs to run once)
   - Load and run **TrinityOS Stack Loader.ic10** first (this only needs to run once)

3. **Load operational scripts:**
   - Load **TrinitySearch.ic10** into the TrinitySearch chip housing
   - Load **TrinityOS.ic10** into the TrinityOS chip housing
   - Load **TrinityAngleSolver.ic10** into the TrinityAngleSolver chip housing
   - Load **TrinityPolynomial.ic10** into the TrinityPolynomial chip housing
   - Load **TrinityPolynomial2.ic10** into the TrinityPolynomial2 chip housing
   - Load **TrinityOne.ic10** into the TrinityOne chip housing
   - Load **TrinityTwo.ic10** into the TrinityTwo chip housing
   - Load **TrinityThree.ic10** into the TrinityThree chip housing
   - Load **TrinityFour.ic10** into the TrinityFour chip housing

4. **Connect hardware devices:**
   - Connect at least one satellite dish (small or large) to the network
   - Connect a Logic Dial (used for trader selection)
   - Connect "Enter" and "Reset" Logic Buttons to the network
   - Connect a 1×3 Console LED display (labeled "TrinityOS") for status output
   - Ensure all devices are properly powered and networked

5. **System startup:**
   - Power on all chip housings
   - TrinityOS will run through its boot sequence and verify all chip housings
   - Press the "Enter" button to begin scanning operations
   - The system will display status messages during all phases of operation

---

## Usage

- **Start Search:** Press the Enter button to begin the spiral scan for traders
- **Reset System:** Press the Reset button at any time to restart the search process
- **Select Trader:** Use the Logic Dial to choose which trader signal to triangulate
- **Interrogate:** When prompted, press Enter to interrogate the selected trader satellite
- **Call Down:** After interrogation, press Enter again to activate the trader call-down procedure

The system will automatically handle scanning, triangulation, angle correction, and coordinate calculation. Real-time status updates appear on the TrinityOS LED display throughout all operations.

---

## System Architecture Notes

- **db Register Communication:** The chips communicate by reading/writing shared db (data bus) registers
- **Stack Memory:** Both TrinityOS and TrinityPolynomial systems use stack memory for data storage
- **Trigger System:** Most math chips wait for db[12] to be set to 1 before executing (trigger-based processing)
- **Message System:** TrinityOS uses 6-character strings because the bit-shift scrolling (srl 40) extracts exactly one 8-bit character from a 48-bit (6-character) string
- **Polynomial Correction:** The system uses empirical correction tables because satellite dish angles are non-linear
- **4-Step Matrix Math:** The distributed matrix computation allows complex 3D triangulation calculations that would exceed single-chip instruction limits

---

## Technical Specifications

- **Scan Pattern:** 0.9° horizontal × 0.055° vertical increments (spiral scan)
- **Triangulation Pattern:** 20° horizontal × 5° vertical triangle (Shot1: H+1°, Shot2: H+21° V+2.5°, Shot3: H+11° V-2.5°)
- **Angle Range:** 0-90° (vertical), 0-360° (horizontal)
- **Correction Table:** 91-point polynomial lookup table (0° to 90°)
- **Display Messages:** 15 pre-configured scrolling status messages
- **String Encoding:** 8 bits per character, 6 characters per chunk (48 bits)
- **Math Precision:** Full floating-point calculations throughout the matrix pipeline

---

**System Version:** Trinity OS 1.0  
**Author:** FlorpyDorp  
**Platform:** Stationeers IC10 Programming Language

For additional support, updates, or contributions, find FlorpyDorp on the Stationeers Discord, or pray.
