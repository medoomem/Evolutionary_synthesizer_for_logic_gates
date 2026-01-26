# Logic Circuit Synthesizer

A high-performance digital logic synthesizer written in C that generates optimized circuits from arbitrary truth tables. The engine combines AIG (And-Inverter Graph) synthesis, structural pattern recognition, and Cartesian Genetic Programming (CGP) to produce minimal gate-count implementations.

Designed for integration with **Digital Logic Simulator 2** by Sebastian Lague.

## Table of Contents

* [Overview](#overview)
* [Features](#features)
* [Requirements](#requirements)
* [Installation](#installation)
* [Usage](#usage)
* [Configuration](#configuration)
* [DLS2 Integration](#dls2-integration)
* [Architecture](#architecture)
* [Troubleshooting](#troubleshooting)
* [License](#license)

## Overview

This synthesizer takes a truth table specification and produces an optimized logic circuit using a three-stage pipeline:

1.  **Structural Detection** — Recognizes common circuit patterns (adders, multipliers, encoders, etc.) and builds them using known-optimal structures
2.  **AIG Synthesis** — Converts arbitrary logic into an And-Inverter Graph representation
3.  **CGP Optimization** — Applies evolutionary optimization to minimize gate count

The output is a functionally correct circuit using your choice of logic gates, exported directly to DLS2-compatible JSON format.

## Features

### Synthesis Capabilities
*   Multi-stage synthesis pipeline combining structural, algebraic, and evolutionary methods
*   40+ pattern detectors for common circuits (arithmetic, encoders, decoders, comparators, etc.)
*   Technology mapping to arbitrary gate sets (NAND-only, NOR-only, standard gates, etc.)
*   Don't-care optimization via `X` entries in truth tables
*   Scalable architecture supporting 10+ inputs (1024+ truth table rows)

### Optimization Features
*   Continuous growth and pruning strategy for finding minimal circuits
*   Topological mutations including gate swapping to optimize wire routing
*   Multi-objective optimization balancing correctness and gate count
*   Plateau detection with automatic termination on convergence
*   Configurable refinement with compile-time feature toggles

### DLS2 Integration
*   Automatic gate discovery from existing chip definitions
*   Complete JSON export with proper wiring and layout
*   Project registration updating `ProjectDescription.json` automatically
*   Pin ID learning from user-created gate chips

## Requirements
*   C compiler with C99 support (GCC, Clang, or MSVC)
*   No external dependencies (standard library only)

## Installation

Clone the repository and compile with optimization flags:

```bash
git clone https://github.com/yourusername/logic-synthesizer.git
cd logic-synthesizer
gcc -O3 -march=native DLS2_AIG_SOLVER.c -o solver
```

For debugging builds:

```bash
gcc -g -Wall -Wextra DLS2_AIG_SOLVER.c -o solver
```

## Usage

### Basic Usage
Edit the truth table in `main()` and run:

```bash
./solver
```

### Truth Table Format
Truth tables are specified as space-separated `input:output` pairs:

```c
const char *truth_table = "00:00 01:10 10:10 11:01";  // Half adder
```

| Character | Meaning |
|-----------|---------|
| 0 | Logic low |
| 1 | Logic high |
| X | Don't care (ignored during optimization) |

### Gate Set Selection
Restrict available gates by modifying the `allowed_gates` string:

```c
const char *allowed_gates = "ALL";           // All standard gates
// const char *allowed_gates = "NAND";       // NAND-only synthesis
// const char *allowed_gates = "AND OR NOT"; // Specific subset
```

Supported gates: `AND`, `OR`, `XOR`, `NOT`, `NAND`, `NOR`, `XNOR`

### Example: BCD to 7-Segment Decoder

```c
const char *truth_table = 
    "0000:1111110 0001:0110000 0010:1101101 0011:1111001 "
    "0100:0110011 0101:1011011 0110:1011111 0111:1110000 "
    "1000:1111111 1001:1111011 1010:XXXXXXX 1011:XXXXXXX "
    "1100:XXXXXXX 1101:XXXXXXX 1110:XXXXXXX 1111:XXXXXXX";

const char *allowed_gates = "ALL";
```

## Configuration

### Compile-Time Options
Configure synthesis behavior via `#define` directives at the top of `DLS2_AIG_SOLVER.c`:

```c
/* Feature toggles (1 = enabled, 0 = disabled) */
#define ENABLE_EVOLUTIONARY_REFINEMENT  1    // CGP optimization pass
#define ENABLE_STRUCTURAL_DETECTION     1    // Pattern recognition
#define ENABLE_TECHNOLOGY_MAPPING       1    // Gate set conversion
#define ENABLE_DLS2_EXPORT              1    // JSON output generation
#define ENABLE_NETLIST_PRINT            1    // Console circuit dump

/* Evolution parameters */
#define EVO_MAX_GENERATIONS         5000000  // Hard generation limit
#define EVO_TERMINATION_PLATEAU     2000000  // Convergence threshold
#define EVO_KICK_THRESHOLD          200000   // Stall recovery trigger
#define EVO_PRINT_INTERVAL          500000   // Progress output frequency

/* Output configuration */
#define DEFAULT_CHIP_NAME           "SYNTH"  // Exported chip name
```

### Capacity Limits
Adjust these macros to change maximum problem size:

| Macro | Default | Description |
|-------|---------|-------------|
| `MAX_INPUTS` | 16 | Maximum input count (affects memory usage) |
| `MAX_OUTPUTS` | 16 | Maximum output count |
| `MAX_GATES` | 100000 | Maximum gates in evolved circuit |
| `MAX_CHUNKS` | 16 | 64-bit words for parallel simulation |

## DLS2 Integration

### Prerequisites
Before running the synthesizer, you must create template chips in DLS2:

1.  Open Digital Logic Simulator 2
2.  Create a new project
3.  For each gate type you intend to use, create and save a chip:
    *   Create chip named **AND** containing a single AND gate
    *   Create chip named **OR** containing a single OR gate
    *   Repeat for XOR, NOT, NAND, NOR, XNOR as needed
4.  Save all chips to generate their JSON files

*This step is required because DLS2 generates unique pin IDs per user installation.*

### Directory Structure
Place the compiled executable in your project's `Chips` directory:

```text
YourProject/
├── ProjectDescription.json
└── Chips/
    ├── AND.json              # Created in DLS2
    ├── OR.json               # Created in DLS2
    ├── XOR.json              # Created in DLS2
    ├── NOT.json              # Created in DLS2
    ├── NAND.json             # Created in DLS2 (optional, built-in available)
    ├── DLS2_AIG_SOLVER.exe   # Synthesizer executable
    └── SYNTH.json            # Generated output
```

### Export Process
The synthesizer automatically:
1.  Scans the `Chips/` directory for existing gate definitions
2.  Extracts pin IDs from each gate's JSON file
3.  Generates a complete chip JSON with proper wiring
4.  Updates `ProjectDescription.json` to register the new chip

### Sample Output

```text
=== Loading DLS2 Gate Pin Mappings ===
  [OK] NAND  : in_a=0, in_b=1, out=2 (built-in)
  [OK] AND   : in_a=1053844711, in_b=823600647, out=393585090
  [OK] OR    : in_a=1130941555, in_b=684334510, out=938233364
  [OK] XOR   : in_a=445289012, in_b=901234567, out=123456789
  [OK] NOT   : in_a=789012345, out=456789012
Loaded 5/7 gate definitions
=======================================

[*] Allowed gates (5): AND OR XOR NOT NAND
[*] Problem: 4 inputs, 7 outputs
[*] Detected BCD TO 7-SEGMENT!
[*] Structural synthesis: 24 gates
[*] Verification: 70 / 70 (PERFECT)
[*] After gate conversion: 24 gates

[*] Starting Evolutionary Refinement...
    Gen 50000: Score 70/70 (18 gates)
    Gen 150000: Score 70/70 (15 gates)

*** CONVERGED ***
[*] Final: 20 gates

Exported to SYNTH.json
```

## Architecture

### Synthesis Pipeline

```text
Truth Table
     │
     ▼
┌─────────────────────────┐
│  Structural Detection   │  Recognizes adders, muxes, encoders, etc.
└───────────┬─────────────┘
            │ (if no pattern found)
            ▼
┌─────────────────────────┐
│    AIG Synthesis        │  Builds And-Inverter Graph
└───────────┬─────────────┘
            │
            ▼
┌─────────────────────────┐
│  Technology Mapping     │  Converts to allowed gate set
└───────────┬─────────────┘
            │
            ▼
┌─────────────────────────┐
│   CGP Optimization      │  Evolutionary gate count reduction
└───────────┬─────────────┘
            │
            ▼
┌─────────────────────────┐
│     DLS2 Export         │  JSON generation and registration
└─────────────────────────┘
```

### Detected Patterns
The structural detector recognizes the following circuit families:

| Category | Patterns |
|----------|----------|
| **Arithmetic** | Adder, Subtractor, Multiplier, Incrementer, Decrementer, Negation, Absolute Value |
| **Comparison** | Equality, Less-than, Greater-than, Zero-detect, Min, Max |
| **Encoding** | Gray code, Priority encoder, Hamming encoder, BCD converters |
| **Decoding** | Binary decoder, BCD to decimal, 7-segment display |
| **Bit Manipulation** | Shift, Rotate, Reverse, Parity, Popcount, CLZ, CTZ |
| **Selection** | 2:1 Mux, Barrel shifter |

### Vectorized Evaluation
The engine uses 64-bit word arrays to evaluate multiple truth table rows simultaneously:

```c
typedef struct {
    uint64_t chunks[MAX_CHUNKS];
} BitVec;
```

Hardware population count instructions (`__builtin_popcountll`) accelerate fitness evaluation.

## Troubleshooting

**Gate Definition Not Found**
> `[--] AND   : AND.json not found`
>
> **Solution:** Create an AND chip in DLS2 and save it. The synthesizer requires user-created gate templates to learn pin IDs.

**Project Description Not Found**
> `Warning: Could not open ../ProjectDescription.json`
>
> **Solution:** Ensure the executable is located inside the `Chips/` subdirectory of a valid DLS2 project.

**Structural Synthesis Verification Errors**
> `[*] Verification: 66 / 70 (ERRORS!)`
> `[!] Structural synthesis has errors, falling back to AIG...`
>
> **Solution:** This indicates a bug in a structural builder. The synthesizer automatically falls back to AIG synthesis. Report the issue with the specific truth table that triggered the error.

**Slow Convergence**
If optimization takes too long:
1.  Reduce `EVO_MAX_GENERATIONS` and `EVO_TERMINATION_PLATEAU`
2.  Set `ENABLE_EVOLUTIONARY_REFINEMENT` to `0` for fast synthesis without optimization
3.  Check if your truth table matches a known pattern (structural detection is faster)

## License
This project is released under the MIT License. See LICENSE for details.

Repository: [github.com/medoomem/Evolutionary_synthesizer_for_logic_gates](https://github.com/medoomem/Evolutionary_synthesizer_for_logic_gates)
