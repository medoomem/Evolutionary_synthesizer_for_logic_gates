# Logic Circuit Evolver in C

A high-performance, evolutionary algorithm implemented in pure C that generates and optimizes digital logic circuits based on arbitrary truth tables.

Unlike standard solvers that simply find *a* solution, this engine employs a **Continuous Growth & Pruning** strategy to find the **smallest possible circuit** (fewest gates) that satisfies the logic requirements.

**✨ NEW: DLS2 Integration** — Directly export optimized circuits to [Digital Logic Simulator 2](https://sebastian.itch.io/digital-logic-sim) by Sebastian Lague! The exporter automatically reads your existing gate definitions and registers new chips in your project.

**✨ v2 Engine:** Features a Multi-Word Bitset architecture, allowing it to solve complex logic problems with **10+ inputs** (1024+ simulation rows) while maintaining extreme speed via hardware-accelerated population counts.

## 🚀 Features

*   **Zero Dependencies:** Written in standard C (`stdlib`, `stdio`, `string`, `time`).
*   **Scalable Architecture:** Uses a chunked bitset engine to break the 64-bit limit. Solves for 8, 10, or even 16 inputs (configurable).
*   **Advanced Mutations:** Includes **Topological Gate Swapping** to untangle crossing wires without breaking logic validity.
*   **Arbitrary Truth Tables:** Solves for any number of inputs and outputs (configured via string).
*   **Gate Set Control:** Restrict the solver to specific logic families (e.g., "NAND" Only, "XOR AND", or "ALL").
*   **DLS2 Export:** Generates fully compatible JSON chip files for Digital Logic Simulator 2.
*   **Smart Optimization Strategy:**
    *   **Continuous Growth:** Adds complexity only when the algorithm stalls.
    *   **Aggressive Pruning:** Immediately attempts to remove redundant gates once a solution is found.
    *   **The "Kick" Mechanism:** Deliberately destabilizes the circuit if the optimization gets stuck in a local minimum.
    *   **Plateau Termination:** Automatically stops when the best solution hasn't improved for a defined number of generations.

## 🛠️ Getting Started

### Prerequisites
You need a C compiler (GCC, Clang, or MSVC).

### Compilation
Compile with high optimization flags for the best performance (essential for bitwise operations):

```bash
gcc -O3 solver.c -o solver
```

### Running
```bash
./solver
```

## 🎮 DLS2 Integration
This solver can export circuits directly to Digital Logic Simulator 2 format. The exported chips are fully functional and can be used immediately in the game.

### ⚠️ Crucial Requirement
**You must manually create your basic logic chips inside DLS2 before running the solver.** 

DLS2 generates unique PinIDs for every user. For the synthesizer to work, it must "learn" your specific IDs. 
1. Open DLS2 and create a new project.
2. Create and save a chip for each gate type you want to use (e.g., create a chip named `AND` using an AND gate, a chip named `XOR` using an XOR gate, etc.).
3. Ensure these `.json` files exist in your `Chips/` folder before running the solver.

### Directory Structure
Place the compiled executable in your DLS2 project's Chips folder:
```text
YourDLS2Project/
├── ProjectDescription.json
└── Chips/
    ├── AND.json          ← Manually created in DLS2
    ├── OR.json           ← Manually created in DLS2
    ├── XOR.json          ← Manually created in DLS2
    ├── NOT.json          ← Manually created in DLS2
    ├── solver.exe        ← Run from here!
    └── SYNTH.json        ← Generated output
```

### How It Works
1.  **Auto-Discovery:** On startup, the solver scans your `Chips/` folder for existing gate JSON files and extracts their internal PinIDs.
2.  **NAND Built-in:** The NAND gate uses hardcoded IDs (0, 1, 2) since it's DLS2's fundamental built-in gate.
3.  **Smart Export:** Generates a complete chip JSON with proper wiring, pin connections, and layout.
4.  **Project Registration:** Automatically updates `../ProjectDescription.json` to add your chip to:
    *   `AllCustomChipNames` array
    *   `StarredList` array
    *   `OTHER` collection in `ChipCollections`

### Configuration
Set your chip name in the source code:
```c
#define DLS2_CHIP_NAME "SYNTH"        // Output filename and chip name
#define DLS2_HEIGHT_MULTIPLIER 0.35f  // Chip height scaling per pin
#define DLS2_BASE_HEIGHT 0.5f         // Minimum chip height
```

### Example Output
```text
=== Loading DLS2 Gate Pin Mappings ===
  [OK] NAND  : in_a=0, in_b=1, out=2 (built-in)
  [OK] AND   : in_a=1053844711, in_b=823600647, out=393585090
  [OK] OR    : in_a=1130941555, in_b=684334510, out=938233364
  ...
Loaded 7/7 gate definitions
=======================================

Problem: 4 inputs -> 7 outputs
Target bits: 70
...
Gen 12345: NEW BEST! 23 gates.

Exported to SYNTH.json (size: 0.73 x 2.25, 23 gates, 52 wires)
Updating ProjectDescription.json for chip 'SYNTH'...
  [ADD]  AllCustomChipNames: 'SYNTH'
  [ADD]  StarredList: 'SYNTH'
  [ADD]  OTHER collection: 'SYNTH'
ProjectDescription.json updated successfully!
```

## ⚙️ Configuration
All configuration is done via macros and variables in the main function and the header section of `solver.c`.

### 1. Defining the Truth Table
In `main()`, modify the `tt` string. The format is `inputs:outputs`.
*   `0` / `1`: Logic Low / High
*   `X`: Don't Care (The solver ignores these bits)

**Example (Half Adder):**
```c
// A B : Sum Carry
const char *tt = "00:00 01:10 10:10 11:01";
```

### 2. Restricting Gate Types
In `main()`, pass the allowed gates to `solver_init`:
```c
// Use all standard gates (AND, OR, NAND, NOR, XOR, XNOR, NOT)
solver_init(&solver, tt, "ALL");

// OR: Restrict to specific gates
solver_init(&solver, tt, "NAND NOR");
```

### 3. Tuning the Algorithm
Adjust the macros at the top of `solver.c` to change capacity and behavior.

**Simulation Capacity:**
| Macro | Default | Description |
|-------|---------|-------------|
| `MAX_INPUTS` | 10 | Max input pins (up to 1024 truth table rows). |
| `MAX_CHUNKS` | 16 | 64-bit words for parallel evaluation (2^MAX_INPUTS / 64). |
| `MAX_GATES` | 300 | Max physical capacity of the netlist. |

## 🧠 How It Works

1.  **Evolution:** The solver starts with a single random gate. It mutates inputs, opcodes, and swaps gate positions to optimize topology.
2.  **Vectorized Evaluation:** The engine uses `uint64_t` chunks to simulate up to 64 truth table rows in a single CPU instruction.
3.  **Selection:** Changes are accepted if the circuit's score is better than or equal to the previous generation.
4.  **Growth:** If the score stops improving (`STALL_LIMIT`), the solver adds a new random gate.
5.  **Pruning:** As soon as the circuit achieves a perfect score, the solver enters "Pruning Mode," attempting to remove every redundant gate.
6.  **Export:** Once converged, the circuit is exported to DLS2-compatible JSON with automatic project registration.

## 🔧 Troubleshooting

**Gate not found errors**
```text
[--] AND   : AND.json not found
```
**Fix:** You must create an `AND` chip inside DLS2 first. See the **Crucial Requirement** section above.

**ProjectDescription.json not found**
```text
Warning: Could not open ../ProjectDescription.json
```
**Fix:** Ensure the `solver.exe` is located inside the `Chips/` sub-folder of your project.

## 📝 License
This project is open-source. Feel free to modify and distribute.
