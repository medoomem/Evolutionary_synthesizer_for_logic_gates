# Logic Circuit Evolver in C

A high-performance, evolutionary algorithm implemented in pure C that generates and optimizes digital logic circuits based on arbitrary truth tables.

Unlike standard solvers that simply find *a* solution, this engine employs a **Continuous Growth & Pruning** strategy to find the **smallest possible circuit** (fewest gates) that satisfies the logic requirements.

**✨ NEW v2 Engine:** Now features a Multi-Word Bitset architecture, allowing it to solve complex logic problems with **10+ inputs** (1024+ simulation rows) while maintaining extreme speed via hardware-accelerated population counts.

## 🚀 Features

*   **Zero Dependencies:** Written in standard C (`stdlib`, `stdio`, `string`, `time`).
*   **Scalable Architecture:** Uses a chunked bitset engine to break the 64-bit limit. Solves for 8, 10, or even 16 inputs (configurable).
*   **Advanced Mutations:** Includes **Topological Gate Swapping** to untangle crossing wires without breaking logic validity.
*   **Arbitrary Truth Tables:** Solves for any number of inputs and outputs (configured via string).
*   **Gate Set Control:** Restrict the solver to specific logic families (e.g., "NAND" Only, "XOR AND", or "ALL").
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
gcc -O3 -march=native solver.c -o solver
```

### Running
```bash
./solver
```

## ⚙️ Configuration

All configuration is done via macros and variables in the `main` function and the header section of `solver.c`.

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

**Simulation Capacity (The Multi-Word Engine):**
| Macro | Default | Description |
| :--- | :--- | :--- |
| `MAX_INPUTS` | `10` | The number of input pins. `10` allows up to 1024 truth table rows. |
| `MAX_CHUNKS` | `16` | How many 64-bit words are used for parallel evaluation. Formula: `2^MAX_INPUTS / 64`. |
| `MAX_GATES` | `300` | The maximum physical capacity of the netlist. |

**Solver Heuristics:**
| Macro | Default | Description |
| :--- | :--- | :--- |
| `MAX_SOLVE_GATES` | `200` | The target limit for circuit size during solving. |
| `STALL_LIMIT` | `75000` | If the score (accuracy) doesn't improve, add a new gate. |
| `OPTIMIZATION_PLATEAU_LIMIT` | `150000` | If the *size* of the solution doesn't drop, apply a "Kick" (random mutation). |
| `TERMINATION_PLATEAU` | `5000000` | If the *best* size found hasn't changed for this long, stop and print results. |

## 🧠 How It Works

1.  **Evolution:** The solver starts with a single random gate. It mutates inputs, opcodes, and **swaps gate positions** to optimize topology.
2.  **Vectorized Evaluation:** The engine uses `uint64_t` chunks to simulate up to 64 truth table rows in a single CPU instruction (SIMD-like behavior without intrinsics).
3.  **Selection:** Changes are accepted if the circuit's score (matching the truth table) is better than or equal to the previous generation.
4.  **Growth:** If the score stops improving (`STALL_LIMIT`), the solver adds a new random gate to increase complexity.
5.  **Pruning:** As soon as the circuit achieves a perfect score (100% match), the solver enters "Pruning Mode," attempting to bypass and remove every gate to see if the solution holds.
6.  **Escape:** If the solution size stagnates, the solver triggers a "Kick"—applying multiple mutations at once to jump out of local minima.

## 📊 Sample Output

```text
============================================================
 NETLIST: 19 GATES
============================================================
Input (3)        ---->    XOR Gate (0)
Input (7)        ---->    XOR Gate (0)
...
NAND Gate (16)   ---->    Output (0)
XOR Gate (18)    ---->    Output (1)
AND Gate (17)    ---->    Output (2)
============================================================
```

## 📝 License

This project is open-source. Feel free to modify and distribute.
