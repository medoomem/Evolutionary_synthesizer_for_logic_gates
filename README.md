# Logic Circuit Evolver in C

A high-performance, evolutionary algorithm implemented in pure C that generates and optimizes digital logic circuits based on arbitrary truth tables. 

Unlike standard solvers that simply find *a* solution, this engine employs a **Continuous Growth & Pruning** strategy to find the **smallest possible circuit** (fewest gates) that satisfies the logic requirements.

## 🚀 Features

*   **Zero Dependencies:** Written in standard C (`stdlib`, `stdio`, `string`, `time`).
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
Compile with high optimization flags for the best performance:

```bash
gcc -O3 solver.c -o solver
```

### Running
```bash
./solver.c
```

## ⚙️ Configuration

All configuration is currently done via macros and variables in the `main` function and the header section of the source code.

### 1. Defining the Truth Table
In `main()`, modify the `tt` string. The format is `inputs:outputs`.
*   `0` / `1`: Logic Low / High
*   `X`: Don't Care (The solver ignores these bits)

**Example (Half Adder):**
```c
// A B : Sum Carry
const char *tt = "00:00 01:10 10:10 11:01";
```

### 2. restricting Gate Types
In `main()`, pass the allowed gates to `solver_init`:
```c
// Use all standard gates (AND, OR, NAND, NOR, XOR, XNOR, NOT)
solver_init(&solver, tt, "ALL");

// OR: Restrict to specific gates
solver_init(&solver, tt, "NAND NOR");
```

### 3. Tuning the Algorithm
Adjust the macros at the top of the file to change solver behavior:

| Macro | Default | Description |
| :--- | :--- | :--- |
| `MAX_SOLVE_GATES` | `200` | The hard limit on circuit size. |
| `STALL_LIMIT` | `75000` | If the score (accuracy) doesn't improve, add a new gate. |
| `OPTIMIZATION_PLATEAU_LIMIT` | `150000` | If the *size* of the solution doesn't drop, apply a "Kick" (random mutation). |
| `TERMINATION_PLATEAU` | `5000000` | If the *best* size found hasn't changed for this long, stop and print results. |

## 🧠 How It Works

1.  **Evolution:** The solver starts with a single random gate. It mutates connections and operations randomly.
2.  **Selection:** Changes are accepted if the circuit's score (matching the truth table) is better than or equal to the previous generation.
3.  **Growth:** If the score stops improving (`STALL_LIMIT`), the solver adds a new random gate to increase complexity.
4.  **Pruning:** As soon as the circuit achieves a perfect score (100% match), the solver enters "Pruning Mode," attempting to bypass and remove every gate to see if the solution holds.
5.  **Escape:** If the solution size stagnates, the solver triggers a "Kick"—applying multiple mutations at once to jump out of local minima.

## 📊 Sample Output

```text
============================================================
 NETLIST: 13 GATES
============================================================
Input (0)        ---->    NOR Gate (0)
Input (1)        ---->    NOR Gate (0)
...
OR Gate (12)     ---->    Output (0)
============================================================
```

## 📝 License

This project is open-source. Feel free to modify and distribute.
