#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <time.h>
#include <stdbool.h>

/* --- Configuration & Constants --- */
#define MAX_INPUTS 6           // Upgraded to allow up to 6 inputs (64 rows)
#define MAX_OUTPUTS 16
#define MAX_GATES 300          // Increased capacity
#define MAX_WIRES (MAX_INPUTS + MAX_GATES)

// --- TUNING PARAMETERS ---
#define STALL_LIMIT 75000              
#define MAX_SOLVE_GATES 200            
#define OPTIMIZATION_PLATEAU_LIMIT 150000 
#define TERMINATION_PLATEAU 5000000    

// Op Codes
#define OP_AND 0
#define OP_OR  1
#define OP_NAND 2
#define OP_NOR 3
#define OP_XOR 4
#define OP_NOT 5
#define OP_XNOR 6
#define NUM_OPS 7

const char* OP_NAMES[] = {"AND", "OR", "NAND", "NOR", "XOR", "NOT", "XNOR"};

/* --- FAST RNG (XorShift64) --- */
uint64_t rng_state;

void seed_rng(uint64_t seed) {
    rng_state = seed ? seed : 88172645463325252LL;
}

// Much faster and better distribution than rand()
uint64_t fast_rand() {
    uint64_t x = rng_state;
    x ^= x << 13;
    x ^= x >> 7;
    x ^= x << 17;
    return rng_state = x;
}

// Helper for range [min, max]
int randint(int min, int max) {
    if (max < min) return min;
    return min + (fast_rand() % (max - min + 1));
}

double rand_double() {
    return (double)fast_rand() / (double)UINT64_MAX;
}

/* --- Structures --- */

typedef struct {
    uint8_t op;       // Small optimization: uint8_t for op codes
    int16_t src_a;    // Small optimization: int16_t for wire indices
    int16_t src_b;
} Gate;

typedef struct {
    int num_inputs;
    int num_outputs;
    int allowed_ops[NUM_OPS];
    int allowed_ops_count;
    Gate gates[MAX_GATES];
    int num_gates;
    int output_map[MAX_OUTPUTS];
} Circuit;

typedef struct {
    uint64_t inputs_bin[MAX_INPUTS]; // UPGRADE: 64-bit for up to 6 inputs
    uint64_t targets_bin[MAX_OUTPUTS];
    uint64_t masks_bin[MAX_OUTPUTS];
    int num_inputs;
    int num_outputs;
    int max_score;
    int allowed_ops[NUM_OPS];
    int allowed_ops_count;
} GrowthSolver;

/* --- Core Logic --- */

// UPGRADE: 64-bit logic evaluation
uint64_t run_gate(int op_id, uint64_t val_a, uint64_t val_b) {
    switch (op_id) {
        case 0: return val_a & val_b;           // AND
        case 1: return val_a | val_b;           // OR
        case 2: return ~(val_a & val_b);        // NAND (Implicit 64-bit mask)
        case 3: return ~(val_a | val_b);        // NOR
        case 4: return val_a ^ val_b;           // XOR
        case 5: return ~val_a;                  // NOT
        case 6: return ~(val_a ^ val_b);        // XNOR
    }
    return 0;
}

int circuit_num_wires(Circuit *c) {
    return c->num_inputs + c->num_gates;
}

int circuit_add_random_gate(Circuit *c) {
    if (c->num_gates >= MAX_GATES) return -1; 

    int op_idx = randint(0, c->allowed_ops_count - 1);
    int op = c->allowed_ops[op_idx];
    
    int max_src = circuit_num_wires(c) - 1;
    int src_a = randint(0, max_src);
    int src_b = randint(0, max_src);

    c->gates[c->num_gates].op = op;
    c->gates[c->num_gates].src_a = src_a;
    c->gates[c->num_gates].src_b = src_b;
    c->num_gates++;

    return circuit_num_wires(c) - 1;
}

void circuit_evaluate(Circuit *c, uint64_t *packed_inputs, uint64_t *results) {
    uint64_t wires[MAX_WIRES];
    
    // Unroll input loading slightly for speed (compiler usually handles this)
    for (int i = 0; i < c->num_inputs; i++) wires[i] = packed_inputs[i];

    int wire_ptr = c->num_inputs;
    for (int i = 0; i < c->num_gates; i++) {
        Gate *g = &c->gates[i];
        uint64_t val_a = wires[g->src_a];
        uint64_t val_b = (g->op != 5) ? wires[g->src_b] : 0;
        wires[wire_ptr++] = run_gate(g->op, val_a, val_b);
    }
    
    for (int i = 0; i < c->num_outputs; i++) results[i] = wires[c->output_map[i]];
}

int circuit_get_active_indices(Circuit *c, int *active_indices) {
    bool used_wire[MAX_WIRES] = {0};
    for (int i = 0; i < c->num_outputs; i++) used_wire[c->output_map[i]] = true;

    int count = 0;
    // Walk backwards from last gate to first
    for (int i = c->num_gates - 1; i >= 0; i--) {
        int wire_idx = c->num_inputs + i;
        if (used_wire[wire_idx]) {
            Gate *g = &c->gates[i];
            used_wire[g->src_a] = true;
            if (g->op != 5) used_wire[g->src_b] = true;
        }
    }
    // Collect forward
    for (int i = 0; i < c->num_gates; i++) {
        int wire_idx = c->num_inputs + i;
        if (used_wire[wire_idx]) active_indices[count++] = i;
    }
    return count;
}

int circuit_count_active(Circuit *c) {
    int dummy[MAX_GATES];
    return circuit_get_active_indices(c, dummy);
}

bool circuit_compact(Circuit *c) {
    int active[MAX_GATES];
    int active_count = circuit_get_active_indices(c, active);
    if (active_count == c->num_gates) return false;

    int old_to_new[MAX_WIRES];
    for (int i = 0; i < c->num_inputs; i++) old_to_new[i] = i;

    Gate new_gates[MAX_GATES];
    int new_gate_idx = 0;

    for (int i = 0; i < active_count; i++) {
        int old_idx = active[i];
        int old_wire = c->num_inputs + old_idx;
        int new_wire = c->num_inputs + new_gate_idx;
        old_to_new[old_wire] = new_wire;

        Gate g = c->gates[old_idx];
        g.src_a = old_to_new[g.src_a];
        if (g.src_b < MAX_WIRES) g.src_b = old_to_new[g.src_b];
        else g.src_b = 0;
        new_gates[new_gate_idx++] = g;
    }

    c->num_gates = new_gate_idx;
    for (int i = 0; i < new_gate_idx; i++) c->gates[i] = new_gates[i];
    for (int i = 0; i < c->num_outputs; i++) c->output_map[i] = old_to_new[c->output_map[i]];
    return true;
}

void circuit_mutate(Circuit *c) {
    if (c->num_gates == 0) return;
    if (rand_double() < 0.75) {
        int idx = randint(0, c->num_gates - 1);
        int max_src = c->num_inputs + idx - 1;
        int what = randint(0, 2);
        if (what == 0) {
            int op_idx = randint(0, c->allowed_ops_count - 1);
            c->gates[idx].op = c->allowed_ops[op_idx];
        } else if (what == 1 && max_src >= 0) {
            c->gates[idx].src_a = randint(0, max_src);
        } else if (max_src >= 0) {
            c->gates[idx].src_b = randint(0, max_src);
        }
    } else {
        int out_idx = randint(0, c->num_outputs - 1);
        c->output_map[out_idx] = randint(0, circuit_num_wires(c) - 1);
    }
}

/* --- Solver --- */

// UPGRADE: Use hardware population count
int count_set_bits(uint64_t n) {
    #ifdef __GNUC__
        return __builtin_popcountll(n);
    #else
        // Fallback for non-GCC compilers
        int count = 0;
        while (n > 0) { n &= (n - 1); count++; }
        return count;
    #endif
}

void solver_init(GrowthSolver *s, const char *tt_str, const char *allowed_gates_str) {
    s->allowed_ops_count = 0;
    if (strcmp(allowed_gates_str, "ALL") == 0) {
        for(int i=0; i<NUM_OPS; i++) s->allowed_ops[i] = i; 
        s->allowed_ops_count = NUM_OPS;
    } else {
        char buf[256];
        strncpy(buf, allowed_gates_str, 255);
        char *token = strtok(buf, " ");
        while(token) {
            for(int i=0; i<NUM_OPS; i++) {
                if (strcmp(token, OP_NAMES[i]) == 0) s->allowed_ops[s->allowed_ops_count++] = i;
            }
            token = strtok(NULL, " ");
        }
    }

    char tt_copy[2048]; // Increased buffer
    strncpy(tt_copy, tt_str, 2047);
    char *saveptr;
    char *row_str = strtok_r(tt_copy, " ", &saveptr);
    
    memset(s->inputs_bin, 0, sizeof(s->inputs_bin));
    memset(s->targets_bin, 0, sizeof(s->targets_bin));
    memset(s->masks_bin, 0, sizeof(s->masks_bin));

    int r_idx = 0;
    s->num_inputs = 0;
    s->num_outputs = 0;

    // Detect dimensions from first row
    if (row_str) {
        char *colon = strchr(row_str, ':');
        if (colon) {
            s->num_inputs = (int)(colon - row_str);
            s->num_outputs = strlen(colon + 1);
        }
    }
    
    if (s->num_inputs > MAX_INPUTS) {
        printf("Error: Input count (%d) exceeds MAX_INPUTS (%d)\n", s->num_inputs, MAX_INPUTS);
        exit(1);
    }

    // Re-parse
    strncpy(tt_copy, tt_str, 2047);
    row_str = strtok_r(tt_copy, " ", &saveptr);

    while (row_str) {
        char *colon = strchr(row_str, ':');
        if (colon) {
            *colon = '\0';
            char *lhs = row_str;
            char *rhs = colon + 1;
            
            // Limit to 64 rows
            if (r_idx < 64) {
                for (int i = 0; i < s->num_inputs; i++) {
                    if (lhs[i] == '1') s->inputs_bin[i] |= (1ULL << r_idx);
                }
                for (int i = 0; i < s->num_outputs; i++) {
                    if (rhs[i] != 'X') {
                        s->masks_bin[i] |= (1ULL << r_idx);
                        if (rhs[i] == '1') s->targets_bin[i] |= (1ULL << r_idx);
                    }
                }
            }
            r_idx++;
        }
        row_str = strtok_r(NULL, " ", &saveptr);
    }

    s->max_score = 0;
    for (int i = 0; i < s->num_outputs; i++) s->max_score += count_set_bits(s->masks_bin[i]);
}

int solver_score(GrowthSolver *s, Circuit *c) {
    uint64_t outputs[MAX_OUTPUTS];
    circuit_evaluate(c, s->inputs_bin, outputs);
    int total = 0;
    for (int i = 0; i < s->num_outputs; i++) {
        uint64_t matches = ~(outputs[i] ^ s->targets_bin[i]) & s->masks_bin[i];
        total += count_set_bits(matches);
    }
    return total;
}

void solver_try_remove_gate(GrowthSolver *s, Circuit *c) {
    bool improved = true;
    int active[MAX_GATES];
    while (improved) {
        improved = false;
        int active_count = circuit_get_active_indices(c, active);
        for (int k = active_count - 1; k >= 0; k--) {
            int gate_idx = active[k];
            int wire_idx = c->num_inputs + gate_idx;
            Gate *gate = &c->gates[gate_idx];
            Circuit backup = *c;

            int replacement = gate->src_a;
            for (int i = 0; i < c->num_outputs; i++) if (c->output_map[i] == wire_idx) c->output_map[i] = replacement;
            for (int i = 0; i < c->num_gates; i++) {
                if (c->gates[i].src_a == wire_idx) c->gates[i].src_a = replacement;
                if (c->gates[i].src_b == wire_idx) c->gates[i].src_b = replacement;
            }
            if (solver_score(s, c) == s->max_score) { improved = true; break; }
            *c = backup;

            if (gate->op != 5) {
                replacement = gate->src_b;
                for (int i = 0; i < c->num_outputs; i++) if (c->output_map[i] == wire_idx) c->output_map[i] = replacement;
                for (int i = 0; i < c->num_gates; i++) {
                    if (c->gates[i].src_a == wire_idx) c->gates[i].src_a = replacement;
                    if (c->gates[i].src_b == wire_idx) c->gates[i].src_b = replacement;
                }
                if (solver_score(s, c) == s->max_score) { improved = true; break; }
                *c = backup;
            }
        }
    }
    circuit_compact(c);
}

bool solver_solve(GrowthSolver *s, int max_gates, int stall_limit, Circuit *result) {
    printf("Problem: %d inputs -> %d outputs\n", s->num_inputs, s->num_outputs);
    printf("Target bits: %d\n", s->max_score);
    printf("Strategy: Deep Opt with Termination Plateau (64-bit Engine)\n\n");

    Circuit parent;
    parent.num_inputs = s->num_inputs;
    parent.num_outputs = s->num_outputs;
    parent.num_gates = 0;
    parent.allowed_ops_count = s->allowed_ops_count;
    memcpy(parent.allowed_ops, s->allowed_ops, sizeof(s->allowed_ops));

    circuit_add_random_gate(&parent);
    for (int i = 0; i < s->num_outputs; i++) parent.output_map[i] = randint(0, circuit_num_wires(&parent) - 1);

    int parent_score = solver_score(s, &parent);
    int best_score_at_size = parent_score;
    
    int gens_since_improvement = 0;        
    int gens_since_kick_trigger = 0;       
    int gens_since_absolute_best = 0;      

    int gen = 0;
    int best_active = 999;
    bool found_solution = false;

    while (parent.num_gates <= max_gates) {
        gen++;
        gens_since_improvement++;
        gens_since_kick_trigger++;
        gens_since_absolute_best++;

        Circuit child = parent;
        int num_muts = (rand_double() < 0.8) ? 1 : randint(2, 3);
        for (int i = 0; i < num_muts; i++) circuit_mutate(&child);
        int child_score = solver_score(s, &child);

        if (child_score >= parent_score) {
            if (child_score > parent_score) {
                gens_since_improvement = 0;
                if (child_score > best_score_at_size) best_score_at_size = child_score;
            }
            parent = child;
            parent_score = child_score;
        }

        if (parent_score == s->max_score) {
            Circuit temp_check = parent;
            circuit_compact(&temp_check); 
            int active = temp_check.num_gates;

            if (active < best_active) {
                best_active = active;
                *result = temp_check;
                found_solution = true;
                
                gens_since_kick_trigger = 0;
                gens_since_absolute_best = 0;
                
                printf("Gen %d: NEW BEST! %d gates.\n", gen, active);
                solver_try_remove_gate(s, result);
                int pruned_active = circuit_count_active(result);
                
                if (pruned_active < best_active) {
                     best_active = pruned_active;
                     // printf("  -> Pruned down to: %d gates\n", best_active);
                }
                
                parent = *result; 
                parent.num_gates = pruned_active;
            }
        }

        if (found_solution && gens_since_kick_trigger > OPTIMIZATION_PLATEAU_LIMIT) {
            // printf("Gen %d: stuck at %d gates (Randomizing for better convergence...)\n", gen, best_active);
            for(int k=0; k<5; k++) circuit_mutate(&parent);
            parent_score = solver_score(s, &parent);
            gens_since_kick_trigger = 0;
        }

        if (found_solution && gens_since_absolute_best > TERMINATION_PLATEAU) {
            printf("\n*** CONVERGED ***\n");
            printf("Best solution (%d gates) hasn't improved for %d generations so we stop now.\n", best_active, TERMINATION_PLATEAU);
            return true;
        }

        if (gens_since_improvement > stall_limit) {
            if (parent.num_gates < max_gates) {
                int new_wire = circuit_add_random_gate(&parent);
                if (rand_double() < 0.3) {
                    int out_idx = randint(0, s->num_outputs - 1);
                    parent.output_map[out_idx] = new_wire;
                }
                parent_score = solver_score(s, &parent);
                best_score_at_size = parent_score;
                gens_since_improvement = 0;
                // printf("Gen %d: stalling. adding gates now its %d gates.\n", gen, parent.num_gates);
            } else {
                gens_since_kick_trigger = OPTIMIZATION_PLATEAU_LIMIT + 1;
            }
        }

        if (gen % 50000 == 0) {
             printf("Gen %d: GateNum: %d, Score: %d/%d, Activated Gates: %s%d\n", 
                   gen, parent.num_gates, parent_score, s->max_score, 
                   found_solution ? "" : "None yet", best_active == 999 ? 0 : best_active);
        }
    }

    return found_solution;
}

void get_source_name(int idx, int num_inputs, Circuit *c, char *buffer) {
    if (idx < num_inputs) {
        sprintf(buffer, "Input (%d)", idx);
    } else {
        int g_idx = idx - num_inputs;
        int op = c->gates[g_idx].op;
        sprintf(buffer, "%s Gate (%d)", OP_NAMES[op], g_idx);
    }
}

void render_circuit(Circuit *c, GrowthSolver *s) {
    printf("\n============================================================\n");
    printf(" NETLIST: %d GATES\n", c->num_gates);
    printf("============================================================\n");

    typedef struct {
        char src[32];
        char dst[32];
    } Line;

    Line *lines = (Line *)malloc(sizeof(Line) * (c->num_gates * 2 + c->num_outputs));
    int line_count = 0;

    for (int i = 0; i < c->num_gates; i++) {
        Gate *g = &c->gates[i];
        char dst_name[32];
        sprintf(dst_name, "%s Gate (%d)", OP_NAMES[g->op], i);
        get_source_name(g->src_a, s->num_inputs, c, lines[line_count].src);
        strcpy(lines[line_count].dst, dst_name);
        line_count++;
        if (g->op != 5) {
            get_source_name(g->src_b, s->num_inputs, c, lines[line_count].src);
            strcpy(lines[line_count].dst, dst_name);
            line_count++;
        }
    }
    for (int i = 0; i < c->num_outputs; i++) {
        get_source_name(c->output_map[i], s->num_inputs, c, lines[line_count].src);
        sprintf(lines[line_count].dst, "Output (%d)", i);
        line_count++;
    }

    int max_src_len = 0;
    for (int i = 0; i < line_count; i++) {
        int len = strlen(lines[i].src);
        if (len > max_src_len) max_src_len = len;
    }
    int col_width = max_src_len + 4;
    for (int i = 0; i < line_count; i++) {
        printf("%s", lines[i].src);
        int pad = col_width - strlen(lines[i].src);
        for(int k=0; k<pad; k++) putchar(' ');
        printf("---->    %s\n", lines[i].dst);
    }
    printf("============================================================\n\n");
    free(lines);
}

int main() {
    seed_rng(time(NULL));

    // input truth table separated by spaces i.e. "00:0 01:1 10:1 11:0"
    const char *tt = 
        "0000:1111110 0001:0110000 0010:1101101 0011:1111001 "
        "0100:0110011 0101:1011011 0110:1011111 0111:1110000 "
        "1000:1111111 1001:1111011 1010:XXXXXXX 1011:XXXXXXX "
        "1100:XXXXXXX 1101:XXXXXXX 1110:XXXXXXX 1111:XXXXXXX";

    // Separated by space you can specify allowed gates to be used "ALL" or "NOR XOR OR XNOR NAND AND NOT" or any combination of allowing.
    GrowthSolver solver;
    solver_init(&solver, tt, "ALL");

    Circuit solution;
    bool solved = solver_solve(&solver, MAX_SOLVE_GATES, STALL_LIMIT, &solution);

    if (solved) {
        render_circuit(&solution, &solver);
    }

    return 0;
}
