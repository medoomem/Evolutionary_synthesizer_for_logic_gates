#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <time.h>
#include <stdbool.h>

/* --- Configuration & Constants --- */
#define MAX_INPUTS 10          // Upgraded to allow up to 10 inputs (1024 rows)
#define MAX_OUTPUTS 16
#define MAX_GATES 300          // Increased capacity
#define MAX_WIRES (MAX_INPUTS + MAX_GATES)
// pls set MAX_CHUNKS to: (2^MAX_INPUTS)/64
#define MAX_CHUNKS 16          // 1024 bits / 64 bits per chunk = 16 chunks

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

/* --- Multi-Word Bitset Structure --- */
typedef struct {
    uint64_t chunks[MAX_CHUNKS];
} BitVec;

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
    BitVec inputs_bin[MAX_INPUTS];   // UPGRADE: Multi-word bitset for up to 10 inputs
    BitVec targets_bin[MAX_OUTPUTS];
    BitVec masks_bin[MAX_OUTPUTS];
    int num_inputs;
    int num_outputs;
    int num_chunks;                   // Actual number of chunks in use
    int max_score;
    int allowed_ops[NUM_OPS];
    int allowed_ops_count;
} GrowthSolver;

/* --- Core Logic --- */

// UPGRADE: Multi-word logic evaluation
// UPGRADE: Multi-word logic evaluation
void run_gate_vec(int op_id, const BitVec *val_a, const BitVec *val_b, BitVec *result, int num_chunks) {
    // Optimization: Switch FIRST, then Loop.
    // This allows the CPU to process the array in one continuous burst.
    switch (op_id) {
        case OP_AND:
            for (int c = 0; c < num_chunks; c++)
                result->chunks[c] = val_a->chunks[c] & val_b->chunks[c];
        break;
        case OP_OR:
            for (int c = 0; c < num_chunks; c++)
                result->chunks[c] = val_a->chunks[c] | val_b->chunks[c];
        break;
        case OP_NAND:
            for (int c = 0; c < num_chunks; c++)
                result->chunks[c] = ~(val_a->chunks[c] & val_b->chunks[c]);
        break;
        case OP_NOR:
            for (int c = 0; c < num_chunks; c++)
                result->chunks[c] = ~(val_a->chunks[c] | val_b->chunks[c]);
        break;
        case OP_XOR:
            for (int c = 0; c < num_chunks; c++)
                result->chunks[c] = val_a->chunks[c] ^ val_b->chunks[c];
        break;
        case OP_NOT:
            for (int c = 0; c < num_chunks; c++)
                result->chunks[c] = ~val_a->chunks[c];
        break;
        case OP_XNOR:
            for (int c = 0; c < num_chunks; c++)
                result->chunks[c] = ~(val_a->chunks[c] ^ val_b->chunks[c]);
        break;
        default:
            for (int c = 0; c < num_chunks; c++)
                result->chunks[c] = 0;
        break;
    }
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

void circuit_evaluate(Circuit *c, BitVec *packed_inputs, BitVec *results, int num_chunks) {
    BitVec wires[MAX_WIRES];
    static const BitVec zero_vec = {0};
    
    // Load inputs
    for (int i = 0; i < c->num_inputs; i++) {
        wires[i] = packed_inputs[i];
    }

    int wire_ptr = c->num_inputs;
    for (int i = 0; i < c->num_gates; i++) {
        Gate *g = &c->gates[i];
        const BitVec *val_a = &wires[g->src_a];
        const BitVec *val_b = (g->op != OP_NOT) ? &wires[g->src_b] : &zero_vec;
        run_gate_vec(g->op, val_a, val_b, &wires[wire_ptr], num_chunks);
        wire_ptr++;
    }
    
    for (int i = 0; i < c->num_outputs; i++) {
        results[i] = wires[c->output_map[i]];
    }
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
            if (g->op != OP_NOT) used_wire[g->src_b] = true;
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

/* --- Topological Gate Swap --- */
void circuit_swap_gates(Circuit *c, int i, int j) {
    if (i == j || i < 0 || j < 0 || i >= c->num_gates || j >= c->num_gates) return;
    
    // Ensure i < j for consistent handling
    if (i > j) { int tmp = i; i = j; j = tmp; }
    
    int wire_i = c->num_inputs + i;
    int wire_j = c->num_inputs + j;
    
    // Swap the gate structures
    Gate temp = c->gates[i];
    c->gates[i] = c->gates[j];
    c->gates[j] = temp;
    
    // Update all wire references: swap wire_i <-> wire_j everywhere
    for (int k = 0; k < c->num_gates; k++) {
        // Update src_a
        if (c->gates[k].src_a == wire_i) c->gates[k].src_a = wire_j;
        else if (c->gates[k].src_a == wire_j) c->gates[k].src_a = wire_i;
        
        // Update src_b
        if (c->gates[k].src_b == wire_i) c->gates[k].src_b = wire_j;
        else if (c->gates[k].src_b == wire_j) c->gates[k].src_b = wire_i;
    }
    
    // Update output_map references
    for (int k = 0; k < c->num_outputs; k++) {
        if (c->output_map[k] == wire_i) c->output_map[k] = wire_j;
        else if (c->output_map[k] == wire_j) c->output_map[k] = wire_i;
    }
    
    // CRITICAL: Fix topological validity for affected gates
    // Gates can only reference wires with index < (num_inputs + gate_index)
    for (int k = i; k <= j; k++) {
        int max_src_k = c->num_inputs + k - 1;
        if (max_src_k < 0) max_src_k = 0;
        
        if (c->gates[k].src_a > max_src_k) {
            c->gates[k].src_a = randint(0, max_src_k);
        }
        if (c->gates[k].src_b > max_src_k) {
            c->gates[k].src_b = randint(0, max_src_k);
        }
    }
}

void circuit_mutate(Circuit *c) {
    if (c->num_gates == 0) return;
    
    double r = rand_double();
    
    // 8% chance for gate swap mutation (if we have at least 2 gates)
    if (c->num_gates >= 2 && r < 0.08) {
        int i = randint(0, c->num_gates - 1);
        int j = randint(0, c->num_gates - 1);
        // Ensure different gates
        while (j == i && c->num_gates > 1) {
            j = randint(0, c->num_gates - 1);
        }
        circuit_swap_gates(c, i, j);
        return;
    }
    
    // Original mutation logic (remaining 92%)
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

// Count bits across multiple chunks
int count_set_bits_vec(const BitVec *v, int num_chunks) {
    int total = 0;
    for (int c = 0; c < num_chunks; c++) {
        total += count_set_bits(v->chunks[c]);
    }
    return total;
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

    char tt_copy[8192]; // Increased buffer for larger truth tables
    strncpy(tt_copy, tt_str, sizeof(tt_copy) - 1);
    tt_copy[sizeof(tt_copy) - 1] = '\0';
    char *saveptr;
    // NEW: Includes Space, Newline, Carriage Return, and Tab
    char *row_str = strtok_r(tt_copy, " \n\r\t", &saveptr);
    
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

    // Calculate number of chunks needed
    int total_rows = 1 << s->num_inputs;
    s->num_chunks = (total_rows + 63) / 64;
    if (s->num_chunks > MAX_CHUNKS) s->num_chunks = MAX_CHUNKS;

    // Re-parse
    strncpy(tt_copy, tt_str, sizeof(tt_copy) - 1);
    tt_copy[sizeof(tt_copy) - 1] = '\0';
    row_str = strtok_r(tt_copy, " ", &saveptr);

    while (row_str) {
        char *colon = strchr(row_str, ':');
        if (colon) {
            *colon = '\0';
            char *lhs = row_str;
            char *rhs = colon + 1;
            
            // UPGRADE: Support up to MAX_CHUNKS * 64 rows
            if (r_idx < MAX_CHUNKS * 64) {
                int chunk_idx = r_idx / 64;
                int bit_idx = r_idx % 64;
                
                for (int i = 0; i < s->num_inputs; i++) {
                    if (lhs[i] == '1') {
                        s->inputs_bin[i].chunks[chunk_idx] |= (1ULL << bit_idx);
                    }
                }
                for (int i = 0; i < s->num_outputs; i++) {
                    if (rhs[i] != 'X') {
                        s->masks_bin[i].chunks[chunk_idx] |= (1ULL << bit_idx);
                        if (rhs[i] == '1') {
                            s->targets_bin[i].chunks[chunk_idx] |= (1ULL << bit_idx);
                        }
                    }
                }
            }
            r_idx++;
        }
        row_str = strtok_r(NULL, " ", &saveptr);
    }

    s->max_score = 0;
    for (int i = 0; i < s->num_outputs; i++) {
        s->max_score += count_set_bits_vec(&s->masks_bin[i], s->num_chunks);
    }
}

int solver_score(GrowthSolver *s, Circuit *c) {
    BitVec outputs[MAX_OUTPUTS];
    circuit_evaluate(c, s->inputs_bin, outputs, s->num_chunks);
    int total = 0;
    for (int i = 0; i < s->num_outputs; i++) {
        for (int ch = 0; ch < s->num_chunks; ch++) {
            uint64_t matches = ~(outputs[i].chunks[ch] ^ s->targets_bin[i].chunks[ch]) 
                             & s->masks_bin[i].chunks[ch];
            total += count_set_bits(matches);
        }
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

            if (gate->op != OP_NOT) {
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
    printf("Strategy: Deep Opt with Termination Plateau (Multi-Word Engine, %d chunks)", s->num_chunks);

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
                   gen/100000, parent.num_gates, parent_score, s->max_score, 
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
        if (g->op != OP_NOT) {
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
    printf("============================================================");
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
    solver_init(&solver, tt, "NAND OR XOR AND");

    Circuit solution;
    bool solved = solver_solve(&solver, MAX_SOLVE_GATES, STALL_LIMIT, &solution);

    if (solved) {
        render_circuit(&solution, &solver);
    }

    return 0;
}
