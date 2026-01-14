#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <time.h>
#include <stdbool.h>

/* --- Configuration & Constants --- */
#define MAX_INPUTS 10          // Upgraded to allow up to 10 inputs (1024 rows)
#define MAX_OUTPUTS 16
#define MAX_GATES 300         // Increased capacity
#define MAX_WIRES (MAX_INPUTS + MAX_GATES)
// pls set MAX_CHUNKS to: (2^MAX_INPUTS)/64
#define MAX_CHUNKS 16          // 1024 bits / 64 bits per chunk = 16 chunks


/* --- DLS2 Export Configuration --- */
#define DLS2_CHIP_NAME "SYNTH"
#define DLS2_HEIGHT_MULTIPLIER 0.35f
#define DLS2_BASE_HEIGHT 0.5f


// --- TUNING PARAMETERS ---
#define STALL_LIMIT 10000
#define MAX_SOLVE_GATES 300
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
void run_gate_vec(int op_id, const BitVec *val_a, const BitVec *val_b, BitVec *result, int num_chunks) {
    for (int c = 0; c < num_chunks; c++) {
        switch (op_id) {
            case OP_AND:  result->chunks[c] = val_a->chunks[c] & val_b->chunks[c]; break;
            case OP_OR:   result->chunks[c] = val_a->chunks[c] | val_b->chunks[c]; break;
            case OP_NAND: result->chunks[c] = ~(val_a->chunks[c] & val_b->chunks[c]); break;
            case OP_NOR:  result->chunks[c] = ~(val_a->chunks[c] | val_b->chunks[c]); break;
            case OP_XOR:  result->chunks[c] = val_a->chunks[c] ^ val_b->chunks[c]; break;
            case OP_NOT:  result->chunks[c] = ~val_a->chunks[c]; break;
            case OP_XNOR: result->chunks[c] = ~(val_a->chunks[c] ^ val_b->chunks[c]); break;
            default:      result->chunks[c] = 0; break;
        }
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







/* --- Project Description Updater --- */
#define PROJECT_DESC_PATH "../ProjectDescription.json"

// Helper: Get current timestamp in ISO 8601 format
static void get_iso_timestamp(char* buffer, size_t size) {
    time_t now = time(NULL);
    struct tm* tm_info = localtime(&now);
    strftime(buffer, size, "%Y-%m-%dT%H:%M:%S.000+00:00", tm_info);
}

// Helper: Insert string at position, shifting rest of content
static bool insert_at(char* json, size_t pos, const char* insert, size_t json_buf_size) {
    size_t json_len = strlen(json);
    size_t insert_len = strlen(insert);

    // Safety check: ensure we have room
    if (json_len + insert_len >= json_buf_size) {
        printf("Warning: Buffer too small for insertion\n");
        return false;
    }

    // Shift existing content
    memmove(json + pos + insert_len, json + pos, json_len - pos + 1);
    // Copy insert string
    memcpy(json + pos, insert, insert_len);
    return true;
}

// Helper: Check if a name exists within a specific range of the JSON
static bool name_exists_in_range(const char* start, const char* end, const char* name) {
    if (!start || !end || end <= start) return false;

    char search[256];
    snprintf(search, sizeof(search), "\"%s\"", name);

    const char* found = start;
    while ((found = strstr(found, search)) != NULL) {
        if (found < end) {
            return true;  // Found within range
        }
        break;  // Found but past our range
    }
    return false;
}

// Helper: Find the closing bracket of a JSON array, handling nested structures
static const char* find_array_end(const char* arr_start) {
    if (!arr_start || *arr_start != '[') return NULL;

    int depth = 1;
    const char* p = arr_start + 1;

    while (*p && depth > 0) {
        if (*p == '[' || *p == '{') depth++;
        else if (*p == ']' || *p == '}') depth--;
        if (depth > 0) p++;
    }

    return (depth == 0) ? p : NULL;
}

// Update ProjectDescription.json to include the new chip
bool update_project_description(const char* chip_name) {
    // Validate chip name
    if (!chip_name || strlen(chip_name) == 0) {
        printf("Warning: Invalid chip name\n");
        return false;
    }

    FILE* f = fopen(PROJECT_DESC_PATH, "r");
    if (!f) {
        printf("Warning: Could not open %s\n", PROJECT_DESC_PATH);
        printf("         Make sure you're running from the Chips folder.\n");
        return false;
    }

    // Read entire file
    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    fseek(f, 0, SEEK_SET);

    if (size <= 0) {
        printf("Warning: ProjectDescription.json is empty or unreadable\n");
        fclose(f);
        return false;
    }

    // Allocate with extra space for insertions
    size_t buf_size = size + 4096;
    char* json = (char*)malloc(buf_size);
    if (!json) {
        fclose(f);
        return false;
    }
    memset(json, 0, buf_size);
    fread(json, 1, size, f);
    fclose(f);

    bool modified = false;
    char insert_buf[512];

    printf("Updating ProjectDescription.json for chip '%s'...\n", chip_name);

    // --- 1. Update LastSaveTime ---
    char timestamp[64];
    get_iso_timestamp(timestamp, sizeof(timestamp));

    const char* last_save = strstr(json, "\"LastSaveTime\"");
    if (last_save) {
        const char* colon = strchr(last_save, ':');
        if (colon) {
            const char* quote1 = strchr(colon, '"');
            if (quote1) {
                const char* quote2 = strchr(quote1 + 1, '"');
                if (quote2) {
                    size_t start = quote1 - json + 1;
                    size_t old_len = quote2 - quote1 - 1;
                    size_t new_len = strlen(timestamp);

                    memmove((char*)quote1 + 1 + new_len, quote2, strlen(quote2) + 1);
                    memcpy((char*)quote1 + 1, timestamp, new_len);
                    modified = true;
                }
            }
        }
    }

    // --- 2. Check and add to AllCustomChipNames ---
    const char* all_chips_key = strstr(json, "\"AllCustomChipNames\"");
    if (all_chips_key) {
        const char* arr_start = strchr(all_chips_key, '[');
        const char* arr_end = arr_start ? find_array_end(arr_start) : NULL;

        if (arr_start && arr_end) {
            if (name_exists_in_range(arr_start, arr_end, chip_name)) {
                printf("  [SKIP] AllCustomChipNames: '%s' already exists\n", chip_name);
            } else {
                // Add the chip
                snprintf(insert_buf, sizeof(insert_buf), ",\n    \"%s\"", chip_name);
                size_t insert_pos = arr_end - json;
                if (insert_at(json, insert_pos, insert_buf, buf_size)) {
                    printf("  [ADD]  AllCustomChipNames: '%s'\n", chip_name);
                    modified = true;
                }
            }
        }
    } else {
        printf("  [WARN] AllCustomChipNames section not found\n");
    }

    // --- 3. Check and add to StarredList ---
    const char* starred_key = strstr(json, "\"StarredList\"");
    if (starred_key) {
        const char* arr_start = strchr(starred_key, '[');
        const char* arr_end = arr_start ? find_array_end(arr_start) : NULL;

        if (arr_start && arr_end) {
            // For StarredList, we need to search for "Name":"CHIPNAME"
            char name_search[256];
            snprintf(name_search, sizeof(name_search), "\"Name\":\"%s\"", chip_name);

            const char* found = strstr(arr_start, name_search);
            if (found && found < arr_end) {
                printf("  [SKIP] StarredList: '%s' already exists\n", chip_name);
            } else {
                snprintf(insert_buf, sizeof(insert_buf),
                         ",\n    {\n      \"Name\":\"%s\",\n      \"IsCollection\":false\n    }",
                         chip_name);
                size_t insert_pos = arr_end - json;
                if (insert_at(json, insert_pos, insert_buf, buf_size)) {
                    printf("  [ADD]  StarredList: '%s'\n", chip_name);
                    modified = true;
                }
            }
        }
    } else {
        printf("  [WARN] StarredList section not found\n");
    }

    // --- 4. Check and add to OTHER collection in ChipCollections ---
    // Need to re-find after previous insertions may have shifted positions
    const char* other_name = strstr(json, "\"Name\":\"OTHER\"");
    if (other_name) {
        // Search backwards for "Chips":[ that belongs to this collection
        const char* search_start = other_name - 500;
        if (search_start < json) search_start = json;

        const char* chips_key = NULL;
        const char* p = other_name;
        while (p > search_start) {
            if (strncmp(p, "\"Chips\":[", 9) == 0) {
                chips_key = p;
                break;
            }
            p--;
        }

        if (chips_key) {
            const char* chips_arr_start = strchr(chips_key, '[');
            const char* chips_arr_end = chips_arr_start ? strchr(chips_arr_start, ']') : NULL;

            if (chips_arr_start && chips_arr_end && chips_arr_end < other_name) {
                if (name_exists_in_range(chips_arr_start, chips_arr_end, chip_name)) {
                    printf("  [SKIP] OTHER collection: '%s' already exists\n", chip_name);
                } else {
                    snprintf(insert_buf, sizeof(insert_buf), ",\"%s\"", chip_name);
                    size_t insert_pos = chips_arr_end - json;
                    if (insert_at(json, insert_pos, insert_buf, buf_size)) {
                        printf("  [ADD]  OTHER collection: '%s'\n", chip_name);
                        modified = true;
                    }
                }
            }
        }
    } else {
        printf("  [WARN] OTHER collection not found in ChipCollections\n");
    }

    // --- Write back if modified ---
    if (modified) {
        f = fopen(PROJECT_DESC_PATH, "w");
        if (!f) {
            printf("Error: Could not write to %s\n", PROJECT_DESC_PATH);
            free(json);
            return false;
        }
        fputs(json, f);
        fclose(f);
        printf("ProjectDescription.json updated successfully!\n");
    } else {
        printf("No changes needed - chip '%s' fully registered\n", chip_name);
    }

    free(json);
    return true;
}







/* --- Dynamic Pin Mapping System --- */
typedef struct {
    int input_a;
    int input_b;    // -1 for NOT gate (single input)
    int output;
    bool loaded;    // Whether successfully loaded from file
} GatePinMapping;

// Mutable pin map - NAND is pre-initialized as the built-in default
static GatePinMapping DLS2_PIN_MAP[NUM_OPS] = {
    [OP_AND]  = {0, 0, 0, false},
    [OP_OR]   = {0, 0, 0, false},
    [OP_NAND] = {0, 1, 2, true},    // NAND is built-in, always available
    [OP_NOR]  = {0, 0, 0, false},
    [OP_XOR]  = {0, 0, 0, false},
    [OP_NOT]  = {0, -1, 0, false},
    [OP_XNOR] = {0, 0, 0, false}
};

// Filenames to search for each gate type (indexed by OP_*)
// NULL means skip loading (use default)
static const char* GATE_FILENAMES[NUM_OPS] = {
    "AND.json",    // OP_AND = 0
    "OR.json",     // OP_OR = 1
    NULL,          // OP_NAND = 2 (built-in, no file needed)
    "NOR.json",    // OP_NOR = 3
    "XOR.json",    // OP_XOR = 4
    "NOT.json",    // OP_NOT = 5
    "XNOR.json"    // OP_XNOR = 6
};

// Helper: Skip whitespace in string
static const char* skip_whitespace(const char* p) {
    while (*p && (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r')) p++;
    return p;
}

// Helper: Find a key in JSON and return pointer to its value
static const char* find_json_key(const char* json, const char* key) {
    char search[64];
    snprintf(search, sizeof(search), "\"%s\"", key);
    const char* pos = strstr(json, search);
    if (!pos) return NULL;
    pos += strlen(search);
    pos = skip_whitespace(pos);
    if (*pos == ':') pos++;
    return skip_whitespace(pos);
}

// Helper: Extract integer at current position
static int extract_int(const char* p) {
    while (*p && (*p < '0' || *p > '9') && *p != '-') p++;
    return atoi(p);
}

// Parse a gate JSON file and extract pin IDs
static bool load_gate_pins_from_file(int op_index, const char* filename) {
    FILE* f = fopen(filename, "r");
    if (!f) return false;

    // Read entire file into memory
    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    fseek(f, 0, SEEK_SET);

    char* json = (char*)malloc(size + 1);
    if (!json) {
        fclose(f);
        return false;
    }
    fread(json, 1, size, f);
    json[size] = '\0';
    fclose(f);

    bool success = false;

    // Find InputPins section
    const char* input_section = find_json_key(json, "InputPins");
    if (!input_section) goto cleanup;

    // Find OutputPins section (to know where InputPins ends)
    const char* output_section = find_json_key(json, "OutputPins");
    if (!output_section) goto cleanup;

    // Extract first input pin ID (input_a)
    const char* id_pos = strstr(input_section, "\"ID\"");
    if (!id_pos || id_pos > output_section) goto cleanup;
    id_pos = find_json_key(id_pos, "ID");
    if (!id_pos) goto cleanup;
    DLS2_PIN_MAP[op_index].input_a = extract_int(id_pos);

    // For 2-input gates, extract second input pin ID (input_b)
    if (op_index != OP_NOT) {
        const char* second_id = strstr(id_pos + 1, "\"ID\"");
        if (second_id && second_id < output_section) {
            second_id = find_json_key(second_id, "ID");
            if (second_id) {
                DLS2_PIN_MAP[op_index].input_b = extract_int(second_id);
            }
        }
    } else {
        DLS2_PIN_MAP[op_index].input_b = -1;
    }

    // Extract output pin ID
    id_pos = strstr(output_section, "\"ID\"");
    if (!id_pos) goto cleanup;
    id_pos = find_json_key(id_pos, "ID");
    if (!id_pos) goto cleanup;
    DLS2_PIN_MAP[op_index].output = extract_int(id_pos);

    DLS2_PIN_MAP[op_index].loaded = true;
    success = true;

    cleanup:
    free(json);
    return success;
}

// Load all gate pin mappings from JSON files
void load_dls2_pin_mappings(void) {
    printf("=== Loading DLS2 Gate Pin Mappings ===\n");

    // NAND is always available (built-in)
    printf("  [OK] NAND  : in_a=%d, in_b=%d, out=%d (built-in)\n",
           DLS2_PIN_MAP[OP_NAND].input_a,
           DLS2_PIN_MAP[OP_NAND].input_b,
           DLS2_PIN_MAP[OP_NAND].output);

    int loaded_count = 1;  // NAND already counted

    for (int i = 0; i < NUM_OPS; i++) {
        // Skip NAND (already loaded as built-in)
        if (i == OP_NAND || GATE_FILENAMES[i] == NULL) continue;

        if (load_gate_pins_from_file(i, GATE_FILENAMES[i])) {
            loaded_count++;
            if (i == OP_NOT) {
                printf("  [OK] %-5s : in=%d, out=%d\n",
                       OP_NAMES[i],
                       DLS2_PIN_MAP[i].input_a,
                       DLS2_PIN_MAP[i].output);
            } else {
                printf("  [OK] %-5s : in_a=%d, in_b=%d, out=%d\n",
                       OP_NAMES[i],
                       DLS2_PIN_MAP[i].input_a,
                       DLS2_PIN_MAP[i].input_b,
                       DLS2_PIN_MAP[i].output);
            }
        } else {
            printf("  [--] %-5s : %s not found\n", OP_NAMES[i], GATE_FILENAMES[i]);
        }
    }

    printf("Loaded %d/%d gate definitions\n", loaded_count, NUM_OPS);
    printf("=======================================\n\n");
}

// Check if a specific gate type is available
bool is_gate_available(int op) {
    return (op >= 0 && op < NUM_OPS && DLS2_PIN_MAP[op].loaded);
}

/* ============================================================
 * DLS2 GRID-BASED MANHATTAN ROUTING SYSTEM
 * Version 3.0 - Clean Alleyway Routing
 * ============================================================ */

typedef struct {
    // === PHYSICAL CONSTANTS ===
    float gate_width;           // Physical width of a gate hitbox
    float gate_height;          // Physical height of a gate hitbox

    // === GRID SPACING ===
    float column_spacing;       // Distance between column centers (must be > gate_width + alley_width)
    float alley_width;          // Width reserved for wire routing between columns
    float min_gate_v_spacing;   // Minimum vertical space between gate centers
    float lane_spacing;         // Space between parallel wires in an alley

    // === PIN LAYOUT ===
    float input_v_spacing;
    float output_v_spacing;
    float pin_margin_x;         // Horizontal distance from first/last column to pins

    // === CHIP SIZING ===
    float chip_padding;         // Padding around content for chip boundary

    // === APPEARANCE ===
    float chip_color_r;
    float chip_color_g;
    float chip_color_b;
} LayoutConfig;

static LayoutConfig get_default_layout_config(void) {
    return (LayoutConfig){
        .gate_width = 0.85f,
        .gate_height = 0.55f,

        .column_spacing = 5.0f,     // Increased from 3.5 (Wider alleys)
        .alley_width = 2.8f,        // Increased from 1.8 (More lanes)
        .min_gate_v_spacing = 3.0f,  // Increased from 1.4 (CRITICAL: Creates space for wires)
        .lane_spacing = 0.25f,      // Increased from 0.18 (Prevents wire-on-wire blending)

        .input_v_spacing = 1.5f,
        .output_v_spacing = 1.5f,
        .pin_margin_x = 4.5f,

        .chip_padding = 1.0f,

        .chip_color_r = 0.22f, .chip_color_g = 0.32f, .chip_color_b = 0.48f
    };
}

/* ============================================================
 * HELPER FUNCTIONS
 * ============================================================ */
static float absf(float x) { return (x < 0) ? -x : x; }
static float minf(float a, float b) { return (a < b) ? a : b; }
static float maxf(float a, float b) { return (a > b) ? a : b; }

static void calculate_gate_depths(Circuit *c, int *depths) {
    for (int i = 0; i < c->num_gates; i++) {
        depths[i] = 0;
        Gate *g = &c->gates[i];

        if (g->src_a >= c->num_inputs) {
            int d = depths[g->src_a - c->num_inputs] + 1;
            if (d > depths[i]) depths[i] = d;
        }
        if (g->op != OP_NOT && g->src_b >= c->num_inputs) {
            int d = depths[g->src_b - c->num_inputs] + 1;
            if (d > depths[i]) depths[i] = d;
        }
    }
}

static int get_max_depth(int *depths, int num_gates) {
    int max_d = 0;
    for (int i = 0; i < num_gates; i++) {
        if (depths[i] > max_d) max_d = depths[i];
    }
    return max_d;
}

/* ============================================================
 * GRID POSITION CALCULATOR
 * Computes column centers and alley centers
 * ============================================================ */
typedef struct {
    float column_x[100];    // X center of each depth column
    float alley_x[101];     // X center of each routing alley
    int alley_lane[101];    // Next available lane in each alley
    float input_x;
    float output_x;
    int num_columns;
    int num_alleys;
} GridLayout;

static void calculate_grid_layout(int max_depth, LayoutConfig *cfg, GridLayout *grid) {
    grid->num_columns = max_depth + 1;
    grid->num_alleys = max_depth + 2;  // One more alley than columns

    // Center the grid around x=0
    float total_width = max_depth * cfg->column_spacing;
    float start_x = -total_width / 2.0f;

    // Column centers
    for (int d = 0; d <= max_depth; d++) {
        grid->column_x[d] = start_x + d * cfg->column_spacing;
    }

    // Alley centers (between columns)
    // Alley[0] = left of column[0] (for inputs)
    // Alley[d] = between column[d-1] and column[d]
    // Alley[num_columns] = right of last column (for outputs)

    float half_col_spacing = cfg->column_spacing / 2.0f;

    grid->alley_x[0] = grid->column_x[0] - half_col_spacing;
    for (int d = 1; d <= max_depth; d++) {
        grid->alley_x[d] = (grid->column_x[d-1] + grid->column_x[d]) / 2.0f;
    }
    grid->alley_x[max_depth + 1] = grid->column_x[max_depth] + half_col_spacing;

    // Initialize lane counters
    for (int i = 0; i < grid->num_alleys; i++) {
        grid->alley_lane[i] = 0;
    }

    // Pin positions
    grid->input_x = grid->alley_x[0] - cfg->pin_margin_x;
    grid->output_x = grid->alley_x[grid->num_alleys - 1] + cfg->pin_margin_x;
}

/* ============================================================
 * COLLISION-FREE GATE Y POSITIONING
 * Aligns with Source A, then nudges to avoid overlaps
 * ============================================================ */
static void position_gates_with_collision_avoidance(
    Circuit *c,
    int *depths,
    int max_depth,
    float *input_y,
    float *gate_x,
    float *gate_y,
    GridLayout *grid,
    LayoutConfig *cfg
) {
    // Track Y positions already used at each depth
    float used_y[100][50];  // [depth][slot] = y position
    int used_count[100] = {0};

    // Process gates in order (they should already be topologically sorted)
    for (int i = 0; i < c->num_gates; i++) {
        Gate *g = &c->gates[i];
        int d = depths[i];

        // X position: column center
        gate_x[i] = grid->column_x[d];

        // Y position: align with Source A (primary input)
        float target_y;
        if (g->src_a < c->num_inputs) {
            target_y = input_y[g->src_a];
        } else {
            target_y = gate_y[g->src_a - c->num_inputs];
        }

        // Check for collisions with existing gates at this depth
        bool collision = true;
        float best_y = target_y;
        float search_offset = 0.0f;
        int direction = 1;  // Alternate up/down

        while (collision) {
            collision = false;
            float test_y = target_y + search_offset * direction;

            for (int j = 0; j < used_count[d]; j++) {
                if (absf(test_y - used_y[d][j]) < cfg->min_gate_v_spacing) {
                    collision = true;
                    break;
                }
            }

            if (!collision) {
                best_y = test_y;
            } else {
                // Try the other direction, then increase offset
                if (direction == 1) {
                    direction = -1;
                } else {
                    direction = 1;
                    search_offset += cfg->min_gate_v_spacing;
                }

                // Safety limit
                if (search_offset > 20.0f) {
                    best_y = target_y + used_count[d] * cfg->min_gate_v_spacing;
                    break;
                }
            }
        }

        gate_y[i] = best_y;

        // Record this position
        if (used_count[d] < 50) {
            used_y[d][used_count[d]++] = best_y;
        }
    }
}

/* ============================================================
 * ALLEYWAY WIRE ROUTING
 * Routes wires through designated alleys between columns
 * ============================================================ */
static void write_wire_alleyway(
    FILE *f,
    int src_pin, int src_owner,
    int tgt_pin, int tgt_owner,
    float src_x, float src_y,
    float tgt_x, float tgt_y,
    int src_depth,      // -1 for input, depth for gate
    int tgt_depth,      // depth for gate, -2 for output
    GridLayout *grid,
    LayoutConfig *cfg,
    bool is_last
) {
    fprintf(f, "    {\n");
    fprintf(f, "      \"SourcePinAddress\":{\"PinID\":%d,\"PinOwnerID\":%d},\n", src_pin, src_owner);
    fprintf(f, "      \"TargetPinAddress\":{\"PinID\":%d,\"PinOwnerID\":%d},\n", tgt_pin, tgt_owner);
    fprintf(f, "      \"ConnectionType\":0,\n");
    fprintf(f, "      \"ConnectedWireIndex\":-1,\n");
    fprintf(f, "      \"ConnectedWireSegmentIndex\":-1,\n");

    float dy = tgt_y - src_y;
    float dx = tgt_x - src_x;

    // Direct connection if very close and nearly horizontal
    if (absf(dy) < 0.3f && absf(dx) < 1.5f) {
        fprintf(f, "      \"Points\":[{\"x\":0.0,\"y\":0.0},{\"x\":0.0,\"y\":0.0}]\n");
    } else {
        // Determine which alley to use for the vertical segment
        // Use the alley immediately LEFT of the target
        int alley_idx;

        if (src_depth == -1) {
            // If it's an INPUT, turn vertical in the first alley (Alley 0)
            // Alley 0 is the space BEFORE the first column of gates.
            alley_idx = 0;
        }
        else if (tgt_depth == -2) {
            // If it's an OUTPUT, turn vertical in the last alley
            alley_idx = grid->num_alleys - 1;
        }
        else {
            // If it's gate-to-gate, pick an alley in the middle
            // This spreads wires out and prevents them from all bunching in one alley
            alley_idx = (src_depth + tgt_depth + 1) / 2;
        }

        // Clamp to valid range
        if (alley_idx < 0) alley_idx = 0;
        if (alley_idx >= grid->num_alleys) alley_idx = grid->num_alleys - 1;

        // Get lane within alley
        int lane = grid->alley_lane[alley_idx]++;

        // Calculate lane X offset (center lane at 0, spread outward)
        float lane_offset = (lane % 2 == 0)
        ? (float)(lane / 2) * cfg->lane_spacing
        : -(float)((lane + 1) / 2) * cfg->lane_spacing;

        float alley_center = grid->alley_x[alley_idx];
        float route_x = alley_center + lane_offset;

        // Ensure route_x doesn't overlap with gate columns
        // (stay within alley bounds)
        float half_alley = cfg->alley_width / 2.0f - 0.1f;
        route_x = maxf(alley_center - half_alley, minf(alley_center + half_alley, route_x));

        // L-shaped route: horizontal to alley, vertical in alley, horizontal to target
        fprintf(f, "      \"Points\":[");
        fprintf(f, "{\"x\":0.0,\"y\":0.0},");
        fprintf(f, "{\"x\":%.4f,\"y\":%.4f},", route_x, src_y);
        fprintf(f, "{\"x\":%.4f,\"y\":%.4f},", route_x, tgt_y);
        fprintf(f, "{\"x\":0.0,\"y\":0.0}]\n");
    }

    fprintf(f, "    }%s\n", is_last ? "" : ",");
}

/* ============================================================
 * BOUNDING BOX TRACKER
 * Tracks min/max coordinates for dynamic chip sizing
 * ============================================================ */
typedef struct {
    float min_x, max_x;
    float min_y, max_y;
    bool initialized;
} BoundingBox;

static void bbox_init(BoundingBox *bb) {
    bb->min_x = bb->min_y = 1e9f;
    bb->max_x = bb->max_y = -1e9f;
    bb->initialized = false;
}

static void bbox_add_point(BoundingBox *bb, float x, float y, float margin) {
    bb->min_x = minf(bb->min_x, x - margin);
    bb->max_x = maxf(bb->max_x, x + margin);
    bb->min_y = minf(bb->min_y, y - margin);
    bb->max_y = maxf(bb->max_y, y + margin);
    bb->initialized = true;
}

/* ============================================================
 * MAIN EXPORT FUNCTION
 * ============================================================ */
void render_dls2_json(Circuit *c, GrowthSolver *s) {
    LayoutConfig cfg = get_default_layout_config();

    // Validate pin mappings
    for (int i = 0; i < c->num_gates; i++) {
        if (!DLS2_PIN_MAP[c->gates[i].op].loaded) {
            printf("Error: Gate %s (#%d) has no pin mapping!\n",
                   OP_NAMES[c->gates[i].op], i);
            return;
        }
    }

    char filename[256];
    snprintf(filename, sizeof(filename), "%s.json", DLS2_CHIP_NAME);

    FILE *f = fopen(filename, "w");
    if (!f) {
        printf("Error: Cannot open %s\n", filename);
        return;
    }

    // === CALCULATE DEPTHS ===
    int depths[MAX_GATES];
    calculate_gate_depths(c, depths);
    int max_depth = get_max_depth(depths, c->num_gates);
    if (c->num_gates == 0) max_depth = 0;

    // === SETUP GRID LAYOUT ===
    GridLayout grid;
    calculate_grid_layout(max_depth, &cfg, &grid);

    // === ALLOCATE IDs ===
    int input_ids[MAX_INPUTS];
    int output_ids[MAX_OUTPUTS];
    int gate_ids[MAX_GATES];

    for (int i = 0; i < c->num_inputs; i++)  input_ids[i]  = 1000 + i;
    for (int i = 0; i < c->num_outputs; i++) output_ids[i] = 2000 + i;
    for (int i = 0; i < c->num_gates; i++)   gate_ids[i]   = 3000 + i;

    // === INPUT Y POSITIONS (centered) ===
    float input_y[MAX_INPUTS];
    float total_input_height = (c->num_inputs - 1) * cfg.input_v_spacing;
    float input_start_y = total_input_height / 2.0f;

    for (int i = 0; i < c->num_inputs; i++) {
        input_y[i] = input_start_y - i * cfg.input_v_spacing;
    }

    // === GATE POSITIONS (with collision avoidance) ===
    float gate_x[MAX_GATES];
    float gate_y[MAX_GATES];

    position_gates_with_collision_avoidance(
        c, depths, max_depth, input_y,
        gate_x, gate_y, &grid, &cfg
    );

    // === OUTPUT Y POSITIONS (centered) ===
    float output_y[MAX_OUTPUTS];
    float total_output_height = (c->num_outputs - 1) * cfg.output_v_spacing;
    float output_start_y = total_output_height / 2.0f;

    for (int i = 0; i < c->num_outputs; i++) {
        output_y[i] = output_start_y - i * cfg.output_v_spacing;
    }

    // === CALCULATE BOUNDING BOX ===
    BoundingBox bbox;
    bbox_init(&bbox);

    // Add inputs
    for (int i = 0; i < c->num_inputs; i++) {
        bbox_add_point(&bbox, grid.input_x, input_y[i], 0.3f);
    }

    // Add outputs
    for (int i = 0; i < c->num_outputs; i++) {
        bbox_add_point(&bbox, grid.output_x, output_y[i], 0.3f);
    }

    // Add gates (with physical size)
    for (int i = 0; i < c->num_gates; i++) {
        bbox_add_point(&bbox, gate_x[i], gate_y[i], cfg.gate_width / 2.0f + 0.2f);
    }

    // Add alleys (for wire routing space)
    for (int i = 0; i < grid.num_alleys; i++) {
        float alley_y_extent = maxf(absf(bbox.min_y), absf(bbox.max_y));
        bbox_add_point(&bbox, grid.alley_x[i], alley_y_extent, 0.1f);
        bbox_add_point(&bbox, grid.alley_x[i], -alley_y_extent, 0.1f);
    }

    // === CALCULATE DYNAMIC CHIP SIZE ===
    float content_width = bbox.max_x - bbox.min_x;
    float content_height = bbox.max_y - bbox.min_y;

    // Chip size formula: content dimensions mapped to chip scale
    // DLS2 uses a specific ratio. The "Size" field is roughly the visual size.
    float chip_width = (content_width / 12.0f) + cfg.chip_padding;
    float chip_height = (content_height / 8.0f) + cfg.chip_padding;

    // Clamp to reasonable bounds
    chip_width = maxf(0.5f, minf(chip_width, 4.0f));
    chip_height = maxf(0.5f, minf(chip_height, 4.0f));

    // === JSON OUTPUT ===
    fprintf(f, "{\n");
    fprintf(f, "  \"DLSVersion\": \"2.1.6\",\n");
    fprintf(f, "  \"Name\": \"%s\",\n", DLS2_CHIP_NAME);
    fprintf(f, "  \"NameLocation\": 0,\n");
    fprintf(f, "  \"ChipType\": 0,\n");
    fprintf(f, "  \"Size\": {\"x\": %.3f, \"y\": %.3f},\n", chip_width, chip_height);
    fprintf(f, "  \"Colour\": {\"r\": %.3f, \"g\": %.3f, \"b\": %.3f, \"a\": 1},\n",
            cfg.chip_color_r, cfg.chip_color_g, cfg.chip_color_b);

    // --- InputPins ---
    fprintf(f, "  \"InputPins\":[\n");
    for (int i = 0; i < c->num_inputs; i++) {
        fprintf(f, "    {\n");
        fprintf(f, "      \"Name\":\"IN%d\",\n", i);
        fprintf(f, "      \"ID\":%d,\n", input_ids[i]);
        fprintf(f, "      \"Position\":{\"x\":%.4f,\"y\":%.4f},\n", grid.input_x, input_y[i]);
        fprintf(f, "      \"BitCount\":1,\n");
        fprintf(f, "      \"Colour\":0,\n");
        fprintf(f, "      \"ValueDisplayMode\":0\n");
        fprintf(f, "    }%s\n", (i < c->num_inputs - 1) ? "," : "");
    }
    fprintf(f, "  ],\n");

    // --- OutputPins ---
    fprintf(f, "  \"OutputPins\":[\n");
    for (int i = 0; i < c->num_outputs; i++) {
        fprintf(f, "    {\n");
        fprintf(f, "      \"Name\":\"OUT%d\",\n", i);
        fprintf(f, "      \"ID\":%d,\n", output_ids[i]);
        fprintf(f, "      \"Position\":{\"x\":%.4f,\"y\":%.4f},\n", grid.output_x, output_y[i]);
        fprintf(f, "      \"BitCount\":1,\n");
        fprintf(f, "      \"Colour\":0,\n");
        fprintf(f, "      \"ValueDisplayMode\":0\n");
        fprintf(f, "    }%s\n", (i < c->num_outputs - 1) ? "," : "");
    }
    fprintf(f, "  ],\n");

    // --- SubChips (Gates) ---
    fprintf(f, "  \"SubChips\":[\n");
    for (int i = 0; i < c->num_gates; i++) {
        Gate *g = &c->gates[i];
        fprintf(f, "    {\n");
        fprintf(f, "      \"Name\":\"%s\",\n", OP_NAMES[g->op]);
        fprintf(f, "      \"ID\":%d,\n", gate_ids[i]);
        fprintf(f, "      \"Label\":\"\",\n");
        fprintf(f, "      \"Position\":{\"x\":%.4f,\"y\":%.4f},\n", gate_x[i], gate_y[i]);
        fprintf(f, "      \"OutputPinColourInfo\":[{\"PinColour\":0,\"PinID\":%d}],\n",
                DLS2_PIN_MAP[g->op].output);
        fprintf(f, "      \"InternalData\":null\n");
        fprintf(f, "    }%s\n", (i < c->num_gates - 1) ? "," : "");
    }
    fprintf(f, "  ],\n");

    // --- Count total wires ---
    int total_wires = c->num_outputs;
    for (int i = 0; i < c->num_gates; i++) {
        total_wires += (c->gates[i].op == OP_NOT) ? 1 : 2;
    }

    // --- Wires (with alleyway routing) ---
    fprintf(f, "  \"Wires\":[\n");
    int wire_num = 0;

    // Gate input wires
    for (int i = 0; i < c->num_gates; i++) {
        Gate *g = &c->gates[i];
        int tgt_depth = depths[i];

        // Wire A (primary input)
        int src_a = g->src_a;
        float sx_a, sy_a;
        int pin_a, owner_a;
        int src_depth_a;

        if (src_a < c->num_inputs) {
            sx_a = grid.input_x;
            sy_a = input_y[src_a];
            pin_a = 0;
            owner_a = input_ids[src_a];
            src_depth_a = -1;  // Input marker
        } else {
            int idx = src_a - c->num_inputs;
            sx_a = gate_x[idx];
            sy_a = gate_y[idx];
            pin_a = DLS2_PIN_MAP[c->gates[idx].op].output;
            owner_a = gate_ids[idx];
            src_depth_a = depths[idx];
        }

        wire_num++;
        write_wire_alleyway(f, pin_a, owner_a,
                            DLS2_PIN_MAP[g->op].input_a, gate_ids[i],
                            sx_a, sy_a, gate_x[i], gate_y[i],
                            src_depth_a, tgt_depth,
                            &grid, &cfg,
                            wire_num == total_wires);

        // Wire B (secondary input, if not NOT gate)
        if (g->op != OP_NOT) {
            int src_b = g->src_b;
            float sx_b, sy_b;
            int pin_b, owner_b;
            int src_depth_b;

            if (src_b < c->num_inputs) {
                sx_b = grid.input_x;
                sy_b = input_y[src_b];
                pin_b = 0;
                owner_b = input_ids[src_b];
                src_depth_b = -1;
            } else {
                int idx = src_b - c->num_inputs;
                sx_b = gate_x[idx];
                sy_b = gate_y[idx];
                pin_b = DLS2_PIN_MAP[c->gates[idx].op].output;
                owner_b = gate_ids[idx];
                src_depth_b = depths[idx];
            }

            wire_num++;
            write_wire_alleyway(f, pin_b, owner_b,
                                DLS2_PIN_MAP[g->op].input_b, gate_ids[i],
                                sx_b, sy_b, gate_x[i], gate_y[i],
                                src_depth_b, tgt_depth,
                                &grid, &cfg,
                                wire_num == total_wires);
        }
    }

    // Output wires
    for (int i = 0; i < c->num_outputs; i++) {
        int src = c->output_map[i];
        float sx, sy;
        int pin, owner;
        int src_depth;

        if (src < c->num_inputs) {
            sx = grid.input_x;
            sy = input_y[src];
            pin = 0;
            owner = input_ids[src];
            src_depth = -1;
        } else {
            int idx = src - c->num_inputs;
            sx = gate_x[idx];
            sy = gate_y[idx];
            pin = DLS2_PIN_MAP[c->gates[idx].op].output;
            owner = gate_ids[idx];
            src_depth = depths[idx];
        }

        wire_num++;
        write_wire_alleyway(f, pin, owner, 0, output_ids[i],
                            sx, sy, grid.output_x, output_y[i],
                            src_depth, -2,  // -2 = output marker
                            &grid, &cfg,
                            wire_num == total_wires);
    }

    fprintf(f, "  ],\n");
    fprintf(f, "  \"Displays\": null\n");
    fprintf(f, "}\n");

    fclose(f);

    // === PRINT SUMMARY ===
    printf("\n");
    printf("╔═══════════════════════════════════════════════════╗\n");
    printf("║       DLS2 GRID MANHATTAN ROUTING EXPORT          ║\n");
    printf("╠═══════════════════════════════════════════════════╣\n");
    printf("║  File: %-41s ║\n", filename);
    printf("║  Chip Size: %.2f x %.2f                           ║\n", chip_width, chip_height);
    printf("║  Gates: %-3d    Depths: %-3d    Wires: %-3d          ║\n",
           c->num_gates, max_depth + 1, total_wires);
    printf("║  Columns: %-2d   Alleys: %-2d                         ║\n",
           grid.num_columns, grid.num_alleys);
    printf("║  Bounds: X[%.1f, %.1f]  Y[%.1f, %.1f]             ║\n",
           bbox.min_x, bbox.max_x, bbox.min_y, bbox.max_y);
    printf("╚═══════════════════════════════════════════════════╝\n");

    update_project_description(DLS2_CHIP_NAME);
}

int main() {
    seed_rng(time(NULL));
    load_dls2_pin_mappings();
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
        render_dls2_json(&solution, &solver);; // New JSON export
    }

    return 0;
}
