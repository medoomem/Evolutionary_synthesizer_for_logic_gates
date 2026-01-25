#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <math.h>

/* ============================================================
 * CONFIGURATION & CONSTANTS
 * ============================================================ */

#define MAX_NODES 100000      
#define HASH_SIZE 65536       
#define MAX_GATES 50000       
#define MAX_EVOL_GATES 1000     
#define MAX_CHUNKS 16
#define MAX_WIRES (64 + MAX_EVOL_GATES)
#define MAX_INPUTS 16
#define MAX_OUTPUTS 64

#define STALL_LIMIT 50000              
#define OPTIMIZATION_PLATEAU_LIMIT 300000 
#define TERMINATION_PLATEAU 2000000
#define KICK_THRESHOLD 200000
#define KICK_RECOVERY_LIMIT 50000
#define GLOBAL_MERGE_INTERVAL 300000
#define GLOBAL_MERGE_CHECKS 10000

#define CONST_ZERO -1
#define CONST_ONE -2

/* ============================================================
 * UNIFIED OP CODES (EVO_* naming convention)
 * ============================================================ */

#define EVO_AND   0
#define EVO_OR    1
#define EVO_XOR   2
#define EVO_NOT   3
#define EVO_NAND  4
#define EVO_NOR   5
#define EVO_XNOR  6
#define EVO_NUM_OPS 7

const char* OP_NAMES[] = { "AND", "OR", "XOR", "NOT", "NAND", "NOR", "XNOR" };

/* ============================================================
 * DLS2 EXPORT CONFIGURATION
 * ============================================================ */

#define DLS2_CHIP_NAME "SYNTH"
#define DLS2_HEIGHT_MULTIPLIER 0.35f
#define DLS2_BASE_HEIGHT 0.5f
#define PROJECT_DESC_PATH "../ProjectDescription.json"

/* ============================================================
 * CORE DATA STRUCTURES
 * ============================================================ */

typedef struct { int num_bits, num_chunks; uint64_t *chunks; } TT;
typedef struct { uint64_t chunks[MAX_CHUNKS]; } BitVec;
typedef struct { int is_const, var_idx, low, high, value, id; } Node;
typedef struct { int op, src_a, src_b, id; } LogicGate;

typedef struct { 
    uint8_t op; 
    int16_t src_a, src_b; 
    bool alive; 
} Gate;

typedef struct {
    int num_inputs, num_outputs, num_gates;
    Gate gates[MAX_EVOL_GATES];
    int output_map[MAX_OUTPUTS];
    int allowed_ops[16];
    int allowed_ops_count;
    int dead_count;
} Circuit;

typedef struct HashEntry { TT *key; int node_idx; struct HashEntry *next; } HashEntry;
typedef struct GateHash { int op, a, b, id; struct GateHash *next; } GateHash;

typedef enum { G_INPUT, G_CONST, G_AND, G_OR, G_XOR, G_NOT } ROBDD_OpType;

/* ============================================================
 * DLS2 EXPORT STRUCTURES
 * ============================================================ */

typedef struct {
    int input_a;
    int input_b;
    int output;
    bool loaded;
} GatePinMapping;

typedef struct {
    float gate_width;
    float gate_height;
    float column_spacing;
    float alley_width;
    float min_gate_v_spacing;
    float lane_spacing;
    float input_v_spacing;
    float output_v_spacing;
    float pin_margin_x;
    float chip_padding;
    float chip_color_r;
    float chip_color_g;
    float chip_color_b;
} LayoutConfig;

typedef struct {
    float column_x[100];
    float alley_x[101];
    int alley_lane[101];
    float input_x;
    float output_x;
    int num_columns;
    int num_alleys;
} GridLayout;

typedef struct {
    float min_x, max_x;
    float min_y, max_y;
    bool initialized;
} BoundingBox;

/* ============================================================
 * GLOBAL VARIABLES
 * ============================================================ */

/* ROBDD globals */
Node nodes[MAX_NODES];
int node_count = 0;
int num_inputs = 0;
int num_outputs = 0;
int output_map_robdd[MAX_OUTPUTS]; 

LogicGate logic_gates[MAX_GATES];
int logic_gate_count = 0;
int node_to_gate[MAX_NODES]; 

HashEntry *memo[HASH_SIZE];
GateHash *gate_cache[HASH_SIZE];

BitVec g_targets[MAX_OUTPUTS], g_masks[MAX_OUTPUTS];
int g_num_chunks = 0;

int g_allowed_ops[EVO_NUM_OPS];
int g_allowed_ops_count = 0;

int var_order[MAX_INPUTS];
int var_level[MAX_INPUTS];

/* RNG state */
uint64_t rng_state;

/* DLS2 Pin Mappings - indexed by EVO_* op codes */
static GatePinMapping DLS2_PIN_MAP[EVO_NUM_OPS] = {
    [EVO_AND]  = {0, 0, 0, false},
    [EVO_OR]   = {0, 0, 0, false},
    [EVO_XOR]  = {0, 0, 0, false},
    [EVO_NOT]  = {0, -1, 0, false},
    [EVO_NAND] = {0, 1, 2, true},    /* NAND is built-in, always available */
    [EVO_NOR]  = {0, 0, 0, false},
    [EVO_XNOR] = {0, 0, 0, false}
};

/* Gate filenames for DLS2 - indexed by EVO_* op codes */
static const char* GATE_FILENAMES[EVO_NUM_OPS] = {
    "AND.json",    /* EVO_AND = 0 */
    "OR.json",     /* EVO_OR = 1 */
    "XOR.json",    /* EVO_XOR = 2 */
    "NOT.json",    /* EVO_NOT = 3 */
    NULL,          /* EVO_NAND = 4 (built-in) */
    "NOR.json",    /* EVO_NOR = 5 */
    "XNOR.json"    /* EVO_XNOR = 6 */
};

/* Configurable chip name */
char g_chip_name[64] = "SYNTH";

/* ============================================================
 * UTILITY FUNCTIONS
 * ============================================================ */

void seed_rng(uint64_t seed) { rng_state = seed ? seed : time(NULL); }

uint64_t fast_rand() { 
    uint64_t x = rng_state; 
    x ^= x << 13; 
    x ^= x >> 7; 
    x ^= x << 17; 
    return rng_state = x; 
}

int randint(int min, int max) { 
    return min + (fast_rand() % (max - min + 1)); 
}

double rand_double() { 
    return (double)fast_rand() / (double)UINT64_MAX; 
}

static float absf(float x) { return (x < 0) ? -x : x; }
static float minf(float a, float b) { return (a < b) ? a : b; }
static float maxf(float a, float b) { return (a > b) ? a : b; }

/* ============================================================
 * TRUTH TABLE FUNCTIONS
 * ============================================================ */

TT* create_tt(int bits) {
    TT *t = (TT*)malloc(sizeof(TT));
    t->num_bits = bits;
    t->num_chunks = (bits + 63) / 64;
    t->chunks = (uint64_t*)calloc(t->num_chunks, sizeof(uint64_t));
    return t;
}

void free_tt(TT *t) { 
    if(t) { 
        if(t->chunks) free(t->chunks); 
        free(t); 
    } 
}

void tt_set_bit(TT *t, int idx, int val) {
    int c = idx / 64, b = idx % 64;
    if (val) t->chunks[c] |= (1ULL << b); 
    else t->chunks[c] &= ~(1ULL << b);
}

int tt_get_bit(TT *t, int idx) { 
    return (t->chunks[idx / 64] >> (idx % 64)) & 1ULL; 
}

uint64_t hash_tt(TT *t) {
    uint64_t hash = 14695981039346656037ULL;
    for(int i=0; i<t->num_chunks; i++) {
        uint64_t k = t->chunks[i];
        for(int j=0; j<8; j++) { 
            hash ^= (k & 0xFF); 
            hash *= 1099511628211ULL; 
            k >>= 8; 
        }
    }
    return hash;
}

/* ============================================================
 * GATE EVALUATION FUNCTIONS
 * ============================================================ */

void robdd_run_gate(int op, const TT *a, const TT *b, TT *result) {
    for(int c = 0; c < result->num_chunks; c++) {
        switch(op) {
            case G_AND: result->chunks[c] = a->chunks[c] & b->chunks[c]; break;
            case G_OR:  result->chunks[c] = a->chunks[c] | b->chunks[c]; break;
            case G_XOR: result->chunks[c] = a->chunks[c] ^ b->chunks[c]; break;
            case G_NOT: result->chunks[c] = ~a->chunks[c]; break;
        }
    }
}

void evo_run_gate(int op, const BitVec *a, const BitVec *b, BitVec *result, int num_chunks) {
    for(int c = 0; c < num_chunks; c++) {
        switch(op) {
            case EVO_AND:  result->chunks[c] = a->chunks[c] & b->chunks[c]; break;
            case EVO_OR:   result->chunks[c] = a->chunks[c] | b->chunks[c]; break;
            case EVO_XOR:  result->chunks[c] = a->chunks[c] ^ b->chunks[c]; break;
            case EVO_NOT:  result->chunks[c] = ~a->chunks[c]; break;
            case EVO_NAND: result->chunks[c] = ~(a->chunks[c] & b->chunks[c]); break;
            case EVO_NOR:  result->chunks[c] = ~(a->chunks[c] | b->chunks[c]); break;
            case EVO_XNOR: result->chunks[c] = ~(a->chunks[c] ^ b->chunks[c]); break;
        }
    }
}

/* ============================================================
 * ALLOWED GATES PARSER
 * ============================================================ */

void parse_allowed_gates(const char *allowed_gates_str) {
    g_allowed_ops_count = 0;
    if (strcmp(allowed_gates_str, "ALL") == 0) {
        for(int i = 0; i < EVO_NUM_OPS; i++) 
            g_allowed_ops[g_allowed_ops_count++] = i;
    } else {
        char buf[256]; 
        strncpy(buf, allowed_gates_str, 255); 
        buf[255] = '\0';
        char *token = strtok(buf, " ");
        while(token) {
            for(int i = 0; i < EVO_NUM_OPS; i++)
                if (strcmp(token, OP_NAMES[i]) == 0) { 
                    g_allowed_ops[g_allowed_ops_count++] = i; 
                    break; 
                }
            token = strtok(NULL, " ");
        }
    }
    printf("[*] Allowed gates (%d): ", g_allowed_ops_count);
    for(int i = 0; i < g_allowed_ops_count; i++) 
        printf("%s ", OP_NAMES[g_allowed_ops[i]]);
    printf("\n");
}

/* ============================================================
 * ROBDD SYNTHESIS
 * ============================================================ */

void split_tt(TT *src, TT **low, TT **high) {
    int half_bits = src->num_bits / 2;
    *low = create_tt(half_bits); 
    *high = create_tt(half_bits);
    for(int i=0; i<half_bits; i++) {
        if(tt_get_bit(src, i)) tt_set_bit(*low, i, 1);
        if(tt_get_bit(src, i + half_bits)) tt_set_bit(*high, i, 1);
    }
}

int check_memo(TT *t) {
    uint64_t h = hash_tt(t); 
    int idx = h % HASH_SIZE;
    HashEntry *e = memo[idx];
    while(e) { 
        if(e->key->num_bits == t->num_bits && 
           memcmp(e->key->chunks, t->chunks, t->num_chunks * 8) == 0) 
            return e->node_idx; 
        e = e->next; 
    }
    return -1;
}

void add_memo(TT *t, int node_idx) {
    uint64_t h = hash_tt(t); 
    int idx = h % HASH_SIZE;
    HashEntry *e = (HashEntry*)malloc(sizeof(HashEntry));
    e->key = create_tt(t->num_bits); 
    memcpy(e->key->chunks, t->chunks, t->num_chunks * 8);
    e->node_idx = node_idx; 
    e->next = memo[idx]; 
    memo[idx] = e;
}

int create_node_const(int val) { 
    nodes[node_count] = (Node){1, -1, -1, -1, val, node_count}; 
    return node_count++; 
}

int create_node_mux(int var, int low, int high) { 
    if (low == high) return low; 
    nodes[node_count] = (Node){0, var, low, high, 0, node_count}; 
    return node_count++; 
}

int synthesize(TT *t, int input_depth) {
    int existing = check_memo(t); 
    if (existing != -1) return existing;
    
    bool all_zero = true, all_one = true;
    for(int i=0; i<t->num_bits; i++) { 
        if (tt_get_bit(t, i)) all_zero = false; 
        else all_one = false; 
        if(!all_zero && !all_one) break; 
    }
    if (all_zero) { 
        int id = create_node_const(0); 
        add_memo(t, id); 
        return id; 
    }
    if (all_one) { 
        int id = create_node_const(1); 
        add_memo(t, id); 
        return id; 
    }
    if (input_depth >= num_inputs) return create_node_const(0); 
    
    TT *low, *high; 
    split_tt(t, &low, &high);
    int node_low = synthesize(low, input_depth + 1);
    int node_high = synthesize(high, input_depth + 1);
    int res = create_node_mux(input_depth, node_low, node_high);
    add_memo(t, res); 
    free_tt(low); 
    free_tt(high); 
    return res;
}

void reset_gate_cache() { 
    memset(gate_cache, 0, sizeof(gate_cache)); 
}

int find_existing_gate(int op, int a, int b) {
    if ((op==G_AND || op==G_OR || op==G_XOR) && a>b) { 
        int t=a; a=b; b=t; 
    }
    uint64_t h = (op * 2654435761ULL) ^ (a * 2654435761ULL) ^ (b * 2654435761ULL);
    int idx = h % HASH_SIZE; 
    GateHash *e = gate_cache[idx];
    while(e) { 
        if(e->op == op && e->a == a && e->b == b) return e->id; 
        e = e->next; 
    }
    return -1;
}

void add_to_cache_manual(int op, int a, int b, int id) {
    if ((op==G_AND || op==G_OR || op==G_XOR) && a>b) { 
        int t=a; a=b; b=t; 
    }
    uint64_t h = (op * 2654435761ULL) ^ (a * 2654435761ULL) ^ (b * 2654435761ULL);
    int idx = h % HASH_SIZE; 
    GateHash *new_e = (GateHash*)malloc(sizeof(GateHash));
    new_e->op=op; new_e->a=a; new_e->b=b; new_e->id=id; 
    new_e->next = gate_cache[idx]; 
    gate_cache[idx] = new_e;
}

int find_or_add_gate(int op, int a, int b) {
    if ((op == G_AND || op == G_OR || op == G_XOR) && a > b) { 
        int t=a; a=b; b=t; 
    }
    if (op == G_AND && a == b) return a; 
    if (op == G_OR && a == b) return a;
    if (op == G_XOR && a == b) return find_or_add_gate(G_CONST, 0, 0);
    
    if (a < logic_gate_count && logic_gates[a].op == G_CONST) {
        int val_a = logic_gates[a].src_a;
        if (op == G_NOT) return find_or_add_gate(G_CONST, !val_a, 0);
        if (op == G_AND) return val_a ? b : a; 
        if (op == G_OR)  return val_a ? a : b;
        if (op == G_XOR) return val_a ? find_or_add_gate(G_NOT, b, 0) : b;
    }
    if (op != G_NOT && b < logic_gate_count && logic_gates[b].op == G_CONST) {
        int val_b = logic_gates[b].src_a;
        if (op == G_AND) return val_b ? a : b; 
        if (op == G_OR)  return val_b ? b : a;
        if (op == G_XOR) return val_b ? find_or_add_gate(G_NOT, a, 0) : a;
    }
    
    int existing = find_existing_gate(op, a, b); 
    if(existing != -1) return existing;
    
    int id = logic_gate_count; 
    logic_gates[id] = (LogicGate){op, a, b, id}; 
    logic_gate_count++;
    add_to_cache_manual(op, a, b, id); 
    return id;
}

int convert_robdd_to_gates(int node_idx) {
    if (node_to_gate[node_idx] != -1) return node_to_gate[node_idx];
    Node *n = &nodes[node_idx];
    
    if (n->is_const) { 
        int g = find_or_add_gate(G_CONST, n->value, 0); 
        node_to_gate[node_idx] = g; 
        return g; 
    }
    
    int sel = n->var_idx; 
    int low_gate = convert_robdd_to_gates(n->low); 
    int high_gate = convert_robdd_to_gates(n->high); 
    int res = -1;
    
    bool L0 = (logic_gates[low_gate].op == G_CONST && logic_gates[low_gate].src_a == 0);
    bool L1 = (logic_gates[low_gate].op == G_CONST && logic_gates[low_gate].src_a == 1);
    bool H0 = (logic_gates[high_gate].op == G_CONST && logic_gates[high_gate].src_a == 0);
    bool H1 = (logic_gates[high_gate].op == G_CONST && logic_gates[high_gate].src_a == 1);
    
    if (L0 && H1) res = sel;
    else if (L1 && H0) res = find_or_add_gate(G_NOT, sel, 0);
    else if (L0) res = find_or_add_gate(G_AND, sel, high_gate);
    else if (H0) { 
        int ns = find_or_add_gate(G_NOT, sel, 0); 
        res = find_or_add_gate(G_AND, ns, low_gate); 
    }
    else if (L1) { 
        int ns = find_or_add_gate(G_NOT, sel, 0); 
        res = find_or_add_gate(G_OR, ns, high_gate); 
    }
    else if (H1) res = find_or_add_gate(G_OR, sel, low_gate);
    else if (logic_gates[high_gate].op == G_NOT && logic_gates[high_gate].src_a == low_gate)
        res = find_or_add_gate(G_XOR, sel, low_gate);
    else if (logic_gates[low_gate].op == G_NOT && logic_gates[low_gate].src_a == high_gate) {
        int xor_gate = find_or_add_gate(G_XOR, sel, high_gate);
        res = find_or_add_gate(G_NOT, xor_gate, 0);
    } else {
        int term1 = find_or_add_gate(G_AND, sel, high_gate);
        int ns = find_or_add_gate(G_NOT, sel, 0);
        int term2 = find_or_add_gate(G_AND, ns, low_gate);
        res = find_or_add_gate(G_OR, term1, term2);
    }
    
    node_to_gate[node_idx] = res; 
    return res;
}

void init_var_order() {
    for (int i = 0; i < num_inputs; i++) {
        var_order[i] = i;
        var_level[i] = i;
    }
}

void compute_var_weights(TT **output_tables, int *weights) {
    memset(weights, 0, sizeof(int) * num_inputs);
    int total_rows = 1 << num_inputs;
    for (int out = 0; out < num_outputs; out++) {
        for (int var = 0; var < num_inputs; var++) {
            bool depends = false;
            int bit_pos = num_inputs - 1 - var;
            for (int row = 0; row < total_rows && !depends; row++) {
                int partner = row ^ (1 << bit_pos);
                if (tt_get_bit(output_tables[out], row) != tt_get_bit(output_tables[out], partner))
                    depends = true;
            }
            if (depends) weights[var]++;
        }
    }
}

void optimize_var_order_by_weight(TT **output_tables) {
    int weights[MAX_INPUTS];
    compute_var_weights(output_tables, weights);
    for (int i = 0; i < num_inputs - 1; i++) {
        for (int j = i + 1; j < num_inputs; j++) {
            if (weights[var_order[j]] > weights[var_order[i]]) {
                int tmp = var_order[i]; 
                var_order[i] = var_order[j]; 
                var_order[j] = tmp;
            }
        }
    }
    for (int i = 0; i < num_inputs; i++) 
        var_level[var_order[i]] = i;
}

void try_interleaved_order() {
    if (num_inputs < 4) return;
    int half = num_inputs / 2;
    for (int i = 0; i < half; i++) {
        var_order[2*i] = i;
        var_order[2*i + 1] = half + i;
    }
    if (num_inputs % 2) var_order[num_inputs - 1] = num_inputs - 1;
    for (int i = 0; i < num_inputs; i++) 
        var_level[var_order[i]] = i;
}

int try_window_permutation(TT **output_tables, int window_start, int window_size) {
    if (window_start + window_size > num_inputs) return -1;
    
    int window[8];
    for (int i = 0; i < window_size; i++) 
        window[i] = var_order[window_start + i];
    
    int best_perm[8];
    memcpy(best_perm, window, sizeof(int) * window_size);
    
    node_count = 0; 
    logic_gate_count = num_inputs;
    memset(memo, 0, sizeof(memo));
    memset(node_to_gate, -1, sizeof(node_to_gate));
    reset_gate_cache();
    
    for (int i = 0; i < num_outputs; i++) {
        int root = synthesize(output_tables[i], 0);
        output_map_robdd[i] = convert_robdd_to_gates(root);
    }
    int best_gates = logic_gate_count - num_inputs;
    
    int c[8] = {0};
    int perm[8];
    memcpy(perm, window, sizeof(int) * window_size);
    
    int i = 0;
    while (i < window_size) {
        if (c[i] < i) {
            if (i % 2 == 0) { 
                int t = perm[0]; perm[0] = perm[i]; perm[i] = t; 
            } else { 
                int t = perm[c[i]]; perm[c[i]] = perm[i]; perm[i] = t; 
            }
            
            for (int j = 0; j < window_size; j++) 
                var_order[window_start + j] = perm[j];
            for (int j = 0; j < num_inputs; j++) 
                var_level[var_order[j]] = j;
            
            node_count = 0; 
            logic_gate_count = num_inputs;
            memset(memo, 0, sizeof(memo));
            memset(node_to_gate, -1, sizeof(node_to_gate));
            reset_gate_cache();
            
            for (int k = 0; k < num_outputs; k++) {
                int root = synthesize(output_tables[k], 0);
                output_map_robdd[k] = convert_robdd_to_gates(root);
            }
            int gates = logic_gate_count - num_inputs;
            
            if (gates < best_gates) {
                best_gates = gates;
                memcpy(best_perm, perm, sizeof(int) * window_size);
            }
            
            c[i]++;
            i = 0;
        } else {
            c[i] = 0;
            i++;
        }
    }
    
    for (int j = 0; j < window_size; j++) 
        var_order[window_start + j] = best_perm[j];
    for (int j = 0; j < num_inputs; j++) 
        var_level[var_order[j]] = j;
    
    return best_gates;
}

void synthesize_all_outputs_improved(TT **output_tables) {
    init_var_order();
    printf("[*] Analyzing variable dependencies...\n");
    
    int best_gates = INT32_MAX;
    int best_order[MAX_INPUTS];
    
    node_count = 0; 
    logic_gate_count = num_inputs;
    memset(memo, 0, sizeof(memo));
    memset(node_to_gate, -1, sizeof(node_to_gate));
    reset_gate_cache();
    
    for (int i = 0; i < num_outputs; i++) {
        int root = synthesize(output_tables[i], 0);
        output_map_robdd[i] = convert_robdd_to_gates(root);
    }
    int default_gates = logic_gate_count - num_inputs;
    printf("    Default order: %d gates\n", default_gates);
    
    if (default_gates < best_gates) {
        best_gates = default_gates;
        memcpy(best_order, var_order, sizeof(int) * num_inputs);
    }
    
    init_var_order();
    optimize_var_order_by_weight(output_tables);
    
    node_count = 0; 
    logic_gate_count = num_inputs;
    memset(memo, 0, sizeof(memo));
    memset(node_to_gate, -1, sizeof(node_to_gate));
    reset_gate_cache();
    
    for (int i = 0; i < num_outputs; i++) {
        int root = synthesize(output_tables[i], 0);
        output_map_robdd[i] = convert_robdd_to_gates(root);
    }
    int weighted_gates = logic_gate_count - num_inputs;
    printf("    Weighted order: %d gates\n", weighted_gates);
    
    if (weighted_gates < best_gates) {
        best_gates = weighted_gates;
        memcpy(best_order, var_order, sizeof(int) * num_inputs);
    }
    
    if (num_inputs >= 4) {
        init_var_order();
        try_interleaved_order();
        
        node_count = 0; 
        logic_gate_count = num_inputs;
        memset(memo, 0, sizeof(memo));
        memset(node_to_gate, -1, sizeof(node_to_gate));
        reset_gate_cache();
        
        for (int i = 0; i < num_outputs; i++) {
            int root = synthesize(output_tables[i], 0);
            output_map_robdd[i] = convert_robdd_to_gates(root);
        }
        int interleaved_gates = logic_gate_count - num_inputs;
        printf("    Interleaved order: %d gates\n", interleaved_gates);
        
        if (interleaved_gates < best_gates) {
            best_gates = interleaved_gates;
            memcpy(best_order, var_order, sizeof(int) * num_inputs);
        }
    }
    
    if (num_inputs >= 4) {
        memcpy(var_order, best_order, sizeof(int) * num_inputs);
        for (int i = 0; i < num_inputs; i++) 
            var_level[var_order[i]] = i;
        
        printf("    Trying window permutations...\n");
        int window_size = (num_inputs >= 6) ? 4 : 3;
        bool improved = true;
        int passes = 0;
        
        while (improved && passes < 3) {
            improved = false;
            passes++;
            
            for (int start = 0; start + window_size <= num_inputs; start++) {
                int new_gates = try_window_permutation(output_tables, start, window_size);
                if (new_gates >= 0 && new_gates < best_gates) {
                    best_gates = new_gates;
                    memcpy(best_order, var_order, sizeof(int) * num_inputs);
                    improved = true;
                    printf("      Window [%d-%d]: improved to %d gates\n", 
                           start, start + window_size - 1, best_gates);
                }
            }
        }
    }
    
    memcpy(var_order, best_order, sizeof(int) * num_inputs);
    for (int i = 0; i < num_inputs; i++) 
        var_level[var_order[i]] = i;
    
    node_count = 0; 
    logic_gate_count = num_inputs;
    memset(memo, 0, sizeof(memo));
    memset(node_to_gate, -1, sizeof(node_to_gate));
    reset_gate_cache();
    
    for (int i = 0; i < num_outputs; i++) {
        int root = synthesize(output_tables[i], 0);
        output_map_robdd[i] = convert_robdd_to_gates(root);
    }
    
    printf("    Best: %d gates\n", logic_gate_count - num_inputs);
}

int verify_robdd_circuit_score() {
    int total_rows = 1 << num_inputs;
    int num_chunks_local = (total_rows + 63) / 64;
    TT *wire_results = (TT*)calloc(logic_gate_count, sizeof(TT));
    TT inputs_tt[MAX_INPUTS];
    
    for(int i = 0; i < num_inputs; i++) {
        inputs_tt[i].num_bits = total_rows; 
        inputs_tt[i].num_chunks = num_chunks_local;
        inputs_tt[i].chunks = (uint64_t*)calloc(num_chunks_local, sizeof(uint64_t));
        for(int r = 0; r < total_rows; r++) 
            if((r >> (num_inputs - 1 - i)) & 1) 
                tt_set_bit(&inputs_tt[i], r, 1);
    }
    
    for(int i = 0; i < logic_gate_count; i++) {
        wire_results[i].num_bits = total_rows; 
        wire_results[i].num_chunks = num_chunks_local;
        wire_results[i].chunks = (uint64_t*)calloc(num_chunks_local, sizeof(uint64_t));
        if (logic_gates[i].op == G_INPUT) 
            memcpy(wire_results[i].chunks, inputs_tt[logic_gates[i].src_a].chunks, num_chunks_local * 8);
        else if (logic_gates[i].op == G_CONST && logic_gates[i].src_a == 1) 
            memset(wire_results[i].chunks, 0xFF, num_chunks_local * 8);
    }
    
    TT zero_vec = {total_rows, num_chunks_local, (uint64_t*)calloc(num_chunks_local, sizeof(uint64_t))};
    
    for (int i = 0; i < logic_gate_count; i++) {
        if(logic_gates[i].op == G_INPUT || logic_gates[i].op == G_CONST) continue;
        LogicGate *g = &logic_gates[i];
        if (g->op == G_NOT) 
            robdd_run_gate(g->op, &wire_results[g->src_a], &zero_vec, &wire_results[i]);
        else 
            robdd_run_gate(g->op, &wire_results[g->src_a], &wire_results[g->src_b], &wire_results[i]);
    }
    
    int total_correct = 0;
    for(int out = 0; out < num_outputs; out++) {
        TT *result = &wire_results[output_map_robdd[out]];
        for(int ch = 0; ch < num_chunks_local; ch++) {
            uint64_t correct = ~(result->chunks[ch] ^ g_targets[out].chunks[ch]) & g_masks[out].chunks[ch];
            total_correct += __builtin_popcountll(correct);
        }
    }
    
    free(zero_vec.chunks);
    for(int i = 0; i < num_inputs; i++) free(inputs_tt[i].chunks);
    for(int i = 0; i < logic_gate_count; i++) free(wire_results[i].chunks);
    free(wire_results);
    
    return total_correct;
}

bool safe_rewriting(int required_score) {
    int total_rows = 1 << num_inputs;
    int num_chunks_local = (total_rows + 63) / 64;
    TT inputs_tt[MAX_INPUTS];
    
    for(int i=0; i<num_inputs; i++) {
        inputs_tt[i].num_bits = total_rows; 
        inputs_tt[i].num_chunks = num_chunks_local;
        inputs_tt[i].chunks = (uint64_t*)calloc(num_chunks_local, sizeof(uint64_t));
        for(int r=0; r<total_rows; r++) 
            if((r >> (num_inputs - 1 - i)) & 1) 
                tt_set_bit(&inputs_tt[i], r, 1);
    }
    
    TT *wire_results = (TT*)calloc(logic_gate_count, sizeof(TT));
    for(int i=0; i<logic_gate_count; i++) {
        wire_results[i].num_bits = total_rows; 
        wire_results[i].num_chunks = num_chunks_local;
        wire_results[i].chunks = (uint64_t*)calloc(num_chunks_local, sizeof(uint64_t));
        if (logic_gates[i].op == G_INPUT) 
            memcpy(wire_results[i].chunks, inputs_tt[logic_gates[i].src_a].chunks, num_chunks_local * 8);
        else if (logic_gates[i].op == G_CONST && logic_gates[i].src_a == 1) 
            memset(wire_results[i].chunks, 0xFF, num_chunks_local * 8);
    }
    
    TT zero_vec = {total_rows, num_chunks_local, (uint64_t*)calloc(num_chunks_local, sizeof(uint64_t))};
    
    for (int i = 0; i < logic_gate_count; i++) {
        if(logic_gates[i].op == G_INPUT || logic_gates[i].op == G_CONST) continue;
        LogicGate *g = &logic_gates[i];
        if (g->op == G_NOT) 
            robdd_run_gate(g->op, &wire_results[g->src_a], &zero_vec, &wire_results[i]);
        else 
            robdd_run_gate(g->op, &wire_results[g->src_a], &wire_results[g->src_b], &wire_results[i]);
    }
    
    TT temp_res = {total_rows, num_chunks_local, (uint64_t*)malloc(num_chunks_local * sizeof(uint64_t))};
    bool any_change = false;
    
    for(int i = num_inputs; i < logic_gate_count; i++) {
        if(logic_gates[i].op == G_INPUT || logic_gates[i].op == G_CONST) continue;
        TT *target_tt = &wire_results[i]; 
        LogicGate backup = logic_gates[i]; 
        bool found = false;
        
        for(int a = 0; a < i && !found; a++) {
            robdd_run_gate(G_NOT, &wire_results[a], &zero_vec, &temp_res);
            if(memcmp(temp_res.chunks, target_tt->chunks, num_chunks_local*8) == 0) {
                logic_gates[i].op = G_NOT; 
                logic_gates[i].src_a = a; 
                logic_gates[i].src_b = 0;
                if (verify_robdd_circuit_score() == required_score) { 
                    found = true; 
                    any_change = true; 
                }
                else logic_gates[i] = backup;
            }
        }
        
        int ops[] = {G_AND, G_OR, G_XOR};
        for(int a = 0; a < i && !found; a++) {
            for(int b = a; b < i && !found; b++) {
                for(int op_idx = 0; op_idx < 3 && !found; op_idx++) {
                    robdd_run_gate(ops[op_idx], &wire_results[a], &wire_results[b], &temp_res);
                    if(memcmp(temp_res.chunks, target_tt->chunks, num_chunks_local*8) == 0) {
                        logic_gates[i].op = ops[op_idx]; 
                        logic_gates[i].src_a = a; 
                        logic_gates[i].src_b = b;
                        if (verify_robdd_circuit_score() == required_score) { 
                            found = true; 
                            any_change = true; 
                        }
                        else logic_gates[i] = backup;
                    }
                }
            }
        }
    }
    
    for(int i=0; i<num_inputs; i++) free(inputs_tt[i].chunks);
    for(int i=0; i<logic_gate_count; i++) 
        if(wire_results[i].chunks) free(wire_results[i].chunks);
    free(wire_results); 
    free(zero_vec.chunks); 
    free(temp_res.chunks);
    
    return any_change;
}

int compact_robdd_gates() {
    int *remap = (int*)malloc(sizeof(int) * logic_gate_count); 
    memset(remap, -1, sizeof(int)*logic_gate_count);
    bool *used = (bool*)calloc(logic_gate_count, sizeof(bool)); 
    int *stack = (int*)malloc(sizeof(int) * logic_gate_count); 
    int sp = 0;
    
    for(int i=0; i<num_outputs; i++) { 
        int g = output_map_robdd[i]; 
        if(!used[g]) { used[g]=true; stack[sp++]=g; } 
    }
    
    while(sp > 0) { 
        int g = stack[--sp]; 
        if (logic_gates[g].op != G_INPUT && logic_gates[g].op != G_CONST) { 
            int a = logic_gates[g].src_a; 
            if(!used[a]) { used[a]=true; stack[sp++]=a; } 
            if (logic_gates[g].op != G_NOT) { 
                int b = logic_gates[g].src_b; 
                if(!used[b]) { used[b]=true; stack[sp++]=b; } 
            }
        }
    }
    
    LogicGate *new_gates = (LogicGate*)malloc(sizeof(LogicGate)*MAX_GATES); 
    reset_gate_cache(); 
    int new_cnt = 0;
    
    for(int i=0; i<logic_gate_count; i++) {
        if (!used[i]) continue; 
        int id;
        if (logic_gates[i].op == G_INPUT || logic_gates[i].op == G_CONST) { 
            new_gates[new_cnt] = logic_gates[i]; 
            new_gates[new_cnt].id = new_cnt; 
            if (logic_gates[i].op == G_CONST) { 
                int existing = find_existing_gate(G_CONST, logic_gates[i].src_a, 0); 
                id = (existing != -1) ? existing : (add_to_cache_manual(G_CONST, logic_gates[i].src_a, 0, new_cnt), new_cnt++); 
            } else id = new_cnt++; 
        } else { 
            int new_a = remap[logic_gates[i].src_a]; 
            int new_b = (logic_gates[i].op != G_NOT) ? remap[logic_gates[i].src_b] : 0; 
            int existing = find_existing_gate(logic_gates[i].op, new_a, new_b); 
            if (existing != -1) id = existing;
            else { 
                new_gates[new_cnt] = (LogicGate){logic_gates[i].op, new_a, new_b, new_cnt}; 
                add_to_cache_manual(logic_gates[i].op, new_a, new_b, new_cnt); 
                id = new_cnt++; 
            }
        }
        remap[i] = id;
    }
    
    for(int i=0; i<num_outputs; i++) 
        output_map_robdd[i] = remap[output_map_robdd[i]];
    memcpy(logic_gates, new_gates, sizeof(LogicGate) * new_cnt);
    free(new_gates); 
    free(remap); 
    free(used); 
    free(stack); 
    return new_cnt;
}

void crush_circuit_loop() {
    printf("[*] Running SAFE Optimization Loop...\n"); 
    int max_score = 0;
    for(int i = 0; i < num_outputs; i++) 
        for(int c = 0; c < g_num_chunks; c++) 
            max_score += __builtin_popcountll(g_masks[i].chunks[c]);
    
    int current_score = verify_robdd_circuit_score();
    printf("    Initial verification: %d / %d\n", current_score, max_score);
    if (current_score != max_score) { 
        printf("    [WARNING] ROBDD incorrect!\n"); 
        return; 
    }
    
    int old_count = logic_gate_count, pass = 0;
    while(pass < 20) { 
        pass++; 
        bool changed = safe_rewriting(max_score);
        int new_count = compact_robdd_gates(); 
        if(new_count < logic_gate_count) changed = true;
        logic_gate_count = new_count;
        int new_score = verify_robdd_circuit_score();
        printf("    Pass %d: %d gates, score %d/%d\n", pass, logic_gate_count - num_inputs, new_score, max_score);
        if (new_score != max_score) { 
            printf("    [ERROR] Broke circuit!\n"); 
            break; 
        }
        if (!changed && old_count == logic_gate_count) break; 
        old_count = logic_gate_count; 
    }
}

/* ============================================================
 * BRIDGE: ROBDD TO EVOLUTIONARY CIRCUIT
 * ============================================================ */

void load_evolutionary_from_robdd(Circuit *dest) {
    dest->num_inputs = num_inputs; 
    dest->num_outputs = num_outputs; 
    dest->num_gates = 0;
    dest->allowed_ops_count = g_allowed_ops_count; 
    memcpy(dest->allowed_ops, g_allowed_ops, sizeof(g_allowed_ops));
    dest->dead_count = 0;
    
    int remap[MAX_GATES]; 
    for(int i = 0; i < MAX_GATES; i++) remap[i] = -999;
    for(int i = 0; i < num_inputs; i++) remap[i] = i;
    for(int i = num_inputs; i < logic_gate_count; i++)
        if(logic_gates[i].op == G_CONST) 
            remap[i] = (logic_gates[i].src_a == 1) ? CONST_ONE : CONST_ZERO;
    
    bool changed = true; 
    int iterations = 0;
    while (changed && iterations < 1000) {
        changed = false; 
        iterations++;
        for(int i = num_inputs; i < logic_gate_count; i++) {
            if (remap[i] != -999) continue;
            LogicGate g = logic_gates[i]; 
            if (g.op == G_CONST) continue;
            int wire_a = remap[g.src_a]; 
            int wire_b = (g.op != G_NOT) ? remap[g.src_b] : 0;
            int val_a = (wire_a == CONST_ZERO) ? 0 : (wire_a == CONST_ONE) ? 1 : -1;
            int val_b = (wire_b == CONST_ZERO) ? 0 : (wire_b == CONST_ONE) ? 1 : -1;
            bool folded = false;
            
            if (g.op == G_NOT) {
                if (val_a == 1) { remap[i] = CONST_ZERO; folded = true; }
                else if (val_a == 0) { remap[i] = CONST_ONE; folded = true; }
            } else if (g.op == G_AND) {
                if (val_a == 0 || val_b == 0) { remap[i] = CONST_ZERO; folded = true; }
                else if (val_a == 1 && val_b == 1) { remap[i] = CONST_ONE; folded = true; }
                else if (val_a == 1) { remap[i] = wire_b; folded = true; }
                else if (val_b == 1) { remap[i] = wire_a; folded = true; }
            } else if (g.op == G_OR) {
                if (val_a == 1 || val_b == 1) { remap[i] = CONST_ONE; folded = true; }
                else if (val_a == 0 && val_b == 0) { remap[i] = CONST_ZERO; folded = true; }
                else if (val_a == 0) { remap[i] = wire_b; folded = true; }
                else if (val_b == 0) { remap[i] = wire_a; folded = true; }
            } else if (g.op == G_XOR) {
                if (val_a == 0) { remap[i] = wire_b; folded = true; }
                else if (val_b == 0) { remap[i] = wire_a; folded = true; }
                else if (val_a == 1 && val_b == 1) { remap[i] = CONST_ZERO; folded = true; }
                else if (val_a == 1 && wire_b >= 0) {
                    dest->gates[dest->num_gates] = (Gate){EVO_NOT, (int16_t)wire_b, 0, true};
                    remap[i] = num_inputs + dest->num_gates++; 
                    folded = true;
                } else if (val_b == 1 && wire_a >= 0) {
                    dest->gates[dest->num_gates] = (Gate){EVO_NOT, (int16_t)wire_a, 0, true};
                    remap[i] = num_inputs + dest->num_gates++; 
                    folded = true;
                }
            }
            if (folded) { changed = true; continue; }
            
            if (wire_a >= 0 && (g.op == G_NOT || wire_b >= 0)) {
                uint8_t evo_op = (g.op == G_AND) ? EVO_AND : (g.op == G_OR) ? EVO_OR : 
                                 (g.op == G_XOR) ? EVO_XOR : (g.op == G_NOT) ? EVO_NOT : 255;
                if (evo_op == 255) continue;
                dest->gates[dest->num_gates] = (Gate){evo_op, (int16_t)wire_a, (int16_t)wire_b, true};
                remap[i] = num_inputs + dest->num_gates++; 
                changed = true;
            }
        }
    }
    
    for(int i = 0; i < num_outputs; i++) {
        int wire = remap[output_map_robdd[i]];
        if (wire == CONST_ZERO) { 
            dest->gates[dest->num_gates] = (Gate){EVO_XOR, 0, 0, true}; 
            wire = num_inputs + dest->num_gates++; 
        }
        else if (wire == CONST_ONE) {
            dest->gates[dest->num_gates] = (Gate){EVO_NOT, 0, 0, true}; 
            int not_idx = dest->num_gates++;
            dest->gates[dest->num_gates] = (Gate){EVO_OR, 0, (int16_t)(num_inputs + not_idx), true}; 
            wire = num_inputs + dest->num_gates++;
        } else if (wire < 0) { 
            printf("    [ERROR] Output %d unresolved!\n", i); 
            wire = 0; 
        }
        dest->output_map[i] = wire;
    }
}

/* ============================================================
 * TECHNOLOGY MAPPING
 * ============================================================ */

bool is_op_allowed(Circuit *c, int op) {
    for (int i = 0; i < c->allowed_ops_count; i++)
        if (c->allowed_ops[i] == op) return true;
    return false;
}

int circuit_add_gate_alive(Circuit *c, int op, int src_a, int src_b) {
    if (c->num_gates >= MAX_EVOL_GATES) return c->num_inputs + c->num_gates - 1;
    c->gates[c->num_gates].op = op;
    c->gates[c->num_gates].src_a = src_a;
    c->gates[c->num_gates].src_b = src_b;
    c->gates[c->num_gates].alive = true;
    return c->num_inputs + c->num_gates++;
}

void build_xor_from_nand(Circuit *c, int a, int b, int *result) {
    int nand_ab = circuit_add_gate_alive(c, EVO_NAND, a, b);
    int t1 = circuit_add_gate_alive(c, EVO_NAND, a, nand_ab);
    int t2 = circuit_add_gate_alive(c, EVO_NAND, b, nand_ab);
    *result = circuit_add_gate_alive(c, EVO_NAND, t1, t2);
}

void build_xor_from_nor(Circuit *c, int a, int b, int *result) {
    int nor_ab = circuit_add_gate_alive(c, EVO_NOR, a, b);
    int na = circuit_add_gate_alive(c, EVO_NOR, a, a);
    int nb = circuit_add_gate_alive(c, EVO_NOR, b, b);
    int nand_ab = circuit_add_gate_alive(c, EVO_NOR, na, nb);
    *result = circuit_add_gate_alive(c, EVO_NOR, nor_ab, nand_ab);
}

void convert_to_allowed_gates_improved(Circuit *c) {
    bool needs_conversion = false;
    for (int i = 0; i < c->num_gates; i++) {
        if (c->gates[i].alive && !is_op_allowed(c, c->gates[i].op)) {
            needs_conversion = true;
            break;
        }
    }
    if (!needs_conversion) {
        printf("    [*] All gates already use allowed operations.\n");
        return;
    }
    
    printf("    [*] Converting gates with pattern matching...\n");
    
    Gate old_gates[MAX_EVOL_GATES];
    int old_num_gates = c->num_gates;
    memcpy(old_gates, c->gates, sizeof(Gate) * old_num_gates);
    
    int old_outputs[MAX_OUTPUTS];
    memcpy(old_outputs, c->output_map, sizeof(int) * c->num_outputs);
    
    int wire_map[MAX_WIRES];
    for (int i = 0; i < c->num_inputs; i++) wire_map[i] = i;
    
    c->num_gates = 0;
    c->dead_count = 0;
    
    bool has_nand = is_op_allowed(c, EVO_NAND);
    bool has_nor = is_op_allowed(c, EVO_NOR);
    bool has_not = is_op_allowed(c, EVO_NOT);
    bool has_and = is_op_allowed(c, EVO_AND);
    bool has_or = is_op_allowed(c, EVO_OR);
    
    for (int i = 0; i < old_num_gates; i++) {
        if (!old_gates[i].alive) {
            wire_map[c->num_inputs + i] = 0;
            continue;
        }
        
        Gate *g = &old_gates[i];
        int old_wire = c->num_inputs + i;
        int a = wire_map[g->src_a];
        int b = wire_map[g->src_b];
        int new_wire;
        
        if (is_op_allowed(c, g->op)) {
            new_wire = circuit_add_gate_alive(c, g->op, a, b);
        }
        else if (has_nand) {
            switch (g->op) {
                case EVO_NOT:
                    new_wire = circuit_add_gate_alive(c, EVO_NAND, a, a);
                    break;
                case EVO_AND: {
                    int t = circuit_add_gate_alive(c, EVO_NAND, a, b);
                    new_wire = circuit_add_gate_alive(c, EVO_NAND, t, t);
                    break;
                }
                case EVO_OR: {
                    int na = circuit_add_gate_alive(c, EVO_NAND, a, a);
                    int nb = circuit_add_gate_alive(c, EVO_NAND, b, b);
                    new_wire = circuit_add_gate_alive(c, EVO_NAND, na, nb);
                    break;
                }
                case EVO_XOR:
                    build_xor_from_nand(c, a, b, &new_wire);
                    break;
                case EVO_NOR: {
                    int na = circuit_add_gate_alive(c, EVO_NAND, a, a);
                    int nb = circuit_add_gate_alive(c, EVO_NAND, b, b);
                    int or_ab = circuit_add_gate_alive(c, EVO_NAND, na, nb);
                    new_wire = circuit_add_gate_alive(c, EVO_NAND, or_ab, or_ab);
                    break;
                }
                case EVO_XNOR: {
                    int xor_result;
                    build_xor_from_nand(c, a, b, &xor_result);
                    new_wire = circuit_add_gate_alive(c, EVO_NAND, xor_result, xor_result);
                    break;
                }
                default:
                    new_wire = a;
            }
        }
        else if (has_nor) {
            switch (g->op) {
                case EVO_NOT:
                    new_wire = circuit_add_gate_alive(c, EVO_NOR, a, a);
                    break;
                case EVO_OR: {
                    int t = circuit_add_gate_alive(c, EVO_NOR, a, b);
                    new_wire = circuit_add_gate_alive(c, EVO_NOR, t, t);
                    break;
                }
                case EVO_AND: {
                    int na = circuit_add_gate_alive(c, EVO_NOR, a, a);
                    int nb = circuit_add_gate_alive(c, EVO_NOR, b, b);
                    new_wire = circuit_add_gate_alive(c, EVO_NOR, na, nb);
                    break;
                }
                case EVO_XOR:
                    build_xor_from_nor(c, a, b, &new_wire);
                    break;
                case EVO_NAND: {
                    int na = circuit_add_gate_alive(c, EVO_NOR, a, a);
                    int nb = circuit_add_gate_alive(c, EVO_NOR, b, b);
                    int and_ab = circuit_add_gate_alive(c, EVO_NOR, na, nb);
                    new_wire = circuit_add_gate_alive(c, EVO_NOR, and_ab, and_ab);
                    break;
                }
                case EVO_XNOR: {
                    int xor_result;
                    build_xor_from_nor(c, a, b, &xor_result);
                    new_wire = circuit_add_gate_alive(c, EVO_NOR, xor_result, xor_result);
                    break;
                }
                default:
                    new_wire = a;
            }
        }
        else {
            switch (g->op) {
                case EVO_NOT:
                    if (has_nand) new_wire = circuit_add_gate_alive(c, EVO_NAND, a, a);
                    else if (has_nor) new_wire = circuit_add_gate_alive(c, EVO_NOR, a, a);
                    else new_wire = a;
                    break;
                case EVO_AND:
                    if (has_or && has_not) {
                        int na = circuit_add_gate_alive(c, EVO_NOT, a, 0);
                        int nb = circuit_add_gate_alive(c, EVO_NOT, b, 0);
                        int or_ab = circuit_add_gate_alive(c, EVO_OR, na, nb);
                        new_wire = circuit_add_gate_alive(c, EVO_NOT, or_ab, 0);
                    } else new_wire = a;
                    break;
                case EVO_OR:
                    if (has_and && has_not) {
                        int na = circuit_add_gate_alive(c, EVO_NOT, a, 0);
                        int nb = circuit_add_gate_alive(c, EVO_NOT, b, 0);
                        int and_ab = circuit_add_gate_alive(c, EVO_AND, na, nb);
                        new_wire = circuit_add_gate_alive(c, EVO_NOT, and_ab, 0);
                    } else new_wire = a;
                    break;
                case EVO_XOR:
                    if (has_and && has_or && has_not) {
                        int na = circuit_add_gate_alive(c, EVO_NOT, a, 0);
                        int nb = circuit_add_gate_alive(c, EVO_NOT, b, 0);
                        int t1 = circuit_add_gate_alive(c, EVO_AND, a, nb);
                        int t2 = circuit_add_gate_alive(c, EVO_AND, na, b);
                        new_wire = circuit_add_gate_alive(c, EVO_OR, t1, t2);
                    } else new_wire = a;
                    break;
                default:
                    new_wire = a;
            }
        }
        
        wire_map[old_wire] = new_wire;
    }
    
    for (int i = 0; i < c->num_outputs; i++) {
        c->output_map[i] = wire_map[old_outputs[i]];
    }
    
    printf("    [*] Conversion complete: %d gates (was %d)\n", c->num_gates, old_num_gates);
}

/* ============================================================
 * EVOLUTIONARY CGP ENGINE
 * ============================================================ */

int evo_get_score(Circuit *c, BitVec *inputs, BitVec *targets, BitVec *masks, int num_chunks_param) {
    BitVec wires[MAX_WIRES];
    static const BitVec zero = {0};
    for (int i = 0; i < c->num_inputs; i++) wires[i] = inputs[i];
    
    for (int i = 0; i < c->num_gates; i++) {
        if (!c->gates[i].alive) continue;
        Gate *g = &c->gates[i];
        BitVec *va = &wires[g->src_a];
        BitVec *vb = (g->op == EVO_NOT) ? (BitVec*)&zero : &wires[g->src_b];
        evo_run_gate(g->op, va, vb, &wires[c->num_inputs + i], num_chunks_param);
    }
    
    int total = 0;
    for (int i = 0; i < c->num_outputs; i++) {
        BitVec *out = &wires[c->output_map[i]];
        for (int ch = 0; ch < num_chunks_param; ch++) {
            uint64_t matches = ~(out->chunks[ch] ^ targets[i].chunks[ch]) & masks[i].chunks[ch];
            total += __builtin_popcountll(matches);
        }
    }
    return total;
}

int circuit_num_wires(Circuit *c) { 
    return c->num_inputs + c->num_gates; 
}

int circuit_get_active_indices(Circuit *c, int *active_indices) {
    bool used_wire[MAX_WIRES] = {0};
    for (int i = 0; i < c->num_outputs; i++) 
        used_wire[c->output_map[i]] = true;
    
    for (int i = c->num_gates - 1; i >= 0; i--) {
        if (!c->gates[i].alive) continue;
        int wire_idx = c->num_inputs + i;
        if (used_wire[wire_idx]) {
            used_wire[c->gates[i].src_a] = true;
            if (c->gates[i].op != EVO_NOT) 
                used_wire[c->gates[i].src_b] = true;
        }
    }
    
    int count = 0;
    for (int i = 0; i < c->num_gates; i++) 
        if (c->gates[i].alive && used_wire[c->num_inputs + i]) 
            active_indices[count++] = i;
    return count;
}

int circuit_count_active(Circuit *c) { 
    int dummy[MAX_EVOL_GATES]; 
    return circuit_get_active_indices(c, dummy); 
}

bool circuit_compact_lazy(Circuit *c, bool force) {
    int alive_count = 0;
    for (int i = 0; i < c->num_gates; i++) 
        if (c->gates[i].alive) alive_count++;
    
    float dead_ratio = (c->num_gates > 0) ? 
        (float)(c->num_gates - alive_count) / c->num_gates : 0;
    
    if (!force && dead_ratio < 0.5 && c->num_gates < MAX_EVOL_GATES - 100)
        return false;
    
    int active[MAX_EVOL_GATES];
    int active_count = circuit_get_active_indices(c, active);
    if (active_count == c->num_gates && alive_count == c->num_gates) return false;
    
    int old_to_new[MAX_WIRES];
    for (int i = 0; i < c->num_inputs; i++) old_to_new[i] = i;
    Gate new_gates[MAX_EVOL_GATES]; 
    int new_gate_idx = 0;
    
    for (int i = 0; i < active_count; i++) {
        int old_idx = active[i];
        old_to_new[c->num_inputs + old_idx] = c->num_inputs + new_gate_idx;
        Gate g = c->gates[old_idx];
        g.src_a = old_to_new[g.src_a];
        if (g.op != EVO_NOT) g.src_b = old_to_new[g.src_b];
        g.alive = true;
        new_gates[new_gate_idx++] = g;
    }
    
    c->num_gates = new_gate_idx;
    c->dead_count = 0;
    for (int i = 0; i < new_gate_idx; i++) c->gates[i] = new_gates[i];
    for (int i = 0; i < c->num_outputs; i++) 
        c->output_map[i] = old_to_new[c->output_map[i]];
    return true;
}

bool circuit_compact(Circuit *c) {
    return circuit_compact_lazy(c, true);
}

void circuit_soft_kick(Circuit *c, double rewire_prob) {
    for (int i = 0; i < c->num_gates; i++) {
        if (!c->gates[i].alive) continue;
        int max_src = c->num_inputs + i - 1;
        if (max_src < 0) continue;
        
        if (rand_double() < rewire_prob) {
            c->gates[i].src_a = randint(0, max_src);
        }
        if (c->gates[i].op != EVO_NOT && rand_double() < rewire_prob) {
            c->gates[i].src_b = randint(0, max_src);
        }
    }
    
    for (int i = 0; i < c->num_outputs; i++) {
        if (rand_double() < rewire_prob * 0.5) {
            c->output_map[i] = randint(0, circuit_num_wires(c) - 1);
        }
    }
}

void circuit_rewire_mutation(Circuit *c) {
    if (c->num_gates < 2) return;
    
    int attempts = 0;
    while (attempts < 10) {
        int gate_idx = randint(0, c->num_gates - 1);
        if (!c->gates[gate_idx].alive) { attempts++; continue; }
        
        int wire_idx = c->num_inputs + gate_idx;
        int replacement = c->gates[gate_idx].src_a;
        
        if (rand_double() < 0.5 && c->gates[gate_idx].op != EVO_NOT) {
            replacement = c->gates[gate_idx].src_b;
        }
        
        for (int i = 0; i < c->num_gates; i++) {
            if (!c->gates[i].alive) continue;
            if (c->gates[i].src_a == wire_idx) c->gates[i].src_a = replacement;
            if (c->gates[i].op != EVO_NOT && c->gates[i].src_b == wire_idx) 
                c->gates[i].src_b = replacement;
        }
        for (int i = 0; i < c->num_outputs; i++) {
            if (c->output_map[i] == wire_idx) c->output_map[i] = replacement;
        }
        return;
    }
}

void circuit_mutate(Circuit *c, double rewire_prob) {
    if (c->num_gates == 0) return;
    double r = rand_double();
    
    if (r < rewire_prob) {
        circuit_rewire_mutation(c);
        return;
    }
    
    if (c->num_gates >= 2 && r < rewire_prob + 0.05) {
        int i = randint(0, c->num_gates - 1);
        int j = randint(0, c->num_gates - 1);
        if (i != j && c->gates[i].alive && c->gates[j].alive) {
            if (i > j) { int t = i; i = j; j = t; }
            int wire_i = c->num_inputs + i, wire_j = c->num_inputs + j;
            Gate temp = c->gates[i]; c->gates[i] = c->gates[j]; c->gates[j] = temp;
            for (int k = 0; k < c->num_gates; k++) {
                if (!c->gates[k].alive) continue;
                if (c->gates[k].src_a == wire_i) c->gates[k].src_a = wire_j;
                else if (c->gates[k].src_a == wire_j) c->gates[k].src_a = wire_i;
                if (c->gates[k].op != EVO_NOT) {
                    if (c->gates[k].src_b == wire_i) c->gates[k].src_b = wire_j;
                    else if (c->gates[k].src_b == wire_j) c->gates[k].src_b = wire_i;
                }
            }
            for (int k = 0; k < c->num_outputs; k++) {
                if (c->output_map[k] == wire_i) c->output_map[k] = wire_j;
                else if (c->output_map[k] == wire_j) c->output_map[k] = wire_i;
            }
            for (int k = i; k <= j; k++) {
                if (!c->gates[k].alive) continue;
                int max_src_k = c->num_inputs + k - 1;
                if (max_src_k >= 0) {
                    if (c->gates[k].src_a > max_src_k) 
                        c->gates[k].src_a = randint(0, max_src_k);
                    if (c->gates[k].op != EVO_NOT && c->gates[k].src_b > max_src_k) 
                        c->gates[k].src_b = randint(0, max_src_k);
                }
            }
        }
        return;
    }
    
    if (rand_double() < 0.75) {
        int attempts = 0;
        while (attempts < 20) {
            int idx = randint(0, c->num_gates - 1);
            if (!c->gates[idx].alive) { attempts++; continue; }
            
            int max_src = c->num_inputs + idx - 1;
            int what = randint(0, 2);
            if (what == 0) {
                int op_idx = randint(0, c->allowed_ops_count - 1);
                c->gates[idx].op = c->allowed_ops[op_idx];
            }
            else if (what == 1 && max_src >= 0) 
                c->gates[idx].src_a = randint(0, max_src);
            else if (max_src >= 0 && c->gates[idx].op != EVO_NOT) 
                c->gates[idx].src_b = randint(0, max_src);
            break;
        }
    } else {
        int out_idx = randint(0, c->num_outputs - 1);
        c->output_map[out_idx] = randint(0, circuit_num_wires(c) - 1);
    }
}

int circuit_add_random_gate(Circuit *c) {
    if (c->num_gates >= MAX_EVOL_GATES) return -1;
    int max_src = circuit_num_wires(c) - 1;
    int op_idx = randint(0, c->allowed_ops_count - 1);
    c->gates[c->num_gates].op = c->allowed_ops[op_idx];
    c->gates[c->num_gates].src_a = randint(0, max_src);
    c->gates[c->num_gates].src_b = randint(0, max_src);
    c->gates[c->num_gates].alive = true;
    c->num_gates++;
    return circuit_num_wires(c) - 1;
}

bool evo_try_merge_global(Circuit *c, BitVec *inputs, BitVec *targets, BitVec *masks, int num_chunks_param, int max_score) {
    int active[MAX_EVOL_GATES];
    int active_count = circuit_get_active_indices(c, active);
    
    int checks = 0;
    while (checks < GLOBAL_MERGE_CHECKS) {
        int i = randint(0, active_count - 1);
        int j = randint(0, active_count - 1);
        if (i >= j) { checks++; continue; }
        
        int gi = active[i], gj = active[j];
        int wire_i = c->num_inputs + gi;
        
        if (c->gates[gj].src_a != wire_i && 
            (c->gates[gj].op == EVO_NOT || c->gates[gj].src_b != wire_i)) {
            checks++;
            continue;
        }
        
        Circuit backup = *c;
        Gate *g_i = &c->gates[gi];
        int other_input = (c->gates[gj].src_a == wire_i) ? c->gates[gj].src_b : c->gates[gj].src_a;
        
        for (int op_try = 0; op_try < c->allowed_ops_count; op_try++) {
            int new_op = c->allowed_ops[op_try];
            if (new_op == EVO_NOT) continue;
            
            c->gates[gj].op = new_op;
            c->gates[gj].src_a = g_i->src_a;
            c->gates[gj].src_b = other_input;
            
            if (evo_get_score(c, inputs, targets, masks, num_chunks_param) == max_score) {
                circuit_compact_lazy(c, false);
                return true;
            }
            *c = backup;
            
            if (g_i->op != EVO_NOT) {
                c->gates[gj].op = new_op;
                c->gates[gj].src_a = g_i->src_b;
                c->gates[gj].src_b = other_input;
                
                if (evo_get_score(c, inputs, targets, masks, num_chunks_param) == max_score) {
                    circuit_compact_lazy(c, false);
                    return true;
                }
                *c = backup;
            }
        }
        checks++;
    }
    return false;
}

bool evo_try_merge_adjacent(Circuit *c, BitVec *inputs, BitVec *targets, BitVec *masks, int num_chunks_param, int max_score) {
    int active[MAX_EVOL_GATES];
    int active_count = circuit_get_active_indices(c, active);
    const int MAX_DISTANCE = 10;
    
    for (int i = 0; i < active_count; i++) {
        for (int j = i + 1; j < active_count && (active[j] - active[i]) <= MAX_DISTANCE; j++) {
            int gi = active[i], gj = active[j];
            int wire_i = c->num_inputs + gi;
            
            if (c->gates[gj].src_a != wire_i && 
                (c->gates[gj].op == EVO_NOT || c->gates[gj].src_b != wire_i)) 
                continue;
            
            Circuit backup = *c;
            Gate *g_i = &c->gates[gi];
            int other_input = (c->gates[gj].src_a == wire_i) ? c->gates[gj].src_b : c->gates[gj].src_a;
            
            for (int op_try = 0; op_try < c->allowed_ops_count; op_try++) {
                int new_op = c->allowed_ops[op_try];
                if (new_op == EVO_NOT) continue;
                
                c->gates[gj].op = new_op;
                c->gates[gj].src_a = g_i->src_a;
                c->gates[gj].src_b = other_input;
                
                if (evo_get_score(c, inputs, targets, masks, num_chunks_param) == max_score) {
                    circuit_compact_lazy(c, false);
                    return true;
                }
                *c = backup;
                
                if (g_i->op != EVO_NOT) {
                    c->gates[gj].op = new_op;
                    c->gates[gj].src_a = g_i->src_b;
                    c->gates[gj].src_b = other_input;
                    
                    if (evo_get_score(c, inputs, targets, masks, num_chunks_param) == max_score) {
                        circuit_compact_lazy(c, false);
                        return true;
                    }
                    *c = backup;
                }
            }
        }
    }
    return false;
}

int evo_deep_prune(Circuit *c, BitVec *inputs, BitVec *targets, BitVec *masks, int num_chunks_param, int max_score) {
    int removed = 0;
    bool improved = true;
    int active[MAX_EVOL_GATES];
    
    while (improved) {
        improved = false;
        int active_count = circuit_get_active_indices(c, active);
        
        for (int k = active_count - 1; k >= 0; k--) {
            int gate_idx = active[k];
            int wire_idx = c->num_inputs + gate_idx;
            Gate *gate = &c->gates[gate_idx];
            Circuit backup = *c;
            
            int replacement = gate->src_a;
            for (int i = 0; i < c->num_outputs; i++) 
                if (c->output_map[i] == wire_idx) c->output_map[i] = replacement;
            for (int i = 0; i < c->num_gates; i++) {
                if (!c->gates[i].alive) continue;
                if (c->gates[i].src_a == wire_idx) c->gates[i].src_a = replacement;
                if (c->gates[i].op != EVO_NOT && c->gates[i].src_b == wire_idx) 
                    c->gates[i].src_b = replacement;
            }
            if (evo_get_score(c, inputs, targets, masks, num_chunks_param) == max_score) { 
                improved = true; removed++; break; 
            }
            *c = backup;
            
            if (gate->op != EVO_NOT) {
                replacement = gate->src_b;
                for (int i = 0; i < c->num_outputs; i++) 
                    if (c->output_map[i] == wire_idx) c->output_map[i] = replacement;
                for (int i = 0; i < c->num_gates; i++) {
                    if (!c->gates[i].alive) continue;
                    if (c->gates[i].src_a == wire_idx) c->gates[i].src_a = replacement;
                    if (c->gates[i].op != EVO_NOT && c->gates[i].src_b == wire_idx) 
                        c->gates[i].src_b = replacement;
                }
                if (evo_get_score(c, inputs, targets, masks, num_chunks_param) == max_score) { 
                    improved = true; removed++; break; 
                }
                *c = backup;
            }
            
            for (int try_wire = 0; try_wire < wire_idx && !improved; try_wire++) {
                for (int i = 0; i < c->num_outputs; i++) 
                    if (c->output_map[i] == wire_idx) c->output_map[i] = try_wire;
                for (int i = 0; i < c->num_gates; i++) {
                    if (!c->gates[i].alive) continue;
                    if (c->gates[i].src_a == wire_idx) c->gates[i].src_a = try_wire;
                    if (c->gates[i].op != EVO_NOT && c->gates[i].src_b == wire_idx) 
                        c->gates[i].src_b = try_wire;
                }
                if (evo_get_score(c, inputs, targets, masks, num_chunks_param) == max_score) { 
                    improved = true; removed++; break; 
                }
                *c = backup;
            }
        }
    }
    circuit_compact(c);
    return removed;
}

bool evo_algebraic_simplify(Circuit *c, BitVec *inputs, BitVec *targets, BitVec *masks, int num_chunks_param, int max_score) {
    bool any_change = false;
    int active[MAX_EVOL_GATES];
    int active_count = circuit_get_active_indices(c, active);
    
    for (int k = 0; k < active_count; k++) {
        int gi = active[k];
        if (!c->gates[gi].alive) continue;
        Gate *g = &c->gates[gi];
        int wire_idx = c->num_inputs + gi;
        Circuit backup = *c;
        
        if (g->op != EVO_NOT && g->src_a == g->src_b) {
            int replacement = -1;
            if (g->op == EVO_AND || g->op == EVO_OR) replacement = g->src_a;
            else if (g->op == EVO_NAND || g->op == EVO_NOR) {
                if (is_op_allowed(c, EVO_NOT)) {
                    g->op = EVO_NOT;
                    if (evo_get_score(c, inputs, targets, masks, num_chunks_param) == max_score) {
                        any_change = true; continue;
                    }
                }
                *c = backup;
            }
            
            if (replacement >= 0) {
                for (int i = 0; i < c->num_outputs; i++) 
                    if (c->output_map[i] == wire_idx) c->output_map[i] = replacement;
                for (int i = 0; i < c->num_gates; i++) {
                    if (!c->gates[i].alive) continue;
                    if (c->gates[i].src_a == wire_idx) c->gates[i].src_a = replacement;
                    if (c->gates[i].op != EVO_NOT && c->gates[i].src_b == wire_idx) 
                        c->gates[i].src_b = replacement;
                }
                if (evo_get_score(c, inputs, targets, masks, num_chunks_param) == max_score) {
                    any_change = true; continue;
                }
                *c = backup;
            }
        }
        
        if (g->op == EVO_NOT && g->src_a >= c->num_inputs) {
            int src_gate_idx = g->src_a - c->num_inputs;
            if (c->gates[src_gate_idx].alive && c->gates[src_gate_idx].op == EVO_NOT) {
                int replacement = c->gates[src_gate_idx].src_a;
                for (int i = 0; i < c->num_outputs; i++) 
                    if (c->output_map[i] == wire_idx) c->output_map[i] = replacement;
                for (int i = 0; i < c->num_gates; i++) {
                    if (!c->gates[i].alive) continue;
                    if (c->gates[i].src_a == wire_idx) c->gates[i].src_a = replacement;
                    if (c->gates[i].op != EVO_NOT && c->gates[i].src_b == wire_idx) 
                        c->gates[i].src_b = replacement;
                }
                if (evo_get_score(c, inputs, targets, masks, num_chunks_param) == max_score) {
                    any_change = true; continue;
                }
                *c = backup;
            }
        }
        
        if (g->op == EVO_NAND && g->src_a == g->src_b && g->src_a >= c->num_inputs) {
            int src_gate_idx = g->src_a - c->num_inputs;
            Gate *src_g = &c->gates[src_gate_idx];
            if (src_g->alive && src_g->op == EVO_NAND && src_g->src_a == src_g->src_b) {
                int replacement = src_g->src_a;
                for (int i = 0; i < c->num_outputs; i++) 
                    if (c->output_map[i] == wire_idx) c->output_map[i] = replacement;
                for (int i = 0; i < c->num_gates; i++) {
                    if (!c->gates[i].alive) continue;
                    if (c->gates[i].src_a == wire_idx) c->gates[i].src_a = replacement;
                    if (c->gates[i].op != EVO_NOT && c->gates[i].src_b == wire_idx) 
                        c->gates[i].src_b = replacement;
                }
                if (evo_get_score(c, inputs, targets, masks, num_chunks_param) == max_score) {
                    any_change = true; continue;
                }
                *c = backup;
            }
        }
    }
    
    if (any_change) circuit_compact_lazy(c, false);
    return any_change;
}

void run_generic_evolution_phase_single(BitVec *inputs, BitVec *targets, BitVec *masks, int num_chunks_param, Circuit *circuit, int run_id) {
    int orig_allowed_ops[16];
    int orig_allowed_count = circuit->allowed_ops_count;
    memcpy(orig_allowed_ops, circuit->allowed_ops, sizeof(orig_allowed_ops));
    
    circuit->allowed_ops_count = 4;
    circuit->allowed_ops[0] = EVO_AND;
    circuit->allowed_ops[1] = EVO_OR;
    circuit->allowed_ops[2] = EVO_XOR;
    circuit->allowed_ops[3] = EVO_NOT;
    
    int max_score = 0;
    for(int i = 0; i < num_outputs; i++)
        for(int ch = 0; ch < num_chunks_param; ch++)
            max_score += __builtin_popcountll(masks[i].chunks[ch]);
    
    int initial_score = evo_get_score(circuit, inputs, targets, masks, num_chunks_param);
    if (initial_score != max_score) {
        circuit->allowed_ops_count = orig_allowed_count;
        memcpy(circuit->allowed_ops, orig_allowed_ops, sizeof(orig_allowed_ops));
        return;
    }
    
    Circuit parent = *circuit;
    int best_active = circuit_count_active(&parent);
    
    int removed = evo_deep_prune(&parent, inputs, targets, masks, num_chunks_param, max_score);
    if (removed > 0) {
        best_active = circuit_count_active(&parent);
    }
    
    const int GENERIC_GENERATIONS = 250000;
    int gens_no_improvement = 0;
    double rewire_prob_local = 0.08;
    
    for (int gen = 0; gen < GENERIC_GENERATIONS; gen++) {
        Circuit child = parent;
        circuit_mutate(&child, rewire_prob_local);
        
        int child_score = evo_get_score(&child, inputs, targets, masks, num_chunks_param);
        if (child_score == max_score) {
            parent = child;
            Circuit temp = parent;
            circuit_compact(&temp);
            if (temp.num_gates < best_active) {
                best_active = temp.num_gates;
                *circuit = temp;
                gens_no_improvement = 0;
                
                evo_deep_prune(circuit, inputs, targets, masks, num_chunks_param, max_score);
                best_active = circuit->num_gates;
                parent = *circuit;
            }
        }
        
        gens_no_improvement++;
        if (gens_no_improvement > 60000) break;
    }
    
    circuit_compact(circuit);
    printf("      Run %d: %d gates\n", run_id, circuit->num_gates);
    
    circuit->allowed_ops_count = orig_allowed_count;
    memcpy(circuit->allowed_ops, orig_allowed_ops, sizeof(orig_allowed_ops));
}

void run_generic_evolution_phase_multistart(BitVec *inputs, BitVec *targets, BitVec *masks, int num_chunks_param, Circuit *circuit, Circuit *robdd_seed) {
    printf("\n[*] Running Multi-Start Generic Logic Pre-Optimization...\n");
    printf("    Initial: %d active gates\n", circuit_count_active(circuit));
    
    Circuit best_result = *circuit;
    int best_gates = circuit_count_active(&best_result);
    
    const int NUM_RUNS = 5;
    
    for (int run = 0; run < NUM_RUNS; run++) {
        Circuit trial = *robdd_seed;
        seed_rng(time(NULL) + run * 12345);
        run_generic_evolution_phase_single(inputs, targets, masks, num_chunks_param, &trial, run + 1);
        
        if (trial.num_gates < best_gates) {
            best_gates = trial.num_gates;
            best_result = trial;
            printf("    -> New best: %d gates\n", best_gates);
        }
    }
    
    *circuit = best_result;
    printf("    Final generic (best of %d): %d gates\n", NUM_RUNS, circuit->num_gates);
}

void run_evolutionary_refinement(BitVec *inputs, BitVec *targets, BitVec *masks, int num_chunks_param, Circuit *best_circuit) {
    printf("\n[*] Starting Enhanced Evolutionary Refinement (Strict Elitism + Soft Kicks)...\n");
    
    int max_score = 0;
    for(int i = 0; i < num_outputs; i++)
        for(int ch = 0; ch < num_chunks_param; ch++)
            max_score += __builtin_popcountll(masks[i].chunks[ch]);
    printf("    Target Score: %d\n", max_score);

    Circuit parent = *best_circuit; 
    int parent_score = evo_get_score(&parent, inputs, targets, masks, num_chunks_param);
    printf("    Seed Score: %d / %d (Gates: %d)\n", parent_score, max_score, parent.num_gates);

    bool seed_is_perfect = (parent_score == max_score);
    printf("    %s\n", seed_is_perfect ? "[OK] Seed is perfect! Deep optimization..." : "[!] Seed imperfect - GROWTH MODE");
    
    if (seed_is_perfect) {
        printf("    [*] Running initial deep pruning...\n");
        int removed = evo_deep_prune(&parent, inputs, targets, masks, num_chunks_param, max_score);
        if (removed > 0) printf("    -> Removed %d gates, now %d\n", removed, parent.num_gates);
        
        if (evo_algebraic_simplify(&parent, inputs, targets, masks, num_chunks_param, max_score)) {
            circuit_compact(&parent);
            printf("    -> After algebraic simplify: %d gates\n", parent.num_gates);
        }
        
        while (evo_try_merge_adjacent(&parent, inputs, targets, masks, num_chunks_param, max_score)) {
            printf("    -> Merged gates, now %d\n", parent.num_gates);
        }
        
        *best_circuit = parent;
    }
    
    int best_score = parent_score;
    int best_active = circuit_count_active(&parent);
    *best_circuit = parent;
    
    int gens_since_gate_imp = 0;
    int total_gens_no_improvement = 0;
    int gen = 0;
    bool found_perfect = seed_is_perfect;
    int last_deep_prune_gen = 0;
    int last_compact_gen = 0;
    int last_global_merge_gen = 0;
    int kick_count = 0;
    
    Circuit pre_kick_circuit;
    int pre_kick_gates = 0;
    int gens_since_kick = 0;
    bool in_kick_recovery = false;
    
    double rewire_prob_local = 0.05;
    
    while(1) {
        gen++;
        gens_since_gate_imp++;
        total_gens_no_improvement++;
        
        if (in_kick_recovery) gens_since_kick++;
        
        if (gens_since_gate_imp > 50000) {
            rewire_prob_local = 0.10 + 0.15 * (gens_since_gate_imp - 50000) / 150000.0;
            if (rewire_prob_local > 0.30) rewire_prob_local = 0.30;
        } else {
            rewire_prob_local = 0.05;
        }
        
        Circuit child = parent;
        int num_muts = (rand_double() < 0.85) ? 1 : randint(2, 4);
        for (int i = 0; i < num_muts; i++) circuit_mutate(&child, rewire_prob_local);
        
        int child_score = evo_get_score(&child, inputs, targets, masks, num_chunks_param);
        
        if (found_perfect) {
            if (child_score == max_score) {
                parent = child;
                parent_score = child_score;
                
                if (gen - last_compact_gen > 10000) {
                    circuit_compact_lazy(&parent, false);
                    last_compact_gen = gen;
                }
                
                Circuit temp = parent;
                circuit_compact(&temp);
                if (temp.num_gates < best_active) {
                    best_active = temp.num_gates;
                    *best_circuit = temp;
                    gens_since_gate_imp = 0;
                    total_gens_no_improvement = 0;
                    in_kick_recovery = false;
                    printf("    Gen %d: NEW BEST! %d gates.\n", gen, best_active);
                    
                    int removed = evo_deep_prune(best_circuit, inputs, targets, masks, num_chunks_param, max_score);
                    if (removed > 0) {
                        best_active = best_circuit->num_gates;
                        printf("    -> Deep pruned: %d gates\n", best_active);
                    }
                    
                    if (evo_algebraic_simplify(best_circuit, inputs, targets, masks, num_chunks_param, max_score)) {
                        circuit_compact(best_circuit);
                        if (best_circuit->num_gates < best_active) {
                            best_active = best_circuit->num_gates;
                            printf("    -> Algebraic: %d gates\n", best_active);
                        }
                    }
                    
                    parent = *best_circuit;
                }
            }
            
            if (in_kick_recovery && gens_since_kick > KICK_RECOVERY_LIMIT) {
                if (circuit_count_active(&parent) >= pre_kick_gates || parent_score < max_score) {
                    printf("    Gen %d: Kick #%d failed, reverting\n", gen, kick_count);
                    parent = pre_kick_circuit;
                    parent_score = max_score;
                }
                in_kick_recovery = false;
            }
            
            if (gen - last_deep_prune_gen > 100000) {
                Circuit temp = *best_circuit;
                int removed = evo_deep_prune(&temp, inputs, targets, masks, num_chunks_param, max_score);
                if (removed > 0 && temp.num_gates < best_active) {
                    best_active = temp.num_gates;
                    *best_circuit = temp;
                    parent = temp;
                    total_gens_no_improvement = 0;
                    printf("    Gen %d: Periodic prune: %d gates\n", gen, best_active);
                }
                last_deep_prune_gen = gen;
            }
            
            if (gen - last_global_merge_gen > GLOBAL_MERGE_INTERVAL) {
                Circuit temp = *best_circuit;
                if (evo_try_merge_global(&temp, inputs, targets, masks, num_chunks_param, max_score)) {
                    circuit_compact(&temp);
                    if (temp.num_gates < best_active) {
                        best_active = temp.num_gates;
                        *best_circuit = temp;
                        parent = temp;
                        total_gens_no_improvement = 0;
                        printf("    Gen %d: Global merge: %d gates\n", gen, best_active);
                    }
                }
                last_global_merge_gen = gen;
            }
            
            if (gens_since_gate_imp > KICK_THRESHOLD && !in_kick_recovery) {
                kick_count++;
                printf("    Gen %d: SOFT KICK #%d! (stuck for %d gens)\n", gen, kick_count, gens_since_gate_imp);
                
                pre_kick_circuit = *best_circuit;
                pre_kick_gates = best_active;
                
                parent = *best_circuit;
                double kick_intensity = 0.30 + 0.05 * (kick_count % 5);
                circuit_soft_kick(&parent, kick_intensity);
                parent_score = evo_get_score(&parent, inputs, targets, masks, num_chunks_param);
                
                in_kick_recovery = true;
                gens_since_kick = 0;
                gens_since_gate_imp = 0;
            }
            
            if (gens_since_gate_imp > OPTIMIZATION_PLATEAU_LIMIT && !in_kick_recovery) {
                parent = *best_circuit;
                for(int k = 0; k < randint(5, 12); k++) 
                    circuit_mutate(&parent, 0.15);
                parent_score = evo_get_score(&parent, inputs, targets, masks, num_chunks_param);
                gens_since_gate_imp = 0;
            }
            
            if (total_gens_no_improvement > TERMINATION_PLATEAU) {
                printf("\n*** CONVERGED ***\n");
                printf("Best solution (%d gates) hasn't improved for %d generations.\n", best_active, TERMINATION_PLATEAU);
                printf("Total kicks: %d\n", kick_count);
                break;
            }
        } else {
            if (child_score > parent_score) {
                parent = child;
                parent_score = child_score;
                
                if (child_score > best_score) {
                    best_score = child_score;
                    *best_circuit = child;
                    circuit_compact(best_circuit);
                    best_active = best_circuit->num_gates;
                    printf("    Gen %d: Score %d/%d (%d gates)\n", gen, best_score, max_score, best_active);
                }
                
                if (child_score == max_score) {
                    found_perfect = true;
                    printf("    Gen %d: PERFECT SOLUTION FOUND!\n", gen);
                    circuit_compact(&parent);
                    best_active = parent.num_gates;
                    *best_circuit = parent;
                    
                    int removed = evo_deep_prune(best_circuit, inputs, targets, masks, num_chunks_param, max_score);
                    if (removed > 0) {
                        best_active = best_circuit->num_gates;
                        printf("    -> Deep pruned: %d gates\n", best_active);
                    }
                    parent = *best_circuit;
                    gens_since_gate_imp = 0;
                    total_gens_no_improvement = 0;
                }
            } else if (child_score == parent_score && rand_double() < 0.05) {
                parent = child;
            }
            
            if (gens_since_gate_imp > STALL_LIMIT) {
                parent = *best_circuit;
                parent_score = best_score;
                circuit_add_random_gate(&parent);
                parent_score = evo_get_score(&parent, inputs, targets, masks, num_chunks_param);
                gens_since_gate_imp = 0;
            }
            
            if (gen > 10000000) {
                printf("\n*** TIMEOUT: Best score %d/%d ***\n", best_score, max_score);
                break;
            }
        }
        
        if (gen % 500000 == 0) {
            printf("    Gen %d: Best %d gates (rewire=%.2f%s)\n", gen, best_active, rewire_prob_local,
                   in_kick_recovery ? ", recovering" : "");
        }
    }
}

/* ============================================================
 * TEXT OUTPUT
 * ============================================================ */

void get_source_name(int idx, int num_inputs_local, Circuit *c, char *buffer) {
    if (idx < num_inputs_local) 
        sprintf(buffer, "Input (%d)", idx);
    else { 
        int g_idx = idx - num_inputs_local; 
        sprintf(buffer, "%s Gate (%d)", OP_NAMES[c->gates[g_idx].op], g_idx); 
    }
}

void render_circuit(Circuit *c) {
    printf("\n============================================================\n");
    printf(" NETLIST: %d GATES\n", c->num_gates);
    printf("============================================================\n");
    
    typedef struct { char src[64]; char dst[64]; } Line;
    Line *lines = (Line *)malloc(sizeof(Line) * (c->num_gates * 2 + c->num_outputs));
    int line_count = 0;
    
    for (int i = 0; i < c->num_gates; i++) {
        if (!c->gates[i].alive) continue;
        Gate *g = &c->gates[i]; 
        char dst_name[64];
        sprintf(dst_name, "%s Gate (%d)", OP_NAMES[g->op], i);
        get_source_name(g->src_a, c->num_inputs, c, lines[line_count].src);
        strcpy(lines[line_count].dst, dst_name); 
        line_count++;
        if (g->op != EVO_NOT) {
            get_source_name(g->src_b, c->num_inputs, c, lines[line_count].src);
            strcpy(lines[line_count].dst, dst_name); 
            line_count++;
        }
    }
    for (int i = 0; i < c->num_outputs; i++) {
        get_source_name(c->output_map[i], c->num_inputs, c, lines[line_count].src);
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
        for(int k = strlen(lines[i].src); k < col_width; k++) putchar(' ');
        printf("---->    %s\n", lines[i].dst);
    }
    printf("============================================================\n");
    free(lines);
}

/* ============================================================
 * DLS2 JSON EXPORT FUNCTIONS
 * ============================================================ */

static const char* skip_whitespace(const char* p) {
    while (*p && (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r')) p++;
    return p;
}

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

static int extract_int(const char* p) {
    while (*p && (*p < '0' || *p > '9') && *p != '-') p++;
    return atoi(p);
}

static bool load_gate_pins_from_file(int op_index, const char* filename) {
    FILE* f = fopen(filename, "r");
    if (!f) return false;

    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    fseek(f, 0, SEEK_SET);

    char* json = (char*)malloc(size + 1);
    if (!json) { fclose(f); return false; }
    fread(json, 1, size, f);
    json[size] = '\0';
    fclose(f);

    bool success = false;
    
    /* Declare all variables at start to avoid goto issues */
    const char* input_section = NULL;
    const char* output_section = NULL;
    const char* id_pos = NULL;
    const char* second_id = NULL;

    input_section = find_json_key(json, "InputPins");
    if (!input_section) goto cleanup;

    output_section = find_json_key(json, "OutputPins");
    if (!output_section) goto cleanup;

    id_pos = strstr(input_section, "\"ID\"");
    if (!id_pos || id_pos > output_section) goto cleanup;
    id_pos = find_json_key(id_pos, "ID");
    if (!id_pos) goto cleanup;
    DLS2_PIN_MAP[op_index].input_a = extract_int(id_pos);

    if (op_index != EVO_NOT) {
        second_id = strstr(id_pos + 1, "\"ID\"");
        if (second_id && second_id < output_section) {
            second_id = find_json_key(second_id, "ID");
            if (second_id) {
                DLS2_PIN_MAP[op_index].input_b = extract_int(second_id);
            }
        }
    } else {
        DLS2_PIN_MAP[op_index].input_b = -1;
    }

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

void load_dls2_pin_mappings(void) {
    printf("=== Loading DLS2 Gate Pin Mappings ===\n");

    printf("  [OK] NAND  : in_a=%d, in_b=%d, out=%d (built-in)\n",
           DLS2_PIN_MAP[EVO_NAND].input_a,
           DLS2_PIN_MAP[EVO_NAND].input_b,
           DLS2_PIN_MAP[EVO_NAND].output);

    int loaded_count = 1;

    for (int i = 0; i < EVO_NUM_OPS; i++) {
        if (i == EVO_NAND || GATE_FILENAMES[i] == NULL) continue;

        if (load_gate_pins_from_file(i, GATE_FILENAMES[i])) {
            loaded_count++;
            if (i == EVO_NOT) {
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

    printf("Loaded %d/%d gate definitions\n", loaded_count, EVO_NUM_OPS);
    printf("=======================================\n\n");
}

bool is_gate_available(int op) {
    return (op >= 0 && op < EVO_NUM_OPS && DLS2_PIN_MAP[op].loaded);
}

static LayoutConfig get_default_layout_config(void) {
    return (LayoutConfig){
        .gate_width = 0.85f,
        .gate_height = 0.55f,
        .column_spacing = 5.0f,
        .alley_width = 2.8f,
        .min_gate_v_spacing = 3.0f,
        .lane_spacing = 0.25f,
        .input_v_spacing = 1.5f,
        .output_v_spacing = 1.5f,
        .pin_margin_x = 4.5f,
        .chip_padding = 1.0f,
        .chip_color_r = 0.22f, 
        .chip_color_g = 0.32f, 
        .chip_color_b = 0.48f
    };
}

static void calculate_gate_depths(Circuit *c, int *depths) {
    for (int i = 0; i < c->num_gates; i++) {
        depths[i] = 0;
        if (!c->gates[i].alive) continue;
        Gate *g = &c->gates[i];

        if (g->src_a >= c->num_inputs) {
            int d = depths[g->src_a - c->num_inputs] + 1;
            if (d > depths[i]) depths[i] = d;
        }
        if (g->op != EVO_NOT && g->src_b >= c->num_inputs) {
            int d = depths[g->src_b - c->num_inputs] + 1;
            if (d > depths[i]) depths[i] = d;
        }
    }
}

static int get_max_depth(int *depths, int num_gates_local) {
    int max_d = 0;
    for (int i = 0; i < num_gates_local; i++) {
        if (depths[i] > max_d) max_d = depths[i];
    }
    return max_d;
}

static void calculate_grid_layout(int max_depth, LayoutConfig *cfg, GridLayout *grid) {
    grid->num_columns = max_depth + 1;
    grid->num_alleys = max_depth + 2;

    float total_width = max_depth * cfg->column_spacing;
    float start_x = -total_width / 2.0f;

    for (int d = 0; d <= max_depth; d++) {
        grid->column_x[d] = start_x + d * cfg->column_spacing;
    }

    float half_col_spacing = cfg->column_spacing / 2.0f;

    grid->alley_x[0] = grid->column_x[0] - half_col_spacing;
    for (int d = 1; d <= max_depth; d++) {
        grid->alley_x[d] = (grid->column_x[d-1] + grid->column_x[d]) / 2.0f;
    }
    grid->alley_x[max_depth + 1] = grid->column_x[max_depth] + half_col_spacing;

    for (int i = 0; i < grid->num_alleys; i++) {
        grid->alley_lane[i] = 0;
    }

    grid->input_x = grid->alley_x[0] - cfg->pin_margin_x;
    grid->output_x = grid->alley_x[grid->num_alleys - 1] + cfg->pin_margin_x;
}

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
    float used_y[100][50];
    int used_count[100] = {0};

    for (int i = 0; i < c->num_gates; i++) {
        if (!c->gates[i].alive) {
            gate_x[i] = 0;
            gate_y[i] = 0;
            continue;
        }
        
        Gate *g = &c->gates[i];
        int d = depths[i];

        gate_x[i] = grid->column_x[d];

        float target_y;
        if (g->src_a < c->num_inputs) {
            target_y = input_y[g->src_a];
        } else {
            target_y = gate_y[g->src_a - c->num_inputs];
        }

        bool collision = true;
        float best_y = target_y;
        float search_offset = 0.0f;
        int direction = 1;

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
                if (direction == 1) {
                    direction = -1;
                } else {
                    direction = 1;
                    search_offset += cfg->min_gate_v_spacing;
                }

                if (search_offset > 20.0f) {
                    best_y = target_y + used_count[d] * cfg->min_gate_v_spacing;
                    break;
                }
            }
        }

        gate_y[i] = best_y;

        if (used_count[d] < 50) {
            used_y[d][used_count[d]++] = best_y;
        }
    }
}

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

static void write_wire_alleyway(
    FILE *f,
    int src_pin, int src_owner,
    int tgt_pin, int tgt_owner,
    float src_x, float src_y,
    float tgt_x, float tgt_y,
    int src_depth,
    int tgt_depth,
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

    if (absf(dy) < 0.3f && absf(dx) < 1.5f) {
        fprintf(f, "      \"Points\":[{\"x\":0.0,\"y\":0.0},{\"x\":0.0,\"y\":0.0}]\n");
    } else {
        int alley_idx;

        if (src_depth == -1) {
            alley_idx = 0;
        }
        else if (tgt_depth == -2) {
            alley_idx = grid->num_alleys - 1;
        }
        else {
            alley_idx = (src_depth + tgt_depth + 1) / 2;
        }

        if (alley_idx < 0) alley_idx = 0;
        if (alley_idx >= grid->num_alleys) alley_idx = grid->num_alleys - 1;

        int lane = grid->alley_lane[alley_idx]++;

        float lane_offset = (lane % 2 == 0)
            ? (float)(lane / 2) * cfg->lane_spacing
            : -(float)((lane + 1) / 2) * cfg->lane_spacing;

        float alley_center = grid->alley_x[alley_idx];
        float route_x = alley_center + lane_offset;

        float half_alley = cfg->alley_width / 2.0f - 0.1f;
        route_x = maxf(alley_center - half_alley, minf(alley_center + half_alley, route_x));

        fprintf(f, "      \"Points\":[");
        fprintf(f, "{\"x\":0.0,\"y\":0.0},");
        fprintf(f, "{\"x\":%.4f,\"y\":%.4f},", route_x, src_y);
        fprintf(f, "{\"x\":%.4f,\"y\":%.4f},", route_x, tgt_y);
        fprintf(f, "{\"x\":0.0,\"y\":0.0}]\n");
    }

    fprintf(f, "    }%s\n", is_last ? "" : ",");
}

/* ============================================================
 * PROJECT DESCRIPTION UPDATER
 * ============================================================ */

static void get_iso_timestamp(char* buffer, size_t size) {
    time_t now = time(NULL);
    struct tm* tm_info = localtime(&now);
    strftime(buffer, size, "%Y-%m-%dT%H:%M:%S.000+00:00", tm_info);
}

static bool insert_at(char* json, size_t pos, const char* insert, size_t json_buf_size) {
    size_t json_len = strlen(json);
    size_t insert_len = strlen(insert);

    if (json_len + insert_len >= json_buf_size) {
        printf("Warning: Buffer too small for insertion\n");
        return false;
    }

    memmove(json + pos + insert_len, json + pos, json_len - pos + 1);
    memcpy(json + pos, insert, insert_len);
    return true;
}

static bool name_exists_in_range(const char* start, const char* end, const char* name) {
    if (!start || !end || end <= start) return false;

    char search[256];
    snprintf(search, sizeof(search), "\"%s\"", name);

    const char* found = start;
    while ((found = strstr(found, search)) != NULL) {
        if (found < end) {
            return true;
        }
        break;
    }
    return false;
}

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

bool update_project_description(const char* chip_name) {
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

    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    fseek(f, 0, SEEK_SET);

    if (size <= 0) {
        printf("Warning: ProjectDescription.json is empty or unreadable\n");
        fclose(f);
        return false;
    }

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
                    size_t new_len = strlen(timestamp);
                    memmove((char*)quote1 + 1 + new_len, quote2, strlen(quote2) + 1);
                    memcpy((char*)quote1 + 1, timestamp, new_len);
                    modified = true;
                }
            }
        }
    }

    const char* all_chips_key = strstr(json, "\"AllCustomChipNames\"");
    if (all_chips_key) {
        const char* arr_start = strchr(all_chips_key, '[');
        const char* arr_end = arr_start ? find_array_end(arr_start) : NULL;

        if (arr_start && arr_end) {
            if (name_exists_in_range(arr_start, arr_end, chip_name)) {
                printf("  [SKIP] AllCustomChipNames: '%s' already exists\n", chip_name);
            } else {
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

    const char* starred_key = strstr(json, "\"StarredList\"");
    if (starred_key) {
        const char* arr_start = strchr(starred_key, '[');
        const char* arr_end = arr_start ? find_array_end(arr_start) : NULL;

        if (arr_start && arr_end) {
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

    const char* other_name = strstr(json, "\"Name\":\"OTHER\"");
    if (other_name) {
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

/* ============================================================
 * MAIN DLS2 JSON EXPORT FUNCTION
 * ============================================================ */

void render_dls2_json(Circuit *c, int num_inputs_param, int num_outputs_param) {
    LayoutConfig cfg = get_default_layout_config();

    /* Validate pin mappings */
    for (int i = 0; i < c->num_gates; i++) {
        if (!c->gates[i].alive) continue;
        if (!DLS2_PIN_MAP[c->gates[i].op].loaded) {
            printf("Error: Gate %s (#%d) has no pin mapping!\n",
                   OP_NAMES[c->gates[i].op], i);
            return;
        }
    }

    char filename[256];
    snprintf(filename, sizeof(filename), "%s.json", g_chip_name);

    FILE *f = fopen(filename, "w");
    if (!f) {
        printf("Error: Cannot open %s\n", filename);
        return;
    }

    /* Calculate depths */
    int depths[MAX_EVOL_GATES];
    calculate_gate_depths(c, depths);
    int max_depth = get_max_depth(depths, c->num_gates);
    if (c->num_gates == 0) max_depth = 0;

    /* Setup grid layout */
    GridLayout grid;
    calculate_grid_layout(max_depth, &cfg, &grid);

    /* Allocate IDs */
    int input_ids[MAX_INPUTS];
    int output_ids[MAX_OUTPUTS];
    int gate_ids[MAX_EVOL_GATES];

    for (int i = 0; i < num_inputs_param; i++)  input_ids[i]  = 1000 + i;
    for (int i = 0; i < num_outputs_param; i++) output_ids[i] = 2000 + i;
    for (int i = 0; i < c->num_gates; i++)      gate_ids[i]   = 3000 + i;

    /* Input Y positions (centered) */
    float input_y[MAX_INPUTS];
    float total_input_height = (num_inputs_param - 1) * cfg.input_v_spacing;
    float input_start_y = total_input_height / 2.0f;

    for (int i = 0; i < num_inputs_param; i++) {
        input_y[i] = input_start_y - i * cfg.input_v_spacing;
    }

    /* Gate positions (with collision avoidance) */
    float gate_x[MAX_EVOL_GATES];
    float gate_y[MAX_EVOL_GATES];

    position_gates_with_collision_avoidance(
        c, depths, max_depth, input_y,
        gate_x, gate_y, &grid, &cfg
    );

    /* Output Y positions (centered) */
    float output_y[MAX_OUTPUTS];
    float total_output_height = (num_outputs_param - 1) * cfg.output_v_spacing;
    float output_start_y = total_output_height / 2.0f;

    for (int i = 0; i < num_outputs_param; i++) {
        output_y[i] = output_start_y - i * cfg.output_v_spacing;
    }

    /* Calculate bounding box */
    BoundingBox bbox;
    bbox_init(&bbox);

    for (int i = 0; i < num_inputs_param; i++) {
        bbox_add_point(&bbox, grid.input_x, input_y[i], 0.3f);
    }

    for (int i = 0; i < num_outputs_param; i++) {
        bbox_add_point(&bbox, grid.output_x, output_y[i], 0.3f);
    }

    for (int i = 0; i < c->num_gates; i++) {
        if (!c->gates[i].alive) continue;
        bbox_add_point(&bbox, gate_x[i], gate_y[i], cfg.gate_width / 2.0f + 0.2f);
    }

    for (int i = 0; i < grid.num_alleys; i++) {
        float alley_y_extent = maxf(absf(bbox.min_y), absf(bbox.max_y));
        bbox_add_point(&bbox, grid.alley_x[i], alley_y_extent, 0.1f);
        bbox_add_point(&bbox, grid.alley_x[i], -alley_y_extent, 0.1f);
    }

    /* Calculate dynamic chip size */
    float content_width = bbox.max_x - bbox.min_x;
    float content_height = bbox.max_y - bbox.min_y;

    float chip_width = (content_width / 12.0f) + cfg.chip_padding;
    float chip_height = (content_height / 8.0f) + cfg.chip_padding;

    chip_width = maxf(0.5f, minf(chip_width, 4.0f));
    chip_height = maxf(0.5f, minf(chip_height, 4.0f));

    /* JSON OUTPUT */
    fprintf(f, "{\n");
    fprintf(f, "  \"DLSVersion\": \"2.1.6\",\n");
    fprintf(f, "  \"Name\": \"%s\",\n", g_chip_name);
    fprintf(f, "  \"NameLocation\": 0,\n");
    fprintf(f, "  \"ChipType\": 0,\n");
    fprintf(f, "  \"Size\": {\"x\": %.3f, \"y\": %.3f},\n", chip_width, chip_height);
    fprintf(f, "  \"Colour\": {\"r\": %.3f, \"g\": %.3f, \"b\": %.3f, \"a\": 1},\n",
            cfg.chip_color_r, cfg.chip_color_g, cfg.chip_color_b);

    /* InputPins */
    fprintf(f, "  \"InputPins\":[\n");
    for (int i = 0; i < num_inputs_param; i++) {
        fprintf(f, "    {\n");
        fprintf(f, "      \"Name\":\"IN%d\",\n", i);
        fprintf(f, "      \"ID\":%d,\n", input_ids[i]);
        fprintf(f, "      \"Position\":{\"x\":%.4f,\"y\":%.4f},\n", grid.input_x, input_y[i]);
        fprintf(f, "      \"BitCount\":1,\n");
        fprintf(f, "      \"Colour\":0,\n");
        fprintf(f, "      \"ValueDisplayMode\":0\n");
        fprintf(f, "    }%s\n", (i < num_inputs_param - 1) ? "," : "");
    }
    fprintf(f, "  ],\n");

    /* OutputPins */
    fprintf(f, "  \"OutputPins\":[\n");
    for (int i = 0; i < num_outputs_param; i++) {
        fprintf(f, "    {\n");
        fprintf(f, "      \"Name\":\"OUT%d\",\n", i);
        fprintf(f, "      \"ID\":%d,\n", output_ids[i]);
        fprintf(f, "      \"Position\":{\"x\":%.4f,\"y\":%.4f},\n", grid.output_x, output_y[i]);
        fprintf(f, "      \"BitCount\":1,\n");
        fprintf(f, "      \"Colour\":0,\n");
        fprintf(f, "      \"ValueDisplayMode\":0\n");
        fprintf(f, "    }%s\n", (i < num_outputs_param - 1) ? "," : "");
    }
    fprintf(f, "  ],\n");

    /* SubChips (Gates) */
    fprintf(f, "  \"SubChips\":[\n");
    int gate_output_idx = 0;
    for (int i = 0; i < c->num_gates; i++) {
        if (!c->gates[i].alive) continue;
        Gate *g = &c->gates[i];
        fprintf(f, "    {\n");
        fprintf(f, "      \"Name\":\"%s\",\n", OP_NAMES[g->op]);
        fprintf(f, "      \"ID\":%d,\n", gate_ids[i]);
        fprintf(f, "      \"Label\":\"\",\n");
        fprintf(f, "      \"Position\":{\"x\":%.4f,\"y\":%.4f},\n", gate_x[i], gate_y[i]);
        fprintf(f, "      \"OutputPinColourInfo\":[{\"PinColour\":0,\"PinID\":%d}],\n",
                DLS2_PIN_MAP[g->op].output);
        fprintf(f, "      \"InternalData\":null\n");
        
        /* Check if this is the last alive gate */
        bool is_last_gate = true;
        for (int j = i + 1; j < c->num_gates; j++) {
            if (c->gates[j].alive) { is_last_gate = false; break; }
        }
        fprintf(f, "    }%s\n", is_last_gate ? "" : ",");
        gate_output_idx++;
    }
    fprintf(f, "  ],\n");

    /* Count total wires */
    int total_wires = num_outputs_param;
    for (int i = 0; i < c->num_gates; i++) {
        if (!c->gates[i].alive) continue;
        total_wires += (c->gates[i].op == EVO_NOT) ? 1 : 2;
    }

    /* Wires (with alleyway routing) */
    fprintf(f, "  \"Wires\":[\n");
    int wire_num = 0;

    /* Gate input wires */
    for (int i = 0; i < c->num_gates; i++) {
        if (!c->gates[i].alive) continue;
        Gate *g = &c->gates[i];
        int tgt_depth = depths[i];

        /* Wire A (primary input) */
        int src_a = g->src_a;
        float sx_a, sy_a;
        int pin_a, owner_a;
        int src_depth_a;

        if (src_a < num_inputs_param) {
            sx_a = grid.input_x;
            sy_a = input_y[src_a];
            pin_a = 0;
            owner_a = input_ids[src_a];
            src_depth_a = -1;
        } else {
            int idx = src_a - num_inputs_param;
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

        /* Wire B (secondary input, if not NOT gate) */
        if (g->op != EVO_NOT) {
            int src_b = g->src_b;
            float sx_b, sy_b;
            int pin_b, owner_b;
            int src_depth_b;

            if (src_b < num_inputs_param) {
                sx_b = grid.input_x;
                sy_b = input_y[src_b];
                pin_b = 0;
                owner_b = input_ids[src_b];
                src_depth_b = -1;
            } else {
                int idx = src_b - num_inputs_param;
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

    /* Output wires */
    for (int i = 0; i < num_outputs_param; i++) {
        int src = c->output_map[i];
        float sx, sy;
        int pin, owner;
        int src_depth;

        if (src < num_inputs_param) {
            sx = grid.input_x;
            sy = input_y[src];
            pin = 0;
            owner = input_ids[src];
            src_depth = -1;
        } else {
            int idx = src - num_inputs_param;
            sx = gate_x[idx];
            sy = gate_y[idx];
            pin = DLS2_PIN_MAP[c->gates[idx].op].output;
            owner = gate_ids[idx];
            src_depth = depths[idx];
        }

        wire_num++;
        write_wire_alleyway(f, pin, owner, 0, output_ids[i],
                            sx, sy, grid.output_x, output_y[i],
                            src_depth, -2,
                            &grid, &cfg,
                            wire_num == total_wires);
    }

    fprintf(f, "  ],\n");
    fprintf(f, "  \"Displays\": null\n");
    fprintf(f, "}\n");

    fclose(f);

    /* Print summary */
    printf("\n");
    printf("╔═══════════════════════════════════════════════════╗\n");
    printf("║       DLS2 GRID MANHATTAN ROUTING EXPORT          ║\n");
    printf("╠═══════════════════════════════════════════════════╣\n");
    printf("║  File: %-41s ║\n", filename);
    printf("║  Chip Size: %.2f x %.2f                           ║\n", chip_width, chip_height);
    printf("║  Gates: %-3d    Depths: %-3d    Wires: %-3d          ║\n",
           gate_output_idx, max_depth + 1, total_wires);
    printf("║  Columns: %-2d   Alleys: %-2d                         ║\n",
           grid.num_columns, grid.num_alleys);
    printf("║  Bounds: X[%.1f, %.1f]  Y[%.1f, %.1f]             ║\n",
           bbox.min_x, bbox.max_x, bbox.min_y, bbox.max_y);
    printf("╚═══════════════════════════════════════════════════╝\n");

    update_project_description(g_chip_name);
}

/* ============================================================
 * TRUTH TABLE PARSING
 * ============================================================ */

void parse_truth_table(const char *tt_str, TT **output_tables) {
    char tt_copy[131072]; 
    strncpy(tt_copy, tt_str, sizeof(tt_copy)-1);
    char *saveptr, *row_str = strtok_r(tt_copy, " \n\r\t,", &saveptr);
    int n_in = 0, n_out = 0;
    
    if (row_str) { 
        char *colon = strchr(row_str, ':'); 
        if (colon) { 
            n_in = (int)(colon - row_str); 
            n_out = strlen(colon + 1); 
        } 
    }
    num_inputs = n_in; 
    num_outputs = n_out;
    printf("[*] Problem: %d inputs, %d outputs\n", n_in, n_out);
    
    int total_rows = 1 << n_in;
    for(int i=0; i<n_out; i++) 
        output_tables[i] = create_tt(total_rows);
    
    strncpy(tt_copy, tt_str, sizeof(tt_copy)-1);
    row_str = strtok_r(tt_copy, " \n\r\t,", &saveptr);
    
    while (row_str) {
        char *colon = strchr(row_str, ':');
        if (colon) {
            *colon = '\0'; 
            char *rhs = colon + 1; 
            int row_idx = 0;
            for(int k=0; k<n_in; k++) 
                row_idx = (row_idx << 1) | (row_str[k] == '1');
            for (int i = 0; i < n_out; i++) 
                if (rhs[i] != 'X' && rhs[i] != 'x') 
                    tt_set_bit(output_tables[i], row_idx, rhs[i] == '1');
        }
        row_str = strtok_r(NULL, " \n\r\t,", &saveptr);
    }
}

/* ============================================================
 * MAIN FUNCTION
 * ============================================================ */

int main() {
    seed_rng(time(NULL));
    
    /* Load DLS2 gate pin mappings */
    load_dls2_pin_mappings();
    
    const char *truth_table = 
  "0000:1111110 0001:0110000 0010:1101101 0011:1111001 "
        "0100:0110011 0101:1011011 0110:1011111 0111:1110000 "
        "1000:1111111 1001:1111011 1010:XXXXXXX 1011:XXXXXXX "
        "1100:XXXXXXX 1101:XXXXXXX 1110:XXXXXXX 1111:XXXXXXX";

    const char *allowed_gates = "ALL";
    
    printf("============================================================\n");
    printf("   HYBRID ROBDD + CGP OPTIMIZER WITH DLS2 EXPORT           \n");
    printf("============================================================\n");

    parse_allowed_gates(allowed_gates);
    TT *outputs[MAX_OUTPUTS]; 
    parse_truth_table(truth_table, outputs);
    
    int total_rows = 1 << num_inputs;
    g_num_chunks = (total_rows + 63) / 64;
    memset(g_targets, 0, sizeof(g_targets)); 
    memset(g_masks, 0, sizeof(g_masks));
    
    char tt_copy[131072]; 
    strncpy(tt_copy, truth_table, sizeof(tt_copy)-1);
    char *saveptr, *row_str = strtok_r(tt_copy, " \n\r\t,", &saveptr);
    while(row_str) {
        char *colon = strchr(row_str, ':');
        if(colon) {
            *colon = '\0'; 
            char *rhs = colon + 1; 
            int row_idx = 0;
            for(int k = 0; k < num_inputs; k++) 
                row_idx = (row_idx << 1) | (row_str[k] == '1');
            int chunk = row_idx / 64, bit = row_idx % 64;
            for(int i = 0; i < num_outputs; i++) {
                if(rhs[i] != 'X' && rhs[i] != 'x') {
                    g_masks[i].chunks[chunk] |= (1ULL << bit);
                    if(rhs[i] == '1') 
                        g_targets[i].chunks[chunk] |= (1ULL << bit);
                }
            }
        }
        row_str = strtok_r(NULL, " \n\r\t,", &saveptr);
    }

    /* Initialize ROBDD logic gates */
    logic_gate_count = 0;
    for(int i = 0; i < num_inputs; i++) { 
        logic_gates[logic_gate_count] = (LogicGate){G_INPUT, i, 0, logic_gate_count}; 
        logic_gate_count++; 
    }
    memset(node_to_gate, -1, sizeof(node_to_gate));
    
    /* ROBDD synthesis with variable ordering optimization */
    synthesize_all_outputs_improved(outputs);
    
    for(int i = 0; i < num_outputs; i++) 
        free_tt(outputs[i]);
    
    printf("[*] Initial ROBDD: %d gates\n", logic_gate_count - num_inputs);
    
    /* Safe optimization loop */
    crush_circuit_loop();
    printf("[*] After optimization: %d gates\n", logic_gate_count - num_inputs);

    /* Bridge to evolutionary circuit */
    Circuit evo_circuit;
    load_evolutionary_from_robdd(&evo_circuit);
    printf("[*] Loaded into CGP: %d gates\n", evo_circuit.num_gates);

    /* Save ROBDD seed for multi-start */
    Circuit robdd_seed = evo_circuit;

    /* Prepare input vectors for evolutionary evaluation */
    BitVec inputs_vec[MAX_INPUTS]; 
    memset(inputs_vec, 0, sizeof(inputs_vec));
    for(int i = 0; i < num_inputs; i++)
        for(int r = 0; r < total_rows; r++)
            if((r >> (num_inputs - 1 - i)) & 1) 
                inputs_vec[i].chunks[r / 64] |= (1ULL << (r % 64));
    
    /* Multi-start generic phase */
    run_generic_evolution_phase_multistart(inputs_vec, g_targets, g_masks, g_num_chunks, &evo_circuit, &robdd_seed);

    /* Technology mapping */
    convert_to_allowed_gates_improved(&evo_circuit);
    circuit_compact(&evo_circuit);
    printf("[*] After gate conversion: %d gates\n", evo_circuit.num_gates);

    /* Verify bridge */
    int seed_score = evo_get_score(&evo_circuit, inputs_vec, g_targets, g_masks, g_num_chunks);
    int max_score = 0;
    for(int i = 0; i < num_outputs; i++) 
        for(int c = 0; c < g_num_chunks; c++) 
            max_score += __builtin_popcountll(g_masks[i].chunks[c]);
    printf("[*] Bridge verification: %d / %d %s\n", seed_score, max_score, 
           seed_score == max_score ? "(PERFECT)" : "(ERRORS!)");

    /* Evolutionary refinement */
    run_evolutionary_refinement(inputs_vec, g_targets, g_masks, g_num_chunks, &evo_circuit);
    
    /* Output results */
    render_circuit(&evo_circuit);
    
    /* Export to DLS2 JSON */
    render_dls2_json(&evo_circuit, num_inputs, num_outputs);

    return 0;
}
