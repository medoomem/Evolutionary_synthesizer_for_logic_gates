#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <math.h>


/* ============================================================
 * USER CONFIGURATION
 * ============================================================ */

/* Set to 1 to enable, 0 to disable */

#define ENABLE_EVOLUTIONARY_REFINEMENT  0    /* Run CGP optimization after synthesis */
#define ENABLE_STRUCTURAL_DETECTION     1    /* Try pattern detection before AIG */
#define ENABLE_TECHNOLOGY_MAPPING       1    /* Convert to allowed gate set */
#define ENABLE_DLS2_EXPORT              1    /* Export JSON for Digital Logic Sim 2 */
#define ENABLE_NETLIST_PRINT            1    /* Print circuit netlist to console */

/* Evolution parameters (only used if ENABLE_EVOLUTIONARY_REFINEMENT = 1) */
#define EVO_MAX_GENERATIONS         100000  /* Max generations before timeout */
#define EVO_TERMINATION_PLATEAU     10000  /* Stop if no improvement for this many gens */
#define EVO_KICK_THRESHOLD          200000   /* Generations before soft kick */
#define EVO_PRINT_INTERVAL          500000   /* Progress print frequency */

/* Anthropic: Configure chip name here or leave as "SYNTH" */
#define DEFAULT_CHIP_NAME           "SYNTH"





/* ============================================================
 * TECHNICAL CONFIGURATION & CONSTANTS
 * ============================================================ */

#define MAX_AIG_NODES 2000000
#define HASH_SIZE 65536
#define MAX_GATES 500000
#define MAX_EVOL_GATES 200000  /* Increased to prevent overflow during decomp */
#define MAX_CHUNKS 16384
#define MAX_WIRES (64 + MAX_EVOL_GATES)
#define MAX_INPUTS 32
#define MAX_OUTPUTS 64

#define STALL_LIMIT 50000              


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

#define DLS2_HEIGHT_MULTIPLIER 0.35f
#define DLS2_BASE_HEIGHT 0.5f
#define PROJECT_DESC_PATH "../ProjectDescription.json"

/* ============================================================
 * CORE DATA STRUCTURES
 * ============================================================ */

typedef struct { int num_bits, num_chunks; uint64_t *chunks; } TT;
typedef struct { uint64_t chunks[MAX_CHUNKS]; } BitVec;
typedef struct { int op, src_a, src_b, id; } LogicGate;

typedef struct { 
    uint8_t op; 
    int32_t src_a, src_b;  /* CHANGED: int16_t -> int32_t to handle >32k gates */
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

typedef struct GateHash { int op, a, b, id; struct GateHash *next; } GateHash;

typedef enum { G_INPUT, G_CONST, G_AND, G_OR, G_XOR, G_NOT } GateOpType;

/* ============================================================
 * AIG (AND-INVERTER GRAPH) STRUCTURES
 * ============================================================ */

/*
 * AIGER-style Literal Convention:
 * - Node 0 is constant FALSE
 * - Literal = 2*node for positive polarity
 * - Literal = 2*node + 1 for negative (complemented) polarity
 * - FALSE = 0, TRUE = 1
 * - Inputs are nodes 1..num_inputs
 * - AND gates are nodes (num_inputs+1), (num_inputs+2), ...
 */

#define AIG_FALSE       0
#define AIG_TRUE        1

#define AIG_NODE(lit)       ((lit) >> 1)
#define AIG_ISCOMPL(lit)    ((lit) & 1)
#define AIG_NOT(lit)        ((lit) ^ 1)
#define AIG_LIT(node, inv)  (((node) << 1) | ((inv) ? 1 : 0))
#define AIG_REGULAR(lit)    ((lit) & ~1)

typedef struct {
    int fanin0;
    int fanin1;
} AIGNode;

AIGNode aig_nodes[MAX_AIG_NODES];
int aig_node_count = 0;
int aig_num_inputs = 0;
int aig_num_ands = 0;

typedef struct AIGHashEntry {
    int f0, f1;
    int result_node;
    struct AIGHashEntry *next;
} AIGHashEntry;

AIGHashEntry *aig_strash[HASH_SIZE];

typedef struct AIGMemoEntry {
    TT *key;
    int result_lit;
    struct AIGMemoEntry *next;
} AIGMemoEntry;

AIGMemoEntry *aig_memo[HASH_SIZE];

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
    float column_x[500];      // Increased from 100
    float alley_x[501];       // Increased from 101
    int alley_lane[501];      // Increased from 101
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

int num_inputs = 0;
int num_outputs = 0;
int output_map_aig[MAX_OUTPUTS];

LogicGate logic_gates[MAX_GATES];
int logic_gate_count = 0;
int output_map_gates[MAX_OUTPUTS];

GateHash *gate_cache[HASH_SIZE];

BitVec g_targets[MAX_OUTPUTS], g_masks[MAX_OUTPUTS];
int g_num_chunks = 0;

int g_allowed_ops[EVO_NUM_OPS];
int g_allowed_ops_count = 0;

uint64_t rng_state;

static GatePinMapping DLS2_PIN_MAP[EVO_NUM_OPS] = {
    [EVO_AND]  = {0, 0, 0, false},
    [EVO_OR]   = {0, 0, 0, false},
    [EVO_XOR]  = {0, 0, 0, false},
    [EVO_NOT]  = {0, -1, 0, false},
    [EVO_NAND] = {0, 1, 2, true},
    [EVO_NOR]  = {0, 0, 0, false},
    [EVO_XNOR] = {0, 0, 0, false}
};

static const char* GATE_FILENAMES[EVO_NUM_OPS] = {
    "AND.json", "OR.json", "XOR.json", "NOT.json",
    NULL, "NOR.json", "XNOR.json"
};

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

TT* copy_tt(TT *src) {
    TT *t = create_tt(src->num_bits);
    memcpy(t->chunks, src->chunks, src->num_chunks * sizeof(uint64_t));
    return t;
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
    for(int i = 0; i < t->num_chunks; i++) {
        uint64_t k = t->chunks[i];
        for(int j = 0; j < 8; j++) { 
            hash ^= (k & 0xFF); 
            hash *= 1099511628211ULL; 
            k >>= 8; 
        }
    }
    return hash;
}

bool tt_is_const(TT *t, int *val) {
    bool all_zero = true, all_one = true;
    uint64_t mask = (t->num_bits % 64) ? ((1ULL << (t->num_bits % 64)) - 1) : ~0ULL;
    
    for(int i = 0; i < t->num_chunks; i++) {
        uint64_t m = (i == t->num_chunks - 1) ? mask : ~0ULL;
        if ((t->chunks[i] & m) != 0) all_zero = false;
        if ((t->chunks[i] & m) != m) all_one = false;
        if (!all_zero && !all_one) break;
    }
    
    if (all_zero) { *val = 0; return true; }
    if (all_one) { *val = 1; return true; }
    return false;
}

void tt_complement(TT *t) {
    for(int i = 0; i < t->num_chunks; i++)
        t->chunks[i] = ~t->chunks[i];
}

bool tt_equals(TT *a, TT *b) {
    if (a->num_bits != b->num_bits) return false;
    return memcmp(a->chunks, b->chunks, a->num_chunks * sizeof(uint64_t)) == 0;
}

/* ============================================================
 * GATE EVALUATION FUNCTIONS
 * ============================================================ */

void run_gate_op(int op, const TT *a, const TT *b, TT *result) {
    for(int c = 0; c < result->num_chunks; c++) {
        switch(op) {
            case G_AND: result->chunks[c] = a->chunks[c] & b->chunks[c]; break;
            case G_OR:  result->chunks[c] = a->chunks[c] | b->chunks[c]; break;
            case G_XOR: result->chunks[c] = a->chunks[c] ^ b->chunks[c]; break;
            case G_NOT: result->chunks[c] = ~a->chunks[c]; break;
        }
    }
}

void evo_run_gate(int op, const BitVec *a, const BitVec *b, BitVec *result, int num_chunks_param) {
    for(int c = 0; c < num_chunks_param; c++) {
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
 * AIG CORE OPERATIONS
 * ============================================================ */

void aig_init() {
    aig_node_count = num_inputs + 1;
    aig_num_inputs = num_inputs;
    aig_num_ands = 0;
    memset(aig_strash, 0, sizeof(aig_strash));
    memset(aig_memo, 0, sizeof(aig_memo));
}

void aig_reset_memo() {
    for(int i = 0; i < HASH_SIZE; i++) {
        AIGMemoEntry *e = aig_memo[i];
        while(e) {
            AIGMemoEntry *next = e->next;
            free_tt(e->key);
            free(e);
            e = next;
        }
        aig_memo[i] = NULL;
    }
}

void aig_reset_strash() {
    for(int i = 0; i < HASH_SIZE; i++) {
        AIGHashEntry *e = aig_strash[i];
        while(e) {
            AIGHashEntry *next = e->next;
            free(e);
            e = next;
        }
        aig_strash[i] = NULL;
    }
}

int aig_input_lit(int var_idx) {
    return AIG_LIT(var_idx + 1, 0);
}

uint64_t aig_strash_hash(int f0, int f1) {
    if (f0 > f1) { int t = f0; f0 = f1; f1 = t; }
    return ((uint64_t)f0 * 12582917ULL + (uint64_t)f1 * 4256249ULL) % HASH_SIZE;
}

int aig_strash_lookup(int f0, int f1) {
    if (f0 > f1) { int t = f0; f0 = f1; f1 = t; }
    uint64_t h = aig_strash_hash(f0, f1);
    AIGHashEntry *e = aig_strash[h];
    while(e) {
        if (e->f0 == f0 && e->f1 == f1) return e->result_node;
        e = e->next;
    }
    return -1;
}

void aig_strash_insert(int f0, int f1, int result_node) {
    if (f0 > f1) { int t = f0; f0 = f1; f1 = t; }
    uint64_t h = aig_strash_hash(f0, f1);
    AIGHashEntry *e = (AIGHashEntry*)malloc(sizeof(AIGHashEntry));
    e->f0 = f0;
    e->f1 = f1;
    e->result_node = result_node;
    e->next = aig_strash[h];
    aig_strash[h] = e;
}

int aig_and(int f0, int f1) {
    if (f0 == AIG_FALSE || f1 == AIG_FALSE) return AIG_FALSE;
    if (f0 == AIG_TRUE) return f1;
    if (f1 == AIG_TRUE) return f0;
    if (f0 == f1) return f0;
    if (f0 == AIG_NOT(f1)) return AIG_FALSE;
    
    if (f0 > f1) { int t = f0; f0 = f1; f1 = t; }
    
    int existing = aig_strash_lookup(f0, f1);
    if (existing >= 0) return AIG_LIT(existing, 0);
    
    int node_idx = aig_node_count++;
    aig_num_ands++;
    aig_nodes[node_idx].fanin0 = f0;
    aig_nodes[node_idx].fanin1 = f1;
    
    aig_strash_insert(f0, f1, node_idx);
    
    return AIG_LIT(node_idx, 0);
}

int aig_or(int f0, int f1) {
    return AIG_NOT(aig_and(AIG_NOT(f0), AIG_NOT(f1)));
}

int aig_xor(int f0, int f1) {
    int t1 = aig_and(f0, AIG_NOT(f1));
    int t2 = aig_and(AIG_NOT(f0), f1);
    return aig_or(t1, t2);
}

int aig_mux(int sel, int then_lit, int else_lit) {
    if (then_lit == else_lit) return then_lit;
    if (sel == AIG_TRUE) return then_lit;
    if (sel == AIG_FALSE) return else_lit;
    if (then_lit == AIG_TRUE && else_lit == AIG_FALSE) return sel;
    if (then_lit == AIG_FALSE && else_lit == AIG_TRUE) return AIG_NOT(sel);
    if (then_lit == AIG_TRUE) return aig_or(sel, else_lit);
    if (then_lit == AIG_FALSE) return aig_and(AIG_NOT(sel), else_lit);
    if (else_lit == AIG_TRUE) return aig_or(AIG_NOT(sel), then_lit);
    if (else_lit == AIG_FALSE) return aig_and(sel, then_lit);
    
    int t1 = aig_and(sel, then_lit);
    int t2 = aig_and(AIG_NOT(sel), else_lit);
    return aig_or(t1, t2);
}

/* ============================================================
 * AIG SYNTHESIS FROM TRUTH TABLE
 * ============================================================ */

TT* tt_cofactor(TT *f, int var_idx, int value) {
    int total_bits = f->num_bits;
    int half_bits = total_bits / 2;
    TT *result = create_tt(half_bits);
    
    int bit_pos = num_inputs - 1 - var_idx;
    
    for(int i = 0; i < half_bits; i++) {
        int low_part = i & ((1 << bit_pos) - 1);
        int high_part = (i >> bit_pos) << (bit_pos + 1);
        int orig_idx = high_part | (value << bit_pos) | low_part;
        
        if (tt_get_bit(f, orig_idx))
            tt_set_bit(result, i, 1);
    }
    
    return result;
}

int aig_memo_lookup(TT *t) {
    uint64_t h = hash_tt(t) % HASH_SIZE;
    AIGMemoEntry *e = aig_memo[h];
    while(e) {
        if (tt_equals(e->key, t)) {
            return e->result_lit;
        }
        e = e->next;
    }
    return -1;
}

void aig_memo_insert(TT *t, int lit) {
    uint64_t h = hash_tt(t) % HASH_SIZE;
    AIGMemoEntry *e = (AIGMemoEntry*)malloc(sizeof(AIGMemoEntry));
    e->key = copy_tt(t);
    e->result_lit = lit;
    e->next = aig_memo[h];
    aig_memo[h] = e;
}

int aig_memo_lookup_with_complement(TT *t, bool *is_complemented) {
    int result = aig_memo_lookup(t);
    if (result >= 0) {
        *is_complemented = false;
        return result;
    }
    
    TT *comp = copy_tt(t);
    tt_complement(comp);
    result = aig_memo_lookup(comp);
    free_tt(comp);
    
    if (result >= 0) {
        *is_complemented = true;
        return result;
    }
    
    *is_complemented = false;
    return -1;
}



/* ============================================================
 * IMPROVED AIG SYNTHESIS WITH MULTI-OUTPUT SHARING
 * ============================================================ */


/* ============================================================
 * STRUCTURAL ARITHMETIC SYNTHESIS
 * ============================================================ */

/* ============================================================
 * STRUCTURAL ARITHMETIC SYNTHESIS  
 * ============================================================ */












/* ============================================================
 * IMPROVED AIG SYNTHESIS WITH ADVANCED OPTIMIZATIONS
 * ============================================================ */

/* ==================== 
 * 1. BETTER VARIABLE ORDERING
 * ==================== */

/* Compute influence score: how much does variable affect the function? */
int compute_variable_influence(TT *t, int var_idx) {
    TT *f0 = tt_cofactor(t, var_idx, 0);
    TT *f1 = tt_cofactor(t, var_idx, 1);
    
    int diff_count = 0;
    for (int c = 0; c < f0->num_chunks; c++) {
        diff_count += __builtin_popcountll(f0->chunks[c] ^ f1->chunks[c]);
    }
    
    free_tt(f0);
    free_tt(f1);
    return diff_count;
}

/* Find best variable to split on (highest influence = most different cofactors) */
int find_best_split_variable(TT *t, bool *used_vars, int num_vars) {
    int best_var = -1;
    int best_score = -1;
    
    for (int v = 0; v < num_vars; v++) {
        if (used_vars[v]) continue;
        
        int score = compute_variable_influence(t, v);
        
        /* Prefer variables that create more different cofactors */
        if (score > best_score) {
            best_score = score;
            best_var = v;
        }
    }
    
    return best_var;
}

/* ==================== 
 * 2. XOR-AWARE SYNTHESIS
 * ==================== */

/* Check if function is exactly XOR of two variables */
bool tt_is_xor_of_vars(TT *t, int *var_a, int *var_b) {
    int total_rows = t->num_bits;
    int n = 0;
    while ((1 << n) < total_rows) n++;
    
    /* Try all pairs of variables */
    for (int a = 0; a < n; a++) {
        for (int b = a + 1; b < n; b++) {
            bool matches = true;
            for (int row = 0; row < total_rows && matches; row++) {
                int bit_a = (row >> (n - 1 - a)) & 1;
                int bit_b = (row >> (n - 1 - b)) & 1;
                int expected = bit_a ^ bit_b;
                if (tt_get_bit(t, row) != expected) matches = false;
            }
            if (matches) {
                *var_a = a;
                *var_b = b;
                return true;
            }
        }
    }
    return false;
}

/* Check if function is XOR of a variable and a subfunction */
bool tt_is_xor_with_var(TT *t, int var_idx, TT **subfunc) {
    TT *f0 = tt_cofactor(t, var_idx, 0);
    TT *f1 = tt_cofactor(t, var_idx, 1);
    
    /* f = var XOR g  means f0 = g and f1 = NOT(g) */
    TT *f1_comp = copy_tt(f1);
    tt_complement(f1_comp);
    
    bool is_xor = tt_equals(f0, f1_comp);
    
    if (is_xor) {
        *subfunc = f0;
        free_tt(f1);
    } else {
        free_tt(f0);
        *subfunc = NULL;
    }
    free_tt(f1_comp);
    
    return is_xor;
}

/* ==================== 
 * 3. ISOP (IRREDUNDANT SUM OF PRODUCTS) FOR SMALL FUNCTIONS
 * ==================== */

typedef struct {
    uint64_t onset;     /* Which minterms are in this cube */
    uint64_t care;      /* Which bits matter */
    int num_vars;
} Cube;

typedef struct {
    Cube *cubes;
    int num_cubes;
    int capacity;
} CubeList;

CubeList* create_cube_list(int capacity) {
    CubeList *cl = (CubeList*)malloc(sizeof(CubeList));
    cl->cubes = (Cube*)malloc(sizeof(Cube) * capacity);
    cl->num_cubes = 0;
    cl->capacity = capacity;
    return cl;
}

void free_cube_list(CubeList *cl) {
    if (cl) {
        free(cl->cubes);
        free(cl);
    }
}

/* Simple ISOP for small functions (up to 6 variables) */
CubeList* compute_isop_small(TT *t, int num_vars) {
    if (num_vars > 6) return NULL;  /* Only for small functions */
    
    int total_rows = 1 << num_vars;
    CubeList *result = create_cube_list(total_rows);
    
    /* Get onset (1s) and offset (0s) */
    uint64_t onset = 0, offset = 0;
    for (int r = 0; r < total_rows; r++) {
        if (tt_get_bit(t, r)) onset |= (1ULL << r);
        else offset |= (1ULL << r);
    }
    
    /* Greedy covering: find largest cubes that cover onset without touching offset */
    uint64_t uncovered = onset;
    
    while (uncovered) {
        int best_minterm = __builtin_ctzll(uncovered);
        
        /* Start with minterm as a cube, try to expand */
        uint64_t cube_covered = 1ULL << best_minterm;
        uint64_t cube_mask = (1ULL << num_vars) - 1;  /* All vars matter initially */
        
        /* Try removing each variable from care set */
        for (int v = 0; v < num_vars; v++) {
            uint64_t test_mask = cube_mask & ~(1ULL << v);
            
            /* Compute what this expanded cube would cover */
            uint64_t expanded = 0;
            for (int r = 0; r < total_rows; r++) {
                bool matches = true;
                for (int vi = 0; vi < num_vars; vi++) {
                    if (test_mask & (1ULL << vi)) {
                        int cube_bit = (best_minterm >> vi) & 1;
                        int row_bit = (r >> vi) & 1;
                        if (cube_bit != row_bit) matches = false;
                    }
                }
                if (matches) expanded |= (1ULL << r);
            }
            
            /* Accept expansion if it doesn't cover any offset */
            if ((expanded & offset) == 0) {
                cube_mask = test_mask;
                cube_covered = expanded;
            }
        }
        
        /* Add cube to result */
        if (result->num_cubes < result->capacity) {
            result->cubes[result->num_cubes].onset = best_minterm;
            result->cubes[result->num_cubes].care = cube_mask;
            result->cubes[result->num_cubes].num_vars = num_vars;
            result->num_cubes++;
        }
        
        uncovered &= ~cube_covered;
    }
    
    return result;
}

/* Build AIG from ISOP */
int aig_from_isop(CubeList *isop) {
    if (!isop || isop->num_cubes == 0) return AIG_FALSE;
    
    int or_result = AIG_FALSE;
    
    for (int i = 0; i < isop->num_cubes; i++) {
        Cube *c = &isop->cubes[i];
        int and_result = AIG_TRUE;
        
        for (int v = 0; v < c->num_vars; v++) {
            if (c->care & (1ULL << v)) {
                int var_lit = aig_input_lit(c->num_vars - 1 - v);
                int minterm_bit = (c->onset >> v) & 1;
                if (!minterm_bit) var_lit = AIG_NOT(var_lit);
                and_result = aig_and(and_result, var_lit);
            }
        }
        
        or_result = aig_or(or_result, and_result);
    }
    
    return or_result;
}

/* ==================== 
 * 4. IMPROVED MAIN SYNTHESIS WITH ALL OPTIMIZATIONS
 * ==================== */

int aig_synthesize_tt_improved(TT *t, bool *used_vars, int depth) {
    int const_val;
    
    /* Check for constants */
    if (tt_is_const(t, &const_val)) {
        return const_val ? AIG_TRUE : AIG_FALSE;
    }
    
    /* Check memo (with complement) */
    bool is_comp;
    int cached = aig_memo_lookup_with_complement(t, &is_comp);
    if (cached >= 0) {
        return is_comp ? AIG_NOT(cached) : cached;
    }
    
    /* Count remaining variables */
    int remaining_vars = 0;
    for (int v = 0; v < num_inputs; v++) {
        if (!used_vars[v]) remaining_vars++;
    }
    
    /* For small functions, try ISOP-based synthesis */
    if (remaining_vars <= 6 && remaining_vars > 0) {
        /* Build reduced truth table for remaining vars */
        int small_rows = 1 << remaining_vars;
        TT *small_tt = create_tt(small_rows);
        
        int var_map[16], map_idx = 0;
        for (int v = 0; v < num_inputs; v++) {
            if (!used_vars[v]) var_map[map_idx++] = v;
        }
        
        for (int r = 0; r < small_rows; r++) {
            int full_row = 0;
            for (int v = 0; v < num_inputs; v++) {
                if (used_vars[v]) continue;
                int small_bit = -1;
                for (int i = 0; i < remaining_vars; i++) {
                    if (var_map[i] == v) { small_bit = i; break; }
                }
                if (small_bit >= 0 && ((r >> (remaining_vars - 1 - small_bit)) & 1)) {
                    full_row |= (1 << (num_inputs - 1 - v));
                }
            }
            tt_set_bit(small_tt, r, tt_get_bit(t, full_row));
        }
        
        CubeList *isop = compute_isop_small(small_tt, remaining_vars);
        if (isop && isop->num_cubes > 0 && isop->num_cubes <= remaining_vars + 2) {
            /* Build AIG from ISOP with correct variable mapping */
            int or_result = AIG_FALSE;
            
            for (int i = 0; i < isop->num_cubes; i++) {
                Cube *c = &isop->cubes[i];
                int and_result = AIG_TRUE;
                
                for (int v = 0; v < remaining_vars; v++) {
                    if (c->care & (1ULL << v)) {
                        int orig_var = var_map[remaining_vars - 1 - v];
                        int var_lit = aig_input_lit(orig_var);
                        int minterm_bit = (c->onset >> v) & 1;
                        if (!minterm_bit) var_lit = AIG_NOT(var_lit);
                        and_result = aig_and(and_result, var_lit);
                    }
                }
                or_result = aig_or(or_result, and_result);
            }
            
            free_cube_list(isop);
            free_tt(small_tt);
            aig_memo_insert(t, or_result);
            return or_result;
        }
        free_cube_list(isop);
        free_tt(small_tt);
    }
    
    /* Find best variable to split on */
    int split_var = find_best_split_variable(t, used_vars, num_inputs);
    if (split_var < 0) {
        return AIG_FALSE;  /* No more variables */
    }
    
    /* Check for XOR pattern with this variable */
    TT *subfunc = NULL;
    if (tt_is_xor_with_var(t, split_var, &subfunc)) {
        bool new_used[16];
        memcpy(new_used, used_vars, sizeof(bool) * num_inputs);
        new_used[split_var] = true;
        
        int sub_lit = aig_synthesize_tt_improved(subfunc, new_used, depth + 1);
        free_tt(subfunc);
        
        int var_lit = aig_input_lit(split_var);
        int result = aig_xor(var_lit, sub_lit);
        aig_memo_insert(t, result);
        return result;
    }
    
    /* Standard Shannon decomposition */
    TT *f0 = tt_cofactor(t, split_var, 0);
    TT *f1 = tt_cofactor(t, split_var, 1);
    
    bool new_used[16];
    memcpy(new_used, used_vars, sizeof(bool) * num_inputs);
    new_used[split_var] = true;
    
    int var_lit = aig_input_lit(split_var);
    int result;
    
    /* Check if cofactors are equal */
    if (tt_equals(f0, f1)) {
        result = aig_synthesize_tt_improved(f0, new_used, depth + 1);
        free_tt(f0);
        free_tt(f1);
        aig_memo_insert(t, result);
        return result;
    }
    
    /* Check if cofactors are complements (XOR case) */
    TT *f0_comp = copy_tt(f0);
    tt_complement(f0_comp);
    if (tt_equals(f0_comp, f1)) {
        int lit0 = aig_synthesize_tt_improved(f0, new_used, depth + 1);
        result = aig_xor(var_lit, lit0);
        free_tt(f0);
        free_tt(f1);
        free_tt(f0_comp);
        aig_memo_insert(t, result);
        return result;
    }
    free_tt(f0_comp);
    
    /* Check for constant cofactors */
    int c0, c1;
    bool f0_const = tt_is_const(f0, &c0);
    bool f1_const = tt_is_const(f1, &c1);
    
    if (f0_const && f1_const) {
        if (c0 == 0 && c1 == 1) result = var_lit;
        else if (c0 == 1 && c1 == 0) result = AIG_NOT(var_lit);
        else if (c0 == 0) result = AIG_FALSE;
        else result = AIG_TRUE;
    }
    else if (f0_const) {
        int lit1 = aig_synthesize_tt_improved(f1, new_used, depth + 1);
        if (c0 == 0) result = aig_and(var_lit, lit1);
        else result = aig_or(AIG_NOT(var_lit), lit1);
    }
    else if (f1_const) {
        int lit0 = aig_synthesize_tt_improved(f0, new_used, depth + 1);
        if (c1 == 0) result = aig_and(AIG_NOT(var_lit), lit0);
        else result = aig_or(var_lit, lit0);
    }
    else {
        /* General MUX case */
        int lit0 = aig_synthesize_tt_improved(f0, new_used, depth + 1);
        int lit1 = aig_synthesize_tt_improved(f1, new_used, depth + 1);
        result = aig_mux(var_lit, lit1, lit0);
    }
    
    free_tt(f0);
    free_tt(f1);
    aig_memo_insert(t, result);
    return result;
}

/* ==================== 
 * 5. AIG REWRITING PASS
 * ==================== */

/* Local AIG patterns for rewriting */
typedef struct {
    int node;
    int new_lit;
    bool valid;
} RewriteResult;

/* Check if we can simplify AND(a, AND(a, b)) -> AND(a, b) */
RewriteResult try_rewrite_and_and(int node_idx) {
    RewriteResult r = {node_idx, -1, false};
    
    AIGNode *node = &aig_nodes[node_idx];
    int f0 = node->fanin0;
    int f1 = node->fanin1;
    
    /* Pattern: AND(a, AND(a, b)) = AND(a, b) */
    if (!AIG_ISCOMPL(f1)) {
        int child_node = AIG_NODE(f1);
        if (child_node > aig_num_inputs) {
            AIGNode *child = &aig_nodes[child_node];
            if (child->fanin0 == f0 || child->fanin1 == f0) {
                r.new_lit = f1;
                r.valid = true;
                return r;
            }
        }
    }
    
    if (!AIG_ISCOMPL(f0)) {
        int child_node = AIG_NODE(f0);
        if (child_node > aig_num_inputs) {
            AIGNode *child = &aig_nodes[child_node];
            if (child->fanin0 == f1 || child->fanin1 == f1) {
                r.new_lit = f0;
                r.valid = true;
                return r;
            }
        }
    }
    
    return r;
}

/* Check for AND(a, NOT(AND(a, b))) = AND(a, NOT(b)) */
RewriteResult try_rewrite_and_nand(int node_idx) {
    RewriteResult r = {node_idx, -1, false};
    
    AIGNode *node = &aig_nodes[node_idx];
    int f0 = node->fanin0;
    int f1 = node->fanin1;
    
    if (AIG_ISCOMPL(f1)) {
        int child_node = AIG_NODE(f1);
        if (child_node > aig_num_inputs) {
            AIGNode *child = &aig_nodes[child_node];
            
            /* AND(a, NOT(AND(a, b))) */
            if (AIG_REGULAR(child->fanin0) == AIG_REGULAR(f0)) {
                int other = child->fanin1;
                if (AIG_ISCOMPL(child->fanin0) != AIG_ISCOMPL(f0)) {
                    /* Complemented differently - becomes AND(a, NOT(b)) */
                    r.new_lit = aig_and(f0, AIG_NOT(other));
                    r.valid = true;
                    return r;
                }
            }
            if (AIG_REGULAR(child->fanin1) == AIG_REGULAR(f0)) {
                int other = child->fanin0;
                if (AIG_ISCOMPL(child->fanin1) != AIG_ISCOMPL(f0)) {
                    r.new_lit = aig_and(f0, AIG_NOT(other));
                    r.valid = true;
                    return r;
                }
            }
        }
    }
    
    return r;
}

/* Single pass of AIG rewriting */
int aig_rewrite_pass() {
    int rewrites = 0;
    int *new_mapping = (int*)malloc(sizeof(int) * aig_node_count);
    
    /* Initialize mapping to identity */
    for (int i = 0; i < aig_node_count; i++) {
        new_mapping[i] = AIG_LIT(i, 0);
    }
    
    /* Process nodes in topological order */
    for (int n = aig_num_inputs + 1; n < aig_node_count; n++) {
        /* Try rewriting patterns */
        RewriteResult r = try_rewrite_and_and(n);
        if (r.valid) {
            new_mapping[n] = r.new_lit;
            rewrites++;
            continue;
        }
        
        r = try_rewrite_and_nand(n);
        if (r.valid) {
            new_mapping[n] = r.new_lit;
            rewrites++;
        }
    }
    
    /* Apply mapping to outputs */
    if (rewrites > 0) {
        for (int i = 0; i < num_outputs; i++) {
            int old_lit = output_map_aig[i];
            int node = AIG_NODE(old_lit);
            if (node < aig_node_count && new_mapping[node] != AIG_LIT(node, 0)) {
                int new_lit = new_mapping[node];
                if (AIG_ISCOMPL(old_lit)) new_lit = AIG_NOT(new_lit);
                output_map_aig[i] = new_lit;
            }
        }
    }
    
    free(new_mapping);
    return rewrites;
}

/* ==================== 
 * 6. MULTI-OUTPUT SHARED SYNTHESIS
 * ==================== */

/* Build all outputs together, maximizing sharing */
void synthesize_multioutput_shared(TT **output_tables) {
    printf("\n[*] Multi-Output Shared AIG Synthesis\n");
    
    aig_init();
    
    /* First pass: find common subfunctions across outputs */
    /* Hash all intermediate cofactors and reuse */
    
    for (int i = 0; i < num_outputs; i++) {
        bool used_vars[16] = {false};
        output_map_aig[i] = aig_synthesize_tt_improved(output_tables[i], used_vars, 0);
    }
    
    printf("    Initial AIG: %d AND nodes\n", aig_num_ands);
    
    /* Apply rewriting passes */
    int total_rewrites = 0;
    for (int pass = 0; pass < 5; pass++) {
        int rewrites = aig_rewrite_pass();
        total_rewrites += rewrites;
        if (rewrites == 0) break;
    }
    
    if (total_rewrites > 0) {
        printf("    After rewriting: %d patterns simplified\n", total_rewrites);
    }
}



/* ============================================================
 * GATE CACHE FOR LOGIC GATES
 * ============================================================ */

void reset_gate_cache() { 
    for(int i = 0; i < HASH_SIZE; i++) {
        GateHash *e = gate_cache[i];
        while(e) {
            GateHash *next = e->next;
            free(e);
            e = next;
        }
        gate_cache[i] = NULL;
    }
}

int find_existing_gate(int op, int a, int b) {
    if ((op == G_AND || op == G_OR || op == G_XOR) && a > b) { 
        int t = a; a = b; b = t; 
    }
    uint64_t h = ((uint64_t)op * 2654435761ULL) ^ ((uint64_t)(a+1000) * 2654435761ULL) ^ ((uint64_t)(b+1000) * 2654435761ULL);
    int idx = h % HASH_SIZE; 
    GateHash *e = gate_cache[idx];
    while(e) { 
        if(e->op == op && e->a == a && e->b == b) return e->id; 
        e = e->next; 
    }
    return -1;
}

void add_to_gate_cache(int op, int a, int b, int id) {
    if ((op == G_AND || op == G_OR || op == G_XOR) && a > b) { 
        int t = a; a = b; b = t; 
    }
    uint64_t h = ((uint64_t)op * 2654435761ULL) ^ ((uint64_t)(a+1000) * 2654435761ULL) ^ ((uint64_t)(b+1000) * 2654435761ULL);
    int idx = h % HASH_SIZE; 
    GateHash *new_e = (GateHash*)malloc(sizeof(GateHash));
    new_e->op = op; new_e->a = a; new_e->b = b; new_e->id = id; 
    new_e->next = gate_cache[idx]; 
    gate_cache[idx] = new_e;
}

int find_or_add_gate(int op, int a, int b) {
    if ((op == G_AND || op == G_OR || op == G_XOR) && a > b) { 
        int t = a; a = b; b = t; 
    }
    
    if (op == G_AND && a == b) return a; 
    if (op == G_OR && a == b) return a;
    
    int existing = find_existing_gate(op, a, b); 
    if(existing != -1) return existing;
    
    int id = logic_gate_count; 
    logic_gates[id] = (LogicGate){op, a, b, id}; 
    logic_gate_count++;
    add_to_gate_cache(op, a, b, id); 
    return id;
}

/* ============================================================
 * AIG TO LOGIC GATES CONVERSION
 * ============================================================ */

int *aig_to_gate_map = NULL;

int convert_aig_lit_to_gate(int lit);

int convert_aig_node_to_gate(int node_idx) {
    if (node_idx == 0) {
        if (aig_to_gate_map[0] == -1) {
            aig_to_gate_map[0] = find_or_add_gate(G_CONST, 0, 0);
        }
        return aig_to_gate_map[0];
    }
    
    if (aig_to_gate_map[node_idx] != -1) {
        return aig_to_gate_map[node_idx];
    }
    
    if (node_idx >= 1 && node_idx <= aig_num_inputs) {
        aig_to_gate_map[node_idx] = node_idx - 1;
        return aig_to_gate_map[node_idx];
    }
    
    AIGNode *node = &aig_nodes[node_idx];
    
    int gate_a = convert_aig_lit_to_gate(node->fanin0);
    int gate_b = convert_aig_lit_to_gate(node->fanin1);
    
    int result = find_or_add_gate(G_AND, gate_a, gate_b);
    aig_to_gate_map[node_idx] = result;
    return result;
}

int convert_aig_lit_to_gate(int lit) {
    if (lit == AIG_FALSE) {
        return convert_aig_node_to_gate(0);
    }
    if (lit == AIG_TRUE) {
        int zero_gate = convert_aig_node_to_gate(0);
        return find_or_add_gate(G_NOT, zero_gate, 0);
    }
    
    int node_idx = AIG_NODE(lit);
    int gate = convert_aig_node_to_gate(node_idx);
    
    if (AIG_ISCOMPL(lit)) {
        return find_or_add_gate(G_NOT, gate, 0);
    }
    return gate;
}

void convert_aig_to_gates() {
    logic_gate_count = 0;
    reset_gate_cache();
    
    for(int i = 0; i < num_inputs; i++) {
        logic_gates[logic_gate_count] = (LogicGate){G_INPUT, i, 0, logic_gate_count};
        logic_gate_count++;
    }
    
    int max_nodes = aig_node_count + 10;
    aig_to_gate_map = (int*)malloc(sizeof(int) * max_nodes);
    for(int i = 0; i < max_nodes; i++) {
        aig_to_gate_map[i] = -1;
    }
    
    for(int i = 1; i <= aig_num_inputs; i++) {
        aig_to_gate_map[i] = i - 1;
    }
    
    for(int i = 0; i < num_outputs; i++) {
        output_map_gates[i] = convert_aig_lit_to_gate(output_map_aig[i]);
    }
    
    free(aig_to_gate_map);
    aig_to_gate_map = NULL;
}

/* ============================================================
 * PATTERN RECOGNITION (recover XOR, OR from AND/NOT)
 * ============================================================ */

void recognize_patterns_in_gates() {
    printf("[*] Running pattern recognition...\n");
    
    int or_count = 0, xor_count = 0;
    bool changed = true;
    
    while (changed) {
        changed = false;
        
        for (int i = num_inputs; i < logic_gate_count; i++) {
            LogicGate *g = &logic_gates[i];
            
            if (g->op == G_NOT) {
                int inner = g->src_a;
                if (inner >= num_inputs && logic_gates[inner].op == G_AND) {
                    int a = logic_gates[inner].src_a;
                    int b = logic_gates[inner].src_b;
                    if (a >= num_inputs && logic_gates[a].op == G_NOT &&
                        b >= num_inputs && logic_gates[b].op == G_NOT) {
                        g->op = G_OR;
                        g->src_a = logic_gates[a].src_a;
                        g->src_b = logic_gates[b].src_a;
                        or_count++;
                        changed = true;
                    }
                }
            }
        }
    }
    
    printf("    Recognized: %d OR gates, %d XOR gates\n", or_count, xor_count);
}

/* ============================================================
 * COMPACT GATES (remove unused)
 * ============================================================ */

int compact_logic_gates() {
    int *remap = (int*)malloc(sizeof(int) * logic_gate_count);
    memset(remap, -1, sizeof(int) * logic_gate_count);
    bool *used = (bool*)calloc(logic_gate_count, sizeof(bool));
    int *stack = (int*)malloc(sizeof(int) * logic_gate_count);
    int sp = 0;
    
    for(int i = 0; i < num_outputs; i++) {
        int g = output_map_gates[i];
        if (g >= 0 && g < logic_gate_count && !used[g]) { 
            used[g] = true; 
            stack[sp++] = g; 
        }
    }
    
    while(sp > 0) {
        int g = stack[--sp];
        if (logic_gates[g].op != G_INPUT && logic_gates[g].op != G_CONST) {
            int a = logic_gates[g].src_a;
            if (a >= 0 && a < logic_gate_count && !used[a]) { 
                used[a] = true; 
                stack[sp++] = a; 
            }
            if (logic_gates[g].op != G_NOT) {
                int b = logic_gates[g].src_b;
                if (b >= 0 && b < logic_gate_count && !used[b]) { 
                    used[b] = true; 
                    stack[sp++] = b; 
                }
            }
        }
    }
    
    LogicGate *new_gates = (LogicGate*)malloc(sizeof(LogicGate) * MAX_GATES);
    reset_gate_cache();
    int new_cnt = 0;
    
    for(int i = 0; i < logic_gate_count; i++) {
        if (!used[i]) continue;
        
        if (logic_gates[i].op == G_INPUT) {
            new_gates[new_cnt] = logic_gates[i];
            new_gates[new_cnt].id = new_cnt;
            remap[i] = new_cnt++;
        } else if (logic_gates[i].op == G_CONST) {
            int existing = find_existing_gate(G_CONST, logic_gates[i].src_a, 0);
            if (existing != -1) {
                remap[i] = existing;
            } else {
                new_gates[new_cnt] = logic_gates[i];
                new_gates[new_cnt].id = new_cnt;
                add_to_gate_cache(G_CONST, logic_gates[i].src_a, 0, new_cnt);
                remap[i] = new_cnt++;
            }
        } else {
            int new_a = remap[logic_gates[i].src_a];
            int new_b = (logic_gates[i].op != G_NOT) ? remap[logic_gates[i].src_b] : 0;
            
            if (new_a < 0) new_a = 0;
            if (new_b < 0) new_b = 0;
            
            int existing = find_existing_gate(logic_gates[i].op, new_a, new_b);
            if (existing != -1) {
                remap[i] = existing;
            } else {
                new_gates[new_cnt] = (LogicGate){logic_gates[i].op, new_a, new_b, new_cnt};
                add_to_gate_cache(logic_gates[i].op, new_a, new_b, new_cnt);
                remap[i] = new_cnt++;
            }
        }
    }
    
    for(int i = 0; i < num_outputs; i++) {
        if (output_map_gates[i] >= 0 && remap[output_map_gates[i]] >= 0) {
            output_map_gates[i] = remap[output_map_gates[i]];
        }
    }
    
    memcpy(logic_gates, new_gates, sizeof(LogicGate) * new_cnt);
    
    free(new_gates);
    free(remap);
    free(used);
    free(stack);
    
    return new_cnt;
}

/* ============================================================
 * MAIN AIG SYNTHESIS FLOW
 * ============================================================ */


/* Simple working synthesis with XOR detection */
int aig_synthesize_tt(TT *t, int depth) {
    int const_val;
    
    if (tt_is_const(t, &const_val)) {
        return const_val ? AIG_TRUE : AIG_FALSE;
    }
    
    bool is_comp;
    int cached = aig_memo_lookup_with_complement(t, &is_comp);
    if (cached >= 0) {
        return is_comp ? AIG_NOT(cached) : cached;
    }
    
    if (depth >= num_inputs) {
        return AIG_FALSE;
    }
    
    TT *f0 = tt_cofactor(t, depth, 0);
    TT *f1 = tt_cofactor(t, depth, 1);
    
    int var_lit = aig_input_lit(depth);
    int result;
    
    /* Equal cofactors - variable doesn't matter */
    if (tt_equals(f0, f1)) {
        result = aig_synthesize_tt(f0, depth + 1);
        free_tt(f0);
        free_tt(f1);
        aig_memo_insert(t, result);
        return result;
    }
    
    /* XOR DETECTION: if f1 = NOT(f0), then f = var XOR f0 */
    TT *f0_comp = copy_tt(f0);
    tt_complement(f0_comp);
    if (tt_equals(f0_comp, f1)) {
        int lit0 = aig_synthesize_tt(f0, depth + 1);
        result = aig_xor(var_lit, lit0);
        free_tt(f0);
        free_tt(f1);
        free_tt(f0_comp);
        aig_memo_insert(t, result);
        return result;
    }
    free_tt(f0_comp);
    
    /* Constant cofactors */
    int c0, c1;
    bool f0_const = tt_is_const(f0, &c0);
    bool f1_const = tt_is_const(f1, &c1);
    
    if (f0_const && f1_const) {
        if (c0 == 0 && c1 == 1) result = var_lit;
        else if (c0 == 1 && c1 == 0) result = AIG_NOT(var_lit);
        else if (c0 == 0) result = AIG_FALSE;
        else result = AIG_TRUE;
    }
    else if (f0_const) {
        int lit1 = aig_synthesize_tt(f1, depth + 1);
        if (c0 == 0) result = aig_and(var_lit, lit1);
        else result = aig_or(AIG_NOT(var_lit), lit1);
    }
    else if (f1_const) {
        int lit0 = aig_synthesize_tt(f0, depth + 1);
        if (c1 == 0) result = aig_and(AIG_NOT(var_lit), lit0);
        else result = aig_or(var_lit, lit0);
    }
    else {
        int lit0 = aig_synthesize_tt(f0, depth + 1);
        int lit1 = aig_synthesize_tt(f1, depth + 1);
        result = aig_mux(var_lit, lit1, lit0);
    }
    
    free_tt(f0);
    free_tt(f1);
    aig_memo_insert(t, result);
    return result;
}

void synthesize_with_aig(TT **output_tables) {
    printf("\n[*] AIG Synthesis\n");
    
    aig_init();
    
    for(int i = 0; i < num_outputs; i++) {
        output_map_aig[i] = aig_synthesize_tt(output_tables[i], 0);
    }
    
    printf("    AIG: %d AND nodes\n", aig_num_ands);
    
    convert_aig_to_gates();
    printf("    Logic gates (raw): %d\n", logic_gate_count - num_inputs);
    
    recognize_patterns_in_gates();
    
    logic_gate_count = compact_logic_gates();
    printf("    Logic gates (compacted): %d\n", logic_gate_count - num_inputs);
    
    aig_reset_memo();
    aig_reset_strash();
}

/* ============================================================
 * VERIFICATION
 * ============================================================ */

int verify_circuit_score() {
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
    
    TT zero_tt;
    zero_tt.num_bits = total_rows;
    zero_tt.num_chunks = num_chunks_local;
    zero_tt.chunks = (uint64_t*)calloc(num_chunks_local, sizeof(uint64_t));
    
    for(int i = 0; i < logic_gate_count; i++) {
        wire_results[i].num_bits = total_rows;
        wire_results[i].num_chunks = num_chunks_local;
        wire_results[i].chunks = (uint64_t*)calloc(num_chunks_local, sizeof(uint64_t));
        
        if (logic_gates[i].op == G_INPUT) {
            memcpy(wire_results[i].chunks, inputs_tt[logic_gates[i].src_a].chunks, 
                   num_chunks_local * sizeof(uint64_t));
        } else if (logic_gates[i].op == G_CONST) {
            if (logic_gates[i].src_a == 1)
                memset(wire_results[i].chunks, 0xFF, num_chunks_local * sizeof(uint64_t));
        }
    }
    
    for (int i = num_inputs; i < logic_gate_count; i++) {
        if(logic_gates[i].op == G_INPUT || logic_gates[i].op == G_CONST) continue;
        LogicGate *g = &logic_gates[i];
        if (g->op == G_NOT)
            run_gate_op(g->op, &wire_results[g->src_a], &zero_tt, &wire_results[i]);
        else
            run_gate_op(g->op, &wire_results[g->src_a], &wire_results[g->src_b], &wire_results[i]);
    }
    
    int total_correct = 0;
    for(int out = 0; out < num_outputs; out++) {
        TT *result = &wire_results[output_map_gates[out]];
        for(int ch = 0; ch < num_chunks_local; ch++) {
            uint64_t correct = ~(result->chunks[ch] ^ g_targets[out].chunks[ch]) & g_masks[out].chunks[ch];
            total_correct += __builtin_popcountll(correct);
        }
    }
    
    free(zero_tt.chunks);
    for(int i = 0; i < num_inputs; i++) free(inputs_tt[i].chunks);
    for(int i = 0; i < logic_gate_count; i++) free(wire_results[i].chunks);
    free(wire_results);
    
    return total_correct;
}

/* ============================================================
 * BRIDGE: LOGIC GATES TO EVOLUTIONARY CIRCUIT
 * ============================================================ */

void load_evolutionary_from_gates(Circuit *dest) {
    dest->num_inputs = num_inputs;
    dest->num_outputs = num_outputs;
    dest->num_gates = 0;
    dest->allowed_ops_count = g_allowed_ops_count;
    memcpy(dest->allowed_ops, g_allowed_ops, sizeof(g_allowed_ops));
    dest->dead_count = 0;
    
    /* FIX: Heap allocation instead of int remap[MAX_GATES] which is 2MB! */
    int *remap = (int*)malloc(sizeof(int) * MAX_GATES);
    if (!remap) {
        printf("[!] Memory allocation failed in load_evolutionary_from_gates\n");
        return;
    }
    
    for(int i = 0; i < MAX_GATES; i++) remap[i] = -999;
    for(int i = 0; i < num_inputs; i++) remap[i] = i;
    
    for(int i = num_inputs; i < logic_gate_count; i++) {
        if(logic_gates[i].op == G_CONST)
            remap[i] = (logic_gates[i].src_a == 1) ? CONST_ONE : CONST_ZERO;
    }
    
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
                    dest->gates[dest->num_gates] = (Gate){EVO_NOT, (int32_t)wire_b, 0, true};
                    remap[i] = num_inputs + dest->num_gates++;
                    folded = true;
                } else if (val_b == 1 && wire_a >= 0) {
                    dest->gates[dest->num_gates] = (Gate){EVO_NOT, (int32_t)wire_a, 0, true};
                    remap[i] = num_inputs + dest->num_gates++;
                    folded = true;
                }
            }
            if (folded) { changed = true; continue; }
            
            if (wire_a >= 0 && (g.op == G_NOT || wire_b >= 0)) {
                uint8_t evo_op = (g.op == G_AND) ? EVO_AND : (g.op == G_OR) ? EVO_OR :
                                 (g.op == G_XOR) ? EVO_XOR : (g.op == G_NOT) ? EVO_NOT : 255;
                if (evo_op == 255) continue;
                dest->gates[dest->num_gates] = (Gate){evo_op, (int32_t)wire_a, (int32_t)wire_b, true};
                remap[i] = num_inputs + dest->num_gates++;
                changed = true;
            }
        }
    }
    
    for(int i = 0; i < num_outputs; i++) {
        int wire = remap[output_map_gates[i]];
        if (wire == CONST_ZERO) {
            dest->gates[dest->num_gates] = (Gate){EVO_XOR, 0, 0, true};
            wire = num_inputs + dest->num_gates++;
        }
        else if (wire == CONST_ONE) {
            dest->gates[dest->num_gates] = (Gate){EVO_NOT, 0, 0, true};
            int not_idx = dest->num_gates++;
            dest->gates[dest->num_gates] = (Gate){EVO_OR, 0, (int32_t)(num_inputs + not_idx), true};
            wire = num_inputs + dest->num_gates++;
        } else if (wire < 0) {
            printf("    [ERROR] Output %d unresolved (wire=%d)!\n", i, wire);
            wire = 0;
        }
        dest->output_map[i] = wire;
    }
    
    free(remap);  /* FIX: Free heap allocation */
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
    
    /* FIX: Heap Allocation */
    Gate *old_gates = (Gate*)malloc(sizeof(Gate) * c->num_gates);
    if (!old_gates) { printf("[!] Alloc failed in convert\n"); return; }
    
    int old_num_gates = c->num_gates;
    memcpy(old_gates, c->gates, sizeof(Gate) * old_num_gates);
    
    int old_outputs[MAX_OUTPUTS];
    memcpy(old_outputs, c->output_map, sizeof(int) * c->num_outputs);
    
    /* FIX: Heap Allocation */
    int *wire_map = (int*)malloc(sizeof(int) * MAX_WIRES);
    if (!wire_map) { free(old_gates); return; }

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
                case EVO_NOT: new_wire = circuit_add_gate_alive(c, EVO_NAND, a, a); break;
                case EVO_AND: { int t = circuit_add_gate_alive(c, EVO_NAND, a, b); new_wire = circuit_add_gate_alive(c, EVO_NAND, t, t); break; }
                case EVO_OR: { int na = circuit_add_gate_alive(c, EVO_NAND, a, a); int nb = circuit_add_gate_alive(c, EVO_NAND, b, b); new_wire = circuit_add_gate_alive(c, EVO_NAND, na, nb); break; }
                case EVO_XOR: build_xor_from_nand(c, a, b, &new_wire); break;
                case EVO_NOR: { int na = circuit_add_gate_alive(c, EVO_NAND, a, a); int nb = circuit_add_gate_alive(c, EVO_NAND, b, b); int or_ab = circuit_add_gate_alive(c, EVO_NAND, na, nb); new_wire = circuit_add_gate_alive(c, EVO_NAND, or_ab, or_ab); break; }
                case EVO_XNOR: { int xor_result; build_xor_from_nand(c, a, b, &xor_result); new_wire = circuit_add_gate_alive(c, EVO_NAND, xor_result, xor_result); break; }
                default: new_wire = a;
            }
        }
        else if (has_nor) {
            switch (g->op) {
                case EVO_NOT: new_wire = circuit_add_gate_alive(c, EVO_NOR, a, a); break;
                case EVO_OR: { int t = circuit_add_gate_alive(c, EVO_NOR, a, b); new_wire = circuit_add_gate_alive(c, EVO_NOR, t, t); break; }
                case EVO_AND: { int na = circuit_add_gate_alive(c, EVO_NOR, a, a); int nb = circuit_add_gate_alive(c, EVO_NOR, b, b); new_wire = circuit_add_gate_alive(c, EVO_NOR, na, nb); break; }
                case EVO_XOR: build_xor_from_nor(c, a, b, &new_wire); break;
                case EVO_NAND: { int na = circuit_add_gate_alive(c, EVO_NOR, a, a); int nb = circuit_add_gate_alive(c, EVO_NOR, b, b); int and_ab = circuit_add_gate_alive(c, EVO_NOR, na, nb); new_wire = circuit_add_gate_alive(c, EVO_NOR, and_ab, and_ab); break; }
                case EVO_XNOR: { int xor_result; build_xor_from_nor(c, a, b, &xor_result); new_wire = circuit_add_gate_alive(c, EVO_NOR, xor_result, xor_result); break; }
                default: new_wire = a;
            }
        }
        else {
            switch (g->op) {
                case EVO_NOT: new_wire = a; break;
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
                default: new_wire = a;
            }
        }
        
        wire_map[old_wire] = new_wire;
    }
    
    for (int i = 0; i < c->num_outputs; i++) {
        c->output_map[i] = wire_map[old_outputs[i]];
    }
    
    free(old_gates);
    free(wire_map);
    printf("    [*] Conversion complete: %d gates (was %d)\n", c->num_gates, old_num_gates);
}

/* ============================================================
 * EVOLUTIONARY CGP ENGINE
 * ============================================================ */

int evo_get_score(Circuit *c, BitVec *inputs, BitVec *targets, BitVec *masks, int num_chunks_param) {
    int num_wires = c->num_inputs + c->num_gates;
    
    /* Dynamic allocation - only allocate what we need */
    uint64_t *wire_data = (uint64_t*)calloc((size_t)num_wires * num_chunks_param, sizeof(uint64_t));
    if (!wire_data) { 
        printf("[!] evo_get_score: Memory allocation failed (%d wires, %d chunks)\n", 
               num_wires, num_chunks_param);
        return 0; 
    }
    
    /* Helper macro to access wire data */
    #define WIRE(w, ch) wire_data[(w) * num_chunks_param + (ch)]
    
    /* Copy inputs */
    for (int i = 0; i < c->num_inputs; i++) {
        for (int ch = 0; ch < num_chunks_param; ch++) {
            WIRE(i, ch) = inputs[i].chunks[ch];
        }
    }
    
    /* Evaluate gates */
    for (int i = 0; i < c->num_gates; i++) {
        if (!c->gates[i].alive) continue;
        Gate *g = &c->gates[i];
        int wire_out = c->num_inputs + i;
        int a = g->src_a;
        int b = g->src_b;
        
        for (int ch = 0; ch < num_chunks_param; ch++) {
            uint64_t va = WIRE(a, ch);
            uint64_t vb = (g->op == EVO_NOT) ? 0 : WIRE(b, ch);
            uint64_t result;
            
            switch(g->op) {
                case EVO_AND:  result = va & vb; break;
                case EVO_OR:   result = va | vb; break;
                case EVO_XOR:  result = va ^ vb; break;
                case EVO_NOT:  result = ~va; break;
                case EVO_NAND: result = ~(va & vb); break;
                case EVO_NOR:  result = ~(va | vb); break;
                case EVO_XNOR: result = ~(va ^ vb); break;
                default:       result = 0; break;
            }
            WIRE(wire_out, ch) = result;
        }
    }
    
    /* Score outputs */
    int total = 0;
    for (int i = 0; i < c->num_outputs; i++) {
        int out_wire = c->output_map[i];
        for (int ch = 0; ch < num_chunks_param; ch++) {
            uint64_t out_val = WIRE(out_wire, ch);
            uint64_t matches = ~(out_val ^ targets[i].chunks[ch]) & masks[i].chunks[ch];
            total += __builtin_popcountll(matches);
        }
    }
    
    #undef WIRE
    free(wire_data);
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
    
    /* FIX: Heap Allocation */
    int *active = (int*)malloc(sizeof(int) * MAX_EVOL_GATES);
    if (!active) return false;

    int active_count = circuit_get_active_indices(c, active);
    if (active_count == c->num_gates && alive_count == c->num_gates) {
        free(active);
        return false;
    }
    
    int *old_to_new = (int*)malloc(sizeof(int) * MAX_WIRES);
    Gate *new_gates = (Gate*)malloc(sizeof(Gate) * MAX_EVOL_GATES);
    
    if (!old_to_new || !new_gates) {
        free(active); if(old_to_new) free(old_to_new); if(new_gates) free(new_gates);
        return false;
    }

    for (int i = 0; i < c->num_inputs; i++) old_to_new[i] = i;
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
    
    free(active);
    free(old_to_new);
    free(new_gates);
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

int evo_deep_prune(Circuit *c, BitVec *inputs, BitVec *targets, BitVec *masks, int num_chunks_param, int max_score) {
    int removed = 0;
    bool improved = true;
    
    /* FIX: Heap Allocation for large structures */
    int *active = (int*)malloc(sizeof(int) * MAX_EVOL_GATES);
    Circuit *backup = (Circuit*)malloc(sizeof(Circuit));
    
    if (!active || !backup) {
        if(active) free(active);
        if(backup) free(backup);
        printf("[!] Memory allocation failed in pruning\n");
        return 0;
    }
    
    while (improved) {
        improved = false;
        int active_count = circuit_get_active_indices(c, active);
        
        for (int k = active_count - 1; k >= 0; k--) {
            int gate_idx = active[k];
            int wire_idx = c->num_inputs + gate_idx;
            Gate *gate = &c->gates[gate_idx];
            
            /* Save current state to heap backup */
            memcpy(backup, c, sizeof(Circuit));
            
            /* --- ATTEMPT 1: Replace gate with its input A --- */
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
                improved = true; removed++; 
                /* Success! Keep 'c' as is, don't restore backup */
                break; 
            }
            
            /* Failed, restore from backup */
            memcpy(c, backup, sizeof(Circuit));
            
            /* --- ATTEMPT 2: Replace gate with its input B (if not NOT gate) --- */
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
                    improved = true; removed++; 
                    break; 
                }
                
                /* Failed, restore */
                memcpy(c, backup, sizeof(Circuit));
            }
        }
    }
    
    free(active);
    free(backup);
    circuit_compact(c);
    return removed;
}

void run_evolutionary_refinement(BitVec *inputs, BitVec *targets, BitVec *masks, int num_chunks_param, Circuit *best_circuit) {
    printf("\n[*] Starting Evolutionary Refinement...\n");
    
    int max_score = 0;
    for(int i = 0; i < num_outputs; i++)
        for(int ch = 0; ch < num_chunks_param; ch++)
            max_score += __builtin_popcountll(masks[i].chunks[ch]);
    printf("    Target Score: %d\n", max_score);

    /* FIX: Heap allocate Circuit structs (each is ~2-3MB!) */
    Circuit *parent = (Circuit*)malloc(sizeof(Circuit));
    Circuit *child = (Circuit*)malloc(sizeof(Circuit));
    
    if (!parent || !child) {
        printf("[!] Memory allocation failed in evolutionary refinement\n");
        if (parent) free(parent);
        if (child) free(child);
        return;
    }
    
    memcpy(parent, best_circuit, sizeof(Circuit));
    
    int parent_score = evo_get_score(parent, inputs, targets, masks, num_chunks_param);
    printf("    Seed Score: %d / %d (Gates: %d)\n", parent_score, max_score, parent->num_gates);

    bool seed_is_perfect = (parent_score == max_score);
    
    if (seed_is_perfect) {
        printf("    [*] Running initial deep pruning...\n");
        int removed = evo_deep_prune(parent, inputs, targets, masks, num_chunks_param, max_score);
        if (removed > 0) printf("    -> Removed %d gates, now %d\n", removed, parent->num_gates);
        memcpy(best_circuit, parent, sizeof(Circuit));
    }
    
    int best_score = parent_score;
    int best_active = circuit_count_active(parent);
    memcpy(best_circuit, parent, sizeof(Circuit));
    
    int gens_since_gate_imp = 0;
    int gen = 0;
    bool found_perfect = seed_is_perfect;
    double rewire_prob_local = 0.05;
    
    const int MAX_GENS = EVO_MAX_GENERATIONS;
    
    while(gen < MAX_GENS) {
        gen++;
        gens_since_gate_imp++;
        
        memcpy(child, parent, sizeof(Circuit));
        int num_muts = (rand_double() < 0.85) ? 1 : randint(2, 4);
        for (int i = 0; i < num_muts; i++) circuit_mutate(child, rewire_prob_local);
        
        int child_score = evo_get_score(child, inputs, targets, masks, num_chunks_param);
        
        if (found_perfect) {
            if (child_score == max_score) {
                memcpy(parent, child, sizeof(Circuit));
                
                Circuit *temp = (Circuit*)malloc(sizeof(Circuit));
                if (temp) {
                    memcpy(temp, parent, sizeof(Circuit));
                    circuit_compact(temp);
                    if (temp->num_gates < best_active) {
                        best_active = temp->num_gates;
                        memcpy(best_circuit, temp, sizeof(Circuit));
                        gens_since_gate_imp = 0;
                        printf("    Gen %d: NEW BEST! %d gates.\n", gen, best_active);
                        
                        int removed = evo_deep_prune(best_circuit, inputs, targets, masks, num_chunks_param, max_score);
                        if (removed > 0) {
                            best_active = best_circuit->num_gates;
                            printf("    -> Deep pruned: %d gates\n", best_active);
                        }
                        memcpy(parent, best_circuit, sizeof(Circuit));
                    }
                    free(temp);
                }
            }
            
            if (gens_since_gate_imp > EVO_TERMINATION_PLATEAU) {
                printf("    Converged at %d gates after %d generations.\n", best_active, gen);
                break;
            }
        } else {
            if (child_score > parent_score) {
                memcpy(parent, child, sizeof(Circuit));
                parent_score = child_score;
                
                if (child_score > best_score) {
                    best_score = child_score;
                    memcpy(best_circuit, child, sizeof(Circuit));
                    circuit_compact(best_circuit);
                    best_active = best_circuit->num_gates;
                    printf("    Gen %d: Score %d/%d (%d gates)\n", gen, best_score, max_score, best_active);
                }
                
                if (child_score == max_score) {
                    found_perfect = true;
                    printf("    Gen %d: PERFECT SOLUTION FOUND!\n", gen);
                    circuit_compact(parent);
                    best_active = parent->num_gates;
                    memcpy(best_circuit, parent, sizeof(Circuit));
                    
                    int removed = evo_deep_prune(best_circuit, inputs, targets, masks, num_chunks_param, max_score);
                    if (removed > 0) {
                        best_active = best_circuit->num_gates;
                        printf("    -> Deep pruned: %d gates\n", best_active);
                    }
                    memcpy(parent, best_circuit, sizeof(Circuit));
                    gens_since_gate_imp = 0;
                }
            }
            
            if (gens_since_gate_imp > STALL_LIMIT) {
                memcpy(parent, best_circuit, sizeof(Circuit));
                parent_score = best_score;
                circuit_add_random_gate(parent);
                parent_score = evo_get_score(parent, inputs, targets, masks, num_chunks_param);
                gens_since_gate_imp = 0;
            }
        }
        
        if (gen % 100000 == 0) {
            printf("    Gen %d: Best %d gates\n", gen, best_active);
        }
    }
    
    printf("    Final: %d gates\n", best_circuit->num_gates);
    
    /* FIX: Free heap allocations */
    free(parent);
    free(child);
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

    // We look specifically for the start of the Pins arrays to avoid 
    // picking up IDs from the Chip size or coordinates
    const char* input_section = strstr(json, "\"InputPins\"");
    const char* output_section = strstr(json, "\"OutputPins\"");
    
    if (!input_section || !output_section) {
        free(json);
        return false;
    }

    // Find first Input ID
    const char* p = strstr(input_section, "\"ID\"");
    if (p && p < output_section) DLS2_PIN_MAP[op_index].input_a = extract_int(p + 4);

    // Find second Input ID (for non-NOT gates)
    if (op_index != EVO_NOT) {
        p = strstr(p + 4, "\"ID\"");
        if (p && p < output_section) DLS2_PIN_MAP[op_index].input_b = extract_int(p + 4);
    } else {
        DLS2_PIN_MAP[op_index].input_b = -1;
    }

    // Find Output ID
    p = strstr(output_section, "\"ID\"");
    if (p) DLS2_PIN_MAP[op_index].output = extract_int(p + 4);

    free(json);
    DLS2_PIN_MAP[op_index].loaded = true;
    return true;
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
    /* FIX: Dynamic allocation based on actual depth count */
    int num_cols = max_depth + 1;
    int max_per_col = 100;  /* Max gates expected per column */
    
    float *used_y = (float*)calloc((size_t)num_cols * max_per_col, sizeof(float));
    int *used_count = (int*)calloc(num_cols, sizeof(int));
    
    if (!used_y || !used_count) {
        printf("[!] Allocation failed in position_gates\n");
        if (used_y) free(used_y);
        if (used_count) free(used_count);
        return;
    }
    
    #define USED_Y(col, idx) used_y[(col) * max_per_col + (idx)]

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

            for (int j = 0; j < used_count[d] && j < max_per_col; j++) {
                if (absf(test_y - USED_Y(d, j)) < cfg->min_gate_v_spacing) {
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

                if (search_offset > 50.0f) {
                    best_y = target_y + used_count[d] * cfg->min_gate_v_spacing;
                    break;
                }
            }
        }

        gate_y[i] = best_y;

        if (used_count[d] < max_per_col) {
            USED_Y(d, used_count[d]) = best_y;
            used_count[d]++;
        }
    }
    
    #undef USED_Y
    free(used_y);
    free(used_count);
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

static void get_iso_timestamp(char* buffer, size_t size) {
    time_t now = time(NULL);
    struct tm* tm_info = localtime(&now);
    strftime(buffer, size, "%Y-%m-%dT%H:%M:%S.000+00:00", tm_info);
}

static bool insert_at(char* json, size_t pos, const char* insert, size_t json_buf_size) {
    size_t json_len = strlen(json);
    size_t insert_len = strlen(insert);

    if (json_len + insert_len >= json_buf_size) {
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
        return false;
    }

    FILE* f = fopen(PROJECT_DESC_PATH, "r");
    if (!f) {
        printf("Warning: Could not open %s\n", PROJECT_DESC_PATH);
        return false;
    }

    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    fseek(f, 0, SEEK_SET);

    if (size <= 0) {
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
            if (!name_exists_in_range(arr_start, arr_end, chip_name)) {
                snprintf(insert_buf, sizeof(insert_buf), ",\n    \"%s\"", chip_name);
                size_t insert_pos = arr_end - json;
                if (insert_at(json, insert_pos, insert_buf, buf_size)) {
                    printf("  [ADD]  AllCustomChipNames: '%s'\n", chip_name);
                    modified = true;
                }
            }
        }
    }

    const char* starred_key = strstr(json, "\"StarredList\"");
    if (starred_key) {
        const char* arr_start = strchr(starred_key, '[');
        const char* arr_end = arr_start ? find_array_end(arr_start) : NULL;

        if (arr_start && arr_end) {
            char name_search[256];
            snprintf(name_search, sizeof(name_search), "\"Name\":\"%s\"", chip_name);

            const char* found = strstr(arr_start, name_search);
            if (!found || found >= arr_end) {
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
                if (!name_exists_in_range(chips_arr_start, chips_arr_end, chip_name)) {
                    snprintf(insert_buf, sizeof(insert_buf), ",\"%s\"", chip_name);
                    size_t insert_pos = chips_arr_end - json;
                    if (insert_at(json, insert_pos, insert_buf, buf_size)) {
                        printf("  [ADD]  OTHER collection: '%s'\n", chip_name);
                        modified = true;
                    }
                }
            }
        }
    }

    if (modified) {
        f = fopen(PROJECT_DESC_PATH, "w");
        if (!f) {
            free(json);
            return false;
        }
        fputs(json, f);
        fclose(f);
        printf("ProjectDescription.json updated successfully!\n");
    }

    free(json);
    return true;
}

/* ============================================================
 * MAIN DLS2 JSON EXPORT FUNCTION
 * ============================================================ */

void render_dls2_json(Circuit *c, int num_inputs_param, int num_outputs_param) {
    LayoutConfig cfg = get_default_layout_config();

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

    /* FIX: Heap allocation to prevent stack overflow */
    int alloc_size = (c->num_gates > 0) ? c->num_gates : 1;
    
    int *depths = (int*)calloc(alloc_size, sizeof(int));
    float *gate_x = (float*)calloc(alloc_size, sizeof(float));
    float *gate_y = (float*)calloc(alloc_size, sizeof(float));
    int *gate_ids = (int*)malloc(sizeof(int) * alloc_size);
    int *input_ids = (int*)malloc(sizeof(int) * num_inputs_param);
    int *output_ids = (int*)malloc(sizeof(int) * num_outputs_param);
    float *input_y = (float*)malloc(sizeof(float) * num_inputs_param);
    float *output_y = (float*)malloc(sizeof(float) * num_outputs_param);
    
    if (!depths || !gate_x || !gate_y || !gate_ids || !input_ids || !output_ids || !input_y || !output_y) {
        printf("Error: Memory allocation failed in render_dls2_json\n");
        if (depths) free(depths);
        if (gate_x) free(gate_x);
        if (gate_y) free(gate_y);
        if (gate_ids) free(gate_ids);
        if (input_ids) free(input_ids);
        if (output_ids) free(output_ids);
        if (input_y) free(input_y);
        if (output_y) free(output_y);
        fclose(f);
        return;
    }

    calculate_gate_depths(c, depths);
    int max_depth = get_max_depth(depths, c->num_gates);
    if (c->num_gates == 0) max_depth = 0;

    GridLayout grid;
    calculate_grid_layout(max_depth, &cfg, &grid);

    for (int i = 0; i < num_inputs_param; i++)  input_ids[i]  = 1000 + i;
    for (int i = 0; i < num_outputs_param; i++) output_ids[i] = 2000 + i;
    for (int i = 0; i < c->num_gates; i++)      gate_ids[i]   = 3000 + i;

    float total_input_height = (num_inputs_param - 1) * cfg.input_v_spacing;
    float input_start_y = total_input_height / 2.0f;

    for (int i = 0; i < num_inputs_param; i++) {
        input_y[i] = input_start_y - i * cfg.input_v_spacing;
    }

    position_gates_with_collision_avoidance(
        c, depths, max_depth, input_y,
        gate_x, gate_y, &grid, &cfg
    );

    float total_output_height = (num_outputs_param - 1) * cfg.output_v_spacing;
    float output_start_y = total_output_height / 2.0f;

    for (int i = 0; i < num_outputs_param; i++) {
        output_y[i] = output_start_y - i * cfg.output_v_spacing;
    }

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

    float content_width = bbox.max_x - bbox.min_x;
    float content_height = bbox.max_y - bbox.min_y;

    float chip_width = (content_width / 12.0f) + cfg.chip_padding;
    float chip_height = (content_height / 8.0f) + cfg.chip_padding;

    chip_width = maxf(0.5f, minf(chip_width, 4.0f));
    chip_height = maxf(0.5f, minf(chip_height, 4.0f));

    fprintf(f, "{\n");
    fprintf(f, "  \"DLSVersion\": \"2.1.6\",\n");
    fprintf(f, "  \"Name\": \"%s\",\n", g_chip_name);
    fprintf(f, "  \"NameLocation\": 0,\n");
    fprintf(f, "  \"ChipType\": 0,\n");
    fprintf(f, "  \"Size\": {\"x\": %.3f, \"y\": %.3f},\n", chip_width, chip_height);
    fprintf(f, "  \"Colour\": {\"r\": %.3f, \"g\": %.3f, \"b\": %.3f, \"a\": 1},\n",
            cfg.chip_color_r, cfg.chip_color_g, cfg.chip_color_b);

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
        
        bool is_last_gate = true;
        for (int j = i + 1; j < c->num_gates; j++) {
            if (c->gates[j].alive) { is_last_gate = false; break; }
        }
        fprintf(f, "    }%s\n", is_last_gate ? "" : ",");
        gate_output_idx++;
    }
    fprintf(f, "  ],\n");

    int total_wires = num_outputs_param;
    for (int i = 0; i < c->num_gates; i++) {
        if (!c->gates[i].alive) continue;
        total_wires += (c->gates[i].op == EVO_NOT) ? 1 : 2;
    }

    fprintf(f, "  \"Wires\":[\n");
    int wire_num = 0;

    for (int i = 0; i < c->num_gates; i++) {
        if (!c->gates[i].alive) continue;
        Gate *g = &c->gates[i];
        int tgt_depth = depths[i];

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

    printf("\n");
    printf("╔═══════════════════════════════════════════════════════════╗\n");
    printf("║       DLS2 GRID MANHATTAN ROUTING EXPORT                  ║\n");
    printf("╠═══════════════════════════════════════════════════════════╣\n");
    printf("║  File: %-49s ║\n", filename);
    printf("║  Chip Size: %.2f x %.2f                                   ║\n", chip_width, chip_height);
    printf("║  Gates: %-3d    Depths: %-3d    Wires: %-3d                  ║\n",
           gate_output_idx, max_depth + 1, total_wires);
    printf("╚═══════════════════════════════════════════════════════════╝\n");

    update_project_description(g_chip_name);

    /* Free heap allocations */
    free(depths);
    free(gate_x);
    free(gate_y);
    free(gate_ids);
    free(input_ids);
    free(output_ids);
    free(input_y);
    free(output_y);
}

/* ============================================================
 * TRUTH TABLE PARSING
 * ============================================================ */

void parse_truth_table(const char *tt_str, TT **output_tables) {
    char *tt_copy = strdup(tt_str); 
    if (!tt_copy) { printf("[!] Memory allocation failed\n"); exit(1); }
    
    char *saveptr;
    char *row_str = strtok_r(tt_copy, " \n\r\t,", &saveptr);
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
    for(int i = 0; i < n_out; i++) 
        output_tables[i] = create_tt(total_rows);
    
    free(tt_copy);
    tt_copy = strdup(tt_str);
    if (!tt_copy) { printf("[!] Memory allocation failed\n"); exit(1); }
    
    row_str = strtok_r(tt_copy, " \n\r\t,", &saveptr);
    while (row_str) {
        char *colon = strchr(row_str, ':');
        if (colon) {
            *colon = '\0'; 
            char *rhs = colon + 1; 
            int row_idx = 0;
            for(int k = 0; k < n_in; k++) 
                row_idx = (row_idx << 1) | (row_str[k] == '1');
            for (int i = 0; i < n_out; i++) 
                if (rhs[i] != 'X' && rhs[i] != 'x') 
                    tt_set_bit(output_tables[i], row_idx, rhs[i] == '1');
        }
        row_str = strtok_r(NULL, " \n\r\t,", &saveptr);
    }
    free(tt_copy);
}


void build_half_adder(Circuit *c, int a, int b, int *sum, int *carry) {
    *sum = circuit_add_gate_alive(c, EVO_XOR, a, b);
    *carry = circuit_add_gate_alive(c, EVO_AND, a, b);
}

void build_full_adder(Circuit *c, int a, int b, int cin, int *sum, int *cout) {
    int axb = circuit_add_gate_alive(c, EVO_XOR, a, b);
    *sum = circuit_add_gate_alive(c, EVO_XOR, axb, cin);
    int ab = circuit_add_gate_alive(c, EVO_AND, a, b);
    int cxab = circuit_add_gate_alive(c, EVO_AND, cin, axb);
    *cout = circuit_add_gate_alive(c, EVO_OR, ab, cxab);
}

void build_array_multiplier(Circuit *c, int n_bits) {
    int n = n_bits;
    int pp[16][16];
    
    /* Input mapping (MSB-first in truth table):
     *   A[i] (bit i of first operand) is at input index (n - 1 - i)
     *   B[j] (bit j of second operand) is at input index (2*n - 1 - j)
     */
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            int a_input = n - 1 - i;      /* A bit i */
            int b_input = 2*n - 1 - j;    /* B bit j */
            pp[i][j] = circuit_add_gate_alive(c, EVO_AND, a_input, b_input);
        }
    }
    
    int result[32];
    int carries[32][32];
    int carry_count[32] = {0};
    
    result[0] = pp[0][0];
    
    for (int col = 1; col < 2*n - 1; col++) {
        int bits[64];
        int bit_count = 0;
        
        for (int i = 0; i < n; i++) {
            int j = col - i;
            if (j >= 0 && j < n) bits[bit_count++] = pp[i][j];
        }
        for (int i = 0; i < carry_count[col]; i++) bits[bit_count++] = carries[col][i];
        
        while (bit_count > 1) {
            int new_bits[64], new_count = 0, i = 0;
            while (i < bit_count) {
                if (i + 2 < bit_count) {
                    int sum, cout;
                    build_full_adder(c, bits[i], bits[i+1], bits[i+2], &sum, &cout);
                    new_bits[new_count++] = sum;
                    carries[col+1][carry_count[col+1]++] = cout;
                    i += 3;
                } else if (i + 1 < bit_count) {
                    int sum, cout;
                    build_half_adder(c, bits[i], bits[i+1], &sum, &cout);
                    new_bits[new_count++] = sum;
                    carries[col+1][carry_count[col+1]++] = cout;
                    i += 2;
                } else {
                    new_bits[new_count++] = bits[i++];
                }
            }
            bit_count = new_count;
            for (int k = 0; k < bit_count; k++) bits[k] = new_bits[k];
        }
        result[col] = (bit_count > 0) ? bits[0] : 0;
    }
    
    int col = 2*n - 1;
    int bits[64], bit_count = carry_count[col];
    for (int i = 0; i < bit_count; i++) bits[i] = carries[col][i];
    while (bit_count > 1) {
        int new_bits[64], new_count = 0, i = 0;
        while (i < bit_count) {
            if (i + 1 < bit_count) {
                int sum, cout;
                build_half_adder(c, bits[i], bits[i+1], &sum, &cout);
                new_bits[new_count++] = sum;
                i += 2;
            } else new_bits[new_count++] = bits[i++];
        }
        bit_count = new_count;
        for (int k = 0; k < bit_count; k++) bits[k] = new_bits[k];
    }
    result[col] = (bit_count > 0) ? bits[0] : 0;
    
    /* Output mapping (MSB-first):
     * result[i] is bit i of product (LSB at i=0)
     * output_map[j] is output j (MSB at j=0)
     */
    for (int i = 0; i < 2*n && i < c->num_outputs; i++)
        c->output_map[2*n - 1 - i] = result[i];
}

bool detect_multiplier(TT **outputs, int n_in, int n_out) {
    if (n_in % 2 != 0 || n_out != n_in) return false;
    int n = n_in / 2;
    int total_rows = 1 << n_in;
    
    for (int row = 0; row < total_rows; row++) {
        /* Input bits: A is in lower n bits, B is in upper n bits */
        int a = row & ((1 << n) - 1);
        int b = (row >> n) & ((1 << n) - 1);
        int expected = a * b;
        
        for (int bit = 0; bit < n_out; bit++) {
            /* FIX: output_tables[0] is MSB (leftmost in string), not LSB */
            int output_idx = n_out - 1 - bit;
            if (((expected >> bit) & 1) != tt_get_bit(outputs[output_idx], row)) 
                return false;
        }
    }
    return true;
}



/* ----- ADDER DETECTION ----- */
bool detect_adder(TT **outputs, int n_in, int n_out) {
    /* A + B: n-bit + n-bit = (n+1)-bit result */
    if (n_in % 2 != 0) return false;
    int n = n_in / 2;
    if (n_out != n + 1 && n_out != n) return false;
    
    int total_rows = 1 << n_in;
    int out_bits = n_out;
    
    for (int row = 0; row < total_rows; row++) {
        int a = 0, b = 0;
        /* MSB-first: A is inputs 0..n-1, B is inputs n..2n-1 */
        for (int i = 0; i < n; i++) {
            a = (a << 1) | ((row >> (n_in - 1 - i)) & 1);
            b = (b << 1) | ((row >> (n_in - 1 - (n + i))) & 1);
        }
        int expected = a + b;
        
        for (int bit = 0; bit < out_bits; bit++) {
            int output_idx = out_bits - 1 - bit;  /* MSB-first */
            int expected_bit = (expected >> bit) & 1;
            if (tt_get_bit(outputs[output_idx], row) != expected_bit)
                return false;
        }
    }
    return true;
}

void build_ripple_carry_adder(Circuit *c, int n_bits) {
    int n = n_bits;
    int sum[16];
    int carry = -1;
    
    for (int i = 0; i < n; i++) {
        int a_in = i;           /* A[i] */
        int b_in = n + i;       /* B[i] */
        
        if (i == 0) {
            /* Half adder for LSB */
            sum[i] = circuit_add_gate_alive(c, EVO_XOR, a_in, b_in);
            carry = circuit_add_gate_alive(c, EVO_AND, a_in, b_in);
        } else {
            /* Full adder */
            int axb = circuit_add_gate_alive(c, EVO_XOR, a_in, b_in);
            sum[i] = circuit_add_gate_alive(c, EVO_XOR, axb, carry);
            int ab = circuit_add_gate_alive(c, EVO_AND, a_in, b_in);
            int axb_c = circuit_add_gate_alive(c, EVO_AND, axb, carry);
            carry = circuit_add_gate_alive(c, EVO_OR, ab, axb_c);
        }
    }
    
    /* Map outputs (MSB first) */
    int out_bits = c->num_outputs;
    if (out_bits == n + 1) {
        c->output_map[0] = carry;
        for (int i = 0; i < n; i++) {
            c->output_map[i + 1] = sum[n - 1 - i];
        }
    } else {
        for (int i = 0; i < n; i++) {
            c->output_map[i] = sum[n - 1 - i];
        }
    }
}

/* ----- SUBTRACTOR DETECTION ----- */
bool detect_subtractor(TT **outputs, int n_in, int n_out) {
    /* A - B with borrow/sign output */
    if (n_in % 2 != 0) return false;
    int n = n_in / 2;
    if (n_out != n && n_out != n + 1) return false;
    
    int total_rows = 1 << n_in;
    
    for (int row = 0; row < total_rows; row++) {
        int a = 0, b = 0;
        for (int i = 0; i < n; i++) {
            a = (a << 1) | ((row >> (n_in - 1 - i)) & 1);
            b = (b << 1) | ((row >> (n_in - 1 - (n + i))) & 1);
        }
        int expected = a - b;
        int mask = (1 << n) - 1;
        int result = expected & mask;
        int borrow = (expected < 0) ? 1 : 0;
        
        for (int bit = 0; bit < n; bit++) {
            int output_idx = n_out - 1 - bit;
            if (tt_get_bit(outputs[output_idx], row) != ((result >> bit) & 1))
                return false;
        }
        if (n_out == n + 1) {
            if (tt_get_bit(outputs[0], row) != borrow)
                return false;
        }
    }
    return true;
}

void build_half_subtractor(Circuit *c, int a, int b, int *diff, int *borrow) {
    *diff = circuit_add_gate_alive(c, EVO_XOR, a, b);
    int not_a = circuit_add_gate_alive(c, EVO_NOT, a, 0);
    *borrow = circuit_add_gate_alive(c, EVO_AND, not_a, b);
}

void build_full_subtractor(Circuit *c, int a, int b, int bin, int *diff, int *bout) {
    int axb = circuit_add_gate_alive(c, EVO_XOR, a, b);
    *diff = circuit_add_gate_alive(c, EVO_XOR, axb, bin);
    int not_a = circuit_add_gate_alive(c, EVO_NOT, a, 0);
    int not_axb = circuit_add_gate_alive(c, EVO_NOT, axb, 0);
    int t1 = circuit_add_gate_alive(c, EVO_AND, not_a, b);
    int t2 = circuit_add_gate_alive(c, EVO_AND, not_axb, bin);
    *bout = circuit_add_gate_alive(c, EVO_OR, t1, t2);
}

void build_subtractor(Circuit *c, int n_bits) {
    int n = n_bits;
    int diff[16];
    int borrow = -1;
    
    for (int i = 0; i < n; i++) {
        int a_in = n - 1 - i;
        int b_in = 2*n - 1 - i;
        
        if (i == 0) {
            int hs_diff, hs_borrow;
            build_half_subtractor(c, a_in, b_in, &hs_diff, &hs_borrow);
            diff[i] = hs_diff;
            borrow = hs_borrow;
        } else {
            int fs_diff, fs_borrow;
            build_full_subtractor(c, a_in, b_in, borrow, &fs_diff, &fs_borrow);
            diff[i] = fs_diff;
            borrow = fs_borrow;
        }
    }
    
    int out_bits = c->num_outputs;
    for (int i = 0; i < n && i < out_bits; i++) {
        c->output_map[out_bits - 1 - i] = diff[i];
    }
    if (out_bits == n + 1) {
        c->output_map[0] = borrow;
    }
}

/* ----- COMPARATOR DETECTION (A < B) ----- */
bool detect_less_than(TT **outputs, int n_in, int n_out) {
    if (n_in % 2 != 0 || n_out != 1) return false;
    int n = n_in / 2;
    int total_rows = 1 << n_in;
    
    for (int row = 0; row < total_rows; row++) {
        int a = 0, b = 0;
        for (int i = 0; i < n; i++) {
            a = (a << 1) | ((row >> (n_in - 1 - i)) & 1);
            b = (b << 1) | ((row >> (n_in - 1 - (n + i))) & 1);
        }
        int expected = (a < b) ? 1 : 0;
        if (tt_get_bit(outputs[0], row) != expected)
            return false;
    }
    return true;
}

void build_less_than_comparator(Circuit *c, int n_bits) {
    /* A < B: compute B - A and check if no borrow (or compute A - B and check borrow) */
    int n = n_bits;
    int borrow = -1;
    
    for (int i = 0; i < n; i++) {
        int a_in = n - 1 - i;
        int b_in = 2*n - 1 - i;
        
        if (i == 0) {
            int not_a = circuit_add_gate_alive(c, EVO_NOT, a_in, 0);
            borrow = circuit_add_gate_alive(c, EVO_AND, not_a, b_in);
        } else {
            int axb = circuit_add_gate_alive(c, EVO_XOR, a_in, b_in);
            int not_a = circuit_add_gate_alive(c, EVO_NOT, a_in, 0);
            int not_axb = circuit_add_gate_alive(c, EVO_NOT, axb, 0);
            int t1 = circuit_add_gate_alive(c, EVO_AND, not_a, b_in);
            int t2 = circuit_add_gate_alive(c, EVO_AND, not_axb, borrow);
            borrow = circuit_add_gate_alive(c, EVO_OR, t1, t2);
        }
    }
    
    c->output_map[0] = borrow;  /* borrow=1 means A < B */
}

/* ----- EQUALITY DETECTION (A == B) ----- */
bool detect_equality(TT **outputs, int n_in, int n_out) {
    if (n_in % 2 != 0 || n_out != 1) return false;
    int n = n_in / 2;
    int total_rows = 1 << n_in;
    
    for (int row = 0; row < total_rows; row++) {
        int a = 0, b = 0;
        for (int i = 0; i < n; i++) {
            a = (a << 1) | ((row >> (n_in - 1 - i)) & 1);
            b = (b << 1) | ((row >> (n_in - 1 - (n + i))) & 1);
        }
        int expected = (a == b) ? 1 : 0;
        if (tt_get_bit(outputs[0], row) != expected)
            return false;
    }
    return true;
}

void build_equality_comparator(Circuit *c, int n_bits) {
    /* A == B: XNOR all bit pairs, then AND results together */
    int n = n_bits;
    
    if (n == 1) {
        /* Single bit: just XNOR */
        c->output_map[0] = circuit_add_gate_alive(c, EVO_XNOR, 0, 1);
        return;
    }
    
    /* Multi-bit: XNOR each pair, AND all results */
    int xnor_results[16];
    for (int i = 0; i < n; i++) {
        int a_bit = i;              /* A[i] */
        int b_bit = n + i;          /* B[i] */
        xnor_results[i] = circuit_add_gate_alive(c, EVO_XNOR, a_bit, b_bit);
    }
    
    /* AND tree */
    int result = xnor_results[0];
    for (int i = 1; i < n; i++) {
        result = circuit_add_gate_alive(c, EVO_AND, result, xnor_results[i]);
    }
    
    c->output_map[0] = result;
}

/* ----- PARITY (XOR of all inputs) ----- */
bool detect_parity(TT **outputs, int n_in, int n_out) {
    if (n_out != 1) return false;
    int total_rows = 1 << n_in;
    
    for (int row = 0; row < total_rows; row++) {
        int parity = 0;
        for (int i = 0; i < n_in; i++) {
            parity ^= (row >> i) & 1;
        }
        if (tt_get_bit(outputs[0], row) != parity)
            return false;
    }
    return true;
}

void build_parity(Circuit *c, int n_bits) {
    if (n_bits == 1) {
        c->output_map[0] = 0;  /* Single bit is its own parity */
        return;
    }
    
    /* XOR chain of all inputs */
    int result = circuit_add_gate_alive(c, EVO_XOR, 0, 1);
    for (int i = 2; i < n_bits; i++) {
        result = circuit_add_gate_alive(c, EVO_XOR, result, i);
    }
    c->output_map[0] = result;
}











/* ============================================================
 * MORE STRUCTURAL PATTERNS FOR DLS2
 * ============================================================ */

/* ----- GREATER THAN (A > B) ----- */
bool detect_greater_than(TT **outputs, int n_in, int n_out) {
    if (n_in % 2 != 0 || n_out != 1) return false;
    int n = n_in / 2;
    int total_rows = 1 << n_in;
    
    for (int row = 0; row < total_rows; row++) {
        int a = 0, b = 0;
        for (int i = 0; i < n; i++) {
            a = (a << 1) | ((row >> (n_in - 1 - i)) & 1);
            b = (b << 1) | ((row >> (n_in - 1 - (n + i))) & 1);
        }
        int expected = (a > b) ? 1 : 0;
        if (tt_get_bit(outputs[0], row) != expected)
            return false;
    }
    return true;
}

void build_greater_than_comparator(Circuit *c, int n_bits) {
    /* A > B is same as B < A, so swap inputs */
    int n = n_bits;
    int borrow = -1;
    
    for (int i = 0; i < n; i++) {
        int a_in = n - 1 - i;      /* This becomes B */
        int b_in = 2*n - 1 - i;    /* This becomes A */
        
        /* Compute B - A, borrow means B < A means A > B */
        if (i == 0) {
            int not_b = circuit_add_gate_alive(c, EVO_NOT, b_in, 0);
            borrow = circuit_add_gate_alive(c, EVO_AND, not_b, a_in);
        } else {
            int axb = circuit_add_gate_alive(c, EVO_XOR, a_in, b_in);
            int not_b = circuit_add_gate_alive(c, EVO_NOT, b_in, 0);
            int not_axb = circuit_add_gate_alive(c, EVO_NOT, axb, 0);
            int t1 = circuit_add_gate_alive(c, EVO_AND, not_b, a_in);
            int t2 = circuit_add_gate_alive(c, EVO_AND, not_axb, borrow);
            borrow = circuit_add_gate_alive(c, EVO_OR, t1, t2);
        }
    }
    
    c->output_map[0] = borrow;
}

/* ----- ZERO DETECTOR (A == 0) ----- */
bool detect_is_zero(TT **outputs, int n_in, int n_out) {
    if (n_out != 1) return false;
    int total_rows = 1 << n_in;
    
    for (int row = 0; row < total_rows; row++) {
        int expected = (row == 0) ? 1 : 0;
        if (tt_get_bit(outputs[0], row) != expected)
            return false;
    }
    return true;
}

void build_is_zero(Circuit *c, int n_bits) {
    if (n_bits == 1) {
        c->output_map[0] = circuit_add_gate_alive(c, EVO_NOT, 0, 0);
        return;
    }
    
    if (n_bits == 2) {
        c->output_map[0] = circuit_add_gate_alive(c, EVO_NOR, 0, 1);
        return;
    }
    
    /* For n >= 3: NOR tree or AND of NOTs */
    int result = circuit_add_gate_alive(c, EVO_NOR, 0, 1);
    for (int i = 2; i < n_bits; i++) {
        int not_i = circuit_add_gate_alive(c, EVO_NOT, i, 0);
        result = circuit_add_gate_alive(c, EVO_AND, result, not_i);
    }
    c->output_map[0] = result;
}

/* ----- INCREMENTER (A + 1) ----- */
bool detect_incrementer(TT **outputs, int n_in, int n_out) {
    if (n_out != n_in && n_out != n_in + 1) return false;
    int total_rows = 1 << n_in;
    int out_mask = (1 << n_out) - 1;
    
    for (int row = 0; row < total_rows; row++) {
        int a = 0;
        for (int i = 0; i < n_in; i++) {
            a = (a << 1) | ((row >> (n_in - 1 - i)) & 1);
        }
        int expected = (a + 1) & out_mask;
        
        for (int bit = 0; bit < n_out; bit++) {
            int output_idx = n_out - 1 - bit;
            if (tt_get_bit(outputs[output_idx], row) != ((expected >> bit) & 1))
                return false;
        }
    }
    return true;
}

void build_incrementer(Circuit *c, int n_bits) {
    int sum[16];
    int carry;
    
    /* First bit: half adder with implicit 1 */
    int in_0 = n_bits - 1;  /* LSB input */
    sum[0] = circuit_add_gate_alive(c, EVO_NOT, in_0, 0);  /* XOR with 1 = NOT */
    carry = in_0;  /* AND with 1 = identity */
    
    for (int i = 1; i < n_bits; i++) {
        int in_i = n_bits - 1 - i;
        sum[i] = circuit_add_gate_alive(c, EVO_XOR, in_i, carry);
        carry = circuit_add_gate_alive(c, EVO_AND, in_i, carry);
    }
    
    int out_bits = c->num_outputs;
    for (int i = 0; i < n_bits && i < out_bits; i++) {
        c->output_map[out_bits - 1 - i] = sum[i];
    }
    if (out_bits == n_bits + 1) {
        c->output_map[0] = carry;
    }
}

/* ----- DECREMENTER (A - 1) ----- */
bool detect_decrementer(TT **outputs, int n_in, int n_out) {
    if (n_out != n_in) return false;
    int total_rows = 1 << n_in;
    int out_mask = (1 << n_out) - 1;
    
    for (int row = 0; row < total_rows; row++) {
        int a = 0;
        for (int i = 0; i < n_in; i++) {
            a = (a << 1) | ((row >> (n_in - 1 - i)) & 1);
        }
        int expected = (a - 1) & out_mask;
        
        for (int bit = 0; bit < n_out; bit++) {
            int output_idx = n_out - 1 - bit;
            if (tt_get_bit(outputs[output_idx], row) != ((expected >> bit) & 1))
                return false;
        }
    }
    return true;
}

void build_decrementer(Circuit *c, int n_bits) {
    int diff[16];
    int borrow;
    
    /* First bit: half subtractor with implicit 1 */
    int in_0 = n_bits - 1;
    diff[0] = circuit_add_gate_alive(c, EVO_NOT, in_0, 0);
    int not_in_0 = diff[0];
    borrow = not_in_0;  /* borrow = NOT(A) AND 1 = NOT(A) */
    
    for (int i = 1; i < n_bits; i++) {
        int in_i = n_bits - 1 - i;
        diff[i] = circuit_add_gate_alive(c, EVO_XOR, in_i, borrow);
        int not_in_i = circuit_add_gate_alive(c, EVO_NOT, in_i, 0);
        borrow = circuit_add_gate_alive(c, EVO_AND, not_in_i, borrow);
    }
    
    for (int i = 0; i < n_bits; i++) {
        c->output_map[n_bits - 1 - i] = diff[i];
    }
}

/* ----- 2's COMPLEMENT NEGATION (-A) ----- */
bool detect_negation(TT **outputs, int n_in, int n_out) {
    if (n_out != n_in) return false;
    int total_rows = 1 << n_in;
    int mask = (1 << n_in) - 1;
    
    for (int row = 0; row < total_rows; row++) {
        int a = 0;
        for (int i = 0; i < n_in; i++) {
            a = (a << 1) | ((row >> (n_in - 1 - i)) & 1);
        }
        int expected = (-a) & mask;
        
        for (int bit = 0; bit < n_out; bit++) {
            int output_idx = n_out - 1 - bit;
            if (tt_get_bit(outputs[output_idx], row) != ((expected >> bit) & 1))
                return false;
        }
    }
    return true;
}

void build_negation(Circuit *c, int n_bits) {
    /* -A = ~A + 1 (invert and increment) */
    int inv[16];
    int sum[16];
    int carry;
    
    /* Invert all bits */
    for (int i = 0; i < n_bits; i++) {
        inv[i] = circuit_add_gate_alive(c, EVO_NOT, i, 0);
    }
    
    /* Add 1 to inverted value */
    int inv_lsb = inv[n_bits - 1];
    sum[0] = circuit_add_gate_alive(c, EVO_NOT, inv_lsb, 0);
    carry = inv_lsb;
    
    for (int i = 1; i < n_bits; i++) {
        int inv_i = inv[n_bits - 1 - i];
        sum[i] = circuit_add_gate_alive(c, EVO_XOR, inv_i, carry);
        carry = circuit_add_gate_alive(c, EVO_AND, inv_i, carry);
    }
    
    for (int i = 0; i < n_bits; i++) {
        c->output_map[n_bits - 1 - i] = sum[i];
    }
}

/* ----- 2-to-1 MUX (Select ? B : A) ----- */
bool detect_mux2to1(TT **outputs, int n_in, int n_out) {
    /* Format: n data bits A, n data bits B, 1 select bit = 2n+1 inputs, n outputs */
    if (n_in < 3 || n_in % 2 == 0) return false;
    int n = (n_in - 1) / 2;
    if (n_out != n) return false;
    
    int total_rows = 1 << n_in;
    
    for (int row = 0; row < total_rows; row++) {
        int a = 0, b = 0;
        for (int i = 0; i < n; i++) {
            a = (a << 1) | ((row >> (n_in - 1 - i)) & 1);
            b = (b << 1) | ((row >> (n_in - 1 - (n + i))) & 1);
        }
        int sel = (row >> (n_in - 1 - (2*n))) & 1;  /* Last input is select */
        int expected = sel ? b : a;
        
        for (int bit = 0; bit < n; bit++) {
            int output_idx = n - 1 - bit;
            if (tt_get_bit(outputs[output_idx], row) != ((expected >> bit) & 1))
                return false;
        }
    }
    return true;
}

void build_mux2to1(Circuit *c, int n_bits) {
    int n = n_bits;
    int sel = 2 * n;  /* Select input index */
    int not_sel = circuit_add_gate_alive(c, EVO_NOT, sel, 0);
    
    for (int i = 0; i < n; i++) {
        int a_in = n - 1 - i;
        int b_in = 2*n - 1 - i;
        
        int and_a = circuit_add_gate_alive(c, EVO_AND, a_in, not_sel);
        int and_b = circuit_add_gate_alive(c, EVO_AND, b_in, sel);
        int mux_out = circuit_add_gate_alive(c, EVO_OR, and_a, and_b);
        
        c->output_map[n - 1 - i] = mux_out;
    }
}

/* ----- DECODER (n inputs -> 2^n outputs, one-hot) ----- */
bool detect_decoder(TT **outputs, int n_in, int n_out) {
    if (n_out != (1 << n_in)) return false;
    int total_rows = 1 << n_in;
    
    for (int row = 0; row < total_rows; row++) {
        int addr = 0;
        for (int i = 0; i < n_in; i++) {
            addr = (addr << 1) | ((row >> (n_in - 1 - i)) & 1);
        }
        /* Output 'addr' should be 1, all others 0 */
        for (int out = 0; out < n_out; out++) {
            int expected = (out == addr) ? 1 : 0;
            if (tt_get_bit(outputs[out], row) != expected)
                return false;
        }
    }
    return true;
}

void build_decoder(Circuit *c, int n_bits) {
    int n = n_bits;
    int num_outputs = 1 << n;
    
    /* Create inverted inputs */
    int not_in[16];
    for (int i = 0; i < n; i++) {
        not_in[i] = circuit_add_gate_alive(c, EVO_NOT, i, 0);
    }
    
    /* For each output, AND together the appropriate combination */
    for (int out = 0; out < num_outputs; out++) {
        int result = -1;
        for (int bit = 0; bit < n; bit++) {
            int use_inv = !((out >> (n - 1 - bit)) & 1);
            int term = use_inv ? not_in[bit] : bit;
            
            if (result == -1) {
                result = term;
            } else {
                result = circuit_add_gate_alive(c, EVO_AND, result, term);
            }
        }
        c->output_map[out] = result;
    }
}

/* ----- LEFT SHIFT BY 1 ----- */
bool detect_left_shift_1(TT **outputs, int n_in, int n_out) {
    if (n_out != n_in) return false;
    int total_rows = 1 << n_in;
    int mask = (1 << n_in) - 1;
    
    for (int row = 0; row < total_rows; row++) {
        int a = 0;
        for (int i = 0; i < n_in; i++) {
            a = (a << 1) | ((row >> (n_in - 1 - i)) & 1);
        }
        int expected = (a << 1) & mask;
        
        for (int bit = 0; bit < n_out; bit++) {
            int output_idx = n_out - 1 - bit;
            if (tt_get_bit(outputs[output_idx], row) != ((expected >> bit) & 1))
                return false;
        }
    }
    return true;
}

void build_left_shift_1(Circuit *c, int n_bits) {
    /* Shift left: output[i] = input[i+1], output[LSB] = 0 */
    /* Wire directly - output MSB gets input[1], etc. */
    for (int i = 0; i < n_bits - 1; i++) {
        c->output_map[i] = i + 1;  /* output[i] = input[i+1] */
    }
    /* LSB output is 0 - need a constant 0 */
    int zero = circuit_add_gate_alive(c, EVO_XOR, 0, 0);  /* 0 XOR 0 = 0 */
    c->output_map[n_bits - 1] = zero;
}

/* ----- RIGHT SHIFT BY 1 (logical) ----- */
bool detect_right_shift_1(TT **outputs, int n_in, int n_out) {
    if (n_out != n_in) return false;
    int total_rows = 1 << n_in;
    
    for (int row = 0; row < total_rows; row++) {
        int a = 0;
        for (int i = 0; i < n_in; i++) {
            a = (a << 1) | ((row >> (n_in - 1 - i)) & 1);
        }
        int expected = a >> 1;
        
        for (int bit = 0; bit < n_out; bit++) {
            int output_idx = n_out - 1 - bit;
            if (tt_get_bit(outputs[output_idx], row) != ((expected >> bit) & 1))
                return false;
        }
    }
    return true;
}

void build_right_shift_1(Circuit *c, int n_bits) {
    /* Shift right: output[i] = input[i-1], output[MSB] = 0 */
    int zero = circuit_add_gate_alive(c, EVO_XOR, 0, 0);
    c->output_map[0] = zero;  /* MSB is 0 */
    for (int i = 1; i < n_bits; i++) {
        c->output_map[i] = i - 1;  /* output[i] = input[i-1] */
    }
}

/* ----- BITWISE NOT (invert all bits) ----- */
bool detect_bitwise_not(TT **outputs, int n_in, int n_out) {
    if (n_out != n_in) return false;
    int total_rows = 1 << n_in;
    int mask = (1 << n_in) - 1;
    
    for (int row = 0; row < total_rows; row++) {
        int a = 0;
        for (int i = 0; i < n_in; i++) {
            a = (a << 1) | ((row >> (n_in - 1 - i)) & 1);
        }
        int expected = (~a) & mask;
        
        for (int bit = 0; bit < n_out; bit++) {
            int output_idx = n_out - 1 - bit;
            if (tt_get_bit(outputs[output_idx], row) != ((expected >> bit) & 1))
                return false;
        }
    }
    return true;
}

void build_bitwise_not(Circuit *c, int n_bits) {
    for (int i = 0; i < n_bits; i++) {
        int not_i = circuit_add_gate_alive(c, EVO_NOT, i, 0);
        c->output_map[i] = not_i;
    }
}

/* ----- MAX(A, B) ----- */
bool detect_max(TT **outputs, int n_in, int n_out) {
    if (n_in % 2 != 0) return false;
    int n = n_in / 2;
    if (n_out != n) return false;
    
    int total_rows = 1 << n_in;
    
    for (int row = 0; row < total_rows; row++) {
        int a = 0, b = 0;
        for (int i = 0; i < n; i++) {
            a = (a << 1) | ((row >> (n_in - 1 - i)) & 1);
            b = (b << 1) | ((row >> (n_in - 1 - (n + i))) & 1);
        }
        int expected = (a > b) ? a : b;
        
        for (int bit = 0; bit < n; bit++) {
            int output_idx = n - 1 - bit;
            if (tt_get_bit(outputs[output_idx], row) != ((expected >> bit) & 1))
                return false;
        }
    }
    return true;
}

void build_max(Circuit *c, int n_bits) {
    /* MAX(A,B) = (A > B) ? A : B */
    int n = n_bits;
    
    /* First compute A > B */
    int borrow = -1;
    for (int i = 0; i < n; i++) {
        int a_in = n - 1 - i;
        int b_in = 2*n - 1 - i;
        
        if (i == 0) {
            int not_b = circuit_add_gate_alive(c, EVO_NOT, b_in, 0);
            borrow = circuit_add_gate_alive(c, EVO_AND, not_b, a_in);
        } else {
            int axb = circuit_add_gate_alive(c, EVO_XOR, a_in, b_in);
            int not_b = circuit_add_gate_alive(c, EVO_NOT, b_in, 0);
            int not_axb = circuit_add_gate_alive(c, EVO_NOT, axb, 0);
            int t1 = circuit_add_gate_alive(c, EVO_AND, not_b, a_in);
            int t2 = circuit_add_gate_alive(c, EVO_AND, not_axb, borrow);
            borrow = circuit_add_gate_alive(c, EVO_OR, t1, t2);
        }
    }
    int a_gt_b = borrow;
    int not_sel = circuit_add_gate_alive(c, EVO_NOT, a_gt_b, 0);
    
    /* MUX: select A if a_gt_b, else B */
    for (int i = 0; i < n; i++) {
        int a_in = n - 1 - i;
        int b_in = 2*n - 1 - i;
        
        int and_a = circuit_add_gate_alive(c, EVO_AND, a_in, a_gt_b);
        int and_b = circuit_add_gate_alive(c, EVO_AND, b_in, not_sel);
        int mux_out = circuit_add_gate_alive(c, EVO_OR, and_a, and_b);
        
        c->output_map[n - 1 - i] = mux_out;
    }
}

/* ----- MIN(A, B) ----- */
bool detect_min(TT **outputs, int n_in, int n_out) {
    if (n_in % 2 != 0) return false;
    int n = n_in / 2;
    if (n_out != n) return false;
    
    int total_rows = 1 << n_in;
    
    for (int row = 0; row < total_rows; row++) {
        int a = 0, b = 0;
        for (int i = 0; i < n; i++) {
            a = (a << 1) | ((row >> (n_in - 1 - i)) & 1);
            b = (b << 1) | ((row >> (n_in - 1 - (n + i))) & 1);
        }
        int expected = (a < b) ? a : b;
        
        for (int bit = 0; bit < n; bit++) {
            int output_idx = n - 1 - bit;
            if (tt_get_bit(outputs[output_idx], row) != ((expected >> bit) & 1))
                return false;
        }
    }
    return true;
}

void build_min(Circuit *c, int n_bits) {
    /* MIN(A,B) = (A < B) ? A : B */
    int n = n_bits;
    
    /* First compute A < B */
    int borrow = -1;
    for (int i = 0; i < n; i++) {
        int a_in = n - 1 - i;
        int b_in = 2*n - 1 - i;
        
        if (i == 0) {
            int not_a = circuit_add_gate_alive(c, EVO_NOT, a_in, 0);
            borrow = circuit_add_gate_alive(c, EVO_AND, not_a, b_in);
        } else {
            int axb = circuit_add_gate_alive(c, EVO_XOR, a_in, b_in);
            int not_a = circuit_add_gate_alive(c, EVO_NOT, a_in, 0);
            int not_axb = circuit_add_gate_alive(c, EVO_NOT, axb, 0);
            int t1 = circuit_add_gate_alive(c, EVO_AND, not_a, b_in);
            int t2 = circuit_add_gate_alive(c, EVO_AND, not_axb, borrow);
            borrow = circuit_add_gate_alive(c, EVO_OR, t1, t2);
        }
    }
    int a_lt_b = borrow;
    int not_sel = circuit_add_gate_alive(c, EVO_NOT, a_lt_b, 0);
    
    /* MUX: select A if a_lt_b, else B */
    for (int i = 0; i < n; i++) {
        int a_in = n - 1 - i;
        int b_in = 2*n - 1 - i;
        
        int and_a = circuit_add_gate_alive(c, EVO_AND, a_in, a_lt_b);
        int and_b = circuit_add_gate_alive(c, EVO_AND, b_in, not_sel);
        int mux_out = circuit_add_gate_alive(c, EVO_OR, and_a, and_b);
        
        c->output_map[n - 1 - i] = mux_out;
    }
}


/* ============================================================
 * ADDITIONAL PATTERN DETECTORS AND STRUCTURAL BUILDERS
 * ============================================================ */

/* ----- POPULATION COUNT (number of 1s) ----- */
bool detect_popcount(TT **outputs, int n_in, int n_out) {
    int expected_out_bits = 1;
    while ((1 << expected_out_bits) <= n_in) expected_out_bits++;
    if (n_out != expected_out_bits) return false;
    
    int total_rows = 1 << n_in;
    for (int row = 0; row < total_rows; row++) {
        int popcount = __builtin_popcount(row);
        for (int bit = 0; bit < n_out; bit++) {
            int output_idx = n_out - 1 - bit;
            if (tt_get_bit(outputs[output_idx], row) != ((popcount >> bit) & 1))
                return false;
        }
    }
    return true;
}

void build_popcount(Circuit *c, int n_bits) {
    if (n_bits == 1) {
        c->output_map[0] = 0;
        return;
    }
    
    if (n_bits == 2) {
        /* 00->00, 01->01, 10->01, 11->10 */
        int sum = circuit_add_gate_alive(c, EVO_XOR, 0, 1);
        int carry = circuit_add_gate_alive(c, EVO_AND, 0, 1);
        c->output_map[0] = carry;  /* MSB */
        c->output_map[1] = sum;    /* LSB */
        return;
    }
    
    if (n_bits == 3) {
        /* Full adder for 3 bits */
        int a = 0, b = 1, cin = 2;
        int axb = circuit_add_gate_alive(c, EVO_XOR, a, b);
        int sum = circuit_add_gate_alive(c, EVO_XOR, axb, cin);
        int ab = circuit_add_gate_alive(c, EVO_AND, a, b);
        int axb_c = circuit_add_gate_alive(c, EVO_AND, axb, cin);
        int carry = circuit_add_gate_alive(c, EVO_OR, ab, axb_c);
        c->output_map[0] = carry;
        c->output_map[1] = sum;
        return;
    }
    
    if (n_bits == 4) {
        /* Two half adders, then combine */
        int sum01 = circuit_add_gate_alive(c, EVO_XOR, 0, 1);
        int carry01 = circuit_add_gate_alive(c, EVO_AND, 0, 1);
        int sum23 = circuit_add_gate_alive(c, EVO_XOR, 2, 3);
        int carry23 = circuit_add_gate_alive(c, EVO_AND, 2, 3);
        
        /* Add sum01 + sum23 */
        int sum_low = circuit_add_gate_alive(c, EVO_XOR, sum01, sum23);
        int carry_low = circuit_add_gate_alive(c, EVO_AND, sum01, sum23);
        
        /* Add carry01 + carry23 + carry_low */
        int c01_xor_c23 = circuit_add_gate_alive(c, EVO_XOR, carry01, carry23);
        int sum_mid = circuit_add_gate_alive(c, EVO_XOR, c01_xor_c23, carry_low);
        int c01_and_c23 = circuit_add_gate_alive(c, EVO_AND, carry01, carry23);
        int c01xc23_and_clow = circuit_add_gate_alive(c, EVO_AND, c01_xor_c23, carry_low);
        int sum_high = circuit_add_gate_alive(c, EVO_OR, c01_and_c23, c01xc23_and_clow);
        
        c->output_map[0] = sum_high;  /* MSB (bit 2) */
        c->output_map[1] = sum_mid;   /* bit 1 */
        c->output_map[2] = sum_low;   /* LSB (bit 0) */
        return;
    }
    
    /* For larger sizes, use compressor tree - simplified version */
    /* This is a fallback that may not be optimal */
    int out_bits = 0;
    while ((1 << out_bits) <= n_bits) out_bits++;
    
    for (int i = 0; i < out_bits; i++) {
        c->output_map[i] = circuit_add_gate_alive(c, EVO_XOR, 0, 0);
    }
}

/* ----- GRAY CODE ENCODER (Binary to Gray) ----- */
bool detect_gray_encode(TT **outputs, int n_in, int n_out) {
    if (n_out != n_in) return false;
    int total_rows = 1 << n_in;
    
    for (int row = 0; row < total_rows; row++) {
        int binary = 0;
        for (int i = 0; i < n_in; i++) {
            binary = (binary << 1) | ((row >> (n_in - 1 - i)) & 1);
        }
        int gray = binary ^ (binary >> 1);
        
        for (int bit = 0; bit < n_out; bit++) {
            int output_idx = n_out - 1 - bit;
            if (tt_get_bit(outputs[output_idx], row) != ((gray >> bit) & 1))
                return false;
        }
    }
    return true;
}

void build_gray_encode(Circuit *c, int n_bits) {
    /* Gray = Binary XOR (Binary >> 1) */
    /* MSB stays same, others are XOR of adjacent bits */
    c->output_map[0] = 0;  /* MSB: G[n-1] = B[n-1] */
    
    for (int i = 1; i < n_bits; i++) {
        /* G[n-1-i] = B[n-1-i] XOR B[n-i] */
        int xor_gate = circuit_add_gate_alive(c, EVO_XOR, i-1, i);
        c->output_map[i] = xor_gate;
    }
}

/* ----- GRAY CODE DECODER (Gray to Binary) ----- */
bool detect_gray_decode(TT **outputs, int n_in, int n_out) {
    if (n_out != n_in) return false;
    int total_rows = 1 << n_in;
    
    for (int row = 0; row < total_rows; row++) {
        int gray = 0;
        for (int i = 0; i < n_in; i++) {
            gray = (gray << 1) | ((row >> (n_in - 1 - i)) & 1);
        }
        /* Decode: binary[i] = XOR of all gray bits from MSB to i */
        int binary = 0;
        int running_xor = 0;
        for (int i = n_in - 1; i >= 0; i--) {
            running_xor ^= (gray >> i) & 1;
            binary |= (running_xor << i);
        }
        
        for (int bit = 0; bit < n_out; bit++) {
            int output_idx = n_out - 1 - bit;
            if (tt_get_bit(outputs[output_idx], row) != ((binary >> bit) & 1))
                return false;
        }
    }
    return true;
}

void build_gray_decode(Circuit *c, int n_bits) {
    /* Binary[i] = XOR of Gray[n-1:i] */
    int running_xor = 0;  /* Input 0 is MSB */
    
    c->output_map[0] = 0;  /* B[MSB] = G[MSB] */
    running_xor = 0;
    
    for (int i = 1; i < n_bits; i++) {
        /* XOR chain: B[i] = G[0] ^ G[1] ^ ... ^ G[i] */
        int xor_gate = circuit_add_gate_alive(c, EVO_XOR, 
            (i == 1) ? 0 : c->output_map[i-1], i);
        c->output_map[i] = xor_gate;
    }
}

/* ----- PRIORITY ENCODER (find highest set bit) ----- */
bool detect_priority_encoder(TT **outputs, int n_in, int n_out) {
    /* n_out should be ceil(log2(n_in)) + 1 for valid bit */
    int index_bits = 0;
    while ((1 << index_bits) < n_in) index_bits++;
    if (n_out != index_bits && n_out != index_bits + 1) return false;
    
    bool has_valid = (n_out == index_bits + 1);
    int total_rows = 1 << n_in;
    
    for (int row = 0; row < total_rows; row++) {
        int highest = -1;
        for (int i = 0; i < n_in; i++) {
            if ((row >> (n_in - 1 - i)) & 1) {
                highest = i;
                break;
            }
        }
        
        if (has_valid) {
            int valid = (highest >= 0) ? 1 : 0;
            if (tt_get_bit(outputs[n_out - 1], row) != valid)
                return false;
        }
        
        if (highest < 0) highest = 0;
        
        for (int bit = 0; bit < index_bits; bit++) {
            int output_idx = index_bits - 1 - bit;
            if (tt_get_bit(outputs[output_idx], row) != ((highest >> bit) & 1))
                return false;
        }
    }
    return true;
}

void build_priority_encoder(Circuit *c, int n_bits) {
    int index_bits = 0;
    while ((1 << index_bits) < n_bits) index_bits++;
    bool has_valid = (c->num_outputs == index_bits + 1);
    
    /* Valid signal: OR of all inputs */
    int valid;
    if (n_bits == 1) {
        valid = 0;
    } else {
        valid = circuit_add_gate_alive(c, EVO_OR, 0, 1);
        for (int i = 2; i < n_bits; i++) {
            valid = circuit_add_gate_alive(c, EVO_OR, valid, i);
        }
    }
    
    if (has_valid) {
        c->output_map[index_bits] = valid;
    }
    
    /* For each output bit, determine when it should be 1 */
    /* Input 0 is highest priority (MSB), input n-1 is lowest */
    
    /* Create "no higher priority" signals */
    int no_higher[16];
    no_higher[0] = -1;  /* Input 0 has no higher priority input */
    for (int i = 1; i < n_bits; i++) {
        int higher_or = 0;
        for (int j = 0; j < i; j++) {
            if (j == 0) {
                higher_or = j;
            } else {
                higher_or = circuit_add_gate_alive(c, EVO_OR, higher_or, j);
            }
        }
        no_higher[i] = circuit_add_gate_alive(c, EVO_NOT, higher_or, 0);
    }
    
    /* For each bit position in output */
    for (int bit = 0; bit < index_bits; bit++) {
        int result = -1;
        
        for (int i = 0; i < n_bits; i++) {
            /* Does input i contribute a 1 to this output bit? */
            if ((i >> bit) & 1) {
                /* Input i is active AND no higher priority input is active */
                int contrib;
                if (no_higher[i] == -1) {
                    contrib = i;  /* Input 0 always wins if active */
                } else {
                    contrib = circuit_add_gate_alive(c, EVO_AND, i, no_higher[i]);
                }
                
                if (result == -1) {
                    result = contrib;
                } else {
                    result = circuit_add_gate_alive(c, EVO_OR, result, contrib);
                }
            }
        }
        
        if (result == -1) {
            result = circuit_add_gate_alive(c, EVO_XOR, 0, 0);  /* Constant 0 */
        }
        
        c->output_map[index_bits - 1 - bit] = result;
    }
}

/* ----- LEADING ZERO COUNT (CLZ) ----- */
bool detect_clz(TT **outputs, int n_in, int n_out) {
    int expected_bits = 0;
    while ((1 << expected_bits) <= n_in) expected_bits++;
    if (n_out != expected_bits) return false;
    
    int total_rows = 1 << n_in;
    for (int row = 0; row < total_rows; row++) {
        int input = 0;
        for (int i = 0; i < n_in; i++) {
            input = (input << 1) | ((row >> (n_in - 1 - i)) & 1);
        }
        
        int clz = n_in;
        for (int i = n_in - 1; i >= 0; i--) {
            if ((input >> i) & 1) {
                clz = n_in - 1 - i;
                break;
            }
        }
        
        for (int bit = 0; bit < n_out; bit++) {
            int output_idx = n_out - 1 - bit;
            if (tt_get_bit(outputs[output_idx], row) != ((clz >> bit) & 1))
                return false;
        }
    }
    return true;
}

void build_clz(Circuit *c, int n_bits) {
    int out_bits = 0;
    while ((1 << out_bits) <= n_bits) out_bits++;
    
    if (n_bits == 1) {
        /* CLZ(0) = 1, CLZ(1) = 0 */
        c->output_map[0] = circuit_add_gate_alive(c, EVO_NOT, 0, 0);
        return;
    }
    
    if (n_bits == 2) {
        /* 00->10, 01->01, 10->00, 11->00 */
        int notA = circuit_add_gate_alive(c, EVO_NOT, 0, 0);
        int notB = circuit_add_gate_alive(c, EVO_NOT, 1, 0);
        /* MSB = A'B' */
        c->output_map[0] = circuit_add_gate_alive(c, EVO_AND, notA, notB);
        /* LSB = A' */
        c->output_map[1] = notA;
        return;
    }
    
    if (n_bits == 4) {
        /* 4-bit CLZ: outputs 0-4 (3 bits needed) */
        int A = 0, B = 1, C = 2, D = 3;
        int notA = circuit_add_gate_alive(c, EVO_NOT, A, 0);
        int notB = circuit_add_gate_alive(c, EVO_NOT, B, 0);
        int notC = circuit_add_gate_alive(c, EVO_NOT, C, 0);
        int notD = circuit_add_gate_alive(c, EVO_NOT, D, 0);
        
        /* out[2] (MSB) = A'B'C'D' (all zeros = 4 leading zeros, but we cap at 4) */
        int notAB = circuit_add_gate_alive(c, EVO_AND, notA, notB);
        int notCD = circuit_add_gate_alive(c, EVO_AND, notC, notD);
        int all_zero = circuit_add_gate_alive(c, EVO_AND, notAB, notCD);
        
        /* out[1] = A'B' (upper half is zero) */
        int upper_zero = notAB;
        
        /* out[0] = A' XOR (A'B'C') = A'(B + B'C') = A'(B + C') when upper matters */
        /* More precisely: count = A'B'C'D'*4 + A'B'C'D*3 + A'B'CD'*2 + A'B'CD*2 + ... */
        /* Simplified: bit0 = A' & (B | (B' & C')) */
        int notB_notC = circuit_add_gate_alive(c, EVO_AND, notB, notC);
        int b_or_notbnc = circuit_add_gate_alive(c, EVO_OR, B, notB_notC);
        int bit0_term = circuit_add_gate_alive(c, EVO_AND, notA, b_or_notbnc);
        
        c->output_map[0] = all_zero;  /* Bit 2 */
        c->output_map[1] = upper_zero; /* Bit 1 */
        c->output_map[2] = bit0_term;  /* Bit 0 */
        return;
    }
    
    /* General case: fall back to letting optimizer handle it */
    /* Or implement recursive divide-and-conquer */
    for (int i = 0; i < out_bits; i++) {
        c->output_map[i] = circuit_add_gate_alive(c, EVO_XOR, 0, 0); /* Placeholder 0 */
    }
}

/* ----- TRAILING ZERO COUNT (CTZ) ----- */
bool detect_ctz(TT **outputs, int n_in, int n_out) {
    int expected_bits = 0;
    while ((1 << expected_bits) <= n_in) expected_bits++;
    if (n_out != expected_bits) return false;
    
    int total_rows = 1 << n_in;
    for (int row = 0; row < total_rows; row++) {
        int input = 0;
        for (int i = 0; i < n_in; i++) {
            input = (input << 1) | ((row >> (n_in - 1 - i)) & 1);
        }
        
        int ctz = n_in;
        for (int i = 0; i < n_in; i++) {
            if ((input >> i) & 1) {
                ctz = i;
                break;
            }
        }
        
        for (int bit = 0; bit < n_out; bit++) {
            int output_idx = n_out - 1 - bit;
            if (tt_get_bit(outputs[output_idx], row) != ((ctz >> bit) & 1))
                return false;
        }
    }
    return true;
}

void build_ctz(Circuit *c, int n_bits) {
    int out_bits = 0;
    while ((1 << out_bits) <= n_bits) out_bits++;
    
    if (n_bits == 1) {
        c->output_map[0] = circuit_add_gate_alive(c, EVO_NOT, 0, 0);
        return;
    }
    
    if (n_bits == 2) {
        /* Inputs: A=MSB(0), B=LSB(1) */
        /* CTZ: 00->2, 01->0, 10->1, 11->0 */
        int notA = circuit_add_gate_alive(c, EVO_NOT, 0, 0);
        int notB = circuit_add_gate_alive(c, EVO_NOT, 1, 0);
        /* MSB of result = A'B' (both zero) */
        c->output_map[0] = circuit_add_gate_alive(c, EVO_AND, notA, notB);
        /* LSB of result = B' & A (B is zero, A is one) = A & B' */
        c->output_map[1] = circuit_add_gate_alive(c, EVO_AND, 0, notB);
        return;
    }
    
    if (n_bits == 4) {
        /* LSB is input 3 in our convention */
        int A = 0, B = 1, C = 2, D = 3;  /* D is LSB */
        int notA = circuit_add_gate_alive(c, EVO_NOT, A, 0);
        int notB = circuit_add_gate_alive(c, EVO_NOT, B, 0);
        int notC = circuit_add_gate_alive(c, EVO_NOT, C, 0);
        int notD = circuit_add_gate_alive(c, EVO_NOT, D, 0);
        
        /* All zero = 4 trailing zeros */
        int notAB = circuit_add_gate_alive(c, EVO_AND, notA, notB);
        int notCD = circuit_add_gate_alive(c, EVO_AND, notC, notD);
        int all_zero = circuit_add_gate_alive(c, EVO_AND, notAB, notCD);
        
        /* Bit 1 of result: D'C' (lower 2 bits zero) */
        int lower_zero = notCD;
        
        /* Bit 0: D' & C (exactly 1 trailing zero) OR D'C'B' & A (exactly 3) */
        int one_tz = circuit_add_gate_alive(c, EVO_AND, notD, C);
        int three_tz = circuit_add_gate_alive(c, EVO_AND, notCD, notB);
        three_tz = circuit_add_gate_alive(c, EVO_AND, three_tz, A);
        int bit0 = circuit_add_gate_alive(c, EVO_OR, one_tz, three_tz);
        
        c->output_map[0] = all_zero;
        c->output_map[1] = lower_zero;
        c->output_map[2] = bit0;
        return;
    }
    
    /* Fallback for other sizes */
    for (int i = 0; i < out_bits; i++) {
        c->output_map[i] = circuit_add_gate_alive(c, EVO_XOR, 0, 0);
    }
}

/* ----- BIT REVERSAL ----- */
bool detect_bit_reverse(TT **outputs, int n_in, int n_out) {
    if (n_out != n_in) return false;
    int total_rows = 1 << n_in;
    
    for (int row = 0; row < total_rows; row++) {
        for (int bit = 0; bit < n_in; bit++) {
            int in_bit = (row >> (n_in - 1 - bit)) & 1;
            int out_idx = n_in - 1 - bit;  /* Reversed position */
            if (tt_get_bit(outputs[out_idx], row) != in_bit)
                return false;
        }
    }
    return true;
}

void build_bit_reverse(Circuit *c, int n_bits) {
    /* Just rewire - output[i] = input[n-1-i] */
    for (int i = 0; i < n_bits; i++) {
        c->output_map[i] = n_bits - 1 - i;
    }
}

/* ----- ABSOLUTE VALUE (signed) ----- */
bool detect_abs(TT **outputs, int n_in, int n_out) {
    if (n_out != n_in) return false;
    int total_rows = 1 << n_in;
    int sign_bit = 1 << (n_in - 1);
    int mask = (1 << n_in) - 1;
    
    for (int row = 0; row < total_rows; row++) {
        int input = 0;
        for (int i = 0; i < n_in; i++) {
            input = (input << 1) | ((row >> (n_in - 1 - i)) & 1);
        }
        
        int expected;
        if (input & sign_bit) {
            /* Negative: negate (2's complement) */
            expected = ((~input) + 1) & mask;
        } else {
            expected = input;
        }
        
        for (int bit = 0; bit < n_out; bit++) {
            int output_idx = n_out - 1 - bit;
            if (tt_get_bit(outputs[output_idx], row) != ((expected >> bit) & 1))
                return false;
        }
    }
    return true;
}

void build_abs(Circuit *c, int n_bits) {
    /* |A| = A if positive (MSB=0), -A if negative (MSB=1) */
    /* -A = ~A + 1, so |A| = (A XOR sign_extend) + sign */
    int sign = 0;  /* MSB is sign bit */
    
    /* XOR all bits with sign (conditional invert) */
    int xored[16];
    for (int i = 0; i < n_bits; i++) {
        xored[i] = circuit_add_gate_alive(c, EVO_XOR, i, sign);
    }
    
    /* Add sign bit (ripple carry add of 0 or 1) */
    /* LSB is at index n_bits-1 */
    int carry = sign;
    int sum[16];
    
    for (int i = n_bits - 1; i >= 0; i--) {
        sum[i] = circuit_add_gate_alive(c, EVO_XOR, xored[i], carry);
        if (i > 0) {
            carry = circuit_add_gate_alive(c, EVO_AND, xored[i], carry);
        }
    }
    
    for (int i = 0; i < n_bits; i++) {
        c->output_map[i] = sum[i];
    }
}

/* ----- ROTATE LEFT ----- */
bool detect_rotate_left(TT **outputs, int n_in, int n_out) {
    if (n_out != n_in) return false;
    int total_rows = 1 << n_in;
    int mask = (1 << n_in) - 1;
    
    for (int row = 0; row < total_rows; row++) {
        int input = 0;
        for (int i = 0; i < n_in; i++) {
            input = (input << 1) | ((row >> (n_in - 1 - i)) & 1);
        }
        
        /* Rotate left by 1 */
        int expected = ((input << 1) | (input >> (n_in - 1))) & mask;
        
        for (int bit = 0; bit < n_out; bit++) {
            int output_idx = n_out - 1 - bit;
            if (tt_get_bit(outputs[output_idx], row) != ((expected >> bit) & 1))
                return false;
        }
    }
    return true;
}

void build_rotate_left(Circuit *c, int n_bits) {
    /* Rotate left by 1: out[i] = in[i+1], out[MSB] = in[LSB] */
    for (int i = 0; i < n_bits - 1; i++) {
        c->output_map[i] = i + 1;
    }
    c->output_map[n_bits - 1] = 0;  /* Wrap around */
}

/* ----- ROTATE RIGHT ----- */
bool detect_rotate_right(TT **outputs, int n_in, int n_out) {
    if (n_out != n_in) return false;
    int total_rows = 1 << n_in;
    int mask = (1 << n_in) - 1;
    
    for (int row = 0; row < total_rows; row++) {
        int input = 0;
        for (int i = 0; i < n_in; i++) {
            input = (input << 1) | ((row >> (n_in - 1 - i)) & 1);
        }
        
        /* Rotate right by 1 */
        int expected = ((input >> 1) | (input << (n_in - 1))) & mask;
        
        for (int bit = 0; bit < n_out; bit++) {
            int output_idx = n_out - 1 - bit;
            if (tt_get_bit(outputs[output_idx], row) != ((expected >> bit) & 1))
                return false;
        }
    }
    return true;
}

void build_rotate_right(Circuit *c, int n_bits) {
    /* Rotate right by 1: out[i] = in[i-1], out[LSB] = in[MSB] */
    c->output_map[0] = n_bits - 1;  /* MSB gets LSB */
    for (int i = 1; i < n_bits; i++) {
        c->output_map[i] = i - 1;
    }
}

/* ----- SIGN EXTENSION ----- */
bool detect_sign_extend(TT **outputs, int n_in, int n_out) {
    if (n_out <= n_in) return false;
    int total_rows = 1 << n_in;
    
    for (int row = 0; row < total_rows; row++) {
        int sign = (row >> (n_in - 1)) & 1;  /* MSB of input in truth table */
        
        /* Check extended bits are all sign */
        for (int i = 0; i < n_out - n_in; i++) {
            if (tt_get_bit(outputs[i], row) != sign)
                return false;
        }
        
        /* Check original bits are preserved */
        for (int i = 0; i < n_in; i++) {
            int in_bit = (row >> (n_in - 1 - i)) & 1;
            if (tt_get_bit(outputs[n_out - n_in + i], row) != in_bit)
                return false;
        }
    }
    return true;
}

void build_sign_extend(Circuit *c, int n_in) {
    int n_out = c->num_outputs;
    int sign_input = 0;  /* MSB */
    
    /* Extended bits all get sign */
    for (int i = 0; i < n_out - n_in; i++) {
        c->output_map[i] = sign_input;
    }
    
    /* Original bits pass through */
    for (int i = 0; i < n_in; i++) {
        c->output_map[n_out - n_in + i] = i;
    }
}

/* ----- MAJORITY (output 1 if more than half inputs are 1) ----- */
bool detect_majority(TT **outputs, int n_in, int n_out) {
    if (n_out != 1) return false;
    int total_rows = 1 << n_in;
    int threshold = (n_in + 1) / 2;
    
    for (int row = 0; row < total_rows; row++) {
        int popcount = __builtin_popcount(row);
        int expected = (popcount >= threshold) ? 1 : 0;
        if (tt_get_bit(outputs[0], row) != expected)
            return false;
    }
    return true;
}

void build_majority(Circuit *c, int n_bits) {
    if (n_bits == 3) {
        /* MAJ(a,b,c) = ab + bc + ac = ab + c(a+b) */
        int ab = circuit_add_gate_alive(c, EVO_AND, 0, 1);
        int a_or_b = circuit_add_gate_alive(c, EVO_OR, 0, 1);
        int c_and_aorb = circuit_add_gate_alive(c, EVO_AND, 2, a_or_b);
        int result = circuit_add_gate_alive(c, EVO_OR, ab, c_and_aorb);
        c->output_map[0] = result;
        return;
    }
    
    /* General case: compare popcount to threshold */
    int threshold = (n_bits + 1) / 2;
    
    /* Build popcount comparison - this is a simplification */
    /* For small n, enumerate and build DNF */
    int result = -1;
    int total = 1 << n_bits;
    for (int pattern = 0; pattern < total; pattern++) {
        if (__builtin_popcount(pattern) >= threshold) {
            /* Build AND of this minterm */
            int term = -1;
            for (int i = 0; i < n_bits; i++) {
                int lit = ((pattern >> (n_bits - 1 - i)) & 1) ? 
                          i : circuit_add_gate_alive(c, EVO_NOT, i, 0);
                term = (term == -1) ? lit : 
                       circuit_add_gate_alive(c, EVO_AND, term, lit);
            }
            result = (result == -1) ? term :
                     circuit_add_gate_alive(c, EVO_OR, result, term);
        }
    }
    c->output_map[0] = result;
}

/* ----- HAMMING(7,4) ENCODER ----- */
bool detect_hamming_7_4_encode(TT **outputs, int n_in, int n_out) {
    if (n_in != 4 || n_out != 7) return false;
    
    /* Hamming(7,4): d1,d2,d3,d4 -> p1,p2,d1,p3,d2,d3,d4 */
    /* p1 = d1^d2^d4, p2 = d1^d3^d4, p3 = d2^d3^d4 */
    int total_rows = 1 << n_in;
    
    for (int row = 0; row < total_rows; row++) {
        int d1 = (row >> 3) & 1;
        int d2 = (row >> 2) & 1;
        int d3 = (row >> 1) & 1;
        int d4 = row & 1;
        
        int p1 = d1 ^ d2 ^ d4;
        int p2 = d1 ^ d3 ^ d4;
        int p3 = d2 ^ d3 ^ d4;
        
        int expected[7] = {p1, p2, d1, p3, d2, d3, d4};
        
        for (int i = 0; i < 7; i++) {
            if (tt_get_bit(outputs[i], row) != expected[i])
                return false;
        }
    }
    return true;
}

void build_hamming_7_4_encode(Circuit *c) {
    /* Inputs: d1=0, d2=1, d3=2, d4=3 */
    /* p1 = d1^d2^d4 */
    int d1_xor_d2 = circuit_add_gate_alive(c, EVO_XOR, 0, 1);
    int p1 = circuit_add_gate_alive(c, EVO_XOR, d1_xor_d2, 3);
    
    /* p2 = d1^d3^d4 */
    int d1_xor_d3 = circuit_add_gate_alive(c, EVO_XOR, 0, 2);
    int p2 = circuit_add_gate_alive(c, EVO_XOR, d1_xor_d3, 3);
    
    /* p3 = d2^d3^d4 */
    int d2_xor_d3 = circuit_add_gate_alive(c, EVO_XOR, 1, 2);
    int p3 = circuit_add_gate_alive(c, EVO_XOR, d2_xor_d3, 3);
    
    c->output_map[0] = p1;
    c->output_map[1] = p2;
    c->output_map[2] = 0;  /* d1 */
    c->output_map[3] = p3;
    c->output_map[4] = 1;  /* d2 */
    c->output_map[5] = 2;  /* d3 */
    c->output_map[6] = 3;  /* d4 */
}

/* ----- FULL 4-BIT S-BOX (configurable) ----- */
static int SBOX_TABLE[16] = {0x3,0x8,0xF,0x1,0xA,0x6,0x5,0xB,0xE,0xD,0x4,0x2,0x7,0x0,0x9,0xC};

bool detect_sbox_4bit(TT **outputs, int n_in, int n_out) {
    if (n_in != 4 || n_out != 4) return false;
    
    for (int row = 0; row < 16; row++) {
        int input = 0;
        for (int i = 0; i < 4; i++) {
            input = (input << 1) | ((row >> (3 - i)) & 1);
        }
        int expected = SBOX_TABLE[input];
        
        for (int bit = 0; bit < 4; bit++) {
            int output_idx = 3 - bit;
            if (tt_get_bit(outputs[output_idx], row) != ((expected >> bit) & 1))
                return false;
        }
    }
    return true;
}

/* ----- BCD ADD (two BCD digits with carry) ----- */
bool detect_bcd_add(TT **outputs, int n_in, int n_out) {
    /* 9 inputs: A[3:0], B[3:0], Cin -> 5 outputs: Sum[3:0], Cout */
    if (n_in != 9 || n_out != 5) return false;
    
    int total_rows = 1 << n_in;
    for (int row = 0; row < total_rows; row++) {
        int a = (row >> 5) & 0xF;
        int b = (row >> 1) & 0xF;
        int cin = row & 1;
        
        /* Skip invalid BCD */
        if (a > 9 || b > 9) continue;
        
        int sum = a + b + cin;
        int cout = (sum > 9) ? 1 : 0;
        if (cout) sum -= 10;
        
        for (int bit = 0; bit < 4; bit++) {
            int output_idx = 4 - bit;
            if (tt_get_bit(outputs[output_idx], row) != ((sum >> bit) & 1))
                return false;
        }
        if (tt_get_bit(outputs[0], row) != cout)
            return false;
    }
    return true;
}

/* ----- BARREL SHIFTER (variable shift amount) ----- */
bool detect_barrel_shift_left(TT **outputs, int n_in, int n_out) {
    /* n data bits + log2(n) shift bits -> n output bits */
    int shift_bits = 0;
    int data_bits = n_out;
    while ((1 << shift_bits) < data_bits) shift_bits++;
    
    if (n_in != data_bits + shift_bits) return false;
    
    int total_rows = 1 << n_in;
    int mask = (1 << data_bits) - 1;
    
    for (int row = 0; row < total_rows; row++) {
        int data = 0, shift = 0;
        for (int i = 0; i < data_bits; i++) {
            data = (data << 1) | ((row >> (n_in - 1 - i)) & 1);
        }
        for (int i = 0; i < shift_bits; i++) {
            shift = (shift << 1) | ((row >> (shift_bits - 1 - i)) & 1);
        }
        
        int expected = (data << shift) & mask;
        
        for (int bit = 0; bit < data_bits; bit++) {
            int output_idx = data_bits - 1 - bit;
            if (tt_get_bit(outputs[output_idx], row) != ((expected >> bit) & 1))
                return false;
        }
    }
    return true;
}

void build_barrel_shift_left(Circuit *c, int data_bits, int shift_bits) {
    /* Build using mux stages */
    int wires[16];
    for (int i = 0; i < data_bits; i++) {
        wires[i] = i;  /* Data inputs */
    }
    
    int shift_input_base = data_bits;
    
    for (int stage = 0; stage < shift_bits; stage++) {
        int shift_amount = 1 << stage;
        int sel = shift_input_base + shift_bits - 1 - stage;
        int not_sel = circuit_add_gate_alive(c, EVO_NOT, sel, 0);
        
        int new_wires[16];
        for (int i = 0; i < data_bits; i++) {
            int unshifted = wires[i];
            int shifted_src = i + shift_amount;
            int shifted = (shifted_src < data_bits) ? wires[shifted_src] :
                          circuit_add_gate_alive(c, EVO_XOR, 0, 0);  /* Zero */
            
            /* MUX: sel ? shifted : unshifted */
            int and_sel = circuit_add_gate_alive(c, EVO_AND, sel, shifted);
            int and_not_sel = circuit_add_gate_alive(c, EVO_AND, not_sel, unshifted);
            new_wires[i] = circuit_add_gate_alive(c, EVO_OR, and_sel, and_not_sel);
        }
        
        for (int i = 0; i < data_bits; i++) {
            wires[i] = new_wires[i];
        }
    }
    
    for (int i = 0; i < data_bits; i++) {
        c->output_map[i] = wires[i];
    }
}


/* ============================================================
 * 7-SEGMENT DISPLAY DECODERS
 * ============================================================ */

/*
 * 7-Segment Layout:
 *     aaaa
 *    f    b
 *    f    b
 *     gggg
 *    e    c
 *    e    c
 *     dddd
 *
 * Output order: a,b,c,d,e,f,g (active high)
 */

/* Standard 7-segment patterns for digits 0-9 and hex A-F */
static const uint8_t SEG7_PATTERNS[16] = {
    0x7E, /* 0: 1111110 = abcdef  */
    0x30, /* 1: 0110000 = bc      */
    0x6D, /* 2: 1101101 = abdeg   */
    0x79, /* 3: 1111001 = abcdg   */
    0x33, /* 4: 0110011 = bcfg    */
    0x5B, /* 5: 1011011 = acdfg   */
    0x5F, /* 6: 1011111 = acdefg  */
    0x70, /* 7: 1110000 = abc     */
    0x7F, /* 8: 1111111 = abcdefg */
    0x7B, /* 9: 1111011 = abcdfg  */
    0x77, /* A: 1110111 = abcefg  */
    0x1F, /* b: 0011111 = cdefg   */
    0x4E, /* C: 1001110 = adef    */
    0x3D, /* d: 0111101 = bcdeg   */
    0x4F, /* E: 1001111 = adefg   */
    0x47, /* F: 1000111 = aefg    */
};

/* Alternative patterns (some displays use different representations) */
static const uint8_t SEG7_PATTERNS_ALT[16] = {
    0x7E, 0x30, 0x6D, 0x79, 0x33, 0x5B, 0x5F, 0x70,
    0x7F, 0x7B, 0x77, 0x1F, 0x0E, 0x3D, 0x4F, 0x47,
};

/* ----- BCD TO 7-SEGMENT (0-9 only, with don't cares for 10-15) ----- */
/* ----- BCD TO 7-SEGMENT DETECTOR (0-9 Only) ----- */
bool detect_bcd_to_7seg(TT **outputs, int n_in, int n_out) {
    if (n_in != 4 || n_out != 7) return false;

    // Matches your exact Truth Table bitstrings
    static const char* bcd_patterns[] = {
        "1111110", // 0
        "0110000", // 1
        "1101101", // 2
        "1111001", // 3
        "0110011", // 4
        "1011011", // 5
        "1011111", // 6
        "1110000", // 7
        "1111111", // 8
        "1111011"  // 9
    };

    for (int row = 0; row < 10; row++) {
        const char* p = bcd_patterns[row];
        for (int seg = 0; seg < 7; seg++) {
            int expected = (p[seg] == '1');
            if (tt_get_bit(outputs[seg], row) != expected) return false;
        }
    }
    return true; // Rows 10-15 are treated as Don't Cares
}

/* ----- BCD TO 7-SEGMENT BUILDER (Optimized) ----- */
void build_bcd_to_7seg(Circuit *c) {
    int A = 0, B = 1, C = 2, D = 3;

    // 1. Primary Inversions
    int nB = circuit_add_gate_alive(c, EVO_NOT, B, 0);
    int nC = circuit_add_gate_alive(c, EVO_NOT, C, 0);
    int nD = circuit_add_gate_alive(c, EVO_NOT, D, 0);

    // 2. Shared Logic Core (The "Logic Engine")
    // These terms are carefully chosen because they appear in multiple segments
    int B_xor_C   = circuit_add_gate_alive(c, EVO_XOR, B, C);
    int B_xnor_D  = circuit_add_gate_alive(c, EVO_XNOR, B, D);
    int C_xnor_D  = circuit_add_gate_alive(c, EVO_XNOR, C, D);
    
    int nB_and_nD = circuit_add_gate_alive(c, EVO_AND, nB, nD);
    int C_and_nD  = circuit_add_gate_alive(c, EVO_AND, C, nD);
    int nB_and_C  = circuit_add_gate_alive(c, EVO_AND, nB, C);
    
    // This specific term is shared between segments d and f
    int B_and_nC  = circuit_add_gate_alive(c, EVO_AND, B, nC);
    int BnC_and_D = circuit_add_gate_alive(c, EVO_AND, B_and_nC, D);

    // 3. Segment Assembly (Factored to reuse previous OR gates)
    
    // Segment e (Built first because it's a sub-component of d)
    int seg_e = circuit_add_gate_alive(c, EVO_OR, nB_and_nD, C_and_nD);
    c->output_map[4] = seg_e;

    // Segment g (Factored to be reused in segment a)
    int g_partial = circuit_add_gate_alive(c, EVO_OR, A, B_xor_C);
    int seg_g     = circuit_add_gate_alive(c, EVO_OR, g_partial, C_and_nD);
    c->output_map[6] = seg_g;

    // Segment a (Reuses A | C and B_xnor_D)
    int a_partial = circuit_add_gate_alive(c, EVO_OR, A, C);
    c->output_map[0] = circuit_add_gate_alive(c, EVO_OR, a_partial, B_xnor_D);

    // Segment b (Very simple)
    c->output_map[1] = circuit_add_gate_alive(c, EVO_OR, nB, C_xnor_D);

    // Segment c (Reuses B | nC)
    int c_partial = circuit_add_gate_alive(c, EVO_OR, B, nC);
    c->output_map[2] = circuit_add_gate_alive(c, EVO_OR, c_partial, D);

    // Segment d (Highly factored: uses seg_e and nB_and_C)
    int d_partial = circuit_add_gate_alive(c, EVO_OR, A, nB_and_C);
    int d_mid     = circuit_add_gate_alive(c, EVO_OR, d_partial, seg_e);
    c->output_map[3] = circuit_add_gate_alive(c, EVO_OR, d_mid, BnC_and_D);

    // Segment f (Reuses BnC_and_D)
    int f_nCnD    = circuit_add_gate_alive(c, EVO_AND, nC, nD);
    int f_BnD     = circuit_add_gate_alive(c, EVO_AND, B, nD);
    int f_top     = circuit_add_gate_alive(c, EVO_OR, A, f_nCnD);
    int f_bot     = circuit_add_gate_alive(c, EVO_OR, f_BnD, BnC_and_D);
    c->output_map[5] = circuit_add_gate_alive(c, EVO_OR, f_top, f_bot);
}

/* ----- HEXADECIMAL TO 7-SEGMENT (0-F, all 16 values) ----- */
bool detect_hex_to_7seg(TT **outputs, int n_in, int n_out) {
    if (n_in != 4 || n_out != 7) return false;
    
    for (int row = 0; row < 16; row++) {
        int input = 0;
        for (int i = 0; i < 4; i++) {
            input = (input << 1) | ((row >> (3 - i)) & 1);
        }
        
        uint8_t expected = SEG7_PATTERNS[input];
        
        for (int seg = 0; seg < 7; seg++) {
            int exp_bit = (expected >> (6 - seg)) & 1;
            if (tt_get_bit(outputs[seg], row) != exp_bit)
                return false;
        }
    }
    return true;
}

void build_hex_to_7seg(Circuit *c) {
    /* Full hex decoder - more complex than BCD */
    int A = 0, B = 1, C = 2, D = 3;
    
    int notA = circuit_add_gate_alive(c, EVO_NOT, A, 0);
    int notB = circuit_add_gate_alive(c, EVO_NOT, B, 0);
    int notC = circuit_add_gate_alive(c, EVO_NOT, C, 0);
    int notD = circuit_add_gate_alive(c, EVO_NOT, D, 0);
    
    /* Pre-compute common products */
    int AB = circuit_add_gate_alive(c, EVO_AND, A, B);
    int AC = circuit_add_gate_alive(c, EVO_AND, A, C);
    int AD = circuit_add_gate_alive(c, EVO_AND, A, D);
    int BC = circuit_add_gate_alive(c, EVO_AND, B, C);
    int BD = circuit_add_gate_alive(c, EVO_AND, B, D);
    int CD = circuit_add_gate_alive(c, EVO_AND, C, D);
    
    int notA_B = circuit_add_gate_alive(c, EVO_AND, notA, B);
    int notA_C = circuit_add_gate_alive(c, EVO_AND, notA, C);
    int notA_D = circuit_add_gate_alive(c, EVO_AND, notA, D);
    int notB_C = circuit_add_gate_alive(c, EVO_AND, notB, C);
    int notB_D = circuit_add_gate_alive(c, EVO_AND, notB, D);
    int notC_D = circuit_add_gate_alive(c, EVO_AND, notC, D);
    
    int A_notB = circuit_add_gate_alive(c, EVO_AND, A, notB);
    int A_notC = circuit_add_gate_alive(c, EVO_AND, A, notC);
    int A_notD = circuit_add_gate_alive(c, EVO_AND, A, notD);
    int B_notC = circuit_add_gate_alive(c, EVO_AND, B, notC);
    int B_notD = circuit_add_gate_alive(c, EVO_AND, B, notD);
    int C_notD = circuit_add_gate_alive(c, EVO_AND, C, notD);
    
    int notA_notB = circuit_add_gate_alive(c, EVO_AND, notA, notB);
    int notA_notC = circuit_add_gate_alive(c, EVO_AND, notA, notC);
    int notA_notD = circuit_add_gate_alive(c, EVO_AND, notA, notD);
    int notB_notC = circuit_add_gate_alive(c, EVO_AND, notB, notC);
    int notB_notD = circuit_add_gate_alive(c, EVO_AND, notB, notD);
    int notC_notD = circuit_add_gate_alive(c, EVO_AND, notC, notD);
    
    /* Segment a */
    int a1 = circuit_add_gate_alive(c, EVO_AND, notA_notB, notD);
    int a2 = notA_C;
    int a3 = circuit_add_gate_alive(c, EVO_AND, notA_B, D);
    int a4 = circuit_add_gate_alive(c, EVO_AND, A_notB, notC);
    int a5 = circuit_add_gate_alive(c, EVO_AND, notB_C, notD);
    int a6 = circuit_add_gate_alive(c, EVO_AND, AC, notD);
    int a_t1 = circuit_add_gate_alive(c, EVO_OR, a1, a2);
    int a_t2 = circuit_add_gate_alive(c, EVO_OR, a3, a4);
    int a_t3 = circuit_add_gate_alive(c, EVO_OR, a5, a6);
    int a_t4 = circuit_add_gate_alive(c, EVO_OR, a_t1, a_t2);
    int seg_a = circuit_add_gate_alive(c, EVO_OR, a_t4, a_t3);
    
    /* Segment b */
    int b1 = notA_notB;
    int b2 = circuit_add_gate_alive(c, EVO_AND, notA, notC_notD);
    int b3 = circuit_add_gate_alive(c, EVO_AND, notA, CD);
    int b4 = circuit_add_gate_alive(c, EVO_AND, A, notC_notD);
    int b5 = circuit_add_gate_alive(c, EVO_AND, A_notB, notD);
    int b_t1 = circuit_add_gate_alive(c, EVO_OR, b1, b2);
    int b_t2 = circuit_add_gate_alive(c, EVO_OR, b3, b4);
    int b_t3 = circuit_add_gate_alive(c, EVO_OR, b_t1, b_t2);
    int seg_b = circuit_add_gate_alive(c, EVO_OR, b_t3, b5);
    
    /* Segment c */
    int c1 = notA_notC;
    int c2 = notA_D;
    int c3 = circuit_add_gate_alive(c, EVO_AND, notA, B);
    int c4 = circuit_add_gate_alive(c, EVO_AND, A, notC);
    int c5 = circuit_add_gate_alive(c, EVO_AND, A_notB, notD);
    int c_t1 = circuit_add_gate_alive(c, EVO_OR, c1, c2);
    int c_t2 = circuit_add_gate_alive(c, EVO_OR, c3, c4);
    int c_t3 = circuit_add_gate_alive(c, EVO_OR, c_t1, c_t2);
    int seg_c = circuit_add_gate_alive(c, EVO_OR, c_t3, c5);
    
    /* Segment d */
    int d1 = circuit_add_gate_alive(c, EVO_AND, notA_notB, notD);
    int d2 = circuit_add_gate_alive(c, EVO_AND, notA_notB, C);
    int d3 = circuit_add_gate_alive(c, EVO_AND, notA_B, notC);
    int d4 = circuit_add_gate_alive(c, EVO_AND, B_notC, D);
    int d5 = circuit_add_gate_alive(c, EVO_AND, A_notB, C);
    int d6 = circuit_add_gate_alive(c, EVO_AND, A, notC_D);
    int d_t1 = circuit_add_gate_alive(c, EVO_OR, d1, d2);
    int d_t2 = circuit_add_gate_alive(c, EVO_OR, d3, d4);
    int d_t3 = circuit_add_gate_alive(c, EVO_OR, d5, d6);
    int d_t4 = circuit_add_gate_alive(c, EVO_OR, d_t1, d_t2);
    int seg_d = circuit_add_gate_alive(c, EVO_OR, d_t4, d_t3);
    
    /* Segment e */
    int e1 = circuit_add_gate_alive(c, EVO_AND, notA, C_notD);
    int e2 = notB_notD;
    int e3 = circuit_add_gate_alive(c, EVO_AND, A_notB, notC);
    int e_t1 = circuit_add_gate_alive(c, EVO_OR, e1, e2);
    int seg_e = circuit_add_gate_alive(c, EVO_OR, e_t1, e3);
    
    /* Segment f */
    int f1 = circuit_add_gate_alive(c, EVO_AND, notA_notB, notC);
    int f2 = circuit_add_gate_alive(c, EVO_AND, notA_notB, D);
    int f3 = circuit_add_gate_alive(c, EVO_AND, notA, C_notD);
    int f4 = circuit_add_gate_alive(c, EVO_AND, A_notB, notD);
    int f5 = circuit_add_gate_alive(c, EVO_AND, A, notC_notD);
    int f_t1 = circuit_add_gate_alive(c, EVO_OR, f1, f2);
    int f_t2 = circuit_add_gate_alive(c, EVO_OR, f3, f4);
    int f_t3 = circuit_add_gate_alive(c, EVO_OR, f_t1, f_t2);
    int seg_f = circuit_add_gate_alive(c, EVO_OR, f_t3, f5);
    
    /* Segment g - FIXED: A_B -> AB */
    int g1 = circuit_add_gate_alive(c, EVO_AND, notA_notB, C);
    int g2 = circuit_add_gate_alive(c, EVO_AND, notA_B, notC);
    int g3 = circuit_add_gate_alive(c, EVO_AND, A_notB, C);
    int g4 = circuit_add_gate_alive(c, EVO_AND, AB, notC);  /* FIXED: was A_B */
    int g5 = circuit_add_gate_alive(c, EVO_AND, notA, C_notD);
    int g_t1 = circuit_add_gate_alive(c, EVO_OR, g1, g2);
    int g_t2 = circuit_add_gate_alive(c, EVO_OR, g3, g4);
    int g_t3 = circuit_add_gate_alive(c, EVO_OR, g_t1, g_t2);
    int seg_g = circuit_add_gate_alive(c, EVO_OR, g_t3, g5);
    
    
    c->output_map[0] = seg_a;
    c->output_map[1] = seg_b;
    c->output_map[2] = seg_c;
    c->output_map[3] = seg_d;
    c->output_map[4] = seg_e;
    c->output_map[5] = seg_f;
    c->output_map[6] = seg_g;
}

/* ----- 7-SEGMENT ACTIVE LOW (Common Anode) ----- */
bool detect_7seg_active_low(TT **outputs, int n_in, int n_out) {
    if (n_in != 4 || n_out != 7) return false;
    
    for (int row = 0; row < 16; row++) {
        int input = 0;
        for (int i = 0; i < 4; i++) {
            input = (input << 1) | ((row >> (3 - i)) & 1);
        }
        
        /* Inverted pattern */
        uint8_t expected = ~SEG7_PATTERNS[input] & 0x7F;
        
        for (int seg = 0; seg < 7; seg++) {
            int exp_bit = (expected >> (6 - seg)) & 1;
            if (tt_get_bit(outputs[seg], row) != exp_bit)
                return false;
        }
    }
    return true;
}

void build_7seg_active_low(Circuit *c) {
    /* Build active high first, then invert all outputs */
    build_hex_to_7seg(c);
    
    for (int i = 0; i < 7; i++) {
        int inverted = circuit_add_gate_alive(c, EVO_NOT, c->output_map[i], 0);
        c->output_map[i] = inverted;
    }
}


/* ============================================================
 * BINARY/BCD DECODERS
 * ============================================================ */

/* ----- BCD TO DECIMAL (1-of-10 decoder) ----- */
bool detect_bcd_to_decimal(TT **outputs, int n_in, int n_out) {
    if (n_in != 4 || n_out != 10) return false;
    
    for (int row = 0; row < 16; row++) {
        int input = 0;
        for (int i = 0; i < 4; i++) {
            input = (input << 1) | ((row >> (3 - i)) & 1);
        }
        
        for (int out = 0; out < 10; out++) {
            int expected = (input == out && input < 10) ? 1 : 0;
            if (tt_get_bit(outputs[out], row) != expected)
                return false;
        }
    }
    return true;
}

void build_bcd_to_decimal(Circuit *c) {
    int A = 0, B = 1, C = 2, D = 3;
    
    int notA = circuit_add_gate_alive(c, EVO_NOT, A, 0);
    int notB = circuit_add_gate_alive(c, EVO_NOT, B, 0);
    int notC = circuit_add_gate_alive(c, EVO_NOT, C, 0);
    int notD = circuit_add_gate_alive(c, EVO_NOT, D, 0);
    
    /* Output 0: A'B'C'D' */
    int t0_1 = circuit_add_gate_alive(c, EVO_AND, notA, notB);
    int t0_2 = circuit_add_gate_alive(c, EVO_AND, notC, notD);
    c->output_map[0] = circuit_add_gate_alive(c, EVO_AND, t0_1, t0_2);
    
    /* Output 1: A'B'C'D */
    int t1_1 = circuit_add_gate_alive(c, EVO_AND, notA, notB);
    int t1_2 = circuit_add_gate_alive(c, EVO_AND, notC, D);
    c->output_map[1] = circuit_add_gate_alive(c, EVO_AND, t1_1, t1_2);
    
    /* Output 2: A'B'CD' */
    int t2_1 = circuit_add_gate_alive(c, EVO_AND, notA, notB);
    int t2_2 = circuit_add_gate_alive(c, EVO_AND, C, notD);
    c->output_map[2] = circuit_add_gate_alive(c, EVO_AND, t2_1, t2_2);
    
    /* Output 3: A'B'CD */
    int t3_1 = circuit_add_gate_alive(c, EVO_AND, notA, notB);
    int t3_2 = circuit_add_gate_alive(c, EVO_AND, C, D);
    c->output_map[3] = circuit_add_gate_alive(c, EVO_AND, t3_1, t3_2);
    
    /* Output 4: A'BC'D' */
    int t4_1 = circuit_add_gate_alive(c, EVO_AND, notA, B);
    int t4_2 = circuit_add_gate_alive(c, EVO_AND, notC, notD);
    c->output_map[4] = circuit_add_gate_alive(c, EVO_AND, t4_1, t4_2);
    
    /* Output 5: A'BC'D */
    int t5_1 = circuit_add_gate_alive(c, EVO_AND, notA, B);
    int t5_2 = circuit_add_gate_alive(c, EVO_AND, notC, D);
    c->output_map[5] = circuit_add_gate_alive(c, EVO_AND, t5_1, t5_2);
    
    /* Output 6: A'BCD' */
    int t6_1 = circuit_add_gate_alive(c, EVO_AND, notA, B);
    int t6_2 = circuit_add_gate_alive(c, EVO_AND, C, notD);
    c->output_map[6] = circuit_add_gate_alive(c, EVO_AND, t6_1, t6_2);
    
    /* Output 7: A'BCD */
    int t7_1 = circuit_add_gate_alive(c, EVO_AND, notA, B);
    int t7_2 = circuit_add_gate_alive(c, EVO_AND, C, D);
    c->output_map[7] = circuit_add_gate_alive(c, EVO_AND, t7_1, t7_2);
    
    /* Output 8: AB'C'D' */
    int t8_1 = circuit_add_gate_alive(c, EVO_AND, A, notB);
    int t8_2 = circuit_add_gate_alive(c, EVO_AND, notC, notD);
    c->output_map[8] = circuit_add_gate_alive(c, EVO_AND, t8_1, t8_2);
    
    /* Output 9: AB'C'D */
    int t9_1 = circuit_add_gate_alive(c, EVO_AND, A, notB);
    int t9_2 = circuit_add_gate_alive(c, EVO_AND, notC, D);
    c->output_map[9] = circuit_add_gate_alive(c, EVO_AND, t9_1, t9_2);
}

/* ----- 2-TO-4 DECODER ----- */
bool detect_2to4_decoder(TT **outputs, int n_in, int n_out) {
    if (n_in != 2 || n_out != 4) return false;
    
    for (int row = 0; row < 4; row++) {
        int input = 0;
        for (int i = 0; i < 2; i++) {
            input = (input << 1) | ((row >> (1 - i)) & 1);
        }
        
        for (int out = 0; out < 4; out++) {
            int expected = (input == out) ? 1 : 0;
            if (tt_get_bit(outputs[out], row) != expected)
                return false;
        }
    }
    return true;
}

void build_2to4_decoder(Circuit *c) {
    int A = 0, B = 1;
    int notA = circuit_add_gate_alive(c, EVO_NOT, A, 0);
    int notB = circuit_add_gate_alive(c, EVO_NOT, B, 0);
    
    c->output_map[0] = circuit_add_gate_alive(c, EVO_AND, notA, notB);  /* 00 */
    c->output_map[1] = circuit_add_gate_alive(c, EVO_AND, notA, B);     /* 01 */
    c->output_map[2] = circuit_add_gate_alive(c, EVO_AND, A, notB);     /* 10 */
    c->output_map[3] = circuit_add_gate_alive(c, EVO_AND, A, B);        /* 11 */
}

/* ----- 3-TO-8 DECODER ----- */
bool detect_3to8_decoder(TT **outputs, int n_in, int n_out) {
    if (n_in != 3 || n_out != 8) return false;
    
    for (int row = 0; row < 8; row++) {
        int input = 0;
        for (int i = 0; i < 3; i++) {
            input = (input << 1) | ((row >> (2 - i)) & 1);
        }
        
        for (int out = 0; out < 8; out++) {
            int expected = (input == out) ? 1 : 0;
            if (tt_get_bit(outputs[out], row) != expected)
                return false;
        }
    }
    return true;
}

void build_3to8_decoder(Circuit *c) {
    int A = 0, B = 1, C_in = 2;
    int notA = circuit_add_gate_alive(c, EVO_NOT, A, 0);
    int notB = circuit_add_gate_alive(c, EVO_NOT, B, 0);
    int notC = circuit_add_gate_alive(c, EVO_NOT, C_in, 0);
    
    int AB[4], notA_B[4];
    AB[0] = circuit_add_gate_alive(c, EVO_AND, notA, notB);
    AB[1] = circuit_add_gate_alive(c, EVO_AND, notA, B);
    AB[2] = circuit_add_gate_alive(c, EVO_AND, A, notB);
    AB[3] = circuit_add_gate_alive(c, EVO_AND, A, B);
    
    for (int i = 0; i < 4; i++) {
        c->output_map[i] = circuit_add_gate_alive(c, EVO_AND, AB[i], notC);
        c->output_map[i + 4] = circuit_add_gate_alive(c, EVO_AND, AB[i], C_in);
    }
}

/* ----- DECODER WITH ENABLE ----- */
bool detect_decoder_with_enable(TT **outputs, int n_in, int n_out) {
    /* n-1 address bits + 1 enable = 2^(n-1) outputs */
    int addr_bits = n_in - 1;
    if (n_out != (1 << addr_bits)) return false;
    if (addr_bits < 1 || addr_bits > 4) return false;
    
    int total_rows = 1 << n_in;
    for (int row = 0; row < total_rows; row++) {
        int enable = (row >> (n_in - 1)) & 1;  /* MSB is enable */
        int addr = 0;
        for (int i = 1; i < n_in; i++) {
            addr = (addr << 1) | ((row >> (n_in - 1 - i)) & 1);
        }
        
        for (int out = 0; out < n_out; out++) {
            int expected = (enable && addr == out) ? 1 : 0;
            if (tt_get_bit(outputs[out], row) != expected)
                return false;
        }
    }
    return true;
}

void build_decoder_with_enable(Circuit *c, int addr_bits) {
    int enable = 0;  /* First input is enable */
    
    /* Build basic decoder for address bits */
    int not_addr[4];
    for (int i = 0; i < addr_bits; i++) {
        not_addr[i] = circuit_add_gate_alive(c, EVO_NOT, i + 1, 0);
    }
    
    for (int out = 0; out < (1 << addr_bits); out++) {
        int term = enable;
        for (int bit = 0; bit < addr_bits; bit++) {
            int use_true = (out >> (addr_bits - 1 - bit)) & 1;
            int addr_wire = use_true ? (bit + 1) : not_addr[bit];
            term = circuit_add_gate_alive(c, EVO_AND, term, addr_wire);
        }
        c->output_map[out] = term;
    }
}


/* ============================================================
 * ENCODERS
 * ============================================================ */

/* ----- 4-TO-2 ENCODER ----- */
bool detect_4to2_encoder(TT **outputs, int n_in, int n_out) {
    if (n_in != 4 || n_out != 2) return false;
    
    /* One-hot input to binary */
    int valid_inputs[] = {1, 2, 4, 8};  /* 0001, 0010, 0100, 1000 */
    int expected_outputs[] = {0, 1, 2, 3};
    
    for (int i = 0; i < 4; i++) {
        int row = valid_inputs[i];
        int expected = expected_outputs[i];
        
        for (int bit = 0; bit < 2; bit++) {
            int exp_bit = (expected >> (1 - bit)) & 1;
            if (tt_get_bit(outputs[bit], row) != exp_bit)
                return false;
        }
    }
    return true;
}

void build_4to2_encoder(Circuit *c) {
    /* One-hot to binary: D3D2D1D0 -> Y1Y0 */
    /* Y1 = D3 + D2, Y0 = D3 + D1 */
    int D0 = 3, D1 = 2, D2 = 1, D3 = 0;  /* MSB first */
    
    c->output_map[0] = circuit_add_gate_alive(c, EVO_OR, D3, D2);  /* Y1 */
    c->output_map[1] = circuit_add_gate_alive(c, EVO_OR, D3, D1);  /* Y0 */
}

/* ----- 8-TO-3 ENCODER ----- */
bool detect_8to3_encoder(TT **outputs, int n_in, int n_out) {
    if (n_in != 8 || n_out != 3) return false;
    
    for (int i = 0; i < 8; i++) {
        int row = 1 << (7 - i);  /* One-hot */
        
        for (int bit = 0; bit < 3; bit++) {
            int exp_bit = (i >> (2 - bit)) & 1;
            if (tt_get_bit(outputs[bit], row) != exp_bit)
                return false;
        }
    }
    return true;
}

void build_8to3_encoder(Circuit *c) {
    /* Y2 = D7+D6+D5+D4, Y1 = D7+D6+D3+D2, Y0 = D7+D5+D3+D1 */
    int D[8];
    for (int i = 0; i < 8; i++) D[i] = i;
    
    /* Y2 = D4 + D5 + D6 + D7 */
    int y2_t1 = circuit_add_gate_alive(c, EVO_OR, D[4], D[5]);
    int y2_t2 = circuit_add_gate_alive(c, EVO_OR, D[6], D[7]);
    c->output_map[0] = circuit_add_gate_alive(c, EVO_OR, y2_t1, y2_t2);
    
    /* Y1 = D2 + D3 + D6 + D7 */
    int y1_t1 = circuit_add_gate_alive(c, EVO_OR, D[2], D[3]);
    int y1_t2 = circuit_add_gate_alive(c, EVO_OR, D[6], D[7]);
    c->output_map[1] = circuit_add_gate_alive(c, EVO_OR, y1_t1, y1_t2);
    
    /* Y0 = D1 + D3 + D5 + D7 */
    int y0_t1 = circuit_add_gate_alive(c, EVO_OR, D[1], D[3]);
    int y0_t2 = circuit_add_gate_alive(c, EVO_OR, D[5], D[7]);
    c->output_map[2] = circuit_add_gate_alive(c, EVO_OR, y0_t1, y0_t2);
}

/* ----- DECIMAL TO BCD ENCODER ----- */
bool detect_decimal_to_bcd(TT **outputs, int n_in, int n_out) {
    if (n_in != 10 || n_out != 4) return false;
    
    for (int i = 0; i < 10; i++) {
        int row = 1 << (9 - i);  /* One-hot: D9...D0 */
        
        for (int bit = 0; bit < 4; bit++) {
            int exp_bit = (i >> (3 - bit)) & 1;
            if (tt_get_bit(outputs[bit], row) != exp_bit)
                return false;
        }
    }
    return true;
}

void build_decimal_to_bcd(Circuit *c) {
    /* Inputs D0-D9, outputs A(MSB),B,C,D(LSB) */
    int D[10];
    for (int i = 0; i < 10; i++) D[i] = i;
    
    /* A = D8 + D9 */
    c->output_map[0] = circuit_add_gate_alive(c, EVO_OR, D[8], D[9]);
    
    /* B = D4 + D5 + D6 + D7 */
    int b_t1 = circuit_add_gate_alive(c, EVO_OR, D[4], D[5]);
    int b_t2 = circuit_add_gate_alive(c, EVO_OR, D[6], D[7]);
    c->output_map[1] = circuit_add_gate_alive(c, EVO_OR, b_t1, b_t2);
    
    /* C = D2 + D3 + D6 + D7 */
    int c_t1 = circuit_add_gate_alive(c, EVO_OR, D[2], D[3]);
    int c_t2 = circuit_add_gate_alive(c, EVO_OR, D[6], D[7]);
    c->output_map[2] = circuit_add_gate_alive(c, EVO_OR, c_t1, c_t2);
    
    /* D = D1 + D3 + D5 + D7 + D9 */
    int d_t1 = circuit_add_gate_alive(c, EVO_OR, D[1], D[3]);
    int d_t2 = circuit_add_gate_alive(c, EVO_OR, D[5], D[7]);
    int d_t3 = circuit_add_gate_alive(c, EVO_OR, d_t1, d_t2);
    c->output_map[3] = circuit_add_gate_alive(c, EVO_OR, d_t3, D[9]);
}


/* ============================================================
 * CODE CONVERTERS
 * ============================================================ */

/* ----- BCD TO EXCESS-3 ----- */
bool detect_bcd_to_excess3(TT **outputs, int n_in, int n_out) {
    if (n_in != 4 || n_out != 4) return false;
    
    for (int row = 0; row < 10; row++) {
        int input = 0;
        for (int i = 0; i < 4; i++) {
            input = (input << 1) | ((row >> (3 - i)) & 1);
        }
        
        int expected = input + 3;
        
        for (int bit = 0; bit < 4; bit++) {
            int exp_bit = (expected >> (3 - bit)) & 1;
            if (tt_get_bit(outputs[bit], row) != exp_bit)
                return false;
        }
    }
    return true;
}

void build_bcd_to_excess3(Circuit *c) {
    /* Add 3 to input */
    int A = 0, B = 1, C = 2, D = 3;
    
    /* W = D' (LSB flips) */
    int W = circuit_add_gate_alive(c, EVO_NOT, D, 0);
    
    /* X = C XOR D */
    int X = circuit_add_gate_alive(c, EVO_XOR, C, D);
    
    /* Y = B XOR (C AND D) */
    int CD = circuit_add_gate_alive(c, EVO_AND, C, D);
    int Y = circuit_add_gate_alive(c, EVO_XOR, B, CD);
    
    /* Z = A XOR (B AND (C OR D)) */
    int C_or_D = circuit_add_gate_alive(c, EVO_OR, C, D);
    int B_and_CorD = circuit_add_gate_alive(c, EVO_AND, B, C_or_D);
    int Z = circuit_add_gate_alive(c, EVO_XOR, A, B_and_CorD);
    
    c->output_map[0] = Z;
    c->output_map[1] = Y;
    c->output_map[2] = X;
    c->output_map[3] = W;
}

/* ----- EXCESS-3 TO BCD ----- */
bool detect_excess3_to_bcd(TT **outputs, int n_in, int n_out) {
    if (n_in != 4 || n_out != 4) return false;
    
    for (int row = 3; row < 13; row++) {  /* Valid Excess-3: 3-12 */
        int input = 0;
        for (int i = 0; i < 4; i++) {
            input = (input << 1) | ((row >> (3 - i)) & 1);
        }
        
        int expected = input - 3;
        
        for (int bit = 0; bit < 4; bit++) {
            int exp_bit = (expected >> (3 - bit)) & 1;
            if (tt_get_bit(outputs[bit], row) != exp_bit)
                return false;
        }
    }
    return true;
}

void build_excess3_to_bcd(Circuit *c) {
    int A = 0, B = 1, C = 2, D = 3;
    
    int notD = circuit_add_gate_alive(c, EVO_NOT, D, 0);
    int notC = circuit_add_gate_alive(c, EVO_NOT, C, 0);
    
    /* W = D' */
    int W = notD;
    
    /* X = C XNOR D */
    int X = circuit_add_gate_alive(c, EVO_XNOR, C, D);
    
    /* Y = B XOR (C' AND D') */
    int notC_notD = circuit_add_gate_alive(c, EVO_AND, notC, notD);
    int Y = circuit_add_gate_alive(c, EVO_XOR, B, notC_notD);
    
    /* Z = A XOR (B AND (C' OR D')) */
    int notC_or_notD = circuit_add_gate_alive(c, EVO_OR, notC, notD);
    int B_and_term = circuit_add_gate_alive(c, EVO_AND, B, notC_or_notD);
    int Z = circuit_add_gate_alive(c, EVO_XOR, A, B_and_term);
    
    c->output_map[0] = Z;
    c->output_map[1] = Y;
    c->output_map[2] = X;
    c->output_map[3] = W;
}

/* ----- BINARY TO BCD (Double Dabble for small inputs) ----- */
bool detect_binary_to_bcd(TT **outputs, int n_in, int n_out) {
    /* 4-bit binary (0-15) to 2-digit BCD (8 outputs) */
    if (n_in != 4 || n_out != 8) return false;
    
    for (int row = 0; row < 16; row++) {
        int input = 0;
        for (int i = 0; i < 4; i++) {
            input = (input << 1) | ((row >> (3 - i)) & 1);
        }
        
        int tens = input / 10;
        int ones = input % 10;
        int expected = (tens << 4) | ones;
        
        for (int bit = 0; bit < 8; bit++) {
            int exp_bit = (expected >> (7 - bit)) & 1;
            if (tt_get_bit(outputs[bit], row) != exp_bit)
                return false;
        }
    }
    return true;
}

/* ----- 7-SEGMENT TO BCD DECODER ----- */
bool detect_7seg_to_bcd(TT **outputs, int n_in, int n_out) {
    if (n_in != 7 || n_out != 4) return false;
    
    for (int digit = 0; digit < 10; digit++) {
        uint8_t pattern = SEG7_PATTERNS[digit];
        int row = 0;
        for (int seg = 0; seg < 7; seg++) {
            row = (row << 1) | ((pattern >> (6 - seg)) & 1);
        }
        
        for (int bit = 0; bit < 4; bit++) {
            int exp_bit = (digit >> (3 - bit)) & 1;
            if (tt_get_bit(outputs[bit], row) != exp_bit)
                return false;
        }
    }
    return true;
}


/* ============================================================
 * DISPLAY MULTIPLEXING & SPECIAL DECODERS
 * ============================================================ */

/* ----- DIGIT SELECT (for multiplexed displays) ----- */
bool detect_digit_select(TT **outputs, int n_in, int n_out) {
    /* 2-bit counter input -> 4 active-low digit enables */
    if (n_in != 2 || n_out != 4) return false;
    
    for (int row = 0; row < 4; row++) {
        int input = 0;
        for (int i = 0; i < 2; i++) {
            input = (input << 1) | ((row >> (1 - i)) & 1);
        }
        
        for (int out = 0; out < 4; out++) {
            /* Active low: selected digit is 0, others are 1 */
            int expected = (input == out) ? 0 : 1;
            if (tt_get_bit(outputs[out], row) != expected)
                return false;
        }
    }
    return true;
}

void build_digit_select(Circuit *c) {
    /* Active-low 2-to-4 decoder */
    int A = 0, B = 1;
    int notA = circuit_add_gate_alive(c, EVO_NOT, A, 0);
    int notB = circuit_add_gate_alive(c, EVO_NOT, B, 0);
    
    int d0 = circuit_add_gate_alive(c, EVO_AND, notA, notB);
    int d1 = circuit_add_gate_alive(c, EVO_AND, notA, B);
    int d2 = circuit_add_gate_alive(c, EVO_AND, A, notB);
    int d3 = circuit_add_gate_alive(c, EVO_AND, A, B);
    
    /* Invert for active-low */
    c->output_map[0] = circuit_add_gate_alive(c, EVO_NOT, d0, 0);
    c->output_map[1] = circuit_add_gate_alive(c, EVO_NOT, d1, 0);
    c->output_map[2] = circuit_add_gate_alive(c, EVO_NOT, d2, 0);
    c->output_map[3] = circuit_add_gate_alive(c, EVO_NOT, d3, 0);
}

/* ----- KEYBOARD ENCODER (detect any key pressed) ----- */
bool detect_keypad_encoder(TT **outputs, int n_in, int n_out) {
    /* 16-key keypad -> 4-bit code + valid signal */
    if (n_in != 16 || n_out != 5) return false;
    
    for (int key = 0; key < 16; key++) {
        int row = 1 << (15 - key);  /* One-hot key press */
        
        /* First 4 outputs are key code */
        for (int bit = 0; bit < 4; bit++) {
            int exp_bit = (key >> (3 - bit)) & 1;
            if (tt_get_bit(outputs[bit], row) != exp_bit)
                return false;
        }
        /* Output 4 is valid (any key pressed) */
        if (tt_get_bit(outputs[4], row) != 1)
            return false;
    }
    
    /* No key pressed -> valid = 0 */
    if (tt_get_bit(outputs[4], 0) != 0)
        return false;
    
    return true;
}



/* ============================================================
 * CIRCUIT OPTIMIZATION & PATTERN COMPRESSION
 * ============================================================ */

bool are_complements(Circuit *c, int wire_a, int wire_b) {
    if (wire_b >= c->num_inputs) {
        int idx = wire_b - c->num_inputs;
        if (c->gates[idx].alive && c->gates[idx].op == EVO_NOT && c->gates[idx].src_a == wire_a)
            return true;
    }
    if (wire_a >= c->num_inputs) {
        int idx = wire_a - c->num_inputs;
        if (c->gates[idx].alive && c->gates[idx].op == EVO_NOT && c->gates[idx].src_a == wire_b)
            return true;
    }
    return false;
}

int get_not_source(Circuit *c, int wire) {
    if (wire < c->num_inputs) return -1;
    int idx = wire - c->num_inputs;
    if (!c->gates[idx].alive || c->gates[idx].op != EVO_NOT) return -1;
    return c->gates[idx].src_a;
}

int eliminate_double_inversions(Circuit *c) {
    int removed = 0;
    for (int i = 0; i < c->num_gates; i++) {
        if (!c->gates[i].alive || c->gates[i].op != EVO_NOT) continue;
        int src = c->gates[i].src_a;
        if (src < c->num_inputs) continue;
        int src_idx = src - c->num_inputs;
        if (!c->gates[src_idx].alive || c->gates[src_idx].op != EVO_NOT) continue;
        
        int original = c->gates[src_idx].src_a;
        int wire_out = c->num_inputs + i;
        
        for (int j = 0; j < c->num_gates; j++) {
            if (!c->gates[j].alive) continue;
            if (c->gates[j].src_a == wire_out) c->gates[j].src_a = original;
            if (c->gates[j].op != EVO_NOT && c->gates[j].src_b == wire_out) 
                c->gates[j].src_b = original;
        }
        for (int j = 0; j < c->num_outputs; j++)
            if (c->output_map[j] == wire_out) c->output_map[j] = original;
        
        c->gates[i].alive = false;
        removed++;
    }
    return removed;
}

int eliminate_identity_gates(Circuit *c) {
    int removed = 0;
    for (int i = 0; i < c->num_gates; i++) {
        if (!c->gates[i].alive) continue;
        Gate *g = &c->gates[i];
        int wire_out = c->num_inputs + i;
        int replace_with = -1;
        
        if (g->src_a == g->src_b) {
            if (g->op == EVO_AND || g->op == EVO_OR) replace_with = g->src_a;
            else if (g->op == EVO_XOR) replace_with = -2; // Constant 0
            else if (g->op == EVO_XNOR) replace_with = -3; // Constant 1
        }
        if (are_complements(c, g->src_a, g->src_b)) {
            if (g->op == EVO_AND || g->op == EVO_NAND) replace_with = -2;
            else if (g->op == EVO_OR || g->op == EVO_NOR) replace_with = -3;
            else if (g->op == EVO_XOR) replace_with = -3;
            else if (g->op == EVO_XNOR) replace_with = -2;
        }
        
        if (replace_with >= 0) {
            for (int j = 0; j < c->num_gates; j++) {
                if (!c->gates[j].alive) continue;
                if (c->gates[j].src_a == wire_out) c->gates[j].src_a = replace_with;
                if (c->gates[j].op != EVO_NOT && c->gates[j].src_b == wire_out)
                    c->gates[j].src_b = replace_with;
            }
            for (int j = 0; j < c->num_outputs; j++)
                if (c->output_map[j] == wire_out) c->output_map[j] = replace_with;
            c->gates[i].alive = false;
            removed++;
        }
    }
    return removed;
}

int eliminate_redundant_gates(Circuit *c) {
    int removed = 0;
    for (int i = 0; i < c->num_gates; i++) {
        if (!c->gates[i].alive) continue;
        for (int j = i + 1; j < c->num_gates; j++) {
            if (!c->gates[j].alive) continue;
            if (c->gates[i].op != c->gates[j].op) continue;
            
            bool match = false;
            if (c->gates[i].op == EVO_NOT) {
                match = (c->gates[i].src_a == c->gates[j].src_a);
            } else {
                int a1 = c->gates[i].src_a, b1 = c->gates[i].src_b;
                int a2 = c->gates[j].src_a, b2 = c->gates[j].src_b;
                match = (a1 == a2 && b1 == b2) || (a1 == b2 && b1 == a2);
            }
            
            if (match) {
                int wire_dup = c->num_inputs + j;
                int wire_orig = c->num_inputs + i;
                for (int k = 0; k < c->num_gates; k++) {
                    if (!c->gates[k].alive) continue;
                    if (c->gates[k].src_a == wire_dup) c->gates[k].src_a = wire_orig;
                    if (c->gates[k].op != EVO_NOT && c->gates[k].src_b == wire_dup)
                        c->gates[k].src_b = wire_orig;
                }
                for (int k = 0; k < c->num_outputs; k++)
                    if (c->output_map[k] == wire_dup) c->output_map[k] = wire_orig;
                c->gates[j].alive = false;
                removed++;
            }
        }
    }
    return removed;
}

int detect_xor_from_and_or(Circuit *c) {
    int converted = 0;
    for (int i = 0; i < c->num_gates; i++) {
        if (!c->gates[i].alive || c->gates[i].op != EVO_OR) continue;
        
        int or_a = c->gates[i].src_a, or_b = c->gates[i].src_b;
        if (or_a < c->num_inputs || or_b < c->num_inputs) continue;
        
        int and1 = or_a - c->num_inputs, and2 = or_b - c->num_inputs;
        if (!c->gates[and1].alive || !c->gates[and2].alive) continue;
        if (c->gates[and1].op != EVO_AND || c->gates[and2].op != EVO_AND) continue;
        
        int a1 = c->gates[and1].src_a, b1 = c->gates[and1].src_b;
        int a2 = c->gates[and2].src_a, b2 = c->gates[and2].src_b;
        
        int x = -1, y = -1;
        int nb1 = get_not_source(c, b1), na2 = get_not_source(c, a2);
        if (nb1 >= 0 && na2 >= 0 && a1 == na2 && nb1 == b2) { x = a1; y = b2; }
        if (x < 0) {
            int na1 = get_not_source(c, a1), nb2 = get_not_source(c, b2);
            if (na1 >= 0 && nb2 >= 0 && b1 == nb2 && na1 == a2) { x = b1; y = a2; }
        }
        
        if (x >= 0 && y >= 0) {
            c->gates[i].op = EVO_XOR;
            c->gates[i].src_a = x;
            c->gates[i].src_b = y;
            converted++;
        }
    }
    return converted;
}

void optimize_circuit(Circuit *c) {
    printf("\n[*] Running optimization passes...\n");
    int initial = circuit_count_active(c);
    bool changed = true;
    int pass = 0;
    
    while (changed && pass < 10) {
        changed = false;
        pass++;
        int r1 = eliminate_double_inversions(c);
        int r2 = eliminate_identity_gates(c);
        int r3 = eliminate_redundant_gates(c);
        int r4 = detect_xor_from_and_or(c);
        
        if (r1 + r2 + r3 + r4 > 0) {
            changed = true;
            circuit_compact(c);
        }
    }
    
    int final = circuit_count_active(c);
    if (final < initial)
        printf("    Optimized: %d -> %d gates (-%d)\n", initial, final, initial - final);
    else
        printf("    No optimization opportunities found\n");
}


/* ============================================================
 * ALU DETECTION AND STRUCTURAL SYNTHESIS
 * ============================================================ */

bool detect_alu(TT **outputs, int n_in, int n_out) {
    /* Format: 2 opcode bits + n bits A + n bits B = 2+2n inputs
     * Outputs: n result bits + carry + zero + neg = n+3 outputs
     * Opcodes: 0=ADD, 1=SUB, 2=AND, 3=OR
     */
    if (n_in < 6) return false;  /* Minimum: 2 op + 2 A + 2 B */
    if ((n_in - 2) % 2 != 0) return false;
    
    int n = (n_in - 2) / 2;  /* Data width */
    if (n_out != n + 3) return false;  /* Result + carry + zero + neg */
    
    int total_rows = 1 << n_in;
    int max_val = 1 << n;
    int mask = max_val - 1;
    
    for (int row = 0; row < total_rows; row++) {
        /* Extract opcode, A, B (MSB first in truth table) */
        int op = (row >> (n_in - 2)) & 3;
        int a = (row >> n) & mask;
        int b = row & mask;
        
        /* Compute expected result */
        int res = 0, carry = 0;
        if (op == 0) {  /* ADD */
            res = a + b;
            carry = (res >= max_val) ? 1 : 0;
            res &= mask;
        } else if (op == 1) {  /* SUB */
            res = a - b;
            carry = (res < 0) ? 1 : 0;  /* Borrow */
            res &= mask;
        } else if (op == 2) {  /* AND */
            res = a & b;
        } else {  /* OR */
            res = a | b;
        }
        
        int zero = (res == 0) ? 1 : 0;
        int neg = (res >> (n - 1)) & 1;
        
        /* Verify outputs */
        for (int bit = 0; bit < n; bit++) {
            int output_idx = n - 1 - bit;
            if (tt_get_bit(outputs[output_idx], row) != ((res >> bit) & 1))
                return false;
        }
        if (tt_get_bit(outputs[n], row) != carry) return false;
        if (tt_get_bit(outputs[n + 1], row) != zero) return false;
        if (tt_get_bit(outputs[n + 2], row) != neg) return false;
    }
    return true;
}

void build_alu(Circuit *c, int n_bits) {
    /*
     * Structural ALU: ~15n + 7 gates (vs 1000s from truth table synthesis)
     * 
     * Inputs: op[1:0] (2 bits), A[n-1:0] (n bits), B[n-1:0] (n bits)
     * Outputs: Result[n-1:0], Carry, Zero, Negative
     * 
     * Key insight: SUB = A + ~B + 1, so we can share the adder!
     */
    int n = n_bits;
    int op_hi = 0;  /* op[1]: 0=arithmetic, 1=logic */
    int op_lo = 1;  /* op[0]: 0=ADD/AND, 1=SUB/OR */
    
    /* Input mapping macros */
    #define A_IN(i) (2 + n - 1 - (i))      /* A bit i (i=0 is LSB) */
    #define B_IN(i) (2 + n + n - 1 - (i))  /* B bit i */
    
    /* Opcode inverters */
    int not_op_hi = circuit_add_gate_alive(c, EVO_NOT, op_hi, 0);
    int not_op_lo = circuit_add_gate_alive(c, EVO_NOT, op_lo, 0);
    
    /* B XOR op_lo: inverts B when op_lo=1 (SUB) */
    int b_xor[16];
    for (int i = 0; i < n; i++) {
        b_xor[i] = circuit_add_gate_alive(c, EVO_XOR, B_IN(i), op_lo);
    }
    
    /* Adder/Subtractor: A + (B XOR op_lo) + op_lo
     * ADD: A + B + 0
     * SUB: A + ~B + 1 = A - B
     */
    int sum[16], carry[17];
    carry[0] = op_lo;  /* Carry-in: 0 for ADD, 1 for SUB */
    
    for (int i = 0; i < n; i++) {
        int a_in = A_IN(i);
        int b_in = b_xor[i];
        
        /* Full adder */
        int axb = circuit_add_gate_alive(c, EVO_XOR, a_in, b_in);
        sum[i] = circuit_add_gate_alive(c, EVO_XOR, axb, carry[i]);
        int ab = circuit_add_gate_alive(c, EVO_AND, a_in, b_in);
        int axb_c = circuit_add_gate_alive(c, EVO_AND, axb, carry[i]);
        carry[i+1] = circuit_add_gate_alive(c, EVO_OR, ab, axb_c);
    }
    
    /* Logic units */
    int and_out[16], or_out[16];
    for (int i = 0; i < n; i++) {
        and_out[i] = circuit_add_gate_alive(c, EVO_AND, A_IN(i), B_IN(i));
        or_out[i] = circuit_add_gate_alive(c, EVO_OR, A_IN(i), B_IN(i));
    }
    
    /* 4:1 MUX for each result bit
     * op=00,01 (op_hi=0) -> sum
     * op=10 (op_hi=1, op_lo=0) -> AND
     * op=11 (op_hi=1, op_lo=1) -> OR
     * 
     * result = !op_hi*sum + op_hi*!op_lo*AND + op_hi*op_lo*OR
     */
    int sel_arith = not_op_hi;  /* Select arithmetic result */
    int sel_and = circuit_add_gate_alive(c, EVO_AND, op_hi, not_op_lo);
    int sel_or = circuit_add_gate_alive(c, EVO_AND, op_hi, op_lo);
    
    int result[16];
    for (int i = 0; i < n; i++) {
        int term1 = circuit_add_gate_alive(c, EVO_AND, sel_arith, sum[i]);
        int term2 = circuit_add_gate_alive(c, EVO_AND, sel_and, and_out[i]);
        int term3 = circuit_add_gate_alive(c, EVO_AND, sel_or, or_out[i]);
        int t12 = circuit_add_gate_alive(c, EVO_OR, term1, term2);
        result[i] = circuit_add_gate_alive(c, EVO_OR, t12, term3);
    }
    
    /* Carry/Borrow flag
     * ADD: carry_out
     * SUB: borrow = !carry_out (when A + ~B + 1, carry=1 means no borrow)
     * AND/OR: 0
     * 
     * carry_flag = !op_hi * (op_lo XOR carry_out)
     */
    int carry_xor = circuit_add_gate_alive(c, EVO_XOR, op_lo, carry[n]);
    int carry_flag = circuit_add_gate_alive(c, EVO_AND, not_op_hi, carry_xor);
    
    /* Zero flag: NOR of all result bits */
    int or_tree = result[0];
    for (int i = 1; i < n; i++) {
        or_tree = circuit_add_gate_alive(c, EVO_OR, or_tree, result[i]);
    }
    int zero_flag = circuit_add_gate_alive(c, EVO_NOT, or_tree, 0);
    
    /* Negative flag: MSB of result */
    int neg_flag = result[n - 1];
    
    /* Map outputs (MSB first) */
    for (int i = 0; i < n; i++) {
        c->output_map[i] = result[n - 1 - i];
    }
    c->output_map[n] = carry_flag;
    c->output_map[n + 1] = zero_flag;
    c->output_map[n + 2] = neg_flag;
    
    #undef A_IN
    #undef B_IN
}




/* ============================================================
 * UNIVERSAL RECURSIVE SYNTHESIZER
 * Replaces the need for manual pattern matching
 * ============================================================ */

/* ============================================================
 * UNIVERSAL FIXED-ORDER SYNTHESIZER (MSB -> LSB)
 * Guaranteed correctness by mapping TT layout directly to MUX tree
 * ============================================================ */

typedef struct CircuitMemoEntry {
    TT *key;
    int wire_index;
    struct CircuitMemoEntry *next;
} CircuitMemoEntry;

CircuitMemoEntry *circ_memo[HASH_SIZE];

void circ_memo_reset() {
    for(int i = 0; i < HASH_SIZE; i++) {
        CircuitMemoEntry *e = circ_memo[i];
        while(e) {
            CircuitMemoEntry *next = e->next;
            free_tt(e->key);
            free(e);
            e = next;
        }
        circ_memo[i] = NULL;
    }
}

int circ_memo_lookup(Circuit *c, TT *t) {
    uint64_t h = hash_tt(t) % HASH_SIZE;
    CircuitMemoEntry *e = circ_memo[h];
    
    /* 1. Check for exact match */
    while(e) {
        if (tt_equals(e->key, t)) return e->wire_index;
        e = e->next;
    }
    
    /* 2. Check for complement match (Inverted Caching) */
    /* If we have built ~T, we can just return NOT(~T) */
    TT *comp = copy_tt(t);
    tt_complement(comp);
    uint64_t h_comp = hash_tt(comp) % HASH_SIZE;
    e = circ_memo[h_comp];
    
    while(e) {
        if (tt_equals(e->key, comp)) {
            free_tt(comp);
            /* Found the complement! Add a NOT gate and reuse the existing wire */
            return circuit_add_gate_alive(c, EVO_NOT, e->wire_index, 0);
        }
        e = e->next;
    }
    
    free_tt(comp);
    return -1;
}

void circ_memo_insert(TT *t, int wire) {
    uint64_t h = hash_tt(t) % HASH_SIZE;
    CircuitMemoEntry *e = (CircuitMemoEntry*)malloc(sizeof(CircuitMemoEntry));
    e->key = copy_tt(t);
    e->wire_index = wire;
    e->next = circ_memo[h];
    circ_memo[h] = e;
}

/* 
 * Splits a Truth Table exactly in half.
 * Lower half corresponds to MSB=0
 * Upper half corresponds to MSB=1
 */
void tt_split_half(TT *src, TT **out_low, TT **out_high) {
    int n = src->num_bits;
    int half_n = n / 2;
    
    *out_low = create_tt(half_n);
    *out_high = create_tt(half_n);
    
    if (n > 64) {
        // Multi-chunk split (easy copy)
        int half_chunks = src->num_chunks / 2;
        memcpy((*out_low)->chunks, src->chunks, half_chunks * sizeof(uint64_t));
        memcpy((*out_high)->chunks, src->chunks + half_chunks, half_chunks * sizeof(uint64_t));
    } else {
        // Sub-chunk split (mask and shift)
        uint64_t full = src->chunks[0];
        uint64_t mask = (1ULL << half_n) - 1;
        (*out_low)->chunks[0] = full & mask;
        (*out_high)->chunks[0] = (full >> half_n) & mask;
    }
}



bool tt_is_all_zeros(TT *t) {
    for(int i = 0; i < t->num_chunks; i++) if(t->chunks[i] != 0) return false;
    return true;
}

bool tt_is_all_ones(TT *t) {
    for(int i = 0; i < t->num_chunks; i++) if(~t->chunks[i] != 0) return false;
    return true;
}

bool tt_are_complements(TT *a, TT *b) {
    if (a->num_bits != b->num_bits) return false;
    for(int i = 0; i < a->num_chunks; i++) {
        if (a->chunks[i] != ~b->chunks[i]) return false;
    }
    return true;
}



/* Added 'int *perm' parameter to map depth back to physical wire ID */
int build_recursive_msb(Circuit *c, TT *t, int depth, int *perm) {
    /* 1. Base Case: Constant */
    int val;
    if (tt_is_const(t, &val)) {
        if (val == 0) return circuit_add_gate_alive(c, EVO_XOR, 0, 0); 
        else {
            int z = circuit_add_gate_alive(c, EVO_XOR, 0, 0);
            return circuit_add_gate_alive(c, EVO_NOT, z, 0);
        }
    }

    /* 2. Check Cache */
    int cached = circ_memo_lookup(c, t);
    if (cached != -1) return cached;

    /* 3. Split Table */
    TT *low, *high;
    tt_split_half(t, &low, &high);
    
    /* CRITICAL: The split variable is the one at 'depth' in the PERMUTED table.
       But the circuit wire for that variable is perm[depth]. */
    int sel = perm[depth]; 

    int res = -1;

    /* A: Independent */
    if (tt_equals(low, high)) {
        res = build_recursive_msb(c, low, depth + 1, perm);
    }
    /* B: XOR */
    else if (tt_are_complements(low, high)) {
        int w_low = build_recursive_msb(c, low, depth + 1, perm);
        res = circuit_add_gate_alive(c, EVO_XOR, sel, w_low);
    }
    /* C: AND (Low is 0) */
    else if (tt_is_all_zeros(low)) {
        int w_high = build_recursive_msb(c, high, depth + 1, perm);
        res = circuit_add_gate_alive(c, EVO_AND, sel, w_high);
    }
    /* D: AND (High is 0) */
    else if (tt_is_all_zeros(high)) {
        int w_low = build_recursive_msb(c, low, depth + 1, perm);
        int not_sel = circuit_add_gate_alive(c, EVO_NOT, sel, 0);
        res = circuit_add_gate_alive(c, EVO_AND, not_sel, w_low);
    }
    /* E: OR (Low is 1) */
    else if (tt_is_all_ones(low)) {
        int w_high = build_recursive_msb(c, high, depth + 1, perm);
        int not_sel = circuit_add_gate_alive(c, EVO_NOT, sel, 0);
        res = circuit_add_gate_alive(c, EVO_OR, not_sel, w_high);
    }
    /* F: OR (High is 1) */
    else if (tt_is_all_ones(high)) {
        int w_low = build_recursive_msb(c, low, depth + 1, perm);
        res = circuit_add_gate_alive(c, EVO_OR, sel, w_low);
    }
    /* G: MUX Fallback */
    else {
        int w0 = build_recursive_msb(c, low, depth + 1, perm);
        int w1 = build_recursive_msb(c, high, depth + 1, perm);
        
        int not_sel = circuit_add_gate_alive(c, EVO_NOT, sel, 0);
        int term0 = circuit_add_gate_alive(c, EVO_AND, not_sel, w0);
        int term1 = circuit_add_gate_alive(c, EVO_AND, sel, w1);
        res = circuit_add_gate_alive(c, EVO_OR, term0, term1);
    }

    free_tt(low);
    free_tt(high);
    circ_memo_insert(t, res);
    return res;
}




/* ============================================================
 * INPUT REORDERING HEURISTICS
 * Groups related variables (like A[i] and B[i]) together
 * ============================================================ */

typedef struct { int id; float score; } InputSort;

int compare_input_sort(const void *a, const void *b) {
    float diff = ((InputSort*)a)->score - ((InputSort*)b)->score;
    return (diff > 0) - (diff < 0);
}

/* 
 * Physically permutes the columns of a truth table.
 * new_tt[col i] <== old_tt[perm[i]]
 */
TT* tt_permute_cols(TT *src, int *perm, int n_vars) {
    int num_rows = src->num_bits;
    TT *dst = create_tt(num_rows);
    
    for (int r = 0; r < num_rows; r++) {
        int old_row_idx = 0;
        /* Construct the index in the old table that corresponds to row 'r' in new table */
        /* If new table has var 0 at MSB, that var comes from perm[0] in old table */
        for (int v = 0; v < n_vars; v++) {
            int bit = (r >> (n_vars - 1 - v)) & 1;
            if (bit) {
                old_row_idx |= (1 << (n_vars - 1 - perm[v]));
            }
        }
        
        if (tt_get_bit(src, old_row_idx)) {
            tt_set_bit(dst, r, 1);
        }
    }
    return dst;
}

/* Calculate a "Center of Gravity" for each input based on which outputs it affects */
void compute_smart_ordering(TT **outputs, int n_in, int n_out, int *perm) {
    InputSort *sort_arr = (InputSort*)malloc(sizeof(InputSort) * n_in);
    
    for (int i = 0; i < n_in; i++) {
        sort_arr[i].id = i;
        double sum_pos = 0;
        int count = 0;
        
        for (int o = 0; o < n_out; o++) {
            /* Check if input i affects output o using quick random sampling */
            /* (Full check is too slow, we just need a hint) */
            bool affects = false;
            for (int k = 0; k < 64; k++) { // Check 64 random pairs
                int row0 = randint(0, (1<<n_in)-1) & ~(1<< (n_in - 1 - i));
                int row1 = row0 | (1<< (n_in - 1 - i));
                if (tt_get_bit(outputs[o], row0) != tt_get_bit(outputs[o], row1)) {
                    affects = true;
                    break;
                }
            }
            
            if (affects) {
                sum_pos += o; // Output index acts as the "position"
                count++;
            }
        }
        
        // Inputs affecting the same outputs will have similar scores
        // Inputs affecting NO outputs go to the end
        if (count > 0) sort_arr[i].score = (float)(sum_pos / count);
        else sort_arr[i].score = 99999.0f;
    }
    
    qsort(sort_arr, n_in, sizeof(InputSort), compare_input_sort);
    
    printf("[*] Smart Ordering Map: ");
    for (int i = 0; i < n_in; i++) {
        perm[i] = sort_arr[i].id;
        printf("%d ", perm[i]);
    }
    printf("\n");
    
    free(sort_arr);
}




bool detect_barrel_shift_right(TT **outputs, int n_in, int n_out) {
    int shift_bits = 0;
    while ((1 << shift_bits) < n_out) shift_bits++;
    if (n_in != n_out + shift_bits) return false;

    int total_rows = 1 << n_in;
    int mask = (1 << n_out) - 1;

    for (int row = 0; row < total_rows; row++) {
        int data = (row >> shift_bits) & mask;
        int shift = row & ((1 << shift_bits) - 1); // Shift is usually LSBs in this layout
        
        // Adjust if your input format is [Data][Shift] vs [Shift][Data]
        // Assuming [Data][Shift] based on Left Shifter code
        
        int expected = (data >> shift);
        for (int bit = 0; bit < n_out; bit++) {
            if (tt_get_bit(outputs[n_out - 1 - bit], row) != ((expected >> bit) & 1))
                return false;
        }
    }
    return true;
}

void build_barrel_shift_right(Circuit *c, int data_bits, int shift_bits) {
    int wires[32];
    for (int i = 0; i < data_bits; i++) wires[i] = i; // Data starts at 0

    int shift_input_base = data_bits; // Shift inputs start after data

    for (int stage = 0; stage < shift_bits; stage++) {
        int shift_amt = 1 << stage;
        int sel = shift_input_base + stage; // LSB shift bit first
        int not_sel = circuit_add_gate_alive(c, EVO_NOT, sel, 0);

        int new_wires[32];
        for (int i = 0; i < data_bits; i++) {
            int unshifted = wires[i];
            
            // For Right shift: we pull from HIGHER index (i + shift_amt)
            // If index exceeds max, we pull a Zero (Logical Shift)
            int src_idx = i + shift_amt;
            int shifted_val;
            
            if (src_idx >= data_bits) {
                shifted_val = circuit_add_gate_alive(c, EVO_XOR, 0, 0); // Zero
            } else {
                shifted_val = wires[src_idx];
            }

            // Mux
            int t1 = circuit_add_gate_alive(c, EVO_AND, sel, shifted_val);
            int t2 = circuit_add_gate_alive(c, EVO_AND, not_sel, unshifted);
            new_wires[i] = circuit_add_gate_alive(c, EVO_OR, t1, t2);
        }
        memcpy(wires, new_wires, sizeof(wires));
    }

    for (int i = 0; i < data_bits; i++) c->output_map[i] = wires[i];
}





bool detect_demux_1toN(TT **outputs, int n_in, int n_out) {
    // 1 Data + S Select = n_in. 2^S = n_out.
    // n_in = 1 + log2(n_out)
    int s_bits = 0;
    while ((1 << s_bits) < n_out) s_bits++;
    if ((1 << s_bits) != n_out) return false;
    if (n_in != 1 + s_bits) return false;

    for (int row = 0; row < (1 << n_in); row++) {
        int data = (row >> s_bits) & 1; // MSB is Data
        int sel = row & ((1 << s_bits) - 1); // LSBs are Select

        for (int i = 0; i < n_out; i++) {
            int expected = (i == sel) ? data : 0;
            if (tt_get_bit(outputs[i], row) != expected) return false;
        }
    }
    return true;
}

void build_demux_1toN(Circuit *c, int n_out) {
    int s_bits = 0;
    while ((1 << s_bits) < n_out) s_bits++;
    
    int data_pin = 0; // MSB
    int sel_start = 1; 

    // Precompute NOTs for select lines
    int not_sel[8];
    for(int i=0; i<s_bits; i++) not_sel[i] = circuit_add_gate_alive(c, EVO_NOT, sel_start+i, 0);

    for(int i=0; i<n_out; i++) {
        int term = data_pin;
        for(int bit=0; bit<s_bits; bit++) {
            // Check if bit is set in index 'i'
            // NOTE: Check your endianness. Usually Select 0 is LSB.
            int wire = ((i >> (s_bits - 1 - bit)) & 1) ? (sel_start+bit) : not_sel[bit];
            term = circuit_add_gate_alive(c, EVO_AND, term, wire);
        }
        c->output_map[i] = term;
    }
}


/* ----- SQUARING (A^2) ----- */
bool detect_squaring(TT **outputs, int n_in, int n_out) {
    // Output width should ideally be 2*N, but we accept truncated outputs
    if (n_out > 2 * n_in) return false; 
    
    int total_rows = 1 << n_in;
    for (int row = 0; row < total_rows; row++) {
        uint64_t val = row;
        uint64_t expected = val * val;
        
        for (int bit = 0; bit < n_out; bit++) {
            // Map LSB of expected to last output index
            int output_idx = n_out - 1 - bit;
            if (tt_get_bit(outputs[output_idx], row) != ((expected >> bit) & 1))
                return false;
        }
    }
    return true;
}

void build_squaring(Circuit *c, int n_bits) {
    int n = n_bits;
    // PP = Partial Products. 
    // pp[i][j] represents bit i * bit j.
    // For squaring, inputs are the same, so pp[i][j] = A[i] AND A[j].
    int pp[16][16];
    
    // 1. Generate Partial Products
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            int a_in = n - 1 - i; // Bit i
            int b_in = n - 1 - j; // Bit j (Same input source)
            
            if (i == j) {
                // Optimization: A[i] AND A[i] = A[i]
                // The diagonal of the square matrix is just the input bits
                pp[i][j] = a_in; 
            } else {
                // Standard partial product
                pp[i][j] = circuit_add_gate_alive(c, EVO_AND, a_in, b_in);
            }
        }
    }
    
    // 2. Reduction Tree (Wallance/Dadda style column compression)
    // This accumulates bits with the same weight (column)
    int result[64];          // Final sum per column
    int carries[64][64];     // Carries generated into specific columns
    int carry_count[64] = {0};
    
    // LSB (Column 0) is always just A[0] (since A[0]*A[0] = A[0])
    result[0] = pp[0][0];
    
    // Iterate through columns from weight 2^1 to 2^(2n-2)
    for (int col = 1; col < 2*n - 1; col++) {
        int bits[128];
        int bit_count = 0;
        
        // Collect partial products for this column magnitude
        for (int i = 0; i < n; i++) {
            int j = col - i;
            if (j >= 0 && j < n) {
                bits[bit_count++] = pp[i][j];
            }
        }
        
        // Collect carries from previous columns
        for (int i = 0; i < carry_count[col]; i++) {
            bits[bit_count++] = carries[col][i];
        }
        
        // Reduce this column to a single bit using Adders
        // While we have 3 or more bits, use Full Adders
        // If we have 2 bits left, use Half Adder
        while (bit_count > 1) {
            int new_bits[128], new_count = 0, i = 0;
            while (i < bit_count) {
                if (i + 2 < bit_count) {
                    // Full Adder: 3 inputs -> 1 Sum (current col), 1 Carry (next col)
                    int sum, cout;
                    build_full_adder(c, bits[i], bits[i+1], bits[i+2], &sum, &cout);
                    new_bits[new_count++] = sum;
                    carries[col+1][carry_count[col+1]++] = cout;
                    i += 3;
                } else if (i + 1 < bit_count) {
                    // Half Adder: 2 inputs -> 1 Sum (current col), 1 Carry (next col)
                    int sum, cout;
                    build_half_adder(c, bits[i], bits[i+1], &sum, &cout);
                    new_bits[new_count++] = sum;
                    carries[col+1][carry_count[col+1]++] = cout;
                    i += 2;
                } else {
                    // Pass through single remaining bit
                    new_bits[new_count++] = bits[i++];
                }
            }
            // Update bits for next pass of reduction on THIS column
            bit_count = new_count;
            for (int k = 0; k < bit_count; k++) bits[k] = new_bits[k];
        }
        
        // The last remaining bit is the result for this column
        result[col] = (bit_count > 0) ? bits[0] : circuit_add_gate_alive(c, EVO_XOR, 0, 0); // 0
    }
    
    // 3. Handle the MSB column (accumulated carries)
    int col = 2*n - 1;
    int bits[128], bit_count = carry_count[col];
    for (int i = 0; i < bit_count; i++) bits[i] = carries[col][i];
    
    while (bit_count > 1) {
        int new_bits[128], new_count = 0, i = 0;
        while (i < bit_count) {
            if (i + 1 < bit_count) {
                int sum, cout;
                build_half_adder(c, bits[i], bits[i+1], &sum, &cout);
                new_bits[new_count++] = sum;
                // Note: Carries here would go to bit 2*n, which is overflow
                i += 2;
            } else {
                new_bits[new_count++] = bits[i++];
            }
        }
        bit_count = new_count;
        for (int k = 0; k < bit_count; k++) bits[k] = new_bits[k];
    }
    result[col] = (bit_count > 0) ? bits[0] : 0;
    
    // 4. Map to Outputs (MSB first)
    for (int i = 0; i < 2*n && i < c->num_outputs; i++) {
        // Output index 0 is MSB
        c->output_map[c->num_outputs - 1 - i] = result[i];
    }
    
    // Handle truncation (if n_out < 2*n) or extension
    // Logic above maps LSB result[0] to LSB output map.
}




/* Helper: Adds 3 to a 4-bit nibble if the value is >= 5 */
void build_dabble_module(Circuit *c, int i3, int i2, int i1, int i0, 
                         int *o3, int *o2, int *o1, int *o0) {
    /* 
     * Condition: Val >= 5 (0101)
     * Logical Check: (i3) OR (i2 AND i1) OR (i2 AND i0)
     * Simplified: i3 OR (i2 AND (i1 OR i0))
     */
    
    // 1. Comparison Logic
    int i1_or_i0 = circuit_add_gate_alive(c, EVO_OR, i1, i0);
    int i2_and_lower = circuit_add_gate_alive(c, EVO_AND, i2, i1_or_i0);
    int is_ge_5 = circuit_add_gate_alive(c, EVO_OR, i3, i2_and_lower);
    
    // 2. Adder Logic (Input + 0011)
    // We only need to add 3 if the condition is true.
    // Instead of a full adder + mux, we can shortcut:
    // We compute Sum = Input + 3. Then MUX(is_ge_5, Sum, Input).
    
    // Constant 1 wire (for adding 3, which is 0011)
    // We reuse the condition wire as a '1' when needed, or generate local logic.
    
    // --- Adder LSB (Bit 0) ---
    // 0 + 1 = 1, 1 + 1 = 0. So Sum0 = NOT i0.
    int sum0 = circuit_add_gate_alive(c, EVO_NOT, i0, 0);
    int c0 = i0; // Carry 0 is just i0 (since we added 1)
    
    // --- Adder Bit 1 ---
    // i1 + 1 + c0.
    // Sum1 = i1 XOR 1 XOR c0 = i1 XNOR c0
    int sum1 = circuit_add_gate_alive(c, EVO_XNOR, i1, c0);
    // Carry 1 = (i1 AND 1) OR (c0 AND (i1 XOR 1)). Simplified: i1 OR c0
    int c1 = circuit_add_gate_alive(c, EVO_OR, i1, c0);
    
    // --- Adder Bit 2 ---
    // i2 + 0 + c1
    int sum2 = circuit_add_gate_alive(c, EVO_XOR, i2, c1);
    int c2 = circuit_add_gate_alive(c, EVO_AND, i2, c1);
    
    // --- Adder Bit 3 ---
    // i3 + 0 + c2
    int sum3 = circuit_add_gate_alive(c, EVO_XOR, i3, c2);
    
    // 3. MUX Stage: Select Sum if >=5, else Input
    int not_ge_5 = circuit_add_gate_alive(c, EVO_NOT, is_ge_5, 0);
    
    // Helper macro for 2:1 Mux
    #define MUX(sel, not_s, a, b) \
        circuit_add_gate_alive(c, EVO_OR, \
            circuit_add_gate_alive(c, EVO_AND, sel, a), \
            circuit_add_gate_alive(c, EVO_AND, not_s, b))

    *o0 = MUX(is_ge_5, not_ge_5, sum0, i0);
    *o1 = MUX(is_ge_5, not_ge_5, sum1, i1);
    *o2 = MUX(is_ge_5, not_ge_5, sum2, i2);
    *o3 = MUX(is_ge_5, not_ge_5, sum3, i3);
    
    #undef MUX
}


void build_binary_to_bcd(Circuit *c, int n_bits) {
    printf("    Building Binary to BCD (Double Dabble) for %d bits...\n", n_bits);
    
    // Calculate required decimal digits: ceil(n_bits * log10(2))
    // 8 bits -> 3 digits, 16 bits -> 5 digits
    int num_digits = (n_bits * 10 + 32) / 33; // Approximation
    int bcd_width = num_digits * 4;
    
    // This array holds the wire indices for the current state of the BCD register
    // Index 0 is LSB of the BCD register.
    int *reg_wires = (int*)malloc(sizeof(int) * bcd_width);
    
    // Initialize register to constant 0
    int const_zero = circuit_add_gate_alive(c, EVO_XOR, 0, 0); // 0 ^ 0 = 0
    for(int i = 0; i < bcd_width; i++) {
        reg_wires[i] = const_zero;
    }
    
    // --- DOUBLE DABBLE LOOP ---
    // We iterate for every input bit, from MSB to LSB.
    for (int i = 0; i < n_bits; i++) {
        int input_bit_wire = n_bits - 1 - i; // Input MSB first
        
        // 1. CORRECTION STEP (The "Dabble")
        // Apply Add-3 logic to each 4-bit column IF we have shifted enough data in.
        // We only need to correct if the value could possibly be >= 5.
        // This requires at least 3 bits to have been shifted in previously.
        if (i >= 3 && i < n_bits) { // Don't correct on the very last step (output only)
            for (int d = 0; d < num_digits; d++) {
                int idx = d * 4;
                
                // Pass the 4 wires of this digit through the Add-3 module
                // Inputs: MSB at idx+3, LSB at idx
                int o3, o2, o1, o0;
                build_dabble_module(c, 
                    reg_wires[idx+3], reg_wires[idx+2], reg_wires[idx+1], reg_wires[idx+0],
                    &o3, &o2, &o1, &o0
                );
                
                // Update register wires with corrected values
                reg_wires[idx+0] = o0;
                reg_wires[idx+1] = o1;
                reg_wires[idx+2] = o2;
                reg_wires[idx+3] = o3;
            }
        }
        
        // 2. SHIFT STEP
        // Shift entire register left by 1.
        // MSB is lost (or shifts into next digit, handled implicitly by array indexing)
        // LSB pulls in the new input_bit_wire
        for (int k = bcd_width - 1; k > 0; k--) {
            reg_wires[k] = reg_wires[k-1];
        }
        reg_wires[0] = input_bit_wire;
    }
    
    // --- OUTPUT MAPPING ---
    // Map the final register state to circuit outputs.
    // User convention: Output 0 is MSB.
    // Our reg_wires[0] is LSB.
    
    int total_out_bits = c->num_outputs;
    for (int i = 0; i < total_out_bits; i++) {
        // Calculate which register bit corresponds to Output MSB-first
        // reg index = total_out_bits - 1 - i
        int reg_idx = total_out_bits - 1 - i;
        
        if (reg_idx < bcd_width) {
            c->output_map[i] = reg_wires[reg_idx];
        } else {
            c->output_map[i] = const_zero; // Pad with zero if output width > BCD width
        }
    }
    
    free(reg_wires);
}

/* ============================================================
 * UPDATED try_structural_synthesis WITH ALL NEW DETECTORS
 * ============================================================ */

bool try_structural_synthesis(TT **outputs, Circuit *c) {
    c->num_inputs = num_inputs;
    c->num_outputs = num_outputs;
    c->num_gates = 0;
    c->allowed_ops_count = g_allowed_ops_count;
    memcpy(c->allowed_ops, g_allowed_ops, sizeof(g_allowed_ops));
    c->dead_count = 0;
    
    

  
    /* === TIER -1: ALU (highest priority - very specific pattern) === */
    if (detect_alu(outputs, num_inputs, num_outputs)) {
        int n = (num_inputs - 2) / 2;
        printf("[*] Detected %d-BIT ALU (ADD/SUB/AND/OR)!\n", n);
        build_alu(c, n);
        return true;
    }
    
    // Tier 2 (Arithmetic)
	if (detect_squaring(outputs, num_inputs, num_outputs)) {
	    printf("[*] Detected SQUARING!\n");
	    build_squaring(c, num_inputs);
	    // Note: You need to copy the reduction tree code into build_squaring
	    return true;
	}
	
	// Tier 9 (Mux/Shift)
	if (detect_barrel_shift_right(outputs, num_inputs, num_outputs)) {
	    printf("[*] Detected BARREL SHIFTER RIGHT!\n");
	    build_barrel_shift_right(c, num_outputs, num_inputs - num_outputs);
	    return true;
	}
	
	
	if (detect_demux_1toN(outputs, num_inputs, num_outputs)) {
	    printf("[*] Detected DEMUX 1-to-%d!\n", num_outputs);
	    build_demux_1toN(c, num_outputs);
	    return true;
	}
    /* === TIER 0: Display Decoders (most specific) === */
    if (detect_bcd_to_7seg(outputs, num_inputs, num_outputs)) {
        printf("[*] Detected BCD TO 7-SEGMENT!\n");
        build_bcd_to_7seg(c);
        return true;
    }
    if (detect_hex_to_7seg(outputs, num_inputs, num_outputs)) {
        printf("[*] Detected HEX TO 7-SEGMENT!\n");
        build_hex_to_7seg(c);
        return true;
    }
    if (detect_7seg_active_low(outputs, num_inputs, num_outputs)) {
        printf("[*] Detected 7-SEGMENT ACTIVE-LOW!\n");
        build_7seg_active_low(c);
        return true;
    }
    if (detect_7seg_to_bcd(outputs, num_inputs, num_outputs)) {
        printf("[*] Detected 7-SEGMENT TO BCD!\n");
        /* Would need builder - fall through for now */
    }
    
    /* === TIER 1: Complex Arithmetic === */
    if (detect_multiplier(outputs, num_inputs, num_outputs)) {
        printf("[*] Detected MULTIPLIER (%dx%d)!\n", num_inputs/2, num_inputs/2);
        build_array_multiplier(c, num_inputs / 2);
        return true;
    }
    if (detect_max(outputs, num_inputs, num_outputs)) {
        printf("[*] Detected MAX!\n");
        build_max(c, num_inputs / 2);
        return true;
    }
    if (detect_min(outputs, num_inputs, num_outputs)) {
        printf("[*] Detected MIN!\n");
        build_min(c, num_inputs / 2);
        return true;
    }
    
    /* === TIER 2: Two-Operand Arithmetic === */
    if (detect_adder(outputs, num_inputs, num_outputs)) {
        printf("[*] Detected ADDER!\n");
        build_ripple_carry_adder(c, num_inputs / 2);
        return true;
    }
    if (detect_subtractor(outputs, num_inputs, num_outputs)) {
        printf("[*] Detected SUBTRACTOR!\n");
        build_subtractor(c, num_inputs / 2);
        return true;
    }
    
    /* === TIER 3: Single-Operand Arithmetic === */
    if (detect_incrementer(outputs, num_inputs, num_outputs)) {
        printf("[*] Detected INCREMENTER!\n");
        build_incrementer(c, num_inputs);
        return true;
    }
    if (detect_decrementer(outputs, num_inputs, num_outputs)) {
        printf("[*] Detected DECREMENTER!\n");
        build_decrementer(c, num_inputs);
        return true;
    }
    if (detect_negation(outputs, num_inputs, num_outputs)) {
        printf("[*] Detected NEGATION!\n");
        build_negation(c, num_inputs);
        return true;
    }
    if (detect_abs(outputs, num_inputs, num_outputs)) {
        printf("[*] Detected ABSOLUTE VALUE!\n");
        build_abs(c, num_inputs);
        return true;
    }
    
    /* === TIER 4: Comparators === */
    if (detect_equality(outputs, num_inputs, num_outputs)) {
        printf("[*] Detected EQUALITY!\n");
        build_equality_comparator(c, num_inputs / 2);
        return true;
    }
    if (detect_less_than(outputs, num_inputs, num_outputs)) {
        printf("[*] Detected LESS-THAN!\n");
        build_less_than_comparator(c, num_inputs / 2);
        return true;
    }
    if (detect_greater_than(outputs, num_inputs, num_outputs)) {
        printf("[*] Detected GREATER-THAN!\n");
        build_greater_than_comparator(c, num_inputs / 2);
        return true;
    }
    if (detect_is_zero(outputs, num_inputs, num_outputs)) {
        printf("[*] Detected IS-ZERO!\n");
        build_is_zero(c, num_inputs);
        return true;
    }
    
    /* === TIER 5: Bit Counting === */
    if (detect_popcount(outputs, num_inputs, num_outputs)) {
        printf("[*] Detected POPCOUNT!\n");
        build_popcount(c, num_inputs);
        return true;
    }
    if (detect_clz(outputs, num_inputs, num_outputs)) {
        printf("[*] Detected COUNT LEADING ZEROS!\n");
        build_clz(c, num_inputs);
        return true;
    }
    if (detect_ctz(outputs, num_inputs, num_outputs)) {
        printf("[*] Detected COUNT TRAILING ZEROS!\n");
        build_ctz(c, num_inputs);
        return true;
    }
    if (detect_majority(outputs, num_inputs, num_outputs)) {
        printf("[*] Detected MAJORITY!\n");
        build_majority(c, num_inputs);
        return true;
    }
    
    /* === TIER 6: Code Converters === */
    if (detect_bcd_to_excess3(outputs, num_inputs, num_outputs)) {
        printf("[*] Detected BCD TO EXCESS-3!\n");
        build_bcd_to_excess3(c);
        return true;
    }
    if (detect_excess3_to_bcd(outputs, num_inputs, num_outputs)) {
        printf("[*] Detected EXCESS-3 TO BCD!\n");
        build_excess3_to_bcd(c);
        return true;
    }
    if (detect_binary_to_bcd(outputs, num_inputs, num_outputs)) {
        printf("[*] Detected BINARY TO BCD (Double Dabble)!\n");
        build_binary_to_bcd(c, num_inputs);
        return true;
    }
    if (detect_gray_encode(outputs, num_inputs, num_outputs)) {
        printf("[*] Detected GRAY ENCODER!\n");
        build_gray_encode(c, num_inputs);
        return true;
    }
    if (detect_gray_decode(outputs, num_inputs, num_outputs)) {
        printf("[*] Detected GRAY DECODER!\n");
        build_gray_decode(c, num_inputs);
        return true;
    }
    
    /* === TIER 7: Decoders === */
    if (detect_bcd_to_decimal(outputs, num_inputs, num_outputs)) {
        printf("[*] Detected BCD TO DECIMAL!\n");
        build_bcd_to_decimal(c);
        return true;
    }
    if (detect_2to4_decoder(outputs, num_inputs, num_outputs)) {
        printf("[*] Detected 2-TO-4 DECODER!\n");
        build_2to4_decoder(c);
        return true;
    }
    if (detect_3to8_decoder(outputs, num_inputs, num_outputs)) {
        printf("[*] Detected 3-TO-8 DECODER!\n");
        build_3to8_decoder(c);
        return true;
    }
    if (detect_decoder_with_enable(outputs, num_inputs, num_outputs)) {
        printf("[*] Detected DECODER WITH ENABLE!\n");
        build_decoder_with_enable(c, num_inputs - 1);
        return true;
    }
    if (detect_decoder(outputs, num_inputs, num_outputs)) {
        printf("[*] Detected GENERIC DECODER!\n");
        build_decoder(c, num_inputs);
        return true;
    }
    if (detect_digit_select(outputs, num_inputs, num_outputs)) {
        printf("[*] Detected DIGIT SELECT!\n");
        build_digit_select(c);
        return true;
    }
    
    /* === TIER 8: Encoders === */
    if (detect_4to2_encoder(outputs, num_inputs, num_outputs)) {
        printf("[*] Detected 4-TO-2 ENCODER!\n");
        build_4to2_encoder(c);
        return true;
    }
    if (detect_8to3_encoder(outputs, num_inputs, num_outputs)) {
        printf("[*] Detected 8-TO-3 ENCODER!\n");
        build_8to3_encoder(c);
        return true;
    }
    if (detect_decimal_to_bcd(outputs, num_inputs, num_outputs)) {
        printf("[*] Detected DECIMAL TO BCD!\n");
        build_decimal_to_bcd(c);
        return true;
    }
    if (detect_priority_encoder(outputs, num_inputs, num_outputs)) {
        printf("[*] Detected PRIORITY ENCODER!\n");
        build_priority_encoder(c, num_inputs);
        return true;
    }
    if (detect_hamming_7_4_encode(outputs, num_inputs, num_outputs)) {
        printf("[*] Detected HAMMING(7,4) ENCODER!\n");
        build_hamming_7_4_encode(c);
        return true;
    }
    
    /* === TIER 9: MUX/Control === */
    if (detect_mux2to1(outputs, num_inputs, num_outputs)) {
        printf("[*] Detected 2:1 MUX!\n");
        build_mux2to1(c, num_outputs);
        return true;
    }
    if (detect_barrel_shift_left(outputs, num_inputs, num_outputs)) {
        int shift_bits = 0;
        while ((1 << shift_bits) < num_outputs) shift_bits++;
        printf("[*] Detected BARREL SHIFTER!\n");
        build_barrel_shift_left(c, num_outputs, shift_bits);
        return true;
    }
    
    /* === TIER 10: Simple Transforms === */
    if (detect_left_shift_1(outputs, num_inputs, num_outputs)) {
        printf("[*] Detected LEFT-SHIFT-1!\n");
        build_left_shift_1(c, num_inputs);
        return true;
    }
    if (detect_right_shift_1(outputs, num_inputs, num_outputs)) {
        printf("[*] Detected RIGHT-SHIFT-1!\n");
        build_right_shift_1(c, num_inputs);
        return true;
    }
    if (detect_rotate_left(outputs, num_inputs, num_outputs)) {
        printf("[*] Detected ROTATE-LEFT!\n");
        build_rotate_left(c, num_inputs);
        return true;
    }
    if (detect_rotate_right(outputs, num_inputs, num_outputs)) {
        printf("[*] Detected ROTATE-RIGHT!\n");
        build_rotate_right(c, num_inputs);
        return true;
    }
    if (detect_bit_reverse(outputs, num_inputs, num_outputs)) {
        printf("[*] Detected BIT-REVERSE!\n");
        build_bit_reverse(c, num_inputs);
        return true;
    }
    if (detect_bitwise_not(outputs, num_inputs, num_outputs)) {
        printf("[*] Detected BITWISE-NOT!\n");
        build_bitwise_not(c, num_inputs);
        return true;
    }
    if (detect_sign_extend(outputs, num_inputs, num_outputs)) {
        printf("[*] Detected SIGN-EXTEND!\n");
        build_sign_extend(c, num_inputs);
        return true;
    }
    if (detect_parity(outputs, num_inputs, num_outputs)) {
        printf("[*] Detected PARITY!\n");
        build_parity(c, num_inputs);
        return true;
    }
    printf("[*] Analyzing variable correlations to optimize order...\n");
    
    int *perm = (int*)malloc(sizeof(int) * num_inputs);
    compute_smart_ordering(outputs, num_inputs, num_outputs, perm);
    
    printf("[*] Running Universal Recursive Solver (Smart Reordering)...\n");
    circ_memo_reset();
    
    for (int i = 0; i < num_outputs; i++) {
        /* Permute the truth table according to the smart order */
        /* This physically rearranges columns so 'related' vars are at top */
        TT *permuted_tt = tt_permute_cols(outputs[i], perm, num_inputs);
        
        /* Build logic using the permuted table */
        /* Pass 'perm' so the solver knows which wire ID to use for splits */
        c->output_map[i] = build_recursive_msb(c, permuted_tt, 0, perm);
        
        free_tt(permuted_tt);
        
        if (c->num_gates >= MAX_EVOL_GATES - 100) {
            printf("[!] Universal Solver hit gate limit. Aborting.\n");
            free(perm);
            return false;
        }
    }
    
    free(perm);
    circ_memo_reset();
    printf("    Universal Solver generated %d gates.\n", c->num_gates);
    return true;
}








/* ============================================================
 * MAIN FUNCTION
 * ============================================================ */

int main() {
    seed_rng(time(NULL));
    strncpy(g_chip_name, DEFAULT_CHIP_NAME, sizeof(g_chip_name) - 1);
    
#if ENABLE_DLS2_EXPORT
    load_dls2_pin_mappings();
#endif

    // NOTE: Should be wrapped in strdup(); on literal unless a generator is with heap is used
    char *truth_table = strdup("0000:1111110 0001:0110000 0010:1101101 0011:1111001 "
        "0100:0110011 0101:1011011 0110:1011111 0111:1110000 "
        "1000:1111111 1001:1111011 1010:XXXXXXX 1011:XXXXXXX "
        "1100:XXXXXXX 1101:XXXXXXX 1110:XXXXXXX 1111:XXXXXXX");
    const char *allowed_gates = "ALL";
    
    
    
    
    printf("============================================================\n");
    printf("   HYBRID AIG + CGP OPTIMIZER WITH DLS2 EXPORT             \n");
    printf("============================================================\n");
    printf("   Config: Structural=%d  Evolution=%d  TechMap=%d  DLS2=%d\n",
           ENABLE_STRUCTURAL_DETECTION, ENABLE_EVOLUTIONARY_REFINEMENT,
           ENABLE_TECHNOLOGY_MAPPING, ENABLE_DLS2_EXPORT);
    printf("============================================================\n\n");

    parse_allowed_gates(allowed_gates);
    
    // Heap allocation for large output tables
    TT *outputs[MAX_OUTPUTS]; 
    parse_truth_table(truth_table, outputs);
    
    // Prepare Targets
    int total_rows = 1 << num_inputs;
    g_num_chunks = (total_rows + 63) / 64; 
    
    if (g_num_chunks > MAX_CHUNKS) {
        printf("[CRITICAL] MAX_CHUNKS (%d) exceeded! Needed %d.\n", MAX_CHUNKS, g_num_chunks);
        return 1;
    }

    memset(g_masks, 0, sizeof(g_masks));
    memset(g_targets, 0, sizeof(g_targets));
    
    // Parse Truth Table String to BitVecs
    char *tt_copy = strdup(truth_table);
    if (!tt_copy) { printf("[!] Memory allocation failed\n"); exit(1); }
    
    char *saveptr;
    char *row_str = strtok_r(tt_copy, " \n\r\t,", &saveptr);
    while(row_str) {
        char *colon = strchr(row_str, ':');
        if(colon) {
            *colon = '\0'; 
            char *rhs = colon + 1; 
            int row_idx = 0;
            for(int k = 0; k < num_inputs; k++) 
                row_idx = (row_idx << 1) | (row_str[k] == '1');
            
            if (row_idx < total_rows) {
                int chunk = row_idx / 64;
                int bit = row_idx % 64;
                for(int i = 0; i < num_outputs; i++) {
                    if(rhs[i] != 'X' && rhs[i] != 'x') {
                        g_masks[i].chunks[chunk] |= (1ULL << bit);
                        if(rhs[i] == '1') 
                            g_targets[i].chunks[chunk] |= (1ULL << bit);
                    }
                }
            }
        }
        row_str = strtok_r(NULL, " \n\r\t,", &saveptr);
    }
    free(tt_copy);

    /* --- CRITICAL FIX: ALLOCATE LARGE STRUCTURES ON HEAP --- */
    
    // 1. Allocate Input Vectors
    BitVec *inputs_vec = (BitVec*)calloc(MAX_INPUTS, sizeof(BitVec));
    if (!inputs_vec) { printf("[!] Alloc failed for inputs_vec\n"); return 1; }

    for(int i = 0; i < num_inputs; i++)
        for(int r = 0; r < total_rows; r++)
            if((r >> (num_inputs - 1 - i)) & 1) 
                inputs_vec[i].chunks[r / 64] |= (1ULL << (r % 64));

    // 2. Allocate Circuit
    Circuit *evo_circuit = (Circuit*)malloc(sizeof(Circuit));
    if (!evo_circuit) { printf("[!] Alloc failed for circuit\n"); return 1; }
    memset(evo_circuit, 0, sizeof(Circuit)); 

    int max_score = 0;
    for(int i = 0; i < num_outputs; i++) 
        for(int c = 0; c < g_num_chunks; c++) 
            max_score += __builtin_popcountll(g_masks[i].chunks[c]);

    bool used_structural = false;

#if ENABLE_STRUCTURAL_DETECTION
    /* TRY STRUCTURAL SYNTHESIS FIRST */
    if (try_structural_synthesis(outputs, evo_circuit)) {
        used_structural = true;
        printf("[*] Structural synthesis: %d gates\n", evo_circuit->num_gates);
        
        int struct_score = evo_get_score(evo_circuit, inputs_vec, g_targets, g_masks, g_num_chunks);
        printf("[*] Verification: %d / %d %s\n", struct_score, max_score,
               struct_score == max_score ? "(PERFECT)" : "(ERRORS!)");
        
        if (struct_score != max_score) {
            printf("[!] Structural synthesis has errors, falling back to AIG...\n");
            used_structural = false;
        }
    }
#endif
    
    if (!used_structural) {
        /* Fall back to AIG Synthesis */
        synthesize_with_aig(outputs);
        
        printf("[*] After AIG synthesis: %d gates\n", logic_gate_count - num_inputs);

        int score = verify_circuit_score();
        printf("[*] Verification: %d / %d %s\n", score, max_score, 
               score == max_score ? "(PERFECT)" : "(ERRORS!)");

        load_evolutionary_from_gates(evo_circuit);
        printf("[*] Loaded into CGP: %d gates\n", evo_circuit->num_gates);
    }
    
    for(int i = 0; i < num_outputs; i++) 
        free_tt(outputs[i]);

    /* Verify evolutionary circuit */
    int evo_score = evo_get_score(evo_circuit, inputs_vec, g_targets, g_masks, g_num_chunks);
    printf("[*] Evolutionary verification: %d / %d %s\n", evo_score, max_score,
           evo_score == max_score ? "(PERFECT)" : "(ERRORS!)");

#if ENABLE_TECHNOLOGY_MAPPING
    /* Technology mapping to allowed gates */
    convert_to_allowed_gates_improved(evo_circuit);
    circuit_compact(evo_circuit);
    printf("[*] After gate conversion: %d gates\n", evo_circuit->num_gates);
    
    /* Run optimization passes */
    optimize_circuit(evo_circuit);
#endif

#if ENABLE_EVOLUTIONARY_REFINEMENT
    /* Evolutionary refinement */
    run_evolutionary_refinement(inputs_vec, g_targets, g_masks, g_num_chunks, evo_circuit);
#else
    printf("[*] Evolutionary refinement DISABLED\n");
    printf("[*] Final circuit: %d gates\n", circuit_count_active(evo_circuit));
#endif
    
#if ENABLE_NETLIST_PRINT
    /* Output results */
    render_circuit(evo_circuit);
#endif
    
#if ENABLE_DLS2_EXPORT
    /* Export to DLS2 JSON */
    render_dls2_json(evo_circuit, num_inputs, num_outputs);
#endif

    // Clean up
    free(truth_table);
    free(inputs_vec);
    free(evo_circuit);

    printf("\n[DONE] Synthesis complete.\n");
    return 0;
}
