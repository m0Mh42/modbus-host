import math

# NIST ITS-90 Type K coefficients for T -> V
def t_to_v(t):
    if t < 0:
        c = [0.0, 39.450128025, 0.023622373598, -0.32858906784e-3, -0.49904828777e-5,
             -0.67509059173e-7, -0.57410327428e-9, -0.31088872893e-11, -0.10451609365e-13,
             -0.19889266878e-16, -0.16322697486e-19]
        return sum(c[i] * (t**i) for i in range(11)) / 1000.0 # return mV
    elif t <= 1372:
        c = [-0.17600413686e1, 38.921204975, 0.018558770032, -0.99457592874e-4,
             0.31840945719e-6, -0.56072844889e-9, 0.56075059059e-12, -0.32020720003e-15,
             0.97151147152e-19, -0.12104721275e-22]
        a0 = 0.1185976e3
        a1 = -0.1183432e-3
        a2 = 0.1269686e3
        res = sum(c[i] * (t**i) for i in range(10)) + a0 * math.exp(a1 * (t - a2)**2)
        return res / 1000.0 # return mV
    else:
        return 0

# generate table from -200 to 1370 step 10
t_min = -200
t_max = 1370
step = 10

table = []
for t in range(t_min, t_max + step, step):
    table.append(t_to_v(t))

c_code = f"""
/* Type K thermocouple lookup table (mV), ITS-90 standard */
/* Temperatures from {t_min}C to {t_max}C in steps of {step}C */
#define TYPE_K_T_MIN {t_min}
#define TYPE_K_T_MAX {t_max}
#define TYPE_K_STEP  {step}
#define TYPE_K_ENTRIES {len(table)}

static const float typeK_table_mV[TYPE_K_ENTRIES] = {{
"""
for i in range(0, len(table), 8):
    chunk = table[i:i+8]
    c_code += "    " + ", ".join([f"{v:.5f}f" for v in chunk]) + ",\n"
c_code += "};\n"

c_code += """
static float typeK_tempToVoltage_mV(float t_C) {
    if (t_C <= TYPE_K_T_MIN) return typeK_table_mV[0];
    if (t_C >= TYPE_K_T_MAX) return typeK_table_mV[TYPE_K_ENTRIES - 1];
    
    float idx_f = (t_C - TYPE_K_T_MIN) / TYPE_K_STEP;
    int idx = (int)idx_f;
    float frac = idx_f - idx;
    
    float v0 = typeK_table_mV[idx];
    float v1 = typeK_table_mV[idx + 1];
    
    return v0 + frac * (v1 - v0);
}

static float typeK_voltageToTemp_C(float v_mV) {
    if (v_mV <= typeK_table_mV[0]) return TYPE_K_T_MIN;
    if (v_mV >= typeK_table_mV[TYPE_K_ENTRIES - 1]) return TYPE_K_T_MAX;
    
    /* Binary search */
    int low = 0;
    int high = TYPE_K_ENTRIES - 1;
    
    while (low <= high) {
        int mid = low + (high - low) / 2;
        if (typeK_table_mV[mid] == v_mV) return TYPE_K_T_MIN + mid * TYPE_K_STEP;
        if (typeK_table_mV[mid] < v_mV) {
            low = mid + 1;
        } else {
            high = mid - 1;
        }
    }
    
    /* low is now the index of the first element > v_mV */
    int idx = low - 1;
    float v0 = typeK_table_mV[idx];
    float v1 = typeK_table_mV[idx + 1];
    float frac = (v_mV - v0) / (v1 - v0);
    
    return TYPE_K_T_MIN + (idx + frac) * TYPE_K_STEP;
}
"""

with open('lut_code.txt', 'w') as f:
    f.write(c_code)

print("Generated lut_code.txt")
