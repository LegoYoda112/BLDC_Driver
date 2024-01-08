#include "trig_luts.h"


// Full sin wave from ()
// Saves some logic and a few addition operations
// with a trade-off in memory
// TODO: Possibly build LUT at runtime on startup?
uint8_t sin_lut_data[] = {
    128, 131, 134, 137, 140, 143, 146, 149, 153, 156, 159, 162, 165,
    168, 171, 174, 177, 180, 182, 185, 188, 191, 194, 196, 199, 201,
    204, 207, 209, 211, 214, 216, 218, 220, 223, 225, 227, 229, 231,
    232, 234, 236, 238, 239, 241, 242, 243, 245, 246, 247, 248, 249,
    250, 251, 252, 253, 253, 254, 254, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 254, 254, 253, 253, 252, 251, 251, 250, 249,
    248, 247, 245, 244, 243, 241, 240, 238, 237, 235, 233, 232, 230,
    228, 226, 224, 222, 219, 217, 215, 213, 210, 208, 205, 203, 200,
    198, 195, 192, 189, 187, 184, 181, 178, 175, 172, 169, 166, 163,
    160, 157, 154, 151, 148, 145, 142, 139, 135, 132, 129, 126, 123,
    120, 116, 113, 110, 107, 104, 101,  98,  95,  92,  89,  86,  83,
    80,  77,  74,  71,  68,  66,  63,  60,  57,  55,  52,  50,  47,
    45,  42,  40,  38,  36,  33,  31,  29,  27,  25,  23,  22,  20,
    18,  17,  15,  14,  12,  11,  10,   8,   7,   6,   5,   4,   4,
    3,   2,   2,   1,   1,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   1,   1,   2,   2,   3,   4,   5,   6,   7,   8,   9,
    10,  12,  13,  14,  16,  17,  19,  21,  23,  24,  26,  28,  30,
    32,  35,  37,  39,  41,  44,  46,  48,  51,  54,  56,  59,  61,
    64,  67,  70,  73,  75,  78,  81,  84,  87,  90,  93,  96,  99,
    102, 106, 109, 112, 115, 118, 121, 124, 127
};

// Returns sin(x) * 0.5 + 0.5 in uint8
// 0 = 0.0 and 256 = 2*pi
uint8_t get_sin_lut_value(uint8_t position){
    return sin_lut_data[position];
}

// Index wraparound version of previous
uint8_t get_sin_lut_value_index_safe(uint8_t position){
    position %= 256;
    return get_sin_lut_value(position);
}

// Returns sin(x) * value/2 + value/2
uint8_t mult_sin_lut_uint8(uint8_t position, uint8_t value){
    // (uint8 * uint8) / 256
    uint16_t mult_value = (uint16_t)value * get_sin_lut_value_index_safe(position);
    return (uint8_t)(mult_value >> 8);
}

int16_t mult_sin_lut_int16(uint8_t position, int16_t value){
    // (uint16 * uint8) / 256
    int32_t mult_value = (value * (get_sin_lut_value_index_safe(position) - 128)) * 2;
    return (int16_t)(mult_value >> 8);
}