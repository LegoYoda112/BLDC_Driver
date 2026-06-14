#include "utils.h"

int16_t int16_min3(int16_t a, int16_t b, int16_t c) {
    int16_t min = a;
    if (b < min) {
        min = b;
    }
    if (c < min) {
        min = c;
    }
    return min;
}

int bound(int value, int min, int max){
    if(value < min){
        return min;
    }else if(value > max){
        return max;
    }else{
        return value;
    }
}

bool enforce_bound(int *value, int min, int max){
    if(*value < min){
        *value = min;
        return true;
    }else if(*value > max){
        *value = max;
        return true;
    }else{
        return false;
    }
}