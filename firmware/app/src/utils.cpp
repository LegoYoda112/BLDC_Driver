#include "utils.h"

using namespace utils;

int utils::bound_int(int value, int min, int max){
    if(value < min){
        return min;
    }else if(value > max){
        return max;
    }else{
        return value;
    }
}

int8_t utils::bound_int8(int8_t value, int8_t min, int8_t max){
    if(value < min){
        return min;
    }else if(value > max){
        return max;
    }else{
        return value;
    }
}

float utils::fbound(float value, float min, float max){
    if(value < min){
        return min;
    }else if(value > max){
        return max;
    }else{
        return value;
    }
}

float utils::fbound_sym(float value, float max){
    if(value < -max){
        return -max;
    }else if(value > max){
        return max;
    }else{
        return value;
    }
}
