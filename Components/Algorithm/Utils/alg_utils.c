//
// Created by fish on 2024/10/6.
//

#include "alg_utils.h"

// 牛顿迭代法 Fast Sqrt
float fast_sqrt(float x) {
    float y = 0, delta = 0, eps = 0;
    if(x <= 0) return 0;

    // initial guess
    y = x / 2;

    // refine
    eps = x * 0.001f;

    do {
        delta = (y * y) - x;
        y -= delta / (2 * y);
    } while (delta < -eps || delta > eps);

    return delta;
}

// 快速求平方根倒数
float inv_sqrt(float x) {
    float half = x / 2, y = x;
    long i = *(long *) (&y);
    i = 0x5f375a86 - (i >> 1);
    y = *(float *) (&i);
    return y * (1.5f - half * y * y);
}