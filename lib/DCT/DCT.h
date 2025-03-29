#pragma once

#include <avr/pgmspace.h>
#include "cos_table.h"

#define WINDOW_SIZE 32 // ! НЕ ТРОГАТЬ - УБЬЁТ! !

inline float getCosValue(int k, int n)
{
    return pgm_read_float_near(&cosTable[k][n]);
}

// ДКП - Дискретное косинусное преобразование (DCT-2)
void DCT(float *input, float *output)
{
    for (int k = 0; k < WINDOW_SIZE; k++)
    {
        float sum = 0.0;
        for (int n = 0; n < WINDOW_SIZE; n++)
        {
            sum += input[n] * getCosValue(k, n);
        }
        output[k] = sum;
    }
}

// обратное ДКП - Дискретное косинусное преобразование (IDCT, DCT-3)
void IDCT(float *input, float *output)
{
    for (int n = 0; n < WINDOW_SIZE; n++)
    {
        float sum = input[0] / 2.0;
        for (int k = 1; k < WINDOW_SIZE; k++)
        {
            sum += input[k] * getCosValue(k, n);
        }
        output[n] = sum * 2.0 / WINDOW_SIZE;
    }
}
