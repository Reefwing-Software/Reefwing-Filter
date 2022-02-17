/******************************************************************
  @file       NexgenFilter.h
  @brief      A collection of filters used in the Magpie Flight Controller.
  @author     David Such
  @copyright  Please see the accompanying LICENSE.txt file.

  Code:        David Such
  Version:     1.0
  Date:        14/02/22

  1.0 Original Release          14/02/22

  Credits - SMA and EMA filter code is extracted from the 
            Arduino-Filters Library by Pieter Pas
            (https://github.com/tttapa/Arduino-Filters)

******************************************************************/

#ifndef NexgenFilter_h
#define NexgenTimer_h

#include "Arduino.h"

template <uint8_t N, class input_t = uint16_t, class sum_t = uint32_t>
class SMA {
  public:
    //  Default constructor (initial state is initialized to all zeros).
    SMA() = default;

    //  Constructor (initial state is initialized to given value).
    SMA(input_t initialValue) : sum(N * (sum_t)initialValue) {
        std::fill(std::begin(previousInputs), std::end(previousInputs),
                  initialValue);
    }

    //  Update the internal state with the new input and return the new output.
    input_t operator()(input_t input) {
        sum -= previousInputs[index];
        sum += input;
        previousInputs[index] = input;
        if (++index == N)
            index = 0;
        return (sum + (N / 2)) / N;
    }
    
    static_assert(
        sum_t(0) < sum_t(-1),  // Check that `sum_t` is an unsigned type
        "Error: sum data type should be an unsigned integer, otherwise, "
        "the rounding operation in the return statement is invalid.");

  private:
    uint8_t index             = 0;
    input_t previousInputs[N] = {};
    sum_t sum                 = 0;
};

template <uint8_t K, class uint_t = uint16_t>
class EMA {
  public:
    /// Update the filter with the given input and return the filtered output.
    uint_t operator()(uint_t input) {
        state += input;
        uint_t output = (state + half) >> K;
        state -= output;
        return output;
    }

    static_assert(
        uint_t(0) < uint_t(-1),  // Check that `uint_t` is an unsigned type
        "The `uint_t` type should be an unsigned integer, otherwise, "
        "the division using bit shifts is invalid.");

    /// Fixed point representation of one half, used for rounding.
    constexpr static uint_t half = 1 << (K - 1);

  private:
    uint_t state = 0;
};

class ComplementaryFilter {
  public:
    ComplementaryFilter() = default;

    ComplementaryFilter(float bias, float A, float B) {
      alpha = bias;
      input_A = A;
      input_B = B;
    }

    static_assert(
        alpha < 1.0,  
        "The weighting factor alpha needs to be less than or equal to 1."
    );

    float result() { return ((alpha * input_A) + (1 - alpha) * input_B); }
    void setValues(float bias, float A, float B) {
      alpha = bias;
      input_A = A;
      input_B = B;
    }

  private:
    float alpha = 0.98;
    float input_A = 0.0;, 
    float input_B = 0.0;
};

#endif