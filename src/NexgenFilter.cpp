/******************************************************************
  @file       NexgenFilter.cpp
  @brief      A collection of filters used in the Magpie Flight Controller.
  @author     David Such
  @copyright  Please see the accompanying LICENSE.txt file.

  Code:        David Such
  Version:     1.0.2
  Date:        24/02/22

  1.0.0 Original Release.           14/02/22
  1.0.1 Fixed Guassian defn.        20/02/22
  1.0.2 Fixed #define               24/02/22

  Credits - SMA and EMA filter code is extracted from the 
            Arduino-Filters Library by Pieter Pas
            (https://github.com/tttapa/Arduino-Filters)
          - Simple Kalman Filter is from the library by 
            Denys Sene (https://github.com/denyssene/SimpleKalmanFilter).

******************************************************************/

#include "Arduino.h"
#include "NexgenFilter.h"
#include <math.h>

/******************************************************************
 * 
 *  Simple Kalman Filter
 * 
 ******************************************************************/

SimpleKalmanFilter::SimpleKalmanFilter(float mea_e, float est_e, float q) {
  _err_measure=mea_e;
  _err_estimate=est_e;
  _q = q;
}

float SimpleKalmanFilter::updateEstimate(float mea) {
  _kalman_gain = _err_estimate/(_err_estimate + _err_measure);
  _current_estimate = _last_estimate + _kalman_gain * (mea - _last_estimate);
  _err_estimate =  (1.0 - _kalman_gain)*_err_estimate + fabs(_last_estimate-_current_estimate)*_q;
  _last_estimate=_current_estimate;

  return _current_estimate;
}

void SimpleKalmanFilter::setMeasurementError(float mea_e) {
  _err_measure=mea_e;
}

void SimpleKalmanFilter::setEstimateError(float est_e) {
  _err_estimate=est_e;
}

void SimpleKalmanFilter::setProcessNoise(float q) {
  _q=q;
}

float SimpleKalmanFilter::getKalmanGain() {
  return _kalman_gain;
}

float SimpleKalmanFilter::getEstimateError() {
  return _err_estimate;
}

/******************************************************************
 * 
 *  Noise Generator
 * 
 ******************************************************************/

const float NoiseGenerator::A[] = { 0.02109238, 0.07113478, 0.68873558 }; // rescaled by (1+P)/(1-P)
const float NoiseGenerator::P[] = { 0.3190,  0.7756,  0.9613  };

bool NoiseGenerator::oneBitLFSR() {
  // Returns 1 bit of noise using a Galois Linear Feedback Shift Register
  // Ref: https://en.wikipedia.org/wiki/Linear_feedback_shift_register#Galois_LFSRs

  static uint16_t lfsr = LFSR_INIT; 
                                
  if (lfsr & 1) { 
    lfsr =  (lfsr >>1) ^ LFSR_MASK; 
    return(true);
  }
  else { 
    lfsr >>= 1;                      
    return(false);
  }
}

double NoiseGenerator::gaussianWithDeviation(int sd /* =1 */) {
  //  Generates an Additive White Gaussian Noise (AWGN) sample.
  //  The default generated sample set will have zero mean and a 
  //  standard deviation of 1. Initialise with a different
  //  standard deviation (sd) if required.

  double distribution, randFloat;
  bool zero = true;

  while (zero) {
    randFloat = (rand() / ((double)RAND_MAX));
    if (randFloat != 0) zero = false;
  }

  distribution = cos((2.0 * (double)PI) * rand() / ((double)RAND_MAX));
  return (sqrt(-2.0 * log(randFloat)) * distribution) * sd;
}

void NoiseGenerator::clearPinkState() {
  srand((unsigned int)millis()); // initialize random generator
  for (size_t i = 0; i < PINK_NOISE_STAGES; i++)
    state[i] = 0.0;
}

float NoiseGenerator::threeStagePink() {
  static const float RMI2 = 2.0 / float(RAND_MAX); // + 1.0; // change for range [0,1)
  static const float offset = A[0] + A[1] + A[2];
  float temp = float(rand());

  state[0] = P[0] * (state[0] - temp) + temp;
  temp = float( rand() );
  state[1] = P[1] * (state[1] - temp) + temp;
  temp = float( rand() );
  state[2] = P[2] * (state[2] - temp) + temp;

  return (A[0]*state[0] + A[1]*state[1] + A[2]*state[2]) * RMI2 - offset;
}
