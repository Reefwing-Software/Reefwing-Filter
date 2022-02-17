## Nexgen Filter
 This library contains a collection of filter classes used in the Nexgen Magpie Drone Flight Controller.

 ## Simple Moving Average (SMA)

 A moving average is a calculation used to analyze data points by creating a series of averages of different subsets of the full data set. It is the most common filter used in Digital Signal Processing (DSP), mostly because it is the easiest to understand and calculate. A moving average is a type of finite impulse response or low pass filter in the frequency domain. It is used to smooth out short-term fluctuations and highlight longer-term trends or cycles.

The more samples you use to compute a new average, the lower the cut-off frequency of your low-pass filter, and the smoother your data will be. The objective is to smooth out the noise without losing quick changes that the drone needs to respond to. 

Similar filters to the moving average filter are the Gaussian, Blackman, and multiple pass moving average. These have better performance in the frequency domain, but are computationally more expensive, and less suited to low power MCU’s.

The moving average filter is the optimal choice for reducing random noise while retaining a sharp step response. It works very well with time domain encoded signals. The moving average is the worst filter for frequency domain encoded signals, with little ability to separate one band of frequencies from another. In other words, the moving average is an exceptionally good smoothing filter (the action in the time domain), but an exceptionally bad low-pass filter (the action in the frequency domain).

 This filter implementation returns the average of the N most recent input values. It has been extracted from the [Arduino-Filters](https://github.com/tttapa/Arduino-Filters) library by Pieter Pas.

 To speed up calculation, rather than adding the N input values each time, we keep track of the previous sum in the accumulator and only have to add the new input and delete the oldest one. The new sum is then divided by N. Previous inputs are kept in a circular buffer.

 A class template allows a variety of variable types to be used with the filter. The input parameters are:
 - ```N```: The number of samples to average.
 - ```input_t```: The type of the input (and output) of the filter. The default is ```uint16_t```.
 - ```sum_t```: The type to use for the accumulator, must be large enough to fit N times the maximum input value. The default is ```uint32_t```.

 If you don't set the input/output and accumulator types then the defaults are used. Declaring a new SMA filter can be done in a number of ways. For example:
 ```c++
 static SMA<25> sma;   //  Simple Moving Average filter of length 25, initialized with a value of 0 and default types used.
 static SMA<10> average = {512};   //  Simple Moving Average filter of length 10, initialized with a value of 512 and default types used.
 static SMA<10, uint32_t, uint64_t> sma;   // for very large inputs (larger than 65,535)
 static SMA<10, int16_t, int32_t> sma;     // for inputs that can go negative
 static SMA<10, float, float> sma;         // for decimal (floating point) values
 static SMA<10, uint16_t, uint16_t> sma;   // for small input values and short averages
```
Some important things to note:
- Unsigned integer types are slightly faster than signed integer types
- Integer types are significantly faster than floating point types. Since most sensors and ADCs return an integer, it's often a good idea to filter the raw integer measurement, before converting it to a floating point number (e.g. voltage or temperature).
- The accumulator has to be large enough to fit `N` times the maximum input value. If your maximum input value is 1023 (e.g. from analogRead), and if the accumulator type is `uint16_t`, as in the last example above, the maximum length `N` of the SMA will be ⌊(2¹⁶ - 1) / 1023⌋ = 64. Alternatively, `analogRead` returns a 10-bit number, so the maximum length is 2¹⁶⁻¹⁰ = 2⁶ = 64, which is the same result as before.

 ## Exponential Moving Average (EMA)

An exponential moving average (EMA) is a type of moving average that places a greater weight and significance on the most recent data points. The exponential moving average is also referred to as the exponentially weighted moving average.

The EMA filter applies weighting factors which decrease exponentially for each prior input. Because recent data has a higher weighting than past data, the EMA responds more quickly and tracks recent changes better than a simple moving average.
An exponential moving average filter is often more appropriate than a simple moving average filter. The SMA uses much more memory and is significantly slower than the EMA. The exponential impulse response of the EMA may be better as well.

The EMA filter applies weighting factors which decrease exponentially for each prior input. The formula for the EMA looks like:
```
EMA(n) = EMA(n-1) + 𝛂 × [input(n) – EMA(n-1)]
```
Where:
- 𝛂 = smoothing factor/time constant between 0 and 1.
- n = the period.

You can use a simple moving average (SMA) when calculating the first `EMA(n)`, since there will not be a previous data point `EMA(n-1)`.
The value used for the smoothing factor alpha is often calculated using:
```
α =  2/(N+1)
```
Where `N` is the number of data values in the period. The problem with this approach is that multiplication with floating point numbers is computationally expensive. Note that `α` is between 0 and 1. We can improve things by choosing a value for `α = 1 / 2^k`. In this case, multiplication with a float can be replaced with a division with a number which is a power of 2, and division by a power of two can be replaced by a very fast right bitshift. Our EMA formula then becomes:
```
EMA(n)  = EMA(n-1) + 𝛂 × [input(n) – EMA(n-1)]
        = EMA(n-1) + (1 / 2^k) × [input(n) – EMA(n-1)]
        = EMA(n-1) + [input(n) – EMA(n-1)] >> k
```

This EMA filter implementation and examples have been extracted from the [Arduino-Filters](https://github.com/tttapa/Arduino-Filters) library by Pieter Pas.

A class template allows a variety of variable types to be used with the EMA filter. The input parameters are:
- `K`: The amount of bits to shift by - replaces `𝛂`. This determines the location of the pole in the EMA transfer function, and therefore the cut-off frequency.  The higher the number, the greater the filtering. The pole location is `1 - 2^(-K)`.
- `input_t`: The integer type to use for the input and output of the filter. Can be signed or unsigned.
- `state_t`: The unsigned integer type to use for the internal state of the filter. A fixed-point representation with `K` fractional bits is used, so this type should be at least `M + K` bits wide, where `M` is the maximum number of bits of the input.

Some examples of different combinations of the class template parameters:

1. Filtering the result of `analogRead`: analogRead returns an integer between 0 and 1023, which can be represented using 10 bits, so `M = 10`. If `input_t` and `output_t` are both `uint16_t`, the maximum shift factor `K` is `16 - M = 6`. If `state_t` is increased to `uint32_t`, the maximum shift factor `K` is `32 - M = 22`.
2. Filtering a signed integer between -32768 and 32767: this can be represented using a 16-bit signed integer, so `input_t` is `int16_t`, and `M = 16`. (2¹⁵ = 32768)Let's say the shift factor `K` is 1, then the minimum width of `state_t` should be `M + K = 17` bits, so `uint32_t` would be a sensible choice.

Declaring a new EMA filter can be done in a number of ways. For example:
 ```c++
static EMA<2> ema;  //  K = 2, use default input_t & state_t types (uint16_t)
static EMA<4, uint32_t> ema;    //  K = 4, input_t & state_t type is uint32_t
 ```

## Complementary Filter (CF)

Mahony *et al*, developed the complementary filter which has been shown to be an efficient and effective solution to gyroscopic drift in an IMU. 

A complementary filter is a sensor fusion technique which combines data from a high-pass and low-pass filter. These filters are designed to remove high and low frequency noise. Compared to the Kalman Filter, this is a computationally inexpensive calculation and is suited to low power processors. Don't be scared by the fancy name though, a complementary filter is basically a weighted average of two numbers.

In its simplest form, the complementary filter calculation is:
```
CF = 𝛂 × Input_A + 𝛃 × Input_B
```
Where:
- 𝛂 + 𝛃 = 1 and are the two weighting factors; and
- Input_A & Input_B are two measurements of the same thing from differeny sensors.

An example of how you would use this filter would be deriving roll or pitch angles from an IMU. Gyroscope data is fast to respond but drifts over time. Angle data calculated from the accelerometer is sensitive to vibration but accurate in the long term. The combined results of these two sensors can provide a better result than either alone. We can use a complementary filter to combine them: 
```
Angle 𝜽(t + ∆t) = 0.5 × (𝜽(t) + ω × ∆t) + 0.5 × (tan-1(ay/az))
```
The 𝛂 = 𝛃 = 0.5 in the above formula is the weighting or bias applied to the data. The two weighting factors must add up to one. The gyroscope provides an angular rate (ω), so we integrate it to work out the angle. We can use the two accelerometer vectors (ay and az) to calculate the same angle using trigonometry.

In most examples that you will see, 𝛂 = 0.98 for the gyro angle input and 𝛃 = 0.02 for the accelerometer contribution. The higher the 𝛂, the quicker the response but the higher the drift. A lower 𝛂 will reduce drift but the accelerometer is susceptible to vibration.

## Examples

### 1. Simple Moving Average - Analogue Read

