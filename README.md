# dds-sweeper
Raspberry Pi Pico interface for the AD9959 DDS

## Specs
The frequency resolution of the AD9959 is 
$= \frac{f_{sys clk}}{2^{32}}$. At the default system clock of 500 MHz, the frequency resolution is $\sim 0.1164$ Hz. Any frequency input to the dds-sweeper will be rounded to an integer multiple of the frequency resolution.

Timing uncertainty of $\sim 70$

Always starts a table at 0 amp/freq/whatever I think

If you are not done a sweep when you go to start a new one, that is a problem

## Usage
Note: Commands must be terminated with `\n`.

* `version`: 
Responds with a string containing the firmware version.


* `status`: 


* `getfreqs`: 
Responds with a multi-line string containing the current operating frequencies of various clocks (you will be most interested in `pll_sys` and `clk_sys`). Multiline string ends with `ok\n`.


* `debug <state:str>`: 
Turns on extra debug messages printed over serial. `state` should be `on` or `off` (no string quotes required).


* `setfreq <channel:int> <frequency:float>`:


* `config <type:int> <no dwell:int>`:
0: single tone
1: amp
2: freq
3: phase

make sure the sweep type is set before you start adding structions


* `set <channel:int> <addr:int> <start_point:double> <end_point:double> <rate:double>`:


* ``:


* ``:


* ``:


* ``:


* ``:



