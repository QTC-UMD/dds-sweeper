# Sweeps

- Setting up a sweep:  
![Sweep Setup Figure](img/sweep-setup.png)  
Sweeps are defined by two parameters, sweep delta and ramp rate.
- Sweep Delta defines the change in output amplitude/frequency/phase on each sweep step.
- Ramp rate defines how often a sweep step is taken. It is based off of the AD9959's sync clock signal which will be one quarter of the AD9959's system clock. The ramp rate parameter specifies the number of sync clock cycles per sweep step. A ramp rate of 1 will cause the sweep delta to be applied every 1 sync clock cycle. For upward sweeps, the ramp rate parameter can have a value of 1-255. For downward sweeps the ramp rate can only be 1.  

The time between sweep steps can be calculated with:
$ t = \frac{\textrm{Ramp Rate}}{\textrm{Sync Clock}} $
Using the Pico's 125 Mhz with a 4 times PLL Multiplier gives the AD9959 a system clock of 500 MHz and therefore a sync clock of 125 MHz.
For upward sweeps the time between sweeps can range from $\frac{1}{125 MHz} = 8 ns$ to $\frac{255}{125 MHz} = 2.04 \mu s$.
Downward sweeps will apply the sweep delta every $\frac{1}{125 MHz} = 8 ns$.  
Given the frequency resolution of $\frac{f_{sys clk}}{2^{32}}$, the smallest sweep delta is $\frac{1}{2^{32}} = 0.1164$ Hz. With the maximum ramp rates, the DDS-sweeper has a minimum sweeping rate of $\approx 47871$ Hz/sec when sweeping upwards or $\approx 12207031.25$ Hz/sec when sweeping downward.

## Finer Sweep Rate Resolution in `set`

The `set` command will internally calculate the sweep delta and ramp rate by given the user provided `sweep_rate` using the continued fraction algorithm. Given a conversion factor $B$ that allows us to convert a sweep rate $R$ to bits per sync clock cycle,  

$$B_{freq} = \frac{2^{32}}{f_{sysclk}} [\frac{\text{bits}}{\text{Hz}}], B_{amp} = 2^{10} [\frac{\text{bits}}{\text{Arb.}}], B_{phase} = 2^{14}[\frac{\text{bits}}{\text{degree}}],$$

the ratio of sweep delta $d$ and ramp rate $r$ can be equated to the following:  

$$
\frac{d}{r} = B\frac{4R}{f_{sysclk}}
$$.

The continued fraction algorithm then returns the integer ratio approximation of the desired `sweep_rate` float and therefore set an integer sweep delta and ramp rate. The algorithm will respect the limits of each sweep mode, so the frequency tuning word sweep deltas can be between $0$ and $2^{32}-1$, amplitude scale factor sweep deltas can be between $0$ and $2^{10}$, and phase offset word sweep deltas can be between $0$ and $2^{14}$. The ramp rates for each mode are constrained to be between $1$ and $255$ for upward sweeps, and held to $1$ for downward sweeps.

## Downward Sweeps

Downward sweeps are not well supported by the AD9959, but they can still be done.
The best method I have found for doing a downward sweep is to send the instructions via serial first, with the sweep autoclear bit set to active and the rising sweep tuning word set to the maximum.
Then issue the IO_UPDATE signal while keeping the profile pin for that channel high.
I believe this clears the sweep accumulator then quickly refills it with a max rate sweep before beginning the downward sweep.
Other combinations of autoclear bit active or not and timing of the profile pins seem to cause even more issues with downward sweeps.
The biggest downside of this method is that you cannot slow down the downward sweep ramp rate as much as the upward sweep ramp rate.

- Autoclear Accumulator Active, drop the pin after the update:  
  Seems to work, you just cannot slow down downward sweeps.
  ![Autoclear Accumulator Active, drop the pin after the update Closeup](img/profile-pin-drop-after.png)  
  ![Autoclear Accumulator Active, drop the pin after the update Multiple Sweeps](img/noise-on-transition.png)

- Autoclear Accumulator Active, drop the pin before the update:  
  Downward sweeps just dont work at all.  
  ![Autoclear Accumulator Active, drop the pin before the update: Closeup](img/profile-pin-drop.png)  
  ![Autoclear Accumulator Active, drop the pin before the update: Multiple Sweeps](img/no-down-sweeps.png)  

- no autoclear, drop pin before update:  
  You cannot do consecutive down sweeps - every down sweep must be preceeded by an up sweep  
  ![no autoclear, drop pin before update: Closeup](img/no-auto-drop-before.png)  
  ![no autoclear, drop pin before update: Multiple Sweeps](img/consecutive-downs.png)  

- no autoclear, drop pin after update:
  A down sweep after an up sweep cannot cover a greater distance than the upward sweep  
  ![no autoclear, drop pin after update: Closeup](img/no-auto-drop-after.png)  
  ![no autoclear, drop pin after update: Multiple Sweeps](img/incomplete-sweep-after.png)  

- You can also generate a downward sweep by operating the AS9959 above the nyquist frequency.
This method does work but causes discontinuity when switching which band the AD9959 is working in.  
![Scope trace of noise on DDS band transitions](img/aan.png)
