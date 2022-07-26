# dds-sweeper
Raspberry Pi Pico interface for the AD9959 DDS

    

## Notes
The frequency resolution of the AD9959 is 
$= \frac{f_{sys clk}}{2^{32}}$. At the default system clock of 500 MHz, the frequency resolution is $\approx 0.1164$ Hz. Any frequency input to the dds-sweeper will be rounded to an integer multiple of the frequency resolution.

The first sweep is delayed for some reason, but then after that they are quite fast, so the first delay needs to be a tad bit longer.

There seems to be a timing uncertainty of $\approx100-200$ ns between when a trigger signal is received and when the next sweep actually starts for amplitudes.

If multiple instruciton tables are run without shutting off the device, the ending state of the previous table will presist until the first instruction of the next table is triggered.

If a sweep has not concluded before the next sweep is triggered, the behavior is not defined. From obersvation, it seems that this causes problems for downward sweeps, but more thorough testing would be required.

When you start a table, you are locked into the parameters that are not being swept.

For the VCO frequency range of 160 MHz to 255 MHz, there is no guarantee of operation.

The sweeper cannot doing anything while waiting for the initial start

## Serial API
Note: Commands must be terminated with `\n`.

* `reset`:  
Kills the current run and resets the DDS-Sweeper to default settings.


* `abort`:  
Kills the current run. No promises about what the output will be after an `abort`.


* `version`:  
Responds with a string containing the firmware version.


* `status`:  
Returns the opperating status of the DDS-Sweeper. `0` status the instruction table is not currently being run. `1` status means the instruction table is currently being run is watching for inputs


* `getfreqs`:  
Responds with a multi-line string containing the current operating frequencies of various clocks (you will be most interested in `pll_sys` and `clk_sys`). Multiline string ends with `ok\n`.


* `debug <state:str>`:  
Turns on extra debug messages printed over serial. `state` should be `on` or `off` (no string quotes required).


* `setfreq <channel:int> <frequency:float>`:  


* `setphase <channel:int> <phase_offset:float>`:  


* `mode <type:int> <triggers:int>`:  
Configures what mode the DDS-Sweeper is operating in
  - 0: single tone
  - 1: amplitude sweep
  - 2: frequency sweep
  - 3: phase sweep  

  The operating mode must be set before instructions can be programmed into the DDS-Sweeper  
  A trigger value of `0` means the Sweeper is expecting external triggers. A value of `1` means the Sweeper will send its own triggers based on the times passed to the `set` command.


* `set`:  
Sets the value of instruction number `addr` for channel `channel` (zero indexed). `addr` starts at 0. It looks different depending on what mode the sweeper is in.
  - Single Stepping (mode 0): `set <channel:int> <addr:int> <frequency:float> <amplitude:float> <phase:float> (<time:int>)`
  - Sweep Mode (modes 1-3): `set <channel:int> <addr:int> <start_point:float> <end_point:float> <delta:float> <div:int> (<time:int>)`

    `start_point` is the value the sweep should start from, and `end_point` is where it will stop. `delta` is the amount that the output should change by every cycle of the sweep clock. In the AD9959, the sweep clock runs at one quarter the system clock. `div` is an additional divider that can applied to slow down the sweep clock further, must be in the range 1-255. The types of values expected for `start_point`, `end_point`, and `delta` different depending on the type of sweep  
      - Amplitude Sweep (mode 1)  
        `start_point` and `end_point` should be decimals between 0 and 1 that represent the desired proprtion of the maximum output amplitude. `delta` is the desired change in that propertion. For all three of those values there is minimum resolution of $\frac{1}{1024} \approx 0.09766\%$
      - Frequency Sweep (mode 2)  
        `start_point`, `end_point`, and `delta` are frequencies in Hz. They can have decimal values, but no matter not they will be rounded to the nearest multiple of the frequency resolution.
      - Phase Sweep (mode 3)
        `start_point`, `end_point`, and `delta` are in degrees. They can have decimal values, but no matter not they will be rounded to the nearest multiple of the phase resolution (always $= 360^\circ / 2^{14} \approx 0.02197^\circ$). 

  


* `numtriggers`:  
Responds with the nmumber of external triggers processed since the last call of `start`


* `setmult <pll_mult:int>`:    
Sets the pll multiplerier the AD9959 uses to get the from the reference clock to the internal system clock. Valid values are 1 or 4-20.

* `setclock <mode:int> <freq:int>`:  
Reconfigures the source/reference clock.
  - Mode `0`: Use pico system clock as reference to the AD9959
  - Mode `1`: Sets the AD9959 to recieve a reference clock not from the pico

* `setchannels <num:int>`:  
Sets how many channels being used by the table mode. Uses the lowest channels first, starting with channel 0.


* `save`:  
Saves the current table to nonvolatile memory so that it can be recovered later. 


* `load`:  
Retrieves the table currently stored in flash and restores it to RAM so that it can be used again. A saved table must be loaded before it can be used.


