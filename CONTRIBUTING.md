# Guide to contributing to DDS Sweeper

## Overview of structure

The DDS Sweeper can be divided into four components:

1. A serial interface to the host computer running on the first core.
2. An SPI interface to the AD9959 running on the second core.
3. A PIO program for internal triggering (referred to as "timing") the DDS Sweeper.
4. A PIO program for sending triggers to the AD9959.

## Serial interface

The serial interface is mostly defined in the `void loop()` function.

The `fast_serial` library is used to read strings of up to 256 characters into a buffer, which is then parsed.
The first word (space separated) in the buffer is used to determine the command through `strncmp` in a large `if`-`else if`-`else` structure.
Certain commands can not be run while the program is executing.
The behavior of most commands is described in the [README](README.md).

## Program storage

The `set`, `seti`, and `setb` commands are used to load a program into RAM.
Programs are stored in the `instructions` array in RAM.
Programs consist of profile pin information, SPI commands and timing information.
Both are stored in the same array, but timing information is offset to be after all the SPI commands.

The `instructions` array (when all four channels are enabled by `setchannels`) thus contains

```text
[profile pin for address 0] [channel 0 instruction at address 0] [channel 1 instruction at address 0] ... [channel 3 instruction at address 0]
[profile pin for address 1] [channel 0 instruction at address 1] [channel 1 instruction at address 1] ... [channel 3 instruction at address 1]
...
[profile pin for last address] [channel 0 instruction at last address] [channel 1 instruction at last address] ... [channel 3 instruction at last address]
0x00 [0x00 for stop or 0xFF for repeat]
...
[time for instruction at address 0]
[time for instruction at address 1]
...
[time for instruction at last address]
```

The profile pin byte is used in sweep modes to determine which profile pins to trigger (although SPI instructions will be sent for all channels, some of these could be empty, in which case nothing would happen to those channels). In single step mode, it is simply set to a non-zero value to indicate the the program does not stop or repeat yet.

## Program execution

The `void background()` function runs on the second core, sending SPI commands to the AD9959.

A multicore fifo is used to instruct the second core to start (or to wait for an external trigger to start).

It then counts the number of instructions (by scanning the `instructions` array for a `stop` instruction). If external timing is enabled, timing information is loaded from the `instructions` array to the timer PIO program using DMA.

It then enters a loop of sending SPI commands and waiting for triggers (via the `void wait(uint channel)` function). Triggers can be provided by an external trigger or the timer PIO program.

Triggers are also routed to the trigger PIO program, which takes in the profile pin byte and uses that to send triggers to the AD9959 profile pins (which starts sweeps that have already been programmed).

## PIO programs

### Timer program

The timer PIO program reads delays in via the FIFO (which is refilled via DMA) and activates a trigger when delays are complete. It also waits for an external trigger if the delay is zero.

### Trigger program

The trigger PIO program sets the profile and update pins of the AD9959, which allows for more precise control over the DDS timing. It reads in a list of profile pins to update, then updates those and activates the AD9959 update pin when triggered.
