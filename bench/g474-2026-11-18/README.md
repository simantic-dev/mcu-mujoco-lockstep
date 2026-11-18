# The bench build on a real NUCLEO-G474RE and in the simulator, 2026-11-18

One ELF (`BENCH=1 OUT=joint-bench sh firmware/build.sh`, sha256 prefix in `elf.sha256`) flashed to a NUCLEO-G474RE (STM32G474RE silicon
rev X, Cortex-M4 r0p1) with nothing attached, and run in `sim` 0.10.4 (build cd27ee45) with nothing attached (`sim/scenario-bench.yaml`).
The bench build prints its clocks, `T <milestone> <micros()>` and `LOOP <iterations in the last second> <micros()>` on PA2/PA3 (the
ST-LINK virtual COM port). Calibration is skipped because there is no motor or encoder; the encoder SPI reads still happen every loop.

| | NUCLEO-G474RE | sim | sim - board |
| --- | --- | --- | --- |
| SYSCLK, PCLK1 | 170 MHz, 170 MHz | 170 MHz, 170 MHz | 0 |
| `T boot` | 10,091 us | 10,093 us | +2 us |
| `T can_up` | 160,029 us | 160,178 us | +149 us |
| `T spin_start` | 1,160,011 us | 1,160,010 us | -1 us |
| `T spin_end` | 5,061,187 us | 5,061,130 us | -57 us |
| `T ready` | 5,071,856 us | 5,071,819 us | -37 us |
| `LOOP` (control loop iterations per second, steady) | 10,162 | 10,622 | +4.5 % |

Everything paced by the clock and timers agrees to within 0.15 ms. The control loop, which is CPU work plus a 6-byte encoder read on
SPI1 at 1 MHz per iteration, runs 4.5 % faster in the simulator (94.1 us per iteration against 98.4 us). Which part of the loop carries
the difference (instruction timing, flash wait states, or the SPI transfer) is not yet measured.

`silicon.log` is the board's console as read over the virtual COM port; `sim.log` is the simulator's capture of the same UART.

## Where the 4.5 % is: cycle counts per part of the loop

A second bench build wraps each part of `loop()` in `DWT->CYCCNT` reads and prints the averages every second (same ELF on both:
`elf-cycles.sha256`; logs `silicon-cycles.log`, `sim-cycles.log`).

| part of `loop()` | NUCLEO-G474RE (cycles) | sim (cycles) | sim vs board |
| --- | --- | --- | --- |
| one bare MT6835 read (SPI1, 6 bytes at 1 MHz) | 13,052 | 12,959 | -0.7 % |
| FOC maths (`loopFOC` minus its SPI read) | 1,843 | 1,433 | -22 % |
| `move()`: angle and velocity PID | 1,570 | 1,398 | -11 % |
| CAN poll and report (HAL FDCAN) | 115 | 96 | -17 % |
| loop body | 16,581 | 15,887 | -4.2 % |

The SPI model is within 1 %. Plain CPU work running from flash is 11 to 22 % fast. The simulator's G474 core timing reuses the
STM32F405's flash and ART model and is documented as unmeasured on the G474 above 16 MHz; this firmware runs at 170 MHz with four flash
wait states. It is on Simantic's simulator backlog: measure the G474's flash interface at 170 MHz with the existing
cycle-count kernels and give the G4 its own parameters. No speed constant is fitted to this firmware; it is only the check.
