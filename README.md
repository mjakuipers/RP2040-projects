# RP2040-projects
HP41C-RP2040 Interfacing

This is a first working base version of my HP41 RP2040 interface. Purpose is to use this as an example for others to build further code (like display mirroring, detailed bus analyzer, full bankswitching MLDL, HEPAX). This version only acts as a simple HP41 Bus Tracer, that dumps all HP41 cycles to the console. Code at https://github.com/mjakuipers/RP2040-projects

Main features:
- offloads the ARM processors by using the pio for HP41 interfacing
- one ROM page (ML ROM) fixed in Page A
- one (empty) QRAM page fixed in page B with basic MLDL functionality
- decodes and executes the WROM (0x040) instruction in page B
- 3 pio state machines running (sync/isa input, data input and isa output)
- core1 executes the time critical bus interfacing, but very relaxed as the real work is done by the pio
- core0 executes the display (to the USB console)
- there is no user interface
- my hardware setup is the standard PICO board with the PicoProbe
- my development setup is VS Code under Windows

Further implementation is not for the faint of heart, Please understand:
- Only (commented) sources are provided. Please verify the pinout of your setup
- If you change pins, some signals MUST be adjacent (ISA+SYNC, D0_TIME+SYNC_TIME)
- get some basic understanding of the RP2040 pio, interrupts and the development tools for the RP2040
- Hardware connections are identical to Andrews setup, with 3 additional signals used for state machine sync and debug (with a logic analyzer)
- you MUST use the proper level shifters when connection the RP2040 to the HP41 to prevent frying the RP2040 with the HP41 6V signals
- the sources can be used to further develop your application
- in some cases, especially the very first launch, the program may be out of sync with the HP41
- HP41 cycle timing in the sources start at the end of SYNC, and not at data bit D0
- I am not really a C programmer, some code will look clumsy

Obviously, there is no warranty as this is still very much a project in progress.

Things to do for the pio implementation:
- testing of the ISA carry output (it is implemented)
- implementation of data output pio state machine
- implementation of FI output pio state machine
- improve the pio coding, I already know some improvements but my first focus is functionality
With the above I think that any existing HP41 peripheral can be emulated. My ultimate goal is to create full emulation of the HP-IL module.

All feedback is welcome of course.

This project is heavily inspired by the work of blackjetrock (Andrew, who started this thread), thanks Andrew!
