/**
 * HP41C interfacing with the RP2040 microcontroller
 * 
 * Version for testing limited functionality:
 * - capture ISA and DATA line
 * - HP41 bus tracing
 * - ROM emulation
 * - QRAM emulation
 * - capture and execute WROM instruction
 * 
 * Project inspired by PICO41 by Andrew Menahue
 * uses the exact same pinout and connections
 * 
 * This code is open source
 *
 */

#include <ctype.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <inttypes.h>
#include <malloc.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "pico/multicore.h"
#include "hardware/pio.h"
#include "pico/util/queue.h"                    // used for safe FIFO management
#include "hardware/structs/systick.h"

#include "ROMImages.h"                          // put the embedded ROM imgaes in this file

#include "hp41_pio.pio.h"                       // code for pio state machine for the HP41 bus interfacing

// definition of HP41 instructions to be decoded

#define SYNC_bit        0x800                   // position of SYNC status from SYNC/ISA state machine

#define HP41_WROM       0x040
#define inst_WROM       0x840                   // WROM instruction or'ed with SYNC status bit
                                                // and duplicated msb in bit 10
                                                // as captured by the SYNC/ISA state machine


// definition of GPIO pins for the HP41
const uint ONBOARD_LED          = 25;          // onboard LED, standard 

// HP41 input pins
const uint P_CLK1               = 7;            // connect through 4050 level shifters
const uint P_CLK2               = 8;
const uint P_DATA               = 9;
const uint P_ISA                = 10;
const uint P_SYNC               = 11;
const uint P_PWO                = 12;

// helper pins for synchronizing PIO State Machines
// not connected externally except for debugging with a logic state analyzer
const uint P_D0_TIME            = 26;   // phase 00, D0 time, also used for CARRY output on ISA
const uint P_SYNC_TIME          = 27;   // phase 46-55, during SYNC and ISA instruction (always)
const uint P_ADDR_END           = 28;   // spare, used for debugging

// HP41 Output and Output Enable signals, onlys ISA_OUT and ISA_OE are used!
// all OE signals are active low
const uint P_DATA_OUT           = 13;   // not used
const uint P_ISA_OUT            = 14;
const uint P_SYNC_OUT           = 15;   // not used
const uint P_FI_OUT             = 16;   // not used
const uint P_FI_OE              = 17;   // not used  
const uint P_SYNC_OE            = 18;
const uint P_ISA_OE             = 19;   // not used
const uint P_DATA_OE            = 20;   // not used

// For the OLED display, not used in this version
const uint SDA_PIN              = 22;   // not used
const uint SCL_PIN              = 21;   // not used


// definition of the FIFO for the analyzer functions
// this is done is a struct to prevent isa and data going out of sync
// and for ease of adding other info
struct TLine {
    uint32_t    cycle_number;       // to count the cycles since the last PWO
    uint16_t    isa_address;        // ISA address
    uint16_t    isa_instruction;    // ISA instruction with SYNC status
    uint32_t    data1;              // DATA D31..D00
    uint32_t    data2;              // DATA D55..D32
    uint16_t    xq_instr;           // instruction to execute
    uint32_t    xq_data;            // data used for instruction xq
};

uint32_t cycle_counter = 0;         // counts cycles since last PWO
struct TLine TraceLine;             // the variable

queue_t TraceBuffer;                // Trace Buffer type
const int TRACELENGTH = 5000;       // Trace Buffer length

// forward declaration for the PWO interrupt handler
void pwo_callback(uint gpio, uint32_t events);

// function to toggle a GPIO signal
void gpio_toggle(uint signal)
{
    gpio_put(signal, !gpio_get(signal));
}

// function to toggle a GPIO signal multiple times, mainly for debugging
void gpio_pulse(uint signal, uint numtimes)
{
    uint i;
    for (i = 0 ; i < numtimes ; i ++)
    {
       gpio_toggle(signal);
       gpio_toggle(signal);
    }
    
}

// initialize all I/O signals
void bus_init(void)
{
    gpio_init(ONBOARD_LED);
    gpio_set_dir(ONBOARD_LED, GPIO_OUT);
    gpio_put(ONBOARD_LED, 1);               // light the led

    //  HP41 inputs
    gpio_init(P_CLK1);
    gpio_set_dir(P_CLK1, GPIO_IN);
    gpio_init(P_CLK2);
    gpio_set_dir(P_CLK2, GPIO_IN);
    gpio_init(P_SYNC);
    gpio_set_dir(P_SYNC, GPIO_IN);
    gpio_init(P_ISA);
    gpio_set_dir(P_ISA, GPIO_IN);
    gpio_init(P_DATA);
    gpio_set_dir(P_DATA, GPIO_IN);
    gpio_init(P_PWO);
    gpio_set_dir(P_PWO, GPIO_IN);

    // helper outputs
    gpio_init(P_D0_TIME);
    gpio_set_dir(P_D0_TIME, GPIO_OUT);
    gpio_init(P_ADDR_END);
    gpio_set_dir(P_ADDR_END, GPIO_OUT);
    gpio_init(P_SYNC_TIME);
    gpio_set_dir(P_SYNC_TIME, GPIO_OUT);

    gpio_set_dir(P_DATA_OE, GPIO_OUT);      // DATA output not supported in this version
    gpio_put(P_DATA_OE, 1);                 // set to high to disable output driver

    gpio_set_dir(P_FI_OE, GPIO_OUT);        // FI output not supported in this version
    gpio_put(P_FI_OE, 1);                   // set to high to disable output driver

    gpio_set_dir(P_ISA_OE, GPIO_OUT);       // used for ROM emulation
    gpio_put(P_ISA, 1);                     // set to high to disable output driver

    gpio_set_dir(P_SYNC_OE, GPIO_OUT);      // SYNC output not used in this version
    gpio_put(P_SYNC_OE, 1);                 // set to high to disable output driver

    gpio_put(P_D0_TIME, 0);                 // all other outputs to 0
    gpio_put(P_ADDR_END, 0);
    gpio_put(P_SYNC_TIME, 0);

    //prepare the IRQ for the PWO rising and falling edge
    gpio_set_irq_enabled_with_callback(P_PWO, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &pwo_callback);

}

// initialize the trace buffer
// if too much memory is used a panic will result
void buffer_init()
{
    queue_init(&TraceBuffer, sizeof(TraceLine), TRACELENGTH);        // reserve trace buffer memory
}

// initialize the pio state machines

PIO pio0_pio = pio0;        // pio instantiation for pio0, here we put the main SYNC/ISA and DATA input state machine
PIO pio1_pio = pio1;        // pio instantiation for pio1, for most other state machines

uint sync_offset;           // offset in SM instruction memory for the SYNC state machine
uint sync_sm = 0;           // pio sm identifier for the SYNC/ISA state machine

uint datain_offset;         // offset in SM instruction memory for the DATAIN state machine
uint datain_sm = 1;         // identifier for the DATAIN state machine

uint isaout_offset;         // offset in SM instruction memory for the ISAOUT state machine
uint isaout_sm = 0;        // identifier for the ISAOUT state machine

void pio_init()
{    
    // load and init pio programs in the pio instruction memory

    sync_sm = pio_claim_unused_sm(pio0_pio, true);                          // claim a state machine in pio0
    sync_offset   = pio_add_program(pio0_pio, &hp41_pio_sync_program);      

    datain_sm = pio_claim_unused_sm(pio0_pio, true);                        // claim a state machine in pio0
    datain_offset = pio_add_program(pio0_pio, &hp41_pio_datain_program);    // in pio 0 !!

    isaout_sm = pio_claim_unused_sm(pio1_pio, true);                        // claim a state machine in pio1    
    isaout_offset = pio_add_program(pio1_pio, &hp41_pio_isaout_program);    // in pio 1 !!
    
    hp41_pio_sync_program_init(pio0_pio, sync_sm, sync_offset, P_ISA, P_D0_TIME);
    hp41_pio_datain_program_init(pio0_pio, datain_sm, datain_offset, P_DATA, P_D0_TIME);
    hp41_pio_isaout_program_init(pio1_pio, isaout_sm, isaout_offset, P_ISA_OUT, P_ISA_OE, 0, 0);

    printf("Result of state machine inititialization: \n");
    printf("  sync   sm: %2d, offset: %d\n", sync_sm, sync_offset);
    printf("  datain sm: %2d, offset: %d\n", datain_sm, datain_offset);
    printf("  isaout sm: %2d, offset: %d\n", isaout_sm, isaout_offset);
}

// PWO GPIO callback for resetting the SYNC/ISA state machine
// called at rising and falling egde of PWO
// this is needed to synchronize the state machines with the HP41 bus cycle
void pwo_callback(uint gpio, uint32_t events) {
    if (gpio_get(P_PWO) == 0) {
        // PWO is now low, HP41 is sleeping, nothing to do
        // the HP41 generates no CLK's and the sync and datain state machines will be stalled waiting for a CLK01 or CLK02
        // and data pending in the ISR's should be cleared, this is done by enforcing a push
        pio_sm_exec(pio0_pio, datain_sm, pio_encode_push(0, 0) ); 
        pio_sm_exec(pio0_pio, sync_sm, pio_encode_push(0, 0) | pio_encode_sideset(2, 1));             
        gpio_put(ONBOARD_LED, 0);           // turn LED off

    } 
    else {
        // upon rising edge of PWO
        // PWO is now high so HP41 is running
        // reset state machine, next SYNC is imminent

        // enforce a jump to the start of the SYNC and DATA state machines and clear the FIFO's
        pio_sm_clear_fifos(pio0_pio, sync_sm); 
        pio_sm_exec(pio0_pio, sync_sm, pio_encode_jmp(sync_offset + hp41_pio_sync_offset_sync_start)| pio_encode_sideset(2, 1));  
        pio_sm_clear_fifos(pio0_pio, datain_sm); 
        pio_sm_exec(pio0_pio, datain_sm, pio_encode_jmp(datain_offset + hp41_pio_datain_offset_data_start));    
        gpio_put(ONBOARD_LED, 1);           // turn LED on
        cycle_counter = 0;
    }
}

// memory statistics functions
uint32_t getTotalHeap(void) {
   extern char __StackLimit, __bss_end__;
   return &__StackLimit  - &__bss_end__;
}

uint32_t getFreeHeap(void) {
   struct mallinfo m = mallinfo();
   return getTotalHeap() - m.uordblks;
}


// Core1 code
// core1_pio() is the main time-critical loop fetching data from the state machines and pushing data when needed
// this code MUST always run in sync with the HP41 bus
// core1 code must be running before starting the SYNC/ISA State Machine
// main engine to process HP41 bus traffic:
//  -- trace address, instruction, data (triggers, addres range??)
//  -- get address and decide if a response is needed (ROM/QRAM hit)
//  -- track DATA for several parameters
//  -- push instruction if there is an address match needed
//  -- decode instruction and decide if a response is needed
//      -- WROM
//      -- HEPAX
//      -- bankswitching
//      -- peripheral emulation
//      -- display tracking, better to do in the other core with the analyzer?
//      -- carry drive (not yet supported)
//      -- FI drive/undrive (not yet supported)
//      -- data drive   (not yet supported)
void core1_pio()
{
    uint32_t rx_addr = 0;
    uint32_t rx_inst = 0;
    uint32_t rx_inst_t = 0;     // for tracing
    uint32_t rx_isa  = 0;
    uint32_t rx_sync = 0;

    uint16_t wrom_addr = 0;     // for WROM instruction
    uint16_t wrom_data = 0;
    uint16_t wrom_page = 0;

    uint32_t rx_data1 = 0;      // tracking DATA 
    uint32_t rx_data2 = 0;

    uint32_t rom_addr = 0;
    uint32_t rom_pg = 0;

    uint32_t isa_out_data = 0;



    printf("\n core1 starting ...\n\n");

    // main loop, always runs, no exit
    while(1)
    {

        // first read is always the ISA INSTRUCTION, this happens at phase 00,
        // almost immediately after the last instruction bit was read on the rising edge at CLK01 
        // and the extra read for getting the SYNC status
        // from the state machine these are 12 bits, left justified in the 32-bit ISR FIFO from the sync_pio state machine
        //   the two MSB's are the SYNC status and a copy of the instruction MSB
        //   the 10 LSB's are the instruction
        //     bits in instruction:   |11|10|09|08|07|06|05|04|03|02|01|00|
        //                             |  | |      <- instruction ->      |
        //                             |  |--> bit 10: same as bit 09
        //                             |---->  bit 11: SYNC status
        // a WROM instruction (0x040) would show as 0x840, with the SYNC bit set
        // this makes for a quick decision if it is a real instruction (SYNC high) or data (SYNC low)
        // to get right justified INSTRUCTION bits simply shift 20 bits, the SYNC bit is in bit 11

        rx_inst = pio_sm_get_blocking(pio0_pio, sync_sm) >> 20;     // first read is INSTRUCTION

        TraceLine.cycle_number = cycle_counter;         
        cycle_counter++;
        gpio_pulse(P_ADDR_END, 2);                                  // for debugging, connected to logis analyzer

        // for peripheral emulation this is the place where the instruction must be decoded and handled
        // fast enough to be ready to send a carry at D0_TIME or data starting at D0_TIME
        // !! sending a carry is not yet supported !!
        // other instructions may take more time to process
        // for example WROM must wait until relevant DATA is available

        // check for supported instructions
        // if it is one, then mark and handle when the capture of the DATA is ready
        // switch (rx_inst) {
        //     case inst_WROM:                 // WROM instruction
        //         ; // no action here. We do another test for the WROM instruction when [D31..D0] is available
        //         break;
        //     default:
        // }
  
        // put instruction in TraceLine for the ISA instruction
        TraceLine.isa_instruction = rx_inst;

        gpio_pulse(P_ADDR_END, 3); 
     
        // after reading ISA INSTRUCTION next will be the completion of the ADDRESS
        // the first part is autopushed after 32 bits, the 2nd part should be in the ISR at phase 02 (D0)
        // prior to D0 the DATA RX FIFO should be empty (this was read already)
        // to sync, ensure this is indeed empty
        // this must be done BEFORE D0_TIME!

        // at this point we have two options: PWO goes low (power down) or we receive new data 
        // we do a busy wait until one of these happens
        // PWO low typically happens within a few clocks
        // DATA will be [D55..D32] to arrive at D0 CLK01 clock, check arrival of DATA without reading
        // this is our clock phase 2

        while (gpio_get(P_PWO) && (pio_sm_get_rx_fifo_level(pio0_pio, datain_sm) == 0))
        {
                // do nothing, just wait until PWO goes low or data arrives in the datain RX FIFO
        }

        // when we get out of the previous loop we can continue if PWO is still high

        gpio_pulse(P_ADDR_END, 4);                                  // for debugging       

        if (gpio_get(P_PWO))
        // in case PWO is high we have D[55..32] in the datain RX FIFO
        { 

            rx_data2 = pio_sm_get_blocking(pio0_pio, datain_sm);        //  blocking read of D[55..32] from datain state machine

            TraceLine.data2 = rx_data2;                                 // put received data in TraceLine

            // package for the TraceLine is now complete, send it to the TraceBuffer
            // this is non-blocking to prevent going out of SYNC on a full trace bufer
            queue_try_add(&TraceBuffer, &TraceLine);                    // add to internal trace buffer for handling by core0

            gpio_pulse(P_ADDR_END, 5);                                    // for debugging

            // PWO is high, HP41 still running, get ISA ADDRESS
            // ISA ADDRESS is on the bus from clock 16 until clock 31
            // the state machine will capture 29 ISA bits from the end of SYN
            // we get more bits than needed and address bits must be aligned

            rx_addr = pio_sm_get_blocking(pio0_pio, sync_sm);           // 2nd read from SYNC/ISA state machine is ADDRESS
            rom_addr = rx_addr >> 16;                                   // allign address
            TraceLine.isa_address = rom_addr;
            // at this point we have a valid address which is left justified in rx_addr and right justified in rom_addr
            // this can be used to check for a ROM hit, read data and present it on the ISA output

            gpio_pulse(P_ADDR_END, 5);                                  // for debugging

            // check for a page hit, in this example
            // Page A: ML-ROM (in romimages.h)
            // PAge B: empty QRAM page (in romimages.h)
            // this is the place to handle any other ROM mapping and bankswitching 
            rom_pg = rom_addr & 0xF000; 
            if ((rom_pg  == 0xA000) || (rom_pg == 0xB000)) 
            {
                // we now have a ROM hit in Page A or B
                // get the instruction from the embedded ROM images

                rom_addr = rom_addr & 0x0FFF;                   // get address in Page

                if (rom_pg == 0xA000) {
                    isa_out_data = embed_rom0[rom_addr];            // embed_rom0 is the image of the ML-ROM                    
                }

                if (rom_pg == 0xB000) {
                    isa_out_data = embed_rom1[rom_addr];            // embed_rom1 is an empty ROM image                    
                }

                // we must force the state machine to jump to the offset of the isa instruction out
                // normally it is stalled waiting for data right before sending the carry out at D0_TIME
                // this is for faster handling of the carry as we have only 2 clock cycles for that
                // pio_encode_sideset(1, 1), to also use the sideset here (1 sideset bit, value 1), otherwise ISA_OE wil be forced low
                // to be OR'ed with the encoded jump instruction
                pio_sm_exec(pio1_pio, isaout_sm, pio_encode_jmp(isaout_offset + hp41_pio_isaout_offset_isa_inst_out) | pio_encode_sideset(1, 1)); 

                // now ready to put the instruction in the TX FIFO, the rest is done by the PIO state machine
                // the state machine will send out the ISA instruction at the right time, no need to worry about that here
                // just make sure that this is done before the start of SYNC_TIME
                // this is blocking, but the FIFO should  always be empty
                pio_sm_put_blocking(pio1_pio, isaout_sm, isa_out_data); 
            }

            gpio_pulse(P_ADDR_END, 6);                                  // for debugging

            // handling of ISA address now complete
            // next up is capture the first part of DATA, D[00..31], available at clock 34

            rx_data1 = pio_sm_get_blocking(pio0_pio, datain_sm);        // blocking read from datain state machine   
            TraceLine.data1 = rx_data1;

            gpio_pulse(P_ADDR_END, 7);                                  // for debugging           

            // we now have D[00..31] in rx_data1
            // initial use is for the WROM instruction, which can now be executed
            // add any other instructions that need to wait for DATA
            // this is only D[00..31]. If more data bits are needed wait for the rest. Not yet supported!
            switch (rx_inst) {
                case inst_WROM :                                // WROM instruction from ISA instruction capture
                    TraceLine.xq_data = rx_data1;
                    TraceLine.xq_instr = rx_inst;                    
                    wrom_addr = (rx_data1 & 0x0FFFF000) >> 12;  // isolate the WROM address from DATA
                    wrom_data = rx_data1 & 0x03FF;              // isolate WROM data from DATA
                    wrom_page = wrom_addr & 0xF000;
                    if (wrom_page == 0xB000)                    // emulation QRAM in Page B
                    {
                        wrom_addr = wrom_addr & 0x0FFF;
                        embed_rom1[wrom_addr] = wrom_data;      // write to QRAM 
                    }
                    break;

                default:
                    TraceLine.xq_instr = 0;
                    TraceLine.xq_data = 0;
            }

            gpio_pulse(P_ADDR_END, 7);                          // for debugging 


        } 
        else {
            // PWO is low, so HP41 is now in deep or light sleep
            // try to empty the buffer but do not block
            // be aware that the PWO interrupt callback is called upon the falling edge of PWO
            
            if (!pio_sm_is_rx_fifo_empty(pio0_pio, sync_sm))
            {           
                // read data if RX FIFO is not empty
                rx_addr = pio_sm_get_blocking(pio0_pio, sync_sm);
            }
            else
            {
                rx_addr = 0;
            }
            // in any case, update the Trace buffer with whatever we have
            queue_try_add(&TraceBuffer, &TraceLine);                    // add to internal trace buffer for handling by core0   
        }


    }
}

int main() {

    // main program, running on core0
    // example to trace HP41 bus traffic

    struct TLine TraceSample;   // ISA and DATA from the trace buffer

    uint32_t ISAsample;
    uint64_t DATAsample;
    uint32_t DATAsample1;
    uint32_t DATAsample2;
 
    uint32_t addr;
    uint32_t instr;
    uint32_t sync;
    uint32_t cycle;

    uint32_t data0;
    uint32_t data1;

    uint32_t data_x;        // DATA exponent, 2 digits
    uint32_t data_xs;       // DATA exponent sign, 1 digit
    uint32_t data_m1;       // DATA mantissa, 5 digits, D12..D31
    uint32_t data_m2;       // DATA mantissa, 5 digits, D28..D51
    uint32_t data_s;        // DATA sign, D52..D55

    // initialize the RP2040 I/O and the standard I/O (here over USB serial)
    stdio_init_all(); 
    bus_init();   

    gpio_toggle(ONBOARD_LED);
    sleep_ms(2000);
    gpio_toggle(ONBOARD_LED);
    printf("\nstdio initialized \n");

    // initialize all relevant GPIO
    // enable PWO interrupt to reset SYNC state machine

    // initialize trace buffer for ISA and DATA
    buffer_init();
    gpio_toggle(ONBOARD_LED);

    printf("\n*************************");
    printf("\n*  Mein Pico 41 Tracer  ");
    printf("\n*     READY to ROLL     ");
    printf("\n*  Total heap: %d bytes", getTotalHeap());
    printf("\n*   Free heap: %d bytes", getFreeHeap());
    printf("\n*************************\n\n");

    // launching core1 code, must be done before starting the PIO state machines
    multicore_launch_core1(core1_pio);  

    // initialize and start the PIO state machines
    pio_init();   

    while (true) {
        // main loop for HP41 bus tracer
        // never ends

        // blocking read from Trace Buffer queue
        // queue tools are used for proper sync between cores
        queue_remove_blocking(&TraceBuffer, &TraceSample);

        instr = TraceSample.isa_instruction & 0x03FF;
        sync  = TraceSample.isa_instruction >> 11;
        addr  = TraceSample.isa_address;
        cycle = TraceSample.cycle_number;
        
        DATAsample1 = TraceSample.data1;        // D31..D00
        DATAsample2 = TraceSample.data2;        // D55..D32, right justified

        DATAsample2 >> 8;    // to right justify the digits

        data_x  =  DATAsample1 & 0x000000FF;                // DATA exponent, 2 digits
        data_xs = (DATAsample1 & 0x00000F00) >>  8;         // DATA exponent sign, 1 digit
        data_m1 = (DATAsample1 & 0xFFFFF000) >> 12;         // DATA mantissa, 5 digits, D12..D31
        data_m2 =  DATAsample2 & 0x000FFFFF;                // DATA mantissa, 5 digits, D28..D51
        data_s  = (DATAsample2 & 0x00F00000) >> 20;         // DATA sign, D52..D55

        printf("s %5d    %04X  %01X  %03X   -  ", cycle, addr, sync, instr);

        printf(" %01X.%05X%05X.%01X.%02X", data_s, data_m2, data_m1, data_xs, data_x);

        printf(" || %04X %08X \n", TraceSample.xq_instr, TraceSample.xq_data);

    }
}