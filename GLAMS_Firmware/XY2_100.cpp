/*  XY2_100 library
    Copyright (c) 2018 Lutz Lisseck

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.
	Required Connections
	--------------------
    pin 2:  CLOCK+
    pin 14: SYNC+
    pin 7:  CHAN1+
    pin 8:  CHAN2+
    pin 6:  CLOCK-
    pin 20: SYNC-
    pin 21: CHAN1-
    pin 5:  CHAN2-
    A21/DAC0: power reference.
    pin 16: TRIG.
*/
#include "XY2_100.h"

uint16_t XY2_100::lastX;
uint16_t XY2_100::lastY;
void * XY2_100::pingBuffer;
void * XY2_100::pongBuffer;

DMAChannel XY2_100::dma;

static DMAMEM int pingMemory[10];
static DMAMEM int pongMemory[10];

// Bit0: 0=Ping buffer content is beeing transmitted
static volatile uint8_t txPing = 0;

XY2_100::XY2_100() {

  pingBuffer = pingMemory;
  pongBuffer = pongMemory;
  txPing = 0;

}

void XY2_100::begin(void) {

  uint32_t bufsize, frequency;
  bufsize = 40;

  // set up the buffers
  memset(pingBuffer, 0, bufsize);
  memset(pongBuffer, 0, bufsize);

  //Unfortunatly, these two registers are no longer avaliable for teensy 4. Check when the avr_emulation comes online.
  GPIOD_PCOR = 0xFF; // 1 1 1 1   1 1 1 1 Port Clear Output Register
  GPIOD_PDOR = 0x0F; //0 0 0 0   1 1 1 1 Port Data Output Register logic leves driven. 0 Logic level 0 is driven on pin. 1 logic level 1 is difen on pin. Set as Output.
  // configure the 8 output pins

  frequency = 4000000; //4Mhz as clock runs high/low in every clock cycle actually 2 Mhz 

  // good explanation of GPIOD-PDOR: https://forum.pjrc.com/archive/index.php/t-17532.html

  // DMA channel writes the data
  dma.sourceBuffer((uint8_t *)pingBuffer, bufsize);
  dma.destination(GPIOD_PDOR);
  dma.transferSize(1);
  dma.transferCount(bufsize);
  //Disabled these to avoid downtown between pulses, now it allows the jenoptik scanner//SEAA
  //Turned these back on it seems to be more stable running overclocked at 240 MHz
  //  dma.disableOnCompletion(); //SEAA
  //  dma.interruptAtCompletion(); //SEAA


//this will be the place to test with occilator

#if defined(__MK66FX1M0__)
  // TEENSY 3.6
  FTM1_SC = 0; // reset reset and control
  FTM1_CNT = 0; //reset counter
  uint32_t mod = (F_BUS + frequency / 2) / frequency; ///sets frequency
  FTM1_MOD = mod - 1; // 11 @96Mhz, 8 @72MHz
  FTM1_SC = FTM_SC_CLKS(1) | FTM_SC_PS(0); // increment on every TPM clock (system), prescaler 1

  // need ISR also
  FTM1_C0SC = 0x69;  //1101001 MSB:MSA 10, ELSB:ELSA 10, DMA on
  FTM1_C0V = (mod * 128) >> 8;  // 256 = 100% of the time

  // route the timer interrupt to trigger the dma channel
  dma.triggerAtHardwareEvent(DMAMUX_SOURCE_FTM1_CH0);
  // enable a done interrupts when channel completes
  dma.attachInterrupt(isr);

  FTM1_C0SC = 0x28; //101000 see page 1147 disable channel interrupts (chie) and disable dma transfers
  noInterrupts();
  FTM1_SC = 0;             // stop FTM1 timer (hopefully before it rolls over)
  FTM1_CNT = 0;

  //PORTB_ISFR = (1<<18);    // clear any prior rising edge
  uint32_t tmp __attribute__((unused));
  FTM1_C0SC = 0x28; // 101000
  tmp = FTM1_C0SC;         // clear any prior timer DMA triggers
  FTM1_C0SC = 0x69; //1101001
  dma.enable();
  FTM1_SC = FTM_SC_CLKS(1) | FTM_SC_PS(0); // restart FTM1 timer
#elif defined(__IMXRT1062__)
  //Teensy 4.0. Doesn't work before the PORTD/GPIOD register becomes avaliable.

  // TEENSY 4.0, using Example: 53.7.5.2.2 Generate Periodic Interrupt By Counting Internal Clocks.
  // This counts up to 20 and resets the counter afterwards. We probably want to count to 20.

  /* TMR0_CTRL: CM=0,PCS=0,SCS=0,ONCE=0,LENGTH=1,DIR=0,COINIT=0,OUTMODE=0 */
  TMR1_CTRL0 = 0x20; /* Stop all functions of the timer */
  /* TMR0_SCTRL: TCF=0,TCFIE=0,TOF=0,TOFIE=0,IEF=0,IEFIE=0,IPS=0,INPUT=0,
    Capture_Mode=0,MSTR=0,EEOF=0,VAL=0,FORCE=0,OPS=0,OEN=0 */
  TMR1_SCTRL0 = 0x00;
  TMR1_LOAD0 = 0x00; /* Reset load register */

  TMR1_COMP10 =  (F_BUS_ACTUAL + frequency / 2) / frequency; /* Set up compare 1 register */
  TMR1_CMPLD10 = (F_BUS_ACTUAL + frequency / 2) / frequency; /* Also set the compare preload register */

  /* TMR0_CSCTRL: DBG_EN=0,FAULT=0,ALT_LOAD=0,ROC=0,TCI=0,UP=0,OFLAG=0,TCF2EN=0,TCF1EN=1,
    TCF2=0,TCF1=0,CL2=0,CL1=1 */
  TMR1_CSCTRL0 = 0x41; /* Enable compare 1 interrupt and compare 1 preload */
  // route the timer interrupt to trigger the dma channel
  dma.triggerAtHardwareEvent(DMAMUX_SOURCE_QTIMER1_WRITE0_CMPLD1);
  TMR1_DMA0 = 0x02; //Trying to set up DMA
  // enable a done interrupts when channel completes
  dma.attachInterrupt(isr);
  noInterrupts();
  uint32_t tmp __attribute__((unused));
  dma.enable()
  TMR1_SCTRL0 = 0x4001;
  TMR1_CNTR0 = 0x00;
  TMR1_CTRL0 = 0x3E26; /* Primary Count Source to IP_bus_clk / 128,  AND runs counter with the preventing roll over (5. bit high)  */
#endif
  interrupts();
}

void XY2_100::isr(void){
   
  dma.clearInterrupt();
  if (txPing & 2) { //true if tx ping is 11 or 10
    txPing &= ~2; // if 11 then this bis 01 -- if 10 then this becomes 00
    if (txPing & 1) { // if true write to pong buffer else use the ping
      dma.sourceBuffer((uint8_t *)pongBuffer, 40);
    } else {
      dma.sourceBuffer((uint8_t *)pingBuffer, 40);
    }
  }
  //txPing |= 128;

#if defined(__MK66FX1M0__)
  //Teensy 3.6
  //SEAA
  FTM1_SC = 0; //reset
  FTM1_SC = FTM_SC_TOF; //TimerOverflowFlag
  uint32_t tmp __attribute__((unused));
  FTM1_C0SC = 0x28;
  tmp = FTM1_C0SC;         // clear any prior timer DMA triggers
  FTM1_C0SC = 0x69;
  FTM1_CNT = 0; //FTM COUNTER VALUE
  dma.enable();		// enable DMA channel
  FTM1_SC = FTM_SC_CLKS(1) | FTM_SC_PS(0); // restart FTM1 timer (use system clock, prescalar factor 0)
  
#elif defined(__IMXRT1062__)
  //Teensy 4.0
  TMR1_SCTRL0 = 0x00; //Sets status and control to zero.
  TMR1_SCTRL0 = TMR_SCTRL_TOF; //Enables the timer overflow interrupt (it overflows when the counter exceeds 20)
  uint32_t tmp __attribute__((unused));
  tmp = TMR1_DMA0;         // clear any prior timer DMA triggers?
  TMR1_DMA0 = 0x02; //Enables DMA
  dma.enable(); // enable DMA channel
  TMR1_CSCTRL0 = 0x341;    //Trying to manually clear the interrupt flag
  TMR1_SCTRL0 = 0x4001; //Sets status and control to zero.
  TMR1_CNTR0 = 0;
  TMR1_CTRL0 = 0x3E20; /* Primary Count Source to IP_bus_clk / 128,  AND runs counter with the preventing roll over (5. bit high)  */
#endif

}

uint8_t XY2_100::stat(void) {
  uint8_t ret = txPing;
  txPing &= ~128;
  return ret;
}

void XY2_100::setXY(uint16_t X, uint16_t Y){
  
  uint32_t *p;
  // << bit shift
  // | The bitwise OR of two bits is 1 if either or both of the input bits is 1, otherwise it is 0
  // & Bitwise AND operates on each bit position of the surrounding expressions independently,
  // according to this rule: if both input bits are 1, the resulting output is 1, otherwise the output is 0.


  uint32_t Ch1 = (((uint32_t)X << 1) | 0x20000ul) & 0x3fffeul; // 0x20000ul = 100000000000000000
  uint32_t Ch2 = (((uint32_t)Y << 1) | 0x20000ul) & 0x3fffeul; // 0x3fffeul = 111111111111111110

  uint8_t parity1 = 0;
  uint8_t parity2 = 0;

  // 0123456789abcdef
  // fedcba9876543210
  // every 16-bit word generates a clock pulse also
  const uint16_t Sync1[4] = { 0xd2c3, 0x9687, 0x5a4b, 0x1e0f  };
  const uint16_t Sync0[4] = { 0xf0e1, 0xb4a5, 0x7869, 0x3c2d  };

  lastX = X; //sets the current pos as the lastPOS for bookkeeping
  lastY = Y;

  //checks the 20 bit for parity
  for (int i = 0; i < 20; i++) {
    if (Ch1 & (1 << i)) parity1++;
    if (Ch2 & (1 << i)) parity2++;
  }
  //|= compound bitwise or: sets the parity so channel is even
  if (parity1 & 1) Ch1 |= 1;
  if (parity2 & 1) Ch2 |= 1;
  //  Serial.print("Channel1: ");
  //  Serial.println(Ch1,BIN);
  if (txPing & 1) {
    p = ((uint32_t *) pingBuffer);
  } else {
    p = ((uint32_t *) pongBuffer);
  }
  //  Serial.print("P: ");
  //  Serial.println(Ch1, BIN);
  // Clock cycle: 1-0, Sync 111...0
  for (int i = 19; i >= 0; i--) {
    int j = 0;
    uint32_t d;
    if (Ch1 & (1 << i)) j = 1;
    if (Ch2 & (1 << i)) j |= 2;
    d = Sync1[j];

    i--;
    j = 0;
    //  Serial.print("Ch1 (move): ");
    //  Serial.println(Ch1, BIN);
    if (Ch1 & (1 << i)) j = 1;
    if (Ch2 & (1 << i)) j |= 2;
    if (i != 0) d |= (uint32_t)Sync1[j] << 16; else d |= (uint32_t)Sync0[j] << 16;

    *p++ = d;

  }
  noInterrupts();
  txPing ^= 1; //XOR Toogle flip
  txPing |= 2; // OR
  interrupts();
}

void XY2_100::pause() {
#if defined(__MK66FX1M0__)
  FTM1_SC = 0;
  //GPIOD_PDOR = 0; //Should i actually reset output?

#elif defined(__IMXRT1062__)
  //Teensy 4.0
  TMR1_SCTRL0 = 0x00; //Sets status and control to zero.
#endif

}

void XY2_100::unpause() {
#if defined(__MK66FX1M0__)
  //Teensy 3.6
  FTM1_SC = FTM_SC_TOF;
  uint32_t tmp __attribute__((unused));
  FTM1_C0SC = 0x28;        // Enables edge select and channel mode.
  tmp = FTM1_C0SC;         // clear any prior timer DMA triggers
  FTM1_C0SC = 0x69;        // Enable DMA transfers, edge select, channel interrupt and channel mode.
  FTM1_CNT = 0;
  FTM1_SC = FTM_SC_CLKS(1) | FTM_SC_PS(0); // restart FTM1 timer

#elif defined(__IMXRT1062__)
  //Teensy 4.0
  TMR1_SCTRL0 = TMR_SCTRL_TOF; //Enables the timer overflow interrupt (it overflows when the counter exceeds 20)
  TMR1_CSCTRL0 = 0x341;    //Trying to manually clear the interrupt flag
  TMR1_SCTRL0 = 0x4001; //Sets status and control to zero.
  TMR1_CNTR0 = 0;
  TMR1_CTRL0 = 0x3E20; /* Primary Count Source to IP_bus_clk / 128,  AND runs counter with the preventing roll over (5. bit high)  */
#endif
}

void XY2_100::setDataOutputMode(uint16_t outputMode) {
  uint32_t *p;

  uint32_t Ch1 = 0x705ul; // 11100000101 output set mode
  uint32_t Ch2 = 0x705ul; // 11100000101 output set mode

  Ch1 = (Ch1 << 8 | outputMode);
  Ch2 = (Ch2 << 8 | outputMode);

  Ch1 = Ch1 << 1;
  Ch2 = Ch2 << 1;

  uint8_t parity1 = 0;
  uint8_t parity2 = 0;

  // 0123456789abcdef
  // fedcba9876543210
  // every 16-bit word generates a clock pulse also
  const uint16_t Sync1[4] = { 0xd2c3, 0x9687, 0x5a4b, 0x1e0f  };
  const uint16_t Sync0[4] = { 0xf0e1, 0xb4a5, 0x7869, 0x3c2d  };

  //checks the 18 bit
  for (int i = 0; i < 20; i++) {
    if (Ch1 & (1 << i)) parity1++;
    if (Ch2 & (1 << i)) parity2++;
  }
  //|= compound bitwise or: sets the parity so channel is even
  if (parity1 & 1) Ch1 |= 1;
  if (parity2 & 1) Ch2 |= 1;

  //  Serial.print("P (set data mode): ");
  //  Serial.println(Ch1, BIN);

  if (txPing & 1) {
    p = ((uint32_t *) pingBuffer);
  } else {
    p = ((uint32_t *) pongBuffer);
  }
  //  Serial.print("P (set data mode): ");
  //  Serial.println(Ch1, BIN);

  // Clock cycle: 1-0, Sync 111...0
  for (int i = 19; i >= 0; i--) {
    int j = 0;
    uint32_t d;
    if (Ch1 & (1 << i)) j = 1;
    if (Ch2 & (1 << i)) j |= 2;
    d = Sync1[j];
    i--;
    j = 0;
    if (Ch1 & (1 << i)) j = 1;
    if (Ch2 & (1 << i)) j |= 2;
    if (i != 0) d |= (uint32_t)Sync1[j] << 16; else d |= (uint32_t)Sync0[j] << 16;
    *p++ = d;
  }
  noInterrupts();
  txPing ^= 1;
  txPing |= 2;
  interrupts();
}
