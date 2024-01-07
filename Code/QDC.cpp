/*
""" ############################################################################################ """
""" ############################################################################################ """
""" Quadratur decoder QDC with Teensy4.x """
""" V1_00 2024-01-07, Armin Rehberger """
""" Libary cpp-file """
""" ############################################################################################ """
""" ############################################################################################ """
*/

#include <Arduino.h>
#include "QDC.h"

// ###############################################################################################################
// ##### Initialize QDC
// ###############################################################################################################
void QDC::InitializeQDC(void (*QDC_ISR_Funct)())
{
  // ##### Local variables
  int i;

  // ##### Pins 0..4 as digital input QDC1
  for(i=0; i<5; i++)
    pinMode(QDC1_XBAR[i].TeensyPinNoDigitalInput, INPUT_PULLDOWN);

  // ##### Enable XBAR1 clock
  // 14.7.23 CCM Clock Gating Register 2 (CCM_CCGR2)
  // Page 1080 Document: 01_IMXRT1060RM_rev3_ProcessorReverenceManual.pdf
  // Register CCM_CCGR2 = 400FC070h
  // #define CCM_CCGR_ON				3
  // xbar1 clock (xbar1_clk_enable) (Bit 22+23)
  CCM_CCGR2 |= CCM_CCGR2_XBAR1(CCM_CCGR_ON); // Set bits 22+23

  // ##### Enable QDC1 clock
  // 14.7.25 CCM Clock Gating Register 4 (CCM_CCGR4)
  // Page 1083 Document: 01_IMXRT1060RM_rev3_ProcessorReverenceManual.pdf
  // Register CCM_CCGR4 = 400FC078h
  // #define CCM_CCGR_ON				3
  // qdc1 clocks (qdc1_clk_enable) (Bit 24+25)
  CCM_CCGR4 |= CCM_CCGR4_ENC1(CCM_CCGR_ON); // Set bits 24+25

  // ##### MuxMode register QDC1 digital inputs 0..4
  /*
  10.1 External Signals and Pin Multiplexing
  Page 289..316 Document: 01_IMXRT1060RM_rev3_ProcessorReverenceManual.pdf

  Instance XBAR1 digital inputs page 315
  Instance:  Port:          Path:
  XBAR1     XBAR_INOUT17    GPIO AD_B0_03 -> XBAR1_INOUT17 connectet to Teensy Pin 0
  XBAR1     XBAR_INOUT16    GPIO AD_B0_02 -> XBAR1_INOUT16 connectet to Teensy Pin 1
  XBAR1     XBAR_INOUT06    GPIO EMC_04   -> XBAR1_INOUT06 connectet to Teensy Pin 2
  XBAR1     XBAR_INOUT07    GPIO EMC_05   -> XBAR1_INOUT07 connectet to Teensy Pin 3
  XBAR1     XBAR_INOUT08    GPIO EMC_06   -> XBAR1_INOUT08 connectet to Teensy Pin 4

  // ##### IOMUXC Memory Map/Register Definition digital inputs 0..4 QDC1
  11.6 IOMUXC Memory Map/Register Definition
  Page 399 Document: 01_IMXRT1060RM_rev3_ProcessorReverenceManual.pdf

  Teensy Pin 0 GPIO AD_B0_03      Page 472
  401F_80C8h -> XBAR1_INOUT17
  001 ALT1 — Select mux mode: ALT1 mux port: XBAR1_INOUT17 of instance: xbar1

  Teensy Pin 1 GPIO AD_B0_02      Page 471
  401F_80C4h  -> XBAR1_INOUT16
  001 ALT1 — Select mux mode: ALT1 mux port: XBAR1_INOUT16 of instance: xbar1

  Teensy Pin 2 GPIO EMC_04
  401F_8024h -> XBAR1_INOUT06     Page 428
  011 ALT3 — Select mux mode: ALT3 mux port: XBAR1_INOUT06 of instance: xbar1

  Teensy Pin 3 GPIO EMC_05        Page 429
  401F_8028h -> XBAR1_INOUT07
  011 ALT3 — Select mux mode: ALT3 mux port: XBAR1_INOUT07 of instance: xbar1

  Teensy Pin 4 GPIO EMC_06        Page 431
  401F_802Ch -> XBAR1_INOUT08
  011 ALT3 — Select mux mode: ALT3 mux port: XBAR1_INOUT08 of instance: xbar1
  */
  for(i=0; i<5; i++)
    *QDC1_XBAR[i].RegisterMuxMode = QDC1_XBAR[i].ValueMuxMode;

  // ##### Daisy Chain register QDC1 digital inputs 0..4
  /*
  11.6 IOMUXC Memory Map/Register Definition
  Page 399 Document: 01_IMXRT1060RM_rev3_ProcessorReverenceManual.pdf

  Teensy Pin 0 GPIO_AD_B0_03      Page 903
  401F_862Ch -> Instance: xbar1, In Pin: xbar_in17
  XBAR1_IN17_SELECT_INPUT DAISY Register
  01 GPIO_AD_B0_03_ALT1 — Selecting Pad: GPIO_AD_B0_03 for Mode: ALT1

  Teensy Pin 1 GPIO_AD_B0_02      Page 911
  401F_864Ch -> Instance: xbar1, In Pin: xbar_in16
  XBAR1_IN16_SELECT_INPUT DAISY Register
  0 GPIO_AD_B0_02_ALT1 — Selecting Pad: GPIO_AD_B0_02 for Mode: ALT1

  Teensy Pin 2 EMC_04             Page 900
  401F_861Ch -> Instance: xbar1, In Pin: xbar_in6
  XBAR1_IN06_SELECT_INPUT DAISY Register
  0 GPIO_EMC_04_ALT3 — Selecting Pad: GPIO_EMC_04 for Mode: ALT3

  Teensy Pin 3 EMC_05             Page 901
  401F_8620h -> Instance: xbar1, In Pin: xbar_in7
  XBAR1_IN07_SELECT_INPUT DAISY Register
  0 GPIO_EMC_05_ALT3 — Selecting Pad: GPIO_EMC_05 for Mode: ALT3

  Teensy Pin 4 EMC_06             Page 902
  401F_8624h -> Instance: xbar1, In Pin: xbar_in8
  XBAR1_IN08_SELECT_INPUT DAISY Register
  0 GPIO_EMC_06_ALT3 — Selecting Pad: GPIO_EMC_06 for Mode: ALT3
  */
  for(i=0; i<5; i++)
    *QDC1_XBAR[i].RegisterDaisyChain = QDC1_XBAR[i].ValueDaisyChain;

  // ##### XBAR1 connect
  /*
  4.6 XBAR Resource Assignments
  Page 61 Document: 01_IMXRT1060RM_rev3_ProcessorReverenceManual.pdf
  XBAR1_IN00 .. XBAR1_IN87
  Page 68 Document: 01_IMXRT1060RM_rev3_ProcessorReverenceManual.pdf
  XBAR1_OUT00 .. XBAR1_OUT131

  61.5 Memory Map and Register Descriptions
  Page 3310 Document: 01_IMXRT1060RM_rev3_ProcessorReverenceManual.pdf
  Register 403B_C000 = Crossbar A Select Register 0 (XBARA1_SEL0)     Bit0..6 XBARA_OUT0    Bit 8..14 XBARA_OUT1
  Register 403B_C002 = Crossbar A Select Register 1 (XBARA1_SEL1)     Bit0..6 XBARA_OUT2    Bit 8..14 XBARA_OUT3
  ...
  Register 403B_C042 = Crossbar A Select Register 33 (XBARA1_SEL33)   Bit0..6 XBARA_OUT66   Bit 8..14 XBARA_OUT67
  ...
  */
  // Route the digital inputs through XBAR1 to inputs QDC1
  // Teensy Pin 0 = Input PhaseA  -> connect IN17 to OUT66
  // Teensy Pin 1 = Input PhaseB  -> connect IN16 to OUT67
  // Teensy Pin 2 = Input Index   -> connect IN06 to OUT68
  // Teensy Pin 3 = Input Home    -> connect IN07 to OUT69
  // Teensy Pin 4 = Input Trigger -> connect IN08 to OUT70
  for(i=0; i<5; i++)
    xbar1_connect(QDC1_XBAR[i].XBAR1InputNo, QDC1_XBAR[i].XBAR1OutputNo);

  // ##### Attach the function to the QDC1-Interrupt
  // Exception callback function interrupt QDC1 (ISR Interrupt service routine)
  attachExceptionFunctionToInterrupt(INTERRUPT_QDC1, QDC_ISR_Funct);

  // ##### Set priority interrupt QDC1
  // Pointer to address 0xE000E400 + Offset interrupt number QDC
  // 0 = highest priority. Value 0..255
  *REGISTER_INTERRUPT_QDC1_PIORITY = (uint8_t)(48);

  // ##### Enable interrupt QDC1
  // Pointer to address 0xE000E110, NVIC_ISER4 Corresponding interrupts 128-159
  // Set bit 1 to enable INTERRUPT_QDC1 (129)
  // INTERRUPT_QDC1 = 129 = 0b1000 0001
  //                   31 = 0b0001 1111
  //                    & = 0b0000 0001 = 1Dec -> Shift 1 times
  // Register 0xE000E110 before: 0000 0000 0000 0000 0000 0000 0000 0000
  // Register 0xE000E110 after:  0000 0000 0000 0000 0000 0000 0000 0010
  *REGISTER_ENABLE_INTERRUPT_QDC1 |= (1 << (INTERRUPT_QDC1 & 31)); // Shift Bit 0 1 times

  // ##### Control Register QDC1 (CTRL) 403C_8000h Offset 0h
  // 56.5.1.2 Control Register (CTRL)
  // Page 3227 Document: 01_IMXRT1060RM_rev3_ProcessorReverenceManual.pdf
  /*
  #define CTRL_CMPIE ((uint16_t)(1<<0)) // Bit 0, Compare Interrupt Enable
  #define CTRL_CMPIRQ ((uint16_t)(1<<1)) // Bit 1, Compare Interrupt Request
  #define CTRL_WDE ((uint16_t)(1<<2)) // Bit 2, Watchdog Enable
  #define CTRL_DIE ((uint16_t)(1<<3)) // Bit 3, Watchdog Timeout Interrupt Enable
  #define CTRL_DIRQ ((uint16_t)(1<<4)) // Bit 4, Watchdog Timeout Interrupt Request
  #define CTRL_XNE ((uint16_t)(1<<5)) // Bit 5, Use Negative Edge of INDEX Pulse. 0=positive edge, 1=negative edge
  #define CTRL_XIE ((uint16_t)(1<<7)) // Bit 7, INDEX Pulse Interrupt Enable
  #define CTRL_XIRQ ((uint16_t)(1<<8)) // Bit 8, INDEX Pulse Interrupt Request
  #define CTRL_SWIP ((uint16_t)(1<<11)) // Bit 11, Software-Triggered Initialization of Position Counters UPOS and LPOS (from UINIT and LINIT)
  #define CTRL_HNE ((uint16_t)(1<<12)) // Bit 12, Use Negative Edge of HOME Input. 0=positive edge, 1=negative edge
  #define CTRL_HIE ((uint16_t)(1<<14)) // Bit 14, HOME Interrupt Enable
  #define CTRL_HIRQ ((uint16_t)(1<<15)) // Bit 15, HOME Signal Transition Interrupt Request
  */
  ENC1_CTRL = 0;
  ENC1_CTRL = CTRL_CMPIE | CTRL_XNE | CTRL_XIE | CTRL_HNE | CTRL_HIE; // Without Watchdog CTRL_WDE CTRL_DIE

  // ##### Control 2 Register QDC1 (CTRL2) 403C_8000h Offset 1Eh
  // 56.5.1.17 Control 2 Register (CTRL2)
  // Page 3242 Document: 01_IMXRT1060RM_rev3_ProcessorReverenceManual.pdf
  /*
  #define CTRL2_UPDHLD ((uint16_t)(1<<0)) // Bit 0, Update Hold Registers. When Update Hold Registers is set (=1), it allows the TRIGGER input to cause an update of the POSDH, REVH, UPOSH, and LPOSH registers
  #define CTRL2_UPDPOS ((uint16_t)(1<<1)) // Bit 1, Update Position Registers. When Update Position Registers is set (=1), it allows the TRIGGER input to clear the POSD, REV, UPOS and LPOS registers.
  #define CTRL2_DIR ((uint16_t)(1<<3)) // Bit 3, Count Direction Flag. The Count Direction Flag indicates the direction of the last count, ReadOnly
  #define CTRL2_RUIE ((uint16_t)(1<<4)) // Bit 4, Roll-under Interrupt Enable
  #define CTRL2_RUIRQ ((uint16_t)(1<<5)) // Bit 5, Roll-under Interrupt Request
  #define CTRL2_ROIE ((uint16_t)(1<<6)) // Bit 6, Roll-over Interrupt Enable
  #define CTRL2_ROIRQ ((uint16_t)(1<<7)) // Bit 7, Roll-over Interrupt Request
  */
  ENC1_CTRL2 = 0;
  ENC1_CTRL2 = CTRL2_UPDPOS | CTRL2_RUIE | CTRL2_ROIE;
} // QDC::InitializeQDC


// ###############################################################################################################
// ##### XBAR1 connect
// ###############################################################################################################
void QDC::xbar1_connect(unsigned int input, unsigned int output)
{
  if (input >= 88) return; // XBAR1 IN00 .. IN87
  if (output >= 132) return; // XBAR1 OUT00 .. OUT131

  volatile uint16_t *xbar = &XBARA1_SEL0 + (output / 2); // Select XBAR register. XBARA1_SEL0 = 403B_C000
  uint16_t val = *xbar;

  if (!(output & 1))  // Output = even
    val |= input;     // Input to bits 0..6
  else                // Output = Odd
    val |= (input << 8); // Input to bits 8..14

  *xbar = val;
}


// ###############################################################################################################
// ##### Set value compare interrupt
// ###############################################################################################################
void QDC::SetValueCompareInterrupt(uint32_t CompareValue)
{
  // ##### Upper/Lower Position Compare Register QDC1 (UCOMP) (LCOMP) 403C_8000h Offset 24h 26h
  // 56.5.1.20 Upper Position Compare Register (UCOMP)
  // Page 3246 Document: 01_IMXRT1060RM_rev3_ProcessorReverenceManual.pdf
  // 56.5.1.21 Lower Position Compare Register (LCOMP)
  // Page 3247 Document: 01_IMXRT1060RM_rev3_ProcessorReverenceManual.pdf
  // Triggers the Compare Interrupt Request -> Compare interrupt = Bit 1 Control Register (CMPIRQ)

  uint16_t UCOMP;
  uint16_t LCOMP;
  UCOMP = (uint16_t)((CompareValue >> 16) & 0x0000FFFF);
  LCOMP = (uint16_t)(CompareValue & 0x0000FFFF);

  // Upper Position Compare Register (UCOMP) (most significant)
  ENC1_UCOMP = UCOMP;

  // Lower Position Compare Register (LCOMP) (least significant)
  ENC1_LCOMP = LCOMP;
}


// ###############################################################################################################
// ##### Set position and revolution
// ###############################################################################################################
void QDC::SetPositionAndRevolution(uint32_t Position, uint16_t Revolution)
{
  // ##### Upper/Lower Initialization Register QDC1 (UINIT) (LINIT) 403C_8000h Offset 16h 18h
  // 56.5.1.13 Upper Initialization Register (UINIT)
  // Page 3238 Document: 01_IMXRT1060RM_rev3_ProcessorReverenceManual.pdf
  // 56.5.1.14 Lower Initialization Register (LINIT)
  // Page 3239 Document: 01_IMXRT1060RM_rev3_ProcessorReverenceManual.pdf

  uint16_t UINIT;
  uint16_t LINIT;
  UINIT = (uint16_t)((Position >> 16) & 0x0000FFFF);
  LINIT = (uint16_t)(Position & 0x0000FFFF);

  // Upper Initialization Register QDC1 (UINIT) (most significant)
  ENC1_UINIT = UINIT;

  // Lower Initialization Register QDC1 (LINIT) (least significant)
  ENC1_LINIT = LINIT;

  // Control Register QDC1 (CTRL) 403C_8000h Offset 0h, set bit 11
  // #define CTRL_SWIP ((uint16_t)(1<<11)) // Bit 11, Software-Triggered Initialization of Position Counters UPOS and LPOS (from UINIT and LINIT)
  ENC1_CTRL |= CTRL_SWIP;

  // ##### Revolution Counter Register QDC1 (REV) 403C_8000h Offset Ah
  // 56.5.1.7 Revolution Counter Register (REV)
  // Page 3234 Document: 01_IMXRT1060RM_rev3_ProcessorReverenceManual.pdf
  uint16_t Rev;
  Rev = (uint16_t)Revolution;
  ENC1_REV = Rev;
}


// ###############################################################################################################
// ##### Get position and revolution
// ###############################################################################################################
QDC::QDC_Position QDC::GetPositionAndRevolution()
{
  // ##### Upper/Lower Position Counter Register QDC1 (UPOS) (LPOS) 403C_8000h Offset Eh 10h
  // 56.5.1.9 Upper Position Counter Register (UPOS)
  // Page 3235 Document: 01_IMXRT1060RM_rev3_ProcessorReverenceManual.pdf
  // 56.5.1.10 Lower Position Counter Register (LPOS)
  // Page 3236 Document: 01_IMXRT1060RM_rev3_ProcessorReverenceManual.pdf

  uint16_t UpperPositionCounterRegister; // most significant
  uint16_t LowerPositionCounterRegister; // least significant
  uint32_t PositionCounter;
  UpperPositionCounterRegister = ENC1_UPOS; // most significant
  LowerPositionCounterRegister = ENC1_LPOS; // least significant
  PositionCounter = UpperPositionCounterRegister << 16;
  PositionCounter |= LowerPositionCounterRegister;
  QDC1_Position.PositionCounter = PositionCounter;

  // ##### Revolution Counter Register QDC1 (REV) 403C_8000h Offset Ah
  // 56.5.1.7 Revolution Counter Register (REV)
  // Page 3234 Document: 01_IMXRT1060RM_rev3_ProcessorReverenceManual.pdf
  QDC1_Position.RevolutionCounter = ENC1_REV;

  return QDC1_Position;
}


// ###############################################################################################################
// ##### Get direction and status
// ###############################################################################################################
QDC::QDC_Status QDC::GetDirectionAndStatus()
{
  // ##### Count Direction Flag QDC1 Control 2 Register (CTRL2) Bit 3. 403C_8000h Offset 1Eh
  // 56.5.1.17 Control 2 Register (CTRL2)
  // Page 3242 Document: 01_IMXRT1060RM_rev3_ProcessorReverenceManual.pdf
  // Bit 3, Count Direction Flag. The Count Direction Flag indicates the direction of the last count, ReadOnly
  // 0b - Last count was in the down direction
  // 1b - Last count was in the up direction
  uint16_t Direction;
  QDC1_Status.DirectionUp = false;
  Direction = ENC1_CTRL2 & CTRL2_DIR;
  if (Direction > 1)
    QDC1_Status.DirectionUp = true;

  // ##### Input Monitor Register QDC1 (IMR) 403C_8000h Offset 1Ah
  // 56.5.1.15 Input Monitor Register (IMR)
  // Page 3240 Document: 01_IMXRT1060RM_rev3_ProcessorReverenceManual.pdf
  // There is no bit for the trigger digital input signal

  // Home ENC1_IMR bit 0
  if((ENC1_IMR & IMR_HOME) >=1) // Home
    QDC1_Status.Home = true;
  else
    QDC1_Status.Home = false;

  // Index ENC1_IMR bit 1
  if((ENC1_IMR & IMR_INDEX) >=1) // Index
    QDC1_Status.Index = true;
  else
    QDC1_Status.Index = false;

  // Phase B ENC1_IMR bit 2
  if((ENC1_IMR & IMR_PHB) >=1) // Phase B
    QDC1_Status.PhaseB = true;
  else
    QDC1_Status.PhaseB = false;

  // Phase A ENC1_IMR bit 3
  if((ENC1_IMR & IMR_PHA) >=1) // Phase A
    QDC1_Status.PhaseA = true;
  else
    QDC1_Status.PhaseA = false;

  return QDC1_Status;
}


