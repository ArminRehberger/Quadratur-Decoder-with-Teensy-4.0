# Quadratur-Decoder-with-Teensy-4.0
Quadratur decoder with Teensy 4.x ARM Cortex M7 IMXRT1062DVL6B NXP Semiconductors


##### Description Quadrature Decoder (QDC)
Libary to implement a singleturn encoder with the ARM processor from NXP Semiconductors IMXRT1062DVL6B

Using 3 input signals from the singleturn encoder (PHASEA, PHASEB, INDEX)

Using 2 input signals from sensors (TRIGGER, HOME)

Trigger input clears the POSD, REV, UPOS and LPOS registers

Home input triggers an interrupt

The inputs INDEX, TRIGGER and HOME are not necessarily needed


##### Provided Functions in libary QDC.h:
Function to set position compare value for compare interrupt: qdc1.SetValueCompareInterrupt(CompareValue);

Function to set actual position and revolution:               qdc1.SetPositionAndRevolution(Position, Revolution);

Function to get actualPosition and actual revolution:         qdc1.QDC1_Position = qdc1.GetPositionAndRevolution();

Function to get direction and status:                         qdc1.QDC1_Status = qdc1.GetDirectionAndStatus();


##### Interrupt function to place your code in:               static void ExceptionQDC1Function_isr(void);
Following interrupts are provided:

Compare interrupt

Index marker interrupt

Home marker interrupt

Roll-under interrupt

Roll-over interrupt


##### Based on Teensy core files: startup.c / digital.c / imxrt.h


##### Digital inputs QDC1 must be wired as follows:
Teensy Pin 0: Input PhaseA   Phase A

Teensy Pin 1: Input PhaseB   Phase B

Teensy Pin 2: Input Index    Index marker interrupt and count revolution. Configured as negative Edge

Teensy Pin 3: Input Home     Home marker interrupt. Configured as negative Edge

Teensy Pin 4: Input Trigger  Trigger input clears the POSD, REV, UPOS and LPOS registers


##### Digital outputs to show the inputs of the encoder on LEDS
Teensy Pin 7: Output PhaseA  Show input Teensy encoder PhaseA on output Teensy

Teensy Pin 8: Output PhaseB  Show input Teensy encoder PhaseB on output Teensy

Teensy Pin 9: Output Index   Show input Teensy encoder Index on output Teensy

Teensy Pin 10: Output Home   Show input Teensy encoder Home on output Teensy


