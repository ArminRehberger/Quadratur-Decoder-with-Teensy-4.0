/*
""" ############################################################################################ """
""" ############################################################################################ """
""" Quadratur decoder QDC with Teensy4.x """
""" V1_00 2024-01-07, Armin Rehberger """
""" Example-file """
""" ############################################################################################ """
""" ############################################################################################ """

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
Teensy Pin 0: Input PhaseA   // Phase A
Teensy Pin 1: Input PhaseB   // Phase B
Teensy Pin 2: Input Index    // Index marker interrupt and count revolution. Configured as negative Edge
Teensy Pin 3: Input Home     // Home marker interrupt. Configured as negative Edge
Teensy Pin 4: Input Trigger  // Trigger input clears the POSD, REV, UPOS and LPOS registers

##### Digital outputs to show the inputs of the encoder on LEDS
Teensy Pin 7: Output PhaseA  // Show input Teensy encoder PhaseA on output Teensy
Teensy Pin 8: Output PhaseB  // Show input Teensy encoder PhaseB on output Teensy
Teensy Pin 9: Output Index   // Show input Teensy encoder Index on output Teensy
Teensy Pin 10: Output Home   // Show input Teensy encoder Home on output Teensy
*/

// ##### Include
#include "QDC.h"

// ##### Digital outputs to show the inputs of the encoder
#define DigitalOutputQDC1_PhaseA 7  // Show input Teensy encoder PhaseA on output Teensy
#define DigitalOutputQDC1_PhaseB 8  // Show input Teensy encoder PhaseB on output Teensy
#define DigitalOutputQDC1_Index 9   // Show input Teensy encoder Index on output Teensy
#define DigitalOutputQDC1_Home 10   // Show input Teensy encoder Home on output Teensy

// ##### Exception callback function QDC1 (ISR Interrupt service routine)
static void ExceptionQDC1Function_isr(void);

// ##### Create and initialize a QDC object
QDC qdc1(ExceptionQDC1Function_isr);

// ###############################################################################################################
// ##### Initialization setup()
// ###############################################################################################################
void setup()
{
  // ##### Serieller Monitor
  Serial.begin(9600);
  Serial.println("Startup");

  // ##### Print CPU speed
  Serial.print("F_CPU_ACTUAL=");
  Serial.println(F_CPU_ACTUAL); // 600.000.000 = 600MHz

  // ##### Define LED on board
  pinMode(LED_BUILTIN, OUTPUT); // LED on board

  // ##### Pins 7..10 as digital output QDC1
  pinMode(DigitalOutputQDC1_PhaseA, OUTPUT);
  pinMode(DigitalOutputQDC1_PhaseB, OUTPUT);
  pinMode(DigitalOutputQDC1_Index, OUTPUT);
  pinMode(DigitalOutputQDC1_Home, OUTPUT);

  // ##### Set position compare value for compare interrupt
  uint32_t CompareValue = 70000;
  qdc1.SetValueCompareInterrupt(CompareValue);

  // ##### Set actual position and revolution
  uint32_t Position = 68000;
  uint16_t Revolution = 0;
  qdc1.SetPositionAndRevolution(Position, Revolution);
} // void setup()

// ###############################################################################################################
// ##### Main program loop()
// ###############################################################################################################
void loop()
{
  // ##### Local variables
  static bool ledon = false;
  static unsigned long delaytimeMillis = 1000;
  static unsigned long previousMillis = 0;
  static unsigned long currentMillis = 0;

  // ##### Delaytime in ms
  if(delaytimeMillis != 0)
  {
    currentMillis = millis();
    previousMillis = currentMillis;
    do
    {
      currentMillis = millis();
    } while (currentMillis - previousMillis < delaytimeMillis);    
  }

  // ##### Toggle LED on board
  ledon = ! ledon;
  digitalWrite(LED_BUILTIN, ledon);

  // ##### Print # or -
  if (ledon == true)
    Serial.println("#####");
  else
    Serial.println("-----");

  // ##### Get ActualPosition and ActualRevolution
  qdc1.QDC1_Position = qdc1.GetPositionAndRevolution();

  Serial.print("ActualPosition ");
  Serial.println(qdc1.QDC1_Position.PositionCounter);
  Serial.print("ActualRevolution ");
  Serial.println(qdc1.QDC1_Position.RevolutionCounter);  

  // ##### Get Direction and status
  qdc1.QDC1_Status = qdc1.GetDirectionAndStatus();

  // Direction
  // 0b - Last count was in the down direction
  // 1b - Last count was in the up direction
  Serial.print("Direction ");
  Serial.println(qdc1.QDC1_Status.DirectionUp);

  // Digital input home
  if(qdc1.QDC1_Status.Home == true) // Home
    digitalWrite(DigitalOutputQDC1_Home, HIGH);
  else
    digitalWrite(DigitalOutputQDC1_Home, LOW);  

  // Digital input Index
  if(qdc1.QDC1_Status.Index == true) // Index
    digitalWrite(DigitalOutputQDC1_Index, HIGH);
  else
    digitalWrite(DigitalOutputQDC1_Index, LOW);

  // Digital input Phase B
  if(qdc1.QDC1_Status.PhaseB == true) // Phase B
    digitalWrite(DigitalOutputQDC1_PhaseB, HIGH);
  else
    digitalWrite(DigitalOutputQDC1_PhaseB, LOW);

  // Digital input Phase A
  if(qdc1.QDC1_Status.PhaseA == true) // Phase A
    digitalWrite(DigitalOutputQDC1_PhaseA, HIGH);
  else
    digitalWrite(DigitalOutputQDC1_PhaseA, LOW);

} // void loop()


// ###############################################################################################################
// ##### Exception callback function QDC1 (ISR Interrupt service routine)
// ###############################################################################################################
// ISR Interrupt service routine

// Compare interrupt          = Bit 1 Control Register (CMPIRQ)
// Watchdog timeout interrupt = Bit 4 Control Register (DIRQ) (not enabled)
// Index marker interrupt     = Bit 8 Control Register (XIRQ)
// Home marker interrupt      = Bit 15 Control Register (HIRQ)

// Roll-under interrupt       = Bit 5 Control2 Register (RUIRQ)
// Roll-over interrupt        = Bit 7 Control2 Register (ROIRQ)

static void ExceptionQDC1Function_isr()
{
  // ########## Compare interrupt = Bit 1 Control Register (CMPIRQ)
  if((ENC1_CTRL & CTRL_CMPIRQ) >= 1)
  {
    ENC1_CTRL |= CTRL_CMPIRQ; // Clear interrupt request, set corresponding bit
    Serial.println("Interrupt QDC1 Compare");

  }

  // ########## Watchdog timeout interrupt = Bit 4 Control Register (DIRQ)
  if((ENC1_CTRL & CTRL_DIRQ) >= 1)
  {
    ENC1_CTRL |= CTRL_DIRQ; // Clear interrupt request, set corresponding bit
    Serial.println("Interrupt QDC1 Watchdog");

  }

  // ########## Index marker interrupt = Bit 8 Control Register (XIRQ)
  if((ENC1_CTRL & CTRL_XIRQ) >= 1)
  {
    ENC1_CTRL |= CTRL_XIRQ; // Clear interrupt request, set corresponding bit
    Serial.println("Interrupt QDC1 IndexMarker");

  }

  // ########## Home marker interrupt = Bit 15 Control Register (HIRQ)
  if((ENC1_CTRL & CTRL_HIRQ) >= 1)
  {
    ENC1_CTRL |= CTRL_HIRQ; // Clear interrupt request, set corresponding bit
    Serial.println("Interrupt QDC1 HomeMarker");

  }

  // ########## Roll-under interrupt = Bit 5 Control2 Register (RUIRQ)
  if((ENC1_CTRL2 & CTRL2_RUIRQ) >= 1)
  {
    ENC1_CTRL2 |= CTRL2_RUIRQ; // Clear interrupt request, set corresponding bit
    Serial.println("Interrupt QDC1 RollUnder");

  }

  // ########## Roll-over interrupt = Bit 7 Control2 Register (ROIRQ)
  if((ENC1_CTRL2 & CTRL2_ROIRQ) >= 1)
  {
    ENC1_CTRL2 |= CTRL2_ROIRQ; // Clear interrupt request, set corresponding bit
    Serial.println("Interrupt QDC1 RollOver");

  }

} // static void ExceptionQDC1Function_isr()




