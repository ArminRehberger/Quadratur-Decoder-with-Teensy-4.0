/*
""" ############################################################################################ """
""" ############################################################################################ """
""" Quadratur decoder QDC with Teensy4.x """
""" V1_00 2024-01-07, Armin Rehberger """
""" Libary h-file """
""" ############################################################################################ """
""" ############################################################################################ """
*/

#ifndef QDC_h
#define QDC_h

#include <Arduino.h>

// ##### Digital inputs QDC1
#define DigitalInputQDC1_PhaseA 0   // Phase A
#define DigitalInputQDC1_PhaseB 1   // Phase B
#define DigitalInputQDC1_Index 2    // Index marker interrupt and count revolution. Configured as negative Edge
#define DigitalInputQDC1_Home 3     // Home marker interrupt. Configured as negative Edge
#define DigitalInputQDC1_Trigger 4  // Trigger input to clear the POSD, REV, UPOS and LPOS registers

// ##### Control Register (CTRL) bits
// 56.5.1.2 Control Register (CTRL)
// Page 3227 Document: 01_IMXRT1060RM_rev3_ProcessorReverenceManual.pdf
// QDC1 403C_8000h Offset 0h
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

// ##### Control 2 Register (CTRL2) bits
// 56.5.1.17 Control 2 Register (CTRL2)
// Page 3242 Document: 01_IMXRT1060RM_rev3_ProcessorReverenceManual.pdf
// QDC1 403C_8000h Offset 1Eh
#define CTRL2_UPDHLD ((uint16_t)(1<<0)) // Bit 0, Update Hold Registers. When Update Hold Registers is set (=1), it allows the TRIGGER input to cause an update of the POSDH, REVH, UPOSH, and LPOSH registers
#define CTRL2_UPDPOS ((uint16_t)(1<<1)) // Bit 1, Update Position Registers. When Update Position Registers is set (=1), it allows the TRIGGER input to clear the POSD, REV, UPOS and LPOS registers.
#define CTRL2_DIR ((uint16_t)(1<<3)) // Bit 3, Count Direction Flag. The Count Direction Flag indicates the direction of the last count, ReadOnly
#define CTRL2_RUIE ((uint16_t)(1<<4)) // Bit 4, Roll-under Interrupt Enable
#define CTRL2_RUIRQ ((uint16_t)(1<<5)) // Bit 5, Roll-under Interrupt Request
#define CTRL2_ROIE ((uint16_t)(1<<6)) // Bit 6, Roll-over Interrupt Enable
#define CTRL2_ROIRQ ((uint16_t)(1<<7)) // Bit 7, Roll-over Interrupt Request

// ##### Input Monitor Register (IMR) bits
// 56.5.1.15 Input Monitor Register (IMR)
// Page 3240 Document: 01_IMXRT1060RM_rev3_ProcessorReverenceManual.pdf
// QDC1 403C_8000h Offset 1Ah
#define IMR_HOME ((uint16_t)(1<<0)) // Bit 0, Raw HOME input
#define IMR_INDEX ((uint16_t)(1<<1)) // Bit 1, Raw INDEX input
#define IMR_PHB ((uint16_t)(1<<2)) // Bit 2, Raw PHASEB input
#define IMR_PHA ((uint16_t)(1<<3)) // Bit 3, Raw PHASEA input


// ##### Attach function to interrupt
// Use the extern declared _VectorsRam adress array of isr-functions (ISR Interrupt service routine)
// _VectorsRam function array declared in cores/teensy4/startup.c
// C:\Program Files (x86)\Arduino\hardware\teensy\avr\cores\teensy4
// 0xE000ED08 Vector Table Offset
// The interrupt vector table contains addresses (function pointers) of interrupt service/routines and exception handler functions. 

// imxrt.h line 170
// #define NVIC_NUM_INTERRUPTS     160

// startup.c line 24+25
// __attribute__ ((used, aligned(1024), section(".vectorsram")))
extern "C" void (* volatile _VectorsRam[NVIC_NUM_INTERRUPTS+16])(void);

// imxrt.h line 433
// // 07_DDI0403Ee_arm_v7m_ref_manual_M7ArchitectureReferenceManual.pdf page 525 B1.5.2
// Exception numbers 1..15 System
// Exception numbers 16..X External interrupt

// Exception 1 Interrupt -15 Reset
//...
// Exception 15 Interrupt -1 Systick
// Exception 16 Interrupt 0 eDMA Channel 0 Transfer Complete
//...
// Exception 138 Interrupt 122 PIT (PeriodicInterruptTimer)

// Register 0xE000ED08, start adress of the isr-function array _VectorsRam
// 07_DDI0403Ee_arm_v7m_ref_manual_M7ArchitectureReferenceManual.pdf page 596 B3.2.2
// imxrt.h line 9802
// #define SCB_VTOR                (*(volatile uint32_t *)0xE000ED08) // Vector Table Offset

// startup.c line 86
// SCB_VTOR = (uint32_t)_VectorsRam;

static inline void attachExceptionFunctionToInterrupt(IRQ_NUMBER_t irq, void (*function)(void))
{
  _VectorsRam[irq + 16] = function;
  asm volatile("": : :"memory"); // creates a compiler level memory barrier forcing optimizer to not re-order memory accesses across the barrier.
}

// ###############################################################################################################
// ##### Class QDC
// ###############################################################################################################
class QDC
{
  public:
  // ##########  Constructor
  QDC(void (*QDC_ISR_Funct)())
  {
    InitializeQDC(QDC_ISR_Funct);
  }

  // ########## Public member variables
  typedef struct
  {
    uint32_t PositionCounter;
    uint16_t RevolutionCounter;
  } QDC_Position;
  QDC_Position QDC1_Position;

  typedef struct
  {
    bool DirectionUp;
    bool Home;
    bool Index;
    bool PhaseB;
    bool PhaseA;
  } QDC_Status;
  QDC_Status QDC1_Status;

  // ########## Public member methods
  void SetValueCompareInterrupt(uint32_t);
  void SetPositionAndRevolution(uint32_t, uint16_t);
  QDC_Position GetPositionAndRevolution();
  QDC_Status GetDirectionAndStatus();

  private:
  // ########## Private member methods
  void InitializeQDC(void (*QDC_ISR_Funct)());
  void xbar1_connect(unsigned int, unsigned int);

  // ########## Private member variables
  // Struct Teensy pins XBAR
  typedef struct
  {
    uint8_t TeensyPinNoDigitalInput;
    uint8_t XBAR1InputNo;
    uint8_t XBAR1OutputNo;
    volatile uint32_t *RegisterMuxMode;
    uint32_t ValueMuxMode;
    volatile uint32_t *RegisterDaisyChain;
    uint32_t ValueDaisyChain;
  } QDC_XBAR;

  const QDC_XBAR QDC1_XBAR[5] = {
                                  DigitalInputQDC1_PhaseA,  XBARA1_IN_IOMUX_XBAR_INOUT17, XBARA1_OUT_ENC1_PHASEA_INPUT, &IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B0_03, 1, &IOMUXC_XBAR1_IN17_SELECT_INPUT, 1, // Phase A
                                  DigitalInputQDC1_PhaseB,  XBARA1_IN_IOMUX_XBAR_INOUT16, XBARA1_OUT_ENC1_PHASEB_INPUT, &IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B0_02, 1, &IOMUXC_XBAR1_IN16_SELECT_INPUT, 0, // Phase B
                                  DigitalInputQDC1_Index,   XBARA1_IN_IOMUX_XBAR_INOUT06, XBARA1_OUT_ENC1_INDEX,        &IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_04,   3, &IOMUXC_XBAR1_IN06_SELECT_INPUT, 0, // Index
                                  DigitalInputQDC1_Home,    XBARA1_IN_IOMUX_XBAR_INOUT07, XBARA1_OUT_ENC1_HOME,         &IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_05,   3, &IOMUXC_XBAR1_IN07_SELECT_INPUT, 0, // Home
                                  DigitalInputQDC1_Trigger, XBARA1_IN_IOMUX_XBAR_INOUT08, XBARA1_OUT_ENC1_TRIGGER,      &IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_06,   3, &IOMUXC_XBAR1_IN08_SELECT_INPUT, 0  // Trigger
                                };


  // ##### Interrupt number QDC1
  // Chapter 4 Interrupts, DMA Events, and XBAR Assignments
  // Page 43 / 49 Document: 01_IMXRT1060RM_rev3_ProcessorReverenceManual.pdf
  static const uint32_t INTERRUPT_QDC1 = 129;

  // Priority register interrupt QDC1
  // Document: 06_DDI0489F_cortex_m7_trm_rev_1_2_M7ReferenceManual.pdf chapter 7.3 page 7-4
  // 0xE000E400-0xE000E4EC NVIC_IPR0-NVIC_IPR59 Interrupt Priority Register

  // Document: 07_DDI0403Ee_arm_v7m_ref_manual_M7ArchitectureReferenceManual.pdf chapter B3.4.3 page B3-626
  // 0xE000E400-0xE000E5EC NVIC_IPR0-NVIC_IPR123 Interrupt Priority Register

  // https://developer.arm.com/documentation/dui0646/a/Cortex-M7-Peripherals/Nested-Vectored-Interrupt-Controller/Interrupt-Priority-Registers#:~:text=Interrupt%20Priority%20Registers,-The%20NVIC_IPR0%2DNVIC_IPR59&text=Each%20priority%20field%20holds%20a,as%20zero%20and%20ignore%20writes.
  // The NVIC_IPR0-NVIC_IPR59 registers provide a priority field for each interrupt.
  // These registers are byte-accessible. Each register holds four priority fields.
  // Each priority field holds a priority value, 0-255. The lower the value, the greater the priority of the corresponding interrupt.
  uint8_t *REGISTER_INTERRUPT_QDC1_PIORITY = 0xE000E400 + INTERRUPT_QDC1; // Pointer to address 0xE000E400 + Offset interrupt number QDC

  // ##### Enable interrupt QDC1
  // Document: 06_DDI0489F_cortex_m7_trm_rev_1_2_M7ReferenceManual.pdf chapter 7.3 page 7-4
  // 0xE000E100-0xE000E11C NVIC_ISER0-NVIC_ISER7 Interrupt Set-Enable Registers

  // Document: 07_DDI0403Ee_arm_v7m_ref_manual_M7ArchitectureReferenceManual.pdf chapter B3.4.3 page B3-626
  // 0xE000E100-0xE000E13C NVIC_ISER0-NVIC_ISER15 Interrupt Set-Enable Registers

  // 0b0000 NVIC_ISER0 Corresponding interrupts 0-31       Register 0xE000E100
  // 0b0001 NVIC_ISER1 Corresponding interrupts 32-63      Register 0xE000E104
  // 0b0010 NVIC_ISER2 Corresponding interrupts 64-95      Register 0xE000E108
  // 0b0011 NVIC_ISER3 Corresponding interrupts 96-127     Register 0xE000E10C
  // 0b0100 NVIC_ISER4 Corresponding interrupts 128-159    Register 0xE000E110
  // 0b0101 NVIC_ISER5 Corresponding interrupts 160-191    Register 0xE000E114
  // 0b0110 NVIC_ISER6 Corresponding interrupts 192-223    Register 0xE000E115
  // 0b0111 NVIC_ISER7 Corresponding interrupts 224-255    Register 0xE000E11C
  #define aNVIC_ISER0 (*(volatile uint32_t *)0xE000E100)
  uint32_t *REGISTER_ENABLE_INTERRUPT_QDC1 = &aNVIC_ISER0 + (INTERRUPT_QDC1 >> 5); // INTERRUPT_QDC1 129 >> 5 = 4 (0b0100) -> Register 0xE000E110

}; // class QDC

#endif // QDC_h
