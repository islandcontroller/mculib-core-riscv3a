/*!****************************************************************************
 * @file
 * core_riscv.h
 *
 * @brief
 * RISC-V3A Core Peripheral Access
 *
 * @date  30.04.2020
 * @date  26.02.2023
 ******************************************************************************/
/******************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * SPDX-License-Identifier: Apache-2.0
 ******************************************************************************/

#ifndef __CORE_RISCV_H__
#define __CORE_RISCV_H__

/* ############################### Common ################################### */
/* Core version identifier: V3A                                               */
#define __RISC_V                      0x301U
#define __RISC_V3A                    1U

/* Volatile/Const Access flag definitions                                     */
#define __I                           volatile const
#define __O                           volatile
#define __IO                          volatile

/* Hint attributes for inlining functions                                     */
#define RV_STATIC_INLINE              static inline
#define RV_STATIC_FORCE_INLINE        RV_STATIC_INLINE __attribute__((always_inline))

/* Hint attributes for Hardware Prologue/Epilogue usage                       */
#ifdef USE_WCH_INTERRUPT_FAST_ATTR
#define RV_INTERRUPT __attribute__((interrupt("WCH-Interrupt-fast")))
#elif defined(USE_INTERRUPT_NAKED_ATTR)
#define RV_INTERRUPT __attribute__((naked))
#else
#define RV_INTERRUPT __attribute__((interrupt))
#endif /* USE_WCH_INTERRUPT_FAST_ATTR || USE_INTERRUPT_NAKED_ATTR */

/* Legacy support integer type definitions                                    */
typedef __I uint32_t vuc32;  /* Read Only */
typedef __I uint16_t vuc16;  /* Read Only */
typedef __I uint8_t vuc8;   /* Read Only */

typedef const uint32_t uc32;  /* Read Only */
typedef const uint16_t uc16;  /* Read Only */
typedef const uint8_t uc8;   /* Read Only */

typedef __I int32_t vsc32;  /* Read Only */
typedef __I int16_t vsc16;  /* Read Only */
typedef __I int8_t vsc8;   /* Read Only */

typedef const int32_t sc32;  /* Read Only */
typedef const int16_t sc16;  /* Read Only */
typedef const int8_t sc8;   /* Read Only */

typedef __IO uint32_t  vu32;
typedef __IO uint16_t vu16;
typedef __IO uint8_t  vu8;

typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;

typedef __IO int32_t  vs32;
typedef __IO int16_t  vs16;
typedef __IO int8_t   vs8;

typedef int32_t  s32;
typedef int16_t s16;
typedef int8_t  s8;

/* Error, function and status flag definitions                                */
typedef enum {ERROR = 0, SUCCESS = !ERROR} ErrorStatus;
typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;
typedef enum {RESET = 0, SET = !RESET} FlagStatus, ITStatus;


/* Core Peripheral Base Address definitions                                   */
#define PFIC_BASE                     0xE000E000UL
#define SysTick_BASE                  0xE000F000UL


/* ################ Programmable Fast Interrupt Controller ################## */
/* Memory-Mapped PFIC Register File                                           */
typedef struct {
  __I  uint32_t ISR[8];
  __I  uint32_t IPR[8];
  __IO uint32_t ITHRESDR;
  __IO uint32_t FIBADDRR;
  __IO uint32_t CFGR;
  __I  uint32_t GISR;
       uint8_t  RESERVED0[0x10];
  __IO uint32_t FIOFADDRR[4];
       uint8_t  RESERVED1[0x90];
  __O  uint32_t IENR[8];
       uint8_t  RESERVED2[0x60];
  __O  uint32_t IRER[8];
       uint8_t  RESERVED3[0x60];
  __O  uint32_t IPSR[8];
       uint8_t  RESERVED4[0x60];
  __O  uint32_t IPRR[8];
       uint8_t  RESERVED5[0x60];
  __IO uint32_t IACTR[8];
       uint8_t  RESERVED6[0xE0];
  __IO uint32_t IPRIOR[64];
       uint8_t  RESERVED7[0x810];
  __IO uint32_t SCTLR;
} PFIC_Type;

/* PFIC peripheral access macro                                               */
#define PFIC                          ((PFIC_Type*)PFIC_BASE)

/* Bit definitions for PFIC Configuration Register (CFGR)                     */
#define PFIC_CFGR_HWSTKCTRL           0x00000001UL
#define PFIC_CFGR_NESTCTRL            0x00000002UL
#define PFIC_CFGR_NMISET              0x00000004UL
#define PFIC_CFGR_NMIRESET            0x00000008UL
#define PFIC_CFGR_EXCSET              0x00000010UL
#define PFIC_CFGR_EXCRESET            0x00000020UL
#define PFIC_CFGR_PFICRESET           0x00000040UL
#define PFIC_CFGR_SYSRESET            0x00000080UL
#define PFIC_CFGR_KEYCODE_KEY1        0xFA050000UL
#define PFIC_CFGR_KEYCODE_KEY2        0xBCAF0000UL
#define PFIC_CFGR_KEYCODE_KEY3        0xBEEF0000UL

/* Bit definitions for PFIC System Control Register (SCTLR)                   */
#define PFIC_SCTLR_SLEEPONEXIT        0x00000002UL
#define PFIC_SCTLR_SLEEPDEEP          0x00000004UL
#define PFIC_SCTLR_WFITOWFE           0x00000008UL
#define PFIC_SCTLR_SEVONPEND          0x00000010UL
#define PFIC_SCTLR_SETEVENT           0x00000020UL

/* Bit definitions for PFIC Interrupt Priority Threshold Register (ITHRESDR)  */
#define PFIC_ITHRESDR_THRESHOLD_LOW   0x00000010UL

/* Interrupt number from IRQn                                                 */
#define PFIC_IRQn_NUM(IRQn)           ((uint32_t)(IRQn) & 0x1FUL)

/* Interrupt register offset from IRQn                                        */
#define PFIC_IRQn_REG(IRQn)           ((uint32_t)(IRQn) >> 5)

/*!****************************************************************************
 * @brief
 * Enable Interrupt
 *
 * @param[in] IRQn        Interrupt Number
 * @date  30.04.2020
 * @date  26.02.2023  Code Style
 ******************************************************************************/
RV_STATIC_INLINE void PFIC_EnableIRQ(IRQn_Type IRQn)
{
  PFIC->IENR[PFIC_IRQn_REG(IRQn)] = 1UL << PFIC_IRQn_NUM(IRQn);
}

/*!****************************************************************************
 * @brief
 * Disable Interrupt
 *
 * @param[in] IRQn        Interrupt Number
 * @date  30.04.2020
 * @date  26.02.2023  Code Style
 ******************************************************************************/
RV_STATIC_INLINE void PFIC_DisableIRQ(IRQn_Type IRQn)
{
  /* Temporarily set to mininum threshold to prevent inter-
   * rupt from firing                                     */
  register uint32_t ulThrBkp = PFIC->ITHRESDR;
  PFIC->ITHRESDR = PFIC_ITHRESDR_THRESHOLD_LOW;

  /* Reset enable-flag                                    */
  PFIC->IRER[PFIC_IRQn_REG(IRQn)] = 1UL << PFIC_IRQn_NUM(IRQn);

  /* Restore priority threshold                           */
  PFIC->ITHRESDR = ulThrBkp;
}

/*!****************************************************************************
 * @brief
 * Get Interrupt Enable State
 *
 * @param[in] IRQn        Interrupt Number
 * @return  (uint32_t)  Interrupt Enable State
 * @retval  0           Interrupt is disabled
 * @retval  1           Interrupt is enabled
 * @date  30.04.2020
 * @date  26.02.2023  Code Style
 ******************************************************************************/
RV_STATIC_INLINE uint32_t PFIC_GetStatusIRQ(IRQn_Type IRQn)
{
  return (PFIC->ISR[PFIC_IRQn_REG(IRQn)] & (1UL << PFIC_IRQn_NUM(IRQn))) ? 1UL : 0UL;
}

/*!****************************************************************************
 * @brief
 * Get Interrupt Pending State
 *
 * @param[in] IRQn        Interrupt Number
 * @return  (uint32_t)  Interrupt Pending State
 * @retval  0           Interrupt is not pending
 * @retval  1           Interrupt is pending
 * @date  30.04.2020
 * @date  26.02.2023  Code Style
 ******************************************************************************/
RV_STATIC_INLINE uint32_t PFIC_GetPendingIRQ(IRQn_Type IRQn)
{
  return (PFIC->IPR[PFIC_IRQn_REG(IRQn)] & (1UL << PFIC_IRQn_NUM(IRQn))) ? 1UL : 0UL;
}

/*!****************************************************************************
 * @brief
 * Set Interrupt Pending State
 *
 * @param[in] IRQn        Interrupt Number
 * @date  30.04.2020
 * @date  26.02.2023  Code Style
 ******************************************************************************/
RV_STATIC_INLINE void PFIC_SetPendingIRQ(IRQn_Type IRQn)
{
  PFIC->IPSR[PFIC_IRQn_REG(IRQn)] = 1UL << PFIC_IRQn_NUM(IRQn);
}

/*!****************************************************************************
 * @brief
 * Clear Interrupt Pending State
 *
 * @param[in] IRQn        Interrupt Number
 * @date  30.04.2020
 * @date  26.02.2023  Code Style
 ******************************************************************************/
RV_STATIC_INLINE void PFIC_ClearPendingIRQ(IRQn_Type IRQn)
{
  PFIC->IPRR[PFIC_IRQn_REG(IRQn)] = 1UL << PFIC_IRQn_NUM(IRQn);
}

/*!****************************************************************************
 * @brief
 * Get Interrupt Active State
 *
 * @param[in] IRQn        Interrupt Number
 * @return  (uint32_t)  Interrupt Active State
 * @retval  0           Interrupt is not active
 * @retval  1           Interrupt is active
 * @date  30.04.2020
 * @date  26.02.2023  Code Style
 ******************************************************************************/
RV_STATIC_INLINE uint32_t PFIC_GetActive(IRQn_Type IRQn)
{
  return (PFIC->IACTR[PFIC_IRQn_REG(IRQn)] & (1UL << PFIC_IRQn_NUM(IRQn))) ? 1UL : 0UL;
}

/*!****************************************************************************
 * @brief
 * Set Interrupt Priority
 *
 * @param[in] IRQn        Interrupt Number
 * @param[in] priority    Priority value (7: pre-emption, 6-4: subpriority)
 * @date  30.04.2020
 * @date  26.02.2023  Code Style; 32-bit wide register access
 ******************************************************************************/
RV_STATIC_INLINE void PFIC_SetPriority(IRQn_Type IRQn, uint8_t priority)
{
  PFIC->IPRIOR[(IRQn >> 2)] = (uint32_t)priority << (IRQn & 0x3UL);
}

/*!****************************************************************************
 * @brief
 * Set Vector-Table-Free (VTF) Interrupt Handler
 *
 * @note All ISR addresses are relative to a common base address specified
 * in FIBADDRR.
 *
 * @note Address bits 23:20 of the ISR must be set to 0.
 *
 * @param[in] channel     VTF Interrupt Channel (0..3)
 * @param[in] address     VTF Interrupt Handler Address
 * @param[in] IRQn        Assigned Interrupt Number
 * @date  30.04.2020
 * @date  26.02.2023  Code Style; Argument Ordering; Function Name
 ******************************************************************************/
RV_STATIC_INLINE void PFIC_ConfigFastIRQ(uint8_t channel, uint32_t address, IRQn_Type IRQn)
{
  PFIC->FIBADDRR = address & 0xF0000000UL;
  PFIC->FIOFADDRR[channel & 0x3UL] = ((uint32_t)IRQn << 24) | (address & 0x000FFFFFUL);
}

/*!****************************************************************************
 * @brief
 * Initiate System Reset
 *
 * @date  30.04.2020
 * @date  26.02.2023  Coding Style
 ******************************************************************************/
RV_STATIC_INLINE void PFIC_SystemReset(void)
{
  PFIC->CFGR = PFIC_CFGR_KEYCODE_KEY3 | PFIC_CFGR_SYSRESET;
}

/*!****************************************************************************
 * @brief
 * Configure HPE (Hardware Prologue/Epilogue) and Interrupt Nesting function
 *
 * @param[in] hpe         Enable or disable HPE
 * @param[in] nest        Enable or disable interrupt nesting
 * @date  26.02.2023
 ******************************************************************************/
RV_STATIC_INLINE void PFIC_Config(FunctionalState hpe, FunctionalState nest)
{
  PFIC->CFGR = PFIC_CFGR_KEYCODE_KEY1           | \
    ((hpe == ENABLE) ? 0 : PFIC_CFGR_HWSTKCTRL) | \
    ((nest == ENABLE) ? 0 : PFIC_CFGR_NESTCTRL);
}


/* ########################### SysTick Timer ################################ */
/* Memory-Mapped SysTick Register File                                        */
typedef struct
{
  __IO uint32_t CTLR;
  union {
    struct __attribute__((packed)) {
      __IO uint8_t CNTL0;
      __IO uint8_t CNTL1;
      __IO uint8_t CNTL2;
      __IO uint8_t CNTL3;
    };
    __I uint32_t CNTL;
  };
  union {
    struct __attribute__((packed)) {
      __IO uint8_t CNTH0;
      __IO uint8_t CNTH1;
      __IO uint8_t CNTH2;
      __IO uint8_t CNTH3;
    };
    __I uint32_t CNTH;
  };
  union {
    struct __attribute__((packed)) {
      __IO uint8_t CMPLR0;
      __IO uint8_t CMPLR1;
      __IO uint8_t CMPLR2;
      __IO uint8_t CMPLR3;
    };
    __I uint32_t CMPLR;
  };
  union {
    struct __attribute__((packed)) {
      __IO uint8_t CMPHR0;
      __IO uint8_t CMPHR1;
      __IO uint8_t CMPHR2;
      __IO uint8_t CMPHR3;
    };
    __I uint32_t CMPHR;
  };
} SysTick_Type;

/* SysTick access macro                                                       */
#define SysTick                       ((SysTick_Type*)SysTick_BASE)

/* Bit definitions for SysTick register                                       */
#define SYSTICK_CTLR_STE              0x00000001UL  /* Counter Enable         */

/* Access functions for 8-bit write registers                                 */
#define SYSTICK_REG_WRITE8(reg,value) {     \
  SysTick->reg##3 = (uint8_t)(value >> 24); \
  SysTick->reg##2 = (uint8_t)(value >> 16); \
  SysTick->reg##1 = (uint8_t)(value >>  8); \
  SysTick->reg##0 = (uint8_t)(value >>  0); \
}

/*!****************************************************************************
 * @brief
 * Set SysTick counter state
 *
 * @param[in] state       Enable or disable counter
 * @date  18.02.2022
 * @date  24.02.2022  Changed naming convention
 ******************************************************************************/
RV_STATIC_FORCE_INLINE void SysTick_Cmd(FunctionalState state)
{
  if (state != DISABLE)
  {
    SysTick->CTLR |= SYSTICK_CTLR_STE;
  }
  else
  {
    SysTick->CTLR &= ~SYSTICK_CTLR_STE;
  }
}

/*!****************************************************************************
 * @brief
 * Set SysTick counter low word
 *
 * Byte-wise copy into SysTick CNTL register
 *
 * @param[in] value       Counter value, low word
 * @date  18.02.2022
 * @date  24.02.2022  Changed naming convention
 ******************************************************************************/
RV_STATIC_FORCE_INLINE void SysTick_SetValueLow(uint32_t value)
{
  SYSTICK_REG_WRITE8(CNTL, value);
}

/*!****************************************************************************
 * @brief
 * Set SysTick counter high word
 *
 * Byte-wise copy into SysTick CNTH register
 *
 * @param[in] value       Counter value, high word
 * @date  18.02.2022
 * @date  24.02.2022  Changed naming convention
 ******************************************************************************/
RV_STATIC_FORCE_INLINE void SysTick_SetValueHigh(uint32_t value)
{
  SYSTICK_REG_WRITE8(CNTH, value);
}

/*!****************************************************************************
 * @brief
 * Set SysTick counter value (doubleword)
 *
 * @param[in] value       64-bit counter value
 * @date  18.02.2022
 * @date  24.02.2022  Changed naming convention
 ******************************************************************************/
RV_STATIC_FORCE_INLINE void SysTick_SetValue(uint64_t value)
{
  SysTick_SetValueHigh((uint32_t)(value >> 32));
  SysTick_SetValueLow((uint32_t)value);
}

/*!****************************************************************************
 * @brief
 * Get SysTick counter value, low word
 *
 * @return  (uint32_t)  Counter value, low word
 * @date  18.02.2022
 * @date  24.02.2022  Changed naming convention
 ******************************************************************************/
RV_STATIC_FORCE_INLINE uint32_t SysTick_GetValueLow(void)
{
  return SysTick->CNTL;
}

/*!****************************************************************************
 * @brief
 * Get SysTick counter value, high word
 *
 * @return  (uint32_t)  Counter value, low word
 * @date  24.02.2022
 ******************************************************************************/
RV_STATIC_FORCE_INLINE uint32_t SysTick_GetValueHigh(void)
{
  return SysTick->CNTH;
}

/*!****************************************************************************
 * @brief
 * Get 64 bit SysTick counter value
 *
 * Low and high words may be out of sync
 *
 * @return  (uint64_t)  Counter value
 * @date  18.02.2022
 ******************************************************************************/
RV_STATIC_FORCE_INLINE uint64_t SysTick_GetValue64(void)
{
  return ((uint64_t)SysTick->CNTH << 32) | (uint64_t)SysTick->CNTL;
}

/*!****************************************************************************
 * @brief
 * Set SysTick compare value low word
 *
 * @param[in] value       Compare value, low word
 * @date  18.02.2022
 * @date  24.02.2022  Changed naming convention
 ******************************************************************************/
RV_STATIC_FORCE_INLINE void SysTick_SetCompareLow(uint32_t value)
{
  SYSTICK_REG_WRITE8(CMPLR, value);
}

/*!****************************************************************************
 * @brief
 * Set SysTick compare value high word
 *
 * @param[in] value       Compare value, high word
 * @date  18.02.2022
 * @date  24.02.2022  Changed naming convention
 ******************************************************************************/
RV_STATIC_FORCE_INLINE void SysTick_SetCompareHigh(uint32_t value)
{
  SYSTICK_REG_WRITE8(CMPHR, value);
}

/*!****************************************************************************
 * @brief
 * Set SysTick compare value (doubleword)
 *
 * @param[in] value       64-bit compare value
 * @date  18.02.2022
 * @date  24.02.2022  Changed naming convention
 ******************************************************************************/
RV_STATIC_FORCE_INLINE void SysTick_SetCompare(uint64_t value)
{
  SysTick_SetCompareHigh((uint32_t)(value >> 32));
  SysTick_SetCompareLow((uint32_t)value);
}


/* ############################ Core Functions ############################## */
/*!****************************************************************************
 * @brief
 * No Operation (NOP)
 *
 * @date  30.04.2020
 ******************************************************************************/
RV_STATIC_FORCE_INLINE void __NOP()
{
  __asm volatile ("nop");
}

/*!****************************************************************************
 * @brief
 * Set Event Flag
 *
 * @date  26.02.2023
 ******************************************************************************/
RV_STATIC_FORCE_INLINE void __SEV()
{
  PFIC->SCTLR |= PFIC_SCTLR_SETEVENT;
}

/*!****************************************************************************
 * @brief
 * Wait For Interrupt (WFI)
 *
 * @date  30.04.2020
 * @date  26.02.2023  Coding style
 ******************************************************************************/
RV_STATIC_FORCE_INLINE void __WFI(void)
{
  /* Instruction is executed as WFI                       */
  PFIC->SCTLR &= ~PFIC_SCTLR_WFITOWFE;
  __asm volatile ("wfi");
}

/*!****************************************************************************
 * @brief
 * Wait For Event (WFE)
 *
 * @date  30.04.2020
 * @date  26.02.2023  Removed internal __sev
 ******************************************************************************/
RV_STATIC_FORCE_INLINE void __WFE(void)
{
  /* Instruction is executed as WFE                       */
  PFIC->SCTLR |= PFIC_SCTLR_WFITOWFE;
  __asm volatile ("wfi");
}

/*!****************************************************************************
 * @brief
 * Signal Debugger Break (EBREAK)
 *
 * @date  26.02.2023
 ******************************************************************************/
RV_STATIC_FORCE_INLINE void __EBREAK(void)
{
  __asm volatile ("ebreak");
}


/* ###################### Machine Register Access ########################### */
#define TEMPLATE_CSR_GETTER_FN(fname, csr)                                     \
/*!****************************************************************************
 * @brief
 * CSR Getter Function Template: uint32_t fname(void)
 *
 * @param[in] fname       (Template symbol) Function name
 * @param[in] csr         (Template string literal) CSR name
 * @return  (uint32_t)  CSR value
 * @date  26.02.2023
 *****************************************************************************/\
  uint32_t fname(void)                                                         \
  {                                                                            \
    uint32_t result;                                                           \
    __asm volatile ("csrr %0, " csr : "=r"(result));                           \
    return result;                                                             \
  }
#define TEMPLATE_CSR_SETTER_FN(fname, csr)                                     \
/*!****************************************************************************
 * @brief
 * CSR Setter Function Template: void fname(value)
 *
 * @param[in] fname       (Template symbol) Function name
 * @param[in] csr         (Template string literal) CSR name
 * @param[in] value       CSR value
 * @date  26.02.2023
 *****************************************************************************/\
  void fname(uint32_t value)                                                   \
  {                                                                            \
    __asm volatile ("csrw " csr ", %0" :: "r"(value));                         \
  }

/* CSR Getter and Setter template instantiations                              */
RV_STATIC_FORCE_INLINE TEMPLATE_CSR_GETTER_FN(__get_MSTATUS,    "mstatus");
RV_STATIC_FORCE_INLINE TEMPLATE_CSR_SETTER_FN(__set_MSTATUS,    "mstatus");
RV_STATIC_FORCE_INLINE TEMPLATE_CSR_GETTER_FN(__get_MISA,       "misa");
RV_STATIC_FORCE_INLINE TEMPLATE_CSR_GETTER_FN(__get_MIE,        "mie");
RV_STATIC_FORCE_INLINE TEMPLATE_CSR_SETTER_FN(__set_MIE,        "mie");
RV_STATIC_FORCE_INLINE TEMPLATE_CSR_GETTER_FN(__get_MTVEC,      "mtvec");
RV_STATIC_FORCE_INLINE TEMPLATE_CSR_SETTER_FN(__set_MTVEC,      "mtvec");
RV_STATIC_FORCE_INLINE TEMPLATE_CSR_GETTER_FN(__get_MSCRATCH,   "mscratch");
RV_STATIC_FORCE_INLINE TEMPLATE_CSR_SETTER_FN(__set_MSCRATCH,   "mscratch");
RV_STATIC_FORCE_INLINE TEMPLATE_CSR_GETTER_FN(__get_MEPC,       "mepc");
RV_STATIC_FORCE_INLINE TEMPLATE_CSR_SETTER_FN(__set_MEPC,       "mepc");
RV_STATIC_FORCE_INLINE TEMPLATE_CSR_GETTER_FN(__get_MCAUSE,     "mcause");
RV_STATIC_FORCE_INLINE TEMPLATE_CSR_SETTER_FN(__set_MCAUSE,     "mcause");
RV_STATIC_FORCE_INLINE TEMPLATE_CSR_GETTER_FN(__get_MTVAL,      "mtval");
RV_STATIC_FORCE_INLINE TEMPLATE_CSR_SETTER_FN(__set_MTVAL,      "mtval");
RV_STATIC_FORCE_INLINE TEMPLATE_CSR_GETTER_FN(__get_MIP,        "mip");
RV_STATIC_FORCE_INLINE TEMPLATE_CSR_SETTER_FN(__set_MIP,        "mip");
RV_STATIC_FORCE_INLINE TEMPLATE_CSR_GETTER_FN(__get_MCYCLE,     "mcycle");
RV_STATIC_FORCE_INLINE TEMPLATE_CSR_SETTER_FN(__set_MCYCLE,     "mcycle");
RV_STATIC_FORCE_INLINE TEMPLATE_CSR_GETTER_FN(__get_MCYCLEH,    "mcycleh");
RV_STATIC_FORCE_INLINE TEMPLATE_CSR_SETTER_FN(__set_MCYCLEH,    "mcycleh");
RV_STATIC_FORCE_INLINE TEMPLATE_CSR_GETTER_FN(__get_MINSTRET,   "minstret");
RV_STATIC_FORCE_INLINE TEMPLATE_CSR_SETTER_FN(__set_MINSTRET,   "minstret");
RV_STATIC_FORCE_INLINE TEMPLATE_CSR_GETTER_FN(__get_MINSTRETH,  "minstreth");
RV_STATIC_FORCE_INLINE TEMPLATE_CSR_SETTER_FN(__set_MINSTRETH,  "minstreth");
RV_STATIC_FORCE_INLINE TEMPLATE_CSR_GETTER_FN(__get_MVENDORID,  "mvendorid");
RV_STATIC_FORCE_INLINE TEMPLATE_CSR_GETTER_FN(__get_MARCHID,    "marchid");
RV_STATIC_FORCE_INLINE TEMPLATE_CSR_GETTER_FN(__get_MIMPID,     "mimpid");
RV_STATIC_FORCE_INLINE TEMPLATE_CSR_GETTER_FN(__get_MHARTID,    "mhartid");


/* ############################## Other ##################################### */
/*!****************************************************************************
 * @brief
 * Get Stack Pointer value
 *
 * @return  (uint32_t)  SP register value
 * @date  30.04.2020
 * @date  26.02.2023
 ******************************************************************************/
RV_STATIC_FORCE_INLINE uint32_t __get_SP(void)
{
  uint32_t result;
  __asm volatile ("mv %0, sp" : "=r"(result));
  return result;
}

/*!****************************************************************************
 * @brief
 * Disable machine interrupts
 *
 * Clears MIE bit in mstatus register
 *
 * @date  26.02.2023
 ******************************************************************************/
RV_STATIC_FORCE_INLINE void __disable_irq(void)
{
  __asm volatile ("csrci mstatus, 0x08" ::: "memory");
}

/*!****************************************************************************
 * @brief
 * Enable machine interrupts
 *
 * Sets MIE bit in mstatus register
 *
 * @date  26.02.2023
 ******************************************************************************/
RV_STATIC_FORCE_INLINE void __enable_irq(void)
{
  __asm volatile ("csrsi mstatus, 0x08" ::: "memory");
}

#endif /* __CORE_RISCV_H__ */
