/******************************************************************************
 * _start
 * Reset handler
 ******************************************************************************/
.section  .text.startup
.balign 4
.globl  _start
_start:
  /* Clear registers                                                          */
  lui   x0,     0                     /* Dummy write to 0 with zeros          */
regfile_init:
  li    x1,     0
  li    x2,     0
  li    x3,     0
  li    x4,     0
  li    x5,     0
  li    x6,     0
  li    x7,     0
  li    x8,     0
  li    x9,     0
  li    x10,    0
  li    x11,    0
  li    x12,    0
  li    x13,    0
  li    x14,    0
  li    x15,    0
  li    x16,    0
  li    x17,    0
  li    x18,    0
  li    x19,    0
  li    x20,    0
  li    x21,    0
  li    x22,    0
  li    x23,    0
  li    x24,    0
  li    x25,    0
  li    x26,    0
  li    x27,    0
  li    x28,    0
  li    x29,    0
  li    x30,    0
  li    x31,    0

/* Disable interrupts during setup                                            */
_csr_preinit:
  csrw  mstatus,  zero                /* Disable all interrupts               */

/* Initialise C-Runtime Library                                               */
_crt0_init:
  .option push
  .option norelax
  la    sp,   _estack                 /* Initialise stack pointer             */
  la    gp,   __global_pointer$       /* Initialise global pointer            */
  .option pop

__crt0_clear_bss:
  la    a0,   _sbss                   /* Start of .bss section                */
  la    a1,   _ebss                   /* End of .bss section                  */
__crt0_clear_bss_loop:
  bgeu  a0,   a1,   __crt0_copy_data  /* Done, or skip if no .bss present     */
  sw    zero, 0(a0)
  addi  a0,   a0,   4                 /* Word-wise increment of write addr    */
  j     __crt0_clear_bss_loop

__crt0_copy_data:
  la    a0,   _sidata                 /* Start LMA of .data section ("src")   */
  la    a1,   _sdata                  /* Start VMA of .data section ("dest")  */
  la    a2,   _edata                  /* End VMA of .data section             */
__crt0_copy_data_loop:
  bgeu  a1,   a2,   _csr_init         /* Done, or skip if no .data present    */
  lw    t0,   0(a0)                   /* Load word from LMA                   */
  sw    t0,   0(a1)                   /* Store word to VMA                    */
  addi  a0,   a0,   4                 /* Word-wise increment of LMA and VMA   */
  addi  a1,   a1,   4
  j     __crt0_copy_data_loop

/* Initialise CSRs for interrupt handling                                     */
_csr_init:
  la    a0,   _vector_base            /* Set trap vector address              */
  ori   a0,   a0,   1                 /* Vectored mode                        */
  csrw  mtvec,  a0
  li    t0,   0x88                    /* Enable interrupts                    */
  csrw  mstatus,  t0

/* Jump to main() function                                                    */
_app_start:
  jal   SystemInit                    /* Core system init                     */
__app_main_enter:
  li    a0,   0                       /* argc = 0                             */
  li    a1,   0                       /* argv = NULL                          */
  jal   ra,   main                    /* Call main() with return address      */
__app_main_exit:
  j     __app_main_exit               /* Endless loop                         */

