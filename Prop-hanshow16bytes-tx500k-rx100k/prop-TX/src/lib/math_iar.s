;/**
; ****************************************************************************************
; *
; * @file math_iar.s
; *
; * @brief
; *
; * Copyright(C) 2015 NXP Semiconductors N.V.
; * All rights reserved.
; *
; * $Rev: 1.0 $
; *
; ****************************************************************************************
; */
        SECTION .test:CODE (4)

        PUBLIC  __rd_reg
__rd_reg
        NOP
        LDR      r0,[r0,#0x00]
        BX       lr


        SECTION .test:CODE (4)

        PUBLIC  __wr_reg
__wr_reg
        NOP
        STR      r1,[r0,#0x00]
        BX       lr

        SECTION .test:CODE (4)

        PUBLIC  __wr_reg_with_msk
__wr_reg_with_msk
        NOP
        LDR      r3, [r0,#0x00]
        BICS     r3, r3, r1
        ANDS     r2, r2, r1
        ORRS     r3, r3, r2
        STR      r3, [r0,#0x00]
        BX       lr

        END
