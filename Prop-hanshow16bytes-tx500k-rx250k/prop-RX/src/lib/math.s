;/**
; ****************************************************************************************
; *
; * @file math.s
; *
; * @brief 64-bits Unsigned Multiplication & Division.
; *
; * Copyright(C) 2015 NXP Semiconductors N.V.
; * All rights reserved.
; *
; * $Rev: 1.0 $
; *
; ****************************************************************************************
; */

                PRESERVE8
                THUMB

    EXPORT __rd_reg
    EXPORT __wr_reg
    EXPORT __wr_reg_with_msk
    EXPORT __aeabi_lmul
    EXPORT __aeabi_uldivmod
        
    AREA    REG_ACCESS_SECTION, CODE, READONLY, ALIGN=2

__rd_reg PROC
    NOP
    LDR      r0,[r0,#0x00]
    BX       lr
    ENDP

    ;AREA    WR_REG_SECTION, CODE, READONLY, ALIGN=2

    ALIGN 4
    
__wr_reg PROC
    NOP
    STR      r1,[r0,#0x00]
    BX       lr
    ENDP

    ALIGN 4
    
__wr_reg_with_msk PROC
    NOP
    LDR      r3, [r0,#0x00]
    BICS     r3, r3, r1
    ANDS     r2, r2, r1
    ORRS     r3, r3, r2
    STR      r3, [r0,#0x00]
    BX       lr
    ENDP

    AREA    |.text|, CODE, READONLY, ALIGN=2

__aeabi_lmul PROC

    PUSH     {r4-r7,lr}
    MOV      r12,r0
    UXTH     r7,r0
    UXTH     r4,r2
    MOV      r5,r7
    SUB      sp,sp,#0x14
    MOV      lr,r4
    MULS     r7,r4,r7
    MOVS     r4,#0x00
    STR      r4,[sp,#0x04]
    LSLS     r4,r1,#16
    LSRS     r0,r0,#16
    ORRS     r0,r0,r4
    MOV      r4,lr
    STR      r0,[sp,#0x08]
    UXTH     r0,r0
    STR      r0,[sp,#0x00]
    MULS     r0,r4,r0
    MOVS     r6,#0x00
    LSLS     r6,r6,#16
    LSRS     r4,r0,#16
    ORRS     r6,r6,r4
    LDR      r4,[sp,#0x04]
    LSLS     r0,r0,#16
    ADDS     r0,r0,r7
    UXTH     r7,r1
    ADCS     r6,r6,r4
    MOV      r4,lr
    MULS     r7,r4,r7
    MOVS     r4,#0x00
    ADDS     r0,r4,r0
    ADCS     r7,r7,r6
    ASRS     r6,r1,#16
    MULS     r6,r2,r6
    LSLS     r6,r6,#16
    ADDS     r0,r4,r0
    ADCS     r6,r6,r7
    MOV      r7,r5
    STR      r0,[sp,#0x10]
    LSLS     r0,r3,#16
    LSRS     r2,r2,#16
    ORRS     r2,r2,r0
    MOV      lr,r2
    MOVS     r0,#0x00
    UXTH     r2,r2
    STR      r2,[sp,#0x04]
    MULS     r7,r2,r7
    LSLS     r0,r0,#16
    LSRS     r2,r7,#16
    ORRS     r2,r2,r0
    LDR      r0,[sp,#0x10]
    LSLS     r7,r7,#16
    ADDS     r7,r7,r0
    ADCS     r6,r6,r2
    LDR      r2,[sp,#0x04]
    LDR      r0,[sp,#0x00]
    MULS     r0,r2,r0
    MOV      r2,r6
    ADDS     r6,r4,r7
    MOV      r7,r5
    ADCS     r0,r0,r2
    MOV      r2,lr
    MULS     r1,r2,r1
    LSLS     r1,r1,#16
    ADDS     r2,r4,r6
    ADCS     r1,r1,r0
    UXTH     r0,r3
    MULS     r7,r0,r7
    LDR      r0,[sp,#0x08]
    ADDS     r2,r4,r2
    ADCS     r7,r7,r1
    MULS     r0,r3,r0
    LSLS     r5,r0,#16
    ADDS     r0,r4,r2
    ADCS     r5,r5,r7
    MOV      r2,r12
    ASRS     r1,r3,#16
    ADD      sp,sp,#0x14
    MULS     r1,r2,r1
    LSLS     r1,r1,#16
    ADDS     r0,r4,r0
    ADCS     r1,r1,r5
    POP      {r4-r7,pc}

    ENDP

__aeabi_uldivmod PROC

    PUSH     {r1-r7,lr}
    MOV      r5,r0
    MOV      r0,r2
    MOV      r4,r1
    ORRS     r0,r0,r3
    BEQ      __pos1
    MOV      lr,r5
    MOV      r12,r1
    MOVS     r0,#0x00
    SUBS     r5,r5,r2
    MOV      r1,r0
    SBCS     r4,r4,r3
    BCC      __pos2
    MOV      r6,r12
    MOVS     r4,#0x01
    MOVS     r5,#0x00
    SUBS     r6,r6,r2
    SBCS     r5,r5,r3
    BCC      __pos3
    MOV      r3,r2
    MOVS     r2,#0x00
    MOVS     r4,#0x21
__pos3
    MOV      r5,r12
    MOV      r6,lr
    LSLS     r7,r5,#16
    LSRS     r6,r6,#16
    ORRS     r6,r6,r7
    LSRS     r5,r5,#16
    SUBS     r6,r6,r2
    SBCS     r5,r5,r3
    BCC      __pos4
    LSRS     r5,r2,#16
    LSLS     r3,r3,#16
    ORRS     r3,r3,r5
    LSLS     r2,r2,#16
    ADDS     r4,r4,#0x10
__pos4
    MOV      r5,r12
    MOV      r6,lr
    LSLS     r7,r5,#24
    LSRS     r6,r6,#8
    ORRS     r6,r6,r7
    LSRS     r5,r5,#8
    SUBS     r6,r6,r2
    SBCS     r5,r5,r3
    BCC      __pos5
    LSRS     r5,r2,#24
    LSLS     r3,r3,#8
    ORRS     r3,r3,r5
    LSLS     r2,r2,#8
    ADDS     r4,r4,#0x08
__pos5
    MOV      r5,r12
    MOV      r6,lr
    LSLS     r7,r5,#28
    LSRS     r6,r6,#4
    ORRS     r6,r6,r7
    LSRS     r5,r5,#4
    SUBS     r6,r6,r2
    SBCS     r5,r5,r3
    BCC      __pos6
    LSRS     r5,r2,#28
    LSLS     r3,r3,#4
    ORRS     r3,r3,r5
    LSLS     r2,r2,#4
    ADDS     r4,r4,#4
__pos6
    MOV      r5,r12
    MOV      r6,lr
    LSLS     r7,r5,#30
    LSRS     r6,r6,#2
    ORRS     r6,r6,r7
    LSRS     r5,r5,#2
    SUBS     r6,r6,r2
    SBCS     r5,r5,r3
    BCC      __pos7
    LSRS     r5,r2,#30
    LSLS     r3,r3,#2
    ORRS     r3,r3,r5
    LSLS     r2,r2,#2
    ADDS     r4,r4,#2
__pos7
    MOV      r5,r12
    MOV      r6,lr
    LSLS     r7,r5,#31
    LSRS     r6,r6,#1
    ORRS     r6,r6,r7
    LSRS     r5,r5,#1
    SUBS     r6,r6,r2
    SBCS     r5,r5,r3
    BCC      __pos8
    ADDS     r2,r2,r2
    ADCS     r3,r3,r3
    ADDS     r4,r4,#1
    B        __pos8
__pos10
    ADDS     r0,r0,r0
    MOV      r6,lr
    MOV      r5,r12
    ADCS     r1,r1,r1
    SUBS     r7,r6,r2
    SBCS     r5,r5,r3
    STR      r1,[sp,#0x04]
    STR      r0,[sp,#0x00]
    BCC      __pos9
    MOV      r0,r12
    SUBS     r1,r6,r2
    SBCS     r0,r0,r3
    MOV      lr,r1
    MOV      r12,r0
    LDR      r0,[sp,#0x00]
    LDR      r1,[sp,#0x04]
    MOVS     r5,#0x00
    ADDS     r0,r0,#1
    ADCS     r1,r1,r5
__pos9
    LSLS     r5,r3,#31
    LSRS     r2,r2,#1
    ORRS     r2,r2,r5
    LSRS     r3,r3,#1
__pos8
    SUBS     r4,r4,#1
    BPL      __pos10
__pos2
    MOV      r2,lr
    MOV      r3,r12
__pos11
    ADD      sp,sp,#0x0C
    POP      {r4-r7,pc}
    B        __pos1
__pos1
    MOVS     r0,#0x00
    MOV      r1,r0
    MOV      r8,r8
    MOV      r8,r8
    MOV      r2,r5
    MOV      r3,r4
    B        __pos11

    ENDP

    END