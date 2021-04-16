.syntax unified
.cpu cortex-m3
.fpu softvfp
.thumb

.equ GPIOB_CRH,   0x40010c04
.equ GPIOB_IDR,   0x40010C08
.equ GPIOB_BSRR,  0x40010c10
.equ GPIOB_BSR,   0x40010c14

.weak sensor_measure
.type sensor_measure, %function
sensor_measure:
    push { r1, r2, r3, r4, r5, r6, r7, r8, r9, r10, lr}
    mov r6, r0
    cmp r1, 8
    it cs
    addcs r6, 0x4 @; CRH or CRL

    mov r3, 1
    mov r3, r3, lsl r1

    add r7, r0, 0x14 @; BSR
    add r8, r0, 0x10 @; BSRR
    add r9, r0, 0x8  @; IDR

    and r4, r1, 7
    mov r4, r4, lsl 2

    mov r0, 3
    mov r5, 0xf
    mov r0, r0, lsl r4

    mov r5, r5, lsl r4
    mvn r5, r5

    ldr r10, [r6]
    and r10, r5
    orr r0, r10
    str r0, [r6]

    ldr r10, [r7]
    orr r0, r3, r10
    str r0, [r7]

    push {r3, r4}
    mov r0, 1
    bl HAL_Delay
    pop {r3, r4}

    mov r0, 8
    mov r0, r0, lsl r4

    ldr r10, [r6]
    and r10, r5
    orr r0, r10
    str r0, [r6]

    ldr r10, [r8]
    orr r0, r3, r10
    str r0, [r8]

    mov r0, -1
    ZALOOP:
        add r0, r0, 1
        cmp r0, 16
        it hs
        bhs ZALOOP_END

        ldr r1, [r9]
        ands r1, r1, r3
        it eq
        beq ZALOOP
    ZALOOP_END:
    sub r0, r0, 1
    pop { r1, r2, r3, r4, r5, r6, r7, r8, r9, r10, lr}
    bx lr