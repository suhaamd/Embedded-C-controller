@ This ARM Assembler code should implement a matching function, for use in the MasterMind program, as
@ described in the CW2 specification. It should produce as output 2 numbers, the first for the
@ exact matches (peg of right colour and in right position) and approximate matches (peg of right
@ color but not in right position). Make sure to count each peg just once!

@ Example (first sequence is secret, second sequence is guess):
@ 1 2 1
@ 3 1 3 ==> 0 1
@ You can return the result as a pointer to two numbers, or two values
@ encoded within one number
@
@ -----------------------------------------------------------------------------

.text
@ this is the matching fct that should be called from the C part of the CW
.global         matches
@ use the name `main` here, for standalone testing of the assembler code
@ when integrating this code into `master-mind.c`, choose a different name
@ otw there will be a clash with the main function in the C code
.global         global
main:
LDR  R2, =secret @ pointer to secret sequence
LDR  R3, =guess @ pointer to guess sequence

@ you probably need to initialise more values here

exit: @MOV R0, R4 @ load result to output register
MOV R7, #1 @ load system call code
SWI 0 @ return this value

@ -----------------------------------------------------------------------------
@ sub-routines

@ this is the matching fct that should be callable from C
matches:
str fp, [sp, #-4]!
add fp, sp, #0
sub sp, sp, #60
str r0, [fp, #-56]
str r1, [fp, #-60]
mov r3, #0
str r3, [fp, #-8]
mov r3, #0
str r3, [fp, #-12]
sub r3, fp, #36
mov r2, #0
str r2, [r3]
str r2, [r3, #4]
str r2, [r3, #8]
sub r3, fp, #48
mov r2, #0
str r2, [r3]
str r2, [r3, #4]
str r2, [r3, #8]
mov r3, #0
str r3, [fp, #-16]
b .L30
.L32:
ldr r3, [fp, #-16]
lsl r3, r3, #2
ldr r2, [fp, #-56]
add r3, r2, r3
ldr r2, [r3]
ldr r3, [fp, #-16]
lsl r3, r3, #2
ldr r1, [fp, #-60]
add r3, r1, r3
ldr r3, [r3]
cmp r2, r3
bne .L31
ldr r3, [fp, #-8]
add r3, r3, #1
str r3, [fp, #-8]
ldr r3, [fp, #-16]
lsl r3, r3, #2
sub r2, fp, #4
add r3, r2, r3
mov r2, #1
str r2, [r3, #-32]
ldr r3, [fp, #-16]
lsl r3, r3, #2
sub r2, fp, #4
add r3, r2, r3
mov r2, #1
str r2, [r3, #-44]
.L31:
ldr r3, [fp, #-16]
add r3, r3, #1
str r3, [fp, #-16]
.L30:
ldr r3, [fp, #-16]
cmp r3, #2
ble .L32
mov r3, #0
str r3, [fp, #-20]
b .L33
.L38:
ldr r3, [fp, #-20]
lsl r3, r3, #2
sub r2, fp, #4
add r3, r2, r3
ldr r3, [r3, #-32]
cmp r3, #0
bne .L34
mov r3, #0
str r3, [fp, #-24]
b .L35
.L37:
ldr r3, [fp, #-24]
lsl r3, r3, #2
sub r2, fp, #4
add r3, r2, r3
ldr r3, [r3, #-44]
cmp r3, #0
bne .L36
ldr r3, [fp, #-20]
lsl r3, r3, #2
ldr r2, [fp, #-56]
add r3, r2, r3
ldr r2, [r3]
ldr r3, [fp, #-24]
lsl r3, r3, #2
ldr r1, [fp, #-60]
add r3, r1, r3
ldr r3, [r3]
cmp r2, r3
bne .L36
ldr r3, [fp, #-12]
add r3, r3, #1
str r3, [fp, #-12]
ldr r3, [fp, #-20]
lsl r3, r3, #2
sub r2, fp, #4
add r3, r2, r3
mov r2, #1
str r2, [r3, #-32]
ldr r3, [fp, #-24]
lsl r3, r3, #2
sub r2, fp, #4
add r3, r2, r3
mov r2, #1
str r2, [r3, #-44]
b .L34
.L36:
ldr r3, [fp, #-24]
add r3, r3, #1
str r3, [fp, #-24]
.L35:
ldr r3, [fp, #-24]
cmp r3, #2
ble .L37
.L34:
ldr r3, [fp, #-20]
add r3, r3, #1
str r3, [fp, #-20]
.L33:
ldr r3, [fp, #-20]
cmp r3, #2
ble .L38
ldr r2, [fp, #-8]
mov r3, r2
lsl r3, r3, #2
add r3, r3, r2
lsl r3, r3, #1
mov r2, r3
ldr r3, [fp, #-12]
add r3, r2, r3
mov r0, r3
add sp, fp, #0
@ sp needed
ldr fp, [sp], #4
bx lr

@ show the sequence in R0, use a call to printf in libc to do the printing, a useful function when debugging
showseq:
    @ Optional

@ =============================================================================

.data

@ constants about the basic setup of the game: length of sequence and number of colors
.equ LEN, 3
.equ COL, 3
.equ NAN1, 3
.equ NAN2, 3

@ a format string for printf that can be used in showseq
f4str: .asciz "Seq:    %d %d %d\n"

@ a memory location, initialised as 0, you may need this in the matching fct
n: .word 0x00

@ INPUT DATA for the matching function
.align 4
secret: .word 1
.word 2
.word 1

.align 4
guess: .word 3
.word 1
.word 3

@ Not strictly necessary, but can be used to test the result
@ Expect Answer: 0 1
.align 4
expect: .byte 0
.byte 1

.align 4
secret1: .word 1
.word 2
.word 3

.align 4
guess1: .word 1
.word 1
.word 2

@ Not strictly necessary, but can be used to test the result
@ Expect Answer: 1 1
.align 4
expect1: .byte 1
.byte 1

.align 4
secret2: .word 2
.word 3
.word 2

.align 4
guess2: .word 3
.word 3
.word 1

@ Not strictly necessary, but can be used to test the result
@ Expect Answer: 1 0
.align 4
expect2: .byte 1
.byte 0
