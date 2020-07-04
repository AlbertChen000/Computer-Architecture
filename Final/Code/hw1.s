.data
    n: .word 10 # You can change this number
    
.text
.globl __start

FUNCTION:
    # Todo: define your own function in HW1
    addi x9, x0, 2
    addi x11, x0, 4
    addi sp, sp, -8 
    sw x1, 4(sp)
    sw x10,0(sp)
    addi x7, x10, -1
    beq x7, x0,  44
    srli x10, x10, 1
    jal x1, -24
    addi x7, x10, 0
    lw x1, 4(sp)
    lw x10,0(sp)
    addi sp, sp, 8
    mul x28, x7, x11
    mul x29, x10, x9
    add x10, x28, x29
    jalr x0, 0(x1)	  
    addi x10, x0, 1
    addi sp, sp, 8
    jalr x0, 0(x1) 
    

# Do not modify this part!!! #
__start:                     #
    la   t0, n               #
    lw   x10, 0(t0)          #
    jal  x1,FUNCTION         #
    la   t0, n               #
    sw   x10, 4(t0)          #
    addi a0,x0,10            #
    ecall                    #
##############################