Main: 
    jal x1, FACT
    ecall

FACT: 
    addi sp, sp, -8
    sw x1, 4(sp)
    sw x12, 0(sp)
    add x18, x0, x12
    addi x5, x0, 2
    bge x12, x5, L1
    addi x10, x0, 1
    addi sp, sp, 8
    jalr x0, x1, 0 
L1:
    addi x12, x12, -1
    jal x1, FACT
    addi x6, x10, 0
    lw x1, 4(sp)
    lw x10, 0(sp)
    addi sp, sp, 8
    mul x10, x10, x6
    jalr x0, x1, 0