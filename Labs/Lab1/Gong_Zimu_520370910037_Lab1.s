.data
str: .string "hello World!"

.text
    la a0, str
    lui a1, 0x10000
    addi a1, a1, 0x100
loop:
    lb t0, 0(a0)
    sb t0, 0(a1)
    addi a0, a0, 1
    addi a1, a1, 1
    bgtz t0, loop
    
    