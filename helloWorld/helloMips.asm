addiu $v0, $zero, 0x0004
syscall
jr $ra
nop

lui $sp 0x7FFF
jal 0x0100014
ori $sp, $sp, 0xFFFC

addiu $v0, $zero, 0x000A
syscall
nop
nop
nop

addiu $sp, $sp, 0xFFE8
sw $ra, 0x0014 $sp
 
sw $fp, 0x0010 $sp
addu $fp,$sp,$zero
lui $v0, 0x0040
addiu $a0, $v0, 0x0090
jal 0x0100008
nop
addu $sp, $fp, $zero

lw $ra, 0x0014 $sp
lw $fp, 0x0010 $sp
addiu $sp, $sp, 0x0018
jr $ra
