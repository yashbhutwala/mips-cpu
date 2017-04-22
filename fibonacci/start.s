.global __start
.text

# this is normally the LibC startup code
# main() also returns here
__start:

	# initialize stack pointer to top of memory
	li $sp, 0x7ffffffc


    jal main
    li $v0, 10  # syscall 10 is exit()
    syscall
