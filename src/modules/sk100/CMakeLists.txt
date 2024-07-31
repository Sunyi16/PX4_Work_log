px4_add_module(
	MODULE module__sk100
	MAIN sk100
	STACK_MAIN 1200
	COMPILE_FLAGS
	SRCS
		sk100.cpp
		sk100.h
		uart.h
)
