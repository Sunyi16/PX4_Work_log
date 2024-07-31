px4_add_module(
	MODULE module__setpoint_6dof
	MAIN setpoint_6dof
	STACK_MAIN 1200
	COMPILE_FLAGS
	SRCS
		setpoint_6dof.cpp
		setpoint_6dof.h
		uart.h
)
