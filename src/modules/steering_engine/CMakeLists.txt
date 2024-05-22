px4_add_module(
	MODULE module__steering_engine
	MAIN steering_engine
	STACK_MAIN 2000
	COMPILE_FLAGS
	SRCS
		steering_engine.cpp
		steering_engine.h
	DEPENDS
		mahony
)
