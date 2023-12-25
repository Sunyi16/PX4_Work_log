/**
 * @file steer_engine.cpp
 *
 * control motor stabilization at one value.
 *
 * @author Sunyi
 */


#include<uORB/uORB.h>
#include"steering_engine.h"
#include<drivers/drv_hrt.h>

extern "C" __EXPORT int steering_engine_main(int argc, char *argv[]);

int Steering_engine::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

Steering_engine::Steering_engine():
	ModuleParams(nullptr)
{
	parameters_updated();
}

Steering_engine::~Steering_engine()
{
	PX4_WARN("hased");
}

void Steering_engine::parameters_updated()
{
        Pwm_value_mid = (float)_param_pwm_value.get();
	Pwm_value = (Pwm_value_mid-1500)/500;;
}

int Steering_engine::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("steering_engine",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT,
				      1024,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0)
	{
		_task_id = -1;
		return -errno;
	}

	return 0;
}

Steering_engine *Steering_engine::instantiate(int argc, char *argv[])
{
	Steering_engine *instance=new Steering_engine();
	if (instance==nullptr)
	{
		PX4_ERR("alloc failed");
	}
	return instance;
}

void Steering_engine::run()
{

	while(1)
	{
		/*update params, if the value of PWM_VALUE has changed*/
		if (_params_sub.updated())
		{
			// clear update
			parameter_update_s param_update;
			_params_sub.copy(&param_update);
			updateParams();
			parameters_updated();
		}
		/*publish the pwm_value*/
		_actuators2.control[4] = Pwm_value;
		_actuators2.timestamp = hrt_absolute_time();
		_actuators2_set.publish(_actuators2);
		PX4_INFO("runing");
		up_pwm_servo_set(9,Pwm_value);
	}
}

int Steering_engine::print_usage(const char *reason )
{
	PX4_WARN("steering_engine start/stop/status");
	if (reason)
	{
		PX4_WARN("%s\n", reason);
	}
	return 0;
}

int steering_engine_main(int argc, char*argv[])
{
	return Steering_engine::main(argc,argv);
}
