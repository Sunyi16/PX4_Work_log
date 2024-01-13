/**
 * @file scd.cpp
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
		if (_scd_sub.updated())
		{
			_scd_sub.copy(&scd_value);
		}

		for(int i=0; i<9; i++){
		PX4_INFO("data:%f",scd_value.data[i]);

		}
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
