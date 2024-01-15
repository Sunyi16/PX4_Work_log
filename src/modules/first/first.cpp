#include<uORB/uORB.h>
#include"first.h"
#include<drivers/drv_hrt.h>
#include<msg/tmp/headers/manual_control_setpoint.h>

using namespace std;
extern "C" __EXPORT int first_main(int argc, char *argv[]);

int First::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

First::	First():
	ModuleParams(nullptr)
{
	/* _actuators_2_pub = nullptr;
	_actuators_id2 = nullptr;
	_actuators_2_circuit_breaker_enabled = false;
	memset(&_actuators2, 0 ,sizeof(_actuators2));//结构体清零 */

}
First::~First()
{
	PX4_WARN("hased");
}
int First::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("first",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT,
				      2000,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return 0;
	}

	return 0;
}
First *First::instantiate(int argc, char *argv[])
{
	First *instance=new First();
	if(instance==nullptr)
	{
		PX4_ERR("alloc failed");
	}
	return instance;
}

void First::run()
{
while(1){

	sr_sub.copy(&sr_value);
	//float distance = 0.00034f*sr_value.pulse_width/2;
	float distance =  sr_value.data[0];
	//PX4_INFO("data:%f",double(distance));


/* 	if (_params_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_params_sub.copy(&param_update);

	angle_1 = (float)_param_angle_one.get()/10;
	angle_2 = (float)_param_angle_two.get()/10;
	angle_3 = (float)_param_angle_three.get()/10;
	angle_4 = (float)_param_angle_four.get()/10;
	dis_tance = (float)_param_distance.get();
	}
 */

	angle_1 =-0.5f ;
	angle_2 =-0.5f ;
	angle_3 =0.0f ;
	angle_4 =-0.5f;
	dis_tance = 3;

	if(distance <= dis_tance)
	{
	_actuators2.control[4] = angle_1;//-30
	_actuators2.control[5] = angle_2;
	_actuators2.control[6] = angle_3;
	_actuators2.control[7] = angle_4;
	}
	else{
	_actuators2.control[4] = -0.5f;//-30
	_actuators2.control[5] = -0.5f;
	_actuators2.control[6] = -0.5f;
	_actuators2.control[7] = -0.5f;
	}

	//_actuators2.control[7] = 1.0f;//对应2000us最高值
	_actuators2.timestamp = hrt_absolute_time();
	//_actuators2.timestamp_sample = _ctrl_state.timestamp;
	_act2_pub.publish(_actuators2);

	PX4_INFO("runing");
}

	}


int First::print_usage(const char *reason )
{
	PX4_WARN("first start/stop");
	if (reason) {
		PX4_WARN("%s\n", reason);
	}
	return 0;
}

int first_main(int argc, char*argv[])
{

	return First::main(argc,argv);
}
