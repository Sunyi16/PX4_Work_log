/**
 * @file steer_engine.cpp
 *
 *
 *
 * @author
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
	PX4_WARN("runing");
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

	while(!PX4_OK)
	{
	sensor_combined_s sensors;

	if (sensor_sub.update(&sensors)){
		gyro[0] = sensors.gyro_rad[0];
		gyro[1] = sensors.gyro_rad[1];
		gyro[2] = sensors.gyro_rad[2];
		accel[0] = sensors.accelerometer_m_s2[0];
		accel[1] = sensors.accelerometer_m_s2[1];
		accel[2] = sensors.accelerometer_m_s2[2];
	}
        pitch = Imu_Update(gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2], 1);
        roll = Imu_Update(gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2], 0);

        att_mahony_s att_h = {};
	att_h.pitch = pitch;
	att_h.roll = roll;
	//printf("data:%d",(double)att[0]);
	/* the instance count is not used here */
	att_h.timestamp = hrt_absolute_time();
	_att_pub.publish(att_h);

	PX4_INFO("data1:%f data2:%f", (double)pitch, (double)roll);

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
