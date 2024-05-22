/**
 * @file steer_engine.h
 *
 * control motor stabilization at one value.
 *
 * @author Sunyi
 */

#ifndef STEERING_ENGINE_H_
#define STEERING_ENGINE_H_

#include <stdint.h>
#include<px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include<uORB/topics/actuator_controls.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/att_mahony.h>
#include <lib/mahony/mahony_CG.cpp>


__BEGIN_DECLS

using namespace std;


class Steering_engine : public ModuleBase<Steering_engine>,  public ModuleParams
{
public:
	Steering_engine();
	~Steering_engine() override;
	static int task_spawn(int argc, char*argv[]);
	void run()override;
	static Steering_engine *instantiate(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);


private:
	uORB::Subscription sensor_sub{ORB_ID(sensor_combined)};
	uORB::Publication<att_mahony_s>	_att_pub{ORB_ID(att_mahony)};

	sensor_combined_s  sensor = {};
	att_mahony_s att = {};

	float gyro[3], accel[3];
	float pitch, roll;

};

__END_DECLS
#endif
