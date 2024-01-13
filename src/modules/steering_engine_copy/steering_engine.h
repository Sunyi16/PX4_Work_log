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
#include <uORB/Subscription.hpp>
#include <uORB/topics/scd.h>


__BEGIN_DECLS

using namespace std;


class Steering_engine : public ModuleBase<Steering_engine>,  public ModuleParams
{
public:
	Steering_engine();
	~Steering_engine() override;
	static int task_spawn(int argc, char*argv[]);
	void run()override;					//update the param of PWM_VALUE
	static Steering_engine *instantiate(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);


private:

	uORB::Subscription _scd_sub{ORB_ID(scd)};
	scd_s scd_value;

};

__END_DECLS
#endif
