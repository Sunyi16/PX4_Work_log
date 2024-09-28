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
#include <uORB/topics/scd.h>
#include <px4_platform_common/px4_config.h>
#include <px4_arch/io_timer.h>
#include <uORB/topics/input_rc.h>


__BEGIN_DECLS

using namespace std;


class Steering_engine : public ModuleBase<Steering_engine>,  public ModuleParams
{
public:
	Steering_engine();
	~Steering_engine() override;
	static int task_spawn(int argc, char*argv[]);
	void run()override;
	void parameters_updated();
	static Steering_engine *instantiate(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	uORB::Subscription	_manual_sub{ORB_ID(input_rc)};


private:
	struct input_rc_s	manual{};

};

__END_DECLS
#endif
