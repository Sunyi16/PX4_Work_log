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
#include<msg/tmp/headers/actuator_controls.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/parameter_update.h>

__BEGIN_DECLS

using namespace std;


class Steering_engine : public ModuleBase<Steering_engine>,  public ModuleParams
{
public:
	Steering_engine();
	~Steering_engine() override;
	static int task_spawn(int argc, char*argv[]);
	void run()override;
	void parameters_updated();					//update the param of PWM_VALUE
	static Steering_engine *instantiate(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);


private:
	float Pwm_value;
	float Pwm_value_mid;
	actuator_controls_s _actuators2;
	uORB::Publication<actuator_controls_s>  _actuators2_set{ORB_ID(actuator_controls_2)};           /*pwm setpoint publication*/
	uORB::Subscription _params_sub{ORB_ID(parameter_update)};			/**< parameter updates subscription */

	/*Define a param to set the pwm value*/
	DEFINE_PARAMETERS(
	(ParamInt<px4::params::PWM_VALUE>) _param_pwm_value

	)
};

__END_DECLS
#endif
