/**
 * @file adrc.h
 *
 * Definition of generic ADRC controller.
 *
 * @author Sunyi
 */

#ifndef FIRST_H_
#define FIRST_H_
#include <stdint.h>
#include<px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include<uORB/topics/actuator_controls.h>
#include <uORB/topics/pwm_input.h>
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/scd.h>


__BEGIN_DECLS

class First : public ModuleBase<First>, public ModuleParams
{
public:
	First();
	~First() override;
	static int task_spawn(int argc, char*argv[]);
	void run()override;
	static First *instantiate(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);


	struct actuator_controls_s	_actuators2;
	uORB::Subscription sr_sub{ORB_ID(scd)};
	struct scd_s sr_value;

	uORB::Subscription _params_sub{ORB_ID(parameter_update)};			/**< parameter updates subscription */
	uORB::Publication<actuator_controls_s> _act2_pub{ORB_ID(actuator_controls_2)};

private:
	float angle_1;
	float angle_2;
	float angle_3;
	float angle_4;
	float dis_tance;

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::ANGLE_ONE>) _param_angle_one,/*The angle of four arms*/
		(ParamInt<px4::params::ANGLE_TWO>) _param_angle_two,
		(ParamInt<px4::params::ANGLE_THREE>) _param_angle_three,
		(ParamInt<px4::params::ANGLE_FOUR>) _param_angle_four,
		(ParamInt<px4::params::DISTANCE>) _param_distance
	)
};
__END_DECLS
#endif
