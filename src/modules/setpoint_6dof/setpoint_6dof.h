/**
 * @file setpoint_6dof.h
 *
 * 通过uart接收来自上层的数据，水下机器人的6自由度的期望值
 *
 * @author Sunyi
 */

#ifndef SETPOINT_6DOF_H_
#define SETPOINT_6DOF_H_
#include <stdint.h>
#include <matrix/matrix/math.hpp>

#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include<px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include<msg/tmp/headers/actuator_controls.h>
//#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/setpoint_6dof.h>

__BEGIN_DECLS

class Setpoint_6dof : public ModuleBase<Setpoint_6dof>, public ModuleParams
{
public:
	Setpoint_6dof();
	~Setpoint_6dof() override;
	static int task_spawn(int argc, char*argv[]);
	void run()override;
	static Setpoint_6dof *instantiate(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	//uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Publication<setpoint_6dof_s> setpoint_6dof_pub{ORB_ID(setpoint_6dof)};

private:
	setpoint_6dof_s setpoint;




/* 	orb_advert_t _actuators_2_pub;
	orb_id_t _actuators_id2;
	bool	_actuators_2_circuit_breaker_enabled;
	struct actuator_controls_s	_actuators2;
	struct vehicle_attitude_s		_v_att {};		 */


};
__END_DECLS
#endif
