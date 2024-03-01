/**
 * @file sk1oo.h
 *
 * uart协议
 * 激光测距仪用于水下机器人的底部避障，此模块实现内容：
 * 将sk100检测到的数据进行计算，得到最终距离，发送到sk100话题
 *
 * @author Sunyi
 */

#ifndef SK100_H_
#define SK100_H_
#include <stdint.h>
#include <matrix/matrix/math.hpp>

#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include<px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include<msg/tmp/headers/actuator_controls.h>
//#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/sk100.h>

__BEGIN_DECLS

class Sk100 : public ModuleBase<Sk100>, public ModuleParams
{
public:
	Sk100();
	~Sk100() override;
	static int task_spawn(int argc, char*argv[]);
	void run()override;
	static Sk100 *instantiate(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	int convert(char str[]);

	//uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Publication<sk100_s> sk100_pub{ORB_ID(sk100)};

private:
	sk100_s dis;




/* 	orb_advert_t _actuators_2_pub;
	orb_id_t _actuators_id2;
	bool	_actuators_2_circuit_breaker_enabled;
	struct actuator_controls_s	_actuators2;
	struct vehicle_attitude_s		_v_att {};		 */


};
__END_DECLS
#endif
