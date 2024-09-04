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
#include <matrix/matrix/math.hpp>

#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include<px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/topics/manual_control_setpoint.h>

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
	void write_ser(int uart_read ,char *con0_write);

	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};

	struct manual_control_setpoint_s		_man_set {};		/**< vehicle attitude */


};
__END_DECLS
#endif
