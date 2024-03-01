#include<uORB/uORB.h>
#include"setpoint_6dof.h"
#include<drivers/drv_hrt.h>
#include<msg/tmp/headers/manual_control_setpoint.h>
#include"uart.h"
#include <sys/types.h>



using namespace matrix;
using namespace std;
extern "C" __EXPORT int setpoint_6dof_main(int argc, char *argv[]);

int Setpoint_6dof::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

Setpoint_6dof::	Setpoint_6dof():
	ModuleParams(nullptr)
{
	/* _actuators_2_pub = nullptr;
	_actuators_id2 = nullptr;
	_actuators_2_circuit_breaker_enabled = false;
	memset(&_actuators2, 0 ,sizeof(_actuators2));//结构体清零 */

}
Setpoint_6dof::~Setpoint_6dof()
{
	PX4_WARN("hased");
}
int Setpoint_6dof::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("Setpoint_6dof",
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
Setpoint_6dof *Setpoint_6dof::instantiate(int argc, char *argv[])
{
	Setpoint_6dof *instance=new Setpoint_6dof();
	if(instance==nullptr)
	{
		PX4_ERR("alloc failed");
	}
	return instance;
}

void Setpoint_6dof::run()
{


	char data = '0';
	char buffer[8] = "0";


	int succe;

	int uart_read = uart_init((char*)"/dev/ttyS2");

        if(false == uart_read)
	{succe = -1;
	PX4_WARN("uart init false");}

        if(false == set_uart_baudrate(uart_read,115200)){
	PX4_WARN("12%f",(double)succe);
        PX4_WARN("[YCM]set_uart_baudrate is failed\n");

        }
	PX4_WARN("BAUD RATE is set");

	while(1)
	{
 		read(uart_read,&data,1);
		if(data == 'R')
		{
    		for(int i = 0;i <8;i++)
		{
		read(uart_read,&data,1);
       		 buffer[i] = data;
		 setpoint.setpoint_6[i] = data;		//顺序为：xyzrpy 加速 舵机；加速和舵机为直通，取值为-30到30，对应为pwm 0到3000
       		 data = '0';}
		PX4_WARN("%s\n",buffer);
		}

		//setpoint.setpoint_6[0] = 1.0;
		setpoint_6dof_pub.publish(setpoint);
		PX4_WARN("RUNING");



	}

	}



int Setpoint_6dof::print_usage(const char *reason )
{
	PX4_WARN("Setpoint_6dof start/stop");
	if (reason) {
		PX4_WARN("%s\n", reason);
	}
	return 0;
}

int setpoint_6dof_main(int argc, char*argv[])
{

	return Setpoint_6dof::main(argc,argv);
}
