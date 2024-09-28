#include<uORB/uORB.h>
#include"first.h"
#include<drivers/drv_hrt.h>
#include"uart.h"


using namespace matrix;
using namespace std;
extern "C" __EXPORT int first_main(int argc, char *argv[]);

int First::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

First::	First():
	ModuleParams(nullptr)
{
	/* _actuators_2_pub = nullptr;
	_actuators_id2 = nullptr;
	_actuators_2_circuit_breaker_enabled = false;
	memset(&_actuators2, 0 ,sizeof(_actuators2));//结构体清零 */

}
First::~First()
{
	PX4_WARN("hased");
}
int First::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("first",
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
First *First::instantiate(int argc, char *argv[])
{
	First *instance=new First();
	if(instance==nullptr)
	{
		PX4_ERR("alloc failed");
	}
	return instance;
}

void First::run()
{
	if(_manual_setpoint_sub.update(&manual))
	{
		printf("已经捕获遥控器输入\n");
	}
	float aux_2 = manual.aux2;
	bool key;	//用来判断是否需要发送数据

	if(aux_2 >= 0)
	{
		key = true;
		printf("发送数据,tx高电平\n");
	}else
	{
	key = false;
	printf("发送数据,tx低电平\n");
	}

	int succe;

	int uart_read = uart_init((char*)"/dev/ttyS6");
        if(false == uart_read)succe = -1;
        if(false == set_uart_baudrate(uart_read,115200)){
	printf("12%f",(double)succe);


        }


        char con1_write[16] ="000000000000000";

	while(key)
	{
	write(uart_read,&con1_write,15);

        printf("running");

	}

	}



int First::print_usage(const char *reason )
{
	PX4_WARN("first start/stop");
	if (reason) {
		PX4_WARN("%s\n", reason);
	}
	return 0;
}

int first_main(int argc, char*argv[])
{

	return First::main(argc,argv);
}
