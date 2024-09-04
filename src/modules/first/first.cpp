#include<uORB/uORB.h>
#include"first.h"
#include<drivers/drv_hrt.h>
#include<msg/tmp/headers/manual_control_setpoint.h>
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
	int succe;

	int uart_read = uart_init((char*)"/dev/ttyS6");
        if(false == uart_read)succe = -1;
        if(false == set_uart_baudrate(uart_read,115200)){
	printf("12%f",(double)succe);
     //   printf("[YCM]set_uart_baudrate is failed\n");

        }

	while(1)
	{

	_manual_control_setpoint_sub.update(&_man_set);	//订阅遥控器输入

	float aux_1 = _man_set.aux1;
	if (aux_1 < 0){
	//1
        char con0_write[16] =#002P1500T0000!;
	char con1_write[16] =#003P1900T0000!;
	char con2_write[16] =#004P1500T0000!;
	char con3_write[16] =#005P1500T0000!;

	write_ser(uart_read , con0_write);
	write_ser(uart_read , con1_write);
	write_ser(uart_read , con2_write);
	write_ser(uart_read , con3_write);
	usleep(10000000);
	//2
        char con0_write[16] =#002P1500T0000!;
	char con1_write[16] =#003P1500T0000!;
	char con2_write[16] =#004P1500T0000!;
	char con3_write[16] =#005P1500T0000!;

	write_ser(uart_read , con0_write);
	write_ser(uart_read , con1_write);
	write_ser(uart_read , con2_write);
	write_ser(uart_read , con3_write);
	usleep(10000000);
	//3
	char con0_write[16] =#002P1900T0000!;
	char con1_write[16] =#003P1500T0000!;
	char con2_write[16] =#004P1300T0000!;
	char con3_write[16] =#005P1300T0000!;

	write_ser(uart_read , con0_write);
	write_ser(uart_read , con1_write);
	write_ser(uart_read , con2_write);
	write_ser(uart_read , con3_write);
	usleep(10000000);
	//4
	char con0_write[16] =#002P1300T0000!;
	char con1_write[16] =#003P1300T0000!;
	char con2_write[16] =#004P1300T0000!;
	char con3_write[16] =#005P1300T0000!;

	write_ser(uart_read , con0_write);
	write_ser(uart_read , con1_write);
	write_ser(uart_read , con2_write);
	write_ser(uart_read , con3_write);
	usleep(10000000);
	//5
	char con0_write[16] =#002P1900T0000!;
	char con1_write[16] =#003P1900T0000!;
	char con2_write[16] =#004P1300T0000!;
	char con3_write[16] =#005P1300T0000!;

	write_ser(uart_read , con0_write);
	write_ser(uart_read , con1_write);
	write_ser(uart_read , con2_write);
	write_ser(uart_read , con3_write);
	usleep(10000000);
	//6
	char con0_write[16] =#002P1300T0000!;
	char con1_write[16] =#003P1500T0000!;
	char con2_write[16] =#004P1650T0000!;
	char con3_write[16] =#005P1650T0000!;

	write_ser(uart_read , con0_write);
	write_ser(uart_read , con1_write);
	write_ser(uart_read , con2_write);
	write_ser(uart_read , con3_write);
	usleep(20000000);

	}
	else{
	char con0_write[16] =#002P1500T0000!;
	char con1_write[16] =#003P1500T0000!;
	char con2_write[16] =#004P1500T0000!;
	char con3_write[16] =#005P1500T0000!;

	write_ser(uart_read , con0_write);
	write_ser(uart_read , con1_write);
	write_ser(uart_read , con2_write);
	write_ser(uart_read , con3_write);

	}

	}


void First::write_ser(int uart_read ,char *con0_write)
{

	char data = '0';
	char buffer[30] = "0";
	write(uart_read,&con0_write,15);
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
