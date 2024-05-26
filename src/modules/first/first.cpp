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
	float distance;

	int uart_read = uart_init((char*)"/dev/ttyS6");

        if(false == uart_read)
	succe = -1;

        if(false == set_uart_baudrate(uart_read,115200)){
		printf("12%f",(double)succe);

        }

	while(1)
	{
		PX4_WARN("runing");
		distance = read_distance(uart_read);
		PX4_WARN("distance:%f",(double)distance);
	}
	}


float First::read_distance(int uart_read)
{
	char data = '0';
	char buffer[17] = "0";
	float distance_true;
	char write_b[] = "happy\r\n";

	read(uart_read,&data,1);
	if(data == '$'){
       		for(int i = 0;i <17;i++){
			read(uart_read,&data,1);
        	        buffer[i] = data;
        	        data = '0';}
		distance_true = (float)(buffer[11]-'0') + (float)(buffer[13]-'0')*0.1f + (float)(buffer[14]-'0')*0.01f;
		write(uart_read, &write_b, sizeof(write_b));
	}
	else
		distance_true = 0.0f;

	return distance_true;
	return 1;
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
