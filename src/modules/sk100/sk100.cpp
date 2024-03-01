#include<uORB/uORB.h>
#include"sk100.h"
#include<drivers/drv_hrt.h>
#include<msg/tmp/headers/manual_control_setpoint.h>
#include"uart.h"
#include <sys/types.h>



using namespace matrix;
using namespace std;
extern "C" __EXPORT int sk100_main(int argc, char *argv[]);

int Sk100::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

Sk100::	Sk100():
	ModuleParams(nullptr)
{
	/* _actuators_2_pub = nullptr;
	_actuators_id2 = nullptr;
	_actuators_2_circuit_breaker_enabled = false;
	memset(&_actuators2, 0 ,sizeof(_actuators2));//结构体清零 */

}
Sk100::~Sk100()
{
	PX4_WARN("hased");
}
int Sk100::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("Sk100",
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
Sk100 *Sk100::instantiate(int argc, char *argv[])
{
	Sk100 *instance=new Sk100();
	if(instance==nullptr)
	{
		PX4_ERR("alloc failed");
	}
	return instance;
}

int Sk100::convert(char str[])
{
    /**********  Begin  **********/
    int i = 0, j=0,num = 0, sum = 0,len=0;
    //int len = strlen(str);//这种方法会将非16进制数记录在内
    while ((str[i] >= 'a' && str[i] <= 'f') || (str[i] >= 'A' && str[i] <= 'F') || (str[i] >= '0' && str[i] <= '9'))
    {
        len++;
        i++;
    } //遍历数组记录16进制数的个数，非16进制数不计在内
    i = 0;
    while (str[i] != '\0')
    {
        //字符转数字
        if (str[i] >= '0' && str[i] <= '9')
            num = str[i] - '0';
        else if (str[i] >= 'a' && str[i] <= 'f')
            num = str[i] - 'a' + 10;
        else if (str[i] >= 'A' && str[i] <= 'F')
            num = str[i] - 'A' + 10;
        else
            break;//遇到'\0'之前的第一个非十六进制数就停止循环
        for (j = 0; j < len - 1; j++)
        {
            num = num * 16;
        }
        sum += num;
        i++;
        len--;//每读取一位就使长度-1
    }
    return sum;
    /**********  End  **********/
}



void Sk100::run()
{


	char data = '0';
	char buffer[30] = "0";
	char value[8] = "0";

	const char cmd1[21] = "0xAA0000200001000425";		//启动连续自动测量，并返回距离值

	int succe;

	int uart_read = uart_init((char*)"/dev/ttyS6");

        if(false == uart_read)
	{succe = -1;
	PX4_WARN("uart init false");}

        if(false == set_uart_baudrate(uart_read,19200)){
	PX4_WARN("12%f",(double)succe);
        PX4_WARN("[YCM]set_uart_baudrate is failed\n");

        }
	PX4_WARN("BAUD RATE is set");

	write(uart_read,cmd1,21);

	while(1)
	{

    		for(int i = 0;i <28;i++)
		{
		read(uart_read,&data,1);
       		 buffer[i] = data;
       		 data = '0';}

		for(int l=0; l<7; l++)
		{
			value[l] = buffer[13+l];
		}

		int sum = convert(value);

		PX4_WARN("%s\n",buffer);

		dis.diatance = sum;
		sk100_pub.publish(dis);
		PX4_WARN("RUNING");



	}

	}



int Sk100::print_usage(const char *reason )
{
	PX4_WARN("Sk100 start/stop");
	if (reason) {
		PX4_WARN("%s\n", reason);
	}
	return 0;
}

int sk100_main(int argc, char*argv[])
{

	return Sk100::main(argc,argv);
}
