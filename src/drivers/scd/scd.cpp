/*
*针对SCD_40的I2C驱动
*
*/

#include "scd.hpp"

Scd::Scd(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config)
{
}

Scd::~Scd()
{
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}


int Scd::init()
{
	uint8_t cmd_reset = RESET_CMD;
	uint8_t cmd_prom1 = PROM_CMD1;
	uint8_t cmd_prom2 = PROM_CMD2;
	uint8_t cmd_prom3 = PROM_CMD3;
	uint8_t cmd_prom4 = PROM_CMD4;
	uint8_t cmd_prom5 = PROM_CMD5;
	uint8_t cmd_prom6 = PROM_CMD6;
	int ret = I2C::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("I2C::init failed (%i)", ret);
		return ret;
	}

	if (ret == PX4_OK) {
		ScheduleNow();
	}

	int reset = transfer(&cmd_reset, 1, nullptr, 0);
	if(reset){
		int prom_cmd1 = transfer(&cmd_prom1, 1, nullptr, 0);
	  prom_cmd1 = transfer(nullptr, 0, &C1[0], 2);
		 prom_cmd1 = transfer(&cmd_prom2, 1, nullptr, 0);
	 prom_cmd1 = transfer(nullptr, 0, &C2[0], 2);
		 prom_cmd1 = transfer(&cmd_prom3, 1, nullptr, 0);
	 prom_cmd1 = transfer(nullptr, 0, &C3[0], 2);
		 prom_cmd1 = transfer(&cmd_prom4, 1, nullptr, 0);
	 prom_cmd1 = transfer(nullptr, 0, &C4[0], 2);
		 prom_cmd1 = transfer(&cmd_prom5, 1, nullptr, 0);
	 prom_cmd1 = transfer(nullptr, 0, &C5[0], 2);
		 prom_cmd1 = transfer(&cmd_prom6, 1, nullptr, 0);
	 prom_cmd1 = transfer(nullptr, 0, &C6[0], 2);


	vC1 = zhuanhuan(C1, 2);
	vC2 = zhuanhuan(C2, 2);
	vC3 = zhuanhuan(C3, 2);
	vC4 = zhuanhuan(C4, 2);
	vC5 = zhuanhuan(C5, 2);
	vC6 = zhuanhuan(C6, 2);

	return prom_cmd1;
	}



	return ret;
}

void Scd::measure(uint8_t cmd)
{
	/*
	 * 发送启动测量数据之前的命令
	 */
	int ret = transfer(&cmd, 2, nullptr, 0);

	if (OK != ret) {
		perf_count(_comms_errors);
	}


}

int Scd::zhuanhuan(uint8_t A[], int n)
{
	int val3;

	if(n==2){
	val3 = A[0] << 8;
	val3 |= A[1];
	}
	else
	{
	int val = A[0] << 16;
	int val1 = A[1]<<8;
	val3 = val | val1 | A[2];
	}
	return val3;

}

int Scd::collect()
{
	/* 读取数据 */
	uint8_t P[24] = {0};
	uint8_t T[24] = {0};
	int32_t vP = 0;
	int32_t vT = 0;

	perf_begin(_sample_perf);

	measure(P_CMD);
	int retp = transfer(nullptr, 0, &P[0], 3);

	measure(T_CMD);
	int rett = transfer(nullptr, 0, &T[0], 3);



	if (retp < 0 || rett < 0) {
		perf_count(_comms_errors);
		return -1;
	}

	vP = zhuanhuan(P, 3);
	vT = zhuanhuan(T, 3);

	int DT = vT - vC5*256;
	int TEMP = 20+DT*vC6/8388608;		//一阶计算后温度
	int OFF = vC2*65536+(vC4*DT)/128;
	int SENS = vC1*32768+(vC3*DT)/256;
	int Pre = (vP*SENS/2097152-OFF)/8192;		//温度补偿后压力


	scd.temp = TEMP;
	scd.pressure = Pre;			//换算后为水深mm
	scd.timestamp = hrt_absolute_time();
	scd_pub.publish(scd);




	perf_end(_sample_perf);

	return PX4_OK;
}

void Scd::RunImpl()
{
	int ret;

	/* collection phase? */
	if (_collect_phase) {

		/* perform collection */
		ret = collect();

		if (OK != ret) {
			perf_count(_comms_errors);
			/* restart the measurement state machine */
			_collect_phase = false;
			_sensor_ok = false;
			ScheduleNow();
			return;
		}

		/* next phase is measurement */
		_collect_phase = false;

		/*
		 * Is there a collect->measure gap?
		 */
		if (_measure_interval > CONVERSION_INTERVAL) {

			/* schedule a fresh cycle call when we are ready to measure again */
			ScheduleDelayed(_measure_interval - CONVERSION_INTERVAL);

			return;
		}
	}

	/* measurement phase */
/* 	ret = measure();

	if (OK != ret) {
		DEVICE_DEBUG("measure error");
	}

	_sensor_ok = (ret == OK); */

	/* next phase is collection */
	_collect_phase = true;

	/* schedule a fresh cycle call when the measurement is done */
	ScheduleDelayed(CONVERSION_INTERVAL);
}
