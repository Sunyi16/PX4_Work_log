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

int Scd::probe()
{
	_retries = 1;
	int ret = measure();

	return ret;
}

int Scd::init()
{
	int ret = I2C::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("I2C::init failed (%i)", ret);
		return ret;
	}

	if (ret == PX4_OK) {
		ScheduleNow();
	}

	return ret;
}

int Scd::measure()
{
	/*
	 * 发送启动测量数据之前的命令
	 */
	uint8_t cmd = START_CMD_DOWM;
	int ret = transfer(&cmd, 2, nullptr, 0);


	bool measure_ok = ret;

	if (OK != measure_ok) {
		perf_count(_comms_errors);
	}

	return measure_ok;
}

int Scd::collect()
{
	/* 读取数据 */
	uint8_t val[3] = {0, 0, 0};

	perf_begin(_sample_perf);
	const hrt_abstime timestamp_sample = hrt_absolute_time();
	int ret = transfer(nullptr, 0, &val[0], 3);

	if (ret < 0) {
		perf_count(_comms_errors);
		return ret;
	}

	scd_s scd{};
	scd.timestamp_sample = timestamp_sample;
	scd.device_id = get_device_id();

	for(int i=0; i<3; i++)
	{
		scd.data[i]=val[i];
	}

	scd.error_count = perf_event_count(_comms_errors);
	scd.timestamp = hrt_absolute_time();
	scd_pub.publish(scd);

	/*设置断点打印调试*/
	for(int loop=0; loop<9; loop++)
	{
		PX4_INFO("%d", scd.data[loop]);
	}


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
	ret = measure();

	if (OK != ret) {
		DEVICE_DEBUG("measure error");
	}

	_sensor_ok = (ret == OK);

	// next phase is collection
	_collect_phase = true;

	// schedule a fresh cycle call when the measurement is done
	ScheduleDelayed(CONVERSION_INTERVAL);
}
