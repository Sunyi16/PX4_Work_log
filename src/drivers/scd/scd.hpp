/**
 * 针对SCD_40的I2C驱动
 */

#pragma once

#include <float.h>

#include <drivers/drv_hrt.h>
#include <lib/drivers/device/i2c.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/scd.h>

static constexpr uint32_t I2C_SPEED = 100 * 1000; // 传输速度
static constexpr uint8_t I2C_ADDRESS_DEFAULT = 0x31; //设备地址

#define READ_CMD	uint8_t(0xec05)	//读数据之前要发送的命令

#define MIN_ACCURATE_DIFF_PRES_PA 0

/* Measurement rate is 100Hz */
#define CONVERSION_INTERVAL	(1000000 / 100)	/* microseconds */

class Scd : public device::I2C, public I2CSPIDriver<Scd>
{
public:
	Scd(const I2CSPIDriverConfig &config);
	~Scd() override;

	static void print_usage();

	void RunImpl();

	int init() override;

private:
	int probe() override;

	int measure();
	int collect();

	uint32_t _measure_interval{CONVERSION_INTERVAL};
	uint32_t _conversion_interval{CONVERSION_INTERVAL};

	bool _sensor_ok{false};
	bool _collect_phase{false};

	uORB::PublicationMulti<scd_s> scd_pub{ORB_ID(scd)};

	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": read")};
	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": communication errors")};
};
