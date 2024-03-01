/**
 * 针对MS5837-30BA的I2C驱动，用于UUV的水深测量,对应话题是scd
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
static constexpr uint8_t I2C_ADDRESS_DEFAULT = 0xec; //设备地址


#define P_CMD		0x48	//压力数据转换
#define T_CMD		0x58	//温度数据转换
#define READ_CMD	0x00	//ADC读取命令
#define RESET_CMD	0x1E	//重置命令
#define PROM_CMD1	0xA2	//读取标定信息C1
#define PROM_CMD2	0xA4	//读取标定信息C2
#define PROM_CMD3	0xA6	//读取标定信息C3
#define PROM_CMD4	0xA8	//读取标定信息C4
#define PROM_CMD5	0xAA	//读取标定信息C5
#define PROM_CMD6	0xAC	//读取标定信息C6

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

	int zhuanhuan(uint8_t A[], int n);

	void measure(uint8_t cmd);
	int collect();

	uint8_t C1[2];  	//C1
	uint8_t C2[2];	//C2
	uint8_t C3[2];
	uint8_t C4[2];
	uint8_t C5[2];
	uint8_t C6[2];

	int16_t vC1;
	int16_t vC2;
	int16_t vC3;
	int16_t vC4;
	int16_t vC5;
	int16_t vC6;

	scd_s scd;

	uint32_t _measure_interval{CONVERSION_INTERVAL};
	uint32_t _conversion_interval{CONVERSION_INTERVAL};

	bool _sensor_ok{false};
	bool _collect_phase{false};

	uORB::PublicationMulti<scd_s> scd_pub{ORB_ID(scd)};

	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": read")};
	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": communication errors")};
};
