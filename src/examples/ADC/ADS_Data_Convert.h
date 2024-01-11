

#pragma once
#include <math.h>
#include <uORB/topics/satellite_health.h>
#include <uORB/topics/adc_report.h>
#include <drivers/drv_hrt.h>
#include <string.h>
#include <px4_platform_common/log.h>

#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/sensor_mag.h>

void ADC1_Conv_Data(float *adc1_conv_buf);
void ADC_Temp_Conv(float *temp_buf);

void Make_Satellite_Health(void);

void Get_Internal_ADC_Data(float *data);
void Get_IMU_Data(float *data);
void Make_HK_Data(uint8_t *data);


