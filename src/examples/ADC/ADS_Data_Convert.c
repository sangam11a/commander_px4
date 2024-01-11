#include <examples/hk_data/ADS_Data_Convert.h>
#include <px4_platform_common/log.h>

struct satellite_health_s raw;	//to publish external adc data
struct adc_report_s int_adc_data;	//to subscribe internal adc data

/*

*/

/*
* @brief		to convert ADC 0 data to temperature data
*
* @param		adc_conv_buf		pointer to the raw data to be converted
*		        temp_buf		converted data buffer pointer
*/
void ADC_Temp_Conv(float *temp_buf)
{
	float root = 0;
	struct adc_report_s adc_conv_buf;

	int adc1_data_fd = orb_subscribe(ORB_ID(external_adc));	//subscribing external adc data
	orb_copy(ORB_ID(external_adc), adc1_data_fd, &adc_conv_buf);

	for (int i = 0; i < 12; i++) {

		if (i == 6) {		// Battery temperature channel
			float res = (double)(adc_conv_buf.raw_data_ext1[i] * 10000) / (2.5 - (double)adc_conv_buf.raw_data_ext1[i]);
			float tempK = (3976 * 298) / (3976 - (298 * log(10000 / res)));
			temp_buf[i] = (tempK - 273) * 100;

		} else {
			root =
				sqrtf(
					(5.506 * 5.506)
					+ (4 * 0.00176
					   * (870.6 - (double)(adc_conv_buf.raw_data_ext1[i] * 1000))));
			temp_buf[i] = ((((5.506 - (double)root) / (2 * (-0.00176))) + 30)) * 100;
		}
	}
}

/*
* @brief        	function to convert ADC1 data
*	         	data is seggregated and different formula is used for respective data
*
* @param		adc1_conv_buf	converted data
*/
void ADC1_Conv_Data(float *adc1_conv_buf)
{
	float root;
	struct adc_report_s adc1_buf;

	int adc1_data_fd = orb_subscribe(ORB_ID(external_adc));	//subscribing external adc data
	orb_copy(ORB_ID(external_adc), adc1_data_fd, &adc1_buf);

	//todo: figure out how to run both ADCs and store the data and edit the raw_data_ext buffer as required
	for (int x = 0; x < 12; x++) {
		switch (x) {
		case 8:		// Solar Panel Total Current
			adc1_conv_buf[x] = (double)(((double)adc1_buf.raw_data_ext2[x] - 1.65) / 264) *
					   1000;  //converting voltage to current. In actual code need to keep separate variable for current
			break;

		case 9:		// Battery current
			adc1_conv_buf[x] = (double)(((double)adc1_buf.raw_data_ext2[x] - 1.65) / 264) *
					   1000; //converting voltage to current. In actual code need to keep separate variable for current

			break;

		case 10:  // COM Board Temp data
			root = sqrtf(
				       (5.506 * 5.506) + (4 * 0.00176 * (870.6 - (double)(adc1_buf.raw_data_ext2[x] * 1000))));
			adc1_conv_buf[x] = ((5.506 - (double)root) / (2 * (-0.00176))) + 30;
			break;

		case 11:	// COM RSSI analog data
			adc1_conv_buf[x] = adc1_buf.raw_data_ext2[x];
			break;

		default:		// baki sabai
			adc1_conv_buf[x] = (double)((double)adc1_buf.raw_data_ext2[x] * (double)(1100.0 + 931.0)) / 931.0;
			break;
		}
	}
}

/*
* @brief	       function to convert the adc 0 data to respective temperature data
*
* @param	       ADC 0 data (after the data is processed and converted to ADC data)
*
* @retval	       none
*/
void Make_Satellite_Health(void)
{
	float Int_ADC_Data[12];
	float IMU_Data[9];
	float EXT_ADC_0_TEMP[12];
	float EXT_ADC_1[12];
	orb_advert_t att_pub_fd = orb_advertise(ORB_ID(satellite_health), &raw);
	raw.timestamp = hrt_absolute_time();

	/*Internal adc data is obtained in the int_adc_data variable (order of data: channel 0 - 11)*/
	Get_Internal_ADC_Data(Int_ADC_Data);
	Get_IMU_Data(IMU_Data);

// #ifdef ADC_0_MODE
	ADC_Temp_Conv(EXT_ADC_0_TEMP);
// #else
	ADC1_Conv_Data(EXT_ADC_1);
// #endif

	/*Temperature data collected from External ADC1*/
	raw.temp_batt = ((int16_t) EXT_ADC_0_TEMP[6]);
	raw.temp_y1 = ((int16_t) EXT_ADC_0_TEMP[0]);
	raw.temp_y = ((int16_t) EXT_ADC_0_TEMP[3]);
	raw.temp_z1 = ((int16_t) EXT_ADC_0_TEMP[1]);
	raw.temp_z = ((int16_t) EXT_ADC_0_TEMP[4]);
	raw.temp_x1 = ((int16_t) EXT_ADC_0_TEMP[2]);
	raw.temp_x = ((int16_t) EXT_ADC_0_TEMP[7]);
	raw.temp_bpb = ((int16_t) EXT_ADC_0_TEMP[5]);

	/*Solar Panel voltage(s), battery voltage, and solar panel and battery current from External ADC2*/
	raw.sol_p1_v = (int16_t)(EXT_ADC_1[1] * 1000);
	raw.sol_p2_v = (int16_t)(EXT_ADC_1[0] * 1000);
	raw.sol_p3_v = (int16_t)(EXT_ADC_1[2] * 1000);
	raw.sol_p4_v = (int16_t)(EXT_ADC_1[3] * 1000);
	raw.sol_p5_v = (int16_t)(EXT_ADC_1[4] * 1000);
	raw.sol_t_v = (int16_t)(EXT_ADC_1[6] * 1000);
	raw.batt_volt = (uint16_t)(EXT_ADC_1[7] * 1000);
	raw.temp_com = ((int16_t)EXT_ADC_1[10]);
	raw.sol_t_c = (int16_t)(EXT_ADC_1[8] * 1000);
	raw.batt_c = (int16_t)(EXT_ADC_1[9] * 1000);

	/*Internal ADC sensor data (data is collected calling the function: Get_Internal_ADC_Data())*/
	raw.rst_3v3_c = (int16_t)(Int_ADC_Data[4]);
	raw.sol_p1_c = (int16_t)(Int_ADC_Data[0]);
	raw.sol_p2_c = (int16_t)(Int_ADC_Data[1]);
	raw.sol_p3_c = (int16_t)(Int_ADC_Data[2]);
	raw.sol_p4_c = (int16_t)(Int_ADC_Data[3]);
	raw.sol_p5_c = (int16_t)(Int_ADC_Data[6]);
	raw.temp_obc = (int16_t)(Int_ADC_Data[5]);

	/*random data assign for now*/
	raw.v3_1_c = 0xffff;	//enter actual data from internal adc
	raw.v3_2_c = 0xffff;	//internal adc data dummy
	raw.v5_c = 0xffff;	//internal adc data dummy
	raw.unreg1_c = 0xffff;	//internal adc data dummy
	raw.unreg2_c = 0xffff;	//internal adc data dummy
	raw.raw_c = 0xffff;	//internal adc data dummy

	raw.gyro_x = (IMU_Data[0]);
	raw.gyro_y = (IMU_Data[1]);
	raw.gyro_z = (IMU_Data[2]);

	raw.accl_x = (IMU_Data[3]);
	raw.accl_y = (IMU_Data[4]);
	raw.accl_z = (IMU_Data[5]);

	raw.mag_x = (IMU_Data[6]);
	raw.mag_y = (IMU_Data[7]);
	raw.mag_z = (IMU_Data[8]);

	PX4_INFO("Data Published.....");

	orb_publish(ORB_ID(satellite_health), att_pub_fd, &raw);
}

//

/*
* @brief	to get internal ADC data and assign to the structure
* @param	data  --  pointer to data buffer
*			  determine the order in which data is received and which data is received
*/
void Get_Internal_ADC_Data(float *data){
	int internal_adc_report_fd = orb_subscribe(ORB_ID(adc_report));
	orb_copy(ORB_ID(adc_report), internal_adc_report_fd, &int_adc_data);
	float temp_buff[12];
	int ADC_SUP = 1.2 * 4095/(int_adc_data.raw_data[6]);
	for(int i=0;i<12;i++){
		temp_buff[i] = (float)int_adc_data.raw_data[i];
		temp_buff[i] = (float)ADC_SUP * temp_buff[i] / 4095;	//finding true supply to MCU
	}
	//todo: add the formula to convert the data here
	// dummy formula for now
	for(int i=0;i<12;i++){
		if(i==5){
			data[i] = temp_buff[i];
		}else{
			data[i] = (((double)temp_buff[i] - 1.65)/264)*1000*1000;
		}
	}
}

/*
* @brief	to get the IMU data (gyroscope and accelerometer)
*
*@param		data -- pointer to the data that is received from the gyro and accelerometer
*		first 3 bytes: gyro data x,y,z
*		second 3 bytes: accelerometer data x,y,z
*/
void Get_IMU_Data(float *data){
	int imu_data_fd = orb_subscribe(ORB_ID(sensor_combined));	//subscribing gyro data
	int mag_data_fd = orb_subscribe(ORB_ID(sensor_mag));

	struct sensor_combined_s imu_data;
	struct sensor_mag_s mag_data;

	orb_copy(ORB_ID(sensor_combined), imu_data_fd, &imu_data);
	orb_copy(ORB_ID(sensor_mag), mag_data_fd, &mag_data);

	data[0] = imu_data.gyro_rad[0];
	data[1] = imu_data.gyro_rad[1];
	data[2] = imu_data.gyro_rad[2];

	//todo: do the same for gyro and accelerometer data as the magnetometer
	data[3] = imu_data.accelerometer_m_s2[0];
	data[4] = imu_data.accelerometer_m_s2[1];
	data[5] = imu_data.accelerometer_m_s2[2];

	//todo: add a if statement to check if the device id is being read, then only assign data to the buffer else put certain value in the buffer
	data[6] = mag_data.x;
	data[7] = mag_data.y;
	data[8] = mag_data.z;
}

/*
* @brief	put data into buffer to send to GS
*
* @param	data -- after packing and ordering
*
*/
void Make_HK_Data(uint8_t *data){
	int hk_data_fd = orb_subscribe(ORB_ID(satellite_health));
	struct satellite_health_s sat_data;
	orb_copy(ORB_ID(satellite_health), hk_data_fd, &sat_data);
	memcpy(data, &sat_data, sizeof(sat_data));
}





