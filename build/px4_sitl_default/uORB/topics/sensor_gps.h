/****************************************************************************
 *
 *   Copyright (C) 2013-2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/* Auto-generated by genmsg_cpp from file /home/sangam/PX4-Autopilot/msg/SensorGps.msg */


#pragma once


#include <uORB/uORB.h>


#ifndef __cplusplus
#define SENSOR_GPS_JAMMING_STATE_UNKNOWN 0
#define SENSOR_GPS_JAMMING_STATE_OK 1
#define SENSOR_GPS_JAMMING_STATE_WARNING 2
#define SENSOR_GPS_JAMMING_STATE_CRITICAL 3
#define SENSOR_GPS_SPOOFING_STATE_UNKNOWN 0
#define SENSOR_GPS_SPOOFING_STATE_NONE 1
#define SENSOR_GPS_SPOOFING_STATE_INDICATED 2
#define SENSOR_GPS_SPOOFING_STATE_MULTIPLE 3
#define SENSOR_GPS_RTCM_MSG_USED_UNKNOWN 0
#define SENSOR_GPS_RTCM_MSG_USED_NOT_USED 1
#define SENSOR_GPS_RTCM_MSG_USED_USED 2

#endif


#ifdef __cplusplus
struct __EXPORT sensor_gps_s {
#else
struct sensor_gps_s {
#endif
	uint64_t timestamp;
	uint64_t timestamp_sample;
	double latitude_deg;
	double longitude_deg;
	double altitude_msl_m;
	double altitude_ellipsoid_m;
	uint64_t time_utc_usec;
	uint32_t device_id;
	float s_variance_m_s;
	float c_variance_rad;
	float eph;
	float epv;
	float hdop;
	float vdop;
	int32_t noise_per_ms;
	int32_t jamming_indicator;
	float vel_m_s;
	float vel_n_m_s;
	float vel_e_m_s;
	float vel_d_m_s;
	float cog_rad;
	int32_t timestamp_time_relative;
	float heading;
	float heading_offset;
	float heading_accuracy;
	float rtcm_injection_rate;
	uint16_t automatic_gain_control;
	uint8_t fix_type;
	uint8_t jamming_state;
	uint8_t spoofing_state;
	bool vel_ned_valid;
	uint8_t satellites_used;
	uint8_t selected_rtcm_instance;
	bool rtcm_crc_failed;
	uint8_t rtcm_msg_used;
	uint8_t _padding0[2]; // required for logger


#ifdef __cplusplus
	static constexpr uint8_t JAMMING_STATE_UNKNOWN = 0;
	static constexpr uint8_t JAMMING_STATE_OK = 1;
	static constexpr uint8_t JAMMING_STATE_WARNING = 2;
	static constexpr uint8_t JAMMING_STATE_CRITICAL = 3;
	static constexpr uint8_t SPOOFING_STATE_UNKNOWN = 0;
	static constexpr uint8_t SPOOFING_STATE_NONE = 1;
	static constexpr uint8_t SPOOFING_STATE_INDICATED = 2;
	static constexpr uint8_t SPOOFING_STATE_MULTIPLE = 3;
	static constexpr uint8_t RTCM_MSG_USED_UNKNOWN = 0;
	static constexpr uint8_t RTCM_MSG_USED_NOT_USED = 1;
	static constexpr uint8_t RTCM_MSG_USED_USED = 2;

#endif
};

#ifdef __cplusplus
namespace px4 {
	namespace msg {
		using SensorGps = sensor_gps_s;
	} // namespace msg
} // namespace px4
#endif

/* register this as object request broker structure */
ORB_DECLARE(sensor_gps);
ORB_DECLARE(vehicle_gps_position);


#ifdef __cplusplus
void print_message(const orb_metadata *meta, const sensor_gps_s& message);
#endif
