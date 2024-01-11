/****************************************************************************
 *
 *   Copyright (c) 2013-2023 PX4 Development Team. All rights reserved.
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

/**
 * @file commander.cpp
 *
 * Main state machine / business logic
 *
 */

#include "Commander.hpp"

#define DEFINE_GET_PX4_CUSTOM_MODE
#include "px4_custom_mode.h"
// #include "ModeUtil/control_mode.hpp"
#include "ModeUtil/conversions.hpp"
#include <lib/modes/ui.hpp>
#include <lib/modes/standard_modes.hpp>

/* PX4 headers */
#include <drivers/drv_hrt.h>
// #include <drivers/drv_tone_alarm.h>
#include <lib/geo/geo.h>
#include <mathlib/mathlib.h>
#include <px4_platform_common/events.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/external_reset_lockout.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/shutdown.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/time.h>
#include <systemlib/mavlink_log.h>

#include <math.h>
#include <float.h>
#include <cstring>
#include <matrix/math.hpp>

#include <uORB/topics/mavlink_log.h>
// #include <uORB/topics/tune_control.h>

typedef enum VEHICLE_MODE_FLAG {
	VEHICLE_MODE_FLAG_CUSTOM_MODE_ENABLED  = 1,   /* 0b00000001 Reserved for future use. | */
	VEHICLE_MODE_FLAG_TEST_ENABLED         = 2,   /* 0b00000010 system has a test mode enabled. This flag is intended for temporary system tests and should not be used for stable implementations. | */
	VEHICLE_MODE_FLAG_AUTO_ENABLED         = 4,   /* 0b00000100 autonomous mode enabled, system finds its own goal positions. Guided flag can be set or not, depends on the actual implementation. | */
	VEHICLE_MODE_FLAG_GUIDED_ENABLED       = 8,   /* 0b00001000 guided mode enabled, system flies MISSIONs / mission items. | */
	VEHICLE_MODE_FLAG_STABILIZE_ENABLED    = 16,  /* 0b00010000 system stabilizes electronically its attitude (and optionally position). It needs however further control inputs to move around. | */
	VEHICLE_MODE_FLAG_HIL_ENABLED          = 32,  /* 0b00100000 hardware in the loop simulation. All motors / actuators are blocked, but internal software is full operational. | */
	VEHICLE_MODE_FLAG_MANUAL_INPUT_ENABLED = 64,  /* 0b01000000 remote control input is enabled. | */
	VEHICLE_MODE_FLAG_SAFETY_ARMED         = 128, /* 0b10000000 MAV safety set to armed. Motors are enabled / running / can start. Ready to fly. Additional note: this flag is to be ignore when sent in the command MAV_CMD_DO_SET_MODE and MAV_CMD_COMPONENT_ARM_DISARM shall be used instead. The flag can still be used to report the armed state. | */
	VEHICLE_MODE_FLAG_ENUM_END             = 129, /*  | */
} VEHICLE_MODE_FLAG;

// TODO: generate
static constexpr bool operator ==(const actuator_armed_s &a, const actuator_armed_s &b)
{
	return (a.armed == b.armed &&
		a.prearmed == b.prearmed &&
		a.ready_to_arm == b.ready_to_arm &&
		a.lockdown == b.lockdown &&
		a.manual_lockdown == b.manual_lockdown &&
		a.force_failsafe == b.force_failsafe &&
		a.in_esc_calibration_mode == b.in_esc_calibration_mode);
}
static_assert(sizeof(actuator_armed_s) == 16, "actuator_armed equality operator review");


#ifndef CONSTRAINED_FLASH
static bool send_vehicle_command(const uint32_t cmd, const float param1 = NAN, const float param2 = NAN,
				 const float param3 = NAN,  const float param4 = NAN, const double param5 = static_cast<double>(NAN),
				 const double param6 = static_cast<double>(NAN), const float param7 = NAN)
{
	vehicle_command_s vcmd{};
	vcmd.command = cmd;
	vcmd.param1 = param1;
	vcmd.param2 = param2;
	vcmd.param3 = param3;
	vcmd.param4 = param4;
	vcmd.param5 = param5;
	vcmd.param6 = param6;
	vcmd.param7 = param7;

	uORB::SubscriptionData<vehicle_status_s> vehicle_status_sub{ORB_ID(vehicle_status)};
	vcmd.source_system = vehicle_status_sub.get().system_id;
	vcmd.target_system = vehicle_status_sub.get().system_id;
	vcmd.source_component = vehicle_status_sub.get().component_id;
	vcmd.target_component = vehicle_status_sub.get().component_id;

	uORB::Publication<vehicle_command_s> vcmd_pub{ORB_ID(vehicle_command)};
	vcmd.timestamp = hrt_absolute_time();
	return vcmd_pub.publish(vcmd);
}

static bool wait_for_vehicle_command_reply(const uint32_t cmd,
		uORB::SubscriptionData<vehicle_command_ack_s> &vehicle_command_ack_sub)
{
	hrt_abstime start = hrt_absolute_time();

	while (hrt_absolute_time() - start < 100_ms) {
		if (vehicle_command_ack_sub.update()) {
			if (vehicle_command_ack_sub.get().command == cmd) {
				return vehicle_command_ack_sub.get().result == vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;
			}
		}

		px4_usleep(10000);
	}

	return false;
}

static bool broadcast_vehicle_command(const uint32_t cmd, const float param1 = NAN, const float param2 = NAN,
				      const float param3 = NAN,  const float param4 = NAN, const double param5 = static_cast<double>(NAN),
				      const double param6 = static_cast<double>(NAN), const float param7 = NAN)
{
	vehicle_command_s vcmd{};
	vcmd.command = cmd;
	vcmd.param1 = param1;
	vcmd.param2 = param2;
	vcmd.param3 = param3;
	vcmd.param4 = param4;
	vcmd.param5 = param5;
	vcmd.param6 = param6;
	vcmd.param7 = param7;

	uORB::SubscriptionData<vehicle_status_s> vehicle_status_sub{ORB_ID(vehicle_status)};
	vcmd.source_system = vehicle_status_sub.get().system_id;
	vcmd.target_system = 0;
	vcmd.source_component = vehicle_status_sub.get().component_id;
	vcmd.target_component = 0;

	uORB::Publication<vehicle_command_s> vcmd_pub{ORB_ID(vehicle_command)};
	vcmd.timestamp = hrt_absolute_time();
	return vcmd_pub.publish(vcmd);
}
#endif

int Commander::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
		print_usage("not running");
		return 1;
	}

#ifndef CONSTRAINED_FLASH

	if (!strcmp(argv[0], "calibrate")) {
		if (argc > 1) {
			if (!strcmp(argv[1], "gyro")) {
				// gyro calibration: param1 = 1
				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_PREFLIGHT_CALIBRATION, 1.f, 0.f, 0.f, 0.f, 0.0, 0.0, 0.f);

			} else if (!strcmp(argv[1], "mag")) {
				if (argc > 2 && (strcmp(argv[2], "quick") == 0)) {
					// magnetometer quick calibration: VEHICLE_CMD_FIXED_MAG_CAL_YAW
					send_vehicle_command(vehicle_command_s::VEHICLE_CMD_FIXED_MAG_CAL_YAW);

				} else {
					// magnetometer calibration: param2 = 1
					send_vehicle_command(vehicle_command_s::VEHICLE_CMD_PREFLIGHT_CALIBRATION, 0.f, 1.f, 0.f, 0.f, 0.0, 0.0, 0.f);
				}

			} else if (!strcmp(argv[1], "baro")) {
				// baro calibration: param3 = 1
				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_PREFLIGHT_CALIBRATION, 0.f, 0.f, 1.f, 0.f, 0.0, 0.0, 0.f);

			} else if (!strcmp(argv[1], "accel")) {
				if (argc > 2 && (strcmp(argv[2], "quick") == 0)) {
					// accelerometer quick calibration: param5 = 3
					send_vehicle_command(vehicle_command_s::VEHICLE_CMD_PREFLIGHT_CALIBRATION, 0.f, 0.f, 0.f, 0.f, 4.0, 0.0, 0.f);

				} else {
					// accelerometer calibration: param5 = 1
					send_vehicle_command(vehicle_command_s::VEHICLE_CMD_PREFLIGHT_CALIBRATION, 0.f, 0.f, 0.f, 0.f, 1.0, 0.0, 0.f);
				}

			}
			// else if (!strcmp(argv[1], "level")) {
			// 	// board level calibration: param5 = 2
			// 	send_vehicle_command(vehicle_command_s::VEHICLE_CMD_PREFLIGHT_CALIBRATION, 0.f, 0.f, 0.f, 0.f, 2.0, 0.0, 0.f);

			// } else if (!strcmp(argv[1], "airspeed")) {
			// 	// airspeed calibration: param6 = 2
			// 	send_vehicle_command(vehicle_command_s::VEHICLE_CMD_PREFLIGHT_CALIBRATION, 0.f, 0.f, 0.f, 0.f, 0.0, 2.0, 0.f);

			// } else if (!strcmp(argv[1], "esc")) {
			// 	// ESC calibration: param7 = 1
			// 	send_vehicle_command(vehicle_command_s::VEHICLE_CMD_PREFLIGHT_CALIBRATION, 0.f, 0.f, 0.f, 0.f, 0.0, 0.0, 1.f);

			// }
			else {
				PX4_ERR("argument %s unsupported.", argv[1]);
				return 1;
			}

			return 0;

		} else {
			PX4_ERR("missing argument");
		}
	}



	if (!strcmp(argv[0], "takeoff")) {
		// switch to takeoff mode and arm
		uORB::SubscriptionData<vehicle_command_ack_s> vehicle_command_ack_sub{ORB_ID(vehicle_command_ack)};
		send_vehicle_command(vehicle_command_s::VEHICLE_CMD_NAV_TAKEOFF);

		if (wait_for_vehicle_command_reply(vehicle_command_s::VEHICLE_CMD_NAV_TAKEOFF, vehicle_command_ack_sub)) {
			send_vehicle_command(vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM,
					     static_cast<float>(vehicle_command_s::ARMING_ACTION_ARM),
					     0.f);
		}

		return 0;
	}

	if (!strcmp(argv[0], "land")) {
		send_vehicle_command(vehicle_command_s::VEHICLE_CMD_NAV_LAND);

		return 0;
	}

	// if (!strcmp(argv[0], "transition")) {
	// 	uORB::Subscription vehicle_status_sub{ORB_ID(vehicle_status)};
	// 	vehicle_status_s vehicle_status{};
	// 	vehicle_status_sub.copy(&vehicle_status);
	// 	send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION,
	// 			     (float)(vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING ?
	// 				     vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW :
	// 				     vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC), 0.0f);

	// 	return 0;
	// }

	// if (!strcmp(argv[0], "mode")) {
	// 	if (argc > 1) {

	// 		if (!strcmp(argv[1], "manual")) {
	// 			send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_MANUAL);

	// 		} else if (!strcmp(argv[1], "altctl")) {
	// 			send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_ALTCTL);

	// 		} else if (!strcmp(argv[1], "posctl")) {
	// 			send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_POSCTL);

	// 		} else if (!strcmp(argv[1], "position:slow")) {
	// 			send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_POSCTL,
	// 					     PX4_CUSTOM_SUB_MODE_POSCTL_SLOW);

	// 		} else if (!strcmp(argv[1], "auto:mission")) {
	// 			send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_AUTO,
	// 					     PX4_CUSTOM_SUB_MODE_AUTO_MISSION);

	// 		} else if (!strcmp(argv[1], "auto:loiter")) {
	// 			send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_AUTO,
	// 					     PX4_CUSTOM_SUB_MODE_AUTO_LOITER);

	// 		} else if (!strcmp(argv[1], "auto:rtl")) {
	// 			send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_AUTO,
	// 					     PX4_CUSTOM_SUB_MODE_AUTO_RTL);

	// 		} else if (!strcmp(argv[1], "acro")) {
	// 			send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_ACRO);

	// 		} else if (!strcmp(argv[1], "offboard")) {
	// 			send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_OFFBOARD);

	// 		} else if (!strcmp(argv[1], "stabilized")) {
	// 			send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_STABILIZED);

	// 		} else if (!strcmp(argv[1], "auto:takeoff")) {
	// 			send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_AUTO,
	// 					     PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF);

	// 		} else if (!strcmp(argv[1], "auto:land")) {
	// 			send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_AUTO,
	// 					     PX4_CUSTOM_SUB_MODE_AUTO_LAND);

	// 		} else if (!strcmp(argv[1], "auto:precland")) {
	// 			send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_AUTO,
	// 					     PX4_CUSTOM_SUB_MODE_AUTO_PRECLAND);

	// 		} else if (!strcmp(argv[1], "ext1")) {
	// 			send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_AUTO,
	// 					     PX4_CUSTOM_SUB_MODE_EXTERNAL1);

	// 		} else {
	// 			PX4_ERR("argument %s unsupported.", argv[1]);
	// 		}

	// 		return 0;

	// 	} else {
	// 		PX4_ERR("missing argument");
	// 	}
	// }

	// if (!strcmp(argv[0], "lockdown")) {

	// 	if (argc < 2) {
	// 		Commander::print_usage("not enough arguments, missing [on, off]");
	// 		return 1;
	// 	}

	// 	bool ret = send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_FLIGHTTERMINATION,
	// 					strcmp(argv[1], "off") ? 2.0f : 0.0f /* lockdown */, 0.0f);

	// 	return (ret ? 0 : 1);
	// }

	if (!strcmp(argv[0], "pair")) {

		// GCS pairing request handled by a companion
		bool ret = broadcast_vehicle_command(vehicle_command_s::VEHICLE_CMD_START_RX_PAIR, 10.f);

		return (ret ? 0 : 1);
	}

	// if (!strcmp(argv[0], "set_ekf_origin")) {
	// 	if (argc > 3) {

	// 		double latitude  = atof(argv[1]);
	// 		double longitude = atof(argv[2]);
	// 		float  altitude  = atof(argv[3]);

	// 		// Set the ekf NED origin global coordinates.
	// 		bool ret = send_vehicle_command(vehicle_command_s::VEHICLE_CMD_SET_GPS_GLOBAL_ORIGIN,
	// 						0.f, 0.f, 0.0, 0.0, latitude, longitude, altitude);
	// 		return (ret ? 0 : 1);

	// 	} else {
	// 		PX4_ERR("missing argument");
	// 		return 0;
	// 	}
	// }

	// if (!strcmp(argv[0], "poweroff")) {

	// 	bool ret = send_vehicle_command(vehicle_command_s::VEHICLE_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
	// 					2.0f);

	// 	return (ret ? 0 : 1);
	// }


#endif

	return print_usage("unknown command");
}

int Commander::print_status()
{
	PX4_INFO("%s", isArmed() ? "Armed" : "Disarmed");
	PX4_INFO("navigation mode: %s", mode_util::nav_state_names[_vehicle_status.nav_state]);
	PX4_INFO("user intended navigation mode: %s", mode_util::nav_state_names[_vehicle_status.nav_state_user_intention]);
	PX4_INFO("in failsafe: %s", _failsafe.inFailsafe() ? "yes" : "no");
	_mode_management.printStatus();
	perf_print_counter(_loop_perf);
	perf_print_counter(_preflight_check_perf);
	return 0;
}

extern "C" __EXPORT int commander_main(int argc, char *argv[])
{
	return Commander::main(argc, argv);
}

static constexpr const char *arm_disarm_reason_str(arm_disarm_reason_t calling_reason)
{
	switch (calling_reason) {
	case arm_disarm_reason_t::transition_to_standby: return "";

	case arm_disarm_reason_t::rc_stick: return "RC";

	case arm_disarm_reason_t::rc_switch: return "RC (switch)";

	case arm_disarm_reason_t::command_internal: return "internal command";

	case arm_disarm_reason_t::command_external: return "external command";

	case arm_disarm_reason_t::mission_start: return "mission start";

	case arm_disarm_reason_t::auto_disarm_land: return "landing";

	case arm_disarm_reason_t::auto_disarm_preflight: return "auto preflight disarming";

	case arm_disarm_reason_t::kill_switch: return "kill-switch";

	case arm_disarm_reason_t::lockdown: return "lockdown";

	case arm_disarm_reason_t::failure_detector: return "failure detector";

	case arm_disarm_reason_t::shutdown: return "shutdown request";

	case arm_disarm_reason_t::unit_test: return "unit tests";

	case arm_disarm_reason_t::rc_button: return "RC (button)";

	case arm_disarm_reason_t::failsafe: return "failsafe";
	}

	return "";
};

transition_result_t Commander::arm(arm_disarm_reason_t calling_reason, bool run_preflight_checks)
{
	if (isArmed()) {
		return TRANSITION_NOT_CHANGED;
	}

	if (_vehicle_status.calibration_enabled
	    || _vehicle_status.rc_calibration_in_progress
	    || _actuator_armed.in_esc_calibration_mode) {

		mavlink_log_critical(&_mavlink_log_pub, "Arming denied: calibrating\t");
		events::send(events::ID("commander_arm_denied_calibrating"), {events::Log::Critical, events::LogInternal::Info},
			     "Arming denied: calibrating");
		// tune_negative(true);
		return TRANSITION_DENIED;
	}

	// allow a grace period for re-arming: preflight checks don't need to pass during that time, for example for accidental in-air disarming
	if (calling_reason == arm_disarm_reason_t::rc_switch
	    && ((_last_disarmed_timestamp != 0) && (hrt_elapsed_time(&_last_disarmed_timestamp) < 5_s))) {

		run_preflight_checks = false;
	}

	if (run_preflight_checks) {
		if (_vehicle_control_mode.flag_control_manual_enabled) {

			if (_vehicle_control_mode.flag_control_climb_rate_enabled &&
			    !_failsafe_flags.manual_control_signal_lost && _is_throttle_above_center) {

				mavlink_log_critical(&_mavlink_log_pub, "Arming denied: throttle above center\t");
				events::send(events::ID("commander_arm_denied_throttle_center"), {events::Log::Critical, events::LogInternal::Info},
					     "Arming denied: throttle above center");
				//tune_negative(true);
				return TRANSITION_DENIED;
			}

			if (!_vehicle_control_mode.flag_control_climb_rate_enabled &&
			    !_failsafe_flags.manual_control_signal_lost && !_is_throttle_low
			    && _vehicle_status.vehicle_type != vehicle_status_s::VEHICLE_TYPE_ROVER) {

				mavlink_log_critical(&_mavlink_log_pub, "Arming denied: high throttle\t");
				events::send(events::ID("commander_arm_denied_throttle_high"), {events::Log::Critical, events::LogInternal::Info},
					     "Arming denied: high throttle");
				//tune_negative(true);
				return TRANSITION_DENIED;
			}

		} else if (calling_reason == arm_disarm_reason_t::rc_stick
			   || calling_reason == arm_disarm_reason_t::rc_switch
			   || calling_reason == arm_disarm_reason_t::rc_button) {

			mavlink_log_critical(&_mavlink_log_pub, "Arming denied: switch to manual mode first\t");
			events::send(events::ID("commander_arm_denied_not_manual"), {events::Log::Critical, events::LogInternal::Info},
				     "Arming denied: switch to manual mode first");
			//tune_negative(true);
			return TRANSITION_DENIED;
		}

		_health_and_arming_checks.update();

		if (!_health_and_arming_checks.canArm(_vehicle_status.nav_state)) {
			//tune_negative(true);
			return TRANSITION_DENIED;
		}
	}

	_vehicle_status.armed_time = hrt_absolute_time();
	_vehicle_status.arming_state = vehicle_status_s::ARMING_STATE_ARMED;
	_vehicle_status.latest_arming_reason = (uint8_t)calling_reason;

	mavlink_log_info(&_mavlink_log_pub, "Armed by %s\t", arm_disarm_reason_str(calling_reason));
	events::send<events::px4::enums::arm_disarm_reason_t>(events::ID("commander_armed_by"), events::Log::Info,
			"Armed by {1}", calling_reason);

	if (_param_com_home_en.get()) {
		_home_position.setHomePosition();
	}

	_status_changed = true;

	return TRANSITION_CHANGED;
}

transition_result_t Commander::disarm(arm_disarm_reason_t calling_reason, bool forced)
{
	if (!isArmed()) {
		return TRANSITION_NOT_CHANGED;
	}

	if (!forced) {
		const bool landed = (_vehicle_land_detected.landed || _vehicle_land_detected.maybe_landed
				      );
		const bool mc_manual_thrust_mode = _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
						   && _vehicle_control_mode.flag_control_manual_enabled
						   && !_vehicle_control_mode.flag_control_climb_rate_enabled;
		const bool commanded_by_rc = (calling_reason == arm_disarm_reason_t::rc_stick)
					     || (calling_reason == arm_disarm_reason_t::rc_switch)
					     || (calling_reason == arm_disarm_reason_t::rc_button);

		if (!landed && !(mc_manual_thrust_mode && commanded_by_rc && _param_com_disarm_man.get())) {
			if (calling_reason != arm_disarm_reason_t::rc_stick) {
				mavlink_log_critical(&_mavlink_log_pub, "Disarming denied: not landed\t");
				events::send(events::ID("commander_disarm_denied_not_landed"),
				{events::Log::Critical, events::LogInternal::Info},
				"Disarming denied: not landed");
			}

			return TRANSITION_DENIED;
		}
	}

	_vehicle_status.armed_time = 0;
	_vehicle_status.arming_state = vehicle_status_s::ARMING_STATE_DISARMED;
	_vehicle_status.latest_disarming_reason = (uint8_t)calling_reason;
	_vehicle_status.takeoff_time = 0;

	_have_taken_off_since_arming = false;

	_last_disarmed_timestamp = hrt_absolute_time();

	_user_mode_intention.onDisarm();

	mavlink_log_info(&_mavlink_log_pub, "Disarmed by %s\t", arm_disarm_reason_str(calling_reason));
	events::send<events::px4::enums::arm_disarm_reason_t>(events::ID("commander_disarmed_by"), events::Log::Info,
			"Disarmed by {1}", calling_reason);

	if (_param_com_force_safety.get()) {
		_safety.activateSafety();
	}

	// update flight uuid
	const int32_t flight_uuid = _param_flight_uuid.get() + 1;
	_param_flight_uuid.set(flight_uuid);
	_param_flight_uuid.commit_no_notification();

	_status_changed = true;

	return TRANSITION_CHANGED;
}

Commander::Commander() :
	ModuleParams(nullptr)
{
	_vehicle_land_detected.landed = true;

	_vehicle_status.arming_state = vehicle_status_s::ARMING_STATE_DISARMED;
	_vehicle_status.system_id = 1;
	_vehicle_status.component_id = 1;
	_vehicle_status.system_type = 0;
	_vehicle_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_UNKNOWN;
	_vehicle_status.nav_state = _user_mode_intention.get();
	_vehicle_status.nav_state_user_intention = _user_mode_intention.get();
	_vehicle_status.nav_state_timestamp = hrt_absolute_time();
	_vehicle_status.gcs_connection_lost = true;
	// _vehicle_status.power_input_valid = true;

	// default for vtol is rotary wing
	_vtol_vehicle_status.vehicle_vtol_state = vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC;

	_param_mav_comp_id = param_find("MAV_COMP_ID");
	_param_mav_sys_id = param_find("MAV_SYS_ID");
	_param_mav_type = param_find("MAV_TYPE");
	_param_rc_map_fltmode = param_find("RC_MAP_FLTMODE");

	updateParameters();
}

Commander::~Commander()
{
	perf_free(_loop_perf);
	perf_free(_preflight_check_perf);
}

bool
Commander::handle_command(const vehicle_command_s &cmd)
{
	/* only handle commands that are meant to be handled by this system and component, or broadcast */
	if (((cmd.target_system != _vehicle_status.system_id) && (cmd.target_system != 0))
	    || ((cmd.target_component != _vehicle_status.component_id) && (cmd.target_component != 0))) {
		return false;
	}

	/* result of the command */
	unsigned cmd_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_UNSUPPORTED;

	/* request to set different system mode */
	switch (cmd.command) {

	case vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM: {

			// Adhere to MAVLink specs, but base on knowledge that these fundamentally encode ints
			// for logic state parameters
			const int8_t arming_action = static_cast<int8_t>(lroundf(cmd.param1));

			if (arming_action != vehicle_command_s::ARMING_ACTION_ARM
			    && arming_action != vehicle_command_s::ARMING_ACTION_DISARM) {
				mavlink_log_critical(&_mavlink_log_pub, "Unsupported ARM_DISARM param: %.3f\t", (double)cmd.param1);
				events::send<float>(events::ID("commander_unsupported_arm_disarm_param"), events::Log::Error,
						    "Unsupported ARM_DISARM param: {1:.3}", cmd.param1);

			} else {
				// Arm is forced (checks skipped) when param2 is set to a magic number.
				const bool forced = (static_cast<int>(lroundf(cmd.param2)) == 21196);

				transition_result_t arming_res = TRANSITION_DENIED;
				arm_disarm_reason_t arm_disarm_reason = cmd.from_external ? arm_disarm_reason_t::command_external :
									arm_disarm_reason_t::command_internal;

				if (arming_action == vehicle_command_s::ARMING_ACTION_ARM) {
					arming_res = arm(arm_disarm_reason, cmd.from_external || !forced);

				} else if (arming_action == vehicle_command_s::ARMING_ACTION_DISARM) {
					arming_res = disarm(arm_disarm_reason, forced);

				}

				if (arming_res == TRANSITION_DENIED) {
					cmd_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;

				} else {
					cmd_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;
				}
			}
		}
		break;



	case vehicle_command_s::VEHICLE_CMD_NAV_TAKEOFF: {
			/* ok, home set, use it to take off */
			if (_user_mode_intention.change(vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF, getSourceFromCommand(cmd))) {
				cmd_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;

			} else {
				printRejectMode(vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF);
				cmd_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
			}
		}
		break;


	case vehicle_command_s::VEHICLE_CMD_MISSION_START: {

			cmd_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED;

			// check if current mission and first item are valid
			if (!_failsafe_flags.auto_mission_missing) {

				// requested first mission item valid
				if (PX4_ISFINITE(cmd.param1) && (cmd.param1 >= -1) && (cmd.param1 < _mission_result_sub.get().seq_total)) {

					// switch to AUTO_MISSION and ARM
					if (_user_mode_intention.change(vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION, getSourceFromCommand(cmd))
					    && (TRANSITION_DENIED != arm(arm_disarm_reason_t::mission_start))) {

						cmd_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;

					} else {
						printRejectMode(vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION);
						cmd_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
					}
				}

			} else {
				mavlink_log_critical(&_mavlink_log_pub, "Mission start denied! No valid mission\t");
				events::send(events::ID("commander_mission_start_denied_no_mission"), {events::Log::Critical, events::LogInternal::Info},
					     "Mission start denied! No valid mission");
			}
		}
		break;



	case vehicle_command_s::VEHICLE_CMD_DO_ORBIT: {

			transition_result_t main_ret;

			if (_vehicle_status.in_transition_mode) {
				main_ret = TRANSITION_DENIED;

			} else if (_vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {
				// for fixed wings the behavior of orbit is the same as loiter
				if (_user_mode_intention.change(vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER, getSourceFromCommand(cmd))) {
					main_ret = TRANSITION_CHANGED;

				} else {
					main_ret = TRANSITION_DENIED;
				}

			} else {
				// Switch to orbit state and let the orbit task handle the command further
				if (_user_mode_intention.change(vehicle_status_s::NAVIGATION_STATE_ORBIT, getSourceFromCommand(cmd))) {
					main_ret = TRANSITION_CHANGED;

				} else {
					main_ret = TRANSITION_DENIED;
				}
			}

			if (main_ret != TRANSITION_DENIED) {
				cmd_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;

			} else {
				cmd_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
				mavlink_log_critical(&_mavlink_log_pub, "Orbit command rejected");
			}
		}
		break;


	case vehicle_command_s::VEHICLE_CMD_ACTUATOR_TEST:
		cmd_result = handleCommandActuatorTest(cmd);
		break;


	case vehicle_command_s::VEHICLE_CMD_PREFLIGHT_CALIBRATION: {

			if (isArmed() || _worker_thread.isBusy()) {

				// reject if armed or shutting down
				answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED);

			} else {

				if ((int)(cmd.param1) == 1) {
					/* gyro calibration */
					answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);
					_vehicle_status.calibration_enabled = true;
					_worker_thread.startTask(WorkerThread::Request::GyroCalibration);

				} else if ((int)(cmd.param1) == vehicle_command_s::PREFLIGHT_CALIBRATION_TEMPERATURE_CALIBRATION ||
					   (int)(cmd.param5) == vehicle_command_s::PREFLIGHT_CALIBRATION_TEMPERATURE_CALIBRATION ||
					   (int)(cmd.param7) == vehicle_command_s::PREFLIGHT_CALIBRATION_TEMPERATURE_CALIBRATION) {
					/* temperature calibration: handled in events module */
					break;

				} else if ((int)(cmd.param2) == 1) {
					/* magnetometer calibration */
					answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);
					_vehicle_status.calibration_enabled = true;
					_worker_thread.startTask(WorkerThread::Request::MagCalibration);

				} else if ((int)(cmd.param3) == 1) {
					/* baro calibration */
					answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);
					_vehicle_status.calibration_enabled = true;
					_worker_thread.startTask(WorkerThread::Request::BaroCalibration);

				} else if ((int)(cmd.param4) == 1) {
					/* RC calibration */
					answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);
					/* disable RC control input completely */
					_vehicle_status.rc_calibration_in_progress = true;
					mavlink_log_info(&_mavlink_log_pub, "Calibration: Disabling RC input\t");
					events::send(events::ID("commander_calib_rc_off"), events::Log::Info,
						     "Calibration: Disabling RC input");

				} else if ((int)(cmd.param4) == 2) {
					/* RC trim calibration */
					answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);
					_vehicle_status.calibration_enabled = true;
					_worker_thread.startTask(WorkerThread::Request::RCTrimCalibration);

				} else if ((int)(cmd.param5) == 1) {
					/* accelerometer calibration */
					answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);
					_vehicle_status.calibration_enabled = true;
					_worker_thread.startTask(WorkerThread::Request::AccelCalibration);

				} else if ((int)(cmd.param5) == 2) {
					// board offset calibration
					answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);
					_vehicle_status.calibration_enabled = true;
					_worker_thread.startTask(WorkerThread::Request::LevelCalibration);

				} else if ((int)(cmd.param5) == 4) {
					// accelerometer quick calibration
					answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);
					_vehicle_status.calibration_enabled = true;
					_worker_thread.startTask(WorkerThread::Request::AccelCalibrationQuick);

				}
				// else if ((int)(cmd.param6) == 1 || (int)(cmd.param6) == 2) {
					// TODO: param6 == 1 is deprecated, but we still accept it for a while (feb 2017)
					/* airspeed calibration */
				// 	answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);
				// 	_vehicle_status.calibration_enabled = true;
				// 	_worker_thread.startTask(WorkerThread::Request::AirspeedCalibration);

				// } else if ((int)(cmd.param7) == 1) {
				// 	/* do esc calibration */
				// 	if (check_battery_disconnected(&_mavlink_log_pub)) {
				// 		answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);

				// 		if (_safety.isButtonAvailable() && !_safety.isSafetyOff()) {
				// 			mavlink_log_critical(&_mavlink_log_pub, "ESC calibration denied! Press safety button first\t");
				// 			events::send(events::ID("commander_esc_calibration_denied"), events::Log::Critical,
				// 				     "ESCs calibration denied");

				// 		} else {
				// 			_vehicle_status.calibration_enabled = true;
				// 			_actuator_armed.in_esc_calibration_mode = true;
				// 			_worker_thread.startTask(WorkerThread::Request::ESCCalibration);
				// 		}

				// 	} else {
				// 		answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED);
				// 	}

				// }
				else if ((int)(cmd.param4) == 0) {
					/* RC calibration ended - have we been in one worth confirming? */
					if (_vehicle_status.rc_calibration_in_progress) {
						/* enable RC control input */
						_vehicle_status.rc_calibration_in_progress = false;
						mavlink_log_info(&_mavlink_log_pub, "Calibration: Restoring RC input\t");
						events::send(events::ID("commander_calib_rc_on"), events::Log::Info,
							     "Calibration: Restoring RC input");
					}

					answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);

				} else {
					answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_UNSUPPORTED);
				}
			}

			break;
		}

	case vehicle_command_s::VEHICLE_CMD_FIXED_MAG_CAL_YAW: {
			// Magnetometer quick calibration using world magnetic model and known heading
			if (isArmed() || _worker_thread.isBusy()) {

				// reject if armed or shutting down
				answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED);

			} else {
				answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);
				// parameter 1: Heading   (degrees)
				// parameter 3: Latitude  (degrees)
				// parameter 4: Longitude (degrees)

				// assume vehicle pointing north (0 degrees) if heading isn't specified
				const float heading_radians = PX4_ISFINITE(cmd.param1) ? math::radians(roundf(cmd.param1)) : 0.f;

				float latitude = NAN;
				float longitude = NAN;

				if (PX4_ISFINITE(cmd.param3) && PX4_ISFINITE(cmd.param4)) {
					// invalid if both lat & lon are 0 (current mavlink spec)
					if ((fabsf(cmd.param3) > 0) && (fabsf(cmd.param4) > 0)) {
						latitude = cmd.param3;
						longitude = cmd.param4;
					}
				}

				_vehicle_status.calibration_enabled = true;
				_worker_thread.setMagQuickData(heading_radians, latitude, longitude);
				_worker_thread.startTask(WorkerThread::Request::MagCalibrationQuick);
			}

			break;
		}

	case vehicle_command_s::VEHICLE_CMD_PREFLIGHT_STORAGE: {

			if (isArmed() || _worker_thread.isBusy()) {

				// reject if armed or shutting down
				answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED);

			} else {

				if (((int)(cmd.param1)) == 0) {
					answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);
					_worker_thread.startTask(WorkerThread::Request::ParamLoadDefault);

				} else if (((int)(cmd.param1)) == 1) {
					answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);
					_worker_thread.startTask(WorkerThread::Request::ParamSaveDefault);

				} else if (((int)(cmd.param1)) == 2) {
					answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);
					_worker_thread.startTask(WorkerThread::Request::ParamResetAllConfig);

				} else if (((int)(cmd.param1)) == 3) {
					answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);
					_worker_thread.startTask(WorkerThread::Request::ParamResetSensorFactory);

				} else if (((int)(cmd.param1)) == 4) {
					answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);
					_worker_thread.startTask(WorkerThread::Request::ParamResetAll);
				}
			}

			break;
		}

	case vehicle_command_s::VEHICLE_CMD_DO_SET_STANDARD_MODE: {
			mode_util::StandardMode standard_mode = (mode_util::StandardMode) roundf(cmd.param1);
			uint8_t nav_state = mode_util::getNavStateFromStandardMode(standard_mode);

			if (nav_state == vehicle_status_s::NAVIGATION_STATE_MAX) {
				answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_FAILED);

			} else {
				if (_user_mode_intention.change(nav_state, getSourceFromCommand(cmd))) {
					answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);

				} else {
					answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED);
				}
			}
		}
		break;

	case vehicle_command_s::VEHICLE_CMD_RUN_PREARM_CHECKS:
		_health_and_arming_checks.update(true);
		answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);
		break;

	case vehicle_command_s::VEHICLE_CMD_START_RX_PAIR:
	case vehicle_command_s::VEHICLE_CMD_CUSTOM_0:
	case vehicle_command_s::VEHICLE_CMD_CUSTOM_1:
	case vehicle_command_s::VEHICLE_CMD_CUSTOM_2:
	case vehicle_command_s::VEHICLE_CMD_DO_MOUNT_CONTROL:
	case vehicle_command_s::VEHICLE_CMD_DO_MOUNT_CONFIGURE:
	case vehicle_command_s::VEHICLE_CMD_DO_MOUNT_CONTROL_QUAT:
	case vehicle_command_s::VEHICLE_CMD_PREFLIGHT_SET_SENSOR_OFFSETS:
	case vehicle_command_s::VEHICLE_CMD_PREFLIGHT_UAVCAN:
	case vehicle_command_s::VEHICLE_CMD_PAYLOAD_PREPARE_DEPLOY:
	case vehicle_command_s::VEHICLE_CMD_PAYLOAD_CONTROL_DEPLOY:
	// case vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION:
	case vehicle_command_s::VEHICLE_CMD_DO_TRIGGER_CONTROL:
	case vehicle_command_s::VEHICLE_CMD_DO_DIGICAM_CONTROL:
	case vehicle_command_s::VEHICLE_CMD_DO_SET_CAM_TRIGG_DIST:
	case vehicle_command_s::VEHICLE_CMD_OBLIQUE_SURVEY:
	case vehicle_command_s::VEHICLE_CMD_DO_SET_CAM_TRIGG_INTERVAL:
	case vehicle_command_s::VEHICLE_CMD_SET_CAMERA_MODE:
	case vehicle_command_s::VEHICLE_CMD_SET_CAMERA_ZOOM:
	case vehicle_command_s::VEHICLE_CMD_SET_CAMERA_FOCUS:
	case vehicle_command_s::VEHICLE_CMD_DO_CHANGE_SPEED:
	case vehicle_command_s::VEHICLE_CMD_DO_LAND_START:
	case vehicle_command_s::VEHICLE_CMD_DO_GO_AROUND:
	case vehicle_command_s::VEHICLE_CMD_LOGGING_START:
	case vehicle_command_s::VEHICLE_CMD_LOGGING_STOP:
	case vehicle_command_s::VEHICLE_CMD_NAV_DELAY:
	case vehicle_command_s::VEHICLE_CMD_DO_SET_ROI:
	case vehicle_command_s::VEHICLE_CMD_NAV_ROI:
	case vehicle_command_s::VEHICLE_CMD_DO_SET_ROI_LOCATION:
	case vehicle_command_s::VEHICLE_CMD_DO_SET_ROI_WPNEXT_OFFSET:
	case vehicle_command_s::VEHICLE_CMD_DO_SET_ROI_NONE:
	case vehicle_command_s::VEHICLE_CMD_INJECT_FAILURE:
	// case vehicle_command_s::VEHICLE_CMD_SET_GPS_GLOBAL_ORIGIN:
	// case vehicle_command_s::VEHICLE_CMD_DO_GIMBAL_MANAGER_PITCHYAW:
	case vehicle_command_s::VEHICLE_CMD_DO_GIMBAL_MANAGER_CONFIGURE:
	case vehicle_command_s::VEHICLE_CMD_CONFIGURE_ACTUATOR:
	case vehicle_command_s::VEHICLE_CMD_DO_SET_ACTUATOR:
	case vehicle_command_s::VEHICLE_CMD_REQUEST_MESSAGE:
	case vehicle_command_s::VEHICLE_CMD_DO_WINCH:
	case vehicle_command_s::VEHICLE_CMD_DO_GRIPPER:
		/* ignore commands that are handled by other parts of the system */
		break;

	default:
		/* Warn about unsupported commands, this makes sense because only commands
		 * to this component ID (or all) are passed by mavlink. */
		answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_UNSUPPORTED);
		break;
	}

	if (cmd_result != vehicle_command_ack_s::VEHICLE_CMD_RESULT_UNSUPPORTED) {
		/* already warned about unsupported commands in "default" case */
		answer_command(cmd, cmd_result);
	}

	return true;
}

ModeChangeSource Commander::getSourceFromCommand(const vehicle_command_s &cmd)
{
	return cmd.source_component >= vehicle_command_s::COMPONENT_MODE_EXECUTOR_START ? ModeChangeSource::ModeExecutor :
	       ModeChangeSource::User;
}

void Commander::handleCommandsFromModeExecutors()
{
	if (_vehicle_command_mode_executor_sub.updated()) {
		const unsigned last_generation = _vehicle_command_mode_executor_sub.get_last_generation();
		vehicle_command_s cmd;

		if (_vehicle_command_mode_executor_sub.copy(&cmd)) {
			if (_vehicle_command_mode_executor_sub.get_last_generation() != last_generation + 1) {
				PX4_ERR("vehicle_command from executor lost, generation %u -> %u", last_generation,
					_vehicle_command_mode_executor_sub.get_last_generation());
			}

			// For commands from mode executors, we check if it is in charge and then publish it on the official
			// command topic
			const int mode_executor_in_charge = _mode_management.modeExecutorInCharge();

			// source_system is set to the mode executor
			if (cmd.source_component == vehicle_command_s::COMPONENT_MODE_EXECUTOR_START + mode_executor_in_charge) {
				cmd.source_system = _vehicle_status.system_id;
				cmd.timestamp = hrt_absolute_time();
				_vehicle_command_pub.publish(cmd);

			} else {
				cmd.source_system = _vehicle_status.system_id;
				answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED);
				PX4_WARN("Got cmd from executor %i not in charge (in charge: %i)", cmd.source_system, mode_executor_in_charge);
			}
		}
	}
}

unsigned Commander::handleCommandActuatorTest(const vehicle_command_s &cmd)
{
	if (isArmed() || (_safety.isButtonAvailable() && !_safety.isSafetyOff())) {
		return vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED;
	}

	if (_param_com_mot_test_en.get() != 1) {
		return vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED;
	}

	actuator_test_s actuator_test{};
	actuator_test.timestamp = hrt_absolute_time();
	actuator_test.function = (int)(cmd.param5 + 0.5);

	if (actuator_test.function < 1000) {
		const int first_motor_function = 1; // from MAVLink ACTUATOR_OUTPUT_FUNCTION
		const int first_servo_function = 33;

		if (actuator_test.function >= first_motor_function
		    && actuator_test.function < first_motor_function + actuator_test_s::MAX_NUM_MOTORS) {
			actuator_test.function = actuator_test.function - first_motor_function + actuator_test_s::FUNCTION_MOTOR1;

		} else if (actuator_test.function >= first_servo_function
			   && actuator_test.function < first_servo_function + actuator_test_s::MAX_NUM_SERVOS) {
			actuator_test.function = actuator_test.function - first_servo_function + actuator_test_s::FUNCTION_SERVO1;

		} else {
			return vehicle_command_ack_s::VEHICLE_CMD_RESULT_UNSUPPORTED;
		}

	} else {
		actuator_test.function -= 1000;
	}

	actuator_test.value = cmd.param1;

	actuator_test.action = actuator_test_s::ACTION_DO_CONTROL;
	int timeout_ms = (int)(cmd.param2 * 1000.f + 0.5f);

	if (timeout_ms <= 0) {
		actuator_test.action = actuator_test_s::ACTION_RELEASE_CONTROL;

	} else {
		actuator_test.timeout_ms = timeout_ms;
	}

	// enforce a timeout and a maximum limit
	if (actuator_test.timeout_ms == 0 || actuator_test.timeout_ms > 3000) {
		actuator_test.timeout_ms = 3000;
	}

	_actuator_test_pub.publish(actuator_test);
	return vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;
}



void Commander::updateParameters()
{
	// update parameters from storage
	updateParams();

	int32_t value_int32 = 0;

	// MAV_SYS_ID => vehicle_status.system_id
	if ((_param_mav_sys_id != PARAM_INVALID) && (param_get(_param_mav_sys_id, &value_int32) == PX4_OK)) {
		_vehicle_status.system_id = value_int32;
	}

	// MAV_COMP_ID => vehicle_status.component_id
	if ((_param_mav_comp_id != PARAM_INVALID) && (param_get(_param_mav_comp_id, &value_int32) == PX4_OK)) {
		_vehicle_status.component_id = value_int32;
	}

	// MAV_TYPE -> vehicle_status.system_type
	if ((_param_mav_type != PARAM_INVALID) && (param_get(_param_mav_type, &value_int32) == PX4_OK)) {
		_vehicle_status.system_type = value_int32;
	}


	_vehicle_status.avoidance_system_required = _param_com_obs_avoid.get();

	_auto_disarm_killed.set_hysteresis_time_from(false, _param_com_kill_disarm.get() * 1_s);


	// _mode_switch_mapped = (RC_MAP_FLTMODE > 0)
	if (_param_rc_map_fltmode != PARAM_INVALID && (param_get(_param_rc_map_fltmode, &value_int32) == PX4_OK)) {
		_mode_switch_mapped = (value_int32 > 0);
	}

}


void Commander::modeManagementUpdate()
{
	ModeManagement::UpdateRequest mode_management_update{};
	_mode_management.update(isArmed(), _vehicle_status.nav_state_user_intention,
				_failsafe.selectedAction() > FailsafeBase::Action::Warn, mode_management_update);

	if (!isArmed() && mode_management_update.change_user_intended_nav_state) {
		_user_mode_intention.change(mode_management_update.user_intended_nav_state);
	}

	if (mode_management_update.control_setpoint_update) {
		_status_changed = true;
	}
}


void Commander::run()
{

	while (!should_exit()) {

		perf_begin(_loop_perf);

		const actuator_armed_s actuator_armed_prev{_actuator_armed};

		/* update parameters */
		const bool params_updated = _parameter_update_sub.updated();

		if (params_updated) {
			// clear update
			parameter_update_s update;
			_parameter_update_sub.copy(&update);

			updateParameters();

			_status_changed = true;
		}

		/* Update OA parameter */
		_vehicle_status.avoidance_system_required = _param_com_obs_avoid.get();

		// handlePowerButtonState();

		// systemPowerUpdate();

		landDetectorUpdate();

		// safetyButtonUpdate();

		_multicopter_throw_launch.update(isArmed());

		// vtolStatusUpdate();

		_home_position.update(_param_com_home_en.get(), !isArmed() && _vehicle_land_detected.landed);

		// handleAutoDisarm();

		// battery_status_check();

		checkForMissionUpdate();

		// manualControlCheck();

		// offboardControlCheck();

		// data link checks which update the status
		dataLinkCheck();

		// Check for failure detector status
		if (_failure_detector.update(_vehicle_status, _vehicle_control_mode)) {
			_vehicle_status.failure_detector_status = _failure_detector.getStatus().value;
			_status_changed = true;
		}

		modeManagementUpdate();

		const hrt_abstime now = hrt_absolute_time();

		// const bool nav_state_or_failsafe_changed = handleModeIntentionAndFailsafe();

		// Run arming checks @ 10Hz
		if ((now >= _last_health_and_arming_check + 100_ms) || _status_changed) {
			_last_health_and_arming_check = now;

			perf_begin(_preflight_check_perf);
			_health_and_arming_checks.update();
			bool pre_flight_checks_pass = _health_and_arming_checks.canArm(_vehicle_status.nav_state);

			if (_vehicle_status.pre_flight_checks_pass != pre_flight_checks_pass) {
				_vehicle_status.pre_flight_checks_pass = pre_flight_checks_pass;
				_status_changed = true;
			}

			perf_end(_preflight_check_perf);
			checkAndInformReadyForTakeoff();
		}

		// handle commands last, as the system needs to be updated to handle them
		handleCommandsFromModeExecutors();

		if (_vehicle_command_sub.updated()) {
			// got command
			const unsigned last_generation = _vehicle_command_sub.get_last_generation();
			vehicle_command_s cmd;

			if (_vehicle_command_sub.copy(&cmd)) {
				if (_vehicle_command_sub.get_last_generation() != last_generation + 1) {
					PX4_ERR("vehicle_command lost, generation %u -> %u", last_generation, _vehicle_command_sub.get_last_generation());
				}

				if (handle_command(cmd)) {
					_status_changed = true;
				}
			}
		}

		if (_action_request_sub.updated()) {
			const unsigned last_generation = _action_request_sub.get_last_generation();
			action_request_s action_request;

			if (_action_request_sub.copy(&action_request)) {
				if (_action_request_sub.get_last_generation() != last_generation + 1) {
					PX4_ERR("action_request lost, generation %u -> %u", last_generation, _action_request_sub.get_last_generation());
				}

				// executeActionRequest(action_request);
			}
		}

		// update actuator_armed
		_actuator_armed.armed = isArmed();
		_actuator_armed.prearmed = getPrearmState();
		_actuator_armed.ready_to_arm = _vehicle_status.pre_flight_checks_pass || isArmed();
		// _actuator_armed.lockdown = ((_vehicle_status.nav_state == _vehicle_status.NAVIGATION_STATE_TERMINATION)
		// 			    || (_vehicle_status.hil_state == vehicle_status_s::HIL_STATE_ON)
		// 			    || _multicopter_throw_launch.isThrowLaunchInProgress());
		// _actuator_armed.manual_lockdown // action_request_s::ACTION_KILL
		_actuator_armed.force_failsafe = (_vehicle_status.nav_state == _vehicle_status.NAVIGATION_STATE_TERMINATION);
		// _actuator_armed.in_esc_calibration_mode // VEHICLE_CMD_PREFLIGHT_CALIBRATION

		// if force_failsafe or manual_lockdown activated send parachute command
		// if ((!actuator_armed_prev.force_failsafe && _actuator_armed.force_failsafe)
		//     || (!actuator_armed_prev.manual_lockdown && _actuator_armed.manual_lockdown)
		//    ) {
		// 	if (isArmed()) {
		// 		send_parachute_command();
		// 	}
		// }

		// publish states (armed, control_mode, vehicle_status, failure_detector_status) at 2 Hz or immediately when changed
		if ((now >= _vehicle_status.timestamp + 500_ms) || _status_changed
		    || !(_actuator_armed == actuator_armed_prev)) {

			// publish actuator_armed first (used by output modules)
			_actuator_armed.timestamp = hrt_absolute_time();
			_actuator_armed_pub.publish(_actuator_armed);

			// update and publish vehicle_control_mode
			updateControlMode();

			// vehicle_status publish (after prearm/preflight updates above)
			_mode_management.getModeStatus(_vehicle_status.valid_nav_states_mask, _vehicle_status.can_set_nav_states_mask);
			_vehicle_status.timestamp = hrt_absolute_time();
			_vehicle_status_pub.publish(_vehicle_status);

			// failure_detector_status publish
			failure_detector_status_s fd_status{};
			fd_status.fd_roll = _failure_detector.getStatusFlags().roll;
			fd_status.fd_pitch = _failure_detector.getStatusFlags().pitch;
			fd_status.fd_alt = _failure_detector.getStatusFlags().alt;
			fd_status.fd_ext = _failure_detector.getStatusFlags().ext;
			fd_status.fd_arm_escs = _failure_detector.getStatusFlags().arm_escs;
			fd_status.fd_battery = _failure_detector.getStatusFlags().battery;
			fd_status.fd_imbalanced_prop = _failure_detector.getStatusFlags().imbalanced_prop;
			fd_status.fd_motor = _failure_detector.getStatusFlags().motor;
			fd_status.imbalanced_prop_metric = _failure_detector.getImbalancedPropMetric();
			fd_status.motor_failure_mask = _failure_detector.getMotorFailures();
			fd_status.timestamp = hrt_absolute_time();
			_failure_detector_status_pub.publish(fd_status);
		}

		checkWorkerThread();

		// updateTunes();
		// control_status_leds(_status_changed, _battery_warning);

		_status_changed = false;

		// arm_auth_update(hrt_absolute_time(), params_updated);

		px4_indicate_external_reset_lockout(LockoutComponent::Commander, isArmed());

		perf_end(_loop_perf);

		// sleep if there are no vehicle_commands or action_requests to process
		if (!_vehicle_command_sub.updated() && !_action_request_sub.updated()) {
			px4_usleep(COMMANDER_MONITORING_INTERVAL);
		}
	}

	// rgbled_set_color_and_mode(led_control_s::COLOR_WHITE, led_control_s::MODE_OFF);

	// /* close fds */
	// led_deinit();
	// buzzer_deinit();
}

void Commander::checkForMissionUpdate()
{
	if (_mission_result_sub.updated()) {
		const mission_result_s &mission_result = _mission_result_sub.get();

		const auto prev_mission_mission_id = mission_result.mission_id;
		_mission_result_sub.update();

		// if mission_result is valid for the current mission
		const bool mission_result_ok = (mission_result.timestamp > _boot_timestamp)
					       && (mission_result.mission_id > 0);

		bool auto_mission_available = mission_result_ok && mission_result.valid;

		if (mission_result_ok) {
			/* Only evaluate mission state if home is set */
			if (!_failsafe_flags.home_position_invalid &&
			    (prev_mission_mission_id != mission_result.mission_id)) {


			}
		}

		if (isArmed() && !_vehicle_land_detected.landed
		    && (mission_result.timestamp >= _vehicle_status.nav_state_timestamp)
		    && mission_result.finished
		    && _mode_management.modeExecutorInCharge() == ModeExecutors::AUTOPILOT_EXECUTOR_ID) {

			if (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF
			    || _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_VTOL_TAKEOFF) {
				// Transition mode to loiter or auto-mission after takeoff is completed.
				if ((_param_takeoff_finished_action.get() == 1) && auto_mission_available) {
					_user_mode_intention.change(vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION);

				} else {
					_user_mode_intention.change(vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER);
				}

			} else if (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION) {
				// Transition to loiter when the mission is cleared and/or finished, and we are still in mission mode.
				_user_mode_intention.change(vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER);
			}
		}
	}
}

bool Commander::getPrearmState() const
{
	if (_vehicle_status.calibration_enabled) {
		return false;
	}

	switch ((PrearmedMode)_param_com_prearm_mode.get()) {
	case PrearmedMode::DISABLED:
		/* skip prearmed state  */
		return false;

	case PrearmedMode::ALWAYS:
		/* safety is not present, go into prearmed
		* (all output drivers should be started / unlocked last in the boot process
		* when the rest of the system is fully initialized)
		*/
		return hrt_elapsed_time(&_boot_timestamp) > 5_s;

	case PrearmedMode::SAFETY_BUTTON:
		if (_safety.isButtonAvailable()) {
			/* safety button is present, go into prearmed if safety is off */
			return _safety.isSafetyOff();
		}

		/* safety button is not present, do not go into prearmed */
		return false;
	}

	return false;
}


void Commander::landDetectorUpdate()
{
	if (_vehicle_land_detected_sub.updated()) {
		const bool was_landed = _vehicle_land_detected.landed;
		_vehicle_land_detected_sub.copy(&_vehicle_land_detected);

		// Only take actions if armed
		if (isArmed()) {
			if (!was_landed && _vehicle_land_detected.landed) {
				mavlink_log_info(&_mavlink_log_pub, "Landing detected\t");
				events::send(events::ID("commander_landing_detected"), events::Log::Info, "Landing detected");

			} else if (was_landed && !_vehicle_land_detected.landed) {
				mavlink_log_info(&_mavlink_log_pub, "Takeoff detected\t");
				events::send(events::ID("commander_takeoff_detected"), events::Log::Info, "Takeoff detected");
				_vehicle_status.takeoff_time = hrt_absolute_time();
				_have_taken_off_since_arming = true;
			}

			// automatically set or update home position
			if (_param_com_home_en.get()) {
				// set the home position when taking off
				if (!_vehicle_land_detected.landed) {
					if (was_landed) {
						_home_position.setHomePosition();

					} else if (_param_com_home_in_air.get()) {
						_home_position.setInAirHomePosition();
					}
				}
			}
		}
	}
}


void Commander::checkWorkerThread()
{
	// check if the worker has finished
	if (_worker_thread.hasResult()) {
		int ret = _worker_thread.getResultAndReset();
		_actuator_armed.in_esc_calibration_mode = false;

		if (_vehicle_status.calibration_enabled) { // did we do a calibration?
			_vehicle_status.calibration_enabled = false;

			if (ret == 0) {
				// //tune_positive(true);

			} else {
				//tune_negative(true);
			}
		}
	}
}


void Commander::checkAndInformReadyForTakeoff()
{
#ifdef CONFIG_ARCH_BOARD_PX4_SITL
	static bool ready_for_takeoff_printed = false;

	if (_vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING ||
	    _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {
		if (!ready_for_takeoff_printed &&
		    _health_and_arming_checks.canArm(vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF)) {
			PX4_INFO("%sReady for takeoff!%s", PX4_ANSI_COLOR_GREEN, PX4_ANSI_COLOR_RESET);
			ready_for_takeoff_printed = true;
		}
	}

#endif // CONFIG_ARCH_BOARD_PX4_SITL
}


void Commander::updateControlMode()
{
	_vehicle_control_mode = {};

	// mode_util::getVehicleControlMode(_vehicle_status.nav_state,
	// 				 _vehicle_status.vehicle_type, _offboard_control_mode_sub.get(), _vehicle_control_mode);
	_mode_management.updateControlMode(_vehicle_status.nav_state, _vehicle_control_mode);

	_vehicle_control_mode.flag_armed = isArmed();
	_vehicle_control_mode.flag_multicopter_position_control_enabled =
		(_vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING)
		&& (_vehicle_control_mode.flag_control_altitude_enabled
		    || _vehicle_control_mode.flag_control_climb_rate_enabled
		    || _vehicle_control_mode.flag_control_position_enabled
		    || _vehicle_control_mode.flag_control_velocity_enabled
		    || _vehicle_control_mode.flag_control_acceleration_enabled);
	_vehicle_control_mode.timestamp = hrt_absolute_time();
	_vehicle_control_mode_pub.publish(_vehicle_control_mode);
}

void Commander::printRejectMode(uint8_t nav_state)
{
	if (hrt_elapsed_time(&_last_print_mode_reject_time) > 1_s) {

		mavlink_log_critical(&_mavlink_log_pub, "Switching to %s is currently not available\t",
				     mode_util::nav_state_names[nav_state]);
		px4_custom_mode custom_mode = get_px4_custom_mode(nav_state);
		uint32_t mavlink_mode = custom_mode.data;
		/* EVENT
		 * @type append_health_and_arming_messages
		 */
		events::send<uint32_t, events::px4::enums::navigation_mode_t>(events::ID("commander_modeswitch_not_avail"), {events::Log::Critical, events::LogInternal::Info},
				"Switching to mode '{2}' is currently not possible", mavlink_mode, mode_util::navigation_mode(nav_state));

		/* only buzz if armed, because else we're driving people nuts indoors
		they really need to look at the leds as well. */
		//tune_negative(isArmed());

		_last_print_mode_reject_time = hrt_absolute_time();
	}
}

void Commander::answer_command(const vehicle_command_s &cmd, uint8_t result)
{
	switch (result) {
	case vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED:
		break;

	case vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED:
		//tune_negative(true);
		break;

	case vehicle_command_ack_s::VEHICLE_CMD_RESULT_FAILED:
		//tune_negative(true);
		break;

	case vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED:
		//tune_negative(true);
		break;

	case vehicle_command_ack_s::VEHICLE_CMD_RESULT_UNSUPPORTED:
		//tune_negative(true);
		break;

	default:
		break;
	}

	/* publish ACK */
	vehicle_command_ack_s command_ack{};
	command_ack.command = cmd.command;
	command_ack.result = result;
	command_ack.target_system = cmd.source_system;
	command_ack.target_component = cmd.source_component;
	command_ack.timestamp = hrt_absolute_time();
	_vehicle_command_ack_pub.publish(command_ack);
}

int Commander::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("commander",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT + 40,
				      3250,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	// wait until task is up & running
	if (wait_until_running() < 0) {
		_task_id = -1;
		return -1;
	}

	return 0;
}

Commander *Commander::instantiate(int argc, char *argv[])
{
	Commander *instance = new Commander();

	if (instance) {
		if (argc >= 2 && !strcmp(argv[1], "-h")) {
			instance->enable_hil();
		}
	}

	return instance;
}

void Commander::enable_hil()
{
	_vehicle_status.hil_state = vehicle_status_s::HIL_STATE_ON;
}

void Commander::dataLinkCheck()
{
	for (auto &telemetry_status :  _telemetry_status_subs) {
		telemetry_status_s telemetry;

		if (telemetry_status.update(&telemetry)) {

			// handle different radio types
			switch (telemetry.type) {
			case telemetry_status_s::LINK_TYPE_USB:
				// set (but don't unset) telemetry via USB as active once a MAVLink connection is up
				_vehicle_status.usb_connected = true;
				break;

			case telemetry_status_s::LINK_TYPE_IRIDIUM: {
					iridiumsbd_status_s iridium_status;

					if (_iridiumsbd_status_sub.update(&iridium_status)) {
						_high_latency_datalink_heartbeat = iridium_status.last_heartbeat;

						if (_vehicle_status.high_latency_data_link_lost) {
							if (hrt_elapsed_time(&_high_latency_datalink_lost) > (_param_com_hldl_reg_t.get() * 1_s)) {
								_vehicle_status.high_latency_data_link_lost = false;
								_status_changed = true;
							}
						}
					}

					break;
				}
			}

			if (telemetry.heartbeat_type_gcs) {
				// Initial connection or recovery from data link lost
				if (_vehicle_status.gcs_connection_lost) {
					_vehicle_status.gcs_connection_lost = false;
					_status_changed = true;

					if (_datalink_last_heartbeat_gcs != 0) {
						mavlink_log_info(&_mavlink_log_pub, "GCS connection regained\t");
						events::send(events::ID("commander_dl_regained"), events::Log::Info, "GCS connection regained");
					}
				}

				_datalink_last_heartbeat_gcs = telemetry.timestamp;
			}

			if (telemetry.heartbeat_type_onboard_controller) {
				if (_onboard_controller_lost) {
					_onboard_controller_lost = false;
					_status_changed = true;

					if (_datalink_last_heartbeat_onboard_controller != 0) {
						mavlink_log_info(&_mavlink_log_pub, "Onboard controller regained\t");
						events::send(events::ID("commander_onboard_ctrl_regained"), events::Log::Info, "Onboard controller regained");
					}
				}

				_datalink_last_heartbeat_onboard_controller = telemetry.timestamp;
			}

			if (telemetry.heartbeat_type_parachute) {
				if (_parachute_system_lost) {
					_parachute_system_lost = false;

					if (_datalink_last_heartbeat_parachute_system != 0) {
						mavlink_log_info(&_mavlink_log_pub, "Parachute system regained\t");
						events::send(events::ID("commander_parachute_regained"), events::Log::Info, "Parachute system regained");
					}
				}

				bool healthy = telemetry.parachute_system_healthy;

				_datalink_last_heartbeat_parachute_system = telemetry.timestamp;
				_vehicle_status.parachute_system_present = true;
				_vehicle_status.parachute_system_healthy = healthy;
			}

			if (telemetry.heartbeat_type_open_drone_id) {
				if (_open_drone_id_system_lost) {
					_open_drone_id_system_lost = false;

					if (_datalink_last_heartbeat_open_drone_id_system != 0) {
						mavlink_log_info(&_mavlink_log_pub, "OpenDroneID system regained\t");
						events::send(events::ID("commander_open_drone_id_regained"), events::Log::Info, "OpenDroneID system regained");
					}
				}

				bool healthy = telemetry.open_drone_id_system_healthy;

				_datalink_last_heartbeat_open_drone_id_system = telemetry.timestamp;
				_vehicle_status.open_drone_id_system_present = true;
				_vehicle_status.open_drone_id_system_healthy = healthy;
			}

			if (telemetry.heartbeat_component_obstacle_avoidance) {
				if (_avoidance_system_lost) {
					_avoidance_system_lost = false;
					_status_changed = true;
				}

				_datalink_last_heartbeat_avoidance_system = telemetry.timestamp;
				_vehicle_status.avoidance_system_valid = telemetry.avoidance_system_healthy;
			}
		}
	}


	// GCS data link loss failsafe
	if (!_vehicle_status.gcs_connection_lost) {
		if ((_datalink_last_heartbeat_gcs != 0)
		    && hrt_elapsed_time(&_datalink_last_heartbeat_gcs) > (_param_com_dl_loss_t.get() * 1_s)) {

			_vehicle_status.gcs_connection_lost = true;
			_vehicle_status.gcs_connection_lost_counter++;

			mavlink_log_info(&_mavlink_log_pub, "Connection to ground station lost\t");
			events::send(events::ID("commander_gcs_lost"), {events::Log::Warning, events::LogInternal::Info},
				     "Connection to ground control station lost");

			_status_changed = true;
		}
	}

	// ONBOARD CONTROLLER data link loss failsafe
	if ((_datalink_last_heartbeat_onboard_controller > 0)
	    && (hrt_elapsed_time(&_datalink_last_heartbeat_onboard_controller) > (_param_com_obc_loss_t.get() * 1_s))
	    && !_onboard_controller_lost) {

		mavlink_log_critical(&_mavlink_log_pub, "Connection to mission computer lost\t");
		events::send(events::ID("commander_mission_comp_lost"), events::Log::Critical, "Connection to mission computer lost");
		_onboard_controller_lost = true;
		_status_changed = true;
	}

	// Parachute system
	if ((hrt_elapsed_time(&_datalink_last_heartbeat_parachute_system) > 3_s)
	    && !_parachute_system_lost) {
		mavlink_log_critical(&_mavlink_log_pub, "Parachute system lost");
		_vehicle_status.parachute_system_present = false;
		_vehicle_status.parachute_system_healthy = false;
		_parachute_system_lost = true;
		_status_changed = true;
	}

	// OpenDroneID system
	if ((hrt_elapsed_time(&_datalink_last_heartbeat_open_drone_id_system) > 3_s)
	    && !_open_drone_id_system_lost) {
		mavlink_log_critical(&_mavlink_log_pub, "OpenDroneID system lost");
		events::send(events::ID("commander_open_drone_id_lost"), events::Log::Critical, "OpenDroneID system lost");
		_vehicle_status.open_drone_id_system_present = false;
		_vehicle_status.open_drone_id_system_healthy = false;
		_open_drone_id_system_lost = true;
		_status_changed = true;
	}

	// AVOIDANCE SYSTEM state check (only if it is enabled)
	if (_vehicle_status.avoidance_system_required && !_onboard_controller_lost) {
		// if heartbeats stop
		if (!_avoidance_system_lost && (_datalink_last_heartbeat_avoidance_system > 0)
		    && (hrt_elapsed_time(&_datalink_last_heartbeat_avoidance_system) > 5_s)) {

			_avoidance_system_lost = true;
			_vehicle_status.avoidance_system_valid = false;
		}
	}

	// high latency data link loss failsafe
	if (_high_latency_datalink_heartbeat > 0
	    && hrt_elapsed_time(&_high_latency_datalink_heartbeat) > (_param_com_hldl_loss_t.get() * 1_s)) {
		_high_latency_datalink_lost = hrt_absolute_time();

		if (!_vehicle_status.high_latency_data_link_lost) {
			_vehicle_status.high_latency_data_link_lost = true;
			mavlink_log_critical(&_mavlink_log_pub, "High latency data link lost\t");
			events::send(events::ID("commander_high_latency_lost"), events::Log::Critical, "High latency data link lost");
			_status_changed = true;
		}
	}
}


int Commander::print_usage(const char *reason)
{
	if (reason) {
		PX4_INFO("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
	### Description
		The commander module contains the state machine for mode switching and failsafe behavior.
		)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("commander", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('h', "Enable HIL mode", true);
#ifndef CONSTRAINED_FLASH
	PRINT_MODULE_USAGE_COMMAND_DESCR("calibrate", "Run sensor calibration");
	PRINT_MODULE_USAGE_ARG("mag|baro|accel|gyro", "Calibration type", false);
	PRINT_MODULE_USAGE_ARG("quick", "Quick calibration (accel only, not recommended)", false);

	PRINT_MODULE_USAGE_COMMAND("takeoff");
	PRINT_MODULE_USAGE_COMMAND("land");
#endif
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 1;
}
