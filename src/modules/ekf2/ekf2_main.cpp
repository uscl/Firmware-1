/****************************************************************************
 *
 *   Copyright (c) 2015-2018 PX4 Development Team. All rights reserved.
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
 * @file ekf2_main.cpp
 * Implementation of the attitude and position estimator.
 *
 * @author Roman Bapst
 */

#include <float.h>

#include <drivers/drv_hrt.h>
#include <lib/ecl/EKF/ekf.h>
#include <lib/mathlib/mathlib.h>
#include <lib/perf/perf_counter.h>
#include <px4_defines.h>
#include <px4_module.h>
#include <px4_module_params.h>
#include <px4_posix.h>
#include <px4_tasks.h>
#include <px4_time.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/ekf2_innovations.h>
#include <uORB/topics/ekf2_timestamps.h>
#include <uORB/topics/ekf_gps_position.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_magnetometer.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/wind_estimate.h>
#include <uORB/topics/serial_com.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/
// defines used to specify the mask position for use of different accuracy metrics in the GPS blending algorithm
#define BLEND_MASK_USE_SPD_ACC      1
#define BLEND_MASK_USE_HPOS_ACC     2
#define BLEND_MASK_USE_VPOS_ACC     4

// define max number of GPS receivers supported and 0 base instance used to access virtual 'blended' GPS solution
#define GPS_MAX_RECEIVERS 2
#define GPS_BLENDED_INSTANCE 2

using math::constrain;
using namespace time_literals;

extern "C" __EXPORT int ekf2_main(int argc, char *argv[]);

class Ekf2 final : public ModuleBase<Ekf2>, public ModuleParams
{
public:
	Ekf2();
	~Ekf2() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static Ekf2 *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

//	void set_replay_mode(bool replay) { _replay_mode = replay; }

	static void	task_main_trampoline(int argc, char *argv[]);

    void LLA2NED(float *home_pos_LLA, float *cur_pos_LLA, float *result);

	int print_status() override;

private:
	bool publish_attitude(const sensor_combined_s &sensors, const hrt_abstime &now);

//	bool 	_replay_mode = false;			///< true when we use replay data from a log

	// time slip monitoring
	uint64_t _start_time_us = 0;		///< system time at EKF start (uSec)
	int64_t _last_time_slip_us = 0;		///< Last time slip (uSec)

	perf_counter_t _perf_update_data;
	perf_counter_t _perf_ekf_update;

	// set pose/velocity as invalid if standard deviation is bigger than max_std_dev
	// TODO: the user should be allowed to set these values by a parameter
	static constexpr float ep_max_std_dev = 100.0f;	///< Maximum permissible standard deviation for estimated position
	static constexpr float eo_max_std_dev = 100.0f;	///< Maximum permissible standard deviation for estimated orientation
	//static constexpr float ev_max_std_dev = 100.0f;	///< Maximum permissible standard deviation for estimated velocity

	// GPS blending and switching
    Vector2f _NE_pos_offset_m[GPS_MAX_RECEIVERS] = {}; ///< Filtered North,East position offset from GPS instance to blended solution in _output_state.location (m)

	int _params_sub{-1};
	int _sensors_sub{-1};
	int _status_sub{-1};
	int _vehicle_land_detected_sub{-1};
    int _serial_com_sub{-1};
    int _home_position_sub{-1};

	// because we can have multiple GPS instances
	int _gps_subs[GPS_MAX_RECEIVERS] {};
	int _gps_orb_instance{-1};

	orb_advert_t _att_pub{nullptr};
	orb_advert_t _wind_pub{nullptr};
    orb_advert_t _control_state_pub{nullptr};

	uORB::Publication<vehicle_local_position_s> _vehicle_local_position_pub;
	uORB::Publication<vehicle_global_position_s> _vehicle_global_position_pub;

	Ekf _ekf;

	parameters *_params;	///< pointer to ekf parameter struct (located in _ekf class instance)

};

Ekf2::Ekf2():
	ModuleParams(nullptr),
	_perf_update_data(perf_alloc_once(PC_ELAPSED, "EKF2 data acquisition")),
	_perf_ekf_update(perf_alloc_once(PC_ELAPSED, "EKF2 update")),
	_vehicle_local_position_pub(ORB_ID(vehicle_local_position)),
	_vehicle_global_position_pub(ORB_ID(vehicle_global_position)),
    _params(_ekf.getParamHandle())
{
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_sensors_sub = orb_subscribe(ORB_ID(sensor_combined));
	_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	_vehicle_land_detected_sub = orb_subscribe(ORB_ID(vehicle_land_detected));

	for (unsigned i = 0; i < GPS_MAX_RECEIVERS; i++) {
		_gps_subs[i] = orb_subscribe_multi(ORB_ID(vehicle_gps_position), i);
	}

	// initialise parameter cache
    //updateParams();
}

Ekf2::~Ekf2()
{
	perf_free(_perf_update_data);
	perf_free(_perf_ekf_update);

	orb_unsubscribe(_params_sub);
	orb_unsubscribe(_sensors_sub);
	orb_unsubscribe(_status_sub);
	orb_unsubscribe(_vehicle_land_detected_sub);

    for (unsigned i = 0; i < GPS_MAX_RECEIVERS; i++) {
		orb_unsubscribe(_gps_subs[i]);
	}
}

int Ekf2::print_status()
{
	PX4_INFO("local position: %s", (_ekf.local_position_is_valid()) ? "valid" : "invalid");
	PX4_INFO("global position: %s", (_ekf.global_position_is_valid()) ? "valid" : "invalid");

	PX4_INFO("time slip: %" PRId64 " us", _last_time_slip_us);

	perf_print_counter(_perf_update_data);
	perf_print_counter(_perf_ekf_update);

	return 0;
}

void Ekf2::LLA2NED(float *home_pos_LLA, float *cur_pos_LLA, float *result)
{
        float exp = 0.08181919f;
        float Ne, Ner;
        float xe, ye, ze;
        float xer, yer, zer;
        //double _position_NED[3] = {0.0};

        // LLA[0] = Latitude, LLA[1] = Longitude, LLA[2] = height
        Ne = 6378137.0f / sqrtf(1.0f-exp*exp*sinf(cur_pos_LLA[0])*sinf(cur_pos_LLA[0]));
        Ner = 6378137.0f / sqrtf(1.0f-exp*exp*sinf(home_pos_LLA[0])*sinf(home_pos_LLA[0]));

        xe = (Ne + cur_pos_LLA[2])*cosf(cur_pos_LLA[0])*cosf(cur_pos_LLA[1]);
        ye = (Ne + cur_pos_LLA[2])*cosf(cur_pos_LLA[0])*sinf(cur_pos_LLA[1]);
        ze = (Ne*(1.0f-exp*exp) + cur_pos_LLA[2])*sinf(cur_pos_LLA[0]);

        xer = (Ner + home_pos_LLA[2])*cosf(home_pos_LLA[0])*cosf(home_pos_LLA[1]);
        yer = (Ner + home_pos_LLA[2])*cosf(home_pos_LLA[0])*sinf(home_pos_LLA[1]);
        zer = (Ner*(1.0f-exp*exp) + home_pos_LLA[2])*sinf(home_pos_LLA[0]);

        result[0] = -sinf(home_pos_LLA[0])*cosf(home_pos_LLA[1])*(xe-xer) -sinf(home_pos_LLA[0])*sinf(home_pos_LLA[1])*(ye-yer) + cosf(home_pos_LLA[0])*(ze-zer);
        result[1] = -sinf(home_pos_LLA[1])*(xe-xer) + cosf(home_pos_LLA[1])*(ye-yer);
        result[2] = -cosf(home_pos_LLA[0])*cosf(home_pos_LLA[1])*(xe-xer) - cosf(home_pos_LLA[0])*sinf(home_pos_LLA[1])*(ye-yer) - sinf(home_pos_LLA[0])*(ze-zer);
}

void Ekf2::run()
{
	px4_pollfd_struct_t fds[1] = {};
	fds[0].fd = _sensors_sub;
	fds[0].events = POLLIN;

	// initialize data structures outside of loop
	// because they will else not always be
	// properly populated
	sensor_combined_s sensors = {};
	vehicle_land_detected_s vehicle_land_detected = {};
	vehicle_status_s vehicle_status = {};
    serial_com_s _serial_com = {};
    home_position_s _home_position = {}; // Added for home position

	while (!should_exit()) {
		int ret = px4_poll(fds, sizeof(fds) / sizeof(fds[0]), 1000);

		if (!(fds[0].revents & POLLIN)) {
			// no new data
			continue;
		}

		if (ret < 0) {
			// Poll error, sleep and try again
			px4_usleep(10000);
			continue;

		} else if (ret == 0) {
			// Poll timeout or no new data, do nothing
			continue;
		}

		perf_begin(_perf_update_data);


        orb_copy(ORB_ID(sensor_combined), _sensors_sub, &sensors);

		// update all other topics if they have new data
		bool vehicle_status_updated = false;

		orb_check(_status_sub, &vehicle_status_updated);
        if (vehicle_status_updated) orb_copy(ORB_ID(vehicle_status), _status_sub, &vehicle_status);

        const hrt_abstime now = hrt_absolute_time();

		// publish attitude immediately (uses quaternion from output predictor)
        //publish_attitude(sensors, now);
		bool vehicle_land_detected_updated = false;
		orb_check(_vehicle_land_detected_sub, &vehicle_land_detected_updated);

		if (vehicle_land_detected_updated) {
            orb_copy(ORB_ID(vehicle_land_detected), _vehicle_land_detected_sub, &vehicle_land_detected);
		}

        bool serial_com_updated = false;
        orb_check(_serial_com_sub, &serial_com_updated);
        if (serial_com_updated) {
           orb_copy(ORB_ID(serial_com), _serial_com_sub, &_serial_com);
        }

        bool home_position_upodated;
        orb_check(_home_position_sub, &home_position_upodated);
        if (home_position_upodated) {
            orb_copy(ORB_ID(home_position), _home_position_sub, &_home_position);
        }

        perf_end(_perf_update_data);

        if (serial_com_updated)
        {
            // generate attitude publication
            struct vehicle_attitude_s att = {};
            matrix::Quaternion<float> q(1.0f, 0.0f, 0.0f, 0.0f);
            att.timestamp = now;

            // generate control state data
            control_state_s ctrl_state = {};
            ctrl_state.timestamp = now;

            // Input data: [deg/sec], with SF 20
            ctrl_state.roll_rate = (float)_serial_com.roll_rate*0.000872664626f;             // Converted to rad/sec (with SF=0.05)
            ctrl_state.pitch_rate = (float)_serial_com.pitch_rate*0.000872664626f;           // Converted to rad/sec (with SF=0.05)
            ctrl_state.yaw_rate = (float)_serial_com.yaw_rate*0.000872664626f;                      // Converted to rad/sec (with SF=0.05)

            // Input data: [deg], with SF 100
            att.roll = (float)_serial_com.roll_angle*0.00017532952f;  // Converted to rad (with SF=0.01)
            att.pitch = (float)_serial_com.pitch_angle*0.00017532952f; // Converted to rad (with SF=0.01)
            att.yaw = (float)_serial_com.yaw_angle*0.00017532952f;            // Converted to rad (with SF=0.01)

            att.rollspeed = ctrl_state.roll_rate;
            att.pitchspeed        = ctrl_state.pitch_rate;
            att.yawspeed   = ctrl_state.yaw_rate;

            // Publish the attitude data
            if (_att_pub == nullptr) {
                _att_pub = orb_advertise(ORB_ID(vehicle_attitude), &att);
            }
            else {
                orb_publish(ORB_ID(vehicle_attitude), _att_pub, &att);
            }

            float euler_phi               = att.roll;
            float euler_theta     = att.pitch;
            float euler_psi               = att.yaw;

            float cosPhi_2 = float(cos(euler_phi*0.5f));
            float cosTheta_2 = float(cos(euler_theta*0.5f));
            float cosPsi_2 = float(cos(euler_psi*0.5f));
            float sinPhi_2 = float(sin(euler_phi*0.5f));
            float sinTheta_2 = float(sin(euler_theta*0.5f));
            float sinPsi_2 = float(sin(euler_psi*0.5f));

            // Compute quaternion
            q(0) = cosPhi_2 * cosTheta_2 * cosPsi_2 + sinPhi_2 * sinTheta_2 * sinPsi_2;
            q(1) = sinPhi_2 * cosTheta_2 * cosPsi_2 - cosPhi_2 * sinTheta_2 * sinPsi_2;
            q(2) = cosPhi_2 * sinTheta_2 * cosPsi_2 + sinPhi_2 * cosTheta_2 * sinPsi_2;
            q(3) = cosPhi_2 * cosTheta_2 * sinPsi_2 - sinPhi_2 * sinTheta_2 * cosPsi_2;

            ctrl_state.q[0] = q(0);
            ctrl_state.q[1] = q(1);
            ctrl_state.q[2] = q(2);
            ctrl_state.q[3] = q(3);

            ctrl_state.x_acc = 0.0f;
            ctrl_state.y_acc = 0.0f;
            ctrl_state.z_acc = 9.8f;

            // generate vehicle local position data
            struct vehicle_local_position_s lpos = {};
            //float pos[3] = {};
            float vel[3] = {};
            float home_pos_LLA[3] = {};
            float pos_LLA[3] = {};
            float pos_NED[3] = {};

            home_pos_LLA[0] = math::radians(_home_position.lat);
            home_pos_LLA[1] = math::radians(_home_position.lon);
            home_pos_LLA[2] = (float)_home_position.alt;

            pos_LLA[0] = math::radians(_serial_com.latitude);
            pos_LLA[1] = math::radians(_serial_com.longitude);
            pos_LLA[2] = _serial_com.altitude;

            LLA2NED(home_pos_LLA, pos_LLA, pos_NED);

            vel[0] = (float)_serial_com.v_n*0.01f;             // Converted to m/sec (SF = 0.01)
            vel[1] = (float)_serial_com.v_e*0.01f;             // Converted to m/sec (SF = 0.01)
            vel[2] = (float)_serial_com.v_d*0.01f;             // Converted to m/sec (SF = 0.01)

            // Velocity in body frame
            Vector3f v_n(vel);
            matrix::Dcm<float> R_to_body(q.inversed());
            Vector3f v_b = R_to_body * v_n;

            ctrl_state.x_vel = v_b(0);
            ctrl_state.y_vel = v_b(1);
            ctrl_state.z_vel = v_b(2);

            // Local Position NED
            ctrl_state.x_pos = pos_LLA[0];
            ctrl_state.y_pos = pos_LLA[1];
            ctrl_state.z_pos = pos_LLA[2];

            ctrl_state.x_acc = (float)_serial_com.x_n*0.01f;
            ctrl_state.y_acc = (float)_serial_com.y_e*0.01f;
            ctrl_state.z_acc = (float)_serial_com.z_d*0.01f;

            ctrl_state.airspeed_valid = false;//true;
            //ctrl_state.airspeed                = _serial_arsp;

            if (_control_state_pub == nullptr) {
                _control_state_pub = orb_advertise(ORB_ID(control_state), &ctrl_state);
            }
            else {
                orb_publish(ORB_ID(control_state), _control_state_pub, &ctrl_state);
            }

            // Position of body origin in local NED frame
            // Generate local position data
            lpos.timestamp = now;
            lpos.x = pos_NED[0];
            lpos.y = pos_NED[1];
            lpos.z = pos_NED[2];

            lpos.vx = vel[0];
            lpos.vy = vel[1];
            lpos.vz = vel[2];

            lpos.ax = ctrl_state.x_acc;
            lpos.ay = ctrl_state.y_acc;
            lpos.az = ctrl_state.z_acc;

            // TODO: better status reporting
            lpos.xy_valid = true;
            lpos.z_valid = true;
            lpos.v_xy_valid = true;
            lpos.v_z_valid = true;

            // Position of local NED origin in GPS / WGS84 frame
            lpos.xy_global =true;// _ekf.global_position_is_valid();
            lpos.z_global = true;                                // true if z is valid and has valid global reference (ref_alt)
            lpos.ref_timestamp = now;
            lpos.ref_lat = math::degrees((double)home_pos_LLA[0]);//_serial_com.latitude;//ekf_origin.lat_rad * 180.0 / M_PI; // Reference point latitude in degrees
            lpos.ref_lon = math::degrees((double)home_pos_LLA[1]);//_serial_com.longitude;//ekf_origin.lon_rad * 180.0 / M_PI; // Reference point longitude in degrees
            lpos.ref_alt = _serial_com.altitude;

            // The rotation of the tangent plane vs. geographical north
            lpos.yaw = att.yaw;
            //float terrain_vpos;
            lpos.dist_bottom_valid = false;//ekf.get_terrain_vert_pos(&terrain_vpos);
            lpos.dist_bottom = -pos_NED[2]; // Distance to bottom surface (ground) in meters
            lpos.dist_bottom_rate = -vel[2]; // Distance to bottom surface (ground) change rate
            lpos.surface_bottom_timestamp = hrt_absolute_time(); // Time when new bottom surface found

            // TODO: uORB definition does not define what these variables are. We have assumed them to be horizontal and vertical 1-std dev accuracy in metres
            lpos.eph = 0.1f;
            lpos.epv = 0.1f;

            // convert NaN to INFINITY
            lpos.vxy_max = 100.0f;
            lpos.vz_max = 100.0f;
            lpos.hagl_min = 0.0f;
            lpos.hagl_max = 1000.0f;


            // publish vehicle local position data
            //_vehicle_local_position_pub.update();
            if (_lpos_pub == nullptr) {
                _lpos_pub = orb_advertise(ORB_ID(vehicle_local_position), &lpos);
            }
            else {
                orb_publish(ORB_ID(vehicle_local_position), _lpos_pub, &lpos);
            }

            // generate and publish global position data
            struct vehicle_global_position_s global_pos = {};
            global_pos.timestamp = now; // Time of this estimate, in microseconds since system start
            global_pos.time_utc_usec = gps.time_utc_usec; // GPS UTC timestamp in microseconds
            global_pos.lat = (double)_serial_com.latitude; // Latitude in degrees
            global_pos.lon = (double)_serial_com.longitude; // Longitude in degrees
            global_pos.alt = _serial_com.altitude;//_serial_com.altitude; // Altitude AMSL in meters

            global_pos.vel_n = lpos.vx; // Ground north velocity, m/s
            global_pos.vel_e = lpos.vy; // Ground east velocity, m/s
            global_pos.vel_d = lpos.vz; // Ground downside velocity, m/s
            global_pos.yaw = att.yaw; // Yaw in radians -PI..+PI.
            global_pos.eph = 0.1f;
            global_pos.epv = 0.1f;

            global_pos.terrain_alt_valid = true;
            global_pos.terrain_alt = (float)_home_position.alt;

            global_pos.dead_reckoning = false;
            global_pos.pressure_alt = global_pos.alt;

            //_vehicle_global_position_pub.update();
            if (_vehicle_global_position_pub == nullptr) {
                _vehicle_global_position_pub = orb_advertise(ORB_ID(vehicle_global_position), &global_pos);
            }
            else {
                orb_publish(ORB_ID(vehicle_global_position), _vehicle_global_position_pub, &global_pos);
            }
        }
    }
}

bool Ekf2::publish_attitude(const sensor_combined_s &sensors, const hrt_abstime &now)
{
	if (_ekf.attitude_valid()) {
		// generate vehicle attitude quaternion data
		vehicle_attitude_s att;
		att.timestamp = now;

		const Quatf q{_ekf.calculate_quaternion()};
		q.copyTo(att.q);

		_ekf.get_quat_reset(&att.delta_q_reset[0], &att.quat_reset_counter);

		// In-run bias estimates
		float gyro_bias[3];
		_ekf.get_gyro_bias(gyro_bias);
		att.rollspeed = sensors.gyro_rad[0] - gyro_bias[0];
		att.pitchspeed = sensors.gyro_rad[1] - gyro_bias[1];
		att.yawspeed = sensors.gyro_rad[2] - gyro_bias[2];

		int instance;
		orb_publish_auto(ORB_ID(vehicle_attitude), &_att_pub, &att, &instance, ORB_PRIO_HIGH);

		return true;

    }

	return false;
}

Ekf2 *Ekf2::instantiate(int argc, char *argv[])
{
	Ekf2 *instance = new Ekf2();

	if (instance) {
		if (argc >= 2 && !strcmp(argv[1], "-r")) {
            //instance->set_replay_mode(true);
		}
	}

	return instance;
}

int Ekf2::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int Ekf2::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Attitude and position estimator using an Extended Kalman Filter. It is used for Multirotors and Fixed-Wing.

The documentation can be found on the [ECL/EKF Overview & Tuning](https://docs.px4.io/en/advanced_config/tuning_the_ecl_ekf.html) page.

ekf2 can be started in replay mode (`-r`): in this mode it does not access the system time, but only uses the
timestamps from the sensor topics.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("ekf2", "estimator");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('r', "Enable replay mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int Ekf2::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("ekf2",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_ESTIMATOR,
				      6600,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

int ekf2_main(int argc, char *argv[])
{
	return Ekf2::main(argc, argv);
}
