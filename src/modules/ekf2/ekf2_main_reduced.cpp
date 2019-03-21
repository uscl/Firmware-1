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
#include <uORB/topics/ekf_gps_drift.h>
#include <uORB/topics/landing_target_pose.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_bias.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/sensor_selection.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_magnetometer.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/wind_estimate.h>
#include <uORB/topics/serial_com.h>
#include <uORB/topics/home_position.h> // Added for home position polling

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

	void set_replay_mode(bool replay) { _replay_mode = replay; }

	static void	task_main_trampoline(int argc, char *argv[]);

	int print_status() override;

	void LLA2NED(float *home_pos_LLA, float *cur_pos_LLA, float *result);

private:
	template<typename Param>
	bool publish_attitude(const sensor_combined_s &sensors, const hrt_abstime &now);

	bool 	_replay_mode = false;			///< true when we use replay data from a log

	// time slip monitoring
	uint64_t _integrated_time_us = 0;	///< integral of gyro delta time from start (uSec)
	uint64_t _start_time_us = 0;		///< system time at EKF start (uSec)
	int64_t _last_time_slip_us = 0;		///< Last time slip (uSec)

	perf_counter_t _perf_update_data;
	perf_counter_t _perf_ekf_update;

	// set pose/velocity as invalid if standard deviation is bigger than max_std_dev
	// TODO: the user should be allowed to set these values by a parameter
	static constexpr float ep_max_std_dev = 100.0f;	///< Maximum permissible standard deviation for estimated position
	static constexpr float eo_max_std_dev = 100.0f;	///< Maximum permissible standard deviation for estimated orientation
	//static constexpr float ev_max_std_dev = 100.0f;	///< Maximum permissible standard deviation for estimated velocity

	int _params_sub{-1};
	int _sensors_sub{-1};
	int _serial_com_sub{-1};
	int _status_sub{-1};
	int _vehicle_land_detected_sub{-1};
	int _home_position_sub{-1};	// Added for Home position

	// because we can have multiple GPS instances
	int _gps_subs[GPS_MAX_RECEIVERS] {};
	int _gps_orb_instance{-1};

	orb_advert_t _att_pub{nullptr};
	orb_advert_t _wind_pub{nullptr};
	orb_advert_t _estimator_status_pub{nullptr};
	orb_advert_t _ekf_gps_drift_pub{nullptr};
	orb_advert_t _estimator_innovations_pub{nullptr};
	orb_advert_t _ekf2_timestamps_pub{nullptr};

	uORB::Publication<vehicle_local_position_s> _vehicle_local_position_pub;
	uORB::Publication<vehicle_global_position_s> _vehicle_global_position_pub;
	uORB::Publication<vehicle_odometry_s> _vehicle_odometry_pub;

	Ekf _ekf;

	parameters *_params;	///< pointer to ekf parameter struct (located in _ekf class instance)

};

Ekf2::Ekf2():
	ModuleParams(nullptr),
	_perf_update_data(perf_alloc_once(PC_ELAPSED, "EKF2 data acquisition")),
	_perf_ekf_update(perf_alloc_once(PC_ELAPSED, "EKF2 update")),
	_vehicle_local_position_pub(ORB_ID(vehicle_local_position)),
    _vehicle_global_position_pub(ORB_ID(vehicle_global_position))
{
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_sensors_sub = orb_subscribe(ORB_ID(sensor_combined));
	_serial_com_sub = orb_subscribe(ORB_ID(serial_com));
	_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	_vehicle_land_detected_sub = orb_subscribe(ORB_ID(vehicle_land_detected));
	_home_position_sub = orb_subscribe(ORB_ID(home_position)); // Added for Home position

	for (unsigned i = 0; i < GPS_MAX_RECEIVERS; i++) {
		_gps_subs[i] = orb_subscribe_multi(ORB_ID(vehicle_gps_position), i);
	}

	// initialise parameter cache
	updateParams();
}

Ekf2::~Ekf2()
{
	perf_free(_perf_update_data);
	perf_free(_perf_ekf_update);

	orb_unsubscribe(_params_sub);
	orb_unsubscribe(_sensors_sub);
	orb_unsubscribe(_serial_com_sub);
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

//	return _position_NED;
}

template<typename Param>
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
	sensor_selection_s sensor_selection = {};

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

		bool serial_com_updated = false;
		orb_check(_serial_com_sub, &serial_com_updated);
		if (serial_com_updated)	orb_copy(ORB_ID(serial_com), _serial_com_sub, &_serial_com);

		bool home_position_upodated = false;
		orb_check(_home_position_sub, &home_position_upodated);
		if (home_position_upodated) orb_copy(ORB_ID(home_position), _home_position_sub, &_home_position);

		const hrt_abstime now = sensors.timestamp;

		// publish attitude immediately (uses quaternion from output predictor)
//		publish_attitude(sensors, now);

		// read gps1 data if available
		bool gps1_updated = false;
		orb_check(_gps_subs[0], &gps1_updated);

		bool vehicle_land_detected_updated = false;
		orb_check(_vehicle_land_detected_sub, &vehicle_land_detected_updated);
		if (vehicle_land_detected_updated) {
			orb_copy(ORB_ID(vehicle_land_detected), _vehicle_land_detected_sub, &vehicle_land_detected);
		}

		perf_end(_perf_update_data);
/*
		if (serial_com_updated)
		{
			// generate attitude publication
			struct vehicle_attitude_s att = {};
			matrix::Quaternion<float> q(1.0f, 0.0f, 0.0f, 0.0f);
			att.timestamp = hrt_absolute_time();

			// generate control state data
			control_state_s ctrl_state = {};
			ctrl_state.timestamp = hrt_absolute_time();

			// Input data: [deg/sec], with SF 20
			ctrl_state.roll_rate = (float)_serial_com.roll_rate*0.000872664626f;		// Converted to rad/sec (with SF=0.05)
			ctrl_state.pitch_rate = (float)_serial_com.pitch_rate*0.000872664626f;		// Converted to rad/sec (with SF=0.05)
			ctrl_state.yaw_rate = (float)_serial_com.yaw_rate*0.000872664626f;			// Converted to rad/sec (with SF=0.05)

			// Input data: [deg], with SF 100
			att.roll = (float)_serial_com.roll_angle*0.00017532952f;	// Converted to rad (with SF=0.01)
			att.pitch = (float)_serial_com.pitch_angle*0.00017532952f;	// Converted to rad (with SF=0.01)
			att.yaw = (float)_serial_com.yaw_angle*0.00017532952f; 		// Converted to rad (with SF=0.01)

			att.rollspeed 	= ctrl_state.roll_rate;
			att.pitchspeed 	= ctrl_state.pitch_rate;
			att.yawspeed 	= ctrl_state.yaw_rate;

			// Publish the attitude data
			if (_att_pub == nullptr)	_att_pub = orb_advertise(ORB_ID(vehicle_attitude), &att);
			else 	orb_publish(ORB_ID(vehicle_attitude), _att_pub, &att);

			float euler_phi		= att.roll;
			float euler_theta	= att.pitch;
			float euler_psi		= att.yaw;

			float cosPhi_2 = float(cos(euler_phi*0.5f));
			float cosTheta_2 = float(cos(euler_theta*0.5f));
			float cosPsi_2 = float(cos(euler_psi*0.5f));
			float sinPhi_2 = float(sin(euler_phi*0.5f));
			float sinTheta_2 = float(sin(euler_theta*0.5f));
			float sinPsi_2 = float(sin(euler_psi*0.5f));
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

			vel[0] = (float)_serial_com.v_n*0.01f;		// Converted to m/sec (SF = 0.01)
			vel[1] = (float)_serial_com.v_e*0.01f;		// Converted to m/sec (SF = 0.01)
			vel[2] = (float)_serial_com.v_d*0.01f;		// Converted to m/sec (SF = 0.01)

			// Velocity in body frame
			//float velocity[3];
			//_ekf.get_velocity(velocity);
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
			ctrl_state.x_acc = (float)_serial_com.y_e*0.01f;
			ctrl_state.x_acc = (float)_serial_com.z_d*0.01f;

			ctrl_state.airspeed_valid = false;//true;
			//ctrl_state.airspeed 		= _serial_arsp;

			if (_control_state_pub == nullptr) 	_control_state_pub = orb_advertise(ORB_ID(control_state), &ctrl_state);
			else 	orb_publish(ORB_ID(control_state), _control_state_pub, &ctrl_state);

			// Position of body origin in local NED frame
			// Generate local position data
			lpos.timestamp = hrt_absolute_time();
			lpos.x = pos_NED[0];
			lpos.y = pos_NED[1];
			lpos.z = pos_NED[2];

			lpos.vx = vel[0];
			lpos.vy = vel[1];
			lpos.vz = vel[2];

			// TODO: better status reporting
			lpos.xy_valid = true;
			lpos.z_valid = true;
			lpos.v_xy_valid = true;
			lpos.v_z_valid = true;

			// Position of local NED origin in GPS / WGS84 frame
			// true if position (x, y) is valid and has valid global reference (ref_lat, ref_lon)
			lpos.xy_global =true;// _ekf.global_position_is_valid();
			lpos.z_global = true;                                // true if z is valid and has valid global reference (ref_alt)
			lpos.ref_lat = math::degrees((double)home_pos_LLA[0]);//_serial_com.latitude;//ekf_origin.lat_rad * 180.0 / M_PI; // Reference point latitude in degrees
			lpos.ref_lon = math::degrees((double)home_pos_LLA[1]);//_serial_com.longitude;//ekf_origin.lon_rad * 180.0 / M_PI; // Reference point longitude in degrees

			// The rotation of the tangent plane vs. geographical north
			lpos.yaw = att.yaw;

			//float terrain_vpos;
			lpos.dist_bottom_valid = false;//ekf.get_terrain_vert_pos(&terrain_vpos);
			//lpos.dist_bottom = terrain_vpos - pos[2]; // Distance to bottom surface (ground) in meters
			lpos.dist_bottom_rate = -vel[2]; // Distance to bottom surface (ground) change rate
			lpos.surface_bottom_timestamp	= hrt_absolute_time(); // Time when new bottom surface found

			// TODO: uORB definition does not define what these variables are. We have assumed them to be horizontal and vertical 1-std dev accuracy in metres
			Vector3f pos_var, vel_var;
			lpos.eph = 0.1f;
			lpos.epv = 0.1f;

			// publish vehicle local position data
			if (_lpos_pub == nullptr) {
				_lpos_pub = orb_advertise(ORB_ID(vehicle_local_position), &lpos);
			} else {
				orb_publish(ORB_ID(vehicle_local_position), _lpos_pub, &lpos);
			}

			// generate and publish global position data
			struct vehicle_global_position_s global_pos = {};

			global_pos.timestamp = hrt_absolute_time(); // Time of this estimate, in microseconds since system start
			global_pos.time_utc_usec = gps.time_utc_usec; // GPS UTC timestamp in microseconds

			global_pos.lat = (double)_serial_com.latitude; // Latitude in degrees
			global_pos.lon = (double)_serial_com.longitude; // Longitude in degrees
			global_pos.alt = _serial_com.altitude;//_serial_com.altitude; // Altitude AMSL in meters

			global_pos.vel_n = vel[0]; // Ground north velocity, m/s
			global_pos.vel_e = vel[1]; // Ground east velocity, m/s
			global_pos.vel_d = vel[2]; // Ground downside velocity, m/s
			global_pos.yaw = att.yaw; // Yaw in radians -PI..+PI.
			global_pos.eph = 1.0f;
			global_pos.epv = 1.0f;

			global_pos.dead_reckoning = false;
			global_pos.pressure_alt = global_pos.alt;

			if (_vehicle_global_position_pub == nullptr)_vehicle_global_position_pub = orb_advertise(ORB_ID(vehicle_global_position), &global_pos);
			else	orb_publish(ORB_ID(vehicle_global_position), _vehicle_global_position_pub, &global_pos);

		}
*/
		if (updated) {

				// generate vehicle local position data
				vehicle_local_position_s &lpos = _vehicle_local_position_pub.get();

				lpos.timestamp = now;

				// Position of body origin in local NED frame
				float position[3];
				_ekf.get_position(position);
				const float lpos_x_prev = lpos.x;
				const float lpos_y_prev = lpos.y;
				lpos.x = (_ekf.local_position_is_valid()) ? position[0] : 0.0f;
				lpos.y = (_ekf.local_position_is_valid()) ? position[1] : 0.0f;
				lpos.z = position[2];

				// Velocity of body origin in local NED frame (m/s)
				float velocity[3];
				_ekf.get_velocity(velocity);
				lpos.vx = velocity[0];
				lpos.vy = velocity[1];
				lpos.vz = velocity[2];

				// Acceleration of body origin in local NED frame
				float vel_deriv[3];
				lpos.ax = vel_deriv[0];
				lpos.ay = vel_deriv[1];
				lpos.az = vel_deriv[2];

				// TODO: better status reporting
				lpos.xy_valid = _ekf.local_position_is_valid() && !_preflt_horiz_fail;
				lpos.z_valid = !_preflt_vert_fail;
				lpos.v_xy_valid = _ekf.local_position_is_valid() && !_preflt_horiz_fail;
				lpos.v_z_valid = !_preflt_vert_fail;

				// true if position (x,y,z) has a valid WGS-84 global reference (ref_lat, ref_lon, alt)
				lpos.xy_global = ekf_origin_valid;
				lpos.z_global = ekf_origin_valid;

				lpos.ref_timestamp = origin_time;
				lpos.ref_lat = ekf_origin.lat_rad * 180.0 / M_PI; // Reference point latitude in degrees
				lpos.ref_lon = ekf_origin.lon_rad * 180.0 / M_PI; // Reference point longitude in degrees

				// The rotation of the tangent plane vs. geographical north
				lpos.yaw = matrix::Eulerf(q).psi();

				float terrain_vpos;
				lpos.dist_bottom = terrain_vpos - lpos.z; // Distance to bottom surface (ground) in meters
				lpos.dist_bottom_rate = -lpos.vz; // Distance to bottom surface (ground) change rate

				_ekf.get_ekf_lpos_accuracy(&lpos.eph, &lpos.epv);
				_ekf.get_ekf_vel_accuracy(&lpos.evh, &lpos.evv);

				// get state reset information of position and velocity
				_ekf.get_posD_reset(&lpos.delta_z, &lpos.z_reset_counter);
				_ekf.get_velD_reset(&lpos.delta_vz, &lpos.vz_reset_counter);
				_ekf.get_posNE_reset(&lpos.delta_xy[0], &lpos.xy_reset_counter);
				_ekf.get_velNE_reset(&lpos.delta_vxy[0], &lpos.vxy_reset_counter);

				// get control limit information
				lpos.vxy_max = INFINITY;
				lpos.vz_max = INFINITY;
				lpos.hagl_min = INFINITY;
				lpos.hagl_max = INFINITY;

				// publish vehicle local position data
				_vehicle_local_position_pub.update();

				if (_ekf.global_position_is_valid() && !_preflt_fail) {
					// generate and publish global position data
					vehicle_global_position_s &global_pos = _vehicle_global_position_pub.get();

					global_pos.timestamp = now;

					if (fabsf(lpos_x_prev - lpos.x) > FLT_EPSILON || fabsf(lpos_y_prev - lpos.y) > FLT_EPSILON) {
						map_projection_reproject(&ekf_origin, lpos.x, lpos.y, &global_pos.lat, &global_pos.lon);
					}

					global_pos.lat_lon_reset_counter = lpos.xy_reset_counter;

					global_pos.alt = -lpos.z + lpos.ref_alt; // Altitude AMSL in meters
					global_pos.alt_ellipsoid = filter_altitude_ellipsoid(global_pos.alt);

					// global altitude has opposite sign of local down position
					global_pos.delta_alt = -lpos.delta_z;

					global_pos.vel_n = lpos.vx; // Ground north velocity, m/s
					global_pos.vel_e = lpos.vy; // Ground east velocity, m/s
					global_pos.vel_d = lpos.vz; // Ground downside velocity, m/s

					global_pos.yaw = lpos.yaw; // Yaw in radians -PI..+PI.

					_ekf.get_ekf_gpos_accuracy(&global_pos.eph, &global_pos.epv);

					global_pos.terrain_alt_valid = lpos.dist_bottom_valid;

					if (global_pos.terrain_alt_valid) {
						global_pos.terrain_alt = lpos.ref_alt - terrain_vpos; // Terrain altitude in m, WGS84

					} else {
						global_pos.terrain_alt = 0.0f; // Terrain altitude in m, WGS84
					}

					global_pos.dead_reckoning = _ekf.inertial_dead_reckoning(); // True if this position is estimated through dead-reckoning

					_vehicle_global_position_pub.update();
				}
			}

			{
				// publish all corrected sensor readings and bias estimates after mag calibration is updated above

			}

			// publish estimator status
			estimator_status_s status;
			status.timestamp = now;
			_ekf.get_state_delayed(status.states);
			status.n_states = 24;
			_ekf.covariances_diagonal().copyTo(status.covariances);
			_ekf.get_gps_check_status(&status.gps_check_fail_flags);
			// only report enabled GPS check failures (the param indexes are shifted by 1 bit, because they don't include
			// the GPS Fix bit, which is always checked)
			status.gps_check_fail_flags &= ((uint16_t)_params->gps_check_mask << 1) | 1;
			status.control_mode_flags = control_status.value;
			_ekf.get_filter_fault_status(&status.filter_fault_flags);
			_ekf.get_innovation_test_status(&status.innovation_check_flags, &status.mag_test_ratio,
							&status.vel_test_ratio, &status.pos_test_ratio,
							&status.hgt_test_ratio, &status.tas_test_ratio,
							&status.hagl_test_ratio, &status.beta_test_ratio);

			status.pos_horiz_accuracy = _vehicle_local_position_pub.get().eph;
			status.pos_vert_accuracy = _vehicle_local_position_pub.get().epv;
			_ekf.get_ekf_soln_status(&status.solution_status_flags);
			_ekf.get_imu_vibe_metrics(status.vibe);
			status.time_slip = _last_time_slip_us / 1e6f;
			status.health_flags = 0.0f; // unused
			status.timeout_flags = 0.0f; // unused
			status.pre_flt_fail = _preflt_fail;

			if (_estimator_status_pub == nullptr) {
				_estimator_status_pub = orb_advertise(ORB_ID(estimator_status), &status);

			} else {
				orb_publish(ORB_ID(estimator_status), _estimator_status_pub, &status);
			}

			// publish GPS drift data only when updated to minimise overhead
			{
				// Check and save the last valid calibration when we are disarmed
				if ((vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_STANDBY)
				    && (status.filter_fault_flags == 0)
				    && (sensor_selection.mag_device_id == (uint32_t)_mag_bias_id.get())) {

					// reset to prevent data being saved too frequently
					_total_cal_time_us = 0;
				}

			}

			{
				// publish estimator innovation data
				if (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_STANDBY) {
					// set the max allowed yaw innovaton depending on whether we are not aiding navigation using
					// observations in the NE reference frame.
					bool doing_ne_aiding = control_status.flags.gps ||  control_status.flags.ev_pos;

					float yaw_test_limit;

					if (doing_ne_aiding && vehicle_status.is_rotary_wing) {
						// use a smaller tolerance when doing NE inertial frame aiding as a rotary wing
						// vehicle which cannot use GPS course to realign heading in flight
						yaw_test_limit = _nav_yaw_innov_test_lim;

					} else {
						// use a larger tolerance when not doing NE inertial frame aiding or
						// if a fixed wing vehicle which can realign heading using GPS course
						yaw_test_limit = _yaw_innov_test_lim;
					}

					// filter the yaw innovations
					_yaw_innov_magnitude_lpf = beta * _yaw_innov_magnitude_lpf + alpha * constrain(innovations.heading_innov,
								   -2.0f * yaw_test_limit, 2.0f * yaw_test_limit);

					_hgt_innov_lpf = beta * _hgt_innov_lpf + alpha * constrain(innovations.vel_pos_innov[5], -_hgt_innov_spike_lim,
							 _hgt_innov_spike_lim);

					// check the yaw and horizontal velocity innovations
					float vel_ne_innov_length = sqrtf(innovations.vel_pos_innov[0] * innovations.vel_pos_innov[0] +
									  innovations.vel_pos_innov[1] * innovations.vel_pos_innov[1]);
					_preflt_horiz_fail = (_vel_ne_innov_lpf.norm() > _vel_innov_test_lim)
							     || (vel_ne_innov_length > 2.0f * _vel_innov_test_lim)
							     || (_yaw_innov_magnitude_lpf > yaw_test_limit);

					// check the vertical velocity and position innovations
					_preflt_vert_fail = (fabsf(_vel_d_innov_lpf) > _vel_innov_test_lim)
							    || (fabsf(innovations.vel_pos_innov[2]) > 2.0f * _vel_innov_test_lim)
							    || (fabsf(_hgt_innov_lpf) > _hgt_innov_test_lim);

					// master pass-fail status
					_preflt_fail = _preflt_horiz_fail || _preflt_vert_fail;

				} else {
					_vel_ne_innov_lpf.zero();
					_vel_d_innov_lpf = 0.0f;
					_hgt_innov_lpf = 0.0f;
					_preflt_horiz_fail = false;
					_preflt_vert_fail = false;
					_preflt_fail = false;
				}

				if (_estimator_innovations_pub == nullptr) {
					_estimator_innovations_pub = orb_advertise(ORB_ID(ekf2_innovations), &innovations);

				} else {
					orb_publish(ORB_ID(ekf2_innovations), _estimator_innovations_pub, &innovations);
				}
			}

		}

		// publish ekf2_timestamps
		if (_ekf2_timestamps_pub == nullptr) {
			_ekf2_timestamps_pub = orb_advertise(ORB_ID(ekf2_timestamps), &ekf2_timestamps);

		} else {
			orb_publish(ORB_ID(ekf2_timestamps), _ekf2_timestamps_pub, &ekf2_timestamps);
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

	}  else if (_replay_mode) {
		// in replay mode we have to tell the replay module not to wait for an update
		// we do this by publishing an attitude with zero timestamp
		vehicle_attitude_s att = {};

		int instance;
		orb_publish_auto(ORB_ID(vehicle_attitude), &_att_pub, &att, &instance, ORB_PRIO_HIGH);
	}

	return false;
}

const Vector3f Ekf2::get_vel_body_wind()
{
	// Used to correct baro data for positional errors

	matrix::Quatf q;
	_ekf.copy_quaternion(q.data());
	matrix::Dcmf R_to_body(q.inversed());

	// Calculate wind-compensated velocity in body frame
	// Velocity of body origin in local NED frame (m/s)
	float velocity[3];
	_ekf.get_velocity(velocity);

	float velNE_wind[2];
	_ekf.get_wind_velocity(velNE_wind);

	Vector3f v_wind_comp = {velocity[0] - velNE_wind[0], velocity[1] - velNE_wind[1], velocity[2]};

	return R_to_body * v_wind_comp;
}


Ekf2 *Ekf2::instantiate(int argc, char *argv[])
{
	Ekf2 *instance = new Ekf2();

	if (instance) {
		if (argc >= 2 && !strcmp(argv[1], "-r")) {
			instance->set_replay_mode(true);
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
