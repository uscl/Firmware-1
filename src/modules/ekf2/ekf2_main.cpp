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

private:
	bool publish_attitude(const sensor_combined_s &sensors, const hrt_abstime &now);

	const Vector3f get_vel_body_wind();

	bool 	_replay_mode = false;			///< true when we use replay data from a log

	// time slip monitoring
	uint64_t _integrated_time_us = 0;	///< integral of gyro delta time from start (uSec)
	uint64_t _start_time_us = 0;		///< system time at EKF start (uSec)
	int64_t _last_time_slip_us = 0;		///< Last time slip (uSec)

	perf_counter_t _perf_update_data;
	perf_counter_t _perf_ekf_update;

	// Used to check, save and use learned magnetometer biases
    //hrt_abstime _last_magcal_us = 0;	///< last time the EKF was operating a mode that estimates magnetomer biases (uSec)
	hrt_abstime _total_cal_time_us = 0;	///< accumulated calibration time since the last save

	// set pose/velocity as invalid if standard deviation is bigger than max_std_dev
	// TODO: the user should be allowed to set these values by a parameter
	static constexpr float ep_max_std_dev = 100.0f;	///< Maximum permissible standard deviation for estimated position
	static constexpr float eo_max_std_dev = 100.0f;	///< Maximum permissible standard deviation for estimated orientation
	//static constexpr float ev_max_std_dev = 100.0f;	///< Maximum permissible standard deviation for estimated velocity

	// GPS blending and switching
	gps_message _gps_state[GPS_MAX_RECEIVERS] {}; ///< internal state data for the physical GPS
	gps_message _gps_blended_state{};		///< internal state data for the blended GPS
	gps_message _gps_output[GPS_MAX_RECEIVERS + 1] {}; ///< output state data for the physical and blended GPS
	Vector2f _NE_pos_offset_m[GPS_MAX_RECEIVERS] = {}; ///< Filtered North,East position offset from GPS instance to blended solution in _output_state.location (m)
	float _hgt_offset_mm[GPS_MAX_RECEIVERS] = {};	///< Filtered height offset from GPS instance relative to blended solution in _output_state.location (mm)
	Vector3f _blended_antenna_offset = {};		///< blended antenna offset
	float _blend_weights[GPS_MAX_RECEIVERS] = {};	///< blend weight for each GPS. The blend weights must sum to 1.0 across all instances.
	uint64_t _time_prev_us[GPS_MAX_RECEIVERS] = {};	///< the previous value of time_us for that GPS instance - used to detect new data.
	uint8_t _gps_best_index = 0;			///< index of the physical receiver with the lowest reported error
	uint8_t _gps_select_index = 0;			///< 0 = GPS1, 1 = GPS2, 2 = blended
	uint8_t _gps_time_ref_index =
		0;		///< index of the receiver that is used as the timing reference for the blending update
	uint8_t _gps_oldest_index = 0;			///< index of the physical receiver with the oldest data
	uint8_t _gps_newest_index = 0;			///< index of the physical receiver with the newest data
	uint8_t _gps_slowest_index = 0;			///< index of the physical receiver with the slowest update rate
	float _gps_dt[GPS_MAX_RECEIVERS] = {};		///< average time step in seconds.
	bool  _gps_new_output_data = false;		///< true if there is new output data for the EKF

	int32_t _gps_alttitude_ellipsoid[GPS_MAX_RECEIVERS] {};	///< altitude in 1E-3 meters (millimeters) above ellipsoid
	uint64_t _gps_alttitude_ellipsoid_previous_timestamp[GPS_MAX_RECEIVERS] {}; ///< storage for previous timestamp to compute dt
	float   _wgs84_hgt_offset = 0;  ///< height offset between AMSL and WGS84

	int _params_sub{-1};
	int _sensors_sub{-1};
	int _status_sub{-1};
	int _vehicle_land_detected_sub{-1};

	// because we can have multiple GPS instances
	int _gps_subs[GPS_MAX_RECEIVERS] {};
	int _gps_orb_instance{-1};

	orb_advert_t _att_pub{nullptr};
	orb_advert_t _wind_pub{nullptr};
	orb_advert_t _estimator_status_pub{nullptr};
	orb_advert_t _ekf2_timestamps_pub{nullptr};
//	orb_advert_t _blended_gps_pub{nullptr};

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

		bool params_updated = false;
		orb_check(_params_sub, &params_updated);

		if (params_updated) {
			// read from param to clear updated flag
			parameter_update_s update;
			orb_copy(ORB_ID(parameter_update), _params_sub, &update);
            //updateParams();
		}

		orb_copy(ORB_ID(sensor_combined), _sensors_sub, &sensors);

		// ekf2_timestamps (using 0.1 ms relative timestamps)
		ekf2_timestamps_s ekf2_timestamps = {};
		ekf2_timestamps.timestamp = sensors.timestamp;

		ekf2_timestamps.distance_sensor_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID;
		ekf2_timestamps.gps_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID;

		// update all other topics if they have new data

		bool vehicle_status_updated = false;

		orb_check(_status_sub, &vehicle_status_updated);
        if (vehicle_status_updated) orb_copy(ORB_ID(vehicle_status), _status_sub, &vehicle_status);

		const hrt_abstime now = sensors.timestamp;

		// publish attitude immediately (uses quaternion from output predictor)
		publish_attitude(sensors, now);


		// read gps1 data if available
		bool gps1_updated = false;
		orb_check(_gps_subs[0], &gps1_updated);

		if (gps1_updated) {
			vehicle_gps_position_s gps;

			if (orb_copy(ORB_ID(vehicle_gps_position), _gps_subs[0], &gps) == PX4_OK) {
				_gps_state[0].time_usec = gps.timestamp;
				_gps_state[0].lat = gps.lat;
				_gps_state[0].lon = gps.lon;
				_gps_state[0].alt = gps.alt;
				_gps_state[0].yaw = gps.heading;
				_gps_state[0].yaw_offset = gps.heading_offset;
				_gps_state[0].fix_type = gps.fix_type;
				_gps_state[0].eph = gps.eph;
				_gps_state[0].epv = gps.epv;
				_gps_state[0].sacc = gps.s_variance_m_s;
				_gps_state[0].vel_m_s = gps.vel_m_s;
				_gps_state[0].vel_ned[0] = gps.vel_n_m_s;
				_gps_state[0].vel_ned[1] = gps.vel_e_m_s;
				_gps_state[0].vel_ned[2] = gps.vel_d_m_s;
				_gps_state[0].vel_ned_valid = gps.vel_ned_valid;
				_gps_state[0].nsats = gps.satellites_used;
				//TODO: add gdop to gps topic
				_gps_state[0].gdop = 0.0f;
				_gps_alttitude_ellipsoid[0] = gps.alt_ellipsoid;

				ekf2_timestamps.gps_timestamp_rel = (int16_t)((int64_t)gps.timestamp / 100 - (int64_t)ekf2_timestamps.timestamp / 100);
			}
		}

		// check for second GPS receiver data
		bool gps2_updated = false;
		orb_check(_gps_subs[1], &gps2_updated);

		if (gps2_updated) {
			vehicle_gps_position_s gps;

			if (orb_copy(ORB_ID(vehicle_gps_position), _gps_subs[1], &gps) == PX4_OK) {
				_gps_state[1].time_usec = gps.timestamp;
				_gps_state[1].lat = gps.lat;
				_gps_state[1].lon = gps.lon;
				_gps_state[1].alt = gps.alt;
				_gps_state[1].yaw = gps.heading;
				_gps_state[1].yaw_offset = gps.heading_offset;
				_gps_state[1].fix_type = gps.fix_type;
				_gps_state[1].eph = gps.eph;
				_gps_state[1].epv = gps.epv;
				_gps_state[1].sacc = gps.s_variance_m_s;
				_gps_state[1].vel_m_s = gps.vel_m_s;
				_gps_state[1].vel_ned[0] = gps.vel_n_m_s;
				_gps_state[1].vel_ned[1] = gps.vel_e_m_s;
				_gps_state[1].vel_ned[2] = gps.vel_d_m_s;
				_gps_state[1].vel_ned_valid = gps.vel_ned_valid;
				_gps_state[1].nsats = gps.satellites_used;
				//TODO: add gdop to gps topic
				_gps_state[1].gdop = 0.0f;
				_gps_alttitude_ellipsoid[1] = gps.alt_ellipsoid;
			}
		}

        // Only use selected receiver data if it has been updated
        if ((gps1_updated && _gps_select_index == 0) || (gps2_updated && _gps_select_index == 1)) {
            _gps_new_output_data = true;
        } else {
					_gps_new_output_data = false;
        }

		bool vehicle_land_detected_updated = false;
		orb_check(_vehicle_land_detected_sub, &vehicle_land_detected_updated);

		if (vehicle_land_detected_updated) {
			if (orb_copy(ORB_ID(vehicle_land_detected), _vehicle_land_detected_sub, &vehicle_land_detected) == PX4_OK) {
				_ekf.set_in_air_status(!vehicle_land_detected.landed);
			}
		}

		perf_end(_perf_update_data);

        if (1) {

            //if (control_status.flags.tilt_align)
            if (1)
            {
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
				_ekf.get_vel_deriv_ned(vel_deriv);
				lpos.ax = vel_deriv[0];
				lpos.ay = vel_deriv[1];
				lpos.az = vel_deriv[2];

				// TODO: better status reporting
                lpos.xy_valid = true;
                lpos.z_valid = true;
                lpos.v_xy_valid = true;
                lpos.v_z_valid = true;

				// Position of local NED origin in GPS / WGS84 frame
				map_projection_reference_s ekf_origin;
				uint64_t origin_time;

				// true if position (x,y,z) has a valid WGS-84 global reference (ref_lat, ref_lon, alt)
				const bool ekf_origin_valid = _ekf.get_ekf_origin(&origin_time, &ekf_origin, &lpos.ref_alt);
				lpos.xy_global = ekf_origin_valid;
				lpos.z_global = ekf_origin_valid;

				if (ekf_origin_valid && (origin_time > lpos.ref_timestamp)) {
					lpos.ref_timestamp = origin_time;
					lpos.ref_lat = ekf_origin.lat_rad * 180.0 / M_PI; // Reference point latitude in degrees
					lpos.ref_lon = ekf_origin.lon_rad * 180.0 / M_PI; // Reference point longitude in degrees
				}

				// The rotation of the tangent plane vs. geographical north
				matrix::Quatf q;
				_ekf.copy_quaternion(q.data());

				lpos.yaw = matrix::Eulerf(q).psi();

				lpos.dist_bottom_valid = _ekf.get_terrain_valid();

				float terrain_vpos;
				_ekf.get_terrain_vert_pos(&terrain_vpos);
				lpos.dist_bottom = terrain_vpos - lpos.z; // Distance to bottom surface (ground) in meters

				// constrain the distance to ground to _rng_gnd_clearance
            //	if (lpos.dist_bottom < _rng_gnd_clearance.get()) {
                    lpos.dist_bottom = 0;//_rng_gnd_clearance.get();
                //}

				lpos.dist_bottom_rate = -lpos.vz; // Distance to bottom surface (ground) change rate

				_ekf.get_ekf_lpos_accuracy(&lpos.eph, &lpos.epv);
				_ekf.get_ekf_vel_accuracy(&lpos.evh, &lpos.evv);

				// get state reset information of position and velocity
				_ekf.get_posD_reset(&lpos.delta_z, &lpos.z_reset_counter);
				_ekf.get_velD_reset(&lpos.delta_vz, &lpos.vz_reset_counter);
				_ekf.get_posNE_reset(&lpos.delta_xy[0], &lpos.xy_reset_counter);
				_ekf.get_velNE_reset(&lpos.delta_vxy[0], &lpos.vxy_reset_counter);

				// get control limit information
				_ekf.get_ekf_ctrl_limits(&lpos.vxy_max, &lpos.vz_max, &lpos.hagl_min, &lpos.hagl_max);

				// convert NaN to INFINITY
                lpos.vxy_max = INFINITY;

                lpos.vz_max = INFINITY;
                lpos.hagl_min = INFINITY;
                lpos.hagl_max = INFINITY;

				// publish vehicle local position data
				_vehicle_local_position_pub.update();


                if (_ekf.global_position_is_valid()) {
					// generate and publish global position data
					vehicle_global_position_s &global_pos = _vehicle_global_position_pub.get();

					global_pos.timestamp = now;

					if (fabsf(lpos_x_prev - lpos.x) > FLT_EPSILON || fabsf(lpos_y_prev - lpos.y) > FLT_EPSILON) {
						map_projection_reproject(&ekf_origin, lpos.x, lpos.y, &global_pos.lat, &global_pos.lon);
					}

					global_pos.lat_lon_reset_counter = lpos.xy_reset_counter;

					global_pos.alt = -lpos.z + lpos.ref_alt; // Altitude AMSL in meters
                    global_pos.alt_ellipsoid = 0.0;//filter_altitude_ellipsoid(global_pos.alt);

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
            status.control_mode_flags = 1;//control_status.value;
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
            status.pre_flt_fail = false;

			if (_estimator_status_pub == nullptr) {
				_estimator_status_pub = orb_advertise(ORB_ID(estimator_status), &status);

			} else {
				orb_publish(ORB_ID(estimator_status), _estimator_status_pub, &status);
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
