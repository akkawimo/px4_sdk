/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#include "px4_sdk/motion_planning/PositionSmoothing.hpp"
#include "px4_sdk/motion_planning/TrajectoryConstraints.hpp"

void PositionSmoothing::_generateSmoothSetpoints(
	const Eigen::Vector3f &local_position,
	const Eigen::Vector3f &target,
	const double timestamp,
	px4_msgs::msg::TrajectorySetpoint &out_setpoints)
{
	Eigen::Vector3f velocity_setpoint{0.f, 0.f, 0.f};

	for (int i = 0; i < target.size(); ++i)
	{
		if (!std::isfinite(target[i]))
		{
			return;
		}
	}

	velocity_setpoint = _generateUnsmoothedVelocitySetpoint(target);

	//std::cout << "Velocity Unsmoothed Setpoint: " << velocity_setpoint << std::endl;

	for (int i = 0; i < velocity_setpoint.size(); ++i)
	{
		if (!std::isfinite(velocity_setpoint[i]))
		{

			//std::cout << "Velocity Unsmoothed Setpoint Is not finite";
			return;
		}
	}

	/* Slow down the trajectory by decreasing the integration time based on the position error.
	 * This is only performed when the drone is behind the trajectory
	 */
	Eigen::Vector2f position_trajectory_xy(_trajectory[0].getCurrentPosition(), _trajectory[1].getCurrentPosition());
	Eigen::Vector2f local_position_xy(local_position.head<2>());
	Eigen::Vector2f vel_traj_xy(_trajectory[0].getCurrentVelocity(), _trajectory[1].getCurrentVelocity());
	Eigen::Vector2f drone_to_trajectory_xy(position_trajectory_xy - local_position_xy);
	float local_position_xy_error = drone_to_trajectory_xy.norm();

	float time_stretch = 1.f - std::clamp(local_position_xy_error / _max_allowed_horizontal_error, 0.f, 1.f);

	// Don't stretch time if the drone is ahead of the position setpoint
	if (drone_to_trajectory_xy.dot(vel_traj_xy) < 0.f)
	{
		time_stretch = 1.f;
	}

	const float delta_time  = math::min((timestamp - _last_update), _timeout) / 1e6f;

	_last_update = timestamp;

	//std::cout << "timestamp PS: " << timestamp << std::endl;
	//std::cout << "_last_update: " << _last_update << std::endl;
	//std::cout << "time_stretch: " << time_stretch << std::endl;
	//std::cout << "delta_time: " << delta_time << std::endl;

	for (int i = 0; i < 3; ++i)
	{
		_trajectory[i].updateTraj(delta_time, time_stretch);
		out_setpoints.jerk[i] = _trajectory[i].getCurrentJerk();
		out_setpoints.acceleration[i] = _trajectory[i].getCurrentAcceleration();
		out_setpoints.velocity[i] = _trajectory[i].getCurrentVelocity();
		out_setpoints.position[i] = _trajectory[i].getCurrentPosition();
	}

	for (int i = 0; i < 3; ++i)
	{
		_trajectory[i].updateDurations(velocity_setpoint(i));
	}

	VelocitySmoothing::timeSynchronization(_trajectory, 3);

}

float PositionSmoothing::_getMaxXYSpeed(const Eigen::Vector3f &target) const
{
	Eigen::Vector3f pos_traj(_trajectory[0].getCurrentPosition(),
							 _trajectory[1].getCurrentPosition(),
							 _trajectory[2].getCurrentPosition());

	math::trajectory::VehicleDynamicLimits config;
	config.z_accept_rad = _vertical_acceptance_radius;			  //_param_nav_mc_alt_rad
	config.xy_accept_rad = _target_acceptance_radius;			  // from triplet
	config.max_acc_xy = _trajectory[0].getMaxAccel();			  //_param_mpc_acc_hor
	config.max_jerk = _trajectory[0].getMaxJerk();				  //_param_mpc_jerk_auto
	config.max_speed_xy = _cruise_speed;						  // from triplet
	config.max_acc_xy_radius_scale = _horizontal_trajectory_gain; //_param_mpc_xy_traj_p

	Eigen::Vector3f pos_to_target[2] = {pos_traj, target};

	return math::trajectory::computeXYSpeedFromWaypoints<2>(pos_to_target, config);
}

float PositionSmoothing::_getMaxZSpeed(const Eigen::Vector3f &target) const
{

	Eigen::Vector3f pos_traj(_trajectory[0].getCurrentPosition(),
							 _trajectory[1].getCurrentPosition(),
							 _trajectory[2].getCurrentPosition());

	const float distance_start_target = fabs(target(2) - pos_traj(2));
	const float arrival_z_speed = 0.f;

	float max_speed = std::min(_trajectory[2].getMaxVel(), math::trajectory::computeMaxSpeedFromDistance(
															   _trajectory[2].getMaxJerk(), _trajectory[2].getMaxAccel(),
															   distance_start_target, arrival_z_speed));

	return max_speed;
}

const Eigen::Vector3f PositionSmoothing::_generateUnsmoothedVelocitySetpoint(const Eigen::Vector3f &target)
{

	Eigen::Vector3f pos_traj(_trajectory[0].getCurrentPosition(),
							 _trajectory[1].getCurrentPosition(),
							 _trajectory[2].getCurrentPosition());
	Eigen::Vector3f vec{(target - pos_traj)};
	Eigen::Vector3f u_pos_traj_to_dest;
	if (vec.norm() > std::numeric_limits<float>::epsilon())
	{
		u_pos_traj_to_dest = vec.normalized();
	}
	else
	{
		u_pos_traj_to_dest = Eigen::Vector3f::Zero();
	}

	float xy_speed = _getMaxXYSpeed(target);
	const float z_speed = _getMaxZSpeed(target);

	Eigen::Vector3f velocity_setpoint = u_pos_traj_to_dest * sqrt(xy_speed * xy_speed + z_speed * z_speed);
	math::trajectory::clampToXYNorm(velocity_setpoint, xy_speed, 0.5f);
	math::trajectory::clampToZNorm(velocity_setpoint, z_speed, 0.5f);

	return velocity_setpoint;
}
