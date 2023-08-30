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

#pragma once

#include <cmath>
#include "VelocitySmoothing.hpp"
#include "px4_sdk/components/setpoints.h"
#include "px4_sdk/mathlib/mathlib.h"

#include <Eigen/Core>

#define PX4_ERROR (-1)
#define PX4_OK 0

/**
 * @brief Class that generates setpoints for a smooth trajectory
 * to position waypoints.
 *
 * This is achieved by first generating an unsmoothed velocity setpoint
 * which then gets smoothed using the VelocitySmoothing library.
 */
class PositionSmoothing
{

public:
	/**
	 * @brief Reset internal state to the given values
	 *
	 * @param acceleration current acceleration
	 * @param velocity current velocity
	 * @param position current position
	 */
	void reset(const Eigen::Vector3f &acceleration, const Eigen::Vector3f &velocity, const Eigen::Vector3f &position)
	{
		for (size_t i = 0; i < 3; i++)
		{
			_trajectory[i].reset(acceleration(i), velocity(i), position(i));
		}
	}

	/**
	 * @return float Current trajectory acceleration in X
	 */
	inline float getCurrentAccelerationX() const
	{
		return _trajectory[0].getCurrentAcceleration();
	}

	/**
	 * @return float Current trajectory acceleration in Y
	 */
	inline float getCurrentAccelerationY() const
	{
		return _trajectory[1].getCurrentAcceleration();
	}

	/**
	 * @return float Current trajectory acceleration in Z
	 */
	inline float getCurrentAccelerationZ() const
	{
		return _trajectory[2].getCurrentAcceleration();
	}

	/**
	 * @return float Current trajectory acceleration
	 */
	inline Eigen::Vector3f getCurrentAcceleration() const
	{
		return {getCurrentAccelerationX(), getCurrentAccelerationY(), getCurrentAccelerationZ()};
	}

	/**
	 * @return float Current trajectory acceleration in X and Y
	 */
	inline Eigen::Vector2f getCurrentAccelerationXY() const
	{
		return {getCurrentAccelerationX(), getCurrentAccelerationY()};
	}

	/**
	 * @return float Current trajectory velocity in X
	 */
	inline float getCurrentVelocityX() const
	{
		return _trajectory[0].getCurrentVelocity();
	}

	/**
	 * @return float Current trajectory velocity in Y
	 */
	inline float getCurrentVelocityY() const
	{
		return _trajectory[1].getCurrentVelocity();
	}

	/**
	 * @return float Current trajectory velocity in Z
	 */
	inline float getCurrentVelocityZ() const
	{
		return _trajectory[2].getCurrentVelocity();
	}

	/**
	 * @return float Current trajectory velocity
	 */
	inline Eigen::Vector3f getCurrentVelocity() const
	{
		return {getCurrentVelocityX(), getCurrentVelocityY(), getCurrentVelocityZ()};
	}

	/**
	 * @return float Current trajectory velocity in X and Y
	 */
	inline Eigen::Vector2f getCurrentVelocityXY() const
	{
		return {getCurrentVelocityX(), getCurrentVelocityY()};
	}

	/**
	 * @return float Current trajectory position in X
	 */
	inline float getCurrentPositionX() const
	{
		return _trajectory[0].getCurrentPosition();
	}

	/**
	 * @return float Current trajectory position in Y
	 */
	inline float getCurrentPositionY() const
	{
		return _trajectory[1].getCurrentPosition();
	}

	/**
	 * @return float Current trajectory position in Z
	 */
	inline float getCurrentPositionZ() const
	{
		return _trajectory[2].getCurrentPosition();
	}

	/**
	 * @return float Current trajectory position
	 */
	inline Eigen::Vector3f getCurrentPosition() const
	{
		return {getCurrentPositionX(), getCurrentPositionY(), getCurrentPositionZ()};
	}

	/**
	 * @param jerk maximum jerk for generated trajectory
	 */
	inline void setMaxJerkXY(float jerk)
	{
		_trajectory[0].setMaxJerk(jerk);
		_trajectory[1].setMaxJerk(jerk);
	}

	/**
	 * @param jerk maximum jerk for generated trajectory
	 */
	inline void setMaxJerkZ(float jerk)
	{
		_trajectory[2].setMaxJerk(jerk);
	}

	/**
	 * @param jerk maximum jerk for generated trajectory
	 */
	inline void setMaxJerk(const Eigen::Vector3f &jerk)
	{
		_trajectory[0].setMaxJerk(jerk(0));
		_trajectory[1].setMaxJerk(jerk(1));
		_trajectory[2].setMaxJerk(jerk(2));
	}

	/**
	 * @param accel maximum acceleration for generated trajectory
	 */
	inline void setMaxAccelerationXY(float accel)
	{
		_trajectory[0].setMaxAccel(accel);
		_trajectory[1].setMaxAccel(accel);
	}

	/**
	 * @param accel maximum acceleration for generated trajectory
	 */
	inline void setMaxAccelerationZ(float accel)
	{
		_trajectory[2].setMaxAccel(accel);
	}

	/**
	 * @param accel maximum acceleration for generated trajectory
	 */
	inline void setMaxAcceleration(const Eigen::Vector3f &accel)
	{
		_trajectory[0].setMaxAccel(accel(0));
		_trajectory[1].setMaxAccel(accel(1));
		_trajectory[2].setMaxAccel(accel(2));
	}

	/**
	 * @param vel maximum velocity for generated trajectory
	 */
	inline void setMaxVelocityXY(float vel)
	{
		_trajectory[0].setMaxVel(vel);
		_trajectory[1].setMaxVel(vel);
	}

	/**
	 * @param vel maximum velocity for generated trajectory
	 */
	inline void setMaxVelocityZ(float vel)
	{
		_trajectory[2].setMaxVel(vel);
	}

	/**
	 * @param vel maximum velocity for generated trajectory
	 */
	inline void setMaxVelocity(const Eigen::Vector3f &vel)
	{
		_trajectory[0].setMaxVel(vel(0));
		_trajectory[1].setMaxVel(vel(1));
		_trajectory[2].setMaxVel(vel(2));
	}

	/**
	 * @param error Maximum horizontal error allowed by the trajectory generator. Often set to param MPC_XY_ERR_MAX
	 */
	inline void setMaxAllowedHorizontalError(float error)
	{
		_max_allowed_horizontal_error = error;
	}

	/**
	 * @param radius  Altitude Acceptance Radius. Often set to NAV_MC_ALT_RAD
	 */
	inline void setVerticalAcceptanceRadius(float radius)
	{
		_vertical_acceptance_radius = radius;
	}

	/**
	 * @param speed vehicle cruise speed
	 */
	inline void setCruiseSpeed(float speed)
	{
		_cruise_speed = speed;
	}

	/**
	 * @param gain Proportional gain for horizontal trajectory position error. Set to MPC_XY_TRAJ_P
	 */
	inline void setHorizontalTrajectoryGain(float gain)
	{
		_horizontal_trajectory_gain = gain;
	}

	/**
	 * @param radius target acceptance radius
	 */
	inline void setTargetAcceptanceRadius(float radius)
	{
		_target_acceptance_radius = radius;
	}

	/**
	 * @brief Set the current position in the trajectory to the given value.
	 * Any coordinate with NAN will not be set
	 *
	 * @param position
	 */
	void forceSetPosition(const Eigen::Vector3f &position)
	{
		for (size_t i = 0; i < 3; i++)
		{
			if (std::isfinite(position(i)))
			{
				_trajectory[i].setCurrentPosition(position(i));
			}
		}
	}

	/**
	 * @brief Set the current velocity in the trajectory to the given value.
	 * Any coordinate with NAN will not be set
	 *
	 * @param velocity
	 */
	void forceSetVelocity(const Eigen::Vector3f &velocity)
	{
		for (size_t i = 0; i < 3; i++)
		{
			if (std::isfinite(velocity(i)))
			{
				_trajectory[i].setCurrentVelocity(velocity(i));
			}
		}
	}

	/**
	 * @brief Set the current acceleration in the trajectory to the given value.
	 * Any coordinate with NAN will not be set
	 *
	 * @param acceleration
	 */
	void forceSetAcceleration(const Eigen::Vector3f &acceleration)
	{
		for (size_t i = 0; i < 3; i++)
		{
			if (std::isfinite(acceleration(i)))
			{
				_trajectory[i].setCurrentAcceleration(acceleration(i));
			}
		}
	}

	void _generateSmoothSetpoints(
		const Eigen::Vector3f &local_position,
		const Eigen::Vector3f &target,
		const double timestamp,
		px4_msgs::msg::TrajectorySetpoint &out_setpoints);

private:
	/* params, only modified from external */
	float _max_allowed_horizontal_error{0.f}; // MPC_XY_ERR_MAX
	float _vertical_acceptance_radius{0.f};
	float _cruise_speed{0.f};
	float _horizontal_trajectory_gain{0.f};
	float _target_acceptance_radius{0.f};
	double _last_update{0.f};

	/* Internal state */
	VelocitySmoothing _trajectory[3]; ///< Trajectories in x, y and z directions
	float _max_speed_previous{0.f};

	double _timeout = 500000; /**< maximal time in us before a loop or data times out */

	/* Internal functions */
	bool _isTurning(const Eigen::Vector3f &target) const;

	const Eigen::Vector3f _generateUnsmoothedVelocitySetpoint(const Eigen::Vector3f &target);
	float _getMaxXYSpeed(const Eigen::Vector3f &target) const;
	float _getMaxZSpeed(const Eigen::Vector3f &target) const;
};
