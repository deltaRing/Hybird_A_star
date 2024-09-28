#ifndef _ROT_FUNCTION_HPP_
#define _ROT_FUNCTION_HPP_

#include <cmath>
#include "../Define/Common.hpp"

f64 pi_2_pi(f64 x) {
	f64 mod_angle = fmod(x + _PI, 2 * _PI) - _PI;
	return mod_angle;
}

f64 mod2pi(f64 x) {
	f64 v = std::fmod(x, 2 * _PI);
	if (v < -_PI) v += 2.0 * _PI;
	else
		if (v > _PI) v -= 2.0 * _PI;
	return v;
}

Eigen::Matrix2d rot_mat_2d(f64 angle) {
	Eigen::Matrix2d mat;
	mat << std::cos(angle), -std::sin(angle), 
		std::sin(angle), std::cos(angle);
	return mat;
}

void move(f64 & _x, f64 & _y, f64 & _yaw, f64 _distance, f64 _steer, f64 _L) {
	_x += _distance * std::cos(_yaw);
	_y += _distance * std::sin(_yaw);
	_yaw += pi_2_pi(_distance * std::tan(_steer) / _L);
}

#endif