#ifndef _CONFIG_HPP_
#define _CONFIG_HPP_

#include "Common.hpp"

class Config {
public:
	Config() {

	}

	Config(Eigen::Vector2d minIndex, Eigen::Vector2d maxIndex,
		f64 _xy_resulotion, f64 _yaw_resulotion) {
		min_x = round(minIndex(0) / _xy_resulotion); max_x = round(maxIndex(0) / _xy_resulotion);
		min_y = round(minIndex(1) / _xy_resulotion); max_y = round(maxIndex(1) / _xy_resulotion);

		x_w = round(max_x - min_x);
		y_w = round(max_y - min_y);

		min_yaw = round(-_PI / _yaw_resulotion) - 1;
		max_yaw = round(_PI / _yaw_resulotion);
		yaw_w = round(max_yaw - min_yaw);
		this->_xy_resulotion = _xy_resulotion;
		this->_yaw_resulotion = _yaw_resulotion;
	}
	l32 min_x = -1, min_y = -1, max_x = -1, max_y = -1;
	l32 x_w = -1, y_w = -1;
	// yaw related
	l32 min_yaw = -1, max_yaw = -1, yaw_w = -1;
	f64 _xy_resulotion = -1;
	f64 _yaw_resulotion = -1;
private:

};

#endif