#ifndef _PATH_HPP_
#define _PATH_HPP_

#include "Common.hpp"

class Path {
public:
	Path() {
	
	}
	Path(std::vector<f64> x_list, std::vector<f64> y_list, std::vector<f64> yaw_list,
		std::vector<b1> direction_list, f64 cost) {
		this->x_list = x_list;
		this->y_list = y_list;
		this->yaw_list = yaw_list;
		this->direction_list = direction_list;
		this->cost = cost;
	}
	std::vector<f64> x_list, y_list;
	std::vector<f64> yaw_list;
	std::vector<b1>  direction_list;
	f64 cost = -1;
private:

};

#endif