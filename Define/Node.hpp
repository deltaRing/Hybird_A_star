#ifndef _NODE_HPP_
#define _NODE_HPP_

#include "Common.hpp"

class Node {
public:
	Node() {}

	Node(l32 x, l32 y, f64 cost, Node* parent) {
		this->x_ind = x;
		this->y_ind = y;
		this->cost = cost;
		this->parent = parent;
	}

	Node(l32 x_ind, l32 y_ind, f64 yaw, b1 direction,
		std::vector<f64> x_list, std::vector<f64> y_list, std::vector<f64> yaw_list,
		std::vector<b1> directions, f64 steer=0, Node* parent=NULL, f64 cost=0) {
		this->x_ind = x_ind;
		this->y_ind = y_ind;
		this->yaw = yaw;
		this->direction = direction;
		this->x_list = x_list;
		this->y_list = y_list;
		this->yaw_list = yaw_list;
		this->directions = directions;
		this->steer = steer;
		this->parent = parent;
		this->cost = cost;
	}
	l32 x_ind = -1, y_ind = -1;
	f64 yaw = -1; b1 direction = false;
	std::vector<f64> x_list, y_list;
	std::vector<f64> yaw_list;
	std::vector<b1> directions;
	f64 steer = -1, cost = -1;
	Node* parent = NULL;
private:

};


#endif