#ifndef _HYBIRD_ASTAR_HPP_
#define _HYBIRD_ASTAR_HPP_

#include "Define/Common.hpp"
#include "Define/Config.hpp"
#include "Define/Node.hpp"
#include "Define/Path.hpp"
#include "DynamicProgramingHeuristic.hpp"

f64 CalculateCost(Node n, std::vector <std::pair<l32, Node*>> h_dp, Config c) {
	l32 ind = (n.y_ind - c.min_y) * c.x_w + (n.x_ind - c.min_x);
	for (int ii = 0; ii < h_dp.size(); ii++) {
		if (h_dp[ii].first == ind) {
			return n.cost + _H_COST * h_dp[ii].second->cost;
		}
	}
	return n.cost + 1e10; // collision
}

l32 CalculateIndexYaw(Node node, Config c) {
	return (node.yaw - c.min_yaw) * c.x_w * c.y_w + \
		(node.y_ind - c.min_y) * c.x_w + (node.x_ind - c.min_x);
}

b1 RectangleCheck(f64 x, f64 y, f64 yaw, 
	std::vector<f64> ox, std::vector<f64> oy) {
	Eigen::Matrix2d rot = rot_mat_2d(yaw);

	for (int ii = 0; ii < ox.size(); ii++) {
		f64 iox = ox[ii], ioy = oy[ii];
		f64 tx = iox - x, ty = ioy - y;
		Eigen::MatrixXd xy(1, 2);
		xy << tx, ty;
		xy = xy * rot;
		f64 rx = xy(0, 0), ry = xy(0, 1);
		if (!(rx > _LF || rx < -_LB || ry > _W / 2.0 || ry < -_W / 2.0)) {
			return 0; // collision
		}
	}
	return 1; // no collision
}

b1 CheckCollision(std::vector<f64> x_list, std::vector<f64> y_list, 
	std::vector<f64> yaw_list,
	Eigen::ArrayXd ox, Eigen::ArrayXd oy) {
	for (int ii = 0; ii < x_list.size(); ii++) {
		f64 cx = x_list[ii] + _BUBBLE_DIST * cos(yaw_list[ii]);
		f64 cy = y_list[ii] + _BUBBLE_DIST * sin(yaw_list[ii]);

		Eigen::ArrayXd _ids_x = abs(ox - cx);
		Eigen::ArrayXd _ids_y = abs(oy - cy);
		Eigen::ArrayXd _ids = (_ids_x * _ids_x + _ids_y * _ids_y).sqrt();
		auto           _ids_g = _ids < _BUBBLE_R;
		b1 _ids_none = _ids_g.any();
		if (!_ids_none) continue;

		std::vector<f64> oxx, oyy;
		for (int jj = 0; jj < _ids_g.size(); jj++) {
			if (_ids_g(jj)) {
				oxx.push_back(ox(jj));
				oyy.push_back(oy(jj));
			}
		}
		if (!RectangleCheck(x_list[ii], y_list[ii], yaw_list[ii],
			oxx, oyy)) {
			return 0;
		}
	}

	return 1; // no collision
}

// 
f64 CalculateRSPathCost(_Path reed_shepp_path) {
	f64 cost = 0.0;
	for (int ii = 0; ii < reed_shepp_path.lengths.size(); ii++) {
		f64 length = reed_shepp_path.lengths(ii);
		if (length >= 0.0) {
			cost += length; // go forward
		}
		else {
			cost += abs(length) * _BACK_COST;
		}
	}

	// switch back penelty
	for (int ii = 0; ii < reed_shepp_path.lengths.size() - 1; ii++) {
		if (reed_shepp_path.lengths(ii) * reed_shepp_path.lengths(ii + 1) < 0.0)
			cost += _SWITCH_BACK_COST;
	}

	// steer penalty
	for (int ii = 0; ii < reed_shepp_path.ctypes.size(); ii++) {
		if (reed_shepp_path.ctypes[ii] != 'S') {
			cost += _YAW_CHANGE_COST * abs(_MAX_STEER);
		}
	}

	l32 n_ctypes = reed_shepp_path.ctypes.size();
	Eigen::ArrayXd u_list = Eigen::ArrayXd::Zero(n_ctypes);
	for (int ii = 0; ii < n_ctypes; ii++) {
		if (reed_shepp_path.ctypes[ii] == 'R') {
			u_list(ii) = -_MAX_STEER;
		}
		else if (reed_shepp_path.ctypes[ii] == 'L') {
			u_list(ii) = _MAX_STEER;
		}
	}

	for (int ii = 0; ii < n_ctypes - 1; ii++) {
		cost += _YAW_CHANGE_COST * abs(u_list[ii + 1] - u_list[ii]);
	}

	return cost;
}

//
_Path AnalyticExpansion(Node current, Node goal, 
	Eigen::ArrayXd ox, Eigen::ArrayXd oy) {
	f64 start_x = current.x_list.back(),
		start_y = current.y_list.back(),
		start_yaw = current.yaw_list.back(),
		
		goal_x = goal.x_list.back(),
		goal_y = goal.y_list.back(),
		goal_yaw = goal.yaw_list.back();

	f64 max_curvature = std::tan(_MAX_STEER) / _WB;

	Eigen::Vector3d start, node;
	start << start_x, start_y, start_yaw;
	node << goal_x, goal_y, goal_yaw;
	std::vector<_Path> paths = CalculatePaths(start, node,
		max_curvature, _MOTION_RESOLUTION);

	_Path _best_path; f64 _best = -1;
	if (paths.size() == 0) {
		return _best_path;
	}

	for (int ii = 0; ii < paths.size(); ii++) {
		_Path _path = paths[ii];
		if (CheckCollision(_path.x, _path.y, _path.yaw, ox, oy)) {
			f64 cost = CalculateRSPathCost(_path);
			if (_best < 0 || _best > cost) {
				_best = cost;
				_best_path = _path;
			}
		}
	}
	return _best_path;
}

// 
void UpdateNodeWithAnalyticExpansion(Node * current, Node goal, Config c,
	Eigen::ArrayXd ox, Eigen::ArrayXd oy, Node & _f_path, b1 & _find) {
	_Path _path = AnalyticExpansion(*current, goal, ox, oy);

	if (_path.x.size() > 0) {
		std::vector<f64> f_x = _path.x,
			f_y = _path.y,
			f_yaw = _path.yaw;

		f64 _f_cost = current->cost + CalculateRSPathCost(_path);
		l32 _f_parent_index = CalculateIndex(*current, c);
		_f_path = Node(current->x_ind, current->y_ind, current->yaw,
			current->direction, f_x, f_y, f_yaw, _path.directions,
			_f_cost, current, 0.0);
		_find = 1;
		return;
	}
	_find = 0;
}

//
Eigen::MatrixXd CalculateMotionInputs() {
	Eigen::MatrixXd _result(_N_STEER * 2 + 2, 2);
	Eigen::ArrayXd _steer = Eigen::ArrayXd::LinSpaced(_N_STEER, -_MAX_STEER, _MAX_STEER);
	for (int ii = 0; ii < _N_STEER; ii++) {
		for (int jj = 0; jj < 2; jj++) {
			_result(ii * 2 + jj, 0) = _steer(ii);
			_result(ii * 2 + jj, 1) = jj == 0?1:-1;
		}
	}
	_result(_N_STEER * 2, 0) = 0.0; _result(_N_STEER * 2, 1) = 1.0;
	_result(_N_STEER * 2 + 1, 0) = 0.0; _result(_N_STEER * 2 + 1, 1) = -1.0;
	return _result;
}

//
b1 VerifyIndex(Node _node, Config c) {
	l32 _x_ind = _node.x_ind, _y_ind = _node.y_ind;
	if (c.min_x <= _x_ind && _x_ind <= c.max_x &&
		c.min_y <= _y_ind && _y_ind <= c.max_y) {
		return 1;
	}
	return 0;
}

//
Node * CalculateNextNode(Node* current, f64 steer, b1 direction,
	Config c, Eigen::ArrayXd ox, Eigen::ArrayXd oy) {

	f64 _x = current->x_list.back(),
		_y = current->y_list.back(),
		_yaw = current->yaw_list.back();
	f64 arc_l = _XY_GRID_RESOLUTION * 1.5;
	l32 _num = arc_l / _MOTION_RESOLUTION;
	std::vector<f64> _x_list, _y_list, _yaw_list;
	Eigen::ArrayXd _arc_l = Eigen::ArrayXd::LinSpaced(_num, 0.0, arc_l);

	for (int ii = 0; ii < _num; ii++) {
		l32 _direction = direction ? 1 : -1;
		move(_x, _y, _yaw, _MOTION_RESOLUTION * _direction, steer, _WB);
		_x_list.push_back(_x);
		_y_list.push_back(_y);
		_yaw_list.push_back(_yaw);
	}

	Node * _node = NULL;
	if (!CheckCollision(_x_list, _y_list, _yaw_list, ox, oy))
		return _node;

	b1 _d = direction;
	l32 _x_ind = round(_x / _XY_GRID_RESOLUTION),
		_y_ind = round(_y / _XY_GRID_RESOLUTION),
		_yaw_ind = round(_yaw / _YAW_GRID_RESOLUTION);

	f64 _added_cost = 0.0;
	if (_d != current->direction) _added_cost += _SWITCH_BACK_COST;

	// steer penalty
	_added_cost += _YAW_COST * abs(steer);
	// steer change penalty
	_added_cost += _YAW_CHANGE_COST * std::abs(current->steer - steer);

	f64 cost = current->cost + _added_cost + arc_l;

	_node = new Node(_x_ind, _y_ind, _yaw_ind, _d,
		_x_list, _y_list, _yaw_list, std::vector<b1> {_d},
		steer, current, cost
	);

	return _node;
}

//
std::vector<Node *> GetNeighbors(Node * current, Config c,
	Eigen::ArrayXd ox, Eigen::ArrayXd oy) {

	std::vector <Node *> _nodes_;
	Eigen::MatrixXd motions = CalculateMotionInputs();
	for (int ii = 0; ii < motions.rows(); ii++) {
		f64 _steer = motions(ii, 0), _d = motions(ii, 1);
		Node * _node = CalculateNextNode(current, _steer, _d > 0 ? 1 : 0,
			c, ox, oy);
		if (_node == NULL) 
			continue;
		if (!_node->x_list.empty() && VerifyIndex(*_node, c)) {
			_nodes_.push_back(_node);
		}
		else {
			delete _node;
		}
	}
	return _nodes_;
}

Path GetFinalPath(Node _goal_node) {
	std::vector<f64> reversed_x, reversed_y, reversed_yaw;
	std::vector<b1> reversed_direction;
	for (int ii = _goal_node.x_list.size()-1; ii >= 0; ii--) {
		reversed_x.push_back(_goal_node.x_list[ii]);
		reversed_y.push_back(_goal_node.y_list[ii]);
		reversed_yaw.push_back(_goal_node.yaw_list[ii]);
		reversed_direction.push_back(_goal_node.directions[ii]);
	}
	Node* nid = _goal_node.parent;
	f64 _final_cost = _goal_node.cost;

	while (nid != NULL) {
		for (int ii = nid->x_list.size() - 1; ii >= 0; ii--) {
			reversed_x.push_back(nid->x_list[ii]);
			reversed_y.push_back(nid->y_list[ii]);
			reversed_yaw.push_back(nid->yaw_list[ii]);
			reversed_direction.push_back(nid->directions[0]);
		}
		nid = nid->parent;
	}

	std::vector<f64> _x, _y, _yaw;
	std::vector<b1> _d;
	for (int ii = reversed_x.size(); ii > 0; ii--) {
		_x.push_back(reversed_x[reversed_x.size() - ii]);
		_y.push_back(reversed_y[reversed_y.size() - ii]);
		_yaw.push_back(reversed_yaw[reversed_yaw.size() - ii]);
		_d.push_back(reversed_direction[reversed_direction.size() - ii]);
	}

	_d[0] = _d[1];
	Path path(_x, _y, _yaw, _d, _final_cost);
	return path;
}

// Input 1: start ( x, y, yaw ) Eigen::Vector3d
// Input 2: goal  ( x, y, yaw ) Eigen::Vector3d
// Input 3: ox (1 x n) x location of the barrier
// Input 4: oy (1 x n) y location of the barrier
// Input 5: lower bound of map x
// Input 6: upper bound of map y
// Input 7: _xy_resolution XY格的分辨率
// Input 8: _yaw_resotion  yaw角的分辨率
Path HyBird_Astar_Planning(Eigen::Vector3d start, Eigen::Vector3d goal,
	Eigen::ArrayXd ox, Eigen::ArrayXd oy, 
	Eigen::Vector2d lmap, Eigen::Vector2d umap,
	f64 _xy_resolution, f64 _yaw_resolution) {


	Config config = Config(lmap, umap, 
		_xy_resolution, _yaw_resolution);

	Node * start_node = new Node(round(start(0) / _xy_resolution),
		round(start(1) / _xy_resolution),
		round(start(2) / _yaw_resolution),
		1,
		std::vector<f64>{start(0)},
		std::vector<f64>{start(1)},
		std::vector<f64>{start(2)},
		std::vector<b1>{1},
		0.0,
		NULL,
		0.0);

	Node * goal_node = new Node(round(goal(0) / _xy_resolution),
		round(goal(1) / _xy_resolution),
		round(goal(2) / _yaw_resolution),
		1,
		std::vector<f64>{goal(0)},
		std::vector<f64>{goal(1)},
		std::vector<f64>{goal(2)},
		std::vector<b1>{1});

	std::vector <std::pair<l32, Node *>> openList, closedList;
	std::vector <std::pair<l32, Node *>> h_dp = CalculateDistanceHeuristic(
		goal_node->x_list.back(),
		goal_node->y_list.back(),
		ox, oy, _xy_resolution, 2.3711, config
	);
	openList.push_back(
		std::make_pair(CalculateIndexYaw(*start_node, config), start_node)
	);
	std::priority_queue<std::vector<f64>, std::vector<std::vector<f64>>, std::greater<std::vector<f64>>> pq;

	pq.push(std::vector<f64>{CalculateCost(*start_node, h_dp, config),
		(f64)CalculateIndexYaw(*start_node, config)});
	Path _path; Node _final_path;

	while (1) {
		if (openList.size() == 0) {
			delete_close(closedList);
			delete_close(openList);

			std::cout << "Path Not Found" << std::endl;

			return _path; // no path
		}

		std::vector<f64> _set = pq.top();
		pq.pop();

		l32 c_id = _set[1]; b1 _find = false; Node* current = NULL;
		for (int ii = 0; ii < openList.size(); ii++) {
			if (openList[ii].first == c_id) {
				current = openList[ii].second;
				closedList.push_back(
					std::make_pair(c_id, current)
				);
				openList.erase(openList.begin() + ii);
				_find = 1;
				break;
			}
		}
		if (!_find)
			continue;

		b1 _is_updated = 0;
		UpdateNodeWithAnalyticExpansion(current, *goal_node, config,
			ox, oy, _final_path, _is_updated);

		if (_is_updated)
			break; // path is found
		
		std::vector<Node *> _nodes = GetNeighbors(current, config, ox, oy);
		for (int ii = 0; ii < _nodes.size(); ii++) {
			l32 _n_index = CalculateIndexYaw(*_nodes[ii], config); b1 _in_close = 0;
			for (int jj = 0; jj < closedList.size(); jj++) {
				if (closedList[jj].first == _n_index) {
					_in_close = 1;
					break;
				}
			}
			if (_in_close) continue;

			b1 _not_in_open = 1,
				cond2 = 0; l32 _record_index = -1;
			for (int jj = 0; jj < openList.size(); jj++) {
				if (openList[jj].first == _n_index) {
					cond2 = openList[jj].second->cost > _nodes[ii]->cost;
					_record_index = jj;
					_not_in_open = 0;
					break;
				}
			}
			if (_not_in_open || cond2) {
				std::vector<f64> pair = {CalculateCost(*_nodes[ii], h_dp, config),
					(f64)CalculateIndexYaw(*_nodes[ii], config) };
				pq.push(pair);

				if (_record_index >= 0) {
					delete openList[_record_index].second;
					openList[_record_index] = std::make_pair(_n_index, _nodes[ii]);
				}
				else {
					openList.push_back(
						std::make_pair(_n_index, _nodes[ii])
					);
				}
			}
		}
	} 

	_path = GetFinalPath(_final_path);

	delete_close(closedList);
	delete_close(openList);

	return _path;
}

#endif