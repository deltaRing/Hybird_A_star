#ifndef _DYNAMIC_PROGRAMMING_HEURISTIC_HPP_
#define _DYNAMIC_PROGRAMMING_HEURISTIC_HPP_

#include "Define/Common.hpp"
#include "Define/Config.hpp"
#include "Define/Node.hpp"
#include "Define/Path.hpp"

// get the obstacle map
// input config
// ox oy: the barrier vector
// vr: ×ªÍä°ë¾¶
void CalculateObstacleMap(Config config, 
	Eigen::ArrayXi ox, Eigen::ArrayXi oy,
	f64 vr, b1 ** obstacle_map) {
	l32 min_x = config.min_x, min_y = config.min_y,
		max_x = config.max_x, max_y = config.max_y; 

	for (int xx = 0; xx < config.x_w; xx++) {
		l32 x = xx + min_x;
		for (int yy = 0; yy < config.y_w; yy++) {
			l32 y = yy + min_y;
			for (int ii = 0; ii < ox.rows(); ii++) {
				f64 oxii = ox(ii), oyii = oy(ii);
				f64 dx = oxii - x, dy = oyii - y;
				f64 d = sqrt(dx * dx + dy * dy);
				if (d <= vr / config._xy_resulotion) {
					obstacle_map[xx][yy] = 1;
					break;
				}
			}
		}
	}
}

// dx dy cost
Eigen::MatrixXd GetMotionModel() {
	Eigen::MatrixXd motion(8, 3);
	motion << 1, 0, 1,
		0, 1, 1,
		-1, 0, 1,
		0, -1, 1,
		-1, -1, sqrt(2.0),
		-1, 1, sqrt(2.0),
		1, -1, sqrt(2.0),
		1, 1, sqrt(2.0);
	return motion;
}

b1 VerifyNode(Node node, b1 ** obstacle_map, Config config) {
	l32 min_x = config.min_x, min_y = config.min_y,
		max_x = config.max_x, max_y = config.max_y;
	if (node.x_ind < min_x) {
		return 0;
	}
	else if (node.x_ind >= max_x) {
		return 0;
	}
	else if (node.y_ind < min_y) {
		return 0;
	}
	else if (node.y_ind >= max_y) {
		return 0;
	}

	if (obstacle_map[node.x_ind][node.y_ind]) {
		return 0;
	}

	return 1;
}

l32 CalculateIndex(Node node, Config config) {
	l32 x_width = config.x_w, x_min = config.min_x, y_min = config.min_y;
	return (node.y_ind - y_min) * x_width + node.x_ind - x_min;
}

void delete_close(std::vector<std::pair<l32, Node*>> _close_set) {
	while (_close_set.size() > 0) {
		std::pair<l32, Node*> _set = _close_set.back();
		_close_set.pop_back();
		delete _set.second;
	}
}

// rr ×ªÍä°ë¾¶
std::vector<std::pair<l32, Node *>> CalculateDistanceHeuristic(
	f64 gx, f64 gy,
	Eigen::ArrayXd ox, Eigen::ArrayXd oy,
	f64 _xy_resolution, f64 rr, Config config) {
	Node * goal_node = new Node(l32(gx / _xy_resolution),
		l32(gy / _xy_resolution), 0.0, NULL);
	Eigen::ArrayXi oxx(ox.rows()), oyy(oy.rows());

	for (int ii = 0; ii < ox.rows(); ii++) {
		oxx(ii) = ox(ii) / _xy_resolution;
		oyy(ii) = oy(ii) / _xy_resolution;
	}

	b1** obstacle_map = new b1 * [config.x_w];
	for (int xx = 0; xx < config.x_w; xx++) {
		obstacle_map[xx] = new b1[config.y_w];
		for (int yy = 0; yy < config.y_w; yy++) {
			obstacle_map[xx][yy] = 0;
		}
	}

	CalculateObstacleMap(config, oxx, oyy, rr, obstacle_map);
	Eigen::MatrixXd motion = GetMotionModel();
	std::vector<std::pair<l32, Node *>> open_set, close_set;
	std::pair<l32, Node *> set = std::make_pair(CalculateIndex(*goal_node, config), goal_node);
	open_set.push_back(set);
	std::priority_queue<std::vector<f64>, std::vector<std::vector<f64>>, std::greater<std::vector<f64>>> priority_queue;
	// std::vector<std::pair<f64, l32>> priority_queue;
	// cost Index
	std::vector<f64> _pair = { 0, (f64)CalculateIndex(*goal_node, config) };
	priority_queue.push(_pair);

	while (1) {
		if (priority_queue.empty()) break;
		std::vector<f64> _set = priority_queue.top();
		priority_queue.pop();

		l32 c_id = _set[1]; b1 _find = false; Node * current = NULL;
		for (int ii = 0; ii < open_set.size(); ii++) {
			if (open_set[ii].first == c_id) {
				current = open_set[ii].second;
				close_set.push_back(
					std::make_pair(c_id, current)
				);
				open_set.erase(open_set.begin() + ii);
				_find = 1;
				break;
			}
		}
		if (!_find)
			continue;

		for (int ii = 0; ii < motion.rows(); ii++) {
			f64 mx = motion(ii, 0),
				my = motion(ii, 1),
				mc = motion(ii, 2);
			Node * node = new Node(current->x_ind + mx,
				current->y_ind + my,
				current->cost + mc,
				current);
			l32 n_id = CalculateIndex(*node, config);

			b1 _closed_set = 0;
			for (int jj = 0; jj < close_set.size(); jj++) {
				if (close_set[jj].first == n_id) {
					_closed_set = 1;
					break;
				}
			}
			if (_closed_set)
				continue;
			if (!VerifyNode(*node, obstacle_map, config))
				continue;

			b1 _open_set = 0; l32 _open_index = -1;
			for (int jj = 0; jj < open_set.size(); jj++) {
				if (open_set[jj].first == n_id) {
					_open_set = 1;
					_open_index = jj;
					break;
				}
			}
			if (!_open_set) {
				open_set.push_back(
					std::make_pair(n_id, node)
				);
				priority_queue.push(
					std::vector<f64>{node->cost, (f64)CalculateIndex(*node, config)}
				);
			}
			else {
				if (open_set[_open_index].second->cost >= node->cost) {
					open_set[_open_index] = std::make_pair(n_id, node);
					priority_queue.push(
						std::vector<f64>{node->cost, (f64)CalculateIndex(*node, config)}
					);
				}
			}
		}
	}

	for (int xx = 0; xx < config.x_w; xx++) delete[] obstacle_map[xx];
	delete[] obstacle_map;

	return close_set;
}

#endif