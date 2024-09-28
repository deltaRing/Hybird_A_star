#include "Algorithm/rot_function.hpp"
#include "Algorithm/ReedsSheppPath.hpp"
#include "HybirdAstar.hpp"

int main() {
	Eigen::ArrayXd ox = Eigen::ArrayXd::Zero(322), 
		oy = Eigen::ArrayXd::Zero(322);
	Eigen::Vector3d start, goal;
	Eigen::Vector2d lmap, umap;

	start << 10.0, 10.0, _PI / 2;
	goal << 50.0, 50.0, -_PI / 2;
	lmap << 0, 0;
	umap << 60, 60;

	l32 index = 0;
	for (int ii = 0; ii < 60; ii++) {
		ox(index + ii) = ii;
		oy(index + ii) = 0.0;
	}index += 60;
	for (int ii = 0; ii < 60; ii++) {
		ox(index + ii) = 60.0;
		oy(index + ii) = ii;
	}index += 60;
	for (int ii = 0; ii < 61; ii++) {
		ox(index + ii) = ii;
		oy(index + ii) = 60.0;
	}index += 61;
	for (int ii = 0; ii < 61; ii++) {
		ox(index + ii) = 0.0;
		oy(index + ii) = ii;
	}index += 61;
	for (int ii = 0; ii < 40; ii++) {
		ox(index + ii) = 20.0;
		oy(index + ii) = ii;
	}index += 40;
	for (int ii = 0; ii < 40; ii++) {
		ox(index + ii) = 40.0;
		oy(index + ii) = 60.0 - ii;
	}index += 40;

	Path _path = HyBird_Astar_Planning(
		start, goal, ox, oy, lmap, umap, _XY_GRID_RESOLUTION, _YAW_GRID_RESOLUTION
	);

	for (int ii = 0; ii < _path.x_list.size(); ii++) {
		std::cout << _path.yaw_list[ii] << " ";
	}

	return 0;
}