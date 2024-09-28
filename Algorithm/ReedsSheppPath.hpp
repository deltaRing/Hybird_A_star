#ifndef _REEDS_SHEPP_PATH_HPP_
#define _REEDS_SHEPP_PATH_HPP_

#include "../Define/Common.hpp"
#include "rot_function.hpp"

struct LocalCourse {
	std::vector<f64> x;
	std::vector<f64> y;
	std::vector<f64> yaw;
	std::vector<b1>  directions;
};

struct Interpolate {
	f64 x = -1;
	f64 y = -1;
	f64 yaw = -1;
	b1 direction = 1;
};

struct _Path {
	Eigen::ArrayXd lengths;
	std::vector<l8> ctypes;
	f64 L = 0;
	std::vector<f64> x;
	std::vector<f64> y;
	std::vector<f64> yaw;
	std::vector<b1> directions;
};

struct _Path_Function {
	b1 _path = 0;
	Eigen::ArrayXd result;
	std::vector<l8> direction;
};

// 
std::vector<_Path> set_path(std::vector<_Path> paths, _Path_Function _path, f64 step_size) {
	_Path path = _Path();
	path.ctypes = _path.direction;
	path.lengths = _path.result;
	path.L = _path.result.abs().sum();

	for (int ii = 0; ii < paths.size(); ii++) {
		_Path _i_path = paths[ii];
		b1 type_is_same = _i_path.ctypes.size() == path.ctypes.size();
		if (type_is_same)
			for (int jj = 0; jj < _i_path.ctypes.size(); jj++) {
				if (_i_path.ctypes[jj] != path.ctypes[jj]) {
					type_is_same = 0;
					break;
				}
			}
		b1 length_is_close = (_i_path.lengths.abs().sum() - path.L) <= step_size;
		if (type_is_same && length_is_close)
			return paths;
	}

	if (path.L <= step_size) {
		return paths;
	}

	paths.push_back(path);
	return paths;
}

// 
void polar(f64 x, f64 y, f64& r, f64& theta) {
	r = std::sqrt(x * x + y * y);
	theta = std::atan2(y, x);
}

//
_Path_Function LeftStraightLeft(f64 x, f64 y, f64 phi) {
	_Path_Function _path;
	f64 u = -1, t = -1;
	polar(x - std::sin(phi), y - 1.0 + std::cos(phi), u, t);
	if (t >= 0.0 && t <= _PI) {
		f64 v = mod2pi(phi - t);
		if (v >= 0.0 && v <= _PI) {
			_path._path = 1;
			_path.result = Eigen::ArrayXd(3);
			_path.result << t, u, v;
			_path.direction = { 'L', 'S', 'L' };
		}
	}
	return _path;
}

//
_Path_Function LeftStraightRight(f64 x, f64 y, f64 phi) {
	_Path_Function _path;
	f64 u = -1, t = -1;
	polar(x + std::sin(phi), y - 1.0 - std::cos(phi), u, t);
	u = u * u;
	if (u >= 4.0) {
		u = sqrt(u - 4.0);
		f64 theta = atan2(2.0, u);
		t = mod2pi(t + theta);
		f64 v = mod2pi(t - phi);
		if (t >= 0.0 && v >= 0.0) {
			_path._path = 1;
			_path.result = Eigen::ArrayXd(3);
			_path.result << t, u, v;
			_path.direction = { 'L', 'S', 'R' };
		}
	}
	return _path;
}

//
_Path_Function LeftXRightXLeft(f64 x, f64 y, f64 phi) {
	_Path_Function _path;
	f64 zeta = x - std::sin(phi),
		eeta = y - 1 + std::cos(phi),
		u1 = -1, theta = -1;
	polar(zeta, eeta, u1, theta);

	if (u1 <= 4.0) {
		f64 A = std::acos(0.25 * u1);
		f64 t = mod2pi(A + theta + _PI / 2);
		f64 u = mod2pi(_PI - 2 * A);
		f64 v = mod2pi(-phi + t + u);
		_path._path = 1;
		_path.result = Eigen::ArrayXd(3);
		_path.result << t, -u, v;
		_path.direction = {'L', 'R', 'L'};
	}

	return _path;
}

//
_Path_Function LeftXRightLeft(f64 x, f64 y, f64 phi) {
	_Path_Function _path;

	f64 zeta = x - std::sin(phi),
		eeta = y - 1 + std::cos(phi),
		u1 = -1, theta = -1;
	polar(zeta, eeta, u1, theta);

	if (u1 <= 4.0) {
		f64 A = acos(0.25 * u1);
		f64 t = mod2pi(A + theta + _PI / 2);
		f64 u = mod2pi(_PI - 2 * A);
		f64 v = mod2pi(-phi + t + u);
		_path._path = 1;
		_path.result = Eigen::ArrayXd(3);
		_path.result << t, -u, v;
		_path.direction = { 'L', 'R', 'L' };
	}

	return _path;
}

//
_Path_Function LeftRightXLeft(f64 x, f64 y, f64 phi) {
	_Path_Function _path;
	f64 zeta = x - std::sin(phi),
		eeta = y - 1 + std::cos(phi),
		u1 = -1, theta = -1;
	polar(zeta, eeta, u1, theta);

	if (u1 <= 4.0) {
		f64 u = std::acos(1.0 - u1 * u1 * 0.125);
		f64 A = std::asin(2.0 * std::sin(u) / u1);
		f64 t = mod2pi(-A + theta + _PI / 2);
		f64 v = mod2pi(t - u - phi);
		_path._path = 1;
		_path.result = Eigen::ArrayXd(3);
		_path.result << t, u, -v;
		_path.direction = {'L', 'R', 'L'};
	}
	return _path;
}

//
_Path_Function LeftRightXLeftRight(f64 x, f64 y, f64 phi) {
	_Path_Function _path;
	f64 zeta = x + std::sin(phi),
		eeta = y - 1 - std::cos(phi),
		u1 = -1, theta = -1;
	polar(zeta, eeta, u1, theta);

	if (u1 <= 2) {
		f64 A = std::acos((u1 + 2) * 0.25);
		f64 t = mod2pi(theta + A + _PI / 2);
		f64 u = mod2pi(A);
		f64 v = mod2pi(phi - t + 2 * u);
		if (t >= 0 && u >= 0 && v >= 0) {
			_path._path = 1;
			_path.result = Eigen::ArrayXd(4);
			_path.result << t, u, -u, -v;
			_path.direction = {'L', 'R', 'L', 'R'};
		}
	}

	return _path;
}

//
_Path_Function LeftXRightLeftXRight(f64 x, f64 y, f64 phi) {
	_Path_Function _path;
	f64 zeta = x + std::sin(phi),
		eeta = y - 1 - std::cos(phi),
		u1 = -1, theta = -1;
	polar(zeta, eeta, u1, theta);
	f64 u2 = (20 - u1 * u1) / 16;

	if (0 <= u2 && u2 <= 1) {
		f64 u = std::acos(u2);
		f64 A = std::asin(2 * std::sin(u) / u1);
		f64 t = mod2pi(theta + A + _PI / 2);
		f64 v = mod2pi(t - phi);
		if (t >= 0 && v >= 0) {
			_path._path = 1;
			_path.result = Eigen::ArrayXd(4);
			_path.result << t, -u, -u, v;
			_path.direction = { 'L', 'R', 'L', 'R' };
		}
	}

	return _path;
}

//
_Path_Function LeftXRight90StartghtLeft(f64 x, f64 y, f64 phi) {
	_Path_Function _path;
	f64 zeta = x - std::sin(phi),
		eeta = y - 1 + std::cos(phi),
		u1 = -1, theta = -1;
	polar(zeta, eeta, u1, theta);

	if (u1 > 2.0) {
		f64 u = std::sqrt(u1 * u1 - 4) - 2;
		f64 A = std::atan2(2, sqrt(u1 * u1 - 4));
		f64 t = mod2pi(theta + A + _PI / 2);
		f64 v = mod2pi(t - phi + _PI / 2);
		if (t >= 0 && v >= 0) {
			_path._path = 1;
			_path.result = Eigen::ArrayXd(4);
			_path.result << t, -_PI / 2, -u, -v;
			_path.direction = { 'L', 'R', 'S', 'L' };
		}
	}

	return _path;
}

//
_Path_Function LeftStraightRight90XLeft(f64 x, f64 y, f64 phi) {
	_Path_Function _path;
	f64 zeta = x - std::sin(phi),
		eeta = y - 1 + std::cos(phi),
		u1 = -1, theta = -1;
	polar(zeta, eeta, u1, theta);

	if (u1 >= 2.0) {
		f64 u = std::sqrt(u1 * u1 - 4.0) - 2.0;
		f64 A = std::atan2(std::sqrt(u1 * u1 - 4), 2);
		f64 t = mod2pi(theta - A + _PI / 2);
		f64 v = mod2pi(t - phi - _PI / 2);
		if (t >= 0 && v >= 0) {
			_path._path = 1;
			_path.result = Eigen::ArrayXd(4);
			_path.result << t, u, _PI / 2, -v;
			_path.direction = { 'L', 'S', 'R', 'L' };
		}
	}

	return _path;
}

//
_Path_Function LeftXRight90StraightRight(f64 x, f64 y, f64 phi) {
	_Path_Function _path;
	f64 zeta = x + std::sin(phi),
		eeta = y - 1 - std::cos(phi),
		u1 = -1, theta = -1;
	polar(zeta, eeta, u1, theta);

	if (u1 >= 2.0) {
		f64 t = mod2pi(theta + _PI / 2);
		f64 u = u1 - 2;
		f64 v = mod2pi(phi - t - _PI / 2);
		if (t >= 0 && v >= 0) {
			_path._path = 1;
			_path.result = Eigen::ArrayXd(4);
			_path.result << t, -_PI / 2, -u, -v;
			_path.direction = { 'L', 'R', 'S', 'R' };
		}
	}

	return _path;
}

//
_Path_Function LeftStraightLeft90XRight(f64 x, f64 y, f64 phi) {
	_Path_Function _path;
	f64 zeta = x + std::sin(phi),
		eeta = y - 1 - std::cos(phi),
		u1 = -1, theta = -1;
	polar(zeta, eeta, u1, theta);

	if (u1 >= 2.0) {
		f64 t = mod2pi(theta);
		f64 u = u1 - 2.0;
		f64 v = mod2pi(phi - t - _PI / 2);
		if (t >= 0 && v >= 0) {
			_path._path = 1;
			_path.result = Eigen::ArrayXd(4);
			_path.result << t, u, _PI / 2, -v;
			_path.direction = { 'L', 'S', 'L', 'R' };
		}
	}

	return _path;
}

//
_Path_Function LeftXRight90StraightLeft90XRight(f64 x, f64 y, f64 phi) {
	_Path_Function _path;
	f64 zeta = x + std::sin(phi),
		eeta = y - 1 - std::cos(phi),
		u1 = -1, theta = -1;
	polar(zeta, eeta, u1, theta);

	if (u1 >= 4.0) {
		f64 u = sqrt(u1 * u1 - 4.0) - 4.0;
		f64 A = std::atan2(2, sqrt(u1 * u1 - 4.0));
		f64 t = mod2pi(theta + A + _PI / 2);
		f64 v = mod2pi(t - phi);
		if (t >= 0 && v >= 0) {
			_path._path = 1;
			_path.result = Eigen::ArrayXd(5);
			_path.result << t,  -_PI / 2, -u, -_PI/2, v;
			_path.direction = { 'L', 'R', 'S', 'L', 'R'};
		}
	}

	return _path;
}

//
Eigen::ArrayXd timeflip(Eigen::ArrayXd td) {
	Eigen::ArrayXd _td = Eigen::ArrayXd(td.size());
	for (int ii = 0; ii < td.size(); ii++)
		_td(ii) = -td(ii);
	return _td;
}

//
std::vector<l8> reflect(std::vector<l8> direction) {
	std::vector<l8> _direction;
	for (int ii = 0; ii < direction.size(); ii++) {
		if (direction[ii] == 'L') _direction.push_back('R');
		if (direction[ii] == 'R') _direction.push_back('L');
		if (direction[ii] == 'S') _direction.push_back('S');
	}
	return _direction;
}

// 
std::vector<_Path> GeneratePath(
	Eigen::Vector3d q0, Eigen::Vector3d q1, f64 max_curvature, f64 step_size
) {
	f64 dx = q1(0) - q0(0), dy = q1(1) - q0(1), dth = q1(2) - q0(2);
	f64 c = std::cos(q0(2)), s = std::sin(q0(2));
	f64 x = (c * dx + s * dy) * max_curvature,
		y = (-s * dx + c * dy) * max_curvature; // Ðý×ª¾ØÕó
	step_size *= max_curvature;
	std::vector<_Path> paths;

	using FuncPtr = _Path_Function(*)(f64, f64, f64);
	FuncPtr Functions []  = {
		LeftStraightLeft, LeftStraightRight,
		LeftXRightXLeft, LeftXRightLeft, LeftRightXLeft,
		LeftRightXLeftRight, LeftXRightLeftXRight,
		LeftXRight90StartghtLeft, LeftXRight90StraightRight,
		LeftStraightRight90XLeft, LeftStraightLeft90XRight,
		LeftXRight90StraightLeft90XRight
	};

	for (int ii = 0; ii < _MOTION_TYPES; ii++) {
		_Path_Function _path = Functions[ii](x, y, dth);
		if (_path._path) {
			for (int ii = 0; ii < _path.result.size(); ii++) {
				f64 distance = _path.result(ii);
				if (0.1 * _path.result.abs().sum() < std::abs(distance) &&
					std::abs(distance) < step_size) {
					std::cout << "Step size is too large for Reeds-Shepp paths.\n";
					paths.clear();
					return paths;
				}
			}
			paths = set_path(paths, _path, step_size);
		}
		_path = Functions[ii](-x, y, -dth);
		if (_path._path) {
			for (int ii = 0; ii < _path.result.size(); ii++) {
				f64 distance = _path.result(ii);
				if (0.1 * _path.result.abs().sum() < std::abs(distance) &&
					std::abs(distance) < step_size) {
					std::cout << "Step size is too large for Reeds-Shepp paths.\n";
					paths.clear();
					return paths;
				}
			}
			_path.result = timeflip(_path.result);
			paths = set_path(paths, _path, step_size);
		}
		_path = Functions[ii](x, -y, -dth);
		if (_path._path) {
			for (int ii = 0; ii < _path.result.size(); ii++) {
				f64 distance = _path.result(ii);
				if (0.1 * _path.result.abs().sum() < std::abs(distance) &&
					std::abs(distance) < step_size) {
					std::cout << "Step size is too large for Reeds-Shepp paths.\n";
					paths.clear();
					return paths;
				}
			}
			_path.direction = reflect(_path.direction);
			paths = set_path(paths, _path, step_size);
		}
		_path = Functions[ii](-x, -y, dth);
		if (_path._path) {
			for (int ii = 0; ii < _path.result.size(); ii++) {
				f64 distance = _path.result(ii);
				if (0.1 * _path.result.abs().sum() < std::abs(distance) &&
					std::abs(distance) < step_size) {
					std::cout << "Step size is too large for Reeds-Shepp paths.\n";
					paths.clear();
					return paths;
				}
			}
			_path.result = timeflip(_path.result);
			_path.direction = reflect(_path.direction);
			paths = set_path(paths, _path, step_size);
		}
	}
	return paths;
}

// ²åÖµ
std::vector<Eigen::ArrayXd> CalculateInterpolateDistsList(
	Eigen::ArrayXd lengths, f64 step_size
) {
	std::vector<Eigen::ArrayXd> interpolate_dists_list;

	for (int ii = 0; ii < lengths.size(); ii++) {
		f64 len = lengths(ii);
		f64 d_dist = step_size * (len >= 0 ? 1 : -1);
		Eigen::ArrayXd interp_dists =
			Eigen::ArrayXd::LinSpaced(l32(len / d_dist) + 2, 0.0, len);
		interpolate_dists_list.push_back(interp_dists);
	}

	return interpolate_dists_list;
}

// ²åÖµ
// ÊäÈë1£º¾àÀë
Interpolate interpolate(f64 dist, f64 len, l8 mode,
	f64 max_curvature, f64 origin_x, f64 origin_y, f64 origin_yaw) {
	Interpolate interp;
	interp.direction = len > 0.0;
	if (mode == 'S') {
		interp.x = origin_x + dist / max_curvature * std::cos(origin_yaw);
		interp.y = origin_y + dist / max_curvature * std::sin(origin_yaw);
		interp.yaw = origin_yaw;
	}
	else {
		f64 ldx = std::sin(dist) / max_curvature,
			ldy = 0.0,
			yaw = -1;
		if (mode == 'L') {
			ldy = (1.0 - std::cos(dist)) / max_curvature;
			yaw = origin_yaw + dist;
		}
		else if (mode == 'R') {
			ldy = (1.0 - std::cos(dist)) / -max_curvature;
			yaw = origin_yaw - dist;
		}
		f64 gdx = std::cos(-origin_yaw) * ldx + std::sin(-origin_yaw) * ldy,
			gdy = -std::sin(-origin_yaw) * ldx + std::cos(-origin_yaw) * ldy;
		interp.x = origin_x + gdx;
		interp.y = origin_y + gdy;
		interp.yaw = yaw;
	}
	return interp;
}

LocalCourse GenerateLocalCourse(Eigen::ArrayXd lengths,
	std::vector<l8> modes, f64 max_curvature, f64 step_size
	) {
	LocalCourse lc;
	std::vector<Eigen::ArrayXd> interpolate_dists_list = CalculateInterpolateDistsList(
		lengths, step_size * max_curvature
	);

	f64 origin_x = 0.0, origin_y = 0.0, origin_yaw = 0.0;

	for (int ii = 0; ii < lengths.size(); ii++) {
		for (int jj = 0; jj < interpolate_dists_list[ii].rows(); jj++) {
			Interpolate interp = interpolate(interpolate_dists_list[ii](jj), lengths(ii),
				modes[ii], max_curvature, origin_x, origin_y, origin_yaw);
			lc.x.push_back(interp.x);
			lc.y.push_back(interp.y);
			lc.yaw.push_back(interp.yaw);
			lc.directions.push_back(interp.direction);
		}
		origin_x = lc.x.back();
		origin_y = lc.y.back();
		origin_yaw = lc.yaw.back();
	}

	return lc;
}

std::vector<_Path> CalculatePaths(Eigen::Vector3d start, Eigen::Vector3d goal, f64 maxc, f64 step_size) {
	std::vector<_Path> paths = GeneratePath(start, goal, maxc, step_size);
	for (int ii = 0; ii < paths.size(); ii++) {
		_Path* path = &paths[ii];
		LocalCourse result = GenerateLocalCourse(path->lengths, path->ctypes,
			maxc, step_size);
		for (int jj = 0; jj < result.x.size(); jj++) {
			f64 xx = result.x[jj] * std::cos(-start(2)) +
				result.y[jj] * std::sin(-start(2)) + start(0);
			f64 yy = result.x[jj] * -std::sin(-start(2)) +
				result.y[jj] * std::cos(-start(2)) + start(1);
			f64 yaw = pi_2_pi(result.yaw[jj] + start(2));
			path->x.push_back(xx);
			path->y.push_back(yy);
			path->yaw.push_back(yaw);
		}
		path->directions = result.directions;
		Eigen::ArrayXd _path_length(path->lengths.size());
		for (int jj = 0; jj < path->lengths.size(); jj++) {
			_path_length(jj) = path->lengths(jj) / maxc;
		}
		path->lengths = _path_length;
		path->L = path->L / maxc;
	}

	return paths;
}

//
_Path ReedsSheppPathPlanning(Eigen::Vector3d start, Eigen::Vector3d goal, 
	f64 maxc, f64 step_size) {
	std::vector<_Path> paths = CalculatePaths(start, goal, maxc, step_size);
	if (paths.size() == 0) {
		_Path _path;
		return _path;
	}

	f64 _minL = 1e12; l32 _minIndex = 1e10;
	for (int ii = 0; ii < paths.size(); ii++) {
		if (_minL > abs(paths[ii].L)) {
			_minL = abs(paths[ii].L);
			_minIndex = ii;
		}
	}

	return paths[_minIndex];
}

void test_RSP() {
	Eigen::Vector3d start, goal;
	start << -1.0, -4.0, -20 / 180.0 * _PI;
	goal << 5.0, 5.0, 25.0 / 180.0 * _PI;
	f64 curvature = 0.1, step_size = 0.05;
	_Path _path = ReedsSheppPathPlanning(start, goal, curvature, step_size);
}

#endif