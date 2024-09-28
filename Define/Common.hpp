#ifndef _COMMON_HPP_
#define _COMMON_HPP_

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <functional>
#include <iostream>
#include <vector>
#include <queue>

#define _MAX_STEER 0.6
#define _PI 3.1415926535
#define _XY_GRID_RESOLUTION 2.0 // meters
#define _YAW_GRID_RESOLUTION (15.0 / 180.0 * _PI) // rads
#define _MOTION_RESOLUTION 0.1 // meters
#define _N_STEER 20 // ?
#define _WB 3.0
#define _W  2.0
#define _LF 3.3
#define _LB 1.0
#define _BUBBLE_DIST 1.15 // 与转弯半径相关
#define _BUBBLE_R 2.37118 // 与转弯半径相关
#define _MOTION_TYPES 12     // 目前有十二种PATH

// Cost define
#define _SWITCH_BACK_COST 1e2
#define _BACK_COST        5.0
#define _YAW_CHANGE_COST  5.0
#define _YAW_COST         1.0
#define _H_COST           5.0

#define u32 unsigned int
#define u64 unsigned long
#define u8  unsigned char
#define f32 float
#define f64 double
#define l8  char
#define l32 int
#define l64 long long 
#define b1  bool

// cost-index pair
typedef class Cost_Index {
public:
	Cost_Index() {
	
	}

	Cost_Index(f64 cost, l32 index) {
		this->cost = cost;
		this->index = index;
	}

	f64 cost = -1;  // the cost
	l32 index = -1; // the index

	// operator overload
	b1 operator<(Cost_Index & a) const {
		return this->cost < a.cost;
	}

	b1 operator>(Cost_Index & b) const {
		return this->cost > b.cost;
	}
} CIpair;

struct cmp {
	b1 operator()(CIpair& a, CIpair& b) const {
		if (a.cost > b.cost) {
			return a.cost > b.cost;
		}
		else if (a.cost == b.cost){
			return a.index > b.index;
		}
	}
};

#endif 