#pragma once
#include "stdafx.h"
#include "opencv2/opencv.hpp"
#include "Eigen/Core"

struct Camera {
	Eigen::Matrix3f in_mat, rot;
	Eigen::Vector3f T;

	Eigen::Vector3f project3d(Eigen::Vector3f x) {
		x = rot*x;
		x = x + T;
		x = in_mat * x;
		return Eigen::Vector3f(x(0) / x(2), x(1) / x(2), x(2));
	}

	Eigen::Vector2f project(Eigen::Vector3f x) {
		x = rot*x;
		x = x + T;
		x = in_mat * x;
		return Eigen::Vector2f(x(0) / x(2), x(1) / x(2));
	}
};

struct Mesh {
	std::vector<Eigen::Vector3f> pts;
	std::vector<Eigen::Vector3i> faces;
};

void rend(Mesh mesh, Camera cam, cv::Mat &out);