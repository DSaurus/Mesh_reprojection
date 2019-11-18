
#include "stdafx.h"
#include "render.h"

void rend(Mesh mesh, Camera cam, cv::Mat &out) {
	std::vector<Eigen::Vector3f> pts3d;
	int r_max = -1e9;
	for (auto pt : mesh.pts) {
		auto pt3d = cam.project3d(pt);
		r_max = std::max((int)pt3d(0), r_max);
		r_max = std::max((int)pt3d(1), r_max);
		pts3d.push_back(pt3d);
		
	}
	out = cv::Mat(r_max + 10, r_max + 10, CV_32F, cv::Scalar(1e9));
	cv::Mat debug_line(r_max + 10, r_max + 10, CV_8UC3);
	float out_max = -1e9, out_min = 1e9;

	for (auto fc : mesh.faces) {
		auto pt0 = pts3d[fc(0)];
		auto pt1 = pts3d[fc(1)];
		auto pt2 = pts3d[fc(2)];
		auto v1 = pt1 - pt0;
		auto v2 = pt2 - pt0;
		

		int x_min = 1e9, x_max = -1e9, y_min = 1e9, y_max = -1e9;
		x_min = std::min(std::min(pt0(0), pt1(0)), pt2(0));
		y_min = std::min(std::min(pt0(1), pt1(1)), pt2(1));
		x_max = std::max(std::max(pt0(0), pt1(0)), pt2(0));
		y_max = std::max(std::max(pt0(1), pt1(1)), pt2(1));
		cv::line(debug_line, cv::Point2f(pt0(0), pt0(1)), cv::Point2f(pt1(0), pt1(1)), cv::Scalar(0, 0, 255));
		cv::line(debug_line, cv::Point2f(pt2(0), pt2(1)), cv::Point2f(pt1(0), pt1(1)), cv::Scalar(0, 0, 255));
		cv::line(debug_line, cv::Point2f(pt0(0), pt0(1)), cv::Point2f(pt2(0), pt2(1)), cv::Scalar(0, 0, 255));
		
		for (int x = x_min; x <= x_max; x++) {
			for (int y = y_min; y <= y_max; y++) {
				Eigen::Vector2f v = Eigen::Vector2f((float)x, (float)y) - Eigen::Vector2f(pt0(0), pt0(1));
				float a = (v(0)*v2(1) - v2(0)*v(1)) / (v1(0)*v2(1) - v2(0)*v1(1));
				float b = (v(0)*v1(1) - v1(0)*v(1)) / (v2(0)*v1(1) - v2(1)*v1(0));
				if (a <= 0 || b <= 0 || a + b > 1) continue;
				if (x < 0 || y < 0 || x >= out.rows || y >= out.rows) continue;
				auto d = a*v1 + b*v2 + pt0;
				out.at<float>(y, x) = std::min(out.at<float>(y, x), d(2));
				out_max = std::max(out_max, d(2));
				out_min = std::min(out_min, d(2));
			}
		}

	}
	std::cout << out_min << " " << out_max << std::endl;
	out_min = 0.6; out_max = 0.7;
	
	cv::Mat debug_show(r_max + 10, r_max + 10, CV_8U, cv::Scalar(0));
	for (int i = 0; i < out.rows; i++) {
		for (int j = 0; j < out.cols; j++) {
			if (out.at<float>(i, j) > 1e8) debug_show.at<uchar>(i, j) = 0;
			else {
				debug_show.at<uchar>(i, j) = std::min((float)250, 250 - (out.at<float>(i, j) - out_min) / (out_max - out_min) * 250);
			}
		}
	}
	out = debug_show;
	//cv::imshow("debug_show", debug_show);
	//cv::waitKey();
}