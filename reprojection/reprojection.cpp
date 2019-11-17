// reprojection.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "opencv2/opencv.hpp"
#include "Eigen/Core"
#include <iostream>
#include <fstream>
#include <vector>

struct Camera {
	Eigen::Matrix3f in_mat, rot;
	Eigen::Vector3f T;

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
}mesh;

std::vector<cv::Mat> imgs;
std::vector<Camera> cameras;


void read_mesh(std::string file_name) {
	std::ifstream fin(file_name);
	std::string off;
	fin >> off;
	

	int t;
	int pts_n, faces_n;
	fin >> pts_n >> faces_n >> t;

	for (int i = 1; i <= pts_n; i++) {
		float x, y, z;
		fin >> x >> y >> z;
		mesh.pts.push_back(Eigen::Vector3f(x, y, z));
	}
	for (int i = 1; i <= faces_n; i++) {
		int x, y, z;
		fin >> t>>x >> y >> z;
		//mesh.pts.push_back(Eigen::Vector3i(x, y, z));
	}
}

void read_camera(std::string file_name) {
	std::ifstream fin(file_name);
	int id;
	while (fin >> id) {
		Camera camera;
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				fin >> camera.in_mat(i, j);
			}
		}
		float x;
		fin >> x >> x;
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				fin >> camera.rot(i, j);
			}
			fin >> camera.T(i);
		}
		cameras.push_back(camera);
	}
}
void read_image(const char* format, int n) {
	for (int i = 1; i <= n; i++) {
		cv::Mat img = cv::imread(cv::format(format, i));
		imgs.push_back(img);
	}
}

void reprojection() {
	for (int i = 0; i < cameras.size(); i++) {
		cv::Mat show_img;
		imgs[i].copyTo(show_img);
		std::cout << mesh.pts.size() << std::endl;
		for (auto pt : mesh.pts) {
			auto pt_2d = cameras[i].project(pt);
			/*std::cout << pt(0) << " " << pt(1) << " " << pt(2) << std::endl;
			std::cout << cameras[0].in_mat << std::endl;
			std::cout << cameras[0].rot << std::endl;
			std::cout << cameras[0].T(0)<<" "<<cameras[0].T(1)<<" "<<cameras[0].T(2) << std::endl;
			std::cout << pt_2d(0)<<" "<<pt_2d(1) << std::endl;*/
			cv::circle(show_img, cv::Point(pt_2d(0), pt_2d(1)), 1, cv::Scalar(0, 0, 255));
		}
		cv::imshow("debug", show_img);
		cv::waitKey();
	}
}


int main()
{
	/*read_camera("F:/rzshao/cmvs(4)/cmvs(4)/OfmStereo2/OfmStereo2/lybe/calibParams.txt");
	read_image("F:/rzshao/cmvs(4)/cmvs(4)/OfmStereo2/OfmStereo2/lybe/lybe_%d_42.png", cameras.size());
	read_mesh("F:/rzshao/cmvs(4)/cmvs(4)/OfmStereo2/OfmStereo2/lybe/lybe_42.off");*/
	read_camera("F:/rzshao/cmvs(4)/cmvs(4)/OfmStereo2/OfmStereo2/dinoSparseRing/calibParams.txt");
	read_image("F:/rzshao/cmvs(4)/cmvs(4)/OfmStereo2/OfmStereo2/dinoSparseRing/dinoSR%04d.png", cameras.size());
	read_mesh("F:/rzshao/cmvs(4)/cmvs(4)/OfmStereo2/OfmStereo2/dinoSparseRing/mesh.off");
	reprojection();	


    return 0;
}

