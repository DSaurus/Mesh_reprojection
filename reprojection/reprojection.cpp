// reprojection.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "render.h"
#include <iostream>
#include <fstream>
#include <vector>

std::vector<cv::Mat> imgs;
std::vector<Camera> cameras;
Mesh mesh;

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
		fin >> t >> x >> y >> z;
		mesh.faces.push_back(Eigen::Vector3i(x, y, z));
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
		int bais = 0;
		cv::Mat show_img(imgs[i].rows, imgs[i].cols*3, CV_8UC3, cv::Scalar(0, 0, 0));
		for (int r = 0; r < imgs[i].rows; r++) {
			for (int c = 0; c < imgs[i].cols; c++) {
				show_img.at<cv::Vec3b>(r, c) = imgs[i].at<cv::Vec3b>(r, c);
			}
		}
		bais += imgs[i].cols;
		std::cout << mesh.pts.size() << std::endl;


		cv::Mat out;
		rend(mesh, cameras[i], out);

		/*for (auto pt : mesh.pts) {
			auto pt_2d = cameras[i].project(pt);
			cv::circle(show_img, cv::Point(pt_2d(0), pt_2d(1)), 1, cv::Scalar(0, 0, 255));
		}*/
		for (int r = 0; r < imgs[i].rows && r < out.rows; r++) {
			for (int c = 0; c < imgs[i].cols && c < out.cols; c++) {
				show_img.at<cv::Vec3b>(r, c + bais) = cv::Vec3b(out.at<uchar>(r, c), out.at<uchar>(r, c), out.at<uchar>(r, c));
			}
		}
		bais += imgs[i].cols;

		for (int r = 0; r < imgs[i].rows && r < out.rows; r++) {
			for (int c = 0; c < imgs[i].cols && c < out.cols; c++) {
				show_img.at<cv::Vec3b>(r, c+bais) = imgs[i].at<cv::Vec3b>(r, c) / 3 + cv::Vec3b(0, out.at<uchar>(r, c), 0) / 3 * 2;
			}
		}
		cv::imshow("debug", show_img);
		cv::imwrite(cv::format("./%d.png", i), show_img);
		cv::waitKey(30);
	}
}


int main()
{
	/*read_camera("F:/rzshao/cmvs(4)/cmvs(4)/OfmStereo2/OfmStereo2/lybe/calibParams.txt");
	read_image("F:/rzshao/cmvs(4)/cmvs(4)/OfmStereo2/OfmStereo2/lybe/lybe_%d_42.png", cameras.size());
	read_mesh("F:/rzshao/cmvs(4)/cmvs(4)/OfmStereo2/OfmStereo2/lybe/lybe_42.off");*/
	read_camera("F:/rzshao/cmvs(4)/dinoRing/dinoRing/calibParams.txt");
	read_image("F:/rzshao/cmvs(4)/dinoRing/dinoRing/silhouette/dinoR%04d.png", cameras.size());
	read_mesh("F:/rzshao/cmvs(4)/dinoRing/dinoRing/mesh.off");
	reprojection();	


    return 0;
}

