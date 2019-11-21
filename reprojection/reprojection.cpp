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
		cv::Mat img = cv::imread(cv::format(format, i-1));
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
				//show_img.at<cv::Vec3b>(r, c+bais) = imgs[i].at<cv::Vec3b>(r, c) / 3 + cv::Vec3b(0, out.at<uchar>(r, c), 0) / 3 * 2;
				if (out.at<uchar>(r, c) == 0) {
					show_img.at<cv::Vec3b>(r, c + bais) = imgs[i].at<cv::Vec3b>(r, c) / 2 + cv::Vec3b(120, 0, 0);
				}
				else {
					show_img.at<cv::Vec3b>(r, c + bais) = imgs[i].at<cv::Vec3b>(r, c) / 2 + cv::Vec3b(0, 0, 120);
				}
				
			}
		}
		cv::imshow("debug", show_img);
		cv::imwrite(cv::format("./%d.png", i), show_img);
		cv::waitKey(30);
	}
}

struct Flow_Visual {
	std::vector<Eigen::Vector4i> ptpairs;
	std::vector<Eigen::Vector3f> pts3d;
	std::vector<Eigen::Vector2i> pts2d;
	void read_flow(std::string file_name) {
		float x, y, z;
		int r1, r2, c1, c2;
		std::ifstream fin(file_name);
		while (fin >> x >> y >> z >> r1 >> c1 >> r2 >> c2) {
			pts3d.push_back(Eigen::Vector3f(x, y, z));
			pts2d.push_back(Eigen::Vector2i(r1, c1));
		}
	}

	void flow_show(std::string file_name, cv::Mat img1, cv::Mat img2, Camera cam1, Camera cam2) {
		float x, y, z;
		int r1, r2, c1, c2;
		std::ifstream fin(file_name);
		std::set<std::pair<int, int>> Count;
		cv::Mat flow_map(img1.rows, img1.cols, CV_32F, cv::Scalar(0));
		cv::Mat map_show(img1.rows, img1.cols, CV_8U, cv::Scalar(0, 0, 0));
		cv::Mat flow_img(img1.rows, img1.cols, CV_8UC3, cv::Scalar(0, 0, 0));

		auto check_intensity = [](std::set<std::pair<int, int>> Count, int r1, int c1, int bais) {
			int flag = 0;
			for (int r = -bais; r <= bais; r++) {
				for (int c = -bais; c <= bais; c++) {
					if (Count.count(std::make_pair(r1 + r, c1 + c)) > 0) flag = 1;
				}
			}
			return flag;
		};

		while (fin >> x >> y >> z >> r1 >> c1 >> r2 >> c2) {
			pts3d.push_back(Eigen::Vector3f(x, y, z));
			auto pt1 = cam1.project(Eigen::Vector3f(x, y, z));
			auto pt2 = cam2.project(Eigen::Vector3f(x, y, z));
			flow_map.at<float>(r1, c1) = atan2(r2, c2);
			flow_map.at<float>(r1, c1) = (pt1 - pt2).norm();
			flow_map.at<float>(r1, c1) = (img2.at<cv::Vec3b>(pt2(1), pt2(0)) - img1.at<cv::Vec3b>(pt1(1), pt1(0))).dot(img2.at<cv::Vec3b>(pt2(1), pt2(0)) - img1.at<cv::Vec3b>(pt1(1), pt1(0)));
			flow_img.at<cv::Vec3b>(pt2(1), pt2(0)) = (img1.at<cv::Vec3b>(pt1(1), pt1(0)));
		}
		cv::normalize(flow_map, map_show, 0, 255, cv::NORM_MINMAX, map_show.type());
		cv::imshow("flow_loc", img1);
		cv::imshow("flow_ref", img2);
		cv::imshow("flow_map", map_show);
		cv::imshow("flow_img", flow_img);
		cv::waitKey();

		int block_n = 10;
		int block_padding = 20;
		int empty_padding = 5;
		int block_width = img1.cols / block_n * 2 + block_padding * 2 + 2*empty_padding;
		int width_bais = 0;

		cv::Mat show(img1.rows, block_width * block_n, CV_8UC3);

		for (int block = 0; block < block_n; block++) {
			int lx1 = (float)block*img1.cols / block_n, rx1 = (float)(block + 1)*img1.cols / block_n;
			cv::Rect rect1(lx1, 0, rx1 - lx1, img1.rows);
			img1(rect1).copyTo(show(cv::Rect(width_bais, 0, rx1-lx1, img1.rows)));
			width_bais += rx1 - lx1 + empty_padding;


			int lx2 = std::max(0, lx1 - block_padding);
			int rx2 = std::min(img1.cols, rx1 + block_padding);
			cv::Rect rect2(lx2, 0, rx2 - lx2, img1.rows);
			img1(rect2).copyTo(show(cv::Rect(width_bais, 0, rx2 - lx2, img1.rows)));
			width_bais += rx2 - lx2 + empty_padding;

			int width_base = width_bais - (rx1 - lx1) - (rx2 - lx2) - 2*empty_padding;

			for (auto pt : pts3d) {
				auto pt2d = cam1.project(pt);
				if (check_intensity(Count, pt2d(1), pt2d(0), 15)) continue;
				if (pt2d(0) < lx1 || pt2d(0) > rx1) continue;
				Count.insert(std::make_pair(pt2d(1), pt2d(0)));
				auto pt2d_y = cam2.project(pt);
				cv::line(show, cv::Point(pt2d(0), pt2d(1)) + cv::Point(width_base - lx1, 0), 
					cv::Point(pt2d_y(0), pt2d_y(1)) + cv::Point(width_base + rx1 - lx1 - lx2 + empty_padding, 0), cv::Scalar(255, 0, 0));
			}
			cv::imshow("flow", show);
			cv::waitKey(30);
		}

		cv::imwrite("F:/rzshao/cmvs(4)/DSR_cmvs/refine/point_6_7.10.png", show);
	}
}fv;

struct Depth_Visual {
	std::vector<Eigen::Vector2i> pts2d;
	std::vector<Eigen::Vector2f> pts2d_f;
	std::vector<Eigen::Vector3f> normals;
	std::vector<Eigen::Vector3f> pts3d;
	void triangle(cv::Mat img, Camera cam, std::string output_name) {
		std::vector<Eigen::Vector3i> faces;
		cv::Subdiv2D div2d;
		float x_min = 1e9, y_min = 1e9, x_max = -1e9, y_max = -1e9;
		for (int i = 0; i < pts2d_f.size(); i++) {
			auto pt2d = pts2d_f[i];
			x_min = std::min(x_min, (float)pt2d(0));
			x_max = std::max(x_max, (float)pt2d(0));
			y_min = std::min(y_min, (float)pt2d(1));
			y_max = std::max(y_max, (float)pt2d(1));
		}
		cv::Point2f pt_bais = cv::Point2f((x_max - x_min)*0.1, (y_max - y_min)*0.1);
		div2d.initDelaunay(cv::Rect2f(cv::Point2f(x_min, y_min) - pt_bais, cv::Point2f(x_max, y_max) + pt_bais));
		
		for (int i = 0; i < pts2d_f.size(); i++) {
			auto pt2d = cv::Point2f(pts2d_f[i](0), pts2d_f[i](1));
			div2d.insert(pt2d);
		}
		std::vector<cv::Vec6f> triList;
		div2d.getTriangleList(triList);
		for (auto fc : triList) {
			cv::Point2f x(fc[0], fc[1]), y(fc[2], fc[3]), z(fc[4], fc[5]);
			int xid, yid, zid, temp;
			div2d.locate(x, temp, xid);
			div2d.locate(y, temp, yid);
			div2d.locate(z, temp, zid);
			xid -= 3; yid -= 3; zid -= 3;
			faces.push_back(Eigen::Vector3i(xid, yid, zid));
		}
		std::ofstream fout(output_name);
		//fout << "OFF" << std::endl;
		//fout << pts3d.size() << " " << faces.size() << " 0" << std::endl;
		for (auto pt : pts3d) {
			fout << "v " << pt(0) << " " << pt(1) << " " << pt(2) << std::endl;
		}
		for (auto n : normals) {
			fout << "vn " << n(0) << " " << n(1) << " " << n(2) << std::endl;
		}
		for (auto f : faces) {
			fout << "f" << " " << f(0) << " " << f(1) << " " << f(2) << std::endl;
		}
	}
	void depth_show(cv::Mat img, Camera cam) {
		cv::Mat out = cv::Mat(img.rows, img.cols, CV_32F, cv::Scalar(-1e9));
		for (int i = 0; i < pts2d.size(); i++) {
			auto pt2d = pts2d[i];
			auto pt3d = pts3d[i];
			//std::cout << (cam.project3d(pt3d))(2) << std::endl;
			out.at<float>(pt2d(0), pt2d(1)) = (cam.project3d(pt3d))(2);
		}
		cv::Mat debug_show(img.rows, img.cols, CV_8U, cv::Scalar(0));
		float i_max = -1e9, i_min = 1e9;
		for (int r = 0; r < img.rows; r++) {
			for (int c = 0; c < img.cols; c++) {
				if (out.at<float>(r, c) < -1e8) continue;
				i_max = std::max(i_max, out.at<float>(r, c));
				i_min = std::min(i_min, out.at<float>(r, c));
			}
		}
		std::cout << i_min << " " << i_max << std::endl;
		i_min -= std::abs(i_min)*0.1;
		for (int r = 0; r < img.rows; r++) {
			for (int c = 0; c < img.cols; c++) {
				if (out.at<float>(r, c) < -1e8) continue;
				debug_show.at<uchar>(r, c) = 250 * (out.at<float>(r, c) - i_min) / (i_max - i_min);
			}
		}
		cv::imshow("depth", debug_show);
		cv::imshow("color", img);
		cv::waitKey();
	}

	void read_depth(std::string file_name) {
		std::ifstream fin(file_name);
		int r, c;
		float x, y, z;
		while (fin >> r >> c >> x >> y >> z) {
			pts2d.push_back(Eigen::Vector2i(r, c));
			pts3d.push_back(Eigen::Vector3f(x, y, z));
		}
	}

	void read_filter(std::string file_name, Camera cam) {
		std::ifstream fin(file_name);
		float x, y, z, nx, ny, nz, score;
		pts3d.clear();
		pts2d_f.clear();
		while (fin >> x >> y >> z >> nx >> ny >> nz >> score) {
			pts3d.push_back(Eigen::Vector3f(x, y, z));
			pts2d_f.push_back(cam.project(Eigen::Vector3f(x, y, z)));
			//normals.push_back(Eigen::Vector3f(nx, ny, nz));
		}
	}
}ds;


int main()
{
	/*read_camera("F:/rzshao/cmvs(4)/cmvs(4)/OfmStereo2/OfmStereo2/lybe/calibParams.txt");
	read_image("F:/rzshao/cmvs(4)/cmvs(4)/OfmStereo2/OfmStereo2/lybe/lybe_%d_42.png", cameras.size());
	read_mesh("F:/rzshao/cmvs(4)/cmvs(4)/OfmStereo2/OfmStereo2/lybe/lybe_42.off");*/
	
	//read_camera("F:/rzshao/cmvs(4)/DSR_cmvs/dino/calibParams.txt");
	//read_image("F:/rzshao/cmvs(4)/DSR_cmvs/dino/image/dinoSR%04d.png", cameras.size());

	read_camera("F:/rzshao/cmvs(4)/DSR_cmvs/dinoRing/calibParams.txt");
	read_image("F:/rzshao/cmvs(4)/DSR_cmvs/dinoRing/image/dinoR%04d.png", cameras.size());

	//read_mesh("F:/rzshao/cmvs(4)/DSR_cmvs/output/mesh.off");
	//reprojection();	
	//ds.read_depth(cv::format("F:/rzshao/cmvs(4)/DSR_cmvs/output/depth%d.txt", 0));
	//ds.depth_show(imgs[0], cameras[0]);
	fv.read_flow("F:/rzshao/cmvs(4)/DSR_cmvs/output/point_9_10.16.txt");
	for (auto pt : fv.pts3d) ds.pts3d.push_back(pt);
	for (auto pt : fv.pts2d) ds.pts2d_f.push_back(Eigen::Vector2f(pt(0), pt(1)));
	//ds.depth_show(imgs[6], cameras[6]);
	ds.triangle(imgs[6], cameras[9], "F:/rzshao/cmvs(4)/DSR_cmvs/refine/mesh_9_10.16.obj");

	fv.flow_show("F:/rzshao/cmvs(4)/DSR_cmvs/output/point_9_10.16.txt", imgs[9], imgs[10], cameras[9], cameras[10]);
	/*for (int i = 0; i < 16; i++) {
		ds.read_filter(cv::format("F:/rzshao/cmvs(4)/DSR_cmvs/output/filter%d.txt", i), cameras[i]);
		ds.triangle(imgs[0], cameras[i], cv::format("F:/rzshao/cmvs(4)/DSR_cmvs/refine/filter%d.obj", i));
	}*/

	return 0;

	fv.read_flow("F:/rzshao/cmvs(4)/DSR_cmvs/dr_test/point_27_28.16.txt");
	for (auto pt : fv.pts3d) ds.pts3d.push_back(pt);
	for (auto pt : fv.pts2d) ds.pts2d_f.push_back(Eigen::Vector2f(pt(0), pt(1)));
	//ds.depth_show(imgs[6], cameras[6]);
	ds.triangle(imgs[6], cameras[27], "F:/rzshao/cmvs(4)/DSR_cmvs/refine/mesh_27_28.16.obj");

	fv.flow_show("F:/rzshao/cmvs(4)/DSR_cmvs/dr_test/point_27_28.16.txt", imgs[27], imgs[28], cameras[27], cameras[28]);
	
    return 0;
}

