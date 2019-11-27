// reprojection.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "render.h"
#include "Eigen/Geometry"
#include <iostream>
#include <fstream>
#include <vector>

std::vector<cv::Mat> imgs;
std::vector<Camera> cameras;
Mesh mesh;

void read_color_obj(std::string file_name) {
	std::ifstream fin(file_name);
	for (int i = 0; i < mesh.pts.size(); i++) {
		auto pt = mesh.pts[i];
		//auto n = mesh.normals[i];
		auto &col = mesh.colors[i];
		//fout << "vn " << n[0] << " " << n[1] << " " << n[2] << "\n";
		std::string temp;
		fin >> temp;
		fin >> pt[0] >> pt[1] >> pt[2] >> col[0] >> col[1] >> col[2];
	}
	/*for (auto n : mesh.normals) {
		std::string temp;
		fin >> temp;
		fin >> n[0] >> n[1] >> n[2];
	}*/
}


void save_color_obj(std::string file_name) {
	std::ofstream fout(file_name);
	for (int i = 0; i < mesh.pts.size(); i++) {
		auto pt = mesh.pts[i];
		//auto n = mesh.normals[i];
		auto col = mesh.colors[i];
		//fout << "vn " << n[0] << " " << n[1] << " " << n[2] << "\n";
		fout << "v " << pt[0] << " " << pt[1] << " " << pt[2] << " " << col[0] << " " << col[1] << " " << col[2] << "\n";
	}
	for (auto n : mesh.normals) {
		fout << "f " << n[0] << " " << n[1] << " " << n[2] << "\n";
	}
}

void read_xyz(std::string file_name) {
	std::ifstream fin(file_name);
	float x, y, z, nx, ny, nz;
	while (fin >> x >> y >> z >> nx >> ny >> nz) {
		mesh.pts.push_back(Eigen::Vector3f(x, y, z));
		mesh.normals.push_back(Eigen::Vector3f(nx, ny, nz));
	}
}

void save_xyz(std::string file_name) {
	std::ofstream fout(file_name);
	for (int i = 0; i < mesh.pts.size(); i++) {
		auto pt = mesh.pts[i];
		auto n = mesh.normals[i];
		fout << pt[0] << " " << pt[1] << " " << pt[2] << " " << n[0] << " " << n[1] << " " << n[2] << std::endl;
	}
}

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

void save_mesh(std::string file_name) {
	std::ofstream fout(file_name);
	fout << "OFF" << std::endl;

	int t;
	int pts_n, faces_n;
	fout << mesh.pts.size() << " " << mesh.faces.size() << " 0" << std::endl;

	for (auto pt : mesh.pts) {
		fout << pt[0] << " " << pt[1] << " " << pt[2] << std::endl;
	}
	for (auto fc : mesh.faces) {
		fout << 3 << " " << fc[0] << " " << fc[1] << " " << fc[2] << std::endl;
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

	void flow_filter(std::string file_name, cv::Mat img1, cv::Mat img2, Camera cam1, Camera cam2) {
		std::ifstream fin(file_name);
		float x, y, z;
		int r1, c1, r2, c2;
		std::vector<Eigen::Vector3f> pts3d;
		std::vector<Eigen::Vector4i> vec4;
		while (fin >> x >> y >> z >> r1 >> c1 >> r2 >> c2) {
			pts3d.push_back(Eigen::Vector3f(x, y, z));
			vec4.push_back(Eigen::Vector4i(r1, c1, r2, c2));
		}

		auto get_image_patch = [](cv::Mat img, Eigen::Vector2f pt, std::vector<Eigen::Vector3f> &vec, int radius) {
			vec.clear();
			if (pt[1] - radius < 0 || pt[1] + radius >= img.rows) return;
			if (pt[0] - radius < 0 || pt[0] + radius >= img.cols) return;
			for (int r = pt[1] - radius; r <= pt[1] + radius; r++) {
				for (int c = pt[0] - radius; c <= pt[0] + radius; c++) {
					auto pix = img.at<cv::Vec3b>(r, c);
					vec.push_back(Eigen::Vector3f(pix[0], pix[1], pix[2]));
				}
			}
		};

		std::map<int, float> patch_cost;
		std::vector<float> cost;
		for (int i = 0; i < pts3d.size(); i++) {
			auto pt3d = pts3d[i];
			auto pt1 = cam1.project(pt3d);
			auto pt2 = cam2.project(pt3d);

			std::vector<Eigen::Vector3f> vec1, vec2;
			
			int radius = 1;
			get_image_patch(img1, pt1, vec1, radius);
			get_image_patch(img2, pt2, vec2, radius);
			if (vec1.size() == 0 || vec2.size() == 0) continue;
			
			float ans = 0;
			for (int j = 0; j < vec1.size(); j++) {
				ans += (vec1[j] - vec2[j]).norm();
			}
			patch_cost[i] = ans;
			cost.push_back(ans);
		}

		std::sort(cost.begin(), cost.end());
		float cost_threshold = cost[std::min(cost.size() - 1, pts3d.size() * 9 / 10)];
		
		std::ofstream fout(file_name);
		for (int i = 0; i < pts3d.size(); i++) {
			auto pt = cam1.project(pts3d[i]);
			if (patch_cost.count(i) > 0) {
				if (patch_cost[i] < cost_threshold) {
					cv::circle(img1, cv::Point(pt(0), pt(1)), 1, cv::Scalar(0, 255, 0));
					fout << pts3d[i][0] << " " << pts3d[i][1] << " " << pts3d[i][2] << " " << vec4[i][0] << " " << vec4[i][1] << " " << vec4[i][2] << " " << vec4[i][3] << "\n";
				}
				else {
					cv::circle(img1, cv::Point(pt(0), pt(1)), 1, cv::Scalar(0, 0, 255));
				}
			}
			else {
				cv::circle(img1, cv::Point(pt(0), pt(1)), 1, cv::Scalar(255, 0, 0));
			}
		}
		cv::imshow("debug", img1);
		cv::waitKey(30);

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

void mesh_translation(Mesh &mesh, Eigen::Vector3f T) {
	for (auto &pt : mesh.pts) {
		pt += T;
	}
}

void clean_reprojection(Mesh &mesh) {
	std::map<int, int> valid;
	std::map<int, int> invalid;
	int ptid = 0;
	std::cout << imgs[0].type() << std::endl;
	for (auto pt : mesh.pts) {
		for (int i = 0; i < cameras.size(); i++) {
			auto cam = cameras[i];
			auto pt2d = cam.project(pt);
			if (pt2d(0) < 0 || pt2d(1) < 0 || pt2d(0) >= imgs[i].cols || pt2d(1) >= imgs[i].rows) continue;
			int r = pt2d(1), c = pt2d(0);
			if (imgs[i].at<cv::Vec3b>(r, c) == cv::Vec3b(0, 0, 0)) invalid[ptid] = 1;
			else valid[ptid] = 1;
		}
		ptid++;
	}
	ptid = 0;
	std::vector<Eigen::Vector3f> pts, normals;
	for (auto pt : mesh.pts) {
		if (invalid.count(ptid) > 0) {
			ptid++;
			continue;
		}
		if (valid.count(ptid) == 0) {
			ptid++;
			continue;
		}
		pts.push_back(pt);
		normals.push_back(mesh.normals[ptid]);
		ptid++;
	}
	mesh.pts.clear(); mesh.normals.clear();
	for (auto pt : pts) mesh.pts.push_back(pt);
	for (auto n : normals) mesh.normals.push_back(n);
}

void flow_filter(int x, int y) {
	/*fv.flow_filter(cv::format("F:/rzshao/cmvs(4)/DSR_cmvs/output/point_%d_%d.1.txt", x, y), imgs[x], imgs[y], cameras[x], cameras[y]);
	fv.flow_filter(cv::format("F:/rzshao/cmvs(4)/DSR_cmvs/output/point_%d_%d.2.txt", x, y), imgs[x], imgs[y], cameras[x], cameras[y]);
	fv.flow_filter(cv::format("F:/rzshao/cmvs(4)/DSR_cmvs/output/point_%d_%d.5.txt", x, y), imgs[x], imgs[y], cameras[x], cameras[y]);
	fv.flow_filter(cv::format("F:/rzshao/cmvs(4)/DSR_cmvs/output/point_%d_%d.10.txt", x, y), imgs[x], imgs[y], cameras[x], cameras[y]);
	fv.flow_filter(cv::format("F:/rzshao/cmvs(4)/DSR_cmvs/output/point_%d_%d.15.txt", x, y), imgs[x], imgs[y], cameras[x], cameras[y]);
	fv.flow_filter(cv::format("F:/rzshao/cmvs(4)/DSR_cmvs/output/point_%d_%d.16.txt", x, y), imgs[x], imgs[y], cameras[x], cameras[y]);*/
	fv.flow_filter(cv::format("F:/rzshao/cmvs(4)/DSR_cmvs/dr_test/point_%d_%d.1.txt", x, y), imgs[x], imgs[y], cameras[x], cameras[y]);
	fv.flow_filter(cv::format("F:/rzshao/cmvs(4)/DSR_cmvs/dr_test/point_%d_%d.2.txt", x, y), imgs[x], imgs[y], cameras[x], cameras[y]);
	fv.flow_filter(cv::format("F:/rzshao/cmvs(4)/DSR_cmvs/dr_test/point_%d_%d.5.txt", x, y), imgs[x], imgs[y], cameras[x], cameras[y]);
	fv.flow_filter(cv::format("F:/rzshao/cmvs(4)/DSR_cmvs/dr_test/point_%d_%d.10.txt", x, y), imgs[x], imgs[y], cameras[x], cameras[y]);
	fv.flow_filter(cv::format("F:/rzshao/cmvs(4)/DSR_cmvs/dr_test/point_%d_%d.15.txt", x, y), imgs[x], imgs[y], cameras[x], cameras[y]);
	fv.flow_filter(cv::format("F:/rzshao/cmvs(4)/DSR_cmvs/dr_test/point_%d_%d.16.txt", x, y), imgs[x], imgs[y], cameras[x], cameras[y]);
}

void translate_to_colmap() {
	std::ofstream fout("F:/rzshao/COLMAP-dev-windows/colmapPar.txt");
	//std::vector<Eigen::Vector3f> C_vector;
	//for (int i = 0; i < cameras.size(); i++) {
	//	
	//	// C = -R^T * T
	//	Eigen::Vector3f C = -cameras[i].rot.transpose()*cameras[i].T;
	//	C_vector.push_back(C);
	//	if (C_vector.size() == 100) std::cout << C_vector[99] << std::endl;
	//}
	//return;
	//Eigen::Vector3f ave(0, 0, 0), std(0, 0, 0);
	//for (auto c : C_vector) {
	//	ave += c;
	//	c = Eigen::Vector3f(c[0] * c[0], c[1] * c[1], c[2] * c[2]);
	//	std += c;
	//}
	//ave /= C_vector.size();
	//std /= C_vector.size();
	//for(int i = 0; i < 3; i++)
	//	std[i] -= ave[i]*ave[i];
	//std::cout << std[0] << std::endl;
	for (int i = 0; i < cameras.size(); i++) {
		Eigen::Quaternionf q;
		q = Eigen::Quaternionf(cameras[i].rot);
		/*auto C = C_vector[i];
		C -= ave; C /= std[0];
		Eigen::Vector3f T = -cameras[i].rot*C;*/
		auto T = cameras[i].T;
		fout << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << " " << T[0] << " " << T[1] << " " << T[2] <<"\n";
	}
}

void silhouette_clean() {
	read_camera("F:/rzshao/cmvs(4)/dinoSparseRing/calibParams.txt");
	std::cout << cameras.size() << std::endl;
	read_image("F:/rzshao/cmvs(4)/dinoSparseRing/silhouette3/dinoSR%04d.png", cameras.size());
	for (int i = 0; i < imgs.size(); i++) {
		cv::Mat temp(imgs[0].rows, imgs[0].cols, CV_8UC3);
		for (int r = 0; r < imgs[i].rows; r++) {
			for (int c = 0; c < imgs[i].cols; c++) {
				int flag = 0;
				for (int rr = -1; rr <= 1; rr++) {
					for (int cc = -1; cc <= 1; cc++) {
						if (r + rr < 0 || c + cc < 0 || r + rr >= temp.rows || c + cc >= temp.cols) continue;
						if (imgs[i].at<cv::Vec3b>(r + rr, c + cc) != cv::Vec3b(0, 0, 0)) {
							flag = 1;
							break;
						}
					}
				}
				if (flag) temp.at<cv::Vec3b>(r, c) = cv::Vec3b(255, 255, 255);
				else temp.at<cv::Vec3b>(r, c) = cv::Vec3b(0, 0, 0);
			}
		}
		temp.copyTo(imgs[i]);
	}
	read_xyz("F:/rzshao/COLMAP-dev-windows/stereo2/fused.xyz");

	mesh_translation(mesh, Eigen::Vector3f(-0.02, -0.02, -0.02));
	/*cv::Mat debug_show;
	imgs[0].copyTo(debug_show);
	for (auto pt : mesh.pts) {
	auto pt2d = cameras[0].project(pt);
	if (pt2d(0) < 0 || pt2d(1) < 0 || pt2d(0) >= imgs[0].cols || pt2d(1) >= imgs[0].rows) continue;
	auto pix = imgs[0].at<cv::Vec3b>(pt2d(1), pt2d(0));
	if (pix[0] < 10 && pix[1] < 10 && pix[2] < 10)
	cv::circle(debug_show, cv::Point(pt2d(0), pt2d(1)), 1, cv::Scalar(0, 255, 0));
	else cv::circle(debug_show, cv::Point(pt2d(0), pt2d(1)), 1, cv::Scalar(0, 0, 255));
	}
	cv::imshow("fuck", debug_show);
	cv::waitKey();
	return 0;*/
	clean_reprojection(mesh);
	save_xyz("360_final_clean.xyz");
}

void color_ref() {
	std::vector<Eigen::Vector3f> pts;
	std::vector<Eigen::Vector3f> colors;
	std::ifstream fin("color_ref.obj");
	std::string temp;
	float A = 0.94561935;
	while (fin >> temp) {
		if (temp[0] != 'v') break;
		float x, y, z, a, b, c;
		fin >> x >> y >> z;
		fin >> temp;
		fin >> x >> y >> z >> a >> b >> c;
		//pts.push_back(Eigen::Vector3f(x*A - 0.02, y*A - 0.02, z*A - 0.02));
		pts.push_back(Eigen::Vector3f(x-0.02, y-0.02, z-0.02));
		colors.push_back(Eigen::Vector3f(a, b, c));
	}
	std::vector<cv::Point3f> source_vec;
	for (auto pt : pts) {
		source_vec.push_back(cv::Point3f(pt[0], pt[1], pt[2]));
	}
	cv::Mat source = cv::Mat(source_vec).reshape(1);
	source.convertTo(source, CV_32F);
	cv::flann::KDTreeIndexParams indexParams(3);
	cv::flann::Index kdtree(source, indexParams);

	int tot = 0;
	std::vector<float> dist;
	for (auto pt : mesh.pts) {
		int query_num = 1;
		std::vector<float> vecQuery;
		for (int i = 0; i < 3; i++) vecQuery.push_back(pt(i));
		std::vector<int> vecIndex(query_num);
		std::vector<float> vecDist(query_num);
		cv::flann::SearchParams params(32);
		kdtree.knnSearch(vecQuery, vecIndex, vecDist, query_num, params);
		mesh.colors.push_back(colors[vecIndex[0]]);

		tot++;
		if (tot % 1000 == 0) std::cout << tot << std::endl;
	}
}

int main(){
	/*read_mesh("F:/rzshao/cmvs(4)/cmvs(4)/OfmStereo2/OfmStereo2/dino/360_final_clean.off");
	mesh_translation(mesh, Eigen::Vector3f(-0.02, -0.02, -0.02));
	save_mesh("F:/rzshao/cmvs(4)/cmvs(4)/OfmStereo2/OfmStereo2/dino/360_final.off");*/
	

	/*read_camera("F:/rzshao/cmvs(4)/cmvs(4)/OfmStereo2/OfmStereo2/lybe/calibParams.txt");
	read_image("F:/rzshao/cmvs(4)/cmvs(4)/OfmStereo2/OfmStereo2/lybe/lybe_%d_42.png", cameras.size());
	read_mesh("F:/rzshao/cmvs(4)/cmvs(4)/OfmStereo2/OfmStereo2/lybe/lybe_42.off");*/
	
	read_camera("F:/rzshao/cmvs(4)/dino/dino/calibParams.txt");
	read_image("F:/rzshao/cmvs(4)/dino/dino/dino%04d.png", cameras.size());
	//read_mesh("F:/rzshao/cmvs(4)/DSR_cmvs/registration/mesh_1_360.off");
	read_mesh("filter4.off");
	
	/*std::vector<std::vector<Eigen::Vector3f> > color_pts;
	cv::imshow("color", imgs[0]);
	for (int i = 0; i < cameras.size(); i += 3) {
		cv::Mat depth;
		rend(mesh, cameras[i], depth);
		get_texture(mesh, cameras[i], depth, imgs[i], color_pts);
	}
	
	for (int i = 0; i < color_pts.size(); i++) {
		Eigen::Vector3f color(0, 0, 0);
		int tot = 0;
		for (auto c : color_pts[i]) {
			if (c[0] <= 0.1) continue;
			color += c;
			tot++;
			break;
		}
		if (tot > 0) color /= tot;
		mesh.colors.push_back(color);
	}*/
	color_ref();
	//mesh.colors.resize(mesh.pts.size());
	//read_color_obj("color_test_colmap.obj");
	save_color_obj("color_test_final.obj");
	

	// colmap
	/*for (int i = 0; i < cameras.size(); i++) {
		cameras[i].in_mat(0, 0) = 3127.46;
		cameras[i].in_mat(1, 1) = 3130.13;
	}*/

	//ours
	mesh_translation(mesh, Eigen::Vector3f(0.02, 0.02, 0.02));

	for (int i = 0; i < cameras.size(); i++) {
		cv::Mat color(imgs[0].rows, imgs[0].cols, CV_8UC3, cv::Scalar(0, 0, 0));
		rend_color(mesh, cameras[i], color);
		cv::imshow("color", color);
		cv::imwrite(cv::format("F:/rzshao/cmvs(4)/dino/reprojection/imgs_%04d_0.png", i), color);
		cv::imwrite(cv::format("F:/rzshao/cmvs(4)/dino/reprojection/imgs_%04d_1.png", i), imgs[i]);
		cv::waitKey(30);
	}

	return 0;
	///*read_camera("F:/rzshao/cmvs(4)/DSR_cmvs/dinoRing/calibParams.txt");
	//read_image("F:/rzshao/cmvs(4)/DSR_cmvs/dinoRing/image/dinoR%04d.png", cameras.size());

	//for (int i = 0; i < 47; i++) {
	//	flow_filter(i, i + 1);
	//	flow_filter(i + 1, i);
	//}
	//
	//return 0;*/


	//reprojection();	
	//ds.read_depth(cv::format("F:/rzshao/cmvs(4)/DSR_cmvs/output/depth%d.txt", 0));
	//ds.depth_show(imgs[0], cameras[0]);
	//fv.read_flow("F:/rzshao/cmvs(4)/DSR_cmvs/output/point_9_10.16.txt");
	//for (auto pt : fv.pts3d) ds.pts3d.push_back(pt);
	//for (auto pt : fv.pts2d) ds.pts2d_f.push_back(Eigen::Vector2f(pt(0), pt(1)));
	//ds.depth_show(imgs[6], cameras[6]);
	//ds.triangle(imgs[6], cameras[9], "F:/rzshao/cmvs(4)/DSR_cmvs/refine/mesh_9_10.16.obj");

	//fv.flow_show("F:/rzshao/cmvs(4)/DSR_cmvs/output/point_9_10.16.txt", imgs[9], imgs[10], cameras[9], cameras[10]);
	for (int i = 0; i < 16; i++) {
		ds.read_filter(cv::format("F:/rzshao/cmvs(4)/DSR_cmvs/output/filter%d.txt", i), cameras[i]);
		ds.triangle(imgs[0], cameras[i], cv::format("F:/rzshao/cmvs(4)/DSR_cmvs/refine/filter%d.obj", i));
	}

	return 0;

	fv.read_flow("F:/rzshao/cmvs(4)/DSR_cmvs/dr_test/point_27_28.16.txt");
	for (auto pt : fv.pts3d) ds.pts3d.push_back(pt);
	for (auto pt : fv.pts2d) ds.pts2d_f.push_back(Eigen::Vector2f(pt(0), pt(1)));
	//ds.depth_show(imgs[6], cameras[6]);
	ds.triangle(imgs[6], cameras[27], "F:/rzshao/cmvs(4)/DSR_cmvs/refine/mesh_27_28.16.obj");

	fv.flow_show("F:/rzshao/cmvs(4)/DSR_cmvs/dr_test/point_27_28.16.txt", imgs[27], imgs[28], cameras[27], cameras[28]);
	
    return 0;
}

