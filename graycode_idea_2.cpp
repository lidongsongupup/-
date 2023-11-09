#include<iostream>
#include<vector>
#include<opencv2/opencv.hpp>
#include<pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h> //半径滤波器头文件
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <pcl/surface/mls.h>
#include <time.h>
#include <opencv2/core/types.hpp>

std::string path = "E:/2021.4.9/123/02/";
//std::string path = "E:/2021-3-17/11/";


int height = 5120;
int width = 5120;
int height_projecter = 1200;
int width_projecter = 1920;
cv::Mat decode_col(std::string );
cv::Mat decode_row(std::string);
void creat_gray_code();
void creat_gray_code_4();
cv::Mat phaseshift_col(int,int,std::string);
cv::Mat phaseshift_row(std::string);
cv::Mat convert(cv::Mat);
void convertType(const cv::Mat& srcImg, cv::Mat& distImg);
void get_chessboard_world_coords(std::vector<cv::Point3f>&, cv::Size, cv::Size);

void unwraping_idea_2(cv::Mat&, cv::Mat&, int ,int, std::string p);

void decode_row_5_idea_2(cv::Mat&, cv::Mat& ,cv::Mat&, std::string p);
void decode_col_5_idea_2(cv::Mat&, cv::Mat&, cv::Mat&, std::string p,int ,int);
void create_phase_Picture();
void calibrate();
void projecter_calibrate();
void reconstruct(int,int,int);
void imageFilter(cv::Mat&);
bool extract_items_corners();
bool find_PNP(void);
void find_project_view(void);
void correct_1(int &i, int &j, int pre, int post, cv::Mat& result_col, cv::Mat& rank_col_4, cv::Mat& rank_col_5);
void correct_2(int &i, int &j, int pre, int post, cv::Mat& result_col, cv::Mat& rank_col_4, cv::Mat& rank_col_5, cv::Mat& rank_col_6);
void correct_3(int& i, int& j, int pre, int post, cv::Mat& result_col, cv::Mat& rank_col_4, cv::Mat& rank_col_5);
void correct_4(int& i, int& j, int pre, int post, cv::Mat& result_col, cv::Mat& rank_col_4, cv::Mat& rank_col_5, cv::Mat& rank_col_6);


cv::Point2d undistortPoints(cv::Point2d);
cv::Mat phaseShift_fourStep(int, std::vector<cv::Mat>);


cv::Mat Dimension = cv::Mat(height, width, CV_64FC3, cv::Scalar::all(0));
cv::Mat color = cv::Mat(height, width, CV_64FC1, cv::Scalar::all(0));
//cv::Mat phaseShift_fourStep(int,const cv::Mat& img1, const cv::Mat& img2, const cv::Mat& img3, const cv::Mat& img4);


int x = 0;//像素偏移个数
int y = 8;//前后检查像素个数
int step = 12;//相移步数
int calibrate_number = 3;//标点图片组数
int item_number = 9;//贴的标记点数量
cv::Size2i corner_item_count(3, 3);//标记点网格尺寸
cv::Size2i corner_count(13, 11);  // 11*9 是把标定板横着放 9*11 是把标定板竖着放  不能弄反，否则会不符合代码逻辑
cv::Size2d corner_size(84, 84);//圆心间距



int sk = 1;
cv::Mat K2 = cv::Mat(height, width, CV_64FC1, cv::Scalar::all(0.0));
cv::Mat K3 = cv::Mat(height, width, CV_64FC1, cv::Scalar::all(0.0));
cv::Mat K1 = cv::Mat(height, width, CV_64FC1, cv::Scalar::all(255));
double _time_[200] = { 0 };
int i = 0;


const double pi = acos(-1);
double T_row = (double)16 * 2 * pi;
double T_col = (double)16 * 2 * pi;
std::vector<std::vector<cv::Point3f> > corners_world;
std::vector<std::vector<cv::Point2f> > corners_camera;
std::vector<std::vector<cv::Point2f> > corners_projecter(calibrate_number);

std::vector<cv::Point2f> cam_item_corners(item_number);//保存标记点图像坐标
std::vector< cv::Point3f > hole_Real_ALL;//数模待投标准点
std::vector< cv::Point3f > hole_Real_Part;//部分数模待投标准点(对应贴在物体上的标点)

std::vector< cv::Point2f > hole_Real_ALL_P_UV;//数模标准点，转换为投影仪图像点
std::vector< cv::Point2f > hole_Real_Part_P_UV;//数模标准点，转换为投影仪图像点
std::vector<cv::Point3f> world_corners_biaodingban;

std::vector<cv::Mat>result_row(calibrate_number);
std::vector<cv::Mat>result_col(calibrate_number);

cv::Mat cam_K_c;//相机内参
cv::Mat cam_kc;//相机畸变矩阵
cv::Mat cam_K_p;//投影仪内参
cv::Mat cam_kp; //投影仪畸变矩阵
cv::Mat Rc;//实际使用的旋转矩阵
cv::Mat Rp;//
cv::Mat Tc;//实际使用的平移矩阵
cv::Mat Tp;//
std::vector<cv::Mat> v_rvecs_c;//保存旋转矩阵,3X1
std::vector<cv::Mat> v_tvecs_c;//保存偏移矩阵,3X1
std::vector<cv::Mat> v_rvecs_p;//保存旋转矩阵,3X1
std::vector<cv::Mat> v_tvecs_p;//保存偏移矩阵,3X1

double cam_error_p;
double cam_error_c;
double cam_error_s;


cv::Mat v_rvecs_item_projector;//保存物体的世界坐标到投影仪的旋转矩阵,3X1
cv::Mat v_tvecs_item_projector;//保存物体的世界坐标到投影仪的偏移矩阵,3X1




using namespace std;
int main()
{
	//图片生成及
	//creat_gray_code();
	//creat_gray_code_4();//1200*1920
	//create_phase_Picture();

	//解相位过程
	//cv::Mat result_row = cv::Mat(height,width,CV_64FC1,cv::Scalar::all(0));
	//cv::Mat result_col = cv::Mat(height, width, CV_64FC1, cv::Scalar::all(0));
	//unwraping_idea_2(result_row, result_col, 0,0,path);

	
	calibrate();
	extract_items_corners();
	find_project_view();
	find_PNP();
	





	reconstruct(0,5 ,2);//第一个参数0代表需要标定再重建，1代表不需要标定，直接读取原有标定数据重建
	ofstream fout("D:/time.txt", ios::out | ios::app);
	if (!fout.is_open())
	{
		cout << "file can not open";
	
	}
	for (int i = 0; i < 200; i++)
	{
		fout << _time_[i]<<endl;
	}
	fout.close();

}
bool  comp1(const cv::Point2f &a, const cv::Point2f& b) {
	return a.y < b.y;
}
bool  comp2(const cv::Point2f& a, const cv::Point2f& b) {
	return a.x < b.x;
}

bool extract_items_corners()//得到物体标记点上的图像坐标
{
	int parm = 0;//0投标记点，1投标定板
	bool all_found = true;
	cam_item_corners.clear();
	cam_item_corners.resize(item_number);

	

	if (parm == 0)
	{
		cv::SimpleBlobDetector::Params params;
		params.minThreshold = 65;		//***二值化的起始阈值，即公式1的T1//数值越大偏黑的部分被忽略，数值越大稳定越差？也影响求解速度
		params.maxThreshold = 120;		//最大二值化值
		params.thresholdStep = 4;		//阈值越小检测越精确，计算速度变慢
		params.filterByInertia = false;	//斑点圆度的限制变量，默认是不限制
		//params.minInertiaRatio = 1.2;
		//params.maxInertiaRatio = 0.8;
		params.filterByColor = true;	//斑点颜色的限制变量
		params.blobColor = 255;			//255表示只提取白色斑点，0表示只提取黑色斑点
		params.filterByArea = true;		//斑点面积的限制变量
		params.minArea = 76;			//斑点的最小面积最大取到120
		params.maxArea = 130;		//斑点的最大面积
										//最小的斑点距离，不同二值图像的斑点间距离小于该值时，被认为是同一个位置的斑点，否则是不同位置上的斑点
		params.minDistBetweenBlobs = 6;//最大22左右,15比较合适

		params.filterByCircularity = true;    //斑点圆度的限制变量，默认是不限制
		params.minCircularity = 0.7f;    //斑点的最小圆度
		//斑点的最大圆度，所能表示的float类型的最大值
		params.maxCircularity = std::numeric_limits<float>::max();

		cv::Ptr<cv::FeatureDetector> blobDetector = cv::SimpleBlobDetector::create(params);
		bool patternfound = true;
		cv::Mat gray_image = cv::imread(path + "/100" + "/00.bmp", 0);
		imageFilter(gray_image);
		if (gray_image.rows < 1)
		{
			std::cout << "圆心提取：读取图片错误";
			return false;//图片有错
		}
		vector<cv::KeyPoint> key_points;
		blobDetector->detect(gray_image, key_points);

		cv::cvtColor(gray_image, gray_image, CV_GRAY2BGR);
		for (int i = 0; i < key_points.size(); i++)
		{
			//cv::circle(gray_image, cam_corners[i],25, cv::Scalar(0, 255, 0), 1, 8, 0);
			//cv::circle(gray_image, key_points[i].pt, key_points[i].size, cv::Scalar(0, 255, 0), -1, 2, 0);
			cv::circle(gray_image, key_points[i].pt, key_points[i].size, cv::Scalar(0, 255, 0), 2);
			cam_item_corners[i] = key_points[i].pt;
		}
		cv::imwrite(path + "/blob" + ".bmp", gray_image);

		//图像点按3x3排序
		sort(cam_item_corners.begin(), cam_item_corners.end(), comp1);
		sort(cam_item_corners.begin(), cam_item_corners.begin() + 3, comp2);
		sort(cam_item_corners.begin() + 3, cam_item_corners.begin() + 6, comp2);
		sort(cam_item_corners.begin() + 6, cam_item_corners.begin() + 9, comp2);
	}
	else
	{
		cv::SimpleBlobDetector::Params params1;
		params1.maxArea = 6000;		//斑点的最大面积
		params1.minThreshold = 45;		//***二值化的起始阈值，即公式1的T1//数值越大偏黑的部分被忽略，数值越大稳定越差？也影响求解速度
		params1.maxThreshold = 190;		//最大二值化值
		params1.thresholdStep = 5;		//阈值越小检测越精确，计算速度变慢
		params1.filterByInertia = true;	//斑点圆度的限制变量，默认是不限制
		params1.filterByColor = true;	//斑点颜色的限制变量
		params1.blobColor = 255;			//255表示只提取白色斑点，0表示只提取黑色斑点
		params1.filterByArea = true;		//斑点面积的限制变量
		params1.minArea = 1100;			//斑点的最小面积最大取到120
									//最小的斑点距离，不同二值图像的斑点间距离小于该值时，被认为是同一个位置的斑点，否则是不同位置上的斑点
		params1.minDistBetweenBlobs = 50;//最大22左右,15比较合适
		cv::Ptr<cv::FeatureDetector> blobDetector1 = cv::SimpleBlobDetector::create(params1);
		bool patternfound = true;

		cv::Mat gray_image = cv::imread(path + "/100" + "/00.bmp", 0);// 读白色  第 i 个 第一张
		//cv::Mat gray_image = cv::imread("C:/Users/1/MVS/Data/24.bmp", 0);// 读白色  第 i 个 第一张
		imageFilter(gray_image);
		if (gray_image.rows < 1)
		{
			return false;//图片有错
		}
		std::vector<cv::Point2f> cam_corners;

		if (!cv::findCirclesGrid(gray_image, corner_count, cam_corners, cv::CALIB_CB_SYMMETRIC_GRID, blobDetector1))
		{
			std::cout <<"圆心寻找失败" << endl;
			return false;
		}
		//划圆心位置，并保存
		for (int i = 0; i < cam_corners.size(); i++)
		{
			//cv::circle(gray_image, cam_corners[i],25, cv::Scalar(0, 255, 0), 1, 8, 0);
			cv::circle(gray_image, cam_corners[i], 1, cv::Scalar(0, 255, 0), -1, 2, 0);
			cv::circle(gray_image, cam_corners[i], 20, cv::Scalar(0, 0, 255), 1, 2, 0);
			cam_item_corners[i] = cam_corners[i];
		}
		//cv::imwrite(path + std::to_string(i + 1) + "/circle99" + ".bmp", gray_image);
		get_chessboard_world_coords(world_corners_biaodingban, corner_count, corner_size);
	}


	return all_found;
}


void find_project_view()
{
	cv::Mat row_Phase = cv::Mat(height, width, CV_64FC1, cv::Scalar::all(0));
	cv::Mat col_Phase = cv::Mat(height, width, CV_64FC1, cv::Scalar::all(0));
	unwraping_idea_2(row_Phase, col_Phase, 0, 0, path + std::to_string(100) + "/");

	std::vector<cv::Point2f>& c_number = cam_item_corners;//相机图像像素坐标
	for (int j = 0; j < c_number.size(); j++)
	{


		//直接使用
		//double row = result_row[i].at<double>(c_number[j].y, c_number[j].x);
		//double col = result_col[i].at<double>(c_number[j].y, c_number[j].x);
		//hole_Real_Part_P_UV.push_back(cv::Point2d(col / T_col * 1920, row / T_row *1200));
		
		//通过点周围平面，寻找单应性矩阵，再将点映射到uv坐标。
		std::vector<cv::Point2d> corners_pre_projecter;
		std::vector<cv::Point2d> corners_post_projecter;
		
		
		for (double m = c_number[j].y - 3; m < c_number[j].y + 3; m++)
		{
			for (double n = c_number[j].x - 3; n < c_number[j].x + 3; n++)
			{
				double row = row_Phase.at<double>(m, n);
				double col = col_Phase.at<double>(m, n);
				corners_pre_projecter.push_back(cv::Point2d(n, m));
				//if (m == c_number[j].y && n ==c_number[j].x)
				//	p_pre= cv::Point2d(col / T_col * 960, row / T_row * 960);
				corners_post_projecter.push_back(cv::Point2d(col / T_col * 1920, row / T_row * 1200));
			}
		}
		cv::Mat H = cv::findHomography(corners_pre_projecter, corners_post_projecter, cv::RANSAC);
		cv::Mat p = cv::Mat(3, 1, CV_64FC1, cv::Scalar::all(0.0));
		p.at<double>(0, 0) = c_number[j].x;
		p.at<double>(1, 0) = c_number[j].y;
		p.at<double>(2, 0) = 1.0;
		cv::Mat pp = H * p;
		
		
		cv::Point3d ppp = cv::Point3d(pp);
		cv::Point2d pppp(ppp.x / ppp.z, ppp.y / ppp.z);//计算出投影仪对应的像素坐标
		hole_Real_Part_P_UV.push_back(pppp);
	}
	cv::Mat final_project = cv::Mat(1200, 1920, CV_64FC3, cv::Scalar::all(255));
	for (int i = 0; i < hole_Real_Part_P_UV.size(); i++)
	{
		cv::circle(final_project, hole_Real_Part_P_UV[i], 3, cv::Scalar(255, 0, 0), -1, 2, 0);
	}
	cv::imwrite(path + "/circle4" + ".bmp", final_project);
}


bool find_PNP()//寻找物体到投影仪的外参
{
	//FILE* fp = fopen("C:/Users/1/Desktop/data.txt", "r"); //以data.txt文件为例
	ifstream fp("C:/Users/1/Desktop/data.txt");
	//2:检测文件是否打开成功；
	if (!fp) {
		printf("打开失败！\n");
		cout<<"读取data文件错误"; //返回异常
		return false;
	}
	

	while (!fp.eof())
	{
		cv::Point3f temp;
		fp >> temp.x;
		fp >> temp.y;
		fp >> temp.z;
		hole_Real_ALL.push_back(temp);
	}
	hole_Real_ALL.pop_back();

	ifstream fp2("C:/Users/1/Desktop/data2.txt");
	//2:检测文件是否打开成功；
	if (!fp2) {
		printf("打开失败！\n");
		cout << "读取data2文件错误"; //返回异常
		return false;
	}
	

	while (!fp2.eof())
	{
		cv::Point3f temp2;
		fp2 >> temp2.x;
		fp2 >> temp2.y;
		fp2 >> temp2.z;
		hole_Real_Part.push_back(temp2);
	}
	hole_Real_Part.pop_back();//不知道为啥多读取了一个。退出。




	//寻找外参
	//图像坐标：cam_item_corners  。内参：cam_K_c。世界坐标：hole_Real_Part。相机畸变：cam_kc。FindExtrinsicCameraParams2
	//cv::solvePnP(hole_Real_Part, hole_Real_Part_P_UV, cam_K_p, cam_kp, v_rvecs_item_projector, v_tvecs_item_projector);//物体点到贴的标点。
	cv::solvePnP(world_corners_biaodingban, hole_Real_Part_P_UV, cam_K_p, cam_kp, v_rvecs_item_projector, v_tvecs_item_projector);//标定板点。


	vector<cv::Point2f> output;
	//cv::projectPoints(hole_Real_Part, v_rvecs_item_projector, v_tvecs_item_projector, cam_K_p, cam_kp, output);//物体真实点
	cv::projectPoints(world_corners_biaodingban, v_rvecs_item_projector, v_tvecs_item_projector, cam_K_p, cam_kp, output);//标定板点


	cv::Mat final_project = cv::Mat(1200, 1920, CV_64FC3, cv::Scalar::all(255));
	for (int i = 0; i < output.size(); i++)
	{
		cv::circle(final_project, output[i], 3, cv::Scalar(255, 0, 0), -1, 2, 0);
	}
	cv::imwrite(path + std::to_string(i + 1) + "/circle1" + ".bmp", final_project);

	vector<cv::Point2f> output2;
	cv::projectPoints(hole_Real_ALL, v_rvecs_item_projector, v_tvecs_item_projector, cam_K_p, cam_kp, output2);
	cv::Mat final_project2 = cv::Mat(1200, 1920, CV_64FC3, cv::Scalar::all(255));
	for (int i = 0; i < output2.size(); i++)
	{
		cv::circle(final_project2, output2[i], 3, cv::Scalar(255, 0, 0), -1, 2, 0);
	}
	cv::imwrite(path + std::to_string(i + 1) + "/circle2" + ".bmp", final_project2);



	return true;
}







void get_chessboard_world_coords(std::vector<cv::Point3f>&world_corners, cv::Size corner_count, cv::Size corner_size)
{
	//generate world object coordinates
	for (int h = 0; h < corner_count.height; h++)
	{
		for (int w = 0; w < corner_count.width; w++)
		{
			world_corners.push_back(cv::Point3f(corner_size.width * w, corner_size.height * h, 0.f));
		}
	}
}

bool extract_chessboard_corners(int count = calibrate_number)//得到图像上和棋盘格上的角点坐标
{
	//有三组角点
	//int count = 3;

	corners_world.clear();
	corners_camera.clear();
	corners_world.resize(count);
	corners_camera.resize(count);

	//cv::Size imageSize(0, 0);
	//int image_scale = 1;
	bool all_found = true;


	cv::SimpleBlobDetector::Params params;
	params.maxArea = 6000;		//斑点的最大面积
	params.minThreshold = 45;		//***二值化的起始阈值，即公式1的T1//数值越大偏黑的部分被忽略，数值越大稳定越差？也影响求解速度
	params.maxThreshold = 190;		//最大二值化值
	params.thresholdStep = 5;		//阈值越小检测越精确，计算速度变慢
	params.filterByInertia = true;	//斑点圆度的限制变量，默认是不限制
	//params.minInertiaRatio = 1.2;
	//params.maxInertiaRatio = 0.8;
	params.filterByColor = true;	//斑点颜色的限制变量
	params.blobColor = 255;			//255表示只提取白色斑点，0表示只提取黑色斑点
	params.filterByArea = true;		//斑点面积的限制变量
	params.minArea =1100;			//斑点的最小面积最大取到120
									//最小的斑点距离，不同二值图像的斑点间距离小于该值时，被认为是同一个位置的斑点，否则是不同位置上的斑点
	params.minDistBetweenBlobs = 50;//最大22左右,15比较合适
	cv::Ptr<cv::FeatureDetector> blobDetector = cv::SimpleBlobDetector::create(params);
	bool patternfound = true;


	for (int i = 0; i < count; i++)
	//for (int i = 0; i < 15; i++)
	{

		cv::Mat gray_image = cv::imread(path + std::to_string(i + 1) + "/00.bmp", 0);// 读白色  第 i 个 第一张
		//cv::Mat gray_image = cv::imread("C:/Users/1/MVS/Data/24.bmp", 0);// 读白色  第 i 个 第一张
		imageFilter(gray_image);
		if (gray_image.rows < 1)
		{
			return false;//图片有错
		}
		std::vector<cv::Point2f> &cam_corners = corners_camera[i];
		std::vector<cv::Point3f> &world_corners = corners_world[i];
		//vector<cv::KeyPoint> key_points;
		//blobDetector->detect(gray_image, key_points);

		if (!cv::findCirclesGrid(gray_image, corner_count, cam_corners, cv::CALIB_CB_SYMMETRIC_GRID, blobDetector))
		{
			std::cout <<"第" << i+1 <<"组图片" << "圆心寻找失败"<<endl;
			return false;
		}


		//划圆心位置，并保存
		for (int i = 0; i < cam_corners.size(); i++)
		{
			//cv::circle(gray_image, cam_corners[i],25, cv::Scalar(0, 255, 0), 1, 8, 0);
			cv::circle(gray_image, cam_corners[i], 1, cv::Scalar(0, 255, 0), -1, 2, 0);
			cv::circle(gray_image, cam_corners[i], 20, cv::Scalar(0, 0, 255), 1, 2, 0);
		}
		cv::imwrite(path+  std::to_string(i + 1) + "/circle" + ".bmp", gray_image);

		//std::vector<double> dis(99);
		//for (int i = 0; i < cam_corners.size()-1; i++)
		//{
		//	if(abs(cam_corners[i + 1].x - cam_corners[i].x) < 300 && abs(cam_corners[i + 1].y - cam_corners[i].y) < 300)
		//		dis[i] = sqrt(pow(cam_corners[i + 1].x - cam_corners[i].x , 2) + pow(cam_corners[i + 1].y - cam_corners[i].y , 2));
		//}


		get_chessboard_world_coords(world_corners, corner_count, corner_size);
	}
	return all_found;
}
void camera_calibrate(std::vector<cv::Mat> &v_rvecs, std::vector<cv::Mat> &v_tvecs)
{

	int count = calibrate_number;
	cv::Size imageSize(0, 0);
	imageSize = cv::Size(width, height); //换成读入图片的尺寸


	if (!extract_chessboard_corners(count))
	{
		return;
	}


	std::vector<std::vector<cv::Point3f> > world_corners_active;
	std::vector<std::vector<cv::Point2f> > camera_corners_active;

	world_corners_active.reserve(count);
	camera_corners_active.reserve(count);

	for (int i = 0; i < count; i++)
	{
		std::vector<cv::Point3f> const& world_corners = corners_world.at(i);
		std::vector<cv::Point2f> const& cam_corners = corners_camera.at(i);

		if (world_corners.size() && cam_corners.size())
		{   //active set
			world_corners_active.push_back(world_corners);
			camera_corners_active.push_back(cam_corners);
		}
	}
	if (world_corners_active.size() < 3)
	{
		std::cout << "ERROR: use at least 3 sets" << std::endl;
		return;
	}
	int cal_flags = 0 + cv::CALIB_FIX_K3;
	//calibrate the camera ////////////////////////////////////
	//std::vector<cv::Mat> cam_rvecs, cam_tvecs;
	int cam_flags = cal_flags;


	cam_error_c = cv::calibrateCamera(world_corners_active, camera_corners_active, imageSize, cam_K_c, cam_kc, v_rvecs, v_tvecs, cam_flags,
		cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 50, DBL_EPSILON));
	//cam_error_c = cv::calibrateCamera(world_corners_active, camera_corners_active, imageSize, cam_K_c, cam_kc, v_rvecs, v_tvecs, CV_CALIB_USE_INTRINSIC_GUESS);
	
	std::cout << "cam_error_c: " << cam_error_c << std::endl
		<< "cam_K_c : " << cam_K_c << std::endl
		<< "cam_kc :" << cam_kc << std::endl;

}

void projecter_calibrate()
{


	for (int i = 1; i <= calibrate_number; i++)
	{
		unwraping_idea_2(result_row[(double)i - 1], result_col[(double)i - 1],0, 0, path + std::to_string(i) + "/");
	}

	cv::Point2d p_pre;
	
	cv::Mat projceter = cv::Mat(height, width, CV_32FC1, cv::Scalar::all(0));
	for (int i = 0; i < corners_camera.size(); i++)
	{
		std::vector<cv::Point2f>& c_number = corners_camera[i];
		std::vector<cv::Point2f>& p_number = corners_projecter[i];
		for (int j = 0; j < c_number.size(); j++)
		{


			//直接使用
			//double row = result_row[i].at<double>(c_number[j].y, c_number[j].x);
			//double col = result_col[i].at<double>(c_number[j].y, c_number[j].x);
			//p_number.push_back(cv::Point2d(col / T_col * 1920, row / T_row *1200));



			//通过点周围平面，寻找单应性矩阵，再将点映射到uv坐标。
			std::vector<cv::Point2d> corners_pre_projecter;
			std::vector<cv::Point2d> corners_post_projecter;
			
			
			for(double m=c_number[j].y-4; m< c_number[j].y +4;  m++)
			{
				for (double n = c_number[j].x - 4; n < c_number[j].x + 4; n++)
				{
					double row = result_row[i].at<double>(m, n);
					double col = result_col[i].at<double>(m, n);
					corners_pre_projecter.push_back(cv::Point2d(n, m));
					//if (m == c_number[j].y && n ==c_number[j].x)
					//	p_pre= cv::Point2d(col / T_col * 960, row / T_row * 960);
					corners_post_projecter.push_back(cv::Point2d(col / T_col * 1920, row / T_row *1200));
				}
			}
			cv::Mat H = cv::findHomography(corners_pre_projecter, corners_post_projecter, cv::RANSAC);
			//cout << H << endl;
			cv::Mat p = cv::Mat(3, 1, CV_64FC1, cv::Scalar::all(0.0));
			p.at<double>(0, 0) = c_number[j].x;
			p.at<double>(1, 0) = c_number[j].y;
			p.at<double>(2, 0) = 1.0;
			cv::Mat pp = H * p;
			
			
			cv::Point3d ppp = cv::Point3d(pp);
			cv::Point2d pppp(ppp.x / ppp.z, ppp.y / ppp.z);//计算出投影仪对应的角点坐标
			p_number.push_back(pppp);
		}
	}

	int cal_flags = 0 + cv::CALIB_FIX_K3;
	int cam_flags = cal_flags;
	cv::Size imageSize(0, 0);
	imageSize = cv::Size(width_projecter, height_projecter);
	cam_error_p = cv::calibrateCamera(corners_world, corners_projecter, imageSize, cam_K_p, cam_kp, v_rvecs_p, v_tvecs_p, cam_flags,
		cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 50, DBL_EPSILON));
	
	//cam_error_p = cv::calibrateCamera(corners_world, corners_projecter, imageSize, cam_K_p, cam_kp, v_rvecs_p, v_tvecs_p, CV_CALIB_USE_INTRINSIC_GUESS);
	std::cout << "cam_error_p: " << cam_error_p << std::endl
		<< "cam_K_p : " << cam_K_p << std::endl
		<< "cam_kp :" << cam_kp << std::endl;


}

cv::Point2d undistortPoints(cv::Point2d point)
{
	cv::Mat inp1(1, 1, CV_64FC2);
	inp1.at<cv::Vec2d>(0, 0) = cv::Vec2d(point.x, point.y);
	cv::Mat outp1;
	//cv::undistortPoints(inp1, outp1, cam_K, cam_kc);//这样调用会有错误 
	cv::undistortPoints(inp1, outp1, cam_K_c, cam_kc, cv::noArray(), cam_K_c);
	cv::Vec2d& outvec1 = outp1.at<cv::Vec2d>(0, 0);
	return cv::Point2d(outvec1);
}
void calibrate()
{
	camera_calibrate(v_rvecs_c, v_tvecs_c);
	projecter_calibrate();
	//保存
	//std::string path = "E:/2021.4.9/100/";
	std::string filename = path+"100/" + "calibration.yml";
	Rc = convert(v_rvecs_c[0]);
	Rp = convert(v_rvecs_p[0]);
	Tc = v_tvecs_c[0];
	Tp = v_tvecs_p[0];
	try
	{
		cv::FileStorage fs(filename, cv::FileStorage::WRITE);
		if (!fs.isOpened())
		{
			return;
		}
		fs << "cam_error_c" << cam_error_c
			<< "cam_error_p" << cam_error_p
			<< "cam_K_c" << cam_K_c
			<< "cam_Kc" << cam_kc
			<< "cam_K_p" << cam_K_p
			<< "cam_Kp" << cam_kp
			<< "Rc" << Rc
			<< "Rp" << Rp
			<< "Tc" << v_tvecs_c[0]
			<< "Tp" << v_tvecs_p[0];
		fs.release();
	}
	catch (std::exception e)
	{
		std::cout << "读取失败" << std::endl;
		return;
	}

}
void reconstruct(int n,int number_i,int k_gray)
{

	//check(result_col[0], result_row[0]);
	cv::Point3d pw;
	cv::Point2d pc;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	cv::Mat phase;//列相位，会使用
	cv::Mat phase_null;//行相位，但不会用
	std::cout << "unwraping..." << endl;
	
	clock_t start, finish;
	start = clock();
	unwraping_idea_2(phase_null, phase, number_i, k_gray, path + "100/");//自己的方法
	finish = clock();
	cout << finish - start;
	_time_[i++] = finish - start;



	std::string path1 = path + "100/";
	std::string filename = path1 + "calibration.yml";
	if (n == 1)
	{
		try
		{
			cv::FileStorage fs(filename, cv::FileStorage::READ);
			if (!fs.isOpened())
			{
				return;
			}
			fs["cam_K_c"] >> cam_K_c;
			fs["cam_Kc"] >> cam_kc;
			fs["cam_K_p"] >> cam_K_p;
			fs["cam_Kp"] >> cam_kp;
			fs["Rc"] >> Rc;
			fs["Rp"] >> Rp;
			fs["Tc"] >> Tc;
			fs["Tp"] >> Tp;
			fs.release();
		}
		catch (std::exception e)
		{
			std::cout << "读取失败" << std::endl;
			return;
		}

	}
	//求M矩阵
	cv::Mat vc = cv::Mat(3, 4, CV_64FC1);
	cv::Mat vp = cv::Mat(3, 4, CV_64FC1);
	cv::Mat G = cv::Mat(3, 3, CV_64FC1);
	cv::Mat _G = cv::Mat(3, 3, CV_64FC1);
	cv::Mat H = cv::Mat(3, 1, CV_64FC1);
	Rc.copyTo(vc.colRange(0,3));
	Tc.copyTo(vc.col(3));
	Rp.copyTo(vp.colRange(0, 3));
	Tp.copyTo(vp.col(3));
	cv::Mat Mc = cam_K_c * vc;
	cv::Mat Mp = cam_K_p * vp;

	for (int i = 0; i < phase.cols; i+=1)
	{
		for (int j = 0;j < phase.rows;j+=1)
		{
			double value = phase.at<double>(j, i);
			if (value == 0)//滤掉相位之外的点
			{
				pcl::PointXYZ point = pcl::PointXYZ(0, 0, 75);
				cloud->push_back(point);
				continue;
			}
			double col = phase.at<double>(j, i);
			double xp = col / T_col * 1920;
			pc = cv::Point2d(i, j);

			pc=undistortPoints(cv::Point2d(i, j));
			G.at<double>(0,0) = Mc.at<double>(0, 0) - Mc.at<double>(2, 0) * pc.x;
			G.at<double>(0,1) = Mc.at<double>(0, 1) - Mc.at<double>(2, 1) * pc.x;
			G.at<double>(0,2) = Mc.at<double>(0, 2) - Mc.at<double>(2, 2) * pc.x;
			G.at<double>(1,0) = Mc.at<double>(1, 0) - Mc.at<double>(2, 0) * pc.y;
			G.at<double>(1,1) = Mc.at<double>(1, 1) - Mc.at<double>(2, 1) * pc.y;
			G.at<double>(1,2) = Mc.at<double>(1, 2) - Mc.at<double>(2, 2) * pc.y;
			G.at<double>(2, 0) = Mp.at<double>(0, 0) - Mp.at<double>(2, 0) *xp;
			G.at<double>(2, 1) = Mp.at<double>(0, 1) - Mp.at<double>(2, 1) *xp;
			G.at<double>(2, 2) = Mp.at<double>(0, 2) - Mp.at<double>(2, 2) *xp;

			H.at<double>(0,0) = Mc.at<double>(2, 3)*pc.x - Mc.at<double>(0, 3) ;
			H.at<double>(1,0) = Mc.at<double>(2, 3)*pc.y - Mc.at<double>(1, 3) ;
			H.at<double>(2,0) = Mp.at<double>(2, 3)*  xp - Mp.at<double>(0, 3) ;
			_G = G.inv();
			cv::Mat pre_pw = _G * H;
			pw.x = pre_pw.at<double>(0,0);
			pw.y = pre_pw.at<double>(1,0);
			pw.z = pre_pw.at<double>(2,0);
			Dimension.at<cv::Vec3d>(j, i) = cv::Vec3d(pw.x, pw.y, pw.z);
			pcl::PointXYZ point = pcl::PointXYZ(pw.x, pw.y, pw.z);
			//if (point.z > -50 && point.z < 20);
			cloud->push_back(point);
		}
		std::cout << i<<endl;
	}

	//std::cout << "点云大小 : " << cloud->size() << std::endl;
	//pcl::io::savePCDFile(path + "100/" + "test.pcd", *cloud);
	cv::imwrite(path + "Dimension" + ".bmp", Dimension);
	cv::imwrite(path +"100/"+ "93" + ".bmp", K2);
	cv::imwrite(path +"100/"+"94" + ".bmp", K3);

	std::cout << "点云大小 : " << cloud->size() << std::endl;
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered1(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	//sor.setInputCloud(cloud);
	//sor.setMeanK(10);
	//sor.setStddevMulThresh(1.0);    // 设置阈值，判断是否为离群点
	//sor.filter(*cloud_filtered1);
	//
	//std::cout << "一次滤波点云大小 : " << cloud_filtered1->size() << std::endl;
	//
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
	//outrem.setInputCloud(cloud_filtered1);
	//outrem.setRadiusSearch(1.5);          // 设置搜索半径
	//outrem.setMinNeighborsInRadius(2);   // 设置最少的邻点数量
	//outrem.filter(*cloud_filtered2);
	//
	//std::cout << "第二次滤波点云大小 : " << cloud_filtered2->size() << std::endl;
	//cloud = cloud_filtered2;
	pcl::io::savePCDFile(path + std::to_string(number_i)+".pcd", *cloud);
}
cv::Mat convert(cv::Mat v_rvecs_c)
{
	cv::Mat R = cv::Mat(3, 3, CV_64FC1);
	cv::Mat I = cv::Mat::eye(cv::Size(3, 3), CV_64FC1);
	cv::Mat temp = cv::Mat::zeros(3, 3, CV_64FC1);
	double r1 = v_rvecs_c.at<double>(0, 0);
	double r2 = v_rvecs_c.at<double>(1, 0);
	double r3 = v_rvecs_c.at<double>(2, 0);
	double norm = sqrt(r1 * r1 + r2 * r2 + r3 * r3);
	v_rvecs_c = v_rvecs_c / norm;
	temp.at<double>(0, 1) = -v_rvecs_c.at<double>(2, 0);
	temp.at<double>(0, 2) = v_rvecs_c.at<double>(1, 0);
	temp.at<double>(1, 0) = v_rvecs_c.at<double>(2, 0);
	temp.at<double>(1, 2) = -v_rvecs_c.at<double>(0, 0);
	temp.at<double>(2, 0) = -v_rvecs_c.at<double>(1, 0);
	temp.at<double>(2, 1) = v_rvecs_c.at<double>(0, 0);

	return R = cos(norm) * I + (1 - cos(norm)) * v_rvecs_c * v_rvecs_c.t() + sin(norm) * temp;
	//v_rvecs_c.push_back(R);
}

void unwraping_idea_2(cv::Mat& result_row, cv::Mat& result_col,int number_i, int flag, string p = path)
{
	
	if (flag == 0)
	{
		result_col = phaseshift_col(0,0, p);
		cv::Mat rank_col_4 = cv::Mat(height, width, CV_8UC1, cv::Scalar::all(0));
		cv::Mat	rank_col_5 = cv::Mat(height, width, CV_8UC1, cv::Scalar::all(0));
		cv::Mat	rank_col_6 = cv::Mat(height, width, CV_8UC1, cv::Scalar::all(0));
		decode_col_5_idea_2(rank_col_4, rank_col_5, rank_col_6, p,0,0);
		cv::Mat rank_row_4 = cv::Mat(height, width, CV_8UC1, cv::Scalar::all(0));
		cv::Mat	rank_row_5 = cv::Mat(height, width, CV_8UC1, cv::Scalar::all(0));
		cv::Mat	rank_row_6 = cv::Mat(height, width, CV_8UC1, cv::Scalar::all(0));
		result_row = phaseshift_row(p);
		decode_row_5_idea_2(rank_row_4, rank_row_5, rank_row_6, p);
		//按列
		for (int i = 1 + y + y; i < height - y - y; ++i)
		{
			for (int j = 1 + y + y; j < width - y - y; ++j)
			{
				if (result_col.at<double>(i, j) != 0
					|| (result_col.at<double>(i, j) == 0 && result_col.at<double>(i, j + 1) != 0 && result_col.at<double>(i, j - 1) != 0)
					)
				{
					result_col.at<double>(i, j) = result_col.at<double>(i, j) + 2 * pi * rank_col_4.at<uchar>(i, j);

					for (int vv = 0; vv < 15; vv++)
					{
						if (rank_col_4.at<uchar>(i, j) == vv
							&& rank_col_4.at<uchar>(i, j + 1) == vv + 1
							&& (result_col.at<double>(i, j) - 2 * pi * rank_col_4.at<uchar>(i, j) < (pi / 4)
								|| result_col.at<double>(i, j) - 2 * pi * rank_col_4.at<uchar>(i, j) > (7 * pi / 4))
							)
						{
							if (vv % 2 == 0)
							{
								correct_1(i, j, vv, vv + 1, result_col, rank_col_4, rank_col_5);
								break;
							}
							else
							{
								correct_2(i, j, vv, vv + 1, result_col, rank_col_4, rank_col_5, rank_col_6);
								break;
							}
						}
					}
				}
			}
		}
		//行
		for (int j = y + y; j < width- y - y; ++j)
		{
			for (int i = y + y; i < height - y - y; ++i)
			{
				if ( (i < height-1 && j < width-1 ) && (result_row.at<double>(i, j) != 0 || (result_row.at<double>(i, j) == 0 ) && result_row.at<double>(i+1, j) != 0 && result_row.at<double>(i-1,j) != 0))
				{

					result_row.at<double>(i, j) = result_row.at<double>(i, j) + 2 * pi * rank_row_4.at<uchar>(i, j);
					for (int vv = 0; vv < 15; vv++)
					{
						if (rank_row_4.at<uchar>(i, j) == vv 
							&& rank_row_4.at<uchar>(i+1, j) == vv + 1
							&& (result_row.at<double>(i, j) - 2 * pi * rank_row_4.at<uchar>(i, j) < (pi / 3)
								||result_row.at<double>(i, j) - 2 * pi * rank_row_4.at<uchar>(i, j) > (5 * pi / 3))
							)
						{
							if (vv % 2 == 0)
								correct_3(i, j, vv, vv + 1, result_row, rank_row_4, rank_row_5);
							else
								correct_4(i, j, vv, vv + 1, result_row, rank_row_4, rank_row_5, rank_row_6);
						}
					}

				}
			}
		}
		cv::Mat phi1_Normal_111;
		cv::Mat phi1_Normal_222;
		cv::normalize(result_col, phi1_Normal_111, 0., 255., cv::NORM_MINMAX);
		cv::normalize(result_row, phi1_Normal_222, 0., 255., cv::NORM_MINMAX);
		cv::imwrite(p + "unwraping_result_col" + ".bmp", phi1_Normal_111);
		cv::imwrite(p + "unwraping_result_row" + ".bmp", phi1_Normal_222);
		std::cout << "finished_calibrate" << endl;
	}
	else
	{
		result_col = phaseshift_col(number_i,flag, p);
		cv::Mat rank_col_4 = cv::Mat(height, width, CV_8UC1, cv::Scalar::all(0));
		cv::Mat	rank_col_5 = cv::Mat(height, width, CV_8UC1, cv::Scalar::all(0));
		cv::Mat	rank_col_6 = cv::Mat(height, width, CV_8UC1, cv::Scalar::all(0));
		decode_col_5_idea_2(rank_col_4, rank_col_5,rank_col_6, p, number_i,flag);

		for (int i = 0; i < height-1; ++i)
		{
			for (int j = 1+y; j < width - y; ++j)
			{
				if (result_col.at<double>(i, j) != 0 
					||(result_col.at<double>(i, j) == 0 && result_col.at<double>(i, j+1) != 0&& result_col.at<double>(i, j-1) != 0)
					//|| rank_col_4.at<uchar>(i, j+1)- rank_col_4.at<uchar>(i, j)==1
					)
				{
					if (result_col.at<double>(i, j) != 0
						|| (result_col.at<double>(i, j) == 0 && result_col.at<double>(i, j + 1) != 0 && result_col.at<double>(i, j - 1) != 0))
					

					result_col.at<double>(i, j) = result_col.at<double>(i, j) + 2 * pi * rank_col_4.at<uchar>(i, j);
					for (int vv = 0; vv < 15; vv++)
					{
						if (rank_col_4.at<uchar>(i, j) == vv 
							&& rank_col_4.at<uchar>(i, j + 1) == vv+1
							&& (result_col.at<double>(i, j)- 2 * pi * rank_col_4.at<uchar>(i, j) < (pi/4) 
								|| result_col.at<double>(i, j)- 2 * pi * rank_col_4.at<uchar>(i, j) > (7 * pi / 4))
							)
						{
							if (vv % 2 == 0)
							{
								correct_1(i, j, vv, vv+1, result_col, rank_col_4, rank_col_5);
								break;
							}
							else
							{
								correct_2(i, j, vv, vv + 1, result_col, rank_col_4, rank_col_5, rank_col_6);
								break;
							}
						}
					}
				}

			}
		}
		cv::imwrite(p + "rank_4" + ".bmp", rank_col_4);
		cv::imwrite(p + "rank_5" + ".bmp", rank_col_5);
		cv::imwrite(p + "rank_6" + ".bmp", rank_col_6);
		cv::Mat phi1_Normal_111;
		cv::normalize(result_col, phi1_Normal_111, 0., 255., cv::NORM_MINMAX);
		cv::imwrite(p + "unwrapping_col" + ".bmp", phi1_Normal_111);
		std::cout << "finished_reconstruct" << endl;

	}
}

void correct_1(int &i, int &j, int pre, int post, cv::Mat& result_col, cv::Mat& rank_col_4, cv::Mat& rank_col_5)
{

	for (int m = j; m > j - y; m--)
	{

		if (rank_col_4.at<uchar>(i, m) != pre)
		{
			break;
		}
		if ((result_col.at<double>(i, m) - 2 * pi * rank_col_4.at<uchar>(i, m)) == 0|| result_col.at<double>(i, m)==0)
			continue;

		if (result_col.at<double>(i, m)-2 * pi * rank_col_4.at<uchar>(i, m) <= (pi/2) )
		{
			result_col.at<double>(i, m) = result_col.at<double>(i, m)- 2 * pi * rank_col_4.at<uchar>(i, m) + 2 * pi * rank_col_5.at<uchar>(i, m) + 2 * pi;
			if(sk==1)
				K2.at<double>(i, m) = 1;
		}
		else if (result_col.at<double>(i, m)-2 * pi * rank_col_4.at<uchar>(i, m) > (pi / 2) && result_col.at<double>(i, m)- 2 * pi * rank_col_4.at<uchar>(i, m) < (3 * pi / 2))
		{
			result_col.at<double>(i, m) = result_col.at<double>(i, m)- 2 * pi * rank_col_4.at<uchar>(i, m) + 2 * pi * rank_col_4.at<uchar>(i, m);
			if (sk == 1)
				K2.at<double>(i, m) = 1;
		}
		else if(result_col.at<double>(i, m) - 2 * pi * rank_col_4.at<uchar>(i, m) >= (3 * pi / 2))
		{
			result_col.at<double>(i, m) = result_col.at<double>(i, m)- 2 * pi * rank_col_4.at<uchar>(i, m) + 2 * pi * rank_col_5.at<uchar>(i, m);
			if (sk == 1)
				K2.at<double>(i, m) = 1;
		}
	}
	int k = j;
	for (int m = j + 1; m < k + y; m++)
	{
		//if (i == 600 && m >= 636)
		//	cout << endl;
		if (rank_col_4.at<uchar>(i, m) != post)
		{
			j = m-1;
			break;
		}
		if (result_col.at<double>(i, m) == 0)
			continue;
		if (result_col.at<double>(i, m) <= (pi/2))
		{
			result_col.at<double>(i, m) = result_col.at<double>(i, m) + 2 * pi * rank_col_5.at<uchar>(i, m) + 2 * pi;
			if (sk == 1)
				K2.at<double>(i, m) = 1;
		}
		else if (result_col.at<double>(i, m) > (pi / 2) && result_col.at<double>(i, m) < (3 * pi / 2))
		{
			result_col.at<double>(i, m) = result_col.at<double>(i, m) + 2 * pi * rank_col_4.at<uchar>(i, m);
			if (sk == 1)
				K2.at<double>(i, m) = 1;
		}
		else if(result_col.at<double>(i, m) >= (3 * pi / 2))
		{
			result_col.at<double>(i, m) = result_col.at<double>(i, m) + 2 * pi * rank_col_5.at<uchar>(i, m);
			if (sk == 1)
				K2.at<double>(i, m) = 1;
		}
		j = m ;
	}
}

void correct_2(int &i, int &j, int pre, int post, cv::Mat& result_col, cv::Mat& rank_col_4, cv::Mat& rank_col_5, cv::Mat& rank_col_6)
{
	
	for (int m = j; m > j - y; m--)
	{

		//if (i == 600 && m >= 636)
		//	cout << endl;
		//
		if (rank_col_4.at<uchar>(i, m) != pre)
		{
			break;
		}
		if ((result_col.at<double>(i, m) - 2 * pi * rank_col_4.at<uchar>(i, m)) == 0|| result_col.at<double>(i, m)==0)
			continue;

		if ((result_col.at<double>(i, m) - 2 * pi * rank_col_4.at<uchar>(i, m)) <= (pi/2) )
		{
			result_col.at<double>(i, m) = result_col.at<double>(i, m)- 2 * pi * rank_col_4.at<uchar>(i, m) + 2 * pi * rank_col_5.at<uchar>(i, m) + 4 * pi;
			if (sk == 1)
				K3.at<double>(i, m) = 1;
			continue;

		}
		else if (result_col.at<double>(i, m) - 2 * pi * rank_col_4.at<uchar>(i, m) > (pi / 2) && result_col.at<double>(i, m) - 2 * pi * rank_col_4.at<uchar>(i, m) < (3 * pi / 2))
		{
			result_col.at<double>(i, m) = result_col.at<double>(i, m)- 2 * pi * rank_col_4.at<uchar>(i, m) + 2 * pi * rank_col_4.at<uchar>(i, m);
			if (sk == 1)
				K3.at<double>(i, m) = 1;
			continue;
		}

		else if(result_col.at<double>(i, m) - 2 * pi * rank_col_4.at<uchar>(i, m) >= (3 * pi / 2))
		{
			result_col.at<double>(i, m) = result_col.at<double>(i, m)- 2 * pi * rank_col_4.at<uchar>(i, m) + 2 * pi * rank_col_5.at<uchar>(i, m)+2*pi;
			if (sk == 1)
				K3.at<double>(i, m) = 1;
			continue;
		}
	}
	int k = j;
	for (int m = j + 1; m < k + y; m++)
	{

		//if (i == 600 && m >= 636)
		//	cout << endl;
		if (rank_col_4.at<uchar>(i, m) != post)
		{
			j = m-1;
			break;
		}

		if (result_col.at<double>(i, m) == 0)
		{
			continue;
		}

		if (result_col.at<double>(i, m) <= (pi/2))
		{
			result_col.at<double>(i, m) = result_col.at<double>(i, m) + 2 * pi * rank_col_6.at<uchar>(i, m) + 4 * pi;
			if (sk == 1)
				K3.at<double>(i, m) = 1;
		}
		else if (result_col.at<double>(i, m) > (pi / 2) && result_col.at<double>(i, m) < (3 * pi / 2))
		{
			result_col.at<double>(i, m) = result_col.at<double>(i, m) + 2 * pi * rank_col_4.at<uchar>(i, m);
			if (sk == 1)
				K3.at<double>(i, m) = 1;
		}
		else if(result_col.at<double>(i, m) >= (3 * pi / 2))
		{
			result_col.at<double>(i, m) = result_col.at<double>(i, m) + 2 * pi * rank_col_6.at<uchar>(i, m)+2*pi;
			if (sk == 1)
				K3.at<double>(i, m) = 1;
		}
		j = m;
	}
}
void correct_3(int& i, int& j, int pre, int post, cv::Mat& result_row, cv::Mat& rank_row_4, cv::Mat& rank_row_5)
{

	for (int m = i; m > i - y; m--)
	{

		if (rank_row_4.at<uchar>(m, j) != pre)
		{
			break;
		}
		if ((result_row.at<double>(m, j) - 2 * pi * rank_row_4.at<uchar>(m, j)) == 0 || result_row.at<double>(m, j)==0)
		{
			continue;
		}
		if ((result_row.at<double>(m, j) - 2 * pi * rank_row_4.at<uchar>(m, j)) <= (pi / 2))
		{
			result_row.at<double>(m, j) = result_row.at<double>(m, j) - 2 * pi * rank_row_4.at<uchar>(m, j) + 2 * pi * rank_row_5.at<uchar>(m, j) + 2 * pi;
		}
		else if (result_row.at<double>(m, j) - 2 * pi * rank_row_4.at<uchar>(m, j) > (pi / 2) && result_row.at<double>(m, j) - 2 * pi * rank_row_4.at<uchar>(m, j) < (3 * pi / 2))
		{
			result_row.at<double>(m, j) = result_row.at<double>(m, j) - 2 * pi * rank_row_4.at<uchar>(m, j) + 2 * pi * rank_row_4.at<uchar>(m, j);
		}
		else if(result_row.at<double>(m, j) - 2 * pi * rank_row_4.at<uchar>(m, j) >= (3 * pi / 2))
		{
			result_row.at<double>(m, j) = result_row.at<double>(m, j) - 2 * pi * rank_row_4.at<uchar>(m, j) + 2 * pi * rank_row_5.at<uchar>(m, j);
		}
	}
	int k = i;
	for (int m = i + 1; m < k + y; m++)
	{
		if (m == 5120 || j == 5120)
			cout << endl;
		if (rank_row_4.at<uchar>(m, j) != post)
		{
			i = m-1;
			break;
		}
		if (result_row.at<double>(m, j) == 0)
			continue;

		if (result_row.at<double>(m, j) <= (pi / 2))
		{
			result_row.at<double>(m, j) = result_row.at<double>(m, j) + 2 * pi * rank_row_5.at<uchar>(m, j) + 2 * pi;
		}
		else if (result_row.at<double>(m, j) > (pi / 2) && result_row.at<double>(m, j) < (3 * pi / 2))
		{
			result_row.at<double>(m, j) = result_row.at<double>(m, j) + 2 * pi * rank_row_4.at<uchar>(m, j);
		}

		else if(result_row.at<double>(m, j) >= (3 * pi / 2))
		{
			result_row.at<double>(m, j) = result_row.at<double>(m, j) + 2 * pi * rank_row_5.at<uchar>(m, j);
		}
		i = m;
	}
}

void correct_4(int& i, int& j, int pre, int post, cv::Mat& result_row, cv::Mat& rank_row_4, cv::Mat& rank_row_5, cv::Mat& rank_row_6)
{
	for (int m = i; m > i - y; m--)
	{
		//if (i == 374&&j==310)
		//	cout << "1";
		if (rank_row_4.at<uchar>(m, j) != pre)
		{
			break;
		}
		if ((result_row.at<double>(m, j) - 2 * pi * rank_row_4.at<uchar>(m, j)) == 0 || result_row.at<double>(m, j)==0)
			continue;
		if ((result_row.at<double>(m, j) - 2 * pi * rank_row_4.at<uchar>(m, j)) <= (pi / 2) )
		{
			result_row.at<double>(m, j) = result_row.at<double>(m, j) - 2 * pi * rank_row_4.at<uchar>(m, j) + 2 * pi * rank_row_5.at<uchar>(m, j) + 4 * pi;
		}
		else if (result_row.at<double>(m, j) - 2 * pi * rank_row_4.at<uchar>(m, j) > (pi / 2) && result_row.at<double>(m, j) - 2 * pi * rank_row_4.at<uchar>(m, j) < (3 * pi / 2))
		{
			result_row.at<double>(m, j) = result_row.at<double>(m, j) - 2 * pi * rank_row_4.at<uchar>(i, m) + 2 * pi * rank_row_4.at<uchar>(m, j);
		}
		else if(result_row.at<double>(m, j) - 2 * pi * rank_row_4.at<uchar>(m, j) >= (3 * pi / 2))
		{
			result_row.at<double>(m, j) = result_row.at<double>(m, j) - 2 * pi * rank_row_4.at<uchar>(m, j) + 2 * pi * rank_row_5.at<uchar>(m, j)+2*pi;
		}
	}
	int k = i;
	for (int m = i + 1; m < k + y; m++)
	{

		if (m >= height || rank_row_4.at<uchar>(m, j) != post)
		{
			i = m-1;
			break;
		}
		if (result_row.at<double>(m, j) == 0)
			continue;

		if (result_row.at<double>(m, j) <= (pi / 2) )
		{
			result_row.at<double>(m, j) = result_row.at<double>(m, j) + 2 * pi * rank_row_6.at<uchar>(m, j) + 4 * pi;
		}
		else if (result_row.at<double>(m, j) > (pi / 2) && result_row.at<double>(m, j) < (3 * pi / 2))
		{
			result_row.at<double>(m, j) = result_row.at<double>(m, j) + 2 * pi * rank_row_4.at<uchar>(m, j);
		}

		else if(result_row.at<double>(m, j) >= (3 * pi / 2))
		{
			result_row.at<double>(m, j) = result_row.at<double>(m, j) + 2 * pi * rank_row_6.at<uchar>(m, j)+2*pi;
		}
		i = m;
	}
}

cv::Mat phaseshift_col(int number_i,int flag1,std::string p = path)
{
	
	vector<cv::Mat>phase(step);
	for (int i = 0; i < step; i++)
		phase[i] = cv::Mat(height, width, CV_64FC1, cv::Scalar::all(0));//初始化
	if (flag1 == 0) 
	{
		for (int i = 9; i < 9 + step; i++)
		{
			if (i < 10)
			{
				convertType(cv::imread(p + "0" + std::to_string(i) + ".bmp", 0), phase[i - 9]);//转换为浮点
				imageFilter(phase[i - 9]);
			}
			else
			{
				convertType(cv::imread(p + std::to_string(i) + ".bmp", 0), phase[i - 9]);
				imageFilter(phase[i - 9]);
			}
		}
	}
	else
	{
		for (int i = 9; i < 9 + step; i++)
		{
			if (i < 10)
			{
				convertType(cv::imread(p + "0" + std::to_string(i) + ".bmp", 0), phase[i - 9]);//转换为浮点
				imageFilter(phase[i - 9]);
			}
			else
			{
				convertType(cv::imread(p + std::to_string(i) + ".bmp", 0), phase[i - 9]);
				imageFilter(phase[i - 9]);
			}
		}
	}




	cv::Mat phase_ = phaseShift_fourStep(flag1,phase);
	//phase_ += pi;
	cv::Mat phi1_Normal = cv::Mat(phase_.size(), phase_.type());
	cv::normalize(phase_, phi1_Normal, 0., 255., cv::NORM_MINMAX);
	cv::imwrite(p + "fourstep_consult_col" + ".bmp", phi1_Normal);
	return phase_;
}

cv::Mat phaseshift_row(std::string p=path)
{
	vector<cv::Mat>phase(step);
	for (int i = 0; i < step; i++)
		phase[i] = cv::Mat(height, width, CV_64FC1, cv::Scalar::all(0));//初始化
	for (int i = 9 + step; i < 9 + step + step; i++)//前一个step是列相位图，不读
	{
		if (i < 10)
		{
			convertType(cv::imread(p + "0" + std::to_string(i) + ".bmp", 0), phase[i - 9 - step]);//转换为浮点
			imageFilter(phase[i - 9 - step]);
		}
		else
		{
			convertType(cv::imread(p + std::to_string(i) + ".bmp", 0), phase[i - 9 - step]);
			imageFilter(phase[i - 9 - step]);
		}
			
	}

	cv::Mat phase_ = phaseShift_fourStep(0,phase);
	//phase_ += pi;
	cv::Mat phi1_Normal = cv::Mat(phase_.size(), phase_.type());
	cv::normalize(phase_, phi1_Normal, 0., 255., cv::NORM_MINMAX);
	cv::imwrite(p + "fourstep_consult_row" + ".bmp", phi1_Normal);
	return phase_;
}
void imageFilter(cv::Mat& imgv)
{
	cv::Mat temp = imgv.clone();
	cv::GaussianBlur(temp, imgv, cv::Size(5, 5), 1);
}
void convertType(const cv::Mat& srcImg, cv::Mat& distImg)
{
	if (srcImg.size() != distImg.size())
		exit(0);

	for (int i = 0; i < srcImg.rows; i++)
		for (int j = 0; j < srcImg.cols; j++)
			distImg.at<double>(i, j) = double(srcImg.at<uchar>(i, j));

	//return distImg;
}

cv::Mat phaseShift_fourStep(int flag1, vector<cv::Mat>phase)
{
	//判断类型，防治at<float>出错
	if (CV_64FC1 != phase[0].type() || CV_64FC1 != phase[1].type() || CV_64FC1 != phase[2].type())
	{
		std::cout << "矩阵类型不是 CV_64FC1 ，无法操作" << std::endl;
		exit(-1);
	}
	cv::Mat phi = cv::Mat(phase[0].size(), phase[0].type());
	for (int i = 0; i < phi.rows; i++)
		for (int j = 0; j < phi.cols; j++)
		{

			double temp1 = 0;
			double temp2 = 0;
			double color1 = 0;
			for (int k = 0; k < phase.size(); k++)
			{
				temp1 = temp1 + phase[k].at<double>(i, j) * sin(2 * pi * k / phase.size());
				temp2 = temp2 + phase[k].at<double>(i, j) * cos(2 * pi * k / phase.size());
				color1 = color1 + phase[k].at<double>(i, j) / phase.size();
			}
			double temp = atan2(-temp1, temp2);
			if (temp1 == 0)
				temp = -temp;
			if (flag1!=0)
			{
				double fs = temp1;
				double fc = temp2;
				double Bc = sqrt(fs * fs + fc * fc)*2 / 3;
				if (Bc < 10)
				{
					phi.at<double>(i, j) = 0;
				}

				else
				{
					phi.at<double>(i, j) = (temp + pi);
					color.at<double>(i, j) = color1 * 2;
				}

			}
			else
				phi.at<double>(i, j) = (temp+pi);
		}

	cv::imwrite(path + "color" + ".bmp", color);
	return phi;
}
//解格雷码

void decode_row_5_idea_2(cv::Mat& rank_4, cv::Mat& rank_5, cv::Mat& rank_6, std::string p)
{

	vector<cv::Mat>gray_code(4);
	vector<cv::Mat>Phase(step);
	cv::Mat	value = cv::Mat(height, width, CV_64FC1, cv::Scalar::all(0));//用以保存阈值


	for (int i = 0; i < 4; ++i)
	{
		gray_code[i] = cv::Mat(height, width, CV_8UC1, cv::Scalar::all(0));	
	}

	for (int i = 0; i < step; ++i)
	{
		Phase[i] = cv::Mat(height, width, CV_64FC1, cv::Scalar::all(0));
	}

	for (int i = 9 + step; i < 9 + step + step; i++)
	{
		convertType(cv::imread(p + std::to_string(i) + ".bmp", 0), Phase[i - 9 - step]);//转换为浮点
		imageFilter(Phase[i - 9 - step]);
	}
	for (int i = 5; i < 9 ; i++)
	{
		gray_code[i-5] = cv::imread(p + "0" + std::to_string(i) + ".bmp", 0);
		imageFilter(gray_code[i-5]);
	}
	double k = 0;
	for (int i = 0; i < height; ++i)
	{
		for (int j = 0; j < width; ++j)
		{
			for (int m = 0; m < Phase.size(); m++)
			{
				value.at<double>(i, j) += Phase[m].at<double>(i, j) / Phase.size();
			}
		}
	}
	for (int i = 0; i < width; ++i)
	{
		for (int j = 0; j < height; ++j)
		{
			//二值化
			if (gray_code[0].at<uchar>(j, i) > value.at<double>(j, i))
				gray_code[0].at<uchar>(j, i) = 255;
			else
				gray_code[0].at<uchar>(j, i) = 0;

			if (gray_code[1].at<uchar>(j, i) > value.at<double>(j, i))
				gray_code[1].at<uchar>(j, i) = 255;
			else
				gray_code[1].at<uchar>(j, i) = 0;

			if (gray_code[2].at<uchar>(j, i) > value.at<double>(j, i))
				gray_code[2].at<uchar>(j, i) = 255;
			else
				gray_code[2].at<uchar>(j, i) = 0;

			if (gray_code[3].at<uchar>(j, i) > value.at<double>(j, i))
				gray_code[3].at<uchar>(j, i) = 255;
			else
				gray_code[3].at<uchar>(j, i) = 0;
			//解码
			uchar temp = 0;
			uchar p1 = gray_code[0].at<uchar>(j, i) == 255 ? 1 : 0;
			uchar p2 = gray_code[1].at<uchar>(j, i) == 255 ? 1 : 0;
			uchar p3 = gray_code[2].at<uchar>(j, i) == 255 ? 1 : 0;
			uchar p4 = gray_code[3].at<uchar>(j, i) == 255 ? 1 : 0;

			int q1 = p1;
			int q2 = p1 ^ p2;
			int q3 = q2 ^ p3;
			int q4 = q3 ^ p4;

			temp = q4 + (q3 << 1) + (q2 << 2) + (q1 << 3);
			rank_4.at<uchar>(j, i) = temp;

			temp = (q3 + (q2 << 1) + (q1 << 2)) * 2;
			rank_5.at<uchar>(j, i) = temp;
			if (temp > 0)
				rank_6.at<uchar>(j, i) = temp - 2;

		}
	}

	//保存二值化结果
	for (int m = 0; m < gray_code.size(); m++)
	{
		cv::imwrite(p + +"p" + std::to_string(m + 24) + ".bmp", gray_code[m]);
	}
	//保存级次结果
	//cv::imwrite(p + "86" + ".bmp", rank_4);
	//cv::imwrite(p + "87" + ".bmp", rank_5);
	//cv::imwrite(p + "88" + ".bmp", rank_6);
}

void decode_col_5_idea_2(cv::Mat& rank_4, cv::Mat& rank_5, cv::Mat& rank_6, std::string p,int number_i,int flag)
{
	vector<cv::Mat>gray_code(4);
	vector<cv::Mat>Phase(step);
	cv::Mat value = cv::Mat(height, width, CV_64FC1, cv::Scalar::all(0));

	for (int i = 0; i < 4; ++i)
	{
		gray_code[i] = cv::Mat(height, width, CV_8UC1, cv::Scalar::all(0));
	}
	for (int i = 0; i < step; ++i)
	{
		Phase[i] = cv::Mat(height, width, CV_64FC1, cv::Scalar::all(0));
	}
	if (flag == 0)
	{
		for (int i = 9; i < 9 + step; i++)
		{
			if (i < 10)
			{
				convertType(cv::imread(p + "0" + std::to_string(i) + ".bmp", 0), Phase[i - 9]);//转换为浮点
				imageFilter(Phase[i - 9]);
			}
			else
			{
				convertType(cv::imread(p + std::to_string(i) + ".bmp", 0), Phase[i - 9]);//转换为浮点
				imageFilter(Phase[i - 9]);
			}

		}
		for (int i = 1; i < 5; i++)
		{
			if (i < 10)
			{
				gray_code[i - 1] = cv::imread(p + "0" + std::to_string(i) + ".bmp", 0);
				imageFilter(gray_code[i - 1]);
			}
			else
			{
				gray_code[i - 1] = cv::imread(p + std::to_string(i) + ".bmp", 0);
				imageFilter(gray_code[i - 1]);
			}
		}
	}
	else
	{

		for (int i = 9; i < 9 + step; i++)
		{
			if (i < 10)
			{
				convertType(cv::imread(p + "0" + std::to_string(i) + ".bmp", 0), Phase[i-9]);//转换为浮点
				imageFilter(Phase[i-9]);
			}
			else
			{
				convertType(cv::imread(p + std::to_string(i) + ".bmp", 0), Phase[i-9]);//转换为浮点
				imageFilter(Phase[i-9]);
			}

		}

		for (int i = 1; i < 5; i++)
		{
			if (i < 10)
			{
				gray_code[i-1] = cv::imread(p + "0" + std::to_string(i) + ".bmp", 0);
				imageFilter(gray_code[i-1]);
			}
			else
			{
				gray_code[i-1] = cv::imread(p + std::to_string(i) + ".bmp", 0);
				imageFilter(gray_code[i-1]);
			}
		}
	}


	double k = 0;
	for (int i = 0; i < height; ++i)
	{
		for (int j = 0; j < width; ++j)
		{
			for (int m = 0; m < Phase.size(); m++)
			{
				value.at<double>(i, j) += Phase[m].at<double>(i, j) / Phase.size();
			}
		}
	}
	for (int i = 0; i < width; ++i)
	{
		for (int j = 0; j < height; ++j)
		{
			//二值化
			for (int h = 0; h < gray_code.size(); h++)
			{
				if (gray_code[h].at<uchar>(j, i) > value.at<double>(j, i))
					gray_code[h].at<uchar>(j, i) = 255;
				else
					gray_code[h].at<uchar>(j, i) = 0;
			}


			//解码
			uchar temp = 0;
			uchar p1 = gray_code[0].at<uchar>(j, i) == 255 ? 1 : 0;
			uchar p2 = gray_code[1].at<uchar>(j, i) == 255 ? 1 : 0;
			uchar p3 = gray_code[2].at<uchar>(j, i) == 255 ? 1 : 0;
			uchar p4 = gray_code[3].at<uchar>(j, i) == 255 ? 1 : 0;

			int q1 = p1;
			int q2 = p1 ^ p2;
			int q3 = q2 ^ p3;
			int q4 = q3 ^ p4;

			temp = q4 + (q3 << 1) + (q2 << 2) + (q1 << 3);
			rank_4.at<uchar>(j, i) = temp;

			temp = (q3 + (q2 << 1) + (q1 << 2)) * 2;
			rank_5.at<uchar>(j, i) = temp;
			if (temp > 0)
				rank_6.at<uchar>(j, i) = temp - 2;
		}
	}

	//保存二值化结果
	for (int m = 0; m < gray_code.size(); m++)
	{
		cv::imwrite(p + +"p" + std::to_string(m + 29) + ".bmp", gray_code[m]);
	}
	//保存级次结果
	//cv::imwrite(p + "79" + ".bmp", rank_4);
	//cv::imwrite(p + "80" + ".bmp", rank_5);
	//cv::imwrite(p + "81" + ".bmp", rank_6);
}

void creat_gray_code()
{

	int n = 6;
	std::vector<std::vector<int> >p(64, std::vector<int>(6));
	p[0][0] = 0;
	p[0][1] = 0;
	p[0][2] = 0;
	p[0][3] = 0;
	p[0][4] = 0;
	p[0][5] = 0;
	for (int i = 1; i < 64; ++i)
	{
		p[i] = p[i - 1];
		std::vector<int>& q = p[i];
		for (int j = 5; j >= 0; j--)
		{
			if (q[j])
			{
				q[j] = 0;
			}
			else
			{
				q[j] = 1;
				break;
			}
		}

	}
	for (int j = 0; j < 64; j++)
	{
		std::vector<int> tep(6);
		std::vector<int>& q = p[j];
		for (int i = 5; i >= 0; i--)
		{
			if (i == 0)
			{
				tep[i] = q[i];
				continue;
			}
			tep[i] = q[i] ^ q[i - 1];
		}
		q = tep;
	}

	std::vector<cv::Mat>g(6);
	std::vector<cv::Mat>in_g(6);
	for (int i = 0; i < 6; i++)
	{
		g[i] = cv::Mat(1080, 1920, CV_8UC1, cv::Scalar::all(0));
		in_g[i] = cv::Mat(1080, 1920, CV_8UC1, cv::Scalar::all(0));
	}

	for (int j = 0; j < 6; j++)
	{
		for (int i = 0; i < 1920; i += 1)
		{

			int k = i / 30;
			if (p[k][j] == 1)
			{
				g[j].col(i) = 255;
				in_g[j].col(i) = 0;
			}

			else
			{
				g[j].col(i) = p[k][j];
				in_g[j].col(i) = 255;
			}

		}
		cv::imwrite(path + "0" + std::to_string(j + 1) + ".bmp", g[j]);
		if(j+7<10)
			cv::imwrite(path + "0" + std::to_string(j + 7) + ".bmp", in_g[j]);
		else
			cv::imwrite(path + std::to_string(j + 7) + ".bmp", in_g[j]);
	}

	for (int i = 0; i < 64; ++i)
	{
		for (int j = 0; j < 6; ++j)
		{
			std::cout << p[i][j];
		}
		std::cout << std::endl;
	}
}




void creat_gray_code_4()
{
	int h = 1200;
	int w = 1920;
	int n = 4;
	std::vector<std::vector<int> >p(32, std::vector<int>(n));
	p[0][0] = 0;
	p[0][1] = 0;
	p[0][2] = 0;
	p[0][3] = 0;
	//p[0][4] = 0;
	//生成二进制码
	for (int i = 1; i < 32; ++i)
	{
		p[i] = p[i - 1];
		std::vector<int>& q = p[i];
		for (int j = n - 1; j >= 0; j--)
		{
			if (q[j])
			{
				q[j] = 0;
			}
			else
			{
				q[j] = 1;
				break;
			}
		}
	}

	//换成0-1格雷码
	for (int j = 0; j < 32; j++)
	{
		std::vector<int> tep(n);
		std::vector<int>& q = p[j];
		for (int i = n - 1; i >= 0; i--)
		{
			if (i == 0)
			{
				tep[i] = q[i];
				continue;
			}
			tep[i] = q[i] ^ q[i - 1];
		}
		q = tep;
	}



	//换成0-255格雷码
	std::vector<cv::Mat>g(n);
	std::vector<cv::Mat>_g(n);
	std::vector<cv::Mat>in_g(n);
	std::vector<cv::Mat>_in_g(n);
	for (int i = 0; i < n; i++)
	{
		g[i] = cv::Mat(h, w, CV_8UC1, cv::Scalar::all(0));
		in_g[i] = cv::Mat(h, w, CV_8UC1, cv::Scalar::all(0));
		_g[i] = cv::Mat(h, w, CV_8UC1, cv::Scalar::all(0));
		_in_g[i] = cv::Mat(h, w, CV_8UC1, cv::Scalar::all(0));
	}

	

	//列格雷myself
	for (int j = 0; j < n; j++)
	{
		for (int i = 0; i < 1920; i += 1)//改
		{
	
			int k = floor(i / 120);//改
			if (p[k][j] == 1)
			{
				g[j].col(i) = 255;
				in_g[j].col(i) = 0;
			}
	
			else
			{
				g[j].col(i) = p[k][j];
				in_g[j].col(i) = 255;
	
			}
		}
		cv::imwrite(path + "0" + std::to_string(j) + ".bmp", g[j]);
	}


	//行格雷码 myself
	for (int j = 0; j <= n-1; j++)
	{
		for (int i = 0+j; i < 1200; i += 1)//改
		{
			int _k = floor((i - j) / 75);//改
			if (p[_k][j] == 1)
			{
				_g[j].row(i) = 255;
				_in_g[j].row(i) = 0;
			}
	
			else
			{
				_g[j].row(i) = p[_k][j];
				_in_g[j].row(i) = 255;
	
			}
		}
	
	
	
	
		cv::imwrite(path +"0"+ std::to_string(j + 4) + ".bmp", _g[j]);
	
		//cv::imwrite(path + std::to_string(j + 16) + ".bmp", _in_g[j]);
	}
}


void create_phase_Picture()
{

	//  
	std::string path = "E:/2021.4.9/";
	int const step = 12;
	cv::Mat img1 = cv::Mat(1200, 1920, CV_8UC1, cv::Scalar::all(255)); //CV_64FC1
	cv::Mat img2 = cv::Mat(1200, 1920, CV_8UC1, cv::Scalar::all(0));
	cv::Mat img[step*2];
	for (int i = 0; i < step*2; i++)
	{
		img[i] = cv::Mat(1200, 1920, CV_8UC1, cv::Scalar::all(0));
	}
	double A = 0.5, B = 0.5;
//row
	for (int h = 0; h <1200 ; h++)
	{
		for (int j = step; j < step*2; j++)
		{
			double m=0;
			m = A + B * cos((2 * pi * (h) / 75 + pi + 2 * pi * (j-step) / step));
			m=pow(m,1/1.00)*255;
			img[j].row(h) = m;//T2=68,//采用条纹宽度

			//m= A + B * cos((2 * pi * 16 / 1920 * h + pi + 2 * pi * j / step));
			//m=pow(m,1/2.37)*255;
			//img[j].col(h) = m;//T2=16，采用的是周期数
		}
	}
//col
	for (int h = 0; h < 1920; h++)
	{
		for (int j = 0; j < step; j++)
		{
			double m = 0;
			m= A + B * cos((2 * pi * h / 120 + pi + 2 * pi * j / step)); 
			m=pow(m,1/1.00)*255;

			img[j].col(h) = m;//T2=16，采用的是周期数
			//img[j].col(h) = pow(0.4, 1.0 / 2.5)* 255;


		}
	}


	//for (int j = 0; j < step; j++)
	//{
	//	double cur = img[j].at<uchar>(0, 0);
	//	int n = 0; ;
	//	for (int k = 0; k < 1920; k++)
	//	{
	//		if (n == 119)
	//		{
	//			n = 0;
	//			cur = img[j].at<uchar>(0, k);
	//		}
	//		n++;
	//		img[j].col(k) = cur;
	//	}
	//}



	//cv::imwrite(path + "00.bmp", img1);
	//for (int i = 0; i < 4; i++)
	//{
	//	if (i + 8 < 10)
	//		cv::imwrite(path + "0" + std::to_string(i + 8) + ".bmp", img[i]);
	//	else
	//		cv::imwrite(path + std::to_string(i + 8) + ".bmp", img[i]);
	//}
	//cv::imwrite(path + std::to_string(20) + ".bmp", img2);
	for (int i = 0; i < 2*step; i++)
	{
		if (i+9  < 10)
			cv::imwrite(path + "0" + std::to_string(i+9) + ".bmp", img[i]);
		else
			cv::imwrite(path + std::to_string(i+9) + ".bmp", img[i]);
	}
}


