#ifndef VISUAL_ODOMETRY_H_
#define VISUAL_ODOMETRY_H_

#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

// ZED includes
#include <sl_zed/Camera.hpp>
#include <pinhole_camera.h>

using namespace cv;

class VisualOdometry
{
public:

	enum FrameStage {
		STAGE_FIRST_FRAME,
		STAGE_SECOND_FRAME,
		STAGE_DEFAULT_FRAME
	};

	VisualOdometry(PinholeCamera* cam);
	virtual ~VisualOdometry();

	/// Add image
	void addImage(const cv::Mat& img_L, const cv::Mat& img_R);
	/// Get r
	cv::Mat getCurrentR() { return cur_R_; }
	/// Get t
	cv::Mat getCurrentT() { return cur_t_; }

protected:
	virtual bool processFirstFrame();
	virtual bool processSecondFrame();
	virtual bool processFrame();
	void featureTracking(cv::Mat image_ref, cv::Mat image_cur, std::vector<cv::Point2f>& px_ref, std::vector<cv::Point2f>& px_cur);
	void BFmatching(std::vector<cv::Point2f> &pre_L, std::vector<cv::Point2f> &cur_L, std::vector<cv::Point2f> &pre_L_in, std::vector<cv::Point2f> &cur_L_in);
	void Tracepoints(std::vector<cv::Point2f> px_ref, cv::Mat point_cloud_ref, std::vector<cv::Point2f> px_cur, cv::Mat point_cloud_cur, std::vector<cv::Point3f> &points_xyz, std::vector<cv::Point2f> &px_cur_inlier);
	void ORB_matching(const cv::Mat &left_img, const cv::Mat &right_img, std::vector<cv::Point2f> &px_L_inlier, std::vector<cv::Point2f> &px_R_inlier, cv::Mat &desc);
	void XYZ(cv::Mat cam_matrix, std::vector<cv::Point2f> &points_L, std::vector<cv::Point2f> &points_R, std::vector<cv::Point3f> &points3D, cv::Mat &Mat3Dpoints);
	void RANSAC(std::vector<cv::Point2f> px_ref, std::vector<cv::Point2f> px_cur, std::vector<cv::Point2f> &px_ref_inlier, std::vector<cv::Point2f> &px_cur_inlier, std::vector< DMatch > &matcher);

protected:
	FrameStage frame_stage_;
	PinholeCamera *cam_;
	cv::Mat new_frame_L;
	cv::Mat last_frame_L;
	cv::Mat new_frame_R;
	cv::Mat last_frame_R;
	cv::Mat ini_frame_;
	cv::Mat new_point_cloud_Mat;
	cv::Mat last_point_cloud_Mat;
	sl::Mat new_point_cloud_;
	sl::Mat last_point_cloud_;

	cv::Mat cameraMatrix_left_;
	cv::Mat cameraDistortion_left_;

	cv::Mat cur_R_;
	cv::Mat cur_t_;

	cv::Mat desc_cur_L;
	cv::Mat desc_ref_L;

	std::vector<cv::Point2f> px_ref_L;
	std::vector<cv::Point2f> px_cur_L;
	std::vector<cv::Point2f> px_ref_R;
	std::vector<cv::Point2f> px_cur_R;
	std::vector<cv::Point2f> px_ini_;
	std::vector<cv::Point2f> px_ref_inlier_tri_L;
	std::vector<cv::Point2f> px_cur_inlier_final_L;
	std::vector<cv::Point3f> points_xyz_;

	Ptr<ORB> L_orb = ORB::create(800,1.35f,5,31,0,2,0,31,20);
	Ptr<ORB> R_orb = ORB::create(800,1.35f,5,31,0,2,0,31,20);

    BFMatcher BFM;

	double focal_;
	cv::Point2d pp_;

};

#endif // VISUAL_ODOMETRY_H_
