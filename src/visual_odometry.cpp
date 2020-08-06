#include <include_all.h>
#include <visual_odometry.h>
#include <string>
#include <fstream>
#include <iostream>

using namespace std;
using namespace cv;

VisualOdometry::VisualOdometry(PinholeCamera* cam):
cam_(cam)
{
    cameraMatrix_left_ = (cv::Mat_<double>(3, 3) << cam_->fx(), 0, cam_->cx(), 0, cam_->fy(), cam_->cy(), 0, 0, 1);
    cameraDistortion_left_ = (cv::Mat_<double>(5, 1) << cam_->k1(), cam_->k2(), cam_->p1(), cam_->p2(), cam_->k3());
	focal_ = cam_->fx();
	pp_ = cv::Point2d(cam_->cx(), cam_->cy());

	frame_stage_ = STAGE_FIRST_FRAME;
}

VisualOdometry::~VisualOdometry()
{

}

void VisualOdometry::addImage(const cv::Mat& img_L, const cv::Mat& img_R)
{

	if (img_L.empty() || img_L.type() != CV_8UC1 || img_L.cols != cam_->width() || img_L.rows != cam_->height())
		throw std::runtime_error("Frame: provided left image has not the same size as the camera model or image is not grayscale");
	if (img_R.empty() || img_R.type() != CV_8UC1 || img_R.cols != cam_->width() || img_R.rows != cam_->height())
		throw std::runtime_error("Frame: provided right image has not the same size as the camera model or image is not grayscale");

	new_frame_L = img_L;
	new_frame_R = img_R;

    auto start2 = high_resolution_clock::now();

    vector<cv::Point2f> px_L_in, px_R_in;
    ORB_matching(new_frame_L, new_frame_R, px_L_in, px_R_in, desc_cur_L);
    //px_cur_L = px_L_in;                                                              //BF used

	// Calculate 3D feature points value
    vector<cv::Point3f> px_3D;
	XYZ(cameraMatrix_left_, px_L_in, px_R_in, px_3D, new_point_cloud_Mat);
	//cout << "px_3D: " << px_3D << endl;
	//cout << "Size of 3D points from ORB matching:  " << px_3D.size() << endl;

	auto stop2 = high_resolution_clock::now();
	auto duration2 = duration_cast<milliseconds>(stop2 - start2);
	//cout << "Computation of Depth map takes: " << duration2.count() << " ms" << endl;

	bool res = true;
	if (frame_stage_ == STAGE_DEFAULT_FRAME){

		auto start10 = high_resolution_clock::now();

		res = processFrame();

		auto stop10 = high_resolution_clock::now();
		auto duration10 = duration_cast<milliseconds>(stop10 - start10);
		//cout << "processFrame takes: " << duration10.count() << " ms" << endl;

	}
	else if (frame_stage_ == STAGE_SECOND_FRAME)
		res = processSecondFrame();
	else if (frame_stage_ == STAGE_FIRST_FRAME){
		res = processFirstFrame();
	}

	last_frame_L = new_frame_L.clone();
	last_frame_R = new_frame_R.clone();

	last_point_cloud_Mat = new_point_cloud_Mat.clone();

	px_ref_L = px_L_in;
	//px_ref_L = px_cur_L;                                                              //BF used
	//desc_ref_L = desc_cur_L;                                                          //BF used

}

bool VisualOdometry::processFirstFrame()
{
	frame_stage_ = STAGE_SECOND_FRAME;

	//cout << "Firstframe OK!" << endl;
	return true;
}

bool VisualOdometry::processSecondFrame()
{
	cv::Mat rvec, R, tvec, inliers;
	Eigen::Isometry3d T;
	vector<cv::Point2f> L_pre_in, L_cur_in;

	// RANSAC parameters
	int iterationsCount = 500;         // number of Ransac iterations.100
	float reprojectionError = 5;       // maximum allowed distance to consider it an inlier.8
	double confidence = 0.95;          // ransac successful confidence.0.99

	featureTracking(last_frame_L.clone(), new_frame_L.clone(), px_ref_L, px_cur_L);

	//BFmatching(px_ref_L, px_cur_L, L_pre_in, L_cur_in);                               //BF used

	//Tracepoints(L_pre_in, last_point_cloud_Mat, L_cur_in, new_point_cloud_Mat, points_xyz_, px_cur_inlier_final_L);

	Tracepoints(px_ref_L, last_point_cloud_Mat, px_cur_L, new_point_cloud_Mat, points_xyz_, px_cur_inlier_final_L);

	cv::solvePnPRansac(points_xyz_, px_cur_inlier_final_L, cameraMatrix_left_, cameraDistortion_left_, rvec, tvec, false, iterationsCount, reprojectionError, confidence, inliers, SOLVEPNP_ITERATIVE);
	cv::Rodrigues(rvec.clone(), R);

	cur_R_ = R.clone();
	cur_t_ = tvec.clone();

	frame_stage_ = STAGE_DEFAULT_FRAME;

	//cout << "Secondframe OK!" << endl;
	return true;
}

bool VisualOdometry::processFrame()
{
	cv::Mat rvec_, R_, tvec_, inliers, cur_t_tmp, cur_R_tmp;
	Eigen::Isometry3d T_;
	Eigen::Matrix3d r_;
	vector<cv::Point2f> L_pre_in, L_cur_in;

	// RANSAC parameters
	int iterationsCount = 500;         // number of Ransac iterations.100
	float reprojectionError = 5;       // maximum allowed distance to consider it an inlier.8
	double confidence = 0.95;          // ransac successful confidence.0.99

	featureTracking(last_frame_L.clone(), new_frame_L.clone(), px_ref_L, px_cur_L);

	//BFmatching(px_ref_L, px_cur_L, L_pre_in, L_cur_in);

	//Tracepoints(L_pre_in, last_point_cloud_Mat, L_cur_in, new_point_cloud_Mat, points_xyz_, px_cur_inlier_final_L);

	Tracepoints(px_ref_L, last_point_cloud_Mat, px_cur_L, new_point_cloud_Mat, points_xyz_, px_cur_inlier_final_L);

	cv::solvePnPRansac(points_xyz_, px_cur_inlier_final_L, cameraMatrix_left_, cameraDistortion_left_, rvec_, tvec_, false, iterationsCount, reprojectionError, confidence, inliers, SOLVEPNP_ITERATIVE);
	cv::Rodrigues(rvec_.clone(), R_);

	cur_t_ = cur_t_.clone() + ((cur_R_.clone())*(tvec_.clone()));
	cur_R_ = ((cur_R_.clone())*(R_.clone()));

	return true;
}

void VisualOdometry::featureTracking(cv::Mat image_ref, cv::Mat image_cur, std::vector<cv::Point2f>& px_ref, std::vector<cv::Point2f>& px_cur)
{
	const double klt_win_size = 21.0;
	const int klt_max_iter = 30;
	const double klt_eps = 0.001;
	std::vector<uchar> status;
	std::vector<float> error;
	std::vector<float> min_eig_vec;
	cv::TermCriteria termcrit(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, klt_max_iter, klt_eps);

	cv::calcOpticalFlowPyrLK(image_ref.clone(), image_cur.clone(), px_ref, px_cur, status, error,
							 cv::Size2i(klt_win_size, klt_win_size), 4, termcrit, 0);

}

void VisualOdometry::BFmatching(std::vector<cv::Point2f> &pre_L, std::vector<cv::Point2f> &cur_L, std::vector<cv::Point2f> &pre_L_in, std::vector<cv::Point2f> &cur_L_in){

	vector< DMatch > matches, RANSAC_matches;
    vector< KeyPoint > kt_pre_L, kt_cur_L, kt_pre_L2, kt_cur_L2;
    vector< Point2f > pre_L_matched, cur_L_matched;
    cv::Mat result, result2;

	BFM.match(desc_ref_L, desc_cur_L, matches);

	KeyPoint::convert(pre_L,kt_pre_L);
	KeyPoint::convert(cur_L,kt_cur_L);

	cv::drawMatches(last_frame_L.clone(), kt_pre_L, new_frame_L.clone(), kt_cur_L, matches, result);

	for(size_t i = 0; i < matches.size(); i++){
    	pre_L_matched.push_back(kt_pre_L[matches[i].queryIdx].pt);
        cur_L_matched.push_back(kt_cur_L[matches[i].trainIdx].pt);
    }

	imshow("initial matches", result);

	RANSAC(pre_L_matched, cur_L_matched, pre_L_in, cur_L_in, RANSAC_matches);

	KeyPoint::convert(pre_L_in,kt_pre_L2);
	KeyPoint::convert(cur_L_in,kt_cur_L2);

	cv::drawMatches(last_frame_L.clone(), kt_pre_L2, new_frame_L.clone(), kt_cur_L2, RANSAC_matches, result2);

	imshow("matched2", result2);
}

void VisualOdometry::Tracepoints(std::vector<cv::Point2f> px_ref, cv::Mat point_cloud_ref, std::vector<cv::Point2f> px_cur, cv::Mat point_cloud_cur, std::vector<cv::Point3f> &points_xyz, std::vector<cv::Point2f> &px_cur_inlier){

	points_xyz.clear();
	px_cur_inlier.clear();

	int pt_num = (int)px_ref.size();


	for(size_t i = 0; i < (px_ref).size(); i++)
	{
		float x_pos_ref = px_ref[i].x;
		float y_pos_ref = px_ref[i].y;
		float x_pos_cur = px_cur[i].x;
		float y_pos_cur = px_cur[i].y;

		float x = point_cloud_ref.at<Vec3f>(y_pos_ref,x_pos_ref)[0];
		float y = point_cloud_ref.at<Vec3f>(y_pos_ref,x_pos_ref)[1];
		float z = point_cloud_ref.at<Vec3f>(y_pos_ref,x_pos_ref)[2];
		float dist_check = sqrt(pow(x,2)+pow(y,2)+pow(z,2));

		if((dist_check > 0) && (dist_check < 5))
		{
    		points_xyz.push_back(cv::Point3f(x,y,z));
    		px_cur_inlier.push_back(cv::Point2f(x_pos_cur,y_pos_cur));
		}
	}
}

void VisualOdometry::ORB_matching(const cv::Mat &left_img, const cv::Mat &right_img, std::vector<cv::Point2f> &px_L_inlier, std::vector<cv::Point2f> &px_R_inlier, cv::Mat &desc){

    auto start3 = high_resolution_clock::now();

	cv::Mat image_L_display, image_R_display;
	cv::Mat left  =  left_img;
    cv::Mat right = right_img;

    std::vector<cv::KeyPoint> keypoints_L, keypoints_R;
    vector<cv::Point2f> px_L, px_R;
    cv::Mat descriptors_L, descriptors_R;

    L_orb -> detectAndCompute(left, cv::Mat(), keypoints_L, descriptors_L);
    //cv::drawKeypoints(left, keypoints_L, image_L_display);

    R_orb -> detectAndCompute(right, cv::Mat(), keypoints_R, descriptors_R);
    //cv::drawKeypoints(right, keypoints_R, image_R_display);

    auto stop3 = high_resolution_clock::now();
    auto duration3 = duration_cast<milliseconds>(stop3 - start3);
    //cout << "Computation of Features detection takes: " << duration3.count() << " ms" << endl;

    auto start4 = high_resolution_clock::now();

    int size1 = descriptors_L.rows;
	int size2 = descriptors_R.rows;
	//cout << "size of left descriptor: " << size1 << endl;
	//Collect features on the right epipolar line beforehand
	vector < vector <int> > ft_cand(right.rows, vector<int>(size2));

	for (int n=0; n< size2; n++){

		float y_cand = keypoints_R[n].pt.y;
		int ft_layer = keypoints_R[n].octave;
		int cand_range_up = y_cand - pow(1.2,ft_layer);
		int cand_range_down = y_cand + pow(1.2,ft_layer);

		for(int q = cand_range_up; q <= cand_range_down; q++){
			ft_cand[q].push_back(n);
		}
	}

    auto stop4 = high_resolution_clock::now();
    auto duration4 = duration_cast<microseconds>(stop4 - start4);
    //cout << "Computation of finding possible candidates takes: " << duration4.count() << " us" << endl;

    auto start5 = high_resolution_clock::now();

	for (int i = 0; i<size1; i++){

		auto start6 = high_resolution_clock::now();
		//cout << "feature on left image: " << keypoints_L[i].pt.x << "\t" << keypoints_L[i].pt.y << endl;
		float x_left = keypoints_L[i].pt.x;
		float y_left = keypoints_L[i].pt.y;
		float x_right_tmpcand;
		float y_right_tmpcand;
		int y_search = round(y_left);
		int min_ham = 1000000000;
		int ham_d = 0;
		int ham_all = 0;
		int search_times = ft_cand[y_search].size();
		int search_range = 50;

		// Search for the corresponding point with smallest hamming distance

		for (int j =0; j< search_times; j++){

			if ((keypoints_L[i].octave == keypoints_R[ft_cand[y_search][j]].octave) && (abs(keypoints_L[i].pt.x - keypoints_R[ft_cand[y_search][j]].pt.x) < search_range)){

				ham_all = 0;
				for (int k = 0; k < 8; k++) {
					unsigned int pt_l = (descriptors_L.at<unsigned int>(i,k));
					unsigned int pt_r = (descriptors_R.at<unsigned int>(ft_cand[y_search][j],k));
					ham_d = __builtin_popcount((unsigned int)pt_l ^ pt_r);
					ham_all = ham_all + ham_d;
				}
				if (ham_all < min_ham){
					//if ((keypoints_L[i].octave == keypoints_R[ft_cand[y_search][j]].octave) && (abs(keypoints_L[i].pt.x - keypoints_R[ft_cand[y_search][j]].pt.x) < search_range)){
						min_ham = ham_all;
						x_right_tmpcand = keypoints_R[ft_cand[y_search][j]].pt.x;
						y_right_tmpcand = keypoints_R[ft_cand[y_search][j]].pt.y;
					//}
				}
			}
		}

		//cout << "ham_distance: " << min_ham << endl;
		if (min_ham < 40 && (x_right_tmpcand < keypoints_L[i].pt.x)){
				px_L_inlier.push_back(cv::Point2d(keypoints_L[i].pt.x,keypoints_L[i].pt.y));
				px_R_inlier.push_back(cv::Point2d(x_right_tmpcand,y_right_tmpcand));
				//cout << "feature on right image: " << x_right_tmpcand << "\t" << y_right_tmpcand << endl;
		}
	    auto stop6 = high_resolution_clock::now();
	    auto duration6 = duration_cast<microseconds>(stop6 - start6);
	    //cout << "Computation of calculating hamming distance takes: " << duration6.count() << " us" << endl;
	}

    auto stop5 = high_resolution_clock::now();
    auto duration5 = duration_cast<milliseconds>(stop5 - start5);
    //cout << "Computation of searching matched features takes: " << duration5.count() << " ms" << endl;
	//cout << px_L_inlier.size() << endl;

	/*for(int i=0; i< px_R_inlier.size(); i++){
		circle(image_R_display, Point(px_R_inlier[i].x,px_R_inlier[i].y), 10, Scalar(0,255,0), 1);
		//line(image_R_display, Point(0,px_R_inlier[i].y), Point(672,px_R_inlier[i].y), Scalar(255,0,0), 1.5);
	}
	for(int i=0; i< px_L_inlier.size(); i++){
		circle(image_L_display, Point(px_L_inlier[i].x,px_L_inlier[i].y), 10, Scalar(0,255,0), 1);
		//line(image_L_display, Point(0,px_L_inlier[i].y), Point(672,px_L_inlier[i].y), Scalar(255,0,0), 1.5);
	}
	imshow("Detection_R", image_R_display);
	imshow("Detection_L", image_L_display);*/

    vector< KeyPoint > kt_L;
    KeyPoint::convert(px_L_inlier, kt_L);
    L_orb -> compute(left, kt_L, desc);

}

void VisualOdometry::XYZ(cv::Mat cam_matrix, vector<cv::Point2f> &points_L, vector<cv::Point2f> &points_R, vector<cv::Point3f> &points3D, cv::Mat &Mat3Dpoints){

	float pixel_length = 0.004;               //millimeter
	float f = cam_matrix.at<double>(0);       //pixel           Note: focal length:2.8mm, pixel length: 0.004 (mm/pixel)
	float cx = cam_matrix.at<double>(2);      //pixel
	float cy = cam_matrix.at<double>(5);      //pixel
	float B = 0.120;                          //cam_matrix.at<double>(3);       //millimeter
	int num = points_L.size();

	Mat3Dpoints.create(376,672,CV_32FC3);

	for (int p=0; p<num; p++)
	{
		float XL = points_L[p].x;
		float YL = points_L[p].y;
		float XR = points_R[p].x;
		float YR = points_R[p].y;
		float disL = XL-cx;
		float disR = cx-XR;
		float dis = (disL+disR); //pixel

		float Z = B*f/dis;
		float X = Z*(XL-cx)/f;
		float Y = Z*(YL-cy)/f;

		points3D.push_back(cv::Point3f(X,Y,Z));

		Mat3Dpoints.at<Vec3f>(YL,XL)[0] = X;
		Mat3Dpoints.at<Vec3f>(YL,XL)[1] = Y;
		Mat3Dpoints.at<Vec3f>(YL,XL)[2] = Z;
	}

}

void VisualOdometry::RANSAC(std::vector<cv::Point2f> px_ref, std::vector<cv::Point2f> px_cur, std::vector<cv::Point2f> &px_ref_inlier, std::vector<cv::Point2f> &px_cur_inlier, std::vector< DMatch > &matcher){

	//px_ref_inlier.clear();
	//px_cur_inlier.clear();
	cv::Mat F, E, mask, OutImage;
	int OutlinerCount = 0, CHECK =0;
	int InlierCount = 0;
	int ptCount_ref = (int)px_ref.size();
   	int ptCount_cur = (int)px_cur.size();
  	vector<DMatch> real_Matches;
    vector<uchar> RANSACStatus;
	cv::Mat p_ref(ptCount_ref,2,CV_32F);
	cv::Mat p_cur(ptCount_cur,2,CV_32F);

	// vector -> Mat
	for(int j=0; j< ptCount_ref ; j++)
	{
		p_ref.at<float>(j,0) = px_ref[j].x;
		p_ref.at<float>(j,1) = px_ref[j].y;
	}
	for(int j=0; j< ptCount_cur ; j++)
	{
		p_cur.at<float>(j,0) = px_cur[j].x;
		p_cur.at<float>(j,1) = px_cur[j].y;
	}

	F = findFundamentalMat(p_ref.clone(), p_cur.clone(), RANSACStatus, FM_RANSAC, 0.5, 0.99);

	for (int i=0; i<ptCount_ref; i++)
	{
		if (RANSACStatus[i] == 0)
		{
			OutlinerCount++;
		}
	}

	InlierCount = ptCount_ref - OutlinerCount; // Get inliers number

	px_ref_inlier.resize(InlierCount);
	px_cur_inlier.resize(InlierCount);
	real_Matches.resize(InlierCount);

	InlierCount = 0;
	//Mat -> vector
	for (int i=0; i < ptCount_ref; i++)
	{
		if ( RANSACStatus[i] != 0)
		{
			px_ref_inlier[InlierCount].x = p_ref.at<float>(i, 0);
			px_ref_inlier[InlierCount].y = p_ref.at<float>(i, 1);
			px_cur_inlier[InlierCount].x = p_cur.at<float>(i, 0);
			px_cur_inlier[InlierCount].y = p_cur.at<float>(i, 1);
			real_Matches[InlierCount].queryIdx = InlierCount;
			real_Matches[InlierCount].trainIdx = InlierCount;
			InlierCount++;
		}
	}

	matcher = real_Matches;

}
