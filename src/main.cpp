#include <include_all.h>


int main(int argc, char **argv)
{
	/*int sock0;
	struct sockaddr_in addr;
	struct sockaddr_in client;
	socklen_t len;
	int sock_client;

	/// 製作 socket
	sock0 = socket(AF_INET, SOCK_STREAM, 0);

	/// 設定 socket
	addr.sin_family = AF_INET;
	addr.sin_port = htons(12345);
	addr.sin_addr.s_addr = INADDR_ANY;
	bind(sock0, (struct sockaddr*)&addr, sizeof(addr));
	printf("\t[Info] binding...\n");

	/// 設定成等待來自 TCP 用戶端的連線要求狀態
	listen(sock0,12);
	printf("\t[Info] listening...\n");

	/// 接受來自 TCP 用戶端地連線要求
	printf("\t[Info] wait for connection...\n");
	len = sizeof(client);
	sock_client = accept(sock0, (struct sockaddr *)&client, &len);
	printf("\t[Info] Testing...\n");
	char *paddr_str = inet_ntoa(client.sin_addr);
	printf("\t[Info] Receive connection from %s...\n", paddr_str);*/

	// Trajectory file
	std::ofstream out("position.txt");

	// Display
	/*char text[100], text2[100];
	int font_face = cv::FONT_HERSHEY_PLAIN, font_face2 = cv::FONT_HERSHEY_COMPLEX_SMALL;
	double font_scale = 1, font_scale2 = 1;
	int thickness = 1, thickness2 = 2;
	cv::Point text_org(10, 50), text_org2(10, 80);
	cv::namedWindow("Trajectory", cv::WINDOW_AUTOSIZE);
	cv::Mat traj = cv::Mat::zeros(600, 600, CV_8UC3);*/

    // Create a ZED camera object
    Camera zed;

    // Set configuration parameters
    InitParameters init_params;
    if (argc > 1) init_params.svo_input_filename.set(argv[1]);
    init_params.camera_resolution = RESOLUTION_VGA;
    init_params.depth_stabilization = true;
    init_params.depth_mode = DEPTH_MODE_PERFORMANCE;
    init_params.coordinate_units = UNIT_METER;

    // Open the camera
    ERROR_CODE err = zed.open(init_params);
    if (err != SUCCESS) {
        //printf("%s\n", toString(err).c_str());
        zed.close();
        return 1; // Quit if an error occurred
    }

    // Set runtime parameters after opening the camera
    RuntimeParameters runtime_parameters;
    runtime_parameters.sensing_mode = SENSING_MODE_FILL;

    // Get camera parameters
	//printf("ZED Model                 : %s\n", toString(zed.getCameraInformation().camera_model).c_str());
	//printf("ZED Serial Number         : %d\n", zed.getCameraInformation().serial_number);
	//printf("ZED Firmware              : %d\n", zed.getCameraInformation().firmware_version);
	//printf("ZED Camera Resolution     : %dx%d\n", (int) zed.getResolution().width, (int) zed.getResolution().height);
	//printf("ZED Camera FPS            : %d\n", (int) zed.getCameraFPS());

	int fps = (int) zed.getCameraFPS();

    // Prepare new image size to retrieve half-resolution images
    Resolution image_size_sdk = zed.getResolution();
    int new_width = image_size_sdk.width;
    int new_height = image_size_sdk.height;

    sl::Mat image_L(new_width, new_height, MAT_TYPE_8U_C4);
    cv::Mat image_ocv_L = slMat2cvMat(image_L);

    sl::Mat image_R(new_width, new_height, MAT_TYPE_8U_C4);
    cv::Mat image_ocv_R = slMat2cvMat(image_R);

    sl::Mat point_cloud_l(zed.getResolution(), MAT_TYPE_32F_C4);

    cv::Mat left_rect, left_u8, img_cur_L, right_rect, right_u8, img_cur_R;

	double x=0.0, y=0.0,z=0.0, roll = 0.0, pitch=0.0, yaw=0.0;
	double vx = 0.0, vy = 0.0, vz = 0.0, pre_x = 0.0, pre_y = 0.0, pre_z = 0.0;

	float cx = zed.getCameraInformation().calibration_parameters.left_cam.cx;
	float cy = zed.getCameraInformation().calibration_parameters.left_cam.cy;
	float fx = zed.getCameraInformation().calibration_parameters.left_cam.fx;
	float fy = zed.getCameraInformation().calibration_parameters.left_cam.fy;

	PinholeCamera *cam = new PinholeCamera(new_width, new_height, fx, fy, cx, cy);

	VisualOdometry vo(cam);

    struct Packet{
    	int cam_x;
    	int cam_y;
    	int cam_z;
    };

	for (int img_id = 0; !AbortRequested(); ++img_id)
	{
		if (zed.grab(runtime_parameters) == SUCCESS) {

			// read images
			zed.retrieveImage(image_L, VIEW_LEFT, MEM_CPU, new_width, new_height);
			zed.retrieveImage(image_R, VIEW_RIGHT, MEM_CPU, new_width, new_height);
			zed.retrieveMeasure(point_cloud_l, MEASURE_XYZRGBA, MEM_CPU, new_width, new_height);

			left_rect = image_ocv_L.clone();
			cv::cvtColor(left_rect.clone(), left_u8, cv::COLOR_BGR2GRAY);
			GaussianBlur(left_u8.clone(), img_cur_L, Size(5,5) ,0 ,0);

			right_rect = image_ocv_R.clone();
			cv::cvtColor(right_rect.clone(), right_u8, cv::COLOR_BGR2GRAY);
			GaussianBlur(right_u8.clone(), img_cur_R, Size(5,5) ,0 ,0);

			auto start = high_resolution_clock::now();

			vo.addImage(img_cur_L, img_cur_R);

			auto stop = high_resolution_clock::now();
			auto duration = duration_cast<milliseconds>(stop - start);
			//cout << "Velocity estimation takes: " << duration.count() << " ms" << endl << endl;

			cv::Mat cur_t = (vo.getCurrentT()).clone();
			cv::Mat cur_R = (vo.getCurrentR()).clone();

        	Eigen::Matrix3d r = Eigen::MatrixXd::Identity(3,3);
            Eigen::Vector3d rpy(0,0,0);

			if (cur_t.rows!=0)
			{
				x = (cur_t.clone()).at<double>(0);
				y = (cur_t.clone()).at<double>(1);
				z = (cur_t.clone()).at<double>(2);

				vx = (x- pre_x)*fps;
				vy = (y- pre_y)*fps;
				vz = (z- pre_z)*fps;

	        	cv::cv2eigen(cur_R.clone(), r);
            	rpy = r.eulerAngles(2,0,1);

				roll = rpy(0)*180/M_PI;
				pitch = rpy(1)*180/M_PI;
				yaw = rpy(2)*180/M_PI;

				pre_x = x; pre_y = y; pre_z = z;
			}

			out << x << " " << y << " " << z << " " << vx << " " << vy << " " << vz << "  " << 1000/duration.count() << endl;
			cout << x << "\t" << y << "\t" << z << "\t" << vx << "\t" << vy << "\t" << vz << "\t" << 1000/duration.count() << endl;
            //Packet *s = new Packet();
            //s->cam_x = htonl(x*1000);
            //s->cam_y = htonl(y*1000);
            //s->cam_z = htonl(z*1000);

			// Send x,y,z data to GCS
			//printf("\t[Info] Send camera xyz to Intel AERO...\n");
            //write (sock_client, s, sizeof(*s));

        	/*if ( img_id%5 == 0 )
        	{

        		int draw_x = int(x)+ 300; // x original points on display image
        		int draw_y = int(z)+ 110; // y original points on display image
        		cv::circle(traj, cv::Point(draw_x, draw_y), 1, CV_RGB(255, 0, 0), 2); //draw trajectory from top view
        		cv::rectangle(traj, cv::Point(10, 30), cv::Point(580, 90), CV_RGB(0, 0, 0), CV_FILLED);
        		sprintf(text, "Coordinates: x = %02fm y = %02fm z = %02fm", x, y, z);
        		cv::putText(traj, text, text_org, font_face, font_scale, cv::Scalar::all(255), thickness, 8);
        		sprintf(text2, "Coordinates: roll = %5.3fdeg pitch = %5.3fdeg yaw = %5.3fdeg", roll, pitch, yaw);
        		cv::putText(traj, text2, text_org2, font_face, font_scale, cv::Scalar::all(255), thickness, 8);
        		cv::imshow("Trajectory", traj);
        	}*/

		}

		if (AbortRequested()) break;
	}

	//1021delete cam;
	//1021out.close();

	/* 結束 TCP 對話
	printf("\t[Info] Close client connection...\n");
	close(sock_client);

	// 結束 listen 的 socket
	printf("\t[Info] Close self connection...\n");
	close(sock0);*/
	return 0;
}

cv::Mat slMat2cvMat(sl::Mat& input) {
    // Mapping between MAT_TYPE and CV_TYPE
    int cv_type = -1;
    switch (input.getDataType()) {
        case MAT_TYPE_32F_C1: cv_type = CV_32FC1; break;
        case MAT_TYPE_32F_C2: cv_type = CV_32FC2; break;
        case MAT_TYPE_32F_C3: cv_type = CV_32FC3; break;
        case MAT_TYPE_32F_C4: cv_type = CV_32FC4; break;
        case MAT_TYPE_8U_C1: cv_type = CV_8UC1; break;
        case MAT_TYPE_8U_C2: cv_type = CV_8UC2; break;
        case MAT_TYPE_8U_C3: cv_type = CV_8UC3; break;
        case MAT_TYPE_8U_C4: cv_type = CV_8UC4; break;
        default: break;
    }

    // Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
    // cv::Mat and sl::Mat will share a single memory structure
    return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(MEM_CPU));
}

bool AbortRequested()
{
    char  key = waitKey( DEFAULT_WAITKEY_DELAY );
    if( key == ' ' )
    {
        key = waitKey( 0 );
    }
    if(( key == 'q' ) || ( key == 'Q' ) || ( key == 27 ) /*ESC*/ )
    {
        return true;
    }
    return false;
}
