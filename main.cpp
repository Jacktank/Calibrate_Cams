#include <iostream>
#include <vector>
#include <sstream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

using namespace cv;

void InitCorners3D(std::vector<std::vector<cv::Point3f> >& Corners3D, CvSize ChessBoardSize, int NImages_succeed, float SquareSize)
{
	Corners3D.resize(0);
	int CurrentImage = 0;
	int CurrentRow = 0;
	int CurrentColumn = 0;
	int NPoints = ChessBoardSize.height*ChessBoardSize.width;

	// for now, assuming we're row-scanning
	for (CurrentImage = 0; CurrentImage < NImages_succeed; CurrentImage++)
	{
	    std::vector<Point3f> v;
	  	for (CurrentRow = 0; CurrentRow < ChessBoardSize.height; CurrentRow++)
	  	{
	  		for (CurrentColumn = 0; CurrentColumn < ChessBoardSize.width; CurrentColumn++)
	  		{
	  			v.push_back(cv::Point3f(float(CurrentRow*SquareSize), float(CurrentColumn*SquareSize), 0));
	  		}
	  	}
	  	Corners3D.push_back(v);
	}
}

int main()
{
		std::vector<std::string> vstring;
		vstring.push_back(std::string("rtsp://admin:12345goccia@10.0.0.104:554//Streaming/Channels/1"));
		//vstring.push_back(std::string("rtsp://admin:12345goccia@10.0.0.105:554//Streaming/Channels/1"));

		std::vector<cv::VideoCapture> vcap;
		for(int i=0;i<vstring.size();i++)
		{
				cv::VideoCapture cap(vstring[i]);
				if(!cap.isOpened())  
				{
						std::cout<<"error:fail to load camera "<<i<<std::endl;
						return -1;
				}
				vcap.push_back(cap);
		}

		std::vector<cv::Mat> vm, vm_display, vm_small;

		std::vector<std::vector<cv::Point3f> > objectPoints(1);	
		std::vector<std::vector<std::vector<cv::Point2f> > > imagePoints;
		imagePoints.resize(vcap.size());

		std::cout<<imagePoints.size()<<std::endl;

		int nx = 6;
		int ny = 9;
		int image_width = vcap[0].get(cv::CAP_PROP_FRAME_WIDTH);
		int image_height = vcap[0].get(cv::CAP_PROP_FRAME_HEIGHT);
		float SquareSize = 25.2;
		int total_per_image = nx*ny;
	  cv::Size imageSize(image_width, image_height);
		cv::Size sz(nx, ny);
		std::string	fileSave="params.xml";

		int save_count = 0;
		int total_num_image = 26;

		float ratio = 0.5;

		while(save_count < total_num_image)
		{

				vm.clear();
				vm_display.clear();
				vm_small.clear();
				for(int i=0;i<vcap.size();i++)
				{
						cv::Mat frame;
						vcap[i]>>frame;
						if(frame.empty())
						{
								std::cout<<"fail to get "<<i<<"th cam image..."<<std::endl;
								break;								
						}

						cv::Mat copy_img = frame.clone();
						vm.push_back(frame);
						vm_display.push_back(copy_img);
					
						cv::Mat tmp;
						cv::resize(frame,tmp, cv::Size(), ratio, ratio, CV_INTER_LINEAR);
						vm_small.push_back(tmp);
				}
				
				if(vm.size() != vcap.size())
					continue;

				for(int i=0;i<vm.size();i++)
				{
						std::string dst_str;
						std::stringstream dst;

						dst<<"display_"<<i;
						dst>>dst_str;
						cv::imshow(dst_str,vm_small[i]);	
				}
	
				char key = cv::waitKey(3);

				if (key == 'g')
				{

						std::cout << save_count << " pictures is processing..." <<std::endl;

						std::vector<std::vector<cv::Point2f> > pts;
						for(int i=0;i<vm.size();i++)
						{
								std::vector<cv::Point2f> corners_buf;
								
								cv::Mat gray;
								cv::cvtColor(vm[i], gray, cv::COLOR_RGB2GRAY);

								int patternfound = findChessboardCorners(gray, sz, corners_buf, CALIB_CB_ADAPTIVE_THRESH +
													CALIB_CB_NORMALIZE_IMAGE + CV_CALIB_CB_FILTER_QUADS); 
								if (!patternfound)          
								{							
										std::cout << corners_buf.size() << ":" << corners_buf.size() <<"failed!"<< std::endl;                  			
										//cv::imshow("RePlay",origin_image1);     			
										//cv::waitKey(0);
										break;
								}
								else
								{
										std::cout <<"cam:"<<i<<"corners nums:"<< corners_buf.size() <<"succeed!"<<std::endl;
										cornerSubPix(gray, corners_buf, Size(7, 7), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 50, 0.001));
										drawChessboardCorners(vm_display[i], sz, Mat(corners_buf), patternfound);
								}
							
								pts.push_back(corners_buf);
															
						}

						if(pts.size() != vm.size())
						{
								std::cout<<"-------------------------"<<std::endl;
								continue;
						}
						
						for(int i=0;i<pts.size();i++)
						{
								imagePoints[i].push_back(pts[i]);
						}

						for(int i=0;i<vm.size();i++)
						{					  
								std::stringstream dst;					      
								std::string dst_str;
								dst<<"cam_"<<i;					      
								dst>>dst_str;					      
								cv::imshow(dst_str, vm_display[i]);

								dst.clear();
								dst_str = "";

								dst<<"./cal_img/cam_"<<i<<"_frame_"<<save_count<<".jpg";
								dst>>dst_str;

								cv::imwrite(dst_str, vm[i]); 

								dst.clear();
								dst_str = "";

								dst<<"./cal_img/cam_"<<i<<"_frame_"<<save_count<<"_corners.jpg";
								dst>>dst_str;

								cv::imwrite(dst_str, vm_display[i]); 

								cv::waitKey(0);		
						}

						save_count++;

				}
				else if (key == 'q')
				{
						break;		
				}

		}
		
		std::vector<cv::Mat> Ms,Ds;
		std::vector<Mat> Rs,Ts;
	
		InitCorners3D(objectPoints, sz, save_count, SquareSize);
		
		for(int i=0;i<imagePoints.size();i++)
		{
				cv::Mat M,D,R,T;
				cv::Size imageSize_(vm[i].cols, vm[i].rows);
				std::cout<<imageSize_<<std::endl;
				cv::calibrateCamera(objectPoints, imagePoints[i], imageSize_, M, D, R, T);//CV_CALIB_RATIONAL_MODEL);//,CV_CALIB_FIX_K3);//InputArrayOfArray;

				//std::cout<<R.rows<<":"<<R.cols<<std::endl;
				//std::cout<<Rs.size()<<std::endl;
				
				Ms.push_back(M);
				Ds.push_back(D);
				Rs.push_back(R);
				Ts.push_back(T);
		}
	//	std::vector<float> reprojErrs;
	//	float totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints,
  //              rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs);

	//	std::cout<<"totalAvgErr:"<<totalAvgErr<<std::endl;
	//	for(int i=0;i<reprojErrs.size();i++)
	//	{
	//			std::cout<<"reprojErr-:"<<i<<":"<<reprojErrs[i]<<std::endl;
	//	}
		
		cv::FileStorage fs(fileSave,cv::FileStorage::WRITE|cv::FileStorage::FORMAT_XML);
		fs<<"Width"<<nx;
		fs<<"Hight"<<ny;
		fs<<"SquareSize"<<SquareSize;
		fs <<"image_width"<<imageSize.width;
		fs <<"image_height"<<imageSize.height;
		
		for(int i=0;i<imagePoints.size();i++)
		{
				std::stringstream sttr;
				sttr<<"M"<<i;
				std::string str;
				sttr>>str;
				fs<<str<<Ms[i];
				
				sttr.clear();
				str = "";
				sttr<<"D"<<i;
				sttr>>str;
				fs<<str<<Ds[i];
		}

		fs.release();

		std::cout << "calibration done..." << std::endl;

#if 0	
		std::cout << "Running stereo calibration ..." << std::endl;

		cv::Mat R, T, E, F, P1, P2, R_l, R_r, H1, H2, Q;

		cv::stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1], Ms[0], Ds[0], Ms[1], Ds[1],
								imageSize, R, T, E, F, cv::CALIB_FIX_K3 + cv::CALIB_FIX_K4 + cv::CALIB_FIX_K5,
								cv::TermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 100, 1e-5));

		//FileStorage fs("SingleCalibrate.xml", FileStorage::WRITE);
		//fs << "M1" << M1 << "D1" << D1 << "M2" << M2 << "D2" << D2;
		//fs.release();

		std::cout << "Please wait for output of calibration XML file" << std::endl;


		double avgErr = 0;
		for (int i = 0; i < total_num_image; i++)
		{
			cv::undistortPoints(imagePoints[0][i], imagePoints[0][i], Ms[0], Ds[0]);//InputArray;
			cv::undistortPoints(imagePoints[1][i], imagePoints[1][i], Ms[1], Ds[1]);//InputArray;

			std::vector<CvPoint3D32f> lines[2];
      cv::Mat L1(1, successes*total_per_image, CV_32FC3, &lines[0]);
			cv::Mat L2(1, successes*total_per_image, CV_32FC3, &lines[1]);

			cv::computeCorrespondEpilines(imagePoints[0][0], 1, F, L1);
			cv::computeCorrespondEpilines(imagePoints[1][0], 2, F, L2);

			for (int j = 0; j < total_per_image; j++)
			{
				double err = fabs(imagePoints[0][i][j].x*L1.at<CvPoint3D32f>(j, 0).x + imagePoints[0][i][j].y*L1.at<CvPoint3D32f>(j, 0).y
										+ L1.at<CvPoint3D32f>(j, 0).z) + fabs(imagePoints[1][i][j].x*L2.at<CvPoint3D32f>(j, 0).x +
										imagePoints[1][i][j].y*L2.at<CvPoint3D32f>(j, 0).y + L2.at<CvPoint3D32f>(j, 0).z);
				avgErr += err;
			}
		}
		printf("avg err = %g\n", avgErr / (total_num_image*total_per_image));

#endif

#if 0 
		cv::Rect validRoi[2];
		cv::Mat mx1, mx2, my1, my2;
	
		stereoRectify(Ms[0], Ds[0], Ms[1], Ds[1], imageSize, R, T, R_l, R_r, P1, P2, Q, CV_CALIB_ZERO_DISPARITY,
								0.8, cv::Size(vm[0].cols, vm[0].rows),	&validRoi[0], &validRoi[1]);

		std::cout << "validRoi[0].size:" << validRoi[0] <<std::endl;
		std::cout << "validRoi[1].size:" << validRoi[1] << std::endl;

		cv::FileStorage fs_stere("Stereo.xml", cv::FileStorage::WRITE);
		fs << "C0" << imageSize << "C1" << Ms[0] << "C2" << Ms[1] << "C3" << Ds[0] << "C4" << Ds[1] << "C5" << R_l << "C6" << R_r << "C7" << P1 << "C8" << P2 << "C9" << Q;
		fs.release();

		cv::initUndistortRectifyMap(Ms[0], Ds[0], R_l, P1, imageSize, CV_16SC2, mx1, my1);
		cv::initUndistortRectifyMap(Ms[1], Ds[1], R_r, P2, imageSize, CV_16SC2, mx2, my2);

		cv::Mat dst0, dst1;
		cv::remap(vm[0], dst0, mx1, my1, INTER_LINEAR);
		cv::remap(vm[1], dst1, mx1, my1, INTER_LINEAR);

		cv::imshow("origin0",vm[0]);
		cv::imshow("after0", dst0);
		cv::imshow("origin1", vm[1]);
		cv::imshow("after1", dst1);
		cv::waitKey(0);
#endif

		return 0;
}

