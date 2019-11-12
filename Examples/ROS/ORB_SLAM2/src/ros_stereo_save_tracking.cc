#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"

//add by hr 190427
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <geometry_msgs/Pose.h>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <nav_msgs/Odometry.h>

#include"../../../include/CoTrans.h"


using namespace std;

cv::Mat img_left,img_right,hr_r;

//geometry_msgs::Pose P;
bool use_gps = false;
bool if_write_pose,InitGPS_from_yaml;
double gps_origin_x,gps_origin_y,gps_origin_z;
cv::Mat c2g_R,c2g_t;
string cam_path,gps_path,gps_init_path;
//pcl::PointCloud<pcl::PointXYZ> cloud;
//ros::Publisher pcl_pub;
//int cur_id = 0;
CCoordinate trans_gps;
bool trans_gps_init=false;

void gps_callback(const nav_msgs::Odometry::ConstPtr& gpsinfo,ros::Publisher pub_cur_pose);
class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight, ros::Publisher pub_cur_pose,ros::Publisher pub_trans_cam_pose);
    //void GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight, ros::Publisher pub_cur_pose,ros::Publisher pcl_pub);
    //void GrabStereo(const sensor_msgs::CompressedImageConstPtr& msgLeft,const sensor_msgs::CompressedImageConstPtr& msgRight);

    ORB_SLAM2::System* mpSLAM;
    bool do_rectify;
    cv::Mat M1l,M2l,M1r,M2r;
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc != 4)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Stereo path_to_vocabulary path_to_settings do_rectify" << endl;
        ros::shutdown();
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,true);

    ImageGrabber igb(&SLAM);

    stringstream ss(argv[3]);
    ss >> boolalpha >> igb.do_rectify;

    if(igb.do_rectify)
    {
        // Load settings related to stereo calibration
        cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
        if(!fsSettings.isOpened())
        {
            cerr << "ERROR: Wrong path to settings" << endl;
            return -1;
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        //**************************add by hr 190524
        fsSettings["GPS.x"] >> gps_origin_x;
        fsSettings["GPS.y"] >> gps_origin_y;
        fsSettings["GPS.z"] >> gps_origin_z;
        fsSettings["Cam2GPS.R"] >> c2g_R;
        fsSettings["Cam2GPS.t"] >> c2g_t;
        fsSettings["Write_pose"] >> if_write_pose;
        fsSettings["InitGPS_from_yaml"] >> InitGPS_from_yaml;
        fsSettings["cam_path_save"] >> cam_path;
        fsSettings["gps_path_save"] >> gps_path;
        fsSettings["gps_init_save"] >> gps_init_path;
        //**************************

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int                        rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
           rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            return -1;
        }

        cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,igb.M1l,igb.M2l);
        cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,igb.M1r,igb.M2r);
    }
    cout<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! : "<<if_write_pose<<" "<<InitGPS_from_yaml<<endl;


    ros::NodeHandle nh;

    //**********************************
    ros::Publisher pub_cur_pose = nh.advertise<geometry_msgs::Pose>("cur_pose",1);
    ros::Publisher pub_trans_cam_pose = nh.advertise<geometry_msgs::Pose>("trans_cam_pose",1);//hr: 仅做测试用

    ros::Subscriber gps_recv = nh.subscribe<nav_msgs::Odometry>("/GPSInfo/gps_info",1,boost::bind(gps_callback,_1,pub_cur_pose));//hr: 注意这个用法，给回调函数传参数；占位符的使用

    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "/mynteye/left/image_raw",1 );
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "/mynteye/right/image_raw",1 );
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;

    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub,right_sub);
    //sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo,&igb,_1,_2));
    sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo,&igb,_1,_2,pub_cur_pose,pub_trans_cam_pose));

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();


    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryTUM("FrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryKITTI("FrameTrajectory_KITTI_Format.txt");

    char IsSaveMap;
    cout << "Do you want to save the map?(y/n)" << endl;
    cin >> IsSaveMap;
    if(IsSaveMap == 'Y' || IsSaveMap == 'y')
        SLAM.SaveMap("MapPointandKeyFrame.bin");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight, ros::Publisher pub_cur_pose,ros::Publisher pub_trans_cam_pose) {
    //ros::Time
    double time_cam = msgLeft->header.stamp.toSec();
    cv_bridge::CvImageConstPtr cv_ptrLeft;
    try {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
        //img_left = cv::imdecode(cv::Mat(msgLeft->data), 1);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv_bridge::CvImageConstPtr cv_ptrRight;
    try {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
        //img_right = cv::imdecode(cv::Mat(msgRight->data), 1);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if (do_rectify) {
        cv::Mat imLeft, imRight;
        cv::remap(cv_ptrLeft->image, imLeft, M1l, M2l, cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image, imRight, M1r, M2r, cv::INTER_LINEAR);
        //cv::remap(img_left,imLeft,M1l,M2l,cv::INTER_LINEAR);
        //cv::remap(img_right,imRight,M1r,M2r,cv::INTER_LINEAR);

        //************************************ hr: 彩色图像直方图均衡这么做，这段要看情况加入，若画面不同区域亮度差距太大(如天空和地面)，则加入
        /*
        cv::Mat left_imgRGB[3], right_imgRGB[3];
        cv::split(imLeft, left_imgRGB);
        cv::split(imRight, right_imgRGB);
        for(int i=0; i<3; i++)
        {
            cv::equalizeHist(left_imgRGB[i], left_imgRGB[i]);
            cv::equalizeHist(right_imgRGB[i],right_imgRGB[i]);
        }
        cv::merge(left_imgRGB,3,imLeft);
        cv::merge(right_imgRGB,3,imRight);
         */
        //***********************************
        cv::equalizeHist(imLeft, imLeft);//黑白图像
        cv::equalizeHist(imRight, imRight);

        mpSLAM->TrackStereo(imLeft, imRight, msgLeft->header.stamp.toSec());

        //add by hr 190504
        //**********************************************

        if (!get_cur_position_pub().empty()) {
            Eigen::Matrix3d rotation_mat;
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    //rotation_mat(i, j) = ORB_SLAM2::Tracking::pso_for_pub.at<float>(i,j);
                    rotation_mat(i, j) = get_cur_position_pub().at<float>(i, j);
            Eigen::Quaterniond q = Eigen::Quaterniond(rotation_mat);
            float cam_raw[3]={0.0,0.0,0.0};
            float cam[3]={0.0,0.0,0.0};
            geometry_msgs::Pose P;
            P.orientation.w = q.w();
            P.orientation.x = q.x();
            P.orientation.y = q.y();
            P.orientation.z = q.z();
            cam_raw[0] = get_cur_position_pub().at<float>(0, 3);
            cam_raw[2] = get_cur_position_pub().at<float>(1, 3);
            cam_raw[1] = get_cur_position_pub().at<float>(2, 3);//hr:顺序

            for(int a=0;a<3;a++)
            {
                float cam_ = 0.0;
                for(int b=0;b<3;b++)
                {
                    cam_ = cam_ + c2g_R.at<double>(a,b)*cam_raw[b];
                }
                cam[a] = cam_ + c2g_t.at<double>(a,0);
            }

            //cv::Mat p2 = (cv::Mat_<float>(3, 1) << x, y, 0.0);
            //cv::Mat p2;
            //p2 = c2g_R * p1 + c2g_t;

            //}
            if (if_write_pose) {
                ofstream output;
                output.setf(ios::fixed, ios::floatfield);
                output.precision(9);
                output.open(cam_path.c_str(), ios::app);
                //output<<time_cam<<" "<<P.position.x<<" "<<P.position.y<<" "<<P.position.z<<endl;
                output << time_cam << " " << cam_raw[0]<< " " << " " << cam_raw[1] << " " << 0.00 << endl;
                output.close();
            }
            P.position.x = cam[0];
            P.position.y = 0;
            P.position.z = cam[1];
            if (use_gps == false) {
                pub_cur_pose.publish(P);
            }
            pub_trans_cam_pose.publish(P);


        }

        //*********************************************
    } else {
        cv::Mat imLeft, imRight;
        /*
        cv::Mat left_imgRGB[3], right_imgRGB[3];
        cv::split(imLeft, left_imgRGB);
        cv::split(imRight, right_imgRGB);
        for(int i=0; i<3; i++)
        {
            cv::equalizeHist(left_imgRGB[i], left_imgRGB[i]);
            cv::equalizeHist(right_imgRGB[i],right_imgRGB[i]);
        }
        cv::merge(left_imgRGB,3,imLeft);
        cv::merge(right_imgRGB,3,img_right);
         */
        cv::equalizeHist(imLeft, imLeft);//黑白图像
        cv::equalizeHist(imRight, imRight);

        //mpSLAM->TrackStereo(imLeft,imRight,msgLeft->header.stamp.toSec());
        mpSLAM->TrackStereo(cv_ptrLeft->image, cv_ptrRight->image, cv_ptrLeft->header.stamp.toSec());
        //mpSLAM->TrackStereo(img_left,img_right,msgLeft->header.stamp.toSec());


        if (!get_cur_position_pub().empty()) {
            Eigen::Matrix3d rotation_mat;
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    //rotation_mat(i, j) = ORB_SLAM2::Tracking::pso_for_pub.at<float>(i,j);
                    rotation_mat(i, j) = get_cur_position_pub().at<float>(i, j);
            Eigen::Quaterniond q = Eigen::Quaterniond(rotation_mat);
            float cam_raw[3]={0.0,0.0,0.0};
            float cam[3]={0.0,0.0,0.0};
            geometry_msgs::Pose P;
            P.orientation.w = q.w();
            P.orientation.x = q.x();
            P.orientation.y = q.y();
            P.orientation.z = q.z();
            cam_raw[0] = get_cur_position_pub().at<float>(0, 3);
            cam_raw[2] = get_cur_position_pub().at<float>(1, 3);
            cam_raw[1] = get_cur_position_pub().at<float>(2, 3);//hr:顺序

            for(int a=0;a<3;a++)
            {
                float cam_ = 0.0;
                for(int b=0;b<3;b++)
                {
                    cam_ = cam_ + c2g_R.at<double>(a,b)*cam_raw[b];
                }
                cam[a] = cam_ + c2g_t.at<double>(a,0);
            }

            //cv::Mat p2 = (cv::Mat_<float>(3, 1) << x, y, 0.0);
            //cv::Mat p2;
            //p2 = c2g_R * p1 + c2g_t;

            //}
            if (if_write_pose) {
                ofstream output;
                output.setf(ios::fixed, ios::floatfield);
                output.precision(9);
                output.open(cam_path.c_str(), ios::app);
                //output<<time_cam<<" "<<P.position.x<<" "<<P.position.y<<" "<<P.position.z<<endl;
                output << time_cam << " " << cam_raw[0]<< " " << " " << cam_raw[1] << " " << 0.00 << endl;
                output.close();
            }
            P.position.x = cam[0];
            P.position.y = 0;
            P.position.z = cam[1];
            if (use_gps == false) {
                pub_cur_pose.publish(P);
            }
            pub_trans_cam_pose.publish(P);

        }

    }

}


//hr: 要特别注意这个ConstPtr的用法
void gps_callback(const nav_msgs::Odometry::ConstPtr& gpsinfo,ros::Publisher pub_cur_pose)
{
    if(!trans_gps_init) //hr: 此处初始化GPS信息
    {
        if (InitGPS_from_yaml)
        {
            trans_gps.InitRadarPara(gps_origin_z,gps_origin_x,gps_origin_y);
            trans_gps_init = true;
        }
        else if (gpsinfo->pose.pose.orientation.x == 4)
        {
            trans_gps.InitRadarPara(gpsinfo->pose.pose.position.z, gpsinfo->pose.pose.position.x, gpsinfo->pose.pose.position.y);
            trans_gps_init = true;
            ofstream output;
            output.setf(ios::fixed, ios::floatfield);
            output.precision(9);
            output.open(gps_init_path.c_str(), ios::app);
            output<<gpsinfo->pose.pose.position.x<<" "<<gpsinfo->pose.pose.position.y<<" "<<gpsinfo->pose.pose.position.z<<endl;
            output.close();
        }
    }
    else {
        geometry_msgs::Pose P;
        //P.position.y = gpsinfo->pose.pose.position.z - gps_origin_z;
        //P.position.x = gpsinfo->pose.pose.position.x - gps_origin_x;
        //P.position.z = gpsinfo->pose.pose.position.y - gps_origin_y;
        XYPOINT gps_xy = trans_gps.LongLat2XY(gpsinfo->pose.pose.position.x,gpsinfo->pose.pose.position.y,gpsinfo->pose.pose.position.z);
        P.position.x = gps_xy.x;
        P.position.y = 0;
        P.position.z =  gps_xy.y;

        P.orientation.x = 4;
        P.orientation.y = 0;
        P.orientation.z = 0;
        P.orientation.w = 0;

        //ros::Time
        double time_gps = gpsinfo->header.stamp.toSec();//hr: 注意坐标轴写入顺序对不对
        if (if_write_pose) {
            ofstream output;
            output.setf(ios::fixed, ios::floatfield);
            output.precision(9);
            output.open(gps_path.c_str(), ios::app);
            output << time_gps << " " << gps_xy.x << " " << gps_xy.y << " " << 0.0 << endl;
            output.close();
        }


        if (gpsinfo->pose.pose.orientation.x == 4)
            use_gps = true;
        else
            use_gps = false;

        //怎么得到一个 正确的 相对位姿 ？
        if (use_gps == true) {
            pub_cur_pose.publish(P);
        }
        cout << "能收到GPS信号！！！！！！！！！！！！！！！！！！！！" << endl;
    }
}



