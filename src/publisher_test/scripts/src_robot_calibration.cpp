#include <ros/ros.h>
#include <math.h>
#include <time.h>
#include <iostream>

#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <sensor_msgs/Image.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>

#define CVUI_IMPLEMENTATION
#include <cowbox_calibration/cvui.h>

bool                                bPattern_;
bool                                bRobotReach_;
bool                                bMeasure_;

trajectory_msgs::JointTrajectory    goalPose_;
ros::Publisher                      joint_pub_;

std::vector<cv::Point2f>            points;
int                                 pID_;

float                               pan_, tilt_;
float                               actuelPan_, actuelTilt_;


void robot_callback(const control_msgs::JointTrajectoryControllerStatePtr& state)
{
    if (!bPattern_) return;
    float err_sum = state->error.velocities[0] + 
                    state->error.velocities[1] + 
                    state->error.velocities[2] + 
                    state->error.velocities[3] + 
                    state->error.velocities[4] + 
                    state->error.velocities[5];
    if (err_sum != 0) bRobotReach_ = false; else bRobotReach_ = true;
    if (bRobotReach_)
    {
        actuelPan_  = state->actual.positions[0];
        actuelTilt_ = state->actual.positions[4];
    }
}

void image_callback(const sensor_msgs::ImageConstPtr& image)
{
    if (image == nullptr) return;


    cv_bridge::CvImagePtr   img;
    cv::Mat                 im;
    img = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::MONO8);
    cv::cvtColor(img->image, im, CV_BayerBG2GRAY);


    if (bPattern_)
    {
        // Laserposition
        cv::GaussianBlur(im,im,cv::Size(5,5),0);
        cv::threshold(im,im,150,255,cv::THRESH_BINARY);
        cv::Moments m = moments(im,true);
        cv::Point2f p(m.m10/m.m00, m.m01/m.m00);

        if (m.m00 == 0) return;

        cv::line(im,cv::Point(points[pID_].x,0),cv::Point(points[pID_].x,im.rows),cv::Scalar(128),1);
        cv::line(im,cv::Point(0,points[pID_].y),cv::Point(im.cols,points[pID_].y),cv::Scalar(128),1);
        cv::line(im,cv::Point(p.x,0),cv::Point(p.x,im.rows),cv::Scalar(255),1);
        cv::line(im,cv::Point(0,p.y),cv::Point(im.cols,p.y),cv::Scalar(255),1);

        // Lasersteuern
        if ((bRobotReach_)&&(bMeasure_))
        {
            float dx = p.x - points[pID_].x;
            float dy = points[pID_].y - p.y;
            pan_  += (p.x - points[pID_].x) / 25000;
            tilt_ += (points[pID_].y - p.y) / 25000;

            goalPose_.points[0].positions = { pan_, -0.5, 1.063, 0, tilt_, 0};
            goalPose_.points[0].time_from_start = ros::Duration(1.0,0.0);
            joint_pub_.publish(goalPose_);
            bRobotReach_ = false;

            if ((fabs(dx)<0.2)&&(fabs(dy)<0.2)) 
            {
                std::cout << pID_ << ": " << actuelPan_ << " " << actuelTilt_ << std::endl;
                pID_++;
            }
            if (pID_ == 48) bMeasure_ = false;
        }



        // Visualisieren
        cv::drawChessboardCorners(im, cv::Size(8, 6),points,true);
        cv::imshow("Basler", im);
        cv::waitKey(1);
        return;
    }


    // Find Circle Grid
    cv::SimpleBlobDetector::Params  params;
	params.minThreshold         = 0;
	params.maxThreshold         = 255;
	params.filterByArea         = false;
	params.filterByCircularity  = false;
	params.filterByConvexity    = false;
	params.filterByInertia      = false;
	params.filterByColor        = false;
    params.minDistBetweenBlobs  = 10;

    params.minThreshold =   0;
	params.maxThreshold =   155;
    params.filterByArea =  true;
    params.minArea      =   800;
    params.maxArea      = 10000;
    params.minDistBetweenBlobs  = 1;

    cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
    
    bool found = cv::findCirclesGrid(im, 
                                     cv::Size(8, 6), 
                                     points, 
                                     cv::CALIB_CB_SYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING, 
                                     detector);
    if (found)
    {
        std::vector<cv::KeyPoint> keypoints;
        detector->detect(im, keypoints);
        cv::drawKeypoints(im, keypoints, im, cv::Scalar(0, 0, 255), 
                          cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        cv::drawChessboardCorners(im, cv::Size(8, 6),points,found);
        bPattern_ = true;
    }
    else
    {
        std::vector<cv::KeyPoint> keypoints;
        detector->detect(im, keypoints);
        cv::drawKeypoints(im, keypoints, im, cv::Scalar(0, 0, 255), 
                          cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    }
    cv::imshow("Basler", im);
    cv::waitKey(1);
}


/**
 *
 */
 int main(int argc, char **argv)
 {
    // Initialisieren des ROS Nodes
    ros::init(argc, argv, "RobotCalibrationNode");
    ros::NodeHandle nh;

    // ROS Node loop
    ROS_INFO("Robot Calibration Node running...");
    bPattern_       = false;
    bRobotReach_    = false;
    pID_            = 0;
    bMeasure_       = true;
    joint_pub_      = nh.advertise<trajectory_msgs::JointTrajectory>("/mitsubishi_trajectory_controller/command", 1);
    ros::Subscriber camera_sub = nh.subscribe("/pylon_camera_node/image_raw", 1, &image_callback);
    ros::Subscriber robot_sub  = nh.subscribe("/mitsubishi_trajectory_controller/state", 1, &robot_callback);

    ros::spinOnce();
    ros::Duration(1).sleep();

    pan_  = 0.0;
    tilt_ = 1.25;
    goalPose_.header.frame_id = "base_link";
    goalPose_.joint_names.resize(6);
    goalPose_.points.resize(1);
    goalPose_.joint_names = { "j1", "j2", "j3", "j4", "j5", "j6" };
    goalPose_.points[0].positions.resize(6);
    goalPose_.points[0].positions = { pan_, -0.5, 1.063, 0, tilt_, 0};
    goalPose_.points[0].time_from_start = ros::Duration(5.0,0.0);
    joint_pub_.publish(goalPose_);



    ros::spin();
    return 0;
 }