/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez,
*   José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós,
*   University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the
* terms of the GNU General Public License as published by the Free Software Foundation,
* either version 3 of the License, or (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
* without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
* See the GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <Eigen/Core>
#include <opencv2/core/core.hpp>

#include "../../../include/System.h"

using namespace std;

class MonoSlamNode : public rclcpp::Node
{
public:
    MonoSlamNode(const string &vocab_path, const string &settings_path)
        : Node("mono_slam"), mpSLAM(nullptr), count_(0),
          cameraWidth_(0), cameraHeight_(0)
    {
        // Create SLAM system. It initializes all system threads and gets ready
        // to process frames.
        RCLCPP_INFO(this->get_logger(), "Started SLAM system setup");
        mpSLAM = new ORB_SLAM3::System(
            vocab_path, settings_path, ORB_SLAM3::System::MONOCULAR, false);
        RCLCPP_INFO(this->get_logger(), "Finished SLAM system setup");

        // Load camera parameters from settings file
        cv::FileStorage fSettings(settings_path, cv::FileStorage::READ);
        cameraWidth_ = fSettings["Camera.width"].operator int();
        cameraHeight_ = fSettings["Camera.height"].operator int();

        // Create tf2 broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Create image subscription
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera_0", 1,
            std::bind(&MonoSlamNode::GrabImage, this, std::placeholders::_1));
    }

    ~MonoSlamNode()
    {
        if (mpSLAM)
        {
            // Stop all threads
            mpSLAM->Shutdown();

            // Save camera trajectory
            mpSLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

            delete mpSLAM;
            mpSLAM = nullptr;
        }
    }

private:
    void GrabImage(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
    {
        RCLCPP_INFO(this->get_logger(), "A message is received");

        // Copy the ros image message to cv::Mat.
        cv_bridge::CvImageConstPtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvShare(msg);
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat img;

        if (cv_ptr->image.cols != cameraWidth_ ||
            cv_ptr->image.rows != cameraHeight_)
        {
            RCLCPP_INFO(this->get_logger(),
                        "Resize image from (%d, %d) to (%d, %d).",
                        cv_ptr->image.cols, cv_ptr->image.rows,
                        cameraWidth_, cameraHeight_);
            cv::resize(cv_ptr->image, img,
                        cv::Size(cameraWidth_, cameraHeight_), cv::INTER_LINEAR);
        }
        else
        {
            img = cv_ptr->image;
        }

        // Track using this image
        Sophus::SE3f se3_tf = mpSLAM->TrackMonocular(
            img, rclcpp::Time(msg->header.stamp).seconds());

        cout << se3_tf.rotationMatrix() << endl;
        cout << se3_tf.translation() << endl;

        string frame_id("map");
        string child_frame_id("camera_");
        child_frame_id = child_frame_id + to_string(count_);
        count_ += 1;

        PublishPose(se3_tf, msg->header.stamp, frame_id, child_frame_id);
    }

    void PublishPose(
        const Sophus::SE3f &T,
        const builtin_interfaces::msg::Time &stamp,
        const string &frame_id,
        const string &child_frame_id)
    {
        geometry_msgs::msg::TransformStamped transform_stamped;

        transform_stamped.header.stamp = stamp;
        transform_stamped.header.frame_id = frame_id;
        transform_stamped.child_frame_id = child_frame_id;

        Eigen::Quaternionf q(T.rotationMatrix());

        transform_stamped.transform.translation.x = T.translation().x();
        transform_stamped.transform.translation.y = T.translation().y();
        transform_stamped.transform.translation.z = T.translation().z();

        transform_stamped.transform.rotation.x = q.x();
        transform_stamped.transform.rotation.y = q.y();
        transform_stamped.transform.rotation.z = q.z();
        transform_stamped.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(transform_stamped);
    }

    ORB_SLAM3::System *mpSLAM;
    int count_;
    int cameraWidth_;
    int cameraHeight_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    if (argc < 3)
    {
        cerr << endl
             << "Usage: ros2 run orb_slam3_ros2 Mono path_to_vocabulary path_to_settings"
             << endl;
        rclcpp::shutdown();
        return 1;
    }

    auto node = std::make_shared<MonoSlamNode>(string(argv[1]), string(argv[2]));

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
