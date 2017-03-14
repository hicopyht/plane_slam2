#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <boost/bind.hpp>

#include "../../include/System.h"

using namespace std;

typedef message_filters::Subscriber<sensor_msgs::Image> ImageSubType;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image>  RgbdSyncPolicy;

class ImageGrabber
{
public:
    ImageGrabber( plane_slam2::System* SLAM )
        : nh_()
        , private_nh_("~")
        , SLAM_(SLAM)
    {
        //
        cv::FileStorage fSettings(SLAM_->getSettingFile(), cv::FileStorage::READ);
        queue_size_ = (int)fSettings["ROS.queue_size"];
        topic_rgb_ = (string)fSettings["ROS.topic_rgb"];
        topic_depth_ = (string)fSettings["ROS.topic_depth"];
        //
        private_nh_.param<int>("queue_size", queue_size_, queue_size_);
        private_nh_.param<string>("topic_rgb", topic_rgb_, topic_rgb_);
        private_nh_.param<string>("topic_depth", topic_depth_, topic_depth_);
        //
        rgb_sub_ = new ImageSubType(nh_, topic_rgb_, 1);
        depth_sub_ = new ImageSubType(nh_, topic_depth_, 1);
        rgbd_sync_ = new message_filters::Synchronizer<RgbdSyncPolicy>(RgbdSyncPolicy(queue_size_), *rgb_sub_, *depth_sub_);
        rgbd_sync_->registerCallback(boost::bind(&ImageGrabber::grabRGBD, this, _1, _2));
        ROS_INFO_STREAM("Subscribe to topics '" << topic_rgb_ << "' and '" << topic_depth_ << "'." );
    }

    void grabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
    {
        // Copy the ros image message to cv::Mat.
        cv_bridge::CvImageConstPtr cv_ptrRGB;
        try{
            cv_ptrRGB = cv_bridge::toCvCopy(msgRGB);
        }catch (cv_bridge::Exception& e){
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        cv_bridge::CvImageConstPtr cv_ptrD;
        try{
            cv_ptrD = cv_bridge::toCvShare(msgD);
        }catch(cv_bridge::Exception& e){
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        ROS_INFO_STREAM("RGBD message: " << msgD->header.seq );
        SLAM_->trackRGBD( cv_ptrRGB->image, cv_ptrD->image );
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    //
    int queue_size_;
    std::string topic_rgb_;
    std::string topic_depth_;
    //
    ImageSubType* rgb_sub_;
    ImageSubType* depth_sub_;
    message_filters::Synchronizer<RgbdSyncPolicy>* rgbd_sync_;
    //
    plane_slam2::System* SLAM_;
};


int main(int argc, char ** argv)
{
    ros::init(argc, argv, "plane_slam_rgbd");

    std::string settingFile = "/home/lizhi/catkin_ws/src/plane_slam2/config/TUM3.yaml";

    if(argc >= 2 )
    {
        std::string ag1 = argv[1];
        if(!ag1.compare("-h"))
        {
            cout << "Usage: ./rgbd <settingFile>" << endl;
            exit(0);
        }
        else
            settingFile = argv[1];
    }

    cout << "  Setting file: " << settingFile << endl;

    plane_slam2::System *SLAM = new plane_slam2::System(settingFile);

    ImageGrabber grabber(SLAM);

    cout << " Spinning." << endl;
    ros::spin();

    ros::shutdown();

    return 0;
}
