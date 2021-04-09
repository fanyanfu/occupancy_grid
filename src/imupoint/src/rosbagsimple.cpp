#include <cmath>
#include <vector>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <Eigen/Core>
#include <Eigen/Dense>
//自己写msg
#include<imupoint/test.h>
imupoint::test self_msg;
//同步器头文件
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <boost/thread/thread.hpp>

using namespace message_filters;

ros::Publisher newPub;
// ros::Publisher imuPub;
double roll, pitch, yaw;
void cloud_Callhandle(const sensor_msgs::PointCloud2ConstPtr &ros_cloud, const sensor_msgs::ImuConstPtr &imuIn)
{
        tf::Quaternion orientation;
        tf::quaternionMsgToTF(imuIn->orientation, orientation);
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        self_msg.roll=roll*57.3;
        self_msg.pitch=pitch*57.3;
        self_msg.yaw=yaw*57.3;

    if (((self_msg.roll> -2) & (self_msg.roll< 2))&((self_msg.pitch>-2)&(self_msg.pitch<2)))
    {
        pcl::PointCloud<pcl::PointXYZI> laserCloudIn;
        pcl::fromROSMsg(*ros_cloud, laserCloudIn);
        // ROS_INFO("test_roll:%f,%f", self_msg.roll, self_msg.pitch);
        //考虑所有的点云进行处理
        sensor_msgs::PointCloud2 laser_deal;
        pcl::toROSMsg(laserCloudIn, laser_deal);
        // pcl::PointCloud<pcl::PointXYZI>::Ptr laser_temp(new   pcl::PointCloud<pcl::PointXYZI>());
        // *laser_temp=laserCloudIn;
        //  sensor_msgs::PointCloud2 laser_deal;
        // pcl::toROSMsg(*laser_temp, laser_deal);
        laser_deal.header.stamp = ros_cloud->header.stamp;
        laser_deal.header.frame_id = "velodyne";
        newPub.publish(laser_deal);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imupoint");
    ros::NodeHandle n;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_point(n, "/velodyne_points", 2000, ros::TransportHints().tcpNoDelay());
    message_filters::Subscriber<sensor_msgs::Imu> sub_imu(n, "/imu/data", 2000, ros::TransportHints().tcpNoDelay());
    typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Imu> syncPolicy;
    Synchronizer<syncPolicy> sync(syncPolicy(10), sub_point, sub_imu);
    sync.registerCallback(boost::bind(&cloud_Callhandle, _1, _2));
    newPub = n.advertise<sensor_msgs::PointCloud2>("/velodyne_new", 100);
    // ros::Rate loop_rate(10);
    ros::spin();
    return 0;
}