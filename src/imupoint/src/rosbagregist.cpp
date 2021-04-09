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
ros::Publisher imuPub;
double roll, pitch, yaw;
void cloud_Callhandle(const sensor_msgs::PointCloud2ConstPtr &ros_cloud, const sensor_msgs::ImuConstPtr &imuIn)
{
    // if ((roll > -0.02) & (roll < 0.02))
    if (1)
    {
        tf::Quaternion orientation;
        tf::quaternionMsgToTF(imuIn->orientation, orientation);
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        self_msg.roll=roll*57.3;
        self_msg.pitch=pitch*57.3;
        self_msg.yaw=yaw*57.3;
        //ROS_INFO("test_roll_pitch:%f,%f", roll*57.3, pitch*57.3);
        pcl::PointCloud<pcl::PointXYZI> laserCloudIn;
        pcl::PointCloud<pcl::PointXYZI> laserCloudTemp;
        pcl::fromROSMsg(*ros_cloud, laserCloudIn);
        // ROS_INFO("test_roll:%f,%f,%f", roll, pitch, yaw);
        //考虑所有的点云进行处理
        int count = laserCloudIn.points.size();
        pcl::PointXYZI point;
        for (int i = 0; i < count; i++)
        {
            point.x = laserCloudIn.points[i].x;
            point.y = laserCloudIn.points[i].y;
            point.z = laserCloudIn.points[i].z;
            Eigen::Vector3d point_before(point.x, point.y, point.z);
            Eigen::Matrix3d T_x;
            T_x << 1, 0, 0, 0, cos(roll), -sin(roll), 0, sin(roll), cos(roll);
            Eigen::Matrix3d T_y;
            T_y << cos(pitch), 0, sin(pitch), 0, 1, 0, -sin(pitch), 0, cos(pitch);
            //  Eigen::Matrix3d T_z;
            //  T_z<<cos(yaw),-sin(yaw),0,sin(pitch),cos(pitch),0,0,0,1;
            Eigen::Vector3d point_after = T_y * T_x * point_before;
            //ROS_INFO("before(%f,%f,%f)",point.x,point.y,point.z);
            point.x = point_after(0, 0);
            point.y = point_after(1, 0);
            point.z = point_after(2, 0);
            laserCloudTemp.push_back(point);
            // ROS_INFO("%d",i);
            //ROS_INFO("after(%f,%f,%f)",point.x,point.y,point.z);
        }
        sensor_msgs::PointCloud2 laser_deal;
        pcl::toROSMsg(laserCloudTemp, laser_deal);
        // pcl::PointCloud<pcl::PointXYZI>::Ptr laser_temp(new   pcl::PointCloud<pcl::PointXYZI>());
        // *laser_temp=laserCloudIn;
        //  sensor_msgs::PointCloud2 laser_deal;
        // pcl::toROSMsg(*laser_temp, laser_deal);
        laser_deal.header.stamp = ros_cloud->header.stamp;
        laser_deal.header.frame_id = "velodyne";
        newPub.publish(laser_deal);
        imuPub.publish(self_msg);
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
    imuPub=n.advertise<imupoint::test>("imu_new",100);
    ros::spin();
    return 0;
}