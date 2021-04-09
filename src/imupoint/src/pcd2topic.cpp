#include<string>
#include<ros/ros.h>
#include<cmath>
#include<pcl/point_cloud.h>
#include <pcl/point_types.h>
#include<pcl_conversions/pcl_conversions.h>
#include<sensor_msgs/PointCloud2.h>
#include<pcl/io/pcd_io.h>
// typedef pcl::pointXYZRGB PointType;
typedef pcl::PointXYZRGB PointType;

int main(int argc,char** argv)
{
    ros::init(argc,argv,"pcd2topic");
    ros::NodeHandle nh;
    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("cloud_map", 1);
    ros::Publisher pcl_pub_old= nh.advertise<sensor_msgs::PointCloud2> ("cloud_map_old", 1);

    pcl::PointCloud<pcl::PointXYZRGB> cloud,cloud1;
    sensor_msgs::PointCloud2 output,output_old;
    // pcl::io::loadPCDFile("/home/vision/ros_workspace/catkin_new/src/imupoint/cloud_map/cloudGlobal.pcd",cloud);
    pcl::io::loadPCDFile("/home/vision/ros_workspace/catkin_new/src/imupoint/cloud_map/floor_newbig.pcd",cloud);

    ROS_INFO("old cloud count %d",cloud.size());
    // cloud1.push_back(cloud.points[0]);
    //去除噪声点
    for(int i=0;i<cloud.size();i++)
    {
        PointType point;
        int dist;
        point.x=cloud.points[i].x;
        point.y=cloud.points[i].y;
        point.z=cloud.points[i].z;

        dist=sqrt(point.x*point.x+point.y*point.y+point.z*point.z);
        if(dist>15) //12太大 10比较合适
        {
            continue;
        }
        cloud1.push_back(point);
    }

    ROS_INFO("new cloud count %d",cloud1.size());

    pcl::toROSMsg(cloud,output_old);
     output_old.header.frame_id="map";

    pcl::toROSMsg(cloud1,output);
    output.header.frame_id="map";
    ros::Rate loop_rate(1);
    while(ros::ok())
    {
        pcl_pub.publish(output);
        pcl_pub_old.publish(output_old);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;

}