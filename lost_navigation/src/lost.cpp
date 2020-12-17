#include "ros/ros.h"
#include "std_msgs/String.h"

#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/LaserScan.h"
#include <sensor_msgs/point_cloud_conversion.h>

#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

/**************************************************************************************************************
 * lost class
 * ************************************************************************************************************/
class Lost
{
private:
  // ros
  ros::NodeHandle n;
  ros::Time now;
  ros::Subscriber subMap;
  ros::Subscriber subScan;

  // fuc
  void mapCallback(const nav_msgs::OccupancyGrid &msg);
  void scanCallback(const sensor_msgs::LaserScan &scanMsg);

  // var
  nav_msgs::OccupancyGrid mapMsg;
  laser_geometry::LaserProjection projector_;
  tf::TransformListener listener;
  tf::StampedTransform transformVec;

public:
  Lost(/* args */);
  ~Lost();
};

/**************************************************************************************************************
 * Lost::Lost
 * ************************************************************************************************************/
Lost::Lost(/* args */)
{
  subMap = n.subscribe("map", 1000, &Lost::mapCallback, this);
  subScan = n.subscribe("scan", 1000, &Lost::scanCallback, this);
  now = ros::Time(0);
}

Lost::~Lost()
{
}

/**************************************************************************************************************
 * map subscribe
 * ************************************************************************************************************/
void Lost::mapCallback(const nav_msgs::OccupancyGrid &msg)
{
  mapMsg = msg;
}

/**************************************************************************************************************
 * laser scan subscribe
 * ************************************************************************************************************/
void Lost::scanCallback(const sensor_msgs::LaserScan &scanMsg)
{
  std::cout << "================================================" << std::endl;
  std::cout << "map:" << mapMsg.info << std::endl;
  std::cout << "map size:" << mapMsg.data.size() << std::endl;
  std::cout << "map width height :" << mapMsg.info.width << mapMsg.info.height << std::endl;

  // std::cout << "map pos:";
  // for(int i = 0; i <  mapMsg.data.size(); i++)
  // {
  //   if(mapMsg.data[i] == 100)
  //   {
  //     std::cout << i << "/" << i % mapMsg.info.width << "/" << i / mapMsg.info.width << ", " ;
  //   }
  // }
  // std::cout << std::endl;

  std::cout << "scan size:" << scanMsg.ranges.size() << std::endl;

  sensor_msgs::PointCloud cloud;
  sensor_msgs::PointCloud2 cloud2;
  projector_.projectLaser(scanMsg, cloud);
  projector_.projectLaser(scanMsg, cloud2);
  std::cout << "cloud size:" << cloud.points.size() << std::endl;
  std::cout << "cloud xy:" << cloud.points[0].x << " " << cloud.points[0].y << std::endl;
  std::cout << "cloud2:" << uint(cloud2.data[0]) << " " << uint(cloud2.data[1]) << " " << uint(cloud2.data[2]) << " " << uint(cloud2.data[3]) << std::endl;

  listener.waitForTransform("/map", "/base_link", now, ros::Duration(1.0));
  listener.lookupTransform("/map", "/base_link", ros::Time(0), transformVec);
  std::cout << "tf_org xy:" << transformVec.getOrigin().x() << " " << transformVec.getOrigin().y() << std::endl;

  sensor_msgs::PointCloud2 base_cloud2;
  pcl_ros::transformPointCloud("/map", cloud2, base_cloud2, listener);
  sensor_msgs::PointCloud base_cloud;
  sensor_msgs::convertPointCloud2ToPointCloud(base_cloud2, base_cloud);
  std::cout << "base_link xy:" << base_cloud.points[0].x << " " << base_cloud.points[0].y << std::endl;

  // for(int i = 0; i < cloud.points.size(); i ++)
  // {
  //   cloud.points[0].x / mapMsg.info.resolution;

  //   geometry_msgs::PointStamped base_point;
  //   listener.transformPoint("base_link", cloud.points, base_point);

  // }
  // std::cout << "cloud xy:" << cloud.points[0].x << " " << cloud.points[0].y << std::endl;
}

/**************************************************************************************************************
 * main
 * ************************************************************************************************************/
int main(int argc, char **argv)
{

  ros::init(argc, argv, "lost");

  Lost lost;

  ros::spin();

  return 0;
}