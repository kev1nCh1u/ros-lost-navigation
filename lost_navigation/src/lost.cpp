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
  ros::Publisher pointPub;
  ros::Timer timer;

  // fuc
  void timerCallback(const ros::TimerEvent &);
  void mapCallback(const nav_msgs::OccupancyGrid &msg);
  void scanCallback(const sensor_msgs::LaserScan &msg);

  // var
  nav_msgs::OccupancyGrid mapMsg;
  sensor_msgs::LaserScan scanMsg;
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
  timer = n.createTimer(ros::Duration(5.0), &Lost::timerCallback, this);
  subMap = n.subscribe("map", 1000, &Lost::mapCallback, this);
  subScan = n.subscribe("scan", 1000, &Lost::scanCallback, this);
  pointPub = n.advertise<sensor_msgs::PointCloud2>("point", 0, this);
  now = ros::Time(0);
}

Lost::~Lost()
{
}

/***************************************************************************************************************************************
* timer
* ***************************************************************************************************************************************/
void Lost::timerCallback(const ros::TimerEvent &)
{
  now = ros::Time(0);

  // std::cout << "================================================" << std::endl;

  // map msg
  // std::cout << "map:" << mapMsg.info << std::endl;
  // std::cout << "map size:" << mapMsg.data.size() << std::endl;
  // std::cout << "map pos:";
  // for(int i = 0; i <  mapMsg.data.size(); i++)
  // {
  //   if(mapMsg.data[i] == 100)
  //   {
  //     std::cout << i << "/" << i % mapMsg.info.width << "/" << i / mapMsg.info.width << ", " ;
  //   }
  // }
  // std::cout << std::endl;

  // scan msg
  // std::cout << "scan size:" << scanMsg.ranges.size() << std::endl;
  sensor_msgs::PointCloud cloud;
  sensor_msgs::PointCloud2 cloud2;
  projector_.projectLaser(scanMsg, cloud);
  projector_.projectLaser(scanMsg, cloud2);

  // tf
  // listener.waitForTransform("/map", "/base_link", now, ros::Duration(1.0));
  // listener.lookupTransform("/map", "/base_link", ros::Time(0), transformVec);
  // std::cout << "tf_org xy:" << transformVec.getOrigin().x() << " " << transformVec.getOrigin().y() << std::endl;

  // pcl tf
  try
  {
    sensor_msgs::PointCloud2 base_cloud2;
    sensor_msgs::PointCloud base_cloud;
    pcl_ros::transformPointCloud("/map", cloud2, base_cloud2, listener);
    sensor_msgs::convertPointCloud2ToPointCloud(base_cloud2, base_cloud);
    std::cout << "base_link xy:" << base_cloud.points[0].x << " " << base_cloud.points[0].y << std::endl;
    pointPub.publish(base_cloud2);

    std::cout << "base_cloud pixel w:" << (base_cloud.points[0].x - mapMsg.info.origin.position.x) / mapMsg.info.resolution << std::endl;
    std::cout << "base_cloud pixel h:" << mapMsg.info.height - (base_cloud.points[0].y - mapMsg.info.origin.position.y) / mapMsg.info.resolution << std::endl;
    std::cout << "base_cloud size:" << base_cloud.points.size() << std::endl;
    int count = 0;
    for (int i = 0; i < base_cloud.points.size(); i++)
    {
      float width = (base_cloud.points[i].x - mapMsg.info.origin.position.x) / mapMsg.info.resolution;
      float height = mapMsg.info.height - (base_cloud.points[i].y - mapMsg.info.origin.position.y) / mapMsg.info.resolution;
      int data_num = width + height * mapMsg.info.width;
      if (mapMsg.data[data_num] == 100)
        count++;
    }
    float lost = float(base_cloud.points.size() - count) / base_cloud.points.size()  * 100;
    std::cout << "count:" << count << " lost:" << lost << " " << std::endl;
  }
  catch (const std::exception &e)
  {
    std::cerr << e.what() << '\n';
  }
}

/**************************************************************************************************************
 * map subscribe
 * ************************************************************************************************************/
void Lost::mapCallback(const nav_msgs::OccupancyGrid &msg)
{
  std::cout << "test" << std::endl;
  mapMsg = msg;
}

/**************************************************************************************************************
 * laser scan subscribe
 * ************************************************************************************************************/
void Lost::scanCallback(const sensor_msgs::LaserScan &msg)
{
  scanMsg = msg;
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