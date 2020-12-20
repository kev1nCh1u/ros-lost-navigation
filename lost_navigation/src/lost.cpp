#include "ros/ros.h"
#include "std_msgs/String.h"

#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/LaserScan.h"
#include <sensor_msgs/point_cloud_conversion.h>
#include <nav_msgs/Odometry.h>

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
  ros::Timer timer;
  ros::Subscriber subMap;
  ros::Subscriber subScan;
  ros::Subscriber subOdom;
  // ros::Publisher pointPub;

  // fuc
  void timerCallback(const ros::TimerEvent &);
  void mapCallback(const nav_msgs::OccupancyGrid &msg);
  void scanCallback(const sensor_msgs::LaserScan &msg);
  void odomCallback(const nav_msgs::Odometry &msg);
  void lostCalc();

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
  // timer = n.createTimer(ros::Duration(1.0), &Lost::timerCallback, this);
  subMap = n.subscribe("map", 1000, &Lost::mapCallback, this);
  subScan = n.subscribe("scan", 10000, &Lost::scanCallback, this);
  // subOdom = n.subscribe("odom", 1000, &Lost::odomCallback, this);
  // pointPub = n.advertise<sensor_msgs::PointCloud>("point", 0, this);
  
  Lost::lostCalc();
}

Lost::~Lost()
{
}

void Lost::lostCalc()
{
  ros::Rate loop_rate(1);
  int loopCount = 0;
  while (ros::ok())
  {
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

    try
    {
      sensor_msgs::PointCloud base_cloud;

      // tf
      listener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(0.1));
      listener.lookupTransform("/map", "/base_link", ros::Time(0), transformVec);
      float orgX = transformVec.getOrigin().x();
      float orgY = transformVec.getOrigin().y();
      std::cout << loopCount << "===============================================================" << std::endl;
      std::cout << "tf_org xy:" << orgX << " " << orgY << std::endl;

      base_cloud = cloud;
      for (int i = 0; i < base_cloud.points.size(); i++)
      {
        base_cloud.points[i].x = cloud.points[i].x + orgX;
        base_cloud.points[i].y = cloud.points[i].y + orgY;
      }

      // pcl tf
      // sensor_msgs::PointCloud2 base_cloud2;
      // pcl_ros::transformPointCloud("/map", cloud2, base_cloud2, listener);
      // sensor_msgs::convertPointCloud2ToPointCloud(base_cloud2, base_cloud);
      // std::cout << "base_link xy:" << base_cloud.points[0].x << " " << base_cloud.points[0].y << std::endl;

      int lostCount = 0;
      for (int i = 0; i < base_cloud.points.size(); i++)
      {
        float width = (base_cloud.points[i].x - mapMsg.info.origin.position.x) / mapMsg.info.resolution;
        float height = mapMsg.info.height - (base_cloud.points[i].y - mapMsg.info.origin.position.y) / mapMsg.info.resolution;
        int data_num = width + height * mapMsg.info.width;
        if (mapMsg.data[data_num] == 100)
        {
          lostCount++;
          // std::cout << "base_cloud pixel wh:" << width << height << std::endl;
          // std::cout << "base_cloud size:" << base_cloud.points.size() << std::endl;
        }
      }
      float lostRate = float(base_cloud.points.size() - lostCount) / base_cloud.points.size() * 100;
      std::cout << "lostCount:" << lostCount << " lostRate:" << lostRate << " " << std::endl;

      // pointPub.publish(base_cloud);
    }
    catch (const std::exception &e)
    {
      std::cerr << e.what() << '\n';
    }
    ros::spinOnce();
    loop_rate.sleep();
    ++loopCount;
  }
}

/***************************************************************************************************************************************
* timer
* ***************************************************************************************************************************************/
// void Lost::timerCallback(const ros::TimerEvent &)
// {
// }

/***************************************************************************************************************************************
* odom subscribe
* ***************************************************************************************************************************************/

// void Lost::odomCallback(const nav_msgs::Odometry &msg)
// {
// }

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
  // ros::MultiThreadedSpinner spinner(10);
  // spinner.spin();

  return 0;
}