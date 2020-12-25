#include "ros/ros.h"
#include "std_msgs/String.h"

#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/LaserScan.h"
#include <sensor_msgs/point_cloud_conversion.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

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
  ros::Subscriber subAmclPose;
  ros::Subscriber subOdom;
  ros::Publisher pointPub;
  ros::Publisher testPointPub;
  ros::Publisher mapPointPub;

  // fuc
  void timerCallback(const ros::TimerEvent &);
  void mapCallback(const nav_msgs::OccupancyGrid &msg);
  void scanCallback(const sensor_msgs::LaserScan &msg);
  void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped &msg);
  void odomCallback(const nav_msgs::Odometry &msg);
  void lostCalc();
  void whileLoop();

  // var
  nav_msgs::OccupancyGrid mapMsg;
  sensor_msgs::LaserScan scanMsg;
  geometry_msgs::PoseWithCovarianceStamped amclMsg;

public:
  Lost(/* args */);
  ~Lost();
};

/**************************************************************************************************************
 * Lost::Lost
 * ************************************************************************************************************/
Lost::Lost(/* args */)
{
  ROS_INFO("#### lost navigation start ####");

  // timer = n.createTimer(ros::Duration(1.0), &Lost::timerCallback, this);
  subMap = n.subscribe("map", 10, &Lost::mapCallback, this);
  subScan = n.subscribe("scan", 10, &Lost::scanCallback, this);
  subAmclPose = n.subscribe("amcl_pose", 10, &Lost::amclPoseCallback, this);
  // subOdom = n.subscribe("odom", 1000, &Lost::odomCallback, this);
  pointPub = n.advertise<sensor_msgs::PointCloud>("point", 0, this);
  testPointPub = n.advertise<sensor_msgs::PointCloud>("testPoint", 0, this);
  mapPointPub = n.advertise<sensor_msgs::PointCloud>("mapPoint", 0, this);

  // Lost::whileLoop();
}

Lost::~Lost()
{
}

/***************************************************************************************************************************************
* whileLoop
* ***************************************************************************************************************************************/
void Lost::whileLoop()
{
  ros::Rate loop_rate(10);
  int loopCount = 0;
  while (ros::ok())
  {

    ros::spinOnce();
    loop_rate.sleep();
    ++loopCount;
  }
}

/***************************************************************************************************************************************
* lostCalc
* ***************************************************************************************************************************************/
void Lost::lostCalc()
{
  ///////////// define ///////////////////
  laser_geometry::LaserProjection projector_;
  tf::TransformListener listener(ros::Duration(10));
  tf::StampedTransform transform;

  //////////// scan msg /////////////////
  // std::cout << "scan size:" << scanMsg.ranges.size() << std::endl;
  // sensor_msgs::PointCloud cloud;
  // projector_.projectLaser(scanMsg, cloud);
  sensor_msgs::PointCloud2 cloud2;
  projector_.projectLaser(scanMsg, cloud2);
  sensor_msgs::PointCloud base_cloud;
  sensor_msgs::PointCloud2 base_cloud2;

  //////////// tf ///////////////////////
  try
  {
    listener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(5.0));
    cloud2.header.stamp = ros::Time(0);
    pcl_ros::transformPointCloud("/map", cloud2, base_cloud2, listener);
    sensor_msgs::convertPointCloud2ToPointCloud(base_cloud2, base_cloud);
  }
  catch (const std::exception &e)
  {
    std::cerr << e.what() << '\n';
  }
  std::cout << "===============================================================" << std::endl;

  /////////// map msg //////////////////
  sensor_msgs::PointCloud map_cloud;
  map_cloud.header = base_cloud.header;
  map_cloud.channels = base_cloud.channels;
  geometry_msgs::Point32 mapPoint;
  // std::cout << "map:" << mapMsg.info << std::endl;
  std::cout << "map size:" << mapMsg.data.size() << std::endl;
  // std::cout << "map pos:";
  int count_100 = 0, count_0 = 0, count_else = 0;
  for(int i = 0; i <  mapMsg.data.size(); i++)
  {
    if(mapMsg.data[i] == 100)
    {
      float width = i % mapMsg.info.width;
      float height =  i / mapMsg.info.width;
      mapPoint.x = width * mapMsg.info.resolution + mapMsg.info.origin.position.x;
      mapPoint.y = height  * mapMsg.info.resolution + mapMsg.info.origin.position.y;
      map_cloud.points.push_back(mapPoint);
      // std::cout << i << "/" << i % mapMsg.info.width << "/" << i / mapMsg.info.width << ", " ;
      count_100 ++;
    }
    else if(mapMsg.data[i] == 0)
    {
      count_0 ++;
    }
    else
    {
      count_else ++;
    }
  }
  std::cout << std::endl;
  std::cout << "100:" << count_100 << " 0:" << count_0 << " else:" << count_else << std::endl;

  //////////// calculate lost //////////////
  // std::cout << "base_cloud size:" << base_cloud.points.size() << std::endl;
  int lostCount = 0;
  sensor_msgs::PointCloud test_cloud;
  test_cloud.header = base_cloud.header;
  // test_cloud.channels = base_cloud.channels;
  for (int i = 0; i < base_cloud.points.size(); i++)
  {
    // std::cout << "test" << base_cloud.points[i].x << " " << mapMsg.info.origin.position.x << std::endl;
    float width = (base_cloud.points[i].x - mapMsg.info.origin.position.x) / mapMsg.info.resolution;
    float height = mapMsg.info.height - (base_cloud.points[i].y - mapMsg.info.origin.position.y) / mapMsg.info.resolution;
    int data_num = width + height * mapMsg.info.width;
    
    if (data_num >= 0 && data_num < mapMsg.data.size())
    {
      if (mapMsg.data[data_num] == 100)
      {
        lostCount++;
        std::cout << "base_cloud pixel wh:" << width << " " << height << std::endl;
        std::cout << "data_num:" << data_num << std::endl;
        test_cloud.points.push_back(base_cloud.points[i]);
      }
    }
  }
  float lostRate = float(base_cloud.points.size() - lostCount) / base_cloud.points.size() * 100;
  std::cout << "lostCount:" << lostCount << " lostRate:" << lostRate << " " << std::endl;

  pointPub.publish(base_cloud);
  testPointPub.publish(test_cloud);
  mapPointPub.publish(map_cloud);
}

/***************************************************************************************************************************************
* timer
* ***************************************************************************************************************************************/
// void Lost::timerCallback(const ros::TimerEvent &)
// {
// }

/***************************************************************************************************************************************
* amcl pose subscribe
* ***************************************************************************************************************************************/
void Lost::amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped &msg)
{
  amclMsg = msg;
}

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

  Lost::lostCalc();
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