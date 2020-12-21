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

  // fuc
  void timerCallback(const ros::TimerEvent &);
  void mapCallback(const nav_msgs::OccupancyGrid &msg);
  void scanCallback(const sensor_msgs::LaserScan &msg);
  void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped &msg);
  void odomCallback(const nav_msgs::Odometry &msg);
  void lostCalc(float orgX = 0.0, float orgY = 0.0, float rotaYaw = 0.0);
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
  // timer = n.createTimer(ros::Duration(1.0), &Lost::timerCallback, this);
  subMap = n.subscribe("map", 10, &Lost::mapCallback, this);
  subScan = n.subscribe("scan", 10, &Lost::scanCallback, this);
  subAmclPose = n.subscribe("amcl_pose", 10, &Lost::amclPoseCallback, this);
  // subOdom = n.subscribe("odom", 1000, &Lost::odomCallback, this);
  pointPub = n.advertise<sensor_msgs::PointCloud>("point", 0, this);

  Lost::whileLoop();
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
    Lost::lostCalc();

    ros::spinOnce();
    loop_rate.sleep();
    ++loopCount;
  }
}

/***************************************************************************************************************************************
* lostCalc
* ***************************************************************************************************************************************/
void Lost::lostCalc(float orgX, float orgY, float rotaYaw)
{
  // define
  laser_geometry::LaserProjection projector_;
  tf::TransformListener listener(ros::Duration(10));
  tf::StampedTransform transform;

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
  projector_.projectLaser(scanMsg, cloud);
  sensor_msgs::PointCloud2 cloud2;
  projector_.projectLaser(scanMsg, cloud2);

  try
  {
    sensor_msgs::PointCloud base_cloud;

    // tf
    listener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(5.0));
    listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
    float orgX = transform.getOrigin().x();
    float orgY = transform.getOrigin().y();
    float rotaYaw = tf::getYaw(transform.getRotation());

    // geometry_msgs::PointStamped laser_point;
    // laser_point.header.frame_id = "base_link";
    // laser_point.header.stamp = ros::Time();
    // laser_point.point.x = 0.0;
    // laser_point.point.y = 0.0;
    // laser_point.point.z = 0.0;
    // geometry_msgs::PointStamped base_point;
    // listener.transformPoint("map", laser_point, base_point);
    // orgX = base_point.point.x;
    // orgY = base_point.point.y;

    // listener.waitForTransform("/map", "/laser", ros::Time(0), ros::Duration(5.0));
    // projector_.transformLaserScanToPointCloud("/map",scanMsg, cloud, listener);

    std::cout << "===============================================================" << std::endl;
    std::cout << "transform:" << orgX << ", " << orgY << ", " << rotaYaw << std::endl;

    // point cloud transform
    base_cloud = cloud;
    base_cloud.header.frame_id = "map";
    std::cout << "base_cloud.header.frame_id"
              << " " << base_cloud.header.frame_id << std::endl;
    for (int i = 0; i < base_cloud.points.size(); i++)
    {
      base_cloud.points[i].x = cloud.points[i].x + orgX;
      base_cloud.points[i].y = cloud.points[i].y + orgY;
      // listener.transformPoint("map", cloud.points[i], base_cloud.points[i]);
    }

    // listener.transformVector("/map", cloud.points, base_cloud.points);

    // pcl tf
    // sensor_msgs::PointCloud2 base_cloud2;
    // pcl_ros::transformPointCloud("/map", cloud2, base_cloud2, listener);
    // sensor_msgs::convertPointCloud2ToPointCloud(base_cloud2, base_cloud);

    std::cout << "base_cloud size:" << base_cloud.points.size() << std::endl;
    int lostCount = 0;
    for (int i = 0; i < base_cloud.points.size(); i++)
    {
      // std::cout << "test" << base_cloud.points[i].x << " " << mapMsg.info.origin.position.x << std::endl;
      float width = (base_cloud.points[i].x - mapMsg.info.origin.position.x) / mapMsg.info.resolution;
      float height = mapMsg.info.height - (base_cloud.points[i].y - mapMsg.info.origin.position.y) / mapMsg.info.resolution;
      int data_num = width + height * mapMsg.info.width;
      // std::cout << "base_cloud pixel wh:" << width << " " << height << std::endl;
      // std::cout << "data_num:" << data_num << std::endl;
      if (data_num >= 0)
      {
        if (mapMsg.data[data_num] == 100)
        {
          lostCount++;
        }
      }
    }
    float lostRate = float(base_cloud.points.size() - lostCount) / base_cloud.points.size() * 100;
    std::cout << "lostCount:" << lostCount << " lostRate:" << lostRate << " " << std::endl;

    pointPub.publish(base_cloud);
  }
  catch (const std::exception &e)
  {
    std::cerr << e.what() << '\n';
  }
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

  // std::cout << "org xy:" << msg.pose.pose.position.x << " " << msg.pose.pose.position.y << std::endl;

  // Lost::lostCalc(msg.pose.pose.position.x, msg.pose.pose.position.y);
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