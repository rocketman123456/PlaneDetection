#include <stdint.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <vector>
#include "RSPD/point.h"
#include "RSPD/pointcloud.h"
#include "RSPD/planedetector.h"
#include "RSPD/normalestimator.h"
#include "RSPD/boundaryvolumehierarchy.h"
#include "RSPD/connectivitygraph.h"
#include <iostream>
#include <fstream>
#include <thread>

typedef pcl::PointCloud<pcl::PointXYZ> PCLPointCloud;
void callback(const PCLPointCloud::ConstPtr &msg)
{
  std::vector<Point3d> points;
  BOOST_FOREACH (const pcl::PointXYZ &pt, msg->points)
  {
    points.push_back(Point3d(Eigen::Vector3f(pt.x, pt.y, pt.z)));
  }
  PointCloud3d *pointCloud = new PointCloud3d(points);
  // you can skip the normal estimation if you point cloud already have normals
  // std::cout << "Estimating normals..." << std::endl;
  ROS_INFO("Estiating normals..");
  size_t normalsNeighborSize = 30;
  Octree octree(pointCloud);
  octree.partition(10, 30);
  ConnectivityGraph *connectivity = new ConnectivityGraph(pointCloud->size());
  pointCloud->connectivity(connectivity);
  NormalEstimator3d estimator(&octree, normalsNeighborSize, NormalEstimator3d::QUICK);
  // std::cout << "Number of samples: " << pointCloud->size() << std::endl;
  ROS_INFO("Number of samples: %zu", pointCloud->size());
  for (size_t i = 0; i < pointCloud->size(); i++)
  {
    if (i % 10000 == 0)
    {
      std::cout << i / float(pointCloud->size()) * 100 << "%..." << std::endl;
    }
    NormalEstimator3d::Normal normal = estimator.estimate(i);
    connectivity->addNode(i, normal.neighbors);
    (*pointCloud)[i].normal(normal.normal);
    (*pointCloud)[i].normalConfidence(normal.confidence);
    (*pointCloud)[i].curvature(normal.curvature);
  }

  // std::cout << "Detecting planes..." << std::endl;
  ROS_INFO("Detecting planes..");

  PlaneDetector detector(pointCloud);
  detector.minNormalDiff(0.5f);
  detector.maxDist(0.258819f);
  detector.outlierRatio(0.75f);

  std::set<Plane *> planes = detector.detect();
  // std::cout << "Found " << planes.size() << " planes!" << std::endl;
  ROS_INFO("Found %zu planes!", planes.size());
  // std::cout << "Saving results..." << std::endl;
  ROS_INFO("visualizing results...");

  Geometry *geometry = pointCloud->geometry();
  for (Plane *plane : planes)
  {
    geometry->addPlane(plane);
  }
  // many output formats are allowed.if you want to run our 'compare_plane_detector', uncomment the line below and comment the rest
  //  pointCloudIO.saveGeometry(geometry, outputFileName);
  std::ofstream outputFile(std::string("planeIGuess") + ".txt");
  for (Plane *plane : planes)
  {
    Eigen::Vector3f v1 = plane->center() + plane->basisU() + plane->basisV();
    Eigen::Vector3f v2 = plane->center() + plane->basisU() - plane->basisV();
    Eigen::Vector3f v3 = plane->center() - plane->basisU() - plane->basisV();
    Eigen::Vector3f v4 = plane->center() - plane->basisU() + plane->basisV();

    outputFile << "Normal: [" << plane->normal()[0] << ", " << plane->normal()[1] << ", " << plane->normal()[2] << "]; "
               << "Center: [" << plane->center()[0] << ", " << plane->center()[1] << ", " << plane->center()[2] << "]; "
               << "Vertices: [[" << v1.x() << "," << v1.y() << "," << v1.z() << "], "
               << "[" << v2.x() << "," << v2.y() << "," << v2.z() << "], "
               << "[" << v3.x() << "," << v3.y() << "," << v3.z() << "], "
               << "[" << v4.x() << "," << v4.y() << "," << v4.z() << "]]" << std::endl;
  }
  std::cout << "Done Saving!" << std::endl;
  ROS_INFO("Done Saving!");
  delete pointCloud;
}

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "TestListener");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  ros::Subscriber sub = n.subscribe("/rtabmap/cloud_map", 1000, callback);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */

  return 0;
}
