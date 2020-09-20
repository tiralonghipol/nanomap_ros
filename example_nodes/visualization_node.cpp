#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>

#include "nanomap.h"
#include "nanomap_visualizer.h"

#include "stopwatch.h"

using namespace std;

Stopwatch global_time;

geometry_msgs::Pose curr_pose;

visualization_msgs::Marker getQueryPtMarker(int status, int id, Vector3 point_pos)
{
  visualization_msgs::Marker marker1;
  marker1.header.frame_id = "delta/base_link";
  marker1.header.stamp = ros::Time();
  marker1.ns = "p" + std::to_string(id);
  marker1.id = id;
  marker1.type = visualization_msgs::Marker::SPHERE;
  marker1.action = visualization_msgs::Marker::ADD;
  marker1.pose.position.x = point_pos(0);
  marker1.pose.position.y = point_pos(1);
  marker1.pose.position.z = point_pos(2);
  marker1.pose.orientation.x = 0.0;
  marker1.pose.orientation.y = 0.0;
  marker1.pose.orientation.z = 0.0;
  marker1.pose.orientation.w = 1.0;
  marker1.scale.x = 1.0;
  marker1.scale.y = 1.0;
  marker1.scale.z = 1.0;
  if (status == 6) // free space
  {
    cout << "Free Space!" << endl;
    marker1.color.a = 1.0; // Don't forget to set the alpha!
    marker1.color.r = 0.0;
    marker1.color.g = 1.0;
    marker1.color.b = 0.0;
  }
  // else if(status == 5) {
  //   marker1.color.a = 1.0; // Don't forget to set the alpha!
  //   marker1.color.r = 1.0;
  //   marker1.color.g = 0.0;
  //   marker1.color.b = 0.0;
  // }
  // else if(status == 3) {
  //   marker1.color.a = 1.0; // Don't forget to set the alpha!
  //   marker1.color.r = 0.0;
  //   marker1.color.g = 0.0;
  //   marker1.color.b = 1.0;
  // }
  // else if(status == 4) {  // yellow
  //   marker1.color.a = 1.0; // Don't forget to set the alpha!
  //   marker1.color.r = 1.0;
  //   marker1.color.g = 1.0;
  //   marker1.color.b = 0.0;
  // }
  else
  {
    // cout << "Occupied Space!" << endl;
    // cyan
    marker1.color.a = 1.0; // Don't forget to set the alpha!
    marker1.color.r = 1.0;
    marker1.color.g = 0.0;
    marker1.color.b = 0.0;
  }
  return marker1;
}

class NanoMapNode
{
public:
  NanoMapNode() : nh("~")
  {
    nanomap_visualizer.Initialize(nh);

    ros::NodeHandle nh;
    NanoMapVisualizer nanomap_visualizer;
    nanomap_visualizer.Initialize(nh);

    Matrix3 K;
    K << 205.27, 0.0, 160.0, 0.0, 205.27, 120.0, 0.0, 0.0, 1.0;
    nanomap.SetCameraInfo(4.0, 320.0, 240.0, K); // this K matrix was for binned, but the actual data is not binned
    nanomap.SetSensorRange(20.0);
    nanomap.SetNumDepthImageHistory(150);
    Matrix3 body_to_rdf;
    body_to_rdf << 0, -1, 0, 0, 0, -1, 1, 0, 0;
    nanomap.SetBodyToRdf(body_to_rdf);

    pcl_sub = nh.subscribe("points", 100, &NanoMapNode::PointCloudCallback, this);
    pose_updates_sub = nh.subscribe("path", 100, &NanoMapNode::SmoothedPosesCallback, this);
    // pose_sub = nh.subscribe("poseStamped", 100, &NanoMapNode::PoseCallback, this);
    odom_sub = nh.subscribe("odometry", 100, &NanoMapNode::OdometryCallback, this);
  };

  ros::NodeHandle nh;
  ros::Subscriber pcl_sub;
  ros::Subscriber pose_updates_sub;
  // ros::Subscriber pose_sub;
  ros::Subscriber odom_sub;
  NanoMap nanomap;
  NanoMapVisualizer nanomap_visualizer;
  ros::Publisher query_points_pub = nh.advertise<visualization_msgs::MarkerArray>("query_points", 0);

  bool initialized = false;

  void PointCloudCallback(const sensor_msgs::PointCloud2 &msg)
  {
    Stopwatch sw;
    sw.Start();
    pcl::PCLPointCloud2 cloud2_rdf;
    pcl_conversions::toPCL(msg, cloud2_rdf);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_rdf(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(cloud2_rdf, *cloud_rdf);
    NanoMapTime nm_time(msg.header.stamp.sec, msg.header.stamp.nsec);
    nanomap.AddPointCloud(cloud_rdf, nm_time, msg.header.seq);

    float insertion_time = sw.ElapsedMillis();
    // std::cout << "insertion_time: " << insertion_time << std::endl;

    sw.Start();

    sw.Stop();
    float distance_update_time = sw.ElapsedMillis();
    //std::cout << "distance_update_time: " << distance_update_time << std::endl;

    sw.Start();
    int num_samples = 10;
    float rad = 15.0;
    float delta = 2 * rad / (num_samples - 1) - .0001; // subtraction needed to get floating point happy in loop

    NanoMapKnnArgs args;
    args.axis_aligned_linear_covariance = Vector3(0.1, 0.1, 0.1);
    args.early_exit = false;

    visualization_msgs::MarkerArray query_points;
    NanoMapKnnReply reply;

    int ni = 0;
    // for (float x = -rad; x <= rad; x =x+delta) {
    //   for (float y = -rad; y <= rad; y =y+delta) {
    //     for (float z = -rad/2; z <= rad/2; z =z+delta) {
    //       ni++;
    //       args.query_point_current_body_frame = Vector3(x, y, z);
    //       NanoMapKnnReply reply = nanomap.KnnQuery(args);
    //       bool hit = false;
    //       for(Vector3 v:reply.closest_points_in_frame_id) {
    //        // std::cout << v(0) << " " << v(1) << " " << v(2) << std::endl;
    //         double closest_dist = (args.query_point_current_body_frame - v).norm();
    //         if(closest_dist <= 10.0) {
    //           std::cout << "Closest distance: " << closest_dist << std::endl;
    //           hit = true;
    //         }
    //       }
    //       int status;
    //       if((int)(reply.fov_status) == 6) {
    //         status = 6;
    //         if(!hit)
    //           status = 6;
    //         else
    //           status = 5;
    //       }
    //       else
    //         status = 5;
    //       // visualization_msgs::Marker mp = getQueryPtMarker((int)(reply.fov_status), ni, args.query_point_current_body_frame);
    //       visualization_msgs::Marker mp = getQueryPtMarker(status, ni, args.query_point_current_body_frame);
    //       query_points.markers.push_back(mp);
    //        // std::cout << "---" << std::endl;
    //     }
    //   }
    // }
    int n = 0;
    for (float m = -1.57; m <= 1.571; m += 0.5)
    {
      for (float x = -rad; x <= rad; x = x + delta)
      {
        n++;
        bool hit = false;
        // for(float dx = -0.5;dx<=0.5;dx+=0.1) {
        //   for(float dy = -0.5;dy<=0.5;dy+=0.1) {
        //     for(float dz = -0.5;dz<=0.5;dz+=0.1) {
        //       args.query_point_current_body_frame = Vector3(x+dx, m*x+dy, 0.0+dz);
        //       reply = nanomap.KnnQuery(args);
        //       if((int)(reply.fov_status) == 6) {
        //         // std::cout << reply.frame_id << std::endl;
        //         for(Vector3 v:reply.closest_points_in_frame_id) {
        //           double cld = (reply.query_point_in_frame_id - v).norm();
        //           if(cld <= 4.0) {
        //             hit = true;
        //             break;
        //           }
        //         }
        //         if(hit)
        //           break;
        //         continue;
        //       }
        //       else {
        //         hit = true;
        //         break;
        //       }
        //     }
        //     if(hit)
        //       break;
        //   }
        //   if(hit)
        //     break;
        // }
        args.query_point_current_body_frame = Vector3(x, m * x, 0.0);
        reply = nanomap.KnnQuery(args); // pass a point to query
        // std::cout << "Query point: "
        //           << args.query_point_current_body_frame(0) << " " << args.query_point_current_body_frame(1) << " " << args.query_point_current_body_frame(2) << std::endl;
        // std::cout << "FOV status: " << (int)(reply.fov_status) << std::endl;

        for (Vector3 v : reply.closest_points_in_frame_id)
        {
          // std::cout << v(0) << " " << v(1) << " " << v(2) << std::endl;
          double cld = (reply.query_point_in_frame_id - v).norm();

          if (cld <= 2.0)
          {
            // std::cout << "Closest distance: " << cld << std::endl;
            hit = true;
            break;
          }
        }
        int status;
        if ((int)(reply.fov_status) == 6)
        {
          // status = 6;
          if (!hit)
            status = 6;
          else
            status = 5;
        }
        else
          status = 5;
        // if(!hit)
        //   status = 6;
        // else
        //   status = 5;
        visualization_msgs::Marker mp0 = getQueryPtMarker(status, n, args.query_point_current_body_frame);
        query_points.markers.push_back(mp0);
      }
    }

    args.query_point_current_body_frame = Vector3(15.0 - curr_pose.position.x, 0.0 - curr_pose.position.y, 0.0 - curr_pose.position.z);
    reply = nanomap.KnnQuery(args);
    // std::cout << "Query point: "
    // << args.query_point_current_body_frame(0) << " " << args.query_point_current_body_frame(1) << " " << args.query_point_current_body_frame(2) << std::endl;
    // std::cout << "FOV status: " << (int)(reply.fov_status) << std::endl;
    // for (Vector3 v : reply.closest_points_in_frame_id)
    // {
    //   // std::cout << v(0) << " " << v(1) << " " << v(2) << std::endl;
    //   // std::cout << "Closest distance: " << (args.query_point_current_body_frame - v).norm() << std::endl;
    // }
    // visualization_msgs::Marker mp1 = getQueryPtMarker((int)(reply.fov_status), 1, args.query_point_current_body_frame);
    // query_points.markers.push_back(mp1);

    // args.query_point_current_body_frame = Vector3(-15.0, 1.0, 1.0);
    // reply = nanomap.KnnQuery(args);
    // std::cout << "Query point: "
    //           << args.query_point_current_body_frame(0) << " " << args.query_point_current_body_frame(1) << " " << args.query_point_current_body_frame(2) << std::endl;
    // std::cout << "FOV status: " << (int)(reply.fov_status) << std::endl;
    // for(Vector3 v:reply.closest_points_in_frame_id) {
    //   std::cout << v(0) << " " << v(1) << " " << v(2) << std::endl;
    //   std::cout << "Closest distance: " << (args.query_point_current_body_frame - v).norm() << std::endl;
    // }
    // visualization_msgs::Marker mp2 = getQueryPtMarker((int)(reply.fov_status), 2, args.query_point_current_body_frame);
    // query_points.markers.push_back(mp2);

    // args.query_point_current_body_frame = Vector3(0.0, 15.0, 1.0);
    // reply = nanomap.KnnQuery(args);
    // std::cout << "Query point: "
    //           << args.query_point_current_body_frame(0) << " " << args.query_point_current_body_frame(1) << " " << args.query_point_current_body_frame(2) << std::endl;
    // std::cout << "FOV status: " << (int)(reply.fov_status) << std::endl;
    // for(Vector3 v:reply.closest_points_in_frame_id) {
    //   std::cout << v(0) << " " << v(1) << " " << v(2) << std::endl;
    //   std::cout << "Closest distance: " << (args.query_point_current_body_frame - v).norm() << std::endl;
    // }
    // visualization_msgs::Marker mp3 = getQueryPtMarker((int)(reply.fov_status), 3, args.query_point_current_body_frame);
    // query_points.markers.push_back(mp3);

    // args.query_point_current_body_frame = Vector3(0.0, -15.0, 1.0);
    // reply = nanomap.KnnQuery(args);
    // std::cout << "Query point: "
    //           << args.query_point_current_body_frame(0) << " " << args.query_point_current_body_frame(1) << " " << args.query_point_current_body_frame(2) << std::endl;
    // std::cout << "FOV status: " << (int)(reply.fov_status) << std::endl;
    // for(Vector3 v:reply.closest_points_in_frame_id) {
    //   std::cout << v(0) << " " << v(1) << " " << v(2) << std::endl;
    //   std::cout << "Closest distance: " << (args.query_point_current_body_frame - v).norm() << std::endl;
    // }
    // visualization_msgs::Marker mp4 = getQueryPtMarker((int)(reply.fov_status), 4, args.query_point_current_body_frame);
    // query_points.markers.push_back(mp4);

    query_points_pub.publish(query_points);

    // std::cout << "---" << std::endl;
    sw.Stop();
    float sample_time = sw.ElapsedMillis();
    // std::cout << "sample_time: " << sample_time << std::endl;
  }

  void DrawNanoMapVisualizer()
  {
    std::vector<Matrix4> edges = nanomap.GetCurrentEdges();
    nanomap_visualizer.DrawFrustums(edges);
  }

  // void PoseCallback(geometry_msgs::PoseStamped const &pose)
  void OdometryCallback(nav_msgs::Odometry const &odom)
  {
    geometry_msgs::Pose pose = odom.pose.pose;

    // cout << "Pose: " << pose;
    Eigen::Quaterniond quat(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    Vector3 pos = Vector3(pose.position.x, pose.position.y, pose.position.z);
    // Eigen::Quaterniond quat(pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z);
    // Vector3 pos = Vector3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
    // NanoMapTime nm_time(pose.header.stamp.sec, pose.header.stamp.nsec);
    NanoMapTime nm_time(odom.header.stamp.sec, odom.header.stamp.nsec);
    NanoMapPose nm_pose(pos, quat, nm_time);
    nanomap.AddPose(nm_pose);

    // curr_pose = pose.pose;
    curr_pose = pose;

    // todo: abstract this into SetLastPose
    Matrix4 transform = Eigen::Matrix4d::Identity();
    transform.block<3, 3>(0, 0) = nm_pose.quaternion.toRotationMatrix();
    transform.block<3, 1>(0, 3) = nm_pose.position;
    nanomap_visualizer.SetLastPose(transform);

    DrawNanoMapVisualizer();
  }

  void SmoothedPosesCallback(nav_msgs::Path path)
  {
    std::vector<NanoMapPose> smoothed_path_vector;
    size_t path_size = path.poses.size();
    for (size_t i = 0; i < path_size; i++)
    {
      geometry_msgs::PoseStamped pose = path.poses.at(i);
      Eigen::Quaterniond quat(pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z);
      Vector3 pos = Vector3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
      NanoMapTime nm_time(pose.header.stamp.sec, pose.header.stamp.nsec);
      NanoMapPose nm_pose(pos, quat, nm_time);
      smoothed_path_vector.push_back(nm_pose);
    }
    nanomap.AddPoseUpdates(smoothed_path_vector);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "nanomap_visualization_example");
  NanoMapNode nanomap_node;
  ros::spin();
  return 0;
}
