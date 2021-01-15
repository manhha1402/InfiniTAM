
#include <cstdlib>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
// MESSAGES
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Int16.h>
#include <visualization_msgs/Marker.h>
#include <image_transport/image_transport.h>

// TF
#include <eigen_conversions/eigen_msg.h>
#include <tf/message_filter.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <pcl/io/obj_io.h>
#include <pcl/surface/simplification_remove_unused_vertices.h>
#include <open3d/Open3D.h>
#include <Eigen/Eigen>
#include "libs/ITMLib/ITMLibDefines.h"
#include "libs/ITMLib/Core/ITMBasicEngine.h"
#include "libs/ITMLib/Core/ITMBasicSurfelEngine.h"
#include "libs/ITMLib/Core/ITMMultiEngine.h"


//using namespace InfiniTAM::Engine;
//using namespace InputSource;

void saveRGBDAndPose(std::string data_path, const Eigen::Matrix4d& cam2world, const cv::Mat& color_image, const cv::Mat& depth_image, const int frame_idx, std::string prefix = "")
{
  std::ofstream pose_f;

  std::ostringstream curr_frame_prefix;
  curr_frame_prefix << std::setw(6) << std::setfill('0') << frame_idx;

  std::string depth_im_file = data_path + "/" + prefix + curr_frame_prefix.str() + ".depth.png";
  std::string rgb_im_file = data_path + "/" + prefix + curr_frame_prefix.str() + ".color.png";
  std::string cam2world_file = data_path + "/" + prefix + curr_frame_prefix.str() + ".pose.txt";
  // save pose
  pose_f.open(cam2world_file);
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 4; j++)
    {
      pose_f << cam2world(i, j) << " ";
    }
    pose_f << std::endl;
  }
  pose_f.close();

  // save rgb-d image
  cv::imwrite(rgb_im_file, color_image);
  cv::imwrite(depth_im_file, depth_image);
}

void savePoseKITTIFormat(const std::string& file_name, const std::vector<Eigen::Matrix4d>& cam2worlds)
{
  /*
   *
      a b c d
      e f g h
      i j k l -> a b c d e f g h i j k l
      0 0 0 1
   *
   */
  std::ofstream pose_f;
  pose_f.open(file_name);
  for (std::size_t frame_id = 0; frame_id < cam2worlds.size(); ++frame_id) {
    std::cout<<frame_id<<std::endl;
    std::cout<<cam2worlds[frame_id]<<std::endl;
    pose_f <<cam2worlds[frame_id](0,0)<<" "<<
             cam2worlds[frame_id](0,1)<<" "<<
             cam2worlds[frame_id](0,2)<<" "<<
             cam2worlds[frame_id](0,3)<<" "<<
             cam2worlds[frame_id](1,0)<<" "<<
             cam2worlds[frame_id](1,1)<<" "<<
             cam2worlds[frame_id](1,2)<<" "<<
             cam2worlds[frame_id](1,3)<<" "<<
             cam2worlds[frame_id](2,0)<<" "<<
             cam2worlds[frame_id](2,1)<<" "<<
             cam2worlds[frame_id](2,2)<<" "<<
             cam2worlds[frame_id](2,3)<<std::endl;
  }
  pose_f.close();

}


using namespace ITMLib;
class InfiniTAMROS
{
public:
  InfiniTAMROS(ros::NodeHandle& node_handle);
  virtual ~InfiniTAMROS();
  void coloredPointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& colored_pointcloud_msg);
  void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &data);
  void publishRayCasting();
  // Services that are used to start, stop and pause and continue the scanning process
  bool startScanning(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp);
  bool stopScanning(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp);

  //////////////////////
  void extractMesh(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_pcl,
      pcl::PolygonMesh::Ptr polygon_mesh_ptr);
  void extractPointCloud( const ITMMesh::Triangle& triangle_array,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_pcl
                          );
private:
  Eigen::Affine3d toEigenMatrix(const ORUtils::SE3Pose& pose);
  // reset internal variables
  void reset();


  ros::NodeHandle node_;
  image_transport::Publisher raycast_img_publisher_;
  image_transport::ImageTransport image_transport_;

  ITMLibSettings *internal_settings_;
  ITMMainEngine *main_engine_;
  ITMLib::ITMRGBDCalib calib_;
  ros::Subscriber camera_info_sub_;
  ros::Subscriber colored_pointcloud_sub_;  ///< Listens for the colored pointcloud
  ros::ServiceServer stop_scanning_service_;
  ros::ServiceServer start_scanning_service_;

  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf2_static_br_; /** tf brodcaster */
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf2_br_; /** tf brodcaster */



  std::string camera_info_topic_;
  std::string colored_point_cloud_topic_;
  Eigen::Matrix3d intrinsic_;
  cv::Mat ray_casting_view_;
  Eigen::Affine3d cam_pose_;

  int width_;
  int height_;
  int frame_idx_;
  bool valid_;

  bool stop_scanning_;
  bool start_scanning_;
  std::vector<cv::Mat> texture_images_;
  std::vector<cv::Mat> depth_images_;
  std::vector<Eigen::Matrix4d> texture_cam2worlds_;
  int diff_;
  int prev_diff_;
  int save_frame_idx_;
  const int collection_time_ = 2;
  std::chrono::system_clock::time_point beginning_time_;

  std::vector<Eigen::Matrix4d> robot_poses_;
  std::vector<Eigen::Matrix4d> camera_poses_;
};
InfiniTAMROS::InfiniTAMROS(ros::NodeHandle& node_handle) : node_(node_handle), image_transport_(node_handle)
{

  ros::NodeHandle pnh("~");
  const std::string base_link = "base_link";
  const std::string tip_link = "camera_color_optical_frame";
  const std::string camera_info_topic = "/camera/color/camera_info";
  const std::string colored_point_cloud_topic = "/camera/depth_registered/points";
  /*
  if (!pnh.param("/camera_info_topic", camera_info_topic_, camera_info_topic))
  {
    ROS_WARN("Failed to get camera_info_topic from parameter server.");
  }
  if (!pnh.param("/pointcloud_topic", colored_point_cloud_topic_, colored_point_cloud_topic))
  {
    ROS_WARN("Failed to get colored_point_cloud_topic from parameter server.");
  }
*/


  internal_settings_ = new ITMLibSettings();
  internal_settings_->libMode = ITMLibSettings::LIBMODE_LOOPCLOSURE;
  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>();
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
  tf2_static_br_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>();
  tf2_br_ = std::make_shared<tf2_ros::TransformBroadcaster>();

  reset();
  camera_info_sub_ = node_.subscribe(camera_info_topic, 1, &InfiniTAMROS::cameraInfoCallback, this);
  colored_pointcloud_sub_ = node_.subscribe(colored_point_cloud_topic, 1, &InfiniTAMROS::coloredPointCloudCallback, this);
  raycast_img_publisher_ = image_transport_.advertise("raycast_image", 1);

  stop_scanning_service_ = node_.advertiseService("stop_scanning", &InfiniTAMROS::stopScanning, this);
  start_scanning_service_ = node_.advertiseService("start_scanning", &InfiniTAMROS::startScanning, this);


}



InfiniTAMROS::~InfiniTAMROS()
{
  delete internal_settings_;
  delete  main_engine_;

}

void InfiniTAMROS::coloredPointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& colored_pointcloud_msg)
{
  if (!(start_scanning_))
  {
    ROS_WARN_THROTTLE(2, "start_scanning: %d", start_scanning_);
    return;
  }
  if (stop_scanning_)
  {
    std::string data_path = "/home/manhha/catkin_ws/data_inifitam/";

    main_engine_->SaveSceneToMesh("/home/manhha/catkin_ws/data_inifitam/mesh.obj",10000000);
    for (std::size_t frame_ix = 0; frame_ix < save_frame_idx_; frame_ix++)
    {
      saveRGBDAndPose(data_path, texture_cam2worlds_[frame_ix], texture_images_[frame_ix], depth_images_[frame_ix], frame_ix);
    }
    savePoseKITTIFormat(data_path+"robot_poses.txt",robot_poses_);
    savePoseKITTIFormat(data_path+"cam_poses.txt",camera_poses_);

    reset();
  }

  if(!valid_)
    return;

  frame_idx_++;
  if (frame_idx_ == 1)
  {
    main_engine_ = new ITMMultiEngine<ITMVoxel, ITMVoxelIndex>(internal_settings_, calib_,
                                                               calib_.intrinsics_rgb.imgSize,
                                                               calib_.intrinsics_d.imgSize);


    Eigen::Affine3d pose;
    pose = toEigenMatrix(*main_engine_->GetTrackingState()->pose_d);

    geometry_msgs::TransformStamped pose_tf;
    pose_tf = tf2::eigenToTransform(pose.inverse());
    pose_tf.header.stamp = ros::Time(0);
    pose_tf.header.frame_id = colored_pointcloud_msg->header.frame_id;
    pose_tf.child_frame_id = "first_frame";
    tf2_static_br_->sendTransform(pose_tf);
    beginning_time_ = std::chrono::system_clock::now();

    geometry_msgs::TransformStamped cam0_to_base_tf;
    cam0_to_base_tf = tf2_buffer_->lookupTransform("base_link", colored_pointcloud_msg->header.frame_id,ros::Time(0));
    Eigen::Affine3d cam0_to_base = tf2::transformToEigen(cam0_to_base_tf);
    robot_poses_.push_back(cam0_to_base.matrix());
    camera_poses_.push_back(cam0_to_base.matrix());
    cam_pose_ = cam0_to_base;
    return;
  }
  // convert point cloud message
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::fromROSMsg(*colored_pointcloud_msg, *input_cloud);
  ITMUChar4Image *color_image = new ITMUChar4Image(calib_.intrinsics_rgb.imgSize,MEMORYDEVICE_CPU);
  ITMShortImage *depth_image = new ITMShortImage(calib_.intrinsics_d.imgSize,MEMORYDEVICE_CPU);
  cv::Mat cv_rgb(calib_.intrinsics_rgb.imgSize.y,calib_.intrinsics_rgb.imgSize.x,CV_8UC3);
  cv::Mat cv_depth(calib_.intrinsics_d.imgSize.y,calib_.intrinsics_d.imgSize.x,CV_16UC1);
  int index = 0;

  Vector4u *rgb = color_image->GetData(MEMORYDEVICE_CPU);
  short *depth = depth_image->GetData(MEMORYDEVICE_CPU);
  for (std::size_t v = 0; v < input_cloud->height; v++)
  {
    for (std::size_t u = 0; u < input_cloud->width; u++, index++)
    {
      pcl::PointXYZRGB& point = (*input_cloud)[index];
      Vector4u rgb_val;
      rgb_val.x = point.b;
      rgb_val.y = point.g;
      rgb_val.z = point.r;
      rgb_val.w = 255;
      cv_rgb.at<cv::Vec3b>(v,u) = cv::Vec3b(point.b, point.g, point.r);
      cv_depth.at<ushort>(v, u) = static_cast<ushort>(point.z * 1000);

      rgb[index] = rgb_val;
      depth[index] = static_cast<unsigned short>(point.z * 1000);
    }
  }
  ITMTrackingState::TrackingResult trackerResult;
  trackerResult = main_engine_->ProcessFrame(color_image,depth_image);
  if(trackerResult !=ITMTrackingState::TrackingResult::TRACKING_GOOD)
  {
    std::cout<<"tracking failed"<<std::endl;
    std::cout<<frame_idx_<<std::endl;
  }
  std::cout<<trackerResult<<std::endl;

  ITMUChar4Image *ray_cast_image = new  ITMUChar4Image(calib_.intrinsics_d.imgSize,MEMORYDEVICE_CPU) ;
  ITMLib::ITMMainEngine::GetImageType type;
  type = ITMLib::ITMMainEngine::InfiniTAM_IMAGE_COLOUR_FROM_NORMAL;
  ORUtils::SE3Pose freeviewPose;
  ITMLib::ITMIntrinsics freeviewIntrinsics;

  main_engine_->GetImage(ray_cast_image,type,&freeviewPose,&freeviewIntrinsics);
  ray_casting_view_.create(ray_cast_image->noDims.y,ray_cast_image->noDims.x,CV_8UC3);
  Vector4u *ray = ray_cast_image->GetData(MEMORYDEVICE_CPU);

  Eigen::Affine3d cam2world, tam_cam2base;

  cam2world = toEigenMatrix(*main_engine_->GetTrackingState()->pose_d);
  //std::cout<<freeviewPose<<std::endl;
  //std::cout<<*main_engine_->GetTrackingState()->pose_d<<std::endl;

  //tam_cam2base = cam_pose_ * cam2world;
  geometry_msgs::TransformStamped cam_to_base_tf;
  cam_to_base_tf = tf2_buffer_->lookupTransform("base_link",colored_pointcloud_msg->header.frame_id,ros::Time(0));
  Eigen::Affine3d cam_to_base = tf2::transformToEigen(cam_to_base_tf);
  Eigen::Affine3d cam_to_cam0 = cam_pose_.inverse() * cam_to_base;
  robot_poses_.push_back(cam_to_cam0.matrix());
  camera_poses_.push_back(cam2world.matrix().inverse());



  //cam2world = toEigenMatrix(freeviewPose);
  geometry_msgs::TransformStamped pose_tf;
  pose_tf = tf2::eigenToTransform(cam2world.inverse());
  pose_tf.header.stamp = ros::Time(0);
  pose_tf.header.frame_id = "first_frame";
  pose_tf.child_frame_id = "cam_pose";
  tf2_br_->sendTransform(pose_tf);
  std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
  diff_ = std::chrono::duration_cast<std::chrono::seconds>(now - beginning_time_).count();  // in seconds
  if (diff_ % collection_time_ == 0 && (diff_ != prev_diff_))
  {
    texture_images_.push_back(cv_rgb);
    depth_images_.push_back(cv_depth);
    texture_cam2worlds_.push_back(cam_to_cam0.matrix());
    save_frame_idx_++;
    prev_diff_ = diff_;

  }
}

void InfiniTAMROS::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &data)
{
  try
  {
    bool reprojection_matrix_changed = false;
    if (! intrinsic_.isZero() && (
          intrinsic_(0, 0) != data->K[0] ||
          intrinsic_(0, 1) != data->K[1] ||
          intrinsic_(0, 2) != data->K[2] ||
          intrinsic_(1, 0) != data->K[3] ||
          intrinsic_(1, 1) != data->K[4] ||
          intrinsic_(1, 2) != data->K[5] ||
          intrinsic_(2, 0) != data->K[6] ||
          intrinsic_(2, 1) != data->K[7] ||
          intrinsic_(2, 2) != data->K[8])
        )
      reprojection_matrix_changed = true;

    if (intrinsic_.isZero() || reprojection_matrix_changed)
    {
      ROS_INFO("[InfiniTAMROS] Received new camera matrix");
      intrinsic_(0, 0) = data->K[0];
      intrinsic_(1, 1) = data->K[4];
      intrinsic_(0, 2) = data->K[2];
      intrinsic_(1, 2) = data->K[5];
      width_ = data->width;
      height_ = data->height;
      calib_.intrinsics_rgb.SetFrom(data->width,data->height,data->K[0],data->K[4],data->K[2],data->K[5]);
      calib_.intrinsics_d = calib_.intrinsics_rgb;
      calib_.disparityCalib.SetStandard();
      std::cout << "\t... / " << std::setw(8) <<  intrinsic_(0, 0) << " ";
      std::cout << std::setw(8) << intrinsic_(0, 1) << " ";
      std::cout << std::setw(8) << intrinsic_(0, 2) << " \\ " << std::endl;
      std::cout << "\t... | " << std::setw(8) << intrinsic_(1, 0) << " ";
      std::cout << std::setw(8) << intrinsic_(1, 1) << " ";
      std::cout << std::setw(8) << intrinsic_(1, 2) << " | "<< std::endl;;
      std::cout << "\t... \\ " << std::setw(8) << intrinsic_(2, 0) << " ";
      std::cout << std::setw(8) << intrinsic_(2, 1) << " ";
      std::cout << std::setw(8) << intrinsic_(2, 2) << " / "<< std::endl << std::endl;
      ROS_INFO("[InfiniTAMROS] Size of images ");
      std::cout<< "image height : " << height_<<std::endl;
      std::cout<< "image width : " << width_<<std::endl;

    }
    valid_ = true;
  }
  catch ( ros::Exception &e )
  {
    ROS_ERROR("SceneRecontruction::cameraInfoCallback: Error occured: %s ", e.what());
    return;
  }
}

void InfiniTAMROS::publishRayCasting()
{
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", ray_casting_view_).toImageMsg();
  raycast_img_publisher_.publish(msg);
}

void InfiniTAMROS::reset()
{
  diff_ = 0;
  prev_diff_ = 10000;
  texture_cam2worlds_.clear();
  save_frame_idx_ = 0;
  texture_images_.clear();
  depth_images_.clear();
  cam_pose_.setIdentity();
  intrinsic_.setIdentity();
  stop_scanning_ = false;
  start_scanning_ = false;
  frame_idx_=0;
  valid_ = false;
  camera_poses_.clear();
  robot_poses_.clear();
}

Eigen::Affine3d InfiniTAMROS::toEigenMatrix(const ORUtils::SE3Pose& pose)
{
  Eigen::Affine3f eigen_pose;



  eigen_pose.matrix()(0,0) = pose.GetM()(0,0);
  eigen_pose.matrix()(0,1) = pose.GetM()(1,0);
  eigen_pose.matrix()(0,2) = pose.GetM()(2,0);
  eigen_pose.matrix()(0,3) = pose.GetM()(3,0);
  eigen_pose.matrix()(1,0) = pose.GetM()(0,1);
  eigen_pose.matrix()(1,1) = pose.GetM()(1,1);
  eigen_pose.matrix()(1,2) = pose.GetM()(2,1);
  eigen_pose.matrix()(1,3) = pose.GetM()(3,1);
  eigen_pose.matrix()(2,0) = pose.GetM()(0,2);
  eigen_pose.matrix()(2,1) = pose.GetM()(1,2);
  eigen_pose.matrix()(2,2) = pose.GetM()(2,2);
  eigen_pose.matrix()(2,3) = pose.GetM()(3,2);
  eigen_pose.matrix()(3,0) = pose.GetM()(0,3);
  eigen_pose.matrix()(3,1) = pose.GetM()(1,3);
  eigen_pose.matrix()(3,2) = pose.GetM()(2,3);
  eigen_pose.matrix()(3,3) = pose.GetM()(3,3);
  return  eigen_pose.cast<double>();


}
/*
void InfiniTAMROS::extractPointCloud( const ITMMesh::Triangle& triangle_array,
                                      pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_pcl
                                      )
{
  ROS_INFO("PCL PointCloud extraction from ITMMesh started.");

  std::size_t nr_triangles = 0u;
  std::size_t nr_points = 0u;
  main_engine_->ITMMainEngine
  nr_triangles = main_engine_->GetMesh()->noTotalTriangles;
  main_engine_->get
  // Infinitam stores the mesh as a set of triangles,
  // number of points is 3x number of triangles. See ITMMesh.h
  nr_points = nr_triangles * 3u;

  // Point cloud has at least 3 points per triangle.
  point_cloud_pcl->width = nr_points;
  point_cloud_pcl->height = 1u;
  point_cloud_pcl->is_dense = true;
  point_cloud_pcl->points.resize(point_cloud_pcl->width *
                                 point_cloud_pcl->height);

  ROS_ERROR_COND(main_engine_->GetMesh()->noTotalTriangles < 1u,
                 "The mesh has too few triangles, only: %d",
                 main_engine_->GetMesh()->noTotalTriangles);

  std::size_t point_number = 0u;

  // All vertices of the mesh are stored in the PCL point cloud.
  for (std::size_t i = 0u; i < nr_triangles; ++i) {
    point_cloud_pcl->points[point_number].x = (&triangle_array)[i].p0.x;
    point_cloud_pcl->points[point_number].y = (&triangle_array)[i].p0.y;
    point_cloud_pcl->points[point_number].z = (&triangle_array)[i].p0.z;
    ++point_number;
    point_cloud_pcl->points[point_number].x = (&triangle_array)[i].p1.x;
    point_cloud_pcl->points[point_number].y = (&triangle_array)[i].p1.y;
    point_cloud_pcl->points[point_number].z = (&triangle_array)[i].p1.z;
    ++point_number;
    point_cloud_pcl->points[point_number].x = (&triangle_array)[i].p2.x;
    point_cloud_pcl->points[point_number].y = (&triangle_array)[i].p2.y;
    point_cloud_pcl->points[point_number].z = (&triangle_array)[i].p2.z;
    ++point_number;
  }
  ROS_INFO("PCL PointCloud extraction from ITMMesh ended.");
}
*/
bool InfiniTAMROS::startScanning(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp)
{
  if (stop_scanning_)
  {
    ROS_WARN("The program is being processed, could not start a new scanning program");
    resp.success = true;
    return true;
  }
  else
  {
    start_scanning_ = true;
    resp.success = true;
  }
  return true;
}
bool InfiniTAMROS::stopScanning(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp)
{
  // stop_scanning_ = true;
  if (!start_scanning_)
  {
    ROS_WARN("The program is not started, could stop program");
    stop_scanning_ = false;
  }
  else
  {
    stop_scanning_ = true;
  }

  resp.success = true;
  return true;
}
int main(int argc, char** argv)
{

  ros::init(argc, argv, "inifinitam_ros_wrapper");
  ros::NodeHandle node;
  auto infinitam_ = std::make_shared<InfiniTAMROS>(node);
  ros::spin();


  //  open3d::geometry::TriangleMesh mesh;
  //  open3d::io::ReadTriangleMesh("/home/manhha/catkin_ws/data_inifitam/mesh.obj",mesh);
  //  std::cout<<mesh.triangles_.size()<<std::endl;
  //  std::cout<<mesh.vertices_.size()<<std::endl;
  //  mesh.RemoveDuplicatedVertices();
  //  mesh.RemoveDuplicatedTriangles();
  //  mesh.RemoveDegenerateTriangles();
  //  mesh.RemoveUnreferencedVertices();
  //  std::cout<<mesh.triangles_.size()<<std::endl;
  //  std::cout<<mesh.vertices_.size()<<std::endl;
  //  open3d::io::WriteTriangleMesh("open3d_mesh.ply",mesh);

}
