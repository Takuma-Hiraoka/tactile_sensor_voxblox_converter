#include <ros/ros.h>
#include <algorithm>
#include <memory>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32MultiArray.h>
#include <voxblox/utils/color_maps.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox/integrator/tsdf_integrator.h>
#include <voxblox_ros/conversions.h>
#include <voxblox_ros/ros_params.h>
#include <voxblox_msgs/Layer.h>
#include <minkindr_conversions/kindr_tf.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <cnoid/EigenTypes>
#include <cnoid/YAMLReader>

namespace tactile_sensor_voxblox_converter {
  class tactile_sensor_voxblox_converter
  {
  public:
    tactile_sensor_voxblox_converter()
      : tfListener_(tfBuffer_)
    {
      ros::NodeHandle pnh("~");

      pnh.param("world_frame", world_frame_,
                std::string("odom_ground"));
      pnh.param("config_filename", config_filename_,
                std::string(""));
      pnh.param("debug", debug_,
                0);

      tsdf_map_pub_ = pnh.advertise<voxblox_msgs::Layer>("output", 1, false);
      tactile_sub_ = pnh.subscribe("input", 1, &tactile_sensor_voxblox_converter::topicCallback, this);
      color_map_.reset(new voxblox::GrayscaleColorMap());
      tsdf_map_.reset(new voxblox::TsdfMap(voxblox::getTsdfMapConfigFromRosParam(pnh)));
      tsdf_integrator_.reset(new voxblox::FastTsdfIntegrator(voxblox::getTsdfIntegratorConfigFromRosParam(pnh), tsdf_map_->getTsdfLayerPtr()));

      // load tactile_sensor_file
      {
	ROS_INFO("load %s", config_filename_.c_str());
	cnoid::YAMLReader reader;
	cnoid::MappingPtr node;
	try {
	  node = reader.loadDocument(config_filename_)->toMapping();
	} catch(const cnoid::ValueNode::Exception& ex) {
	  ROS_ERROR_STREAM(ex.message());
	}
	// load
	auto& tactileSensorList = *node->findListing("tactile_sensor");
	if (!tactileSensorList.isValid()) {
	  ROS_ERROR_STREAM("cannot load config file");
	  return;
	} else {
	  for (int i=0; i< tactileSensorList.size(); i++) {
	    cnoid::Mapping* info = tactileSensorList[i].toMapping();
	    TactileSensor sensor;
	    // link
	    std::string linkName;
	    info->extract("link", linkName);
	    sensor.linkName = linkName;
	    // ポイントクラウド用にリンク名を保存
	    auto result = std::find(sensorFrames_.begin(), sensorFrames_.end(), linkName);
	    if (result == sensorFrames_.end()) {
	      sensorFrames_.push_back(linkName);
	    }
	    // translation
	    auto translation_ = info->extract("translation");
	    auto& translationTmp = *translation_->toListing();
	    cnoid::Vector3 translation = cnoid::Vector3(translationTmp[0].toDouble(), translationTmp[1].toDouble(), translationTmp[2].toDouble());
	    sensor.translation = translation;
	    this->tactileSensorList_.push_back(sensor);
	  }
	}
      }
      pointcloud_pubs_.resize(sensorFrames_.size());
      for (int i=0; i< sensorFrames_.size(); i++) {
	pointcloud_pubs_[i] = pnh.advertise<sensor_msgs::PointCloud2>("tactile_pointcloud_" + sensorFrames_[i], 10);
      }
    }

    void topicCallback(const std_msgs::Float32MultiArray& msg)
    {
      if(msg.data.size() != this->tactileSensorList_.size()*3) {
	ROS_ERROR_STREAM("data length is different. port data length : " << msg.data.size() << " config file sensor length : " << this->tactileSensorList_.size()*3);
	return;
      }
      std::vector<pcl::PointCloud<pcl::PointXYZ>> clouds(sensorFrames_.size()); // sensorFrames_ のリンク名の順番
      for (int i=0; i<this->tactileSensorList_.size(); i++) {
	cnoid::Vector3 force(msg.data[3*i + 0], msg.data[3*i + 1], msg.data[3*i + 2]);
	if (force.norm() > contact_threshould_) {
	  pcl::PointXYZ point(tactileSensorList_[i].translation[0], tactileSensorList_[i].translation[1], tactileSensorList_[i].translation[2]);
	  auto itr = std::find(sensorFrames_.begin(), sensorFrames_.end(), tactileSensorList_[i].linkName);
	  int index = std::distance(sensorFrames_.begin(), itr);
	  clouds[index].points.push_back(point);
	}
      }

      // cloudsを整形してPointCloud2へ
      for (int i=0; i < clouds.size(); i++) {
	clouds[i].width = clouds[i].points.size();
	clouds[i].height = 1;
	// デバッグ用publish
	if (debug_ >= 1) {
	  sensor_msgs::PointCloud2 cloud_msg;
	  pcl::toROSMsg(clouds[i], cloud_msg);
	  cloud_msg.header.frame_id = sensorFrames_[i];
	  pointcloud_pubs_[i].publish(cloud_msg);
	}
      }

      // convertion. see https://github.com/ethz-asl/voxblox/blob/master/voxblox_ros/src/tsdf_server.cc#L213
      for (int i=0; i < clouds.size(); i++) {
	voxblox::Pointcloud points_C;
	voxblox::Colors colors;
	voxblox::convertPointcloud(clouds[i], color_map_, &points_C, &colors);
	geometry_msgs::TransformStamped trans;
	try{
        trans = tfBuffer_.lookupTransform(world_frame_, sensorFrames_[i], ros::Time(0), ros::Duration(3));
	} catch (std::exception& ex) {
	  ROS_ERROR_STREAM(ex.what());
	  return;
	}
	voxblox::Transformation T_G_C;
	tf::transformTFToKindr(tf::Transform(tf::Quaternion(trans.transform.rotation.x,
							    trans.transform.rotation.y,
							    trans.transform.rotation.z,
							    trans.transform.rotation.w),
					     tf::Vector3(trans.transform.translation.x,
							 trans.transform.translation.y,
							 trans.transform.translation.z)),
			       &T_G_C);
	tsdf_integrator_->integratePointCloud(T_G_C, points_C, colors, false); // is_freespace_pointcloud
      }

      // publish
      voxblox_msgs::Layer layer_msg;
      voxblox::serializeLayerAsMsg<voxblox::TsdfVoxel>(this->tsdf_map_->getTsdfLayer(),
						       true, &layer_msg, voxblox::MapDerializationAction::kMerge);
      this->tsdf_map_pub_.publish(layer_msg);
    }

  protected:
    ros::NodeHandle nh_;

    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;
    std::vector<ros::Publisher> pointcloud_pubs_;
    ros::Publisher tsdf_map_pub_;
    ros::Subscriber tactile_sub_;
    std::string world_frame_;
    std::string config_filename_;
    double contact_threshould_ = 2;
    int debug_ = 0;

    class TactileSensor
    {
    public:
      std::string linkName;
      cnoid::Vector3 translation; // リンク座標系でどこに取り付けられているか
    };
    std::vector<TactileSensor> tactileSensorList_;
    std::vector<std::string> sensorFrames_;
    std::shared_ptr<voxblox::ColorMap> color_map_;
    std::shared_ptr<voxblox::TsdfMap> tsdf_map_;
    std::unique_ptr<voxblox::TsdfIntegratorBase> tsdf_integrator_;
  };
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tactile_sensor_voxblox_converter");
  tactile_sensor_voxblox_converter::tactile_sensor_voxblox_converter t;
  ros::spin();
}
