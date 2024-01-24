#include <ros/ros.h>
#include <algorithm>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32MultiArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cnoid/EigenTypes>
#include <cnoid/YAMLReader>

namespace tactile_sensor_voxblox_converter {
  class tactile_sensor_voxblox_converter
  {
  public:
    tactile_sensor_voxblox_converter()
    {
      ros::NodeHandle pnh("~");

      pnh.param("output", output_tsdf_layer_,
                std::string(""));
      pnh.param("config_filename", config_filename_,
                std::string(""));
      pnh.param("debug", debug_,
                0);

      tactile_sub_ = pnh.subscribe("input", 1, &tactile_sensor_voxblox_converter::topicCallback, this);

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
      std::vector<sensor_msgs::PointCloud2> cloud_msgs;
      for (int i=0; i < clouds.size(); i++) {
	clouds[i].width = clouds[i].points.size();
	clouds[i].height = 1;
	sensor_msgs::PointCloud2 cloud_msg;
	pcl::toROSMsg(clouds[i], cloud_msg);
	cloud_msg.header.frame_id = sensorFrames_[i];
	cloud_msgs.push_back(cloud_msg);
      }

      // デバッグ用publish
      if (debug_ >= 1) {
	for (int i=0; i< pointcloud_pubs_.size(); i++) {
	  pointcloud_pubs_[i].publish(cloud_msgs[i]);
	}
      }
    }

  protected:
    ros::NodeHandle nh_;

    std::vector<ros::Publisher> pointcloud_pubs_;
    ros::Subscriber tactile_sub_;
    std::string output_tsdf_layer_;
    std::string config_filename_;
    double contact_threshould_ = 5;
    int debug_ = 0;
    class TactileSensor
    {
    public:
      std::string linkName;
      cnoid::Vector3 translation; // リンク座標系でどこに取り付けられているか
    };
    std::vector<TactileSensor> tactileSensorList_;
    std::vector<std::string> sensorFrames_;
  };
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tactile_sensor_voxblox_converter");
  tactile_sensor_voxblox_converter::tactile_sensor_voxblox_converter t;
  ros::spin();
}
