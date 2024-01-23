#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
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

      pointcloud_pub_ = pnh.advertise<sensor_msgs::PointCloud2>("test_pointcloud", 1, true);
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
	    // translation
	    auto translation_ = info->extract("translation");
	    auto& translationTmp = *translation_->toListing();
	    cnoid::Vector3 translation = cnoid::Vector3(translationTmp[0].toDouble(), translationTmp[1].toDouble(), translationTmp[2].toDouble());
	    sensor.translation = translation;
	    // rotation
	    auto rotation_ = info->extract("rotation");
	    auto& rotationTmp = *rotation_->toListing();
	    cnoid::Matrix3d rotation;
	    rotation << rotationTmp[0].toDouble(), rotationTmp[1].toDouble(), rotationTmp[2].toDouble(),
	      rotationTmp[3].toDouble(), rotationTmp[4].toDouble(), rotationTmp[5].toDouble(),
	      rotationTmp[6].toDouble(), rotationTmp[7].toDouble(), rotationTmp[8].toDouble();
	    this->tactileSensorList_.push_back(sensor);
	  }
	}
      }
    }

    void topicCallback(const sensor_msgs::PointCloud2::Ptr& msg)
    {
    }

  protected:
    ros::NodeHandle nh_;

    ros::Publisher pointcloud_pub_;
    ros::Subscriber tactile_sub_;
    std::string output_tsdf_layer_;
    std::string config_filename_;
    class TactileSensor
    {
    public:
      std::string linkName;
      cnoid::Vector3 translation; // リンク座標系でどこに取り付けられているか                                                                                                                                                                          
    };
    std::vector<TactileSensor> tactileSensorList_;
  };
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tactile_sensor_voxblox_converter");
  tactile_sensor_voxblox_converter::tactile_sensor_voxblox_converter t;
  ros::spin();
}
