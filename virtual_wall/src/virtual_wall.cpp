#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <time.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <dirent.h>
#include <atomic>
#include <vector>
#include <map>
#include <algorithm>

// map check
#include "nav_msgs/msg/occupancy_grid.hpp"
// std
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_srvs/srv/empty.hpp"
// geometry msg
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/point.hpp"
// sensor msg
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

// tf2
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// Virtual Obstacles Service
#include "virtual_wall/srv/add_virtual_wall.hpp"

using namespace std;
using std::placeholders::_1;
using namespace std::chrono_literals;

// Active map check flag
bool m_bActive_map = false;

// CALC TF distance data
double m_dTF_calc_poseX = 0.0;
double m_dTF_calc_poseY = 0.0;
double m_dTF_calc_theta = 0.0;

typedef struct TF_POSE2
{
	double poseTFx = 0.0;
	double poseTFy = 0.0;
	double poseTFz = 0.0;
	double poseTFqx = 0.0;
	double poseTFqy = 0.0;
	double poseTFqz = 0.0;
	double poseTFqw = 1.0;

	double pose_euler_roll = 0.0;
	double pose_euler_pitch = 0.0;
	double pose_euler_yaw = 0.0;
	
} TF_POSE2;
TF_POSE2 _pTF_pose2;

// Virtual polygons
typedef struct VIRTUAL_POLYGON
{
	geometry_msgs::msg::Polygon polygons[255];
	int iPolygon_size = 0;
	int iID[255] = {0,};
	int iID_array_size = 0;
	bool remove = false;
	bool noEmpty = true;
    
} VIRTUAL_POLYGON;
VIRTUAL_POLYGON _pVirtual_polygon;

std::atomic<bool> stop_requested(false);
void signal_handler(int signal) 
{
    stop_requested = true;
    printf("Signal received: %d. Preparing to shutdown...\n", signal);
}

class VIRTUAL_WALL: public rclcpp::Node
{
public:
	VIRTUAL_WALL() : Node("virtual_wall")
	{
		// Initialize the TF2 buffer and listener
        tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

		// virtual obstacle publisher
		global_virtual_obstacles_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("Global_virtual_wall", 10);
		local_virtual_obstacles_publisher  = this->create_publisher<sensor_msgs::msg::PointCloud2>("Local_virtual_wall", 10);

		// subscribe
		map_subscriber = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
			"map", rclcpp::SensorDataQoS(), std::bind(&VIRTUAL_WALL::map_Callback, this, _1));

		// Service list
		virtual_obstacles_cmd_srv = create_service<virtual_wall::srv::AddVirtualWall>(
            "add_virtual_wall_cmd",
        std::bind(&VIRTUAL_WALL::Add_Virtual_Obstacle_Command, this, std::placeholders::_1, std::placeholders::_2));
		
		// map -> odom TF CALC Timer
		CALC_TF_timer_ = this->create_wall_timer(1000ms, std::bind(&VIRTUAL_WALL::CALC_TF_Timer, this)); // 1sec
	}

	rclcpp::Time current_time;
	rclcpp::TimerBase::SharedPtr CALC_TF_timer_;
    std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

	// virtual_obstacles_publisher
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_virtual_obstacles_publisher;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr local_virtual_obstacles_publisher;
	std::map<int, sensor_msgs::msg::PointCloud2> polygon_global_costmap;
	std::map<int, sensor_msgs::msg::PointCloud2> polygon_local_costmap;

	// Subscription
	rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscriber;

	// Subscribe Callback Function
	void map_Callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
	{
		if(msg->data.size() > 0)
		{
			m_bActive_map = true;
			polygon_global_costmap.clear();
			polygon_local_costmap.clear();
		}
		else
		{
			m_bActive_map = false;
			printf(" No Active_map !! \n");
		}
	}

	// CALC TF odom Timer Loop
	void CALC_TF_Timer()
	{
		if(m_bActive_map && _pVirtual_polygon.noEmpty)
		{
			try 
			{
				geometry_msgs::msg::TransformStamped transformStamped;
				transformStamped = tf2_buffer_->lookupTransform("map", "odom", tf2::TimePointZero);

				_pTF_pose2.poseTFx = transformStamped.transform.translation.x;
				_pTF_pose2.poseTFy = transformStamped.transform.translation.y;
				_pTF_pose2.poseTFz = transformStamped.transform.translation.z;

				_pTF_pose2.poseTFqx = transformStamped.transform.rotation.x;
				_pTF_pose2.poseTFqy = transformStamped.transform.rotation.y;
				_pTF_pose2.poseTFqz = transformStamped.transform.rotation.z;
				_pTF_pose2.poseTFqw = transformStamped.transform.rotation.w;

				tf2::Quaternion q2;
				q2.setW(transformStamped.transform.rotation.w);
				q2.setX(transformStamped.transform.rotation.x);
				q2.setY(transformStamped.transform.rotation.y);
				q2.setZ(transformStamped.transform.rotation.z);

				tf2::Matrix3x3 m(q2);
				m.getRotation(q2);
				m.getRPY(_pTF_pose2.pose_euler_roll, _pTF_pose2.pose_euler_pitch, _pTF_pose2.pose_euler_yaw);
				m_dTF_calc_theta = _pTF_pose2.pose_euler_yaw;
				m_dTF_calc_poseX = (((_pTF_pose2.poseTFx * cos(m_dTF_calc_theta)) + (_pTF_pose2.poseTFy * sin(m_dTF_calc_theta))));
				m_dTF_calc_poseY = (((_pTF_pose2.poseTFx * -sin(m_dTF_calc_theta)) + (_pTF_pose2.poseTFy * cos(m_dTF_calc_theta))));

				int id = 0;
				if (_pVirtual_polygon.remove)
				{
					id = _pVirtual_polygon.iID_array_size;
					(void)id;
				}
				else
				{
					polygon_local_costmap.clear();
					for (size_t i = 0; i < (size_t)_pVirtual_polygon.iPolygon_size; ++i) 
					{
						const auto& polygon = _pVirtual_polygon.polygons[i];
						id = _pVirtual_polygon.iID[i];

						std::vector<geometry_msgs::msg::Point> points;
						for (const auto& point32 : polygon.points) 
						{
							geometry_msgs::msg::Point point;
							point.x = point32.x;
							point.y = point32.y;
							point.z = point32.z;
							points.push_back(point);
						}
						auto dense_points2 = generateDensePoints2(points, 0.05);
						auto cloud_msg2 = createPointCloud2(dense_points2);
						polygon_local_costmap[id] = cloud_msg2;
					}
				}

				sensor_msgs::msg::PointCloud2 combined_cloud2;
				combined_cloud2 = sensor_msgs::msg::PointCloud2();
				for (const auto& [cid, cloud2] : polygon_local_costmap) 
				{
					(void)cid;
					mergePointClouds(combined_cloud2, cloud2);
				}

				if (polygon_local_costmap.empty()) 
				{
					polygon_local_costmap.clear();
					combined_cloud2 = sensor_msgs::msg::PointCloud2();
					combined_cloud2.data.clear();
					combined_cloud2.fields.clear();
					combined_cloud2.header.frame_id = "map";
					combined_cloud2.header.stamp = this->now();

					ensurePointCloudFields(combined_cloud2);
					local_virtual_obstacles_publisher->publish(combined_cloud2);
				}
				else
				{
					local_virtual_obstacles_publisher->publish(combined_cloud2);
				}
			} 
			catch (tf2::TransformException &ex) 
			{
				// Transform may be temporarily unavailable
			}
		}
	}

	// Global costmap dense points
	std::vector<geometry_msgs::msg::Point> generateDensePoints(const std::vector<geometry_msgs::msg::Point>& vertices, double resolution) 
	{
		std::vector<geometry_msgs::msg::Point> dense_points;
		if (vertices.size() < 4) 
		{
			RCLCPP_ERROR(rclcpp::get_logger("[Global]virtual_obstacle_service"), "Insufficient vertices for a rectangle.");
			return dense_points;
		}
		auto bottom_left = vertices[0];
		auto bottom_right = vertices[1];
		auto top_right = vertices[2];
		auto top_left = vertices[3];

		double min_x = std::min(bottom_left.x, top_left.x);
		double max_x = std::max(bottom_right.x, top_right.x);
		double min_y = std::min(bottom_left.y, bottom_right.y);
		double max_y = std::max(top_left.y, top_right.y);

		for (double x = min_x; x <= max_x; x += resolution) 
		{
			for (double y = min_y; y <= max_y; y += resolution) 
			{
				geometry_msgs::msg::Point point;
				point.x = x;
				point.y = y;
				point.z = 0.0;
				dense_points.push_back(point);
			}
		}
		return dense_points;
	}

	// Local costmap dense points
	std::vector<geometry_msgs::msg::Point> generateDensePoints2(const std::vector<geometry_msgs::msg::Point>& vertices, double resolution) 
	{
		std::vector<geometry_msgs::msg::Point> dense_points2;

		if (vertices.size() < 4) 
		{
			RCLCPP_ERROR(rclcpp::get_logger("[Local]virtual_obstacle_service"), "Insufficient vertices for a rectangle.");
			return dense_points2;
		}
		auto bottom_left = vertices[0];
		auto bottom_right = vertices[1];
		auto top_right = vertices[2];
		auto top_left = vertices[3];

		double min_x = std::min(bottom_left.x, top_left.x);
		double max_x = std::max(bottom_right.x, top_right.x);
		double min_y = std::min(bottom_left.y, bottom_right.y);
		double max_y = std::max(top_left.y, top_right.y);

		for (double x = min_x; x <= max_x; x += resolution) 
		{
			for (double y = min_y; y <= max_y; y += resolution) 
			{
				geometry_msgs::msg::Point point;
				point.x = x;
				point.y = y;
				point.z = 0.0;
				dense_points2.push_back(point);
			}
		}
		return dense_points2;
	}

	// Global costmap point cloud
	sensor_msgs::msg::PointCloud2 createPointCloud(const std::vector<geometry_msgs::msg::Point>& dense_points) 
	{
		sensor_msgs::msg::PointCloud2 cloud_msg;
		cloud_msg.header.frame_id = "map";
		cloud_msg.header.stamp = rclcpp::Clock().now();
		cloud_msg.height = 1;
		cloud_msg.width = dense_points.size();
		cloud_msg.is_bigendian = false;
		cloud_msg.is_dense = true;

		sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
		modifier.setPointCloud2FieldsByString(1, "xyz");

		sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
		sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
		sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
		for (const auto& point : dense_points) 
		{
			*iter_x = point.x;
			*iter_y = point.y;
			*iter_z = point.z;
			++iter_x; ++iter_y; ++iter_z;
		}
		return cloud_msg;
	}

	// Local costmap point cloud
	sensor_msgs::msg::PointCloud2 createPointCloud2(const std::vector<geometry_msgs::msg::Point>& dense_points) 
	{
		sensor_msgs::msg::PointCloud2 cloud_msg2;
		cloud_msg2.header.frame_id = "map";
		cloud_msg2.header.stamp = rclcpp::Clock().now();

		cloud_msg2.height = 1;
		cloud_msg2.width = dense_points.size();
		cloud_msg2.is_bigendian = false;
		cloud_msg2.is_dense = true;

		sensor_msgs::PointCloud2Modifier modifier2(cloud_msg2);
		modifier2.setPointCloud2FieldsByString(1, "xyz");

		sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg2, "x");
		sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg2, "y");
		sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg2, "z");

		for (const auto& point : dense_points) 
		{
			*iter_x = point.x;
			*iter_y = point.y;
			*iter_z = point.z;
			++iter_x; ++iter_y; ++iter_z;
		}

		return cloud_msg2;
	}

	// Merge Pointcloud
	void mergePointClouds(sensor_msgs::msg::PointCloud2 &combined_cloud, const sensor_msgs::msg::PointCloud2 &new_cloud) 
	{
		if (combined_cloud.data.empty()) 
		{
			combined_cloud = new_cloud;
			return;
		}
		if (combined_cloud.fields.size() != new_cloud.fields.size()) 
		{
			throw std::runtime_error("PointCloud2 fields do not match for merging.");
		}
		for (size_t i = 0; i < combined_cloud.fields.size(); ++i) 
		{
			if (combined_cloud.fields[i].name != new_cloud.fields[i].name ||
				combined_cloud.fields[i].datatype != new_cloud.fields[i].datatype) 
			{
				throw std::runtime_error("PointCloud2 field names or types do not match for merging.");
			}
		}
		combined_cloud.data.insert(combined_cloud.data.end(), new_cloud.data.begin(), new_cloud.data.end());
		combined_cloud.width += new_cloud.width;
		combined_cloud.row_step = combined_cloud.width * combined_cloud.point_step;
	}

	// Ensure PointCloud2 has valid fields
	void ensurePointCloudFields(sensor_msgs::msg::PointCloud2 &cloud)
	{
		sensor_msgs::msg::PointField field_x;
		field_x.name = "x";
		field_x.offset = 0;
		field_x.datatype = sensor_msgs::msg::PointField::FLOAT32;
		field_x.count = 1;
		sensor_msgs::msg::PointField field_y = field_x;
		field_y.name = "y";
		field_y.offset = 4;
		sensor_msgs::msg::PointField field_z = field_x;
		field_z.name = "z";
		field_z.offset = 8;

		cloud.fields = {field_x, field_y, field_z};
		cloud.point_step = 12;
		cloud.is_dense = false;
	}

	// virtual obstacle Callback
	void Add_Virtual_Obstacle_Command(
		const std::shared_ptr<virtual_wall::srv::AddVirtualWall::Request>  request, 
		const std::shared_ptr<virtual_wall::srv::AddVirtualWall::Response> response)
	{
		_pVirtual_polygon.remove = request->remove;

		if (request->remove)
		{
			_pVirtual_polygon.iID_array_size = request->ids.size();
			for (const auto& id : request->ids)
			{
				if (polygon_global_costmap.erase(id) > 0) 
				{
					RCLCPP_INFO(this->get_logger(), "Global Virtual_costmap_Polygon with ID %d deleted.", id);
				} 
				else 
				{
					RCLCPP_WARN(this->get_logger(), "Global Virtual_costmap_Polygon with ID %d not found.", id);
				}

				if (polygon_local_costmap.erase(id) > 0) 
				{
					RCLCPP_INFO(this->get_logger(), "Local Virtual_costmap_Polygon with ID %d deleted.", id);
				} 
				else 
				{
					RCLCPP_WARN(this->get_logger(), "Local Virtual_costmap_Polygon with ID %d not found.", id);
				}
			}
			response->success = true;
			response->message = "Virtual Polygons deleted successfully.";
		} 
		else 
		{
			polygon_global_costmap.clear();
			_pVirtual_polygon.iPolygon_size = request->polygons.size();
			for (size_t i = 0; i < request->polygons.size(); ++i) 
			{
				_pVirtual_polygon.polygons[i] = request->polygons[i];
				_pVirtual_polygon.iID[i] = request->ids[i];
				const auto& polygon = request->polygons[i];
				int id = request->ids[i];
				std::vector<geometry_msgs::msg::Point> points;
				for (const auto& point32 : polygon.points) 
				{
					geometry_msgs::msg::Point point;
					point.x = point32.x;
					point.y = point32.y;
					point.z = point32.z;
					points.push_back(point);
				}
				auto dense_points = generateDensePoints(points, 0.05);
				auto cloud_msg = createPointCloud(dense_points);
				polygon_global_costmap[id] = cloud_msg;
			}

			response->success = true;
			response->message = "Virtual Polygons added successfully.";
		}

		sensor_msgs::msg::PointCloud2 combined_cloud;
		combined_cloud = sensor_msgs::msg::PointCloud2();
		for (const auto& [cid, cloud] : polygon_global_costmap) 
		{
			(void)cid;
			mergePointClouds(combined_cloud, cloud);
		}

		if (polygon_global_costmap.empty()) 
		{
			polygon_global_costmap.clear();
			combined_cloud = sensor_msgs::msg::PointCloud2();
			combined_cloud.data.clear();
			combined_cloud.fields.clear();
			combined_cloud.header.frame_id = "map";  
			combined_cloud.header.stamp = this->now();

			ensurePointCloudFields(combined_cloud);
			global_virtual_obstacles_publisher->publish(combined_cloud);
			_pVirtual_polygon.noEmpty = false;
		}
		else
		{
			global_virtual_obstacles_publisher->publish(combined_cloud);
			_pVirtual_polygon.noEmpty = true;
		}
	}

private:
	rclcpp::Service<virtual_wall::srv::AddVirtualWall>::SharedPtr virtual_obstacles_cmd_srv;
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	std::signal(SIGINT, signal_handler);
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	auto node = std::make_shared<VIRTUAL_WALL>();
	node->current_time = node->now();
	rclcpp::WallRate loop_rate(30);

	while (rclcpp::ok() && !stop_requested)
	{
		rclcpp::spin_some(node);
		loop_rate.sleep();
    }

	rclcpp::shutdown();
    return 0;
}
