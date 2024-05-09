#include <iostream>
#include <cmath>

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

#define root2 1.41421
#define pi 3.141592
#define rootpi 1.772453 
#define root2pi 2.506628
#define _root2pi 0.398942

namespace std {

class vector2d {

public:
	vector2d();
	vector2d(double x, double y);

	double x_component;
	double y_component;

public:
	double get_length();
	void normalize();
	vector2d operator+(const vector2d&);
	vector2d operator-(const vector2d&);
	vector2d operator*(const vector2d&); // dot product
};

}

namespace std {

vector2d::vector2d() {
	this->x_component = static_cast<double>(0.0);
	this->y_component = static_cast<double>(0.0);
}

vector2d::vector2d(double x, double y) {
	this->x_component = x;
	this->y_component = y;
}

double vector2d::get_length() {
	return std::sqrt(this->x_component * this->x_component + 
		this->y_component * this->y_component);
}

void vector2d::normalize() {
	double length = get_length();
	if (length != 0.0) {
		this->x_component /= length;
		this->y_component /= length;
	}
}

vector2d vector2d::operator+(const vector2d& other) {
	return vector2d(this->x_component + other.x_component, 
		this->y_component + other.y_component);
}

vector2d vector2d::operator-(const vector2d& other) {
	return vector2d(this->x_component - other.x_component, 
		this->y_component - other.y_component);
}

vector2d vector2d::operator*(const vector2d& other) {
	return vector2d(this->x_component * other.x_component,
		this->y_component * other.y_component);
}

}

double normal_potential(double dx, double dy, double s) {
	double exponent_term = -0.5 * (dx*dx + dy*dy) / (s * s);
	double exp_term = std::exp(exponent_term);
	return _root2pi / s * exp_term;
}

std::vector2d normal_grad(double dx, double dy, double s) {
	double exponent_term = -0.5 * (dx*dx + dy*dy) / (s * s);
	double exp_term = std::exp(exponent_term);
	double _dx = - dx / (s * s);
	double _dy = - dy / (s * s);
	double x_derivative = _root2pi / s * exp_term * _dx; 
	double y_derivative = _root2pi / s * exp_term * _dy; 
	return std::vector2d(x_derivative, y_derivative);
}

class potential_fields {

public:
	
	potential_fields(int32_t argc, char **argv) {
		ros::init(argc, argv, "potential_fields");
		ros::NodeHandle node;

		ros::param::get("/topic/map/map", this->map_topic);
		ros::param::get("/topic/odom", this->odom_topic);

		ros::param::get("/frame/odom_frame", this->odom_frame);

		ros::param::get("/goal_params/deviation/obstacle", this->ob_std);
		ros::param::get("/goal_params/deviation/bot", this->bot_std);
		ros::param::get("/goal_params/amplitude/obstacle", this->ob_amp);
		ros::param::get("/goal_params/amplitude/bot", this->bot_amp);
		ros::param::get("/goal_params/force_rate", this->force_rate);
		ros::param::get("/goal_params/start_distance", this->start_distance);
		ros::param::get("/goal_params/distance_gap", this->distance_gap);
		ros::param::get("/goal_params/queue_size", this->queue_size);
		ros::param::get("/goal_params/roi_of_compute/roi_height", this->roi_height);
		ros::param::get("/goal_params/roi_of_compute/roi_width", this->roi_width);

		this->map_sub = node.subscribe(this->map_topic, 1, &potential_fields::map_callback, this);
		this->odom_sub = node.subscribe(this->odom_topic, 1, &potential_fields::odom_callback, this);

		this->field_pub = node.advertise<sensor_msgs::PointCloud>("/field", 1, true);
		this->goal_pub = node.advertise<visualization_msgs::Marker>("/goal", 1, true);
		this->actual_goal_pub = node.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, true);

		this->got_map = false;
		this->got_odom = false;
		this->got_field_info = false;
		this->first_time = true;

		this->goal.header.frame_id = this->odom_frame;
		this->goal.ns = "goal";
		this->goal.id = 0;
		this->goal.type = visualization_msgs::Marker::SPHERE;
		this->goal.action = visualization_msgs::Marker::ADD;
		this->goal.color.r = 1;
		this->goal.color.g = 1;
		this->goal.color.b = 1;
		this->goal.color.a = 1;

		this->goal.scale.x = 2;
		this->goal.scale.y = 2;
		this->goal.scale.z = 2;

		this->goal.pose.orientation.x = 0;
		this->goal.pose.orientation.y = 0;
		this->goal.pose.orientation.z = 0;
		this->goal.pose.orientation.w = 1;

	}

private:

	visualization_msgs::Marker goal;

	std::string map_topic;
	std::string odom_topic;

	std::string odom_frame;

	// goal params
	float ob_std;
	float bot_std;

	float ob_amp;
	float bot_amp;

	float force_rate;
	float start_distance;
	float distance_gap;
	int32_t queue_size;

	int32_t roi_height;
	int32_t roi_width;

	ros::Subscriber map_sub;
	ros::Subscriber odom_sub;

	ros::Publisher field_pub;
	ros::Publisher goal_pub;
	ros::Publisher actual_goal_pub;

	nav_msgs::OccupancyGrid map;
	nav_msgs::Odometry odom;

	bool got_map;
	bool got_odom;
	bool got_field_info;
	bool first_time;

	std::vector<std::vector2d> poses;
	std::vector2d prev_pose;

public:

	void run() {
		ros::AsyncSpinner spinner(4);
		spinner.start();

		ros::Rate rate(60);

		while (ros::ok()) {

			if (!(this->got_map && this->got_odom)) {
				std::cout << "MAP|ODOM" << this->got_map << this->got_odom << std::endl;
				continue;
			}

			if (this->first_time) {
				this->first_time = false;
				double steer = 2 * std::atan2(this->odom.pose.pose.orientation.w, this->odom.pose.pose.orientation.z);
				this->goal.pose.position.x = this->odom.pose.pose.position.x - std::cos(steer) * this->start_distance;
				this->goal.pose.position.y = this->odom.pose.pose.position.y - std::sin(steer) * this->start_distance;
				this->goal.pose.position.z = 0;
				this->poses.push_back(std::vector2d(this->odom.pose.pose.position.x, this->odom.pose.pose.position.y));
				continue;
			}

			double dx = this->odom.pose.pose.position.x - this->poses[this->poses.size()-1].x_component;
			double dy = this->odom.pose.pose.position.y - this->poses[this->poses.size()-1].y_component;

			if (dx * dx + dy * dy > this->distance_gap * this->distance_gap) {
					this->poses.push_back(std::vector2d(this->odom.pose.pose.position.x, this->odom.pose.pose.position.y));
			}

			std::vector<std::vector2d> obstacles = this->get_obstacles();
			sensor_msgs::PointCloud field = this->update_field(obstacles, this->poses);

			std::vector2d grad = this->gradient(this->goal.pose, obstacles, this->poses);
			this->goal.pose.position.x += this->force_rate * grad.x_component;
			this->goal.pose.position.y += this->force_rate * grad.y_component;

			geometry_msgs::PoseStamped actual_goal;
			actual_goal.header.frame_id = this->odom_frame;
			actual_goal.header.stamp = ros::Time::now();
			actual_goal.pose = this->goal.pose;
			this->actual_goal_pub.publish(actual_goal);
			this->goal_pub.publish(this->goal);
			this->field_pub.publish(field);

			rate.sleep();

		}
	}

private:

	void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& map_ptr) {
		this->map = *map_ptr;
		this->got_map = true;
	}

	void odom_callback(const nav_msgs::Odometry::ConstPtr& odom_ptr) {
		this->odom = *odom_ptr;
		this->got_odom = true;
	}

	std::vector<std::vector2d> get_obstacles() {
		uint64_t w = (this->odom.pose.pose.position.x - this->map.info.origin.position.x) / this->map.info.resolution;
		uint64_t h = (this->odom.pose.pose.position.y - this->map.info.origin.position.y) / this->map.info.resolution;
		std::vector<std::vector2d> obstacles;

		for (int64_t dw = -this->roi_width/2; dw < this->roi_width/2; ++dw) {
			for (int64_t dh = -this->roi_height/2; dh < this->roi_height/2; ++dh) {
				
				if (this->map.data[w + dw + this->map.info.width * (h + dh)] < 1) {
					continue;
				}

				double x = this->odom.pose.pose.position.x + dw * this->map.info.resolution;
				double y = this->odom.pose.pose.position.y + dh * this->map.info.resolution;
				obstacles.push_back(std::vector2d(x, y));
			}
		}
		return obstacles;
	}

	std::vector2d gradient(const geometry_msgs::Pose& pose, 
        const std::vector<std::vector2d>& obstacles, 
        const std::vector<std::vector2d>& prev_poses) {

		std::vector2d flow(0.0, 0.0);
		for (const std::vector2d& v: obstacles) {
			double dx = pose.position.x - v.x_component;
			double dy = pose.position.y - v.y_component; 
			std::vector2d ob_cost = normal_grad(dx, dy, ob_std);
			flow = flow + std::vector2d(ob_cost.x_component * ob_amp, ob_cost.y_component * ob_amp);
		}

		int32_t size = prev_poses.size()-queue_size;
		for (int32_t t = std::max(0, size); t < prev_poses.size(); ++t) {
		
				double dx = pose.position.x - prev_poses[t].x_component; 
				double dy = pose.position.y - prev_poses[t].y_component;

				std::vector2d bot_cost = normal_grad(dx, dy, this->bot_std);
				flow = flow + std::vector2d(bot_cost.x_component * this->bot_amp, 
						bot_cost.y_component * this->bot_amp);
		}

		return flow;
	} 

	sensor_msgs::PointCloud update_field(const std::vector<std::vector2d>& obstacles, 
		std::vector<std::vector2d>& prev_poses) {

		geometry_msgs::Point32 point;

		sensor_msgs::PointCloud field;
		field.header.frame_id = "odom";

		for (int64_t dw = -this->roi_width/2; dw < this->roi_width/2; ++dw) {
			for (int64_t dh = -this->roi_height/2; dh < this->roi_height/2; ++dh) {
				double x = this->odom.pose.pose.position.x + dw * this->map.info.resolution;
				double y = this->odom.pose.pose.position.y + dh * this->map.info.resolution;
				double cost = 0.0;
				for (const std::vector2d& v: obstacles) {
					double dx = x - v.x_component;
					double dy = y - v.y_component;
					cost += this->ob_amp * normal_potential(dx, dy, this->ob_std);
				}

				int32_t size = prev_poses.size()-queue_size;
				for (int32_t t = std::max(0, size); t < prev_poses.size(); ++t) {
					double dx = x - prev_poses[t].x_component;
					double dy = y - prev_poses[t].y_component;

					cost += this->bot_amp * normal_potential(dx, dy, this->bot_std);
				}

				cost += this->bot_amp * normal_potential(dw * this->map.info.resolution, 
						dh * this->map.info.resolution, this->bot_std);
				point.x = x;
				point.y = y;
				point.z = cost;
				field.points.push_back(point);
			}
		}	

		field.header.stamp = ros::Time::now();
		return field;
	}

};

int32_t main(int32_t argc, char **argv) {

	potential_fields node(argc, argv);
	node.run();

	return 0;
}

