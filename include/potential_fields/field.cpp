#include "field.h"

#include <geometry_msgs/Point32.h>

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

namespace normal {

/* potential definition */

potential::potential() {
	this->value = 0.0;
	this->has_grad = false;
}

potential::potential(double value) {
	this->value = value;
	this->has_grad = false;
}

double potential::get_potential() {
	return this->value;
}

/* potential field definition */

potential_field::potential_field() {}

potential_field::potential_field(uint64_t height, uint64_t width) {
	this->field.resize(height * width);
	this->is_evaluated = std::vector<bool>(height * width, false);
}

potential_field::potential_field(const nav_msgs::MapMetaData& info) {
	this->field.resize(info.height * info.width);
	this->is_evaluated = std::vector<bool>(info.height * info.width, false);
	this->info = info;
}

sensor_msgs::PointCloud potential_field::get_topo() {
	sensor_msgs::PointCloud cloud;
	geometry_msgs::Point32 point;
	for (int32_t t = 0; t < this->field.size(); ++t) {
		point.x = this->info.origin.position.x + 
			this->info.resolution * (t%(this->info.width));
		point.y = this->info.origin.position.y + 
			this->info.resolution * (t/(this->info.width));
		point.z = this->field[t].get_potential();

		cloud.points.push_back(point);
	}return cloud;
}

}


