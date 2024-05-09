#include <cmath>
#include <vector>

#include <geometry_msgs/Pose.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/MapMetaData.h>

namespace std {

class vector2d {

public:
	vector2d();
	vector2d(double x, double y);

private:
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

namespace normal {

class potential {
public:
	potential();
	potential(double);

private:
	double value;
	std::vector2d gradient;
	bool has_grad;

public:
	double get_potential();
};

class potential_field {
public:
	potential_field();
	potential_field(uint64_t, uint64_t);
	potential_field(const nav_msgs::MapMetaData&);

private:
	std::vector<potential> field;
	std::vector<bool> is_evaluated;
	nav_msgs::MapMetaData info;

public:
	sensor_msgs::PointCloud get_topo();
};

/*
				| dU/dx |
	F = - |       | 
				| dU/dy |
*/

}



