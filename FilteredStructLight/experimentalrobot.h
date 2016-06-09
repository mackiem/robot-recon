#pragma once
#include "robot.h"

class ExperimentalRobot : public Robot
{
private:
	glm::vec3 random_direction_;
public:
	ExperimentalRobot(UniformLocations& locations, unsigned int id, SwarmOccupancyTree* octree, SwarmCollisionTree* collision_tree,
		double explore_constant, double separation_constant, double alignment_constant, double cluster_constant, double perimeter_constant, double work_constant,
		Range explore_range, Range separation_range, Range alignment_range, Range cluster_range, Range perimeter_range, Range obstacle_avoidance_near_range,
		Range obstacle_avoidance_far_range,
		double sensor_range, int discovery_range, int neighborhood_count,
		double separation_distance, glm::vec3 position, QGLShaderProgram* shader, bool render, double magic_k, bool collide_with_robots);
	virtual ~ExperimentalRobot() override;
	glm::vec3 calculate_separation_velocity();
	glm::vec3 calculate_alignment_velocity();
	glm::vec3 calculate_clustering_velocity();
	void update(int timestamp);
	glm::vec3 bounce_off_corners_velocity();
	glm::vec3 calculate_random_direction();
	glm::vec3 calculate_explore_velocity();
};

