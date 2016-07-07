#pragma once
#include "robot.h"

class ExperimentalRobot : public Robot
{
private:
	glm::vec3 random_direction_;
	std::vector<int> robot_ids_;
	int previous_no_of_explored_cells_;
	glm::ivec3 previous_explore_cell;
	glm::vec3 previous_nminus2_position_; 
	double square_radius_;
	double bounce_function_power_;
	double bounce_function_multiplier_;
public:
	ExperimentalRobot(UniformLocations& locations, unsigned int id, SwarmOccupancyTree* octree, SwarmCollisionTree* collision_tree,
		double explore_constant, double separation_constant, double alignment_constant, double cluster_constant, double perimeter_constant, double work_constant,
		Range explore_range, Range separation_range, Range alignment_range, Range cluster_range, Range perimeter_range, Range obstacle_avoidance_near_range,
		Range obstacle_avoidance_far_range,
		double sensor_range, int discovery_range, int neighborhood_count,
		double separation_distance, glm::vec3 position, QGLShaderProgram* shader, bool render, double magic_k, bool collide_with_robots,
		double square_radius, double bounce_function_power, double bounce_function_multipler);

	ExperimentalRobot(UniformLocations& locations, unsigned id, SwarmOccupancyTree* octree,
		SwarmCollisionTree* collision_tree, double separation_constant, double alignment_constant,
		double cluster_constant, double explore_constant, double sensor_range,
		int discovery_range, double separation_distance, glm::vec3 position,
		double square_radius, double bounce_function_power, double bounce_function_multiplier, bool collide_with_robots, bool render, QGLShaderProgram* shader);



	virtual ~ExperimentalRobot() override;

	glm::vec3 calculate_separation_velocity();
	glm::vec3 calculate_alignment_velocity();
	glm::vec3 calculate_clustering_velocity();
	void update(int timestamp);
	glm::vec3 bounce_off_corners_velocity();
	glm::vec3 calculate_random_direction();
	glm::vec3 calculate_explore_velocity();
	bool is_colliding_precisely(const glm::vec3& interior_cell);
	bool is_colliding_precisely(const std::vector<glm::vec3>& interior_cells);
	glm::vec3 calculate_bounce_explore_velocity(const std::vector<glm::vec3>& interior_cells) const;
	glm::vec3 calculate_bounce_explore_velocity(const glm::vec3& interior_cell) const;
	std::vector<glm::vec3> get_corners(const glm::vec3& interior_cell) const;
};

