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
	Swarm3DReconTree* recon_tree_;
	QMutex recon_mutex_;
	std::vector<glm::ivec3> reconstructed_positions_;
	std::vector<cv::Vec4f> colors_;
	int death_time_;
	bool dead_color_changed_;
	std::set<glm::ivec3, IVec3Comparator> past_reconstructed_positions_;
	std::unordered_map<int, int> occlusions_per_timestamp_map_;
	std::unordered_map<int, int> clustered_neighbors_per_timestamp_map_;
	int max_time_;
	int measurement_time_step_;
	int current_timestamp_;
public:
	//ExperimentalRobot(UniformLocations& locations, unsigned int id, SwarmOccupancyTree* octree, SwarmCollisionTree* collision_tree, Swarm3DReconTree* recon_tree,
	//	double explore_constant, double separation_constant, double alignment_constant, double cluster_constant, double perimeter_constant, double work_constant,
	//	Range explore_range, Range separation_range, Range alignment_range, Range cluster_range, Range perimeter_range, Range obstacle_avoidance_near_range,
	//	Range obstacle_avoidance_far_range,
	//	double sensor_range, int discovery_range, int neighborhood_count,
	//	double separation_distance, glm::vec3 position, QGLShaderProgram* shader, bool render, double magic_k, bool collide_with_robots,
	//	double square_radius, double bounce_function_power, double bounce_function_multipler);

	ExperimentalRobot(UniformLocations& locations, unsigned id, SwarmOccupancyTree* octree,
		SwarmCollisionTree* collision_tree, Swarm3DReconTree* recon_tree, int cluster_id, double separation_constant, double alignment_constant,
		double cluster_constant, double explore_constant, double sensor_range,
		int discovery_range, double separation_distance, glm::vec3 position,
		double square_radius, double bounce_function_power, double bounce_function_multiplier, int max_time,
		bool collide_with_robots, bool render, QGLShaderProgram* shader);
	void populate_occlusion_map();
	void populate_clustering_map();
	virtual void update_visualization_structs() override;
	void change_color(cv::Vec4f& color);
	void set_colors_buffer(std::vector<cv::Vec4f>& colors);

	void set_death_time(int death_time);
	void set_cluster_id(int cluster_id);

	virtual ~ExperimentalRobot() override;

	glm::vec3 calculate_separation_velocity();
	glm::vec3 calculate_alignment_velocity();
	glm::vec3 calculate_clustering_velocity();
	void update(int timestamp);
	glm::vec3 bounce_off_corners_velocity();
	glm::vec3 calculate_random_direction();
	glm::vec3 calculate_explore_velocity();
	void reconstruct_points();
	bool is_colliding_precisely(const glm::vec3& interior_cell);
	bool is_colliding_precisely(const std::vector<glm::vec3>& interior_cells);
	glm::vec3 calculate_bounce_explore_velocity(const std::vector<glm::vec3>& interior_cells) const;
	glm::vec3 calculate_bounce_explore_velocity(const glm::vec3& interior_cell) const;
	std::vector<glm::vec3> get_corners(const glm::vec3& interior_cell) const;
	double calculate_occulsion();
	double calculate_clustering();
};

