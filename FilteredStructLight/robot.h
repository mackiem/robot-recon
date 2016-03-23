#pragma once

#include "renderentity.h"
#include "fsl_common.h"
#include <random>
#include <stdexcept>
#include "octree.h"
#include <memory>


class Robot : public VisObject {

private:

	enum RobotState {
		MOVING = 0,
		STOPPING = 1,
		STOPPED = 2
	};

	enum SwarmingState {
		EXPLORING = 0,
		AT_EDGE = 1,
		GOING_INWARD
	};

	void change_state(RobotState state);

	void change_state(SwarmingState state);
	SwarmingState current_swarming_state_;
	RobotState current_robot_state_;

	unsigned int id_;
	glm::vec3 velocity_;
	glm::vec3 position_;
	long long timeout_;
	long long last_timeout_;
	long long last_updated_time_;
	std::mt19937 rng_;
	std::uniform_real_distribution<float> velocity_generator_;
	std::uniform_int_distribution<int> position_generator_;

	// goal position
	glm::vec3 goal_position_;

	float explore_constant_;
	float separation_constant_;
	float work_constant_;

	glm::vec3 explore_force_;
	glm::vec3 separation_force_;
	glm::vec3 work_force_;
	glm::vec3 out_of_bounds_force_;

	// seperation force requirements
	//glm::vec3 center_of_mass_;
	std::vector<std::shared_ptr<Robot>> robots_;
	float minimum_seperation_distance_;
	float separation_distance_threshold_;

	// exploration force requirements
	// location in grid for frontier calculation
	float mass_;
	float max_velocity_;
	bool travelling_in_bound_;
	glm::vec3 resultant_force_;
	float distance_to_goal_threshold_;
	glm::vec3 stopping_force_;
	glm::vec3 calculate_inward_force();
	bool is_velocity_non_zero();
	//int grid_cube_length_;
	//int grid_resolution_per_side_;
	//std::vector<glm::ivec3> get_adjacent_cells(const glm::ivec3& position) const;
	//bool frontier_bread_first_search(const glm::ivec3& current_cell, glm::ivec3& result_cell) const;
	//glm::ivec3 map_to_grid(const glm::vec3& position) const;
	//glm::vec3 map_to_position(const glm::ivec3& grid_position) const;
	//bool is_out_of_bounds(const glm::ivec3& position) const ;
	//bool explored_by(const glm::ivec3& position) const ;

	// kinematics
	glm::vec3 calculate_force(glm::vec3 move_to_position);
	glm::vec3 calculate_force(glm::vec3 move_to_position, float constant) const;


	std::shared_ptr<SwarmOctTree> octree_;
	QGLShaderProgram* shader_;

public:
	void set_explore_constant(float constant);
	void set_seperation_constant(float constant);
	void set_work_constant(float constant);

	void calculate_explore_force();
	void calculate_separation_force();
	void calculate_work_force();
	bool is_at_edge();
	void stop();
	void nullify_forces();
	void calculate_out_of_bounds_force();
	void visualize_force(const int& mesh_id, const glm::vec3& force, const cv::Vec4f& color, bool initialize);

	glm::vec3 calculate_resultant_force();
	Robot(UniformLocations& locations, unsigned id, 
		std::shared_ptr<SwarmOctTree> octree, 
		double explore_constant, double seperation_constant, 
		double work_constant, double seperation_distance, QGLShaderProgram* shader);
	//Robot(UniformLocations& locations, unsigned int id, std::shared_ptr<SwarmOctTree> octree);
	void set_velocity(glm::vec3 velocity);
	void set_random_velocity();
	void update_explored(const glm::vec3& position);
	void update_robots(const std::vector<std::shared_ptr<Robot>>& robots);
	void calculate_stopping_force();
	void handle_input();
	//void handle_input();
	virtual void update(glm::mat4 global_model);

	Robot& operator=(const Robot& other);

	// calculate forces, 3 types of forces for now
	// calculate acceleration and integrate for velocity and position

};

