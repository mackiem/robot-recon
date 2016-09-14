#pragma once
#include "robot.h"
#include "stlastar.h"

typedef mm::Quadtree<char> LocalMap;

class MapSearchNode
{
public:
	//int x;	 // the (x,y) positions of the node
	//int y;	
	glm::ivec3 position_;
	LocalMap* local_map_;
	
	MapSearchNode() { };
	MapSearchNode( glm::ivec3 position, LocalMap* local_map) : position_(position), local_map_(local_map) { };

	int GetMap( const glm::ivec3& position) const;

	float GoalDistanceEstimate( MapSearchNode &nodeGoal );
	bool IsGoal( MapSearchNode &nodeGoal );
	bool GetSuccessors( AStarSearch<MapSearchNode> *astarsearch, MapSearchNode *parent_node );
	float GetCost( MapSearchNode &successor );
	bool IsSameState( MapSearchNode &rhs );

	void PrintNodeInfo(); 


};
class VisibilityQuadrant {

	int sensor_width_;
	int sensor_height_;
	int half_sensor_width_;
	int half_sensor_height_;;
	int sensor_range_;

	char** quadrants_;
	bool is_visible_to_robot(const glm::ivec3& robot_position, const glm::ivec3& interior_position,
		const glm::ivec3& point_to_test) const;
	void cleanup_internal();

	static VisibilityQuadrant* instance_;
	VisibilityQuadrant(int sensor_range);

	
public:
	enum QUADRANT {
		SW = 0,
		SE = 1,
		NE = 2,
		NW = 3
	};
	VisibilityQuadrant::QUADRANT VisibilityQuadrant::get_quadrant(const glm::ivec3& pt) const;
	static void cleanup();
	static VisibilityQuadrant* visbility_quadrant(int sensor_range);
	static int INVISIBLE;
	static int VISIBLE;
	bool is_sensor_cell_visible(const glm::ivec3& robot_position, const glm::ivec3& interior_position,
		const glm::ivec3& point_to_test);
	void create_visibility_quadrant();
	~VisibilityQuadrant();
	
};

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
	std::unordered_map<int, double> occlusions_per_timestamp_map_;
	std::unordered_map<int, int> clustered_neighbors_per_timestamp_map_;
	int max_time_;
	int measurement_time_step_;
	int current_timestamp_;
	float random_constant_;
	glm::ivec3 previous_local_explore_cell;
	LocalMap local_map_;
	int local_no_of_unexplored_cells_;
	int previous_no_of_local_explored_cells_;
	cv::Vec4f color_;
	float diagonal_grid_length_;

	bool figure_mode_;

	// incremental sensor
	int sensor_width_;
	int sensor_height_;
	int* sensor_cells_;
	std::deque<int> x_sensor_index_;
	std::deque<int> y_sensor_index_;
	std::set<glm::ivec3, IVec3Comparator> incremental_interior_list_;


	std::set<glm::ivec3, IVec3Comparator> local_interior_list_;
	glm::ivec3 previous_local_move_to_grid_cell;
	bool astar_search(const glm::ivec3& start, const glm::ivec3& end, glm::ivec3& next, std::vector<glm::ivec3>& path);

	bool is_local_interior(const glm::ivec3& grid_position) const;
	//static int OUT_OF_BOUNDS;
	//static int INVISIBLE;
	//enum SENSOR_STATE {
	//	NORMAL = 0,
	//	INVISIBLE = 1,
	//	INTERIOR = 2,
	//	OUT_OF_BOUNDS = 3
	//};


	// visibility quadrant
	int half_sensor_width_;
	int half_sensor_height_;
	cv::Vec4f search_color_;
	cv::Vec4f move_to_color_;
	int same_cell_count_;;

	//int no_of_bits_;
	//int no_of_char_arrays_;


	//int no_of_edge_cells_;
	//int no_of_edge_cell_chars_;

	enum LOCAL_SEARCH_STATE {
		EXPLORE = 0,
		PERIMETER = 1
	};

	LOCAL_SEARCH_STATE local_explore_state_;


	//ExperimentalRobot::QUADRANT get_quadrant(const glm::ivec3& pt) const;
	//glm::ivec3 flip_to_SW(const glm::ivec3& pt, ExperimentalRobot::QUADRANT quadrant) const;
	void increment_sensor_range(glm::ivec3 increment);

	void update_adjacent_and_interior(const glm::vec3& previous_position, const glm::vec3& current_position);

public:
	//ExperimentalRobot(UniformLocations& locations, unsigned int id, SwarmOccupancyTree* octree, SwarmCollisionTree* collision_tree, Swarm3DReconTree* recon_tree,
	//	double explore_constant, double separation_constant, double alignment_constant, double cluster_constant, double perimeter_constant, double work_constant,
	//	Range explore_range, Range separation_range, Range alignment_range, Range cluster_range, Range perimeter_range, Range obstacle_avoidance_near_range,
	//	Range obstacle_avoidance_far_range,
	//	double sensor_range, int discovery_range, int neighborhood_count,
	//	double separation_distance, glm::vec3 position, QGLShaderProgram* shader, bool render, double magic_k, bool collide_with_robots,
	//	double square_radius, double bounce_function_power, double bounce_function_multipler);

	static int INTERIOR;
	static int EMPTY;

	ExperimentalRobot(UniformLocations& locations, unsigned id, SwarmOccupancyTree* octree,
		SwarmCollisionTree* collision_tree, Swarm3DReconTree* recon_tree, int cluster_id, double separation_constant, double alignment_constant,
		double cluster_constant, double explore_constant, double sensor_range,
		int discovery_range, double separation_distance, glm::vec3 position,
		double square_radius, double bounce_function_power, double bounce_function_multiplier, int max_time,
		bool collide_with_robots, bool render, QGLShaderProgram* shader);
	void populate_occlusion_map();
	void populate_clustering_map();
	void init_sensor_range();
	void set_sensor_value(int x, int y, int value) const;
	int get_sensor_value(int x, int y) const;
	virtual void update_visualization_structs() override;
	void change_color(cv::Vec4f& color);
	void set_colors_buffer(std::vector<cv::Vec4f>& colors);

	void set_death_time(int death_time);
	void set_cluster_id(int cluster_id);
	void set_figure_mode(bool figure_mode);

	virtual ~ExperimentalRobot() override;

	glm::vec3 calculate_separation_velocity();
	glm::vec3 calculate_alignment_velocity();
	glm::vec3 calculate_clustering_velocity();
	void update(int timestamp);
	//glm::vec3 bounce_off_corners_velocity();
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
	glm::vec3 get_random_velocity();
	bool not_locally_visited(const glm::ivec3& grid_position);
	bool local_explore_search(glm::ivec3& explore_cell_position);
	glm::vec3 calculate_local_explore_velocity();
	
	void mark_locally_covered(const glm::ivec3& grid_position, bool interior);
	void mark_othere_robots_ranges();
	glm::vec3 calculate_obstacle_avoidance_velocity();
	bool  local_perimeter_search(glm::ivec3& explore_cell_position);
};

