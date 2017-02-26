#pragma once
#include "fsl_common.h"
#include "quadtree.h"
#include <memory>
#include <queue>
#include <functional>
#include <QMutex>

#define GRID_MAX 100000
//class OutOfGridBoundsException : public std::exception {
//public:
//OutOfGridBoundsException(const char* message) : std::exception(message) {};
//};

struct VisibleCell {
	glm::ivec3 cell;
	bool visible;
	VisibleCell() : visible(true) {
	}
	bool is_visible() const { return visible; };
};

struct IVec3Hasher {
	std::size_t operator()(const glm::ivec3& k) const {
		return k.x + GRID_MAX * k.x + GRID_MAX * GRID_MAX * k.z;
	}
};

struct ExploredPosition {
	glm::ivec3 grid_pos_;
	bool explored_;

	ExploredPosition(glm::ivec3 grid_pos) : grid_pos_(grid_pos), explored_(false) {}
};

struct IVec3Comparator {
	bool operator()(const glm::ivec3& lhs_grid_pos, const glm::ivec3& rhs_grid_pos) const {
		//auto& lhs_grid_pos = lhs;
		//auto& rhs_grid_pos = rhs;
		if (lhs_grid_pos.x == rhs_grid_pos.x) {
			if (lhs_grid_pos.y == rhs_grid_pos.y) {
				return lhs_grid_pos.z < rhs_grid_pos.z;
			}
			else {
				return lhs_grid_pos.y < rhs_grid_pos.y;
			}
		}
		else {
			return lhs_grid_pos.x < rhs_grid_pos.x;
		}
	}
};

struct IVec3Equals {
	bool operator()(const glm::ivec3& lhs_grid_pos, const glm::ivec3& rhs_grid_pos) const {
		if ((lhs_grid_pos.x == rhs_grid_pos.x) &&
			(lhs_grid_pos.y == rhs_grid_pos.y) && 
			(lhs_grid_pos.z == rhs_grid_pos.z)) {
			return true;
		}
		return false;
	}
};


struct Vec3Comparator {
	bool operator()(const glm::vec3& lhs_grid_pos, const glm::vec3& rhs_grid_pos) const {
		//auto& lhs_grid_pos = lhs;
		//auto& rhs_grid_pos = rhs;
		if (lhs_grid_pos.x == rhs_grid_pos.x) {
			if (lhs_grid_pos.y == rhs_grid_pos.y) {
				return lhs_grid_pos.z < rhs_grid_pos.z;
			}
			else {
				return lhs_grid_pos.y < rhs_grid_pos.y;
			}
		}
		else {
			return lhs_grid_pos.x < rhs_grid_pos.x;
		}
	}
};

typedef std::unordered_map<glm::ivec3, double, IVec3Hasher, IVec3Equals> SimSampMap;

class ThreadSafeSimSampMap {
	
	QMutex lock_;
	SimSampMap map_;

public:
	void set_map(SimSampMap& map) {
		lock_.lock();
		map_ = map;
		lock_.unlock();
	}
	SimSampMap get_map() {
		lock_.lock();
		auto map = map_;
		lock_.unlock();
		return map;
	}

	void clear() {
		lock_.lock();
		map_ = SimSampMap();
		lock_.unlock();
	}
}; 

struct PerimeterPos {
	float distance_;
	glm::ivec3 grid_position_;

	PerimeterPos(float distance, glm::ivec3 grid_position) : distance_(distance), grid_position_(grid_position) {
	}

	PerimeterPos() {
		
	}

	bool operator<(const PerimeterPos& other) const {
		return distance_ < other.distance_;
	}

	bool operator>(const PerimeterPos& other) const {
		return distance_ > other.distance_;
	}
};

class SwarmOccupancyTree : public mm::Quadtree<int> {

private:
	struct BFSNode {
		BFSNode(glm::ivec3 grid_position);
		glm::ivec3 grid_position_;
	};

	friend bool operator<(const BFSNode& lhs, const BFSNode& rhs) {
		auto& lhs_grid_pos = lhs.grid_position_;
		auto& rhs_grid_pos = rhs.grid_position_;
		if (lhs_grid_pos.x == rhs_grid_pos.x) {
			if (lhs_grid_pos.y == rhs_grid_pos.y) {
				return lhs_grid_pos.z < rhs_grid_pos.z;
			} else {
				return lhs_grid_pos.y < rhs_grid_pos.y;
			}
		} else {
			return lhs_grid_pos.x < rhs_grid_pos.x;
		}

	};

	//int grid_cube_length_;
	//int grid_resolution_width_;
	//int grid_resolution_height_;
	//int grid_resolution_;
	glm::ivec3 offset_;

	//std::shared_ptr<std::unordered_map<glm::ivec3, std::unordered_map<int, std::unordered_map<int, int>>
	//	, IVec3Hasher, IVec3Equals>> sampling_tracker_;
	//std::unordered_map<glm::ivec3, std::map<int, std::map<int, int>>
	//	, IVec3Hasher, IVec3Equals>* sampling_tracker_;

	std::set<glm::ivec3, IVec3Comparator> explore_perimeter_list_;
	std::set<glm::ivec3, IVec3Comparator> empty_space_list_;

	std::set<glm::ivec3, IVec3Comparator> static_perimeter_list_;
	std::set<glm::ivec3, IVec3Comparator> interior_list_;
	std::set<glm::ivec3, IVec3Comparator> explore_interior_list_;
	//int empty_value_;

	int pool_size_;
	int current_pool_count_;
	std::vector<std::vector<PerimeterPos>> heap_pool_;
	float* leak_;
	bool update_multisampling_;
	long last_multisample_timestep_;

	struct Sampling {
		glm::ivec3 grid_cell;
		int robot_id;
		long timestamp;
		int cluster_id;
	};


	std::vector<Sampling>* sampling_tracker_;

	std::vector<std::pair<double, int>> sampling_avg_storage_;

	std::unordered_map<glm::ivec3, int, IVec3Hasher, IVec3Equals>  no_of_simul_samples_per_timestep_per_gridcell;
	std::unordered_map<glm::ivec3, int, IVec3Hasher, IVec3Equals>  no_of_sampled_timesteps_per_gridcell;

	std::vector<PerimeterPos> perimeter_vector_;
public:

	bool is_interior_interior(const glm::ivec3& position);
	bool is_perimeter(const glm::ivec3& grid_position) const;

	bool going_through_interior_test(const glm::ivec3& robot_position, const glm::ivec3& point_to_test) const;
	bool find_closest_perimeter(const glm::ivec3& robot_grid_position,
		glm::ivec3& perimeter_position);
	bool find_closest_empty_space(const glm::ivec3& robot_grid_position,
		glm::ivec3& perimeter_position);
	bool find_closest_position_from_list(const std::set<glm::ivec3, IVec3Comparator>& explore_perimeter_list, const glm::ivec3& robot_grid_position,
		glm::ivec3& explore_position);
	bool find_closest_position_from_list(const std::set<glm::ivec3, IVec3Comparator>& explore_perimeter_list, const glm::ivec3& robot_grid_position,
		glm::ivec3& explore_position, float range_min, float range_max);

	bool closest_perimeter(const glm::ivec3& robot_grid_position,
		glm::ivec3& perimeter_position, float range_min, float range_max);

	bool find_closest_2_positions_from_list(const std::set<glm::ivec3, IVec3Comparator>& perimeter_list,
		const glm::ivec3& robot_grid_position,
		std::vector<glm::ivec3>& explore_positions, float range_min, float range_max, bool enable_interior_test = true);

	bool closest_2_interior_positions(const glm::ivec3& robot_grid_position,
		std::vector<glm::ivec3>& perimeter_position, float range_min, float range_max);
 
	std::set<glm::ivec3, IVec3Comparator> get_unexplored_perimeter_list();
	int no_of_unexplored_cells();
	int no_of_interior_cells() const;
	std::set<glm::ivec3, IVec3Comparator> get_static_perimeter_list();
	std::set<glm::ivec3, IVec3Comparator> get_interior_list();

	void create_interior_list();
	int get_interior_mark();
	void mark_floor_plan();
	//SwarmOccupancyTree(int grid_cube_length, int grid_resolution);
	//SwarmOccupancyTree::SwarmOccupancyTree(float grid_square_length, int grid_height, int grid_width);
	SwarmOccupancyTree(int grid_cube_length, int grid_width, int grid_height, char empty_value);
	void get_adjacent_cells(const glm::ivec3& position, std::vector<glm::ivec3>& cells, int sensor_range) const;
	bool visited(std::set<BFSNode>* visited_nodes, const BFSNode& bfs_node) const;
	bool visited(const SwarmOccupancyTree& visited_nodes, const BFSNode& bfs_node) const;
	//bool visited(std::set<glm::ivec3>* visited_nodes, const glm::ivec3& bfs_node) const;
	//bool is_unexplored_perimeter(const glm::ivec3& grid_position) const;

	void init_coverage_map(std::unordered_map<glm::ivec3, int, IVec3Hasher, IVec3Equals>& coverage_map_) const;

	void mark_visited(SwarmOccupancyTree& swarm_oct_tree, const BFSNode& bfs_node) const;
	bool frontier_bread_first_search(const glm::ivec3& current_cell, glm::ivec3& result_cell, 
		int max_depth) const;
	void mark_interior_line(glm::vec3 a, glm::vec3 b);
	void remove_inner_interiors();
	void mark_perimeter_covered_by_robot(glm::ivec3 grid_cell, int timestep, int robot_id, int cluster_id);
	double calculate_simultaneous_sampling_factor();
	double calculate_multi_sampling_factor();
	void calculate_simultaneous_sampling_per_cluster();
	double calculate_coverage();
	static int INTERIOR_MARK;
	static int PERIMETER;
	static int SEARCH_VISITED;
	static int SEARCH_NOT_VISITED;
	glm::ivec3 map_to_grid(const glm::vec3& position) const;
	glm::vec3 map_to_position(const glm::ivec3& grid_position) const;
	bool is_out_of_bounds(const glm::ivec3& position) const ;
	int explored_by(const glm::ivec3& position) const ;
	bool has_explored(const glm::ivec3& position) const;

	bool is_interior(const glm::ivec3& position) const;
	std::vector<glm::vec3> find_adjacent_interiors(const std::vector<glm::ivec3>& adjacent_cells) const;

	//int get_grid_resolution_per_side();
	//int get_grid_cube_length();
	void create_perimeter_list();
	void create_empty_space_list();
	bool next_cell_to_explore(const glm::ivec3& robot_grid_position,
		glm::ivec3& explore_position);

	bool next_cell_to_explore(const glm::ivec3& robot_grid_position,
		glm::ivec3& explore_position, float range_min, float range_max);

	bool next_cell_to_explore_visibility_non_aware(const glm::ivec3& robot_grid_position,
		glm::ivec3& explore_position, float range_min, float range_max);

	bool find_closest_position_from_list_visibility_non_aware(const std::set<glm::ivec3, IVec3Comparator>& perimeter_list,
		const glm::ivec3& robot_grid_position,
		glm::ivec3& explore_position, float range_min, float range_max);

	bool mark_explored_in_interior_list(const glm::ivec3& grid_position);
	void mark_explored_in_perimeter_list(const glm::ivec3& grid_position);
	void mark_explored_in_empty_space_list(const glm::ivec3& grid_position);
	bool mark_explored_in_list(std::set<glm::ivec3, IVec3Comparator>& position_list, const glm::ivec3& grid_position);

	SimSampMap calculate_simultaneous_sampling_per_grid_cell();
	SimSampMap calculate_multi_sampling_per_grid_cell();
	
	virtual ~SwarmOccupancyTree();
};

class SwarmCollisionTree : public mm::Quadtree<std::set<int>*> {

public:
	std::vector<int> find_adjacent_robots(int robot_id, const std::vector<glm::ivec3>& adjacent_cells) const;
	std::vector<int> find_adjacent_robots(int robot_id, const glm::ivec3& position) const;
	void find_adjacent_robots_memory_save(int robot_id, const std::vector<VisibleCell>& adjacent_cells, const int current_adjacent_cells,
		std::vector<int>& adjacent_robots, int& current_no_of_robots) const;
	SwarmCollisionTree(unsigned width, unsigned height);
	void insert(int robot_id, const glm::ivec3& position);
	void update(int robot_id, const glm::ivec3& previous_position, const glm::ivec3& current_position);
	virtual ~SwarmCollisionTree() override;
};

class Swarm3DReconTree : public mm::Quadtree<std::vector<glm::vec3>*> {
private:
	int grid_cube_length_;
	int grid_resolution_;
	std::unordered_map<glm::ivec3, int, IVec3Hasher> multi_sampling_map_;
	int total_no_of_3d_points;
public:
	double calculate_multi_sampling_factor();
	double calculate_density();
	void update_multi_sampling_map(const glm::ivec3& position);

	void mark_random_points_in_triangle(glm::vec3 a, glm::vec3 b, glm::vec3 c);
	void mark_interior_line(glm::vec3 a, glm::vec3 b);
	//bool is_out_of_bounds(const glm::ivec3& position) const;
	void mark_floor_plan();
	//glm::ivec3 map_to_grid(const glm::vec3& position) const;
	Swarm3DReconTree(float grid_cube_length, int grid_width, int grid_height);
	void insert(glm::vec3& points, const glm::ivec3& position);
	std::vector<glm::vec3>* get_3d_points(const glm::ivec3& position);
	virtual ~Swarm3DReconTree() override;
};

//class LocalMap : public mm::Quadtree<int> {
//	LocalMap(unsigned width, unsigned height, float grid_square_length, int empty_value)
//		: Quadtree<int>(width, height, grid_square_length, empty_value)
//
//	}
//
//
//	
//};



typedef mm::Quadtree<int> LocalMap;
