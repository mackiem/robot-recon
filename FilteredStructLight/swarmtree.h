#pragma once
#include "fsl_common.h"
#include "quadtree.h"
#include <memory>
#include <queue>
#include <functional>

class OutOfGridBoundsException : public std::exception {
public:
OutOfGridBoundsException(const char* message) : std::exception(message) {};
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

class SwarmOccupancyTree : public mm::Quadtree< int > {

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

	int grid_cube_length_;
	int grid_resolution_per_side_;
	int grid_resolution_;
	glm::ivec3 offset_;

	std::set<glm::ivec3, IVec3Comparator> explore_perimeter_list_;
	std::set<glm::ivec3, IVec3Comparator> empty_space_list_;

	std::set<glm::ivec3, IVec3Comparator> static_perimeter_list_;
	std::set<glm::ivec3, IVec3Comparator> interior_list_;
	//int empty_value_;

	int pool_size_;
	int current_pool_count_;
	std::vector<std::vector<PerimeterPos>> heap_pool_;

public:
	bool going_through_interior_test(const glm::vec3& robot_position, const glm::vec3& point_to_test) const;
	bool find_closest_perimeter(const glm::ivec3& robot_grid_position,
		glm::ivec3& perimeter_position);
	bool find_closest_empty_space(const glm::ivec3& robot_grid_position,
		glm::ivec3& perimeter_position);
	bool find_closest_position_from_list(const std::set<glm::ivec3, IVec3Comparator>& explore_perimeter_list, const glm::ivec3& robot_grid_position,
		glm::ivec3& explore_position);
	bool find_closest_position_from_list(const std::set<glm::ivec3, IVec3Comparator>& explore_perimeter_list, const glm::ivec3& robot_grid_position,
		glm::ivec3& explore_position, float range_min, float range_max);
 
	std::set<glm::ivec3, IVec3Comparator> get_unexplored_perimeter_list();
	std::set<glm::ivec3, IVec3Comparator> get_static_perimeter_list();
	std::set<glm::ivec3, IVec3Comparator> get_interior_list();

	void create_interior_list();
	int get_interior_mark();
	void mark_floor_plan();
	SwarmOccupancyTree(int grid_cube_length, int grid_resolution);
	SwarmOccupancyTree(int grid_cube_length, int grid_resolution, int empty_value);
	void get_adjacent_cells(const glm::ivec3& position, std::vector<glm::ivec3>& cells, int sensor_range) const;
	bool visited(std::set<BFSNode>* visited_nodes, const BFSNode& bfs_node) const;
	bool visited(const SwarmOccupancyTree& visited_nodes, const BFSNode& bfs_node) const;
	//bool visited(std::set<glm::ivec3>* visited_nodes, const glm::ivec3& bfs_node) const;
	bool is_unexplored_perimeter(const glm::ivec3& grid_position) const;

	void mark_visited(SwarmOccupancyTree& swarm_oct_tree, const BFSNode& bfs_node) const;
	bool frontier_bread_first_search(const glm::ivec3& current_cell, glm::ivec3& result_cell, 
		int max_depth) const;
	void mark_interior_line(glm::vec3 a, glm::vec3 b);
	static int INTERIOR_MARK;
	static int SEARCH_VISITED;
	static int SEARCH_NOT_VISITED;
	glm::ivec3 map_to_grid(const glm::vec3& position) const;
	glm::vec3 map_to_position(const glm::ivec3& grid_position) const;
	bool is_out_of_bounds(const glm::ivec3& position) const ;
	int explored_by(const glm::ivec3& position) const ;
	bool has_explored(const glm::ivec3& position) const;

	bool is_interior(const glm::ivec3& position) const;
	std::vector<glm::vec3> find_adjacent_interiors(const std::vector<glm::ivec3>& adjacent_cells) const;

	int get_grid_resolution_per_side();
	int get_grid_cube_length();
	void create_perimeter_list();
	void create_empty_space_list();
	bool next_cell_to_explore(const glm::ivec3& robot_grid_position,
		glm::ivec3& explore_position);

	bool next_cell_to_explore(const glm::ivec3& robot_grid_position,
		glm::ivec3& explore_position, float range_min, float range_max);
	void mark_explored_in_perimeter_list(const glm::ivec3& grid_position);
	void mark_explored_in_empty_space_list(const glm::ivec3& grid_position);
	void mark_explored_in_list(std::set<glm::ivec3, IVec3Comparator>& position_list, const glm::ivec3& grid_position);
	
};

class SwarmCollisionTree : public mm::Quadtree<std::set<int>*> {

public:
	std::vector<int> find_adjacent_robots(int robot_id, const std::vector<glm::ivec3>& adjacent_cells) const;
	std::vector<int> find_adjacent_robots(int robot_id, const glm::ivec3& position) const;
	SwarmCollisionTree(unsigned resolution);
	void insert(int robot_id, const glm::ivec3& position);
	void update(int robot_id, const glm::ivec3& previous_position, const glm::ivec3& current_position);
	~SwarmCollisionTree() override;
};

