#pragma once
#include <octree/octree.h>
#include "fsl_common.h"
#include <queue>

class OutOfGridBoundsException : public std::exception {
public:
OutOfGridBoundsException(const char* message) : std::exception(message) {};
};

class SwarmOctTree : public Octree < int > {

private:
	struct BFSNode {
		BFSNode(glm::ivec3 grid_position);
		glm::ivec3 grid_position_;
		glm::ivec3 parent_grid_position_;
		int visited_;
	};
	int grid_cube_length_;
	int grid_resolution_per_side_;
	int grid_resolution_;
	glm::ivec3 offset_;
	int empty_value_;
public:
	int get_interior_mark();
	void mark_floor_plan();
	SwarmOctTree(int grid_cube_length, int grid_resolution);
	std::vector<glm::ivec3> get_adjacent_cells(const glm::ivec3& position) const;
	bool visited(std::vector<BFSNode>& visited_nodes, BFSNode bfs_node) const;
	bool visited(const SwarmOctTree& visited_nodes, const BFSNode& bfs_node) const;
	bool is_unexplored_perimeter(const glm::ivec3& grid_position) const;

	void mark_visited(SwarmOctTree& swarm_oct_tree, const BFSNode& bfs_node) const;
	bool frontier_bread_first_search(const glm::ivec3& current_cell, glm::ivec3& result_cell) const;
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

	int get_grid_resolution_per_side();
	int get_grid_cube_length();
	
};

