#pragma once
#include <octree/octree.h>
#include "fsl_common.h"

class OutOfGridBoundsException : public std::exception {
public:
OutOfGridBoundsException(const char* message) : std::exception(message) {};
};

class SwarmOctTree : public Octree<int> {

private:
	int grid_cube_length_;
	int grid_resolution_per_side_;
	int grid_resolution_;
	glm::ivec3 offset_;
public:
	SwarmOctTree(int grid_cube_length, int grid_resolution);
	std::vector<glm::ivec3> get_adjacent_cells(const glm::ivec3& position) const;
	bool frontier_bread_first_search(const glm::ivec3& current_cell, glm::ivec3& result_cell) const;
	glm::ivec3 map_to_grid(const glm::vec3& position) const;
	glm::vec3 map_to_position(const glm::ivec3& grid_position) const;
	bool is_out_of_bounds(const glm::ivec3& position) const ;
	bool has_explored(const glm::ivec3& position) const ;
	
};

