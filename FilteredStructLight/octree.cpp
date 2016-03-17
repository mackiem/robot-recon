#include "octree.h"
#include <queue>

glm::ivec3 SwarmOctTree::map_to_grid(const glm::vec3& position) const  {
	glm::ivec3 grid_pos =  (position / static_cast<float>(grid_cube_length_));
	grid_pos += offset_;
//glm::vec3(0.5f, 0.f, 0.5f) +

	// if greater than grid resolution, throw error
	if (is_out_of_bounds(grid_pos)) {
		throw OutOfGridBoundsException("Out of bounds");
	}

	return grid_pos;
}

glm::vec3 SwarmOctTree::map_to_position(const glm::ivec3& grid_position)  const {
	glm::vec3 position = ((grid_position - offset_)
		* grid_cube_length_);
	position += glm::vec3(0.5f, 0.f, 0.5f);
	return position;
}

SwarmOctTree::SwarmOctTree(int grid_cube_length, int grid_resolution) : Octree<int>(grid_resolution), 
	 grid_cube_length_(grid_cube_length), grid_resolution_(grid_resolution) {
	grid_resolution_per_side_ = std::pow(grid_resolution_, 1.f/3.f);
	//offset_ = glm::ivec3(grid_resolution_per_side_ / 2, 0, grid_resolution_per_side_ / 2);
	offset_ = glm::ivec3(0, 0, 0);
}

std::vector<glm::ivec3> SwarmOctTree::get_adjacent_cells(const glm::ivec3& position) const {
	// if 3D we need to get 24 cells
	// getting 8 for 2D
	int z = 0;
	std::vector<glm::ivec3> cells;

	for (int x = -1; x < 2; ++x) {
		for (int y = -1; y < 2; ++y) {
			glm::ivec3 adjacent_operator(x, y, z);
			glm::ivec3 adjacent_cell = position + adjacent_operator;
			if (!is_out_of_bounds(adjacent_cell)) {
				cells.push_back(adjacent_cell);
			}
		}
	}

	return cells;
}

bool SwarmOctTree::is_out_of_bounds(const glm::ivec3& position) const {
	if (position.x > grid_resolution_per_side_
		|| position.y > grid_resolution_per_side_
		|| position.z > grid_resolution_per_side_) {
		return true;
	}

	if (position.x < 0
		|| position.y < 0
		|| position.z < 0) {
		return true;
	}

	return false;
}

bool SwarmOctTree::has_explored(const glm::ivec3& position) const {
	if (!is_out_of_bounds(position)) {
		// throw error
	}

	bool explored = at(position.x, position.y, position.z);

	return explored;
}

bool SwarmOctTree::frontier_bread_first_search(const glm::ivec3& current_position, glm::ivec3& result_cell) const {
	std::queue<glm::ivec3> nodes;

	nodes.push(current_position);

	while (!nodes.empty()) {
		auto current_cell = nodes.front();

		// pick frontier
		if (!has_explored(current_cell)) {
			result_cell = current_cell;
			return true;
		}

		// adds only cells within bound, no need to check again
		std::vector<glm::ivec3> adjacent_cells = get_adjacent_cells(current_cell);
		for (auto& adjacent_cell : adjacent_cells) {
			if (!has_explored(adjacent_cell)) {
				nodes.push(adjacent_cell);
			}
		}

		nodes.pop();
	}

	// all cells explored, we are done
	// no search result
	return false;
}
