#include "octree.h"
#include <queue>
#include <memory>

// hoping there won't be 10k robots ever!
int SwarmOctTree::INTERIOR_MARK = 10000;

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
	position += glm::vec3(grid_cube_length_ / 2.f, 0.f, grid_cube_length_ / 2.f);
	return position;
}

SwarmOctTree::BFSNode::BFSNode(glm::ivec3 grid_position) : grid_position_(grid_position), visited_(0) {
}

int SwarmOctTree::get_interior_mark() {
	return INTERIOR_MARK;
}

void SwarmOctTree::mark_floor_plan() {
	int y = 0;
	int edge = grid_resolution_per_side_ - 1;
	for (int x = 0; x < grid_resolution_per_side_; ++x) {
		for (int z = 0; z < grid_resolution_per_side_; ++z) {
			if (x == 0 || x == edge || z == 0 || z == edge) {
				set(x, y, z, INTERIOR_MARK);
			}
		}
	}
}


SwarmOctTree::SwarmOctTree(int grid_cube_length, int grid_resolution) : Octree<int>(grid_resolution), 
	 grid_cube_length_(grid_cube_length), grid_resolution_(grid_resolution) {
	grid_resolution_per_side_ = std::pow(grid_resolution_, 1.f/3.f);
	//offset_ = glm::ivec3(grid_resolution_per_side_ / 2, 0, grid_resolution_per_side_ / 2);
	offset_ = glm::ivec3(0, 0, 0);
	empty_value_ = -1;
	setEmptyValue(empty_value_);
	mark_floor_plan();
}

std::vector<glm::ivec3> SwarmOctTree::get_adjacent_cells(const glm::ivec3& position) const {
	// if 3D we need to get 24 cells
	// getting 8 for 2D
	int y = 0;
	std::vector<glm::ivec3> cells;

	for (int x = -1; x < 2; ++x) {
		for (int z = -1; z < 2; ++z) {
			glm::ivec3 adjacent_operator(x, y, z);
			glm::ivec3 adjacent_cell = position + adjacent_operator;
			if (!is_out_of_bounds(adjacent_cell)) {
				cells.push_back(adjacent_cell);
			}
		}
	}

	return cells;
}

bool SwarmOctTree::visited(std::vector<BFSNode>& visited_nodes, BFSNode bfs_node) const {
	for (auto& visited_node : visited_nodes) {
		if (visited_node.grid_position_ == bfs_node.grid_position_) {
			return true;
		}
	}
	return false;
}

bool SwarmOctTree::is_unexplored_perimeter(const glm::ivec3& grid_position) const {
	if (!is_out_of_bounds(grid_position)) {
		// throw error
	}

	int explored = at(grid_position.x, grid_position.y, grid_position.z);

	return (explored == INTERIOR_MARK);
}

bool SwarmOctTree::is_out_of_bounds(const glm::ivec3& position) const {
	int max_grid_no = grid_resolution_per_side_ - 1;
	if (position.x > max_grid_no
		|| position.y > max_grid_no
		|| position.z > max_grid_no) {
		return true;
	}

	
	if (position.x < 0
		// only 2D commenting this out
		//|| position.y <= 0
		|| position.z < 0) {
		return true;
	}

	return false;
}

int SwarmOctTree::explored_by(const glm::ivec3& position) const {
	if (!is_out_of_bounds(position)) {
		// throw error
	}

	int explored = at(position.x, position.y, position.z);

	return explored;
}

bool SwarmOctTree::has_explored(const glm::ivec3& position) const {
	if (!is_out_of_bounds(position)) {
		// throw error
	}

	int explored = at(position.x, position.y, position.z);

	return (explored != empty_value_);
}

int SwarmOctTree::get_grid_resolution_per_side() {
	return grid_resolution_per_side_;
}

int SwarmOctTree::get_grid_cube_length() {
	return grid_cube_length_;
}

bool SwarmOctTree::frontier_bread_first_search(const glm::ivec3& current_position, glm::ivec3& result_cell) const {
	std::queue<BFSNode> nodes;
	std::vector<BFSNode> visited_nodes;

	nodes.push(BFSNode(current_position));
	visited_nodes.push_back(BFSNode(current_position));

	while (!nodes.empty()) {
		auto& current_cell = nodes.front();

		// pick frontier
		//if (!has_explored(current_cell.grid_position_)) {
		//	result_cell = current_cell.grid_position_;
		//	return true;
		//}

		if (is_unexplored_perimeter(current_cell.grid_position_)) {
			result_cell = current_cell.grid_position_;
			return true;
		}



		// adds only cells within bound, no need to check again
		std::vector<glm::ivec3> adjacent_cells = get_adjacent_cells(current_cell.grid_position_);
		for (auto& adjacent_cell : adjacent_cells) {
			// inefficient rechecking all adjacent cells, without only checking UNVISITED cells
			// copy octree?
			BFSNode adjacent_node(adjacent_cell);
			if (!visited(visited_nodes, adjacent_node)) {
				nodes.push(adjacent_node);
				visited_nodes.push_back(adjacent_node);
			}
		}

		nodes.pop();
	}

	// all cells explored, we are done
	// no search result
	return false;
}
