#include "swarmtree.h"


#include <queue>
#include <memory>
#include "swarmutils.h"
#include <map>

using namespace mm;

// hoping there won't be 10k robots ever!
int SwarmOccupancyTree::INTERIOR_MARK = 10000;

int SwarmOccupancyTree::SEARCH_VISITED = 1;
int SwarmOccupancyTree::SEARCH_NOT_VISITED = 0;


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

glm::ivec3 SwarmOccupancyTree::map_to_grid(const glm::vec3& position) const  {
	glm::vec3 grid_pos_float =  (position / static_cast<float>(grid_cube_length_));
	glm::ivec3 grid_pos(grid_pos_float.x, grid_pos_float.y, grid_pos_float.z);
	grid_pos += offset_;
//glm::vec3(0.5f, 0.f, 0.5f) +

	// if greater than grid resolution, throw error
	if (is_out_of_bounds(grid_pos)) {
		throw OutOfGridBoundsException("Out of bounds");
	}

	return grid_pos;
}

glm::vec3 SwarmOccupancyTree::map_to_position(const glm::ivec3& grid_position)  const {
	glm::vec3 position = ((grid_position - offset_)
		* grid_cube_length_);
	position += glm::vec3(grid_cube_length_ / 2.f, 0.f, grid_cube_length_ / 2.f);
	return position;
}

SwarmOccupancyTree::BFSNode::BFSNode(glm::ivec3 grid_position) : grid_position_(grid_position) {
}

int SwarmOccupancyTree::get_interior_mark() {
	return INTERIOR_MARK;
}

void SwarmOccupancyTree::mark_floor_plan() {
	int y = 0;
	int edge = grid_resolution_per_side_ - 1;
	for (int x = 0; x < grid_resolution_per_side_; ++x) {
		for (int z = 0; z < grid_resolution_per_side_; ++z) {
			if (x == 0 || x == edge || z == 0 || z == edge) {
				set(x, z, INTERIOR_MARK);
			}
		}
	}
}


SwarmOccupancyTree::SwarmOccupancyTree(int grid_cube_length, int grid_resolution) :
Quadtree<int>(grid_resolution, -1), grid_cube_length_(grid_cube_length), grid_resolution_(grid_resolution) {
	grid_resolution_per_side_ = get_resolution_per_side();
	//offset_ = glm::ivec3(grid_resolution_per_side_ / 2, 0, grid_resolution_per_side_ / 2);
	offset_ = glm::ivec3(0, 0, 0);
	//empty_value_ = -1;
	//setEmptyValue(empty_value_);
	mark_floor_plan();

}

SwarmOccupancyTree::SwarmOccupancyTree(int grid_cube_length, int grid_resolution, int empty_value) : 
	Quadtree<int>(grid_resolution, empty_value), grid_cube_length_(grid_cube_length), grid_resolution_(grid_resolution) {
	grid_resolution_per_side_ = get_resolution_per_side();
	//offset_ = glm::ivec3(grid_resolution_per_side_ / 2, 0, grid_resolution_per_side_ / 2);
	offset_ = glm::ivec3(0, 0, 0);
	//empty_value_ = -1;
	//setEmptyValue(empty_value_);
	mark_floor_plan();
}

void SwarmOccupancyTree::get_adjacent_cells(const glm::ivec3& position, std::vector<glm::ivec3>& cells) const {
	// if 3D we need to get 24 cells
	// getting 8 for 2D
	int y = 0;
	//std::vector<glm::ivec3> cells;
	//cells.reserve(9);

	for (int x = -1; x < 2; ++x) {
		for (int z = -1; z < 2; ++z) {
			//if (x != 0 && z != 0) {
				glm::ivec3 adjacent_operator(x, y, z);
				glm::ivec3 adjacent_cell = position + adjacent_operator;
				if (!is_out_of_bounds(adjacent_cell)) {
					cells.push_back(adjacent_cell);
				}
			//}
		}
	}
	//std::cout << "adjacent cell size : " << cells.size() << std::endl;

}

bool SwarmOccupancyTree::visited(std::set<BFSNode>* visited_nodes, const BFSNode& bfs_node) const {
	if (visited_nodes->find(bfs_node) != visited_nodes->end()) {
		return true;
	}
	return false;
}

//bool SwarmOccupancyTree::visited(std::set<glm::ivec3>* visited_nodes, const glm::ivec3& bfs_node) const {
//	if (visited_nodes->find(bfs_node) != visited_nodes->end()) {
//		return true;
//	}
//	return false;
//}

bool SwarmOccupancyTree::visited(const SwarmOccupancyTree& visited_nodes, const BFSNode& bfs_node) const {
	auto position = bfs_node.grid_position_;
	int visited_result = visited_nodes.at(position.x, position.z);
	return (visited_result == SEARCH_VISITED);
}

bool SwarmOccupancyTree::is_unexplored_perimeter(const glm::ivec3& grid_position) const {
	if (!is_out_of_bounds(grid_position)) {
		// throw error
	}

	int explored = at(grid_position.x, grid_position.z);

	return (explored == INTERIOR_MARK);
}

void SwarmOccupancyTree::mark_visited(SwarmOccupancyTree& swarm_oct_tree, const BFSNode& bfs_node) const {
	auto pos = bfs_node.grid_position_;
	swarm_oct_tree.set(pos.x, pos.z, SEARCH_VISITED);
}

bool SwarmOccupancyTree::is_out_of_bounds(const glm::ivec3& position) const {
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

int SwarmOccupancyTree::explored_by(const glm::ivec3& position) const {
	if (!is_out_of_bounds(position)) {
		// throw error
	}

	int explored = at(position.x,  position.z);

	return explored;
}

bool SwarmOccupancyTree::has_explored(const glm::ivec3& position) const {
	if (!is_out_of_bounds(position)) {
		// throw error
	}

	int explored = at(position.x, position.z);

	return (explored != empty_value_);
}

bool SwarmOccupancyTree::is_interior(const glm::ivec3& position) const {
	int explored = at(position.x,  position.z);
	return (explored == INTERIOR_MARK);
}

std::vector<glm::vec3> SwarmOccupancyTree::find_adjacent_interiors(const std::vector<glm::ivec3>& adjacent_cells) const {
	std::vector<glm::vec3> interior_cell_positions;
	interior_cell_positions.reserve(10);
	for (auto& adjacent_cell : adjacent_cells) {
		if (is_interior(adjacent_cell)) {
			interior_cell_positions.push_back(map_to_position(adjacent_cell));
		}
	}
	return interior_cell_positions;
}

int SwarmOccupancyTree::get_grid_resolution_per_side() {
	return grid_resolution_per_side_;
}

int SwarmOccupancyTree::get_grid_cube_length() {
	return grid_cube_length_;
}

SwarmCollisionTree::SwarmCollisionTree(unsigned resolution): Quadtree<std::set<int>*>(resolution, nullptr) {
	for (int x = 0; x < resolution_per_side_; ++x) {
		for (int z = 0; z < resolution_per_side_; ++z) {
			auto new_set = new std::set<int>();
			set(x, z, new_set);
		}
	}
}

void SwarmCollisionTree::insert(int robot_id, const glm::ivec3& position) {
	auto robot_positions = at(position.x, position.z);
	robot_positions->insert(robot_id);
}


std::vector<int> SwarmCollisionTree::find_adjacent_robots(int robot_id, const glm::ivec3& position) const {
	// if 3D we need to get 24 cells
	// getting 8 for 2D
	int y = 0;
	std::vector<int> robots;
	robots.reserve(10);

	for (int x = -1; x < 2; ++x) {
		for (int z = -1; z < 2; ++z) {
				glm::ivec3 adjacent_operator(x, y, z);
				glm::ivec3 adjacent_cell = position + adjacent_operator;
				if ((adjacent_cell.x >= 0 && adjacent_cell.x < resolution_per_side_- 1) 
					&& (adjacent_cell.x >= 0 && adjacent_cell.z < resolution_per_side_ -1)) {
					auto robots_in_cell = at(adjacent_cell.x, adjacent_cell.z);
					if (robots_in_cell->size() > 0) {
						for (auto& other_robot_id : *robots_in_cell) {
							if (other_robot_id != robot_id) {
								robots.push_back(other_robot_id);
							}
						}
					}
				}
		}
	}
	return robots;
}


std::vector<int> SwarmCollisionTree::find_adjacent_robots(int robot_id, const std::vector<glm::ivec3>& adjacent_cells) const {
	// if 3D we need to get 24 cells
	// getting 8 for 2D
	int y = 0;
	std::vector<int> robots;
	robots.reserve(10);

	for (auto& adjacent_cell : adjacent_cells) {
		auto robots_in_cell = at(adjacent_cell.x, adjacent_cell.z);
		if (robots_in_cell->size() > 0) {
			for (auto& other_robot_id : *robots_in_cell) {
				if (other_robot_id != robot_id) {
					robots.push_back(other_robot_id);
				}
			}
		}
	}
	return robots;
}

void SwarmCollisionTree::update(int robot_id, const glm::ivec3& previous_position, const glm::ivec3& current_position) {
	if (previous_position == current_position) {
		return;
	}
	//auto previous_position_robot_id = at(previous_position.x, previous_position.z);
	//// check if, already someone else has updated
	//if (previous_position_robot_id == robot_id) {
	//	unset(previous_position.x, previous_position.z);
	//	insert(robot_id, current_position);
	//}

	auto previous_position_robots = at(previous_position.x, previous_position.z);
	auto position_iterator = previous_position_robots->find(robot_id);
	if (position_iterator != previous_position_robots->end()) {
		previous_position_robots->erase(position_iterator);
		insert(robot_id, current_position);
	} else {
		// unexpected behavior
		std::cout << "Robot not found in previous position" << std::endl;
		
	}
}

SwarmCollisionTree::~SwarmCollisionTree() {
	for (int x = 0; x < resolution_per_side_; ++x) {
		for (int z = 0; z < resolution_per_side_; ++z) {
			auto set = at(x, z);
			delete set;
			unset(x, z);
		}
	}
}

bool SwarmOccupancyTree::frontier_bread_first_search(const glm::ivec3& current_position, glm::ivec3& result_cell,
	int max_depth) const {

	//std::unique_ptr < std::queue<BFSNode>> nodes(new std::queue<BFSNode>() );
	//std::unique_ptr<std::set<BFSNode>> visited_nodes(new std::set<BFSNode>());

	//std::unique_ptr < std::queue<BFSNode>> nodes(new std::queue<BFSNode>() );
	//std::unique_ptr<std::set<BFSNode>> visited_nodes(new std::set<BFSNode>());

	std::queue<BFSNode> nodes;
	std::set<BFSNode> visited_nodes;

	//std::queue<glm::ivec3> nodes;
	//std::set<glm::ivec3> visited_nodes;

	//std::unique_ptr<SwarmOccupancyTree> search_octree = 
		//SwarmOccupancyTree(grid_cube_length_, grid_resolution_, SEARCH_NOT_VISITED);

	//search_octree.setEmptyValue(SEARCH_NOT_VISITED);

	nodes.push(BFSNode(current_position));
	//visited_nodes.push_back(BFSNode(current_position));

	int max = -1;
	int current_depth = 0,
		elements_to_depth_increase = 1;
	int not_visited_count = 0;

	bool pending_depth_increase = false;

	while (!nodes.empty()) {
		auto& current_cell = nodes.front();

		// pick frontier
		//if (!has_explored(current_cell.grid_position_)) {
		//	result_cell = current_cell.grid_position_;
		//	return true;
		//}

		if (!is_unexplored_perimeter(current_cell.grid_position_)
			&& !has_explored(current_cell.grid_position_)) {
			std::vector<glm::ivec3> adjacent_cells;
			adjacent_cells.reserve(9);
			get_adjacent_cells(current_cell.grid_position_, adjacent_cells);
			for (auto& adjacent_cell : adjacent_cells) {
				if (is_unexplored_perimeter(adjacent_cell)) {
					result_cell = current_cell.grid_position_;
					//std::cout << "Max size of visited nodes : " << max << std::endl;
					//std::cout << "Not visited count : " << not_visited_count << std::endl;
					return true;
				}
			}
		}

		//if (!is_unexplored_perimeter(current_cell)
		//	&& !has_explored(current_cell)) {
		//	std::vector<glm::ivec3> adjacent_cells;
		//	adjacent_cells.reserve(9);
		//	get_adjacent_cells(current_cell, adjacent_cells);
		//	for (auto& adjacent_cell : adjacent_cells) {
		//		if (is_unexplored_perimeter(adjacent_cell)) {
		//			result_cell = current_cell;
		//			//std::cout << "Max size of visited nodes : " << max << std::endl;
		//			//std::cout << "Not visited count : " << not_visited_count << std::endl;
		//			return true;
		//		}
		//	}
		//}


		// adds only cells within bound, no need to check again
		std::vector<glm::ivec3> adjacent_cells;
		adjacent_cells.reserve(9);
		get_adjacent_cells(current_cell.grid_position_, adjacent_cells);
		//get_adjacent_cells(current_cell, adjacent_cells);


		for (auto& adjacent_cell : adjacent_cells) {
			// inefficient rechecking all adjacent cells, without only checking UNVISITED cells
			// copy octree?
			//BFSNode adjacent_node(adjacent_cell);
			//adjacent_node.parent_grid_position_ = current_cell.grid_position_;
			//if (!visited(visited_nodes.get(), adjacent_node)) {
			//if (!visited(&visited_nodes, adjacent_node)) {
			if (!visited(&visited_nodes, adjacent_cell)) {
				if (pending_depth_increase) {
					elements_to_depth_increase = nodes.size();
					pending_depth_increase = false;

				}
				//nodes.push(adjacent_node);
				//visited_nodes.insert(adjacent_node);
				nodes.push(adjacent_cell);
				visited_nodes.insert(adjacent_cell);
				max = std::max((int)visited_nodes.size(), max);
			}
			//if (!visited(*search_octree, adjacent_node)) {
			//	nodes.push(adjacent_node);
			//	mark_visited(*search_octree, adjacent_node);
			//}
		}

		nodes.pop();

		if (--elements_to_depth_increase == 0) {
			if (++current_depth > max_depth) return false;
			pending_depth_increase = true;
			//elements_to_depth_increase = next_elements_to_depth_increase;
			//next_elements_to_depth_increase = 0;
			//std::cout << "current depth : " << current_depth << std::endl;
			//std::cout << "Max size of visited nodes : " << max << std::endl;
		}

	}

	//std::cout << "Max size of visited nodes : " << max << std::endl;
	//std::cout << "Not visited count : " << not_visited_count << std::endl;

	// all cells explored, we are done
	// no search result
	return false;
}

//void SwarmOccupancyTree::frontier_depth_first_search(const glm::ivec3& current_position, glm::ivec3& result_cell) const {
//
//};

void SwarmOccupancyTree::mark_interior_line(glm::vec3 a, glm::vec3 b) {
	
	
	float length = glm::length(b - a);
	float division_factor = (get_grid_cube_length() / 2.f);
	int no_of_segments = (length / division_factor) + 1;
#ifdef DEBUG
	std::cout << "No of segments : " << no_of_segments << std::endl;
#endif


	glm::vec3 direction;
	if (length > 1e-3) {
		direction = glm::normalize(b - a);
	}

	for (int i = 0; i < no_of_segments; ++i) {
		glm::vec3 position = a + direction * (division_factor) * static_cast<float>(i);
		try {
			auto grid_position = map_to_grid(position);
			set(grid_position.x, grid_position.z, INTERIOR_MARK);
			//SwarmUtils::print_vector("mark interior A", a);
			//SwarmUtils::print_vector("mark interior B", b);
			SwarmUtils::print_vector("marking grid position", grid_position);
			SwarmUtils::print_vector("marking position", position);
		} catch (OutOfGridBoundsException& ex) {
			// ignore
		}
	}
}
