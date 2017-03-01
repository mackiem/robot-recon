#include "swarmtree.h"


#include <queue>
#include <memory>
#include "swarmutils.h"
#include <map>
#include <functional>
#include <chrono>
#include <random>
#include <glm/detail/type_mat.hpp>
#include <glm/detail/type_mat.hpp>
#include <glm/detail/type_mat.hpp>
#include <glm/detail/type_mat.hpp>
#include <glm/detail/type_mat.hpp>

//#include <vld.h>

using namespace mm;

// hoping there won't be 100k robots ever!
int SwarmOccupancyTree::INTERIOR_MARK = 100000;
int SwarmOccupancyTree::PERIMETER = 5000;

int SwarmOccupancyTree::SEARCH_VISITED = 1;
int SwarmOccupancyTree::SEARCH_NOT_VISITED = 0;



glm::ivec3 SwarmOccupancyTree::map_to_grid(const glm::vec3& position) const  {
	glm::ivec3 grid_position;
	mm::Quadtree<int>::map_to_grid(position.x, position.z, grid_position.x, grid_position.z);

	return grid_position;
}

//glm::ivec3 Swarm3DReconTree::map_to_grid(const glm::vec3& position) const  {
//	glm::vec3 grid_pos_float =  (position / static_cast<float>(grid_cube_length_));
//	glm::ivec3 grid_pos(grid_pos_float.x, grid_pos_float.y, grid_pos_float.z);
//
//	// if greater than grid resolution, throw error
//	if (is_out_of_bounds(grid_pos)) {
//		throw OutOfGridBoundsException("Out of bounds");
//	}
//
//	return grid_pos;
//}
//
//bool Swarm3DReconTree::is_out_of_bounds(const glm::ivec3& position) const {
//	int max_grid_no = resolution_per_side_ - 1;
//	if (position.x > max_grid_no
//		|| position.y > max_grid_no
//		|| position.z > max_grid_no) {
//		return true;
//	}
//
//	
//	if (position.x < 0
//		// only 2D commenting this out
//		//|| position.y <= 0
//		|| position.z < 0) {
//		return true;
//	}
//
//	return false;
//}

glm::vec3 SwarmOccupancyTree::map_to_position(const glm::ivec3& grid_position)  const {
	glm::vec3 position;
	mm::Quadtree<int>::map_to_position(grid_position.x, grid_position.z, position.x, position.z);
	return position;
}

SwarmOccupancyTree::BFSNode::BFSNode(glm::ivec3 grid_position) : grid_position_(grid_position) {
}

int SwarmOccupancyTree::get_interior_mark() {
	return INTERIOR_MARK;
}

void SwarmOccupancyTree::mark_floor_plan() {
	int y = 0;
	int edge_x = grid_width_ - 1;
	int edge_z = grid_height_ - 1;

	for (int x = 0; x < grid_width_; ++x) {
		for (int z = 0; z < grid_height_; ++z) {
			if (x == 0 || x == edge_x || z == 0 || z == edge_z) {
				set(x, z, INTERIOR_MARK);
			}
		}
	}
}

void Swarm3DReconTree::mark_floor_plan() {
	int y = 0;
	int edge_x = grid_width_ - 1;
	int edge_z = grid_height_ - 1;

	for (int x = 0; x < grid_width_; ++x) {
		for (int z = 0; z < grid_height_; ++z) {
			if (x == 0 || x == edge_x || z == 0 || z == edge_z) {
				glm::vec3 points((x + 0.5) * grid_cube_length_, y, (x + 0.5) * grid_cube_length_);
				insert(points, glm::ivec3(x, y, z));
			}
		}
	}
}


//SwarmOccupancyTree::SwarmOccupancyTree(float grid_square_length, int grid_height, int grid_width) :
//Quadtree<char>(grid_width, grid_height, grid_square_length, -1) {
//	//grid_resolution_per_side_ = get_resolution_per_side();
//	//offset_ = glm::ivec3(grid_resolution_per_side_ / 2, 0, grid_resolution_per_side_ / 2);
//	offset_ = glm::ivec3(0, 0, 0);
//	//empty_value_ = -1;
//	//setEmptyValue(empty_value_);
//	mark_floor_plan();
//	//create_perimeter_list();
//
//	// create pool
//	pool_size_ = 100000;
//	current_pool_count_ = 100000;
//	heap_pool_.resize(pool_size_);
//
//	//sampling_tracker_ = std::make_shared<std::unordered_map<glm::ivec3, 
//	//	std::unordered_map<int, std::unordered_map<int, int>>, IVec3Hasher, IVec3Equals>>();
//
//	sampling_tracker_ = new std::vector<Sampling>();
//	update_multisampling_ = false;
//	//sampling_tracker_->reserve(pool_size_);
//	//sampling_tracker_ = new std::unordered_map<glm::ivec3, std::map<int, std::map<int, int>>
//	//	, IVec3Hasher, IVec3Equals>();
//	//sampling_tracker_ = std::make_shared < std::map < glm::ivec3, std::map < int, std::map<int, int> >
//	//	, IVec3Comparator >> ();
//
//	//heap_pool_.resize(pool_size_);
//	//std::fill(heap_pool_.begin(), heap_pool_.end(), )
//	//leak_ = new float();
//}

SwarmOccupancyTree::SwarmOccupancyTree(int grid_cube_length, int grid_width, int grid_height, char empty_value) : 
	Quadtree<int>(grid_width, grid_height, grid_cube_length, empty_value) {

	offset_ = glm::ivec3(0, 0, 0);
	mark_floor_plan();
	sampling_tracker_ = new std::vector<Sampling>();
	update_multisampling_ = false;

	// create pool
	pool_size_ = 100000;
	current_pool_count_ = 100000;
	heap_pool_.resize(pool_size_);

	perimeter_vector_.resize(grid_width * grid_height);

	grid_stats_.resize(grid_width * grid_height);
}

 std::set<glm::ivec3, IVec3Comparator> SwarmOccupancyTree::get_unexplored_perimeter_list() {
	 return explore_perimeter_list_;
}

 int SwarmOccupancyTree::no_of_unexplored_cells() {
	 //return explore_perimeter_list_.size();
	 return explore_interior_list_.size();
}

 int SwarmOccupancyTree::no_of_interior_cells() const {
	 //return explore_perimeter_list_.size();
	 return interior_list_.size();
}

std::set<glm::ivec3, IVec3Comparator> SwarmOccupancyTree::get_static_perimeter_list() {
	 return static_perimeter_list_;
}

std::set<glm::ivec3, IVec3Comparator> SwarmOccupancyTree::get_interior_list() {
	 return interior_list_;
}

void SwarmOccupancyTree::get_adjacent_cells(const glm::ivec3& position, std::vector<glm::ivec3>& cells, int sensor_range) const {
	// if 3D we need to get 24 cells
	// getting 8 for 2D
	int y = 0;
	//std::vector<glm::ivec3> cells;
	//cells.reserve(9);

	for (int x = -sensor_range; x < sensor_range + 1; ++x) {
		for (int z = -sensor_range; z < sensor_range + 1; ++z) {
			//if (x != 0 && z != 0) {
				glm::ivec3 adjacent_operator(x, y, z);
				glm::ivec3 adjacent_cell = position + adjacent_operator;
				if (!mm::Quadtree<int>::is_out_of_bounds(adjacent_cell.x, adjacent_cell.z)) {
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

//bool SwarmOccupancyTree::is_unexplored_perimeter(const glm::ivec3& grid_position) const {
//	if (!is_out_of_bounds(grid_position)) {
//		// throw error
//	}
//
//	int explored = at(grid_position.x, grid_position.z);
//
//	return (explored == INTERIOR_MARK);
//}

void SwarmOccupancyTree::mark_visited(SwarmOccupancyTree& swarm_oct_tree, const BFSNode& bfs_node) const {
	auto pos = bfs_node.grid_position_;
	swarm_oct_tree.set(pos.x, pos.z, SEARCH_VISITED);
}

bool SwarmOccupancyTree::is_out_of_bounds(const glm::ivec3& position) const {
	return mm::Quadtree<int>::is_out_of_bounds(position.x, position.z);
}

int SwarmOccupancyTree::explored_by(const glm::ivec3& position) const {
	if (!mm::Quadtree<int>::is_out_of_bounds(position.x, position.z)) {
		// throw error
	}

	int explored = at(position.x,  position.z);

	return explored;
}

bool SwarmOccupancyTree::has_explored(const glm::ivec3& position) const {
	if (!mm::Quadtree<int>::is_out_of_bounds(position.x, position.z)) {
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
			glm::vec3 adj_pos;
			mm::Quadtree<int>::map_to_position(adjacent_cell.x, adjacent_cell.z, adj_pos.x, adj_pos.z);
			interior_cell_positions.push_back(adj_pos);
		}
	}
	return interior_cell_positions;
}

//int SwarmOccupancyTree::get_grid_resolution_per_side() {
//	return grid_resolution_per_side_;
//}
//
//int SwarmOccupancyTree::get_grid_cube_length() {
//	return grid_cube_length_;
//}

SwarmCollisionTree::SwarmCollisionTree(unsigned width, unsigned height): Quadtree<std::set<int>*>(width, height, 1, nullptr) {

	int edge_x = grid_width_ - 1;
	int edge_z = grid_height_ - 1;

	for (int x = 0; x < grid_width_; ++x) {
		for (int z = 0; z < grid_height_; ++z) {
			auto new_set = new std::set<int>();
			set(x, z, new_set);
		}
	}
}

void SwarmCollisionTree::insert(int robot_id, const glm::ivec3& position) {
	auto robot_positions = at(position.x, position.z);
	robot_positions->insert(robot_id);
}

Swarm3DReconTree::Swarm3DReconTree(float grid_cube_length, int grid_width, int grid_height): 
Quadtree<std::vector<glm::vec3>*>(grid_width, grid_height, grid_cube_length, nullptr) {

	for (int x = 0; x < grid_width_; ++x) {
		for (int z = 0; z < grid_height_; ++z) {
			auto new_vector = new std::vector<glm::vec3>();
			set(x, z, new_vector);
		}
	}
	mark_floor_plan();
}

void Swarm3DReconTree::insert(glm::vec3& points, const glm::ivec3& position) {
	auto points_3d = at(position.x, position.z);
	points_3d->push_back(points);
	multi_sampling_map_[position] = 0;
}

std::vector<glm::vec3>* Swarm3DReconTree::get_3d_points(const glm::ivec3& position) {
	auto points_3d = at(position.x, position.z);
	return points_3d;
}

void Swarm3DReconTree::update_multi_sampling_map(const glm::ivec3& position) {
	multi_sampling_map_[position]++;
}

Swarm3DReconTree::~Swarm3DReconTree() {
	for (int x = 0; x < grid_width_; ++x) {
		for (int z = 0; z < grid_height_; ++z) {
			auto container = at(x, z);
			delete container;
			unset(x, z);
		}
	}
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

				if (!is_out_of_bounds(adjacent_cell.x, adjacent_cell.z)) {
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

void SwarmCollisionTree::find_adjacent_robots_memory_save(int robot_id, const std::vector<VisibleCell>& adjacent_cells, const int current_adjacent_cells,
	std::vector<int>& adjacent_robots, int& current_no_of_robots) const {

	// if 3D we need to get 24 cells
	// getting 8 for 2D
	//int y = 0;
	//std::vector<int> robots;
	//robots.reserve(10);

	current_no_of_robots = 0;

	for (int i = 0; i < current_adjacent_cells; ++i) {
		auto& visibility_aware_adjacent_cell = adjacent_cells[i];
		if (!visibility_aware_adjacent_cell.is_visible()) {
			continue;
		}
		auto& adjacent_cell = visibility_aware_adjacent_cell.cell;
		auto robots_in_cell = at(adjacent_cell.x, adjacent_cell.z);
		if (robots_in_cell->size() > 0) {
			for (auto& other_robot_id : *robots_in_cell) {
				if (other_robot_id != robot_id) {
					adjacent_robots[current_no_of_robots++] = other_robot_id;
				}
			}
		}
	}
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
	for (int x = 0; x < grid_width_; ++x) {
		for (int z = 0; z < grid_height_; ++z) {
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

		if (!is_interior(current_cell.grid_position_)
			&& !has_explored(current_cell.grid_position_)) {
			std::vector<glm::ivec3> adjacent_cells;
			adjacent_cells.reserve(9);
			get_adjacent_cells(current_cell.grid_position_, adjacent_cells, 1);
			for (auto& adjacent_cell : adjacent_cells) {
				if (is_interior(adjacent_cell)) {
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
		get_adjacent_cells(current_cell.grid_position_, adjacent_cells, 1);
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
	float division_factor = (get_grid_square_length() / 2.f);
	int no_of_segments = (length / division_factor) + 1;
#ifdef DEBUG
	//std::cout << "No of segments : " << no_of_segments << std::endl;
#endif

	glm::vec3 direction;
	if (length > 1e-3) {
		direction = glm::normalize(b - a);
	}

	for (int i = 0; i < no_of_segments; ++i) {
		glm::vec3 position = a + direction * (division_factor) * static_cast<float>(i);
		try {
			glm::ivec3 grid_position;
			mm::Quadtree<int>::map_to_grid(position.x, position.z, grid_position.x, grid_position.y);
			set(grid_position.x, grid_position.z, INTERIOR_MARK);
			//SwarmUtils::print_vector("mark interior A", a);
			//SwarmUtils::print_vector("mark interior B", b);
			//SwarmUtils::print_vector("marking grid position", grid_position);
			//SwarmUtils::print_vector("marking position", position);
		} catch (OutOfGridBoundsException& ex) {
			// ignore
		}
	}
}

void Swarm3DReconTree::mark_interior_line(glm::vec3 a, glm::vec3 b) {

	float length = glm::length(b - a);
	float division_factor = (grid_cube_length_ / 6.f);
	int no_of_segments = (length / division_factor) + 1;

	glm::vec3 direction;
	if (length > 1e-3) {
		direction = glm::normalize(b - a);
	}

	for (int i = 0; i < no_of_segments; ++i) {
		glm::vec3 position = a + direction * (division_factor) * static_cast<float>(i);
		try {
			glm::ivec3 grid_position;
			map_to_grid(position.x, position.z, grid_position.x, grid_position.y);
			insert(position, grid_position);
			//SwarmUtils::print_vector("mark interior A", a);
			//SwarmUtils::print_vector("mark interior B", b);
			//SwarmUtils::print_vector("marking grid position", grid_position);
			//SwarmUtils::print_vector("marking position", position);
		} catch (OutOfGridBoundsException& ex) {
			// ignore
		}
	}
}

double Swarm3DReconTree::calculate_multi_sampling_factor() {
	int total_no_of_samples = 0;
	for (auto& map_entry : multi_sampling_map_) {
		total_no_of_samples += map_entry.second;
	}
	return (multi_sampling_map_.size() > 0)  ? (total_no_of_samples / double(multi_sampling_map_.size())) : 0.0;
}

double Swarm3DReconTree::calculate_density() {
	int total_no_of_points = 0;
	int sampled_no_of_points = 0;


	for (int x = 0; x < grid_width_; ++x) {
		for (int z = 0; z < grid_height_; ++z) {
			auto grid_position = glm::ivec3(x, 0, z);
			auto points_3d = get_3d_points(grid_position);
			if (points_3d->size() > 0) {
				total_no_of_points++;
				int no_of_samples = multi_sampling_map_[grid_position];
				if (no_of_samples > 0) {
					sampled_no_of_points++;
				}
			}
		}
	}
	return (total_no_of_points > 0) ? sampled_no_of_points / (double)(total_no_of_points) : 0.0;
}

void Swarm3DReconTree::mark_random_points_in_triangle(glm::vec3 a, glm::vec3 b, glm::vec3 c) {

	std::random_device rd;
	std::mt19937 eng(rd());
	std::uniform_real_distribution<double> dist(0, 1);

	int no_of_points_per_unit_area = 1;

	// max length of plane
	glm::vec3 ab = b - a;
	glm::vec3 bc = c - b;
	glm::vec3 ca = a - c;

	float ab_len = glm::length(ab);
	float bc_len = glm::length(bc);
	float ca_len = glm::length(ca);

	float p = (ab_len + bc_len + ca_len) / 2.f;
	
	// heron's formula
	float triangle_area = std::sqrt(p * (p - ab_len) * (p - bc_len) * (p - ca_len));
	int triangle_area_iterations = 20;
	if (triangle_area > 1.f) {
		triangle_area_iterations = (triangle_area > 100.f) ? std::min(triangle_area * 0.2f, 1250.f) : triangle_area_iterations;
	}


	for (int i = 0; i < (triangle_area_iterations); ++i) {
		for (int j = 0; j < no_of_points_per_unit_area; ++j) {

			double r1 = dist(eng);
			double r2 = dist(eng);

			// barycentric coordinates
			if ((r1 + r2) > 1.0) {
				r1 = 1.0 - r1;
				r2 = 1.0 - r2;
			}
			double r3 = 1.0 - (r1 + r2);

			
			//glm::vec3 p_in_triangle = (1 - sqrt(r1)) * a + (sqrt(r1) * (1 - r2)) * b + (sqrt(r1) * r2) * c;
			glm::vec3 p_in_triangle = (float)r1 * a + ((float)r2) * b + (float(r3)) * c;

			try {
				auto xz_position = p_in_triangle;
				xz_position.y = 0.f;

				glm::ivec3 grid_position;
				map_to_grid(xz_position.x, xz_position.z, grid_position.x, grid_position.y);

				//auto grid_position = map_to_grid(xz_position);

				insert(p_in_triangle, grid_position);
			}
			catch (OutOfGridBoundsException& ex) {
				// ignore
				std::cout << "Out of grid while deriving points\n";
			}
		}
	}
	
}

void SwarmOccupancyTree::remove_inner_interiors() {
	for (int x = 0; x < grid_width_; ++x) {
		for (int z = 0; z < grid_height_; ++z) {
			glm::ivec3 grid_position(x, 0, z);

			if (is_interior(grid_position)) {
				std::vector<glm::ivec3> adjacent_cells;
				adjacent_cells.reserve(9);
				get_adjacent_cells(grid_position, adjacent_cells, 1);
				bool unnecessary_interior = true;
				for (auto& adjacent_cell : adjacent_cells) {
					if (grid_position != adjacent_cell && !is_interior(adjacent_cell)) {
						unnecessary_interior = false;
						break;
					}
				}
				if (unnecessary_interior) {
					set(grid_position.x, grid_position.z, empty_value_);
				}
			}
		}
	}

}


void SwarmOccupancyTree::update_interior_stats(glm::ivec3 grid_cell, int timestep, int robot_id, int cluster_id) {
	//auto entry = sampling_tracker_->find(grid_cell);
	//if (entry != sampling_tracker_->end()) {
		//entry->second[timestep][robot_id] = 1;
	//}
	//(*sampling_tracker_)[grid_cell][timestep][robot_id] = 1;
	auto& stat = grid_stats_[grid_cell.x * grid_height_ + grid_cell.z];
	if (stat.last_timestamp != timestep) {
		// new timestamp
		stat.last_timestamp = timestep;
		stat.current_simul_samples = 0;
	}

	stat.current_simul_samples++;
	stat.max_simul_samples = std::max(stat.max_simul_samples, stat.current_simul_samples);

	/*
	Sampling sample;
	sample.grid_cell = grid_cell;
	sample.timestamp = timestep;
	sample.robot_id = robot_id;
	sample.cluster_id = cluster_id;
	sampling_tracker_->push_back(sample);

	if (sampling_tracker_->size() % 100000 == 0) {
		//std::cout << "tracker memory : " << sampling_tracker_->size() * sizeof(Sampling) / (1024 * 1024) << "\n";
		last_multisample_timestep_ = timestep;
		update_multisampling_ = true;
	}

	if (update_multisampling_) {
		if (timestep != last_multisample_timestep_) {
			calculate_simultaneous_sampling_per_cluster();
			sampling_tracker_->clear();
			update_multisampling_ = false;
		}
	}
	*/
}

SimSampMap SwarmOccupancyTree::calculate_simultaneous_sampling_per_grid_cell() {

	//calculate_simultaneous_sampling_per_cluster();

	SimSampMap simultaneous_sampling_map;

	//for (auto& sample_per_timestep : no_of_simul_samples_per_timestep_per_gridcell) {
	//	auto grid_cell = sample_per_timestep.first;
	//	auto no_of_total_samples_per_grid_cell = sample_per_timestep.second;
	//	// we have to know the actual timesteps this was sampled
	//	auto no_of_timesteps_grid_cell_was_sampled =  no_of_sampled_timesteps_per_gridcell[grid_cell];
	//	simultaneous_sampling_map[grid_cell] = (double) no_of_total_samples_per_grid_cell / no_of_timesteps_grid_cell_was_sampled;
	//}
	for (int x = 0; x < grid_width_; ++x) {
		for (int z = 0; z < grid_height_; ++z) {
			glm::ivec3 grid_cell(x, 0, z);
			if (is_interior(grid_cell) && !is_interior_interior(grid_cell)) {
				auto& stat = grid_stats_[grid_cell.x * grid_height_ + grid_cell.z];
				simultaneous_sampling_map[grid_cell] = (double)stat.max_simul_samples;
			}
		}
	}

	return simultaneous_sampling_map;
}

SimSampMap SwarmOccupancyTree::calculate_multi_sampling_per_grid_cell() {

	calculate_simultaneous_sampling_per_cluster();

	SimSampMap simultaneous_sampling_map;

	for (auto& sample_per_timestep : no_of_simul_samples_per_timestep_per_gridcell) {
		auto grid_cell = sample_per_timestep.first;
		auto no_of_total_samples_per_grid_cell = sample_per_timestep.second;
		// we have to know the actual timesteps this was sampled
		auto no_of_timesteps_grid_cell_was_sampled =  no_of_sampled_timesteps_per_gridcell[grid_cell];
		simultaneous_sampling_map[grid_cell] = (double) no_of_total_samples_per_grid_cell / no_of_timesteps_grid_cell_was_sampled;
	}

	return simultaneous_sampling_map;
}

double SwarmOccupancyTree::calculate_simultaneous_sampling_factor() {

	//calculate_simultaneous_sampling_per_cluster();

	//double sampling_factor = 0.0;

	//for (auto& sample_per_timestep : no_of_simul_samples_per_timestep_per_gridcell) {
	//	auto grid_cell = sample_per_timestep.first;
	//	auto no_of_total_samples_per_grid_cell = sample_per_timestep.second;
	//	// we have to know the actual timesteps this was sampled
	//	auto no_of_timesteps_grid_cell_was_sampled =  no_of_sampled_timesteps_per_gridcell[grid_cell];
	//	sampling_factor += (double) no_of_total_samples_per_grid_cell / no_of_timesteps_grid_cell_was_sampled;
	//}

	//if (no_of_sampled_timesteps_per_gridcell.size() > 0) {
	//	sampling_factor /= interior_list_.size();
	//}

	/*
	double sampling_factor = 0.0;

	auto simultaneous_sampling_map = calculate_simultaneous_sampling_per_grid_cell();

	for (auto& cell_entry : simultaneous_sampling_map) {
		auto simultaneous_sampling_per_grid_cell = cell_entry.second;
		sampling_factor += simultaneous_sampling_per_grid_cell;
	}

	if (simultaneous_sampling_map.size() > 0) {
		sampling_factor /= interior_list_.size();
	}

	return sampling_factor;
	*/

	SimSampMap map = calculate_simultaneous_sampling_per_grid_cell();
	double avg_sim_sampling = 0.0;

	for (auto& entry : map) {
		avg_sim_sampling += entry.second;
	}
	
	if (interior_list_.size() > 0) {
		avg_sim_sampling /= interior_list_.size();
	}
	return avg_sim_sampling;

}

double SwarmOccupancyTree::calculate_multi_sampling_factor() {

	//calculate_simultaneous_sampling_per_cluster();

	//double sampling_factor = 0.0;

	//for (auto& sample_per_timestep : no_of_simul_samples_per_timestep_per_gridcell) {
	//	auto grid_cell = sample_per_timestep.first;
	//	auto no_of_total_samples_per_grid_cell = sample_per_timestep.second;
	//	// we have to know the actual timesteps this was sampled
	//	auto no_of_timesteps_grid_cell_was_sampled =  no_of_sampled_timesteps_per_gridcell[grid_cell];
	//	sampling_factor += (double) no_of_total_samples_per_grid_cell / no_of_timesteps_grid_cell_was_sampled;
	//}

	//if (no_of_sampled_timesteps_per_gridcell.size() > 0) {
	//	sampling_factor /= interior_list_.size();
	//}

	double sampling_factor = 0.0;

	auto simultaneous_sampling_map = calculate_multi_sampling_per_grid_cell();

	for (auto& cell_entry : simultaneous_sampling_map) {
		auto simultaneous_sampling_per_grid_cell = cell_entry.second;
		sampling_factor += simultaneous_sampling_per_grid_cell;
	}

	if (simultaneous_sampling_map.size() > 0) {
		sampling_factor /= interior_list_.size();
	}

	return sampling_factor;
}

bool SwarmOccupancyTree::is_interior_interior(const glm::ivec3& position) {
	std::vector<glm::ivec3> cells;
	get_adjacent_cells(position, cells, 1);
	bool is_interior_of_interior = true;
	for (auto& cell : cells) {
		if (!is_interior(cell)) {
			is_interior_of_interior = false;
			break;
		}
	}
	return is_interior_of_interior;
}

void SwarmOccupancyTree::calculate_simultaneous_sampling_per_cluster() {


	double sampling_factor = 0.0;

	long long last_timestamp = -1;
	std::unordered_map<glm::ivec3, int, IVec3Hasher, IVec3Equals> simultaneous_samples_per_timestamp;

	int no_of_timesteps = 0;

	for (auto& sampling_tracker_entry : *sampling_tracker_) {

		// the tracker will be ordered by timestamp
		if (sampling_tracker_entry.timestamp != last_timestamp) {
			for (auto& simultaneous_samples : simultaneous_samples_per_timestamp) {
				glm::ivec3 grid_cell = simultaneous_samples.first;
				if (no_of_simul_samples_per_timestep_per_gridcell.find(grid_cell) != no_of_simul_samples_per_timestep_per_gridcell.end()) {
					no_of_simul_samples_per_timestep_per_gridcell[grid_cell] += simultaneous_samples.second;
					no_of_sampled_timesteps_per_gridcell[grid_cell]++;
				} else {
					no_of_simul_samples_per_timestep_per_gridcell[grid_cell] = simultaneous_samples.second;
					no_of_sampled_timesteps_per_gridcell[grid_cell] = 1;
				}
			}
			last_timestamp = sampling_tracker_entry.timestamp;
			simultaneous_samples_per_timestamp.clear();
			no_of_timesteps++;
		}
		if (interior_list_.find(sampling_tracker_entry.grid_cell) != interior_list_.end()) {
			if (!is_interior_interior(sampling_tracker_entry.grid_cell)) {
				if (simultaneous_samples_per_timestamp.find(sampling_tracker_entry.grid_cell) == simultaneous_samples_per_timestamp.end()) {
					simultaneous_samples_per_timestamp[sampling_tracker_entry.grid_cell] = 1;
				}
				else {
					simultaneous_samples_per_timestamp[sampling_tracker_entry.grid_cell] += 1;
				}
			}
		}
	}

	//// no of samples per gridcell per cluster per timestamp
	//double sampling_factor = 0.0;

	//long long last_timestamp = -1;
	//std::unordered_map<glm::ivec3, std::unordered_map<int, int>, IVec3Hasher, IVec3Equals> simultaneous_samples_per_timestamp;

	//int no_of_timesteps = 0;

	//for (auto& sampling_tracker_entry : *sampling_tracker_) {

	//	// the tracker will be ordered by timestamp
	//	if (sampling_tracker_entry.timestamp != last_timestamp) {
	//		for (auto& simultaneous_samples : simultaneous_samples_per_timestamp) {
	//			glm::ivec3 grid_cell = simultaneous_samples.first;
	//			if (no_of_simul_samples_per_timestep_per_gridcell.find(grid_cell) != no_of_simul_samples_per_timestep_per_gridcell.end()) {
	//				no_of_simul_samples_per_timestep_per_gridcell[grid_cell] += simultaneous_samples.second;
	//				no_of_sampled_timesteps_per_gridcell[grid_cell]++;
	//			} else {
	//				no_of_simul_samples_per_timestep_per_gridcell[grid_cell] = simultaneous_samples.second;
	//				no_of_sampled_timesteps_per_gridcell[grid_cell] = 1;
	//			}
	//		}
	//		last_timestamp = sampling_tracker_entry.timestamp;
	//		simultaneous_samples_per_timestamp.clear();
	//		no_of_timesteps++;
	//	}
	//	if (interior_list_.find(sampling_tracker_entry.grid_cell) != interior_list_.end()) {
	//		if (simultaneous_samples_per_timestamp.find(sampling_tracker_entry.grid_cell) == simultaneous_samples_per_timestamp.end()) {
	//			simultaneous_samples_per_timestamp[sampling_tracker_entry.grid_cell] = 1;
	//		} else {
	//			simultaneous_samples_per_timestamp[sampling_tracker_entry.grid_cell] += 1;
	//		}
	//	}
	//}

}

double SwarmOccupancyTree::calculate_coverage() {
	double coverage = 0.0;

	if (interior_list_.size() > 0) {
		coverage = 1.0 - (no_of_unexplored_cells() / (double)(interior_list_.size()));
	}

	
	return coverage;
}

bool SwarmOccupancyTree::is_perimeter(const glm::ivec3& grid_position) const {
	if (!is_interior(grid_position)) {
		std::vector<glm::ivec3> adjacent_cells;
		adjacent_cells.reserve(9);
		get_adjacent_cells(grid_position, adjacent_cells, 1);
		for (auto& adjacent_cell : adjacent_cells) {
			if (is_interior(adjacent_cell)) {
				return true;
			}
		}
	}
	return false;
}

void SwarmOccupancyTree::create_perimeter_list() {
	for (int x = 0; x < grid_width_; ++x) {
		for (int z = 0; z < grid_height_; ++z) {
			glm::ivec3 grid_position(x, 0, z);
			if (!is_interior(grid_position)) {
				std::vector<glm::ivec3> adjacent_cells;
				adjacent_cells.reserve(9);
				get_adjacent_cells(grid_position, adjacent_cells, 1);
				for (auto& adjacent_cell : adjacent_cells) {
					if (is_interior(adjacent_cell)) {
						explore_perimeter_list_.insert(grid_position);
					}
				}
			}
		}
	}
	static_perimeter_list_ = explore_perimeter_list_;
}

void SwarmOccupancyTree::create_interior_list() {
	for (int x = 0; x < grid_width_; ++x) {
		for (int z = 0; z < grid_height_; ++z) {
			glm::ivec3 grid_position(x, 0, z);
			if (is_interior(grid_position)) {
				std::vector<glm::ivec3> adjacent_cells;
				adjacent_cells.reserve(9);
				get_adjacent_cells(grid_position, adjacent_cells, 1);
				bool perimeter_found = false;
				for (auto& adjacent_cell : adjacent_cells) {
					if (!is_interior(adjacent_cell)) {
						perimeter_found = true;
						break;
					}
				}

				if (perimeter_found) {
					interior_list_.insert(grid_position);
					//std::unordered_map<int, std::unordered_map<int, int>> timestamp_robots;
					std::map<int, std::map<int, int>> timestamp_robots;
					//timestamp_robots.rereserve(100);
					//for (int i = -10; i < 0; ++i) {
					//	std::unordered_map<int, int> temp_map;
					//	for (int j = -10; j < 0; ++j) {
					//		temp_map[j] = 0;
					//	}

					//	timestamp_robots[i] = temp_map;
					//}
					//(*sampling_tracker_)[grid_position] = timestamp_robots;
				}
			}
		}
	}
	explore_interior_list_ = interior_list_;
}


void SwarmOccupancyTree::create_empty_space_list() {
	for (int x = 0; x < grid_width_; ++x) {
		for (int z = 0; z < grid_height_; ++z) {
			glm::ivec3 grid_position(x, 0, z);
			if (!is_interior(grid_position)) {
				empty_space_list_.insert(grid_position);
			}
		}
	}
}

void SwarmOccupancyTree::init_coverage_map(std::unordered_map<glm::ivec3, int, IVec3Hasher, IVec3Equals>& coverage_map) const
{
	for (int x = 0; x < grid_width_; ++x) {
		for (int z = 0; z < grid_height_; ++z) {
			glm::ivec3 grid_position(x, 0, z);
			if (!is_interior(grid_position)) {
				coverage_map[grid_position] = 0;
			}
		}
	}
	
}

bool SwarmOccupancyTree::mark_explored_in_list(std::set<glm::ivec3, IVec3Comparator>& position_list, const glm::ivec3& grid_position) {
	auto result = position_list.find(grid_position);
	if (result != position_list.end()) {
		position_list.erase(result);
		return true;
	}
	return false;
}

bool SwarmOccupancyTree::is_unexplored_perimeter(const glm::ivec3& grid_position) const {
	auto result = explore_perimeter_list_.find(grid_position);
	return (result != explore_perimeter_list_.end());
	
}

SwarmOccupancyTree::~SwarmOccupancyTree() {
	//sampling_tracker_->clear();
	delete sampling_tracker_;
}

bool SwarmOccupancyTree::mark_explored_in_interior_list(const glm::ivec3& grid_position) {
	return mark_explored_in_list(explore_interior_list_, grid_position);
}

void SwarmOccupancyTree::mark_explored_in_perimeter_list(const glm::ivec3& grid_position) {
	mark_explored_in_list(explore_perimeter_list_, grid_position);
}

void SwarmOccupancyTree::mark_explored_in_empty_space_list(const glm::ivec3& grid_position) {
	mark_explored_in_list(empty_space_list_, grid_position);
}

bool SwarmOccupancyTree::find_closest_empty_space(const glm::ivec3& robot_grid_position,
	glm::ivec3& perimeter_position) {

	return find_closest_position_from_list(empty_space_list_, robot_grid_position, perimeter_position);
}


bool SwarmOccupancyTree::find_closest_perimeter(const glm::ivec3& robot_grid_position,
	glm::ivec3& perimeter_position) {

	return find_closest_position_from_list(static_perimeter_list_, robot_grid_position, perimeter_position);
}

bool SwarmOccupancyTree::going_through_interior_test(const glm::ivec3& robot_position, const glm::ivec3& point_to_test) const {

	float length = glm::length((glm::vec3(point_to_test - robot_position)));
	float division_factor = (1.f / 4.f);
	int no_of_segments = (length / division_factor) + 1;
#ifdef DEBUG
	//std::cout << "No of segments : " << no_of_segments << std::endl;
#endif

	glm::vec3 direction;
	if (length > 1e-3) {
		direction = glm::normalize(glm::vec3(point_to_test - robot_position));
	}

	bool interior_found = false;
	for (int i = 0; i < no_of_segments; ++i) {
		glm::vec3 testing_grid_position = glm::vec3(robot_position) + direction * (division_factor)* static_cast<float>(i);
		if (is_interior(testing_grid_position)) {
			interior_found = true;
			break;
		}
	}

	return interior_found;
}

bool SwarmOccupancyTree::find_closest_position_from_list(const std::set<glm::ivec3, IVec3Comparator>& perimeter_list,
	const glm::ivec3& robot_grid_position,
	glm::ivec3& explore_position) {

	// create a pool, to stop micro memory allocations all the time
	if (current_pool_count_ == pool_size_) {
		current_pool_count_ = 0;
		std::vector<PerimeterPos> sample_vector;
		sample_vector.reserve(perimeter_list.size());
		std::fill(heap_pool_.begin(), heap_pool_.end(), sample_vector);
	}


	std::vector<PerimeterPos>& perimeter_vector = heap_pool_[current_pool_count_++];

	int k = 0;
	for (auto itr = perimeter_list.begin(); itr != perimeter_list.end(); ++itr) {
		auto perimeter_grid_position = *itr;
		float distance = glm::length(glm::vec3(perimeter_grid_position - robot_grid_position));
		perimeter_vector.push_back(PerimeterPos(distance, perimeter_grid_position));
	}

	std::sort(perimeter_vector.begin(), perimeter_vector.end());
	glm::vec3 a = (robot_grid_position);
	bool perimeter_found = false;

	for (auto j = 0; j < perimeter_vector.size(); ++j) {
		glm::vec3 b = perimeter_vector[j].grid_position_;
		bool interior_found = going_through_interior_test(a, b);

		if (!interior_found) {
			explore_position = glm::ivec3(b);
			return true;
		}
	}

	return false;
}

bool SwarmOccupancyTree::find_closest_position_from_list(const std::set<glm::ivec3, IVec3Comparator>& perimeter_list,
	const glm::ivec3& robot_grid_position,
	glm::ivec3& explore_position, float range_min, float range_max) {

	// create a pool, to stop micro memory allocations all the time
	if (current_pool_count_ == pool_size_) {
		current_pool_count_ = 0;
		std::vector<PerimeterPos> sample_vector;
		sample_vector.reserve(perimeter_list.size());
		//heap_pool_.clear();
		//heap_pool_.resize(pool_size_);
		std::fill(heap_pool_.begin(), heap_pool_.end(), sample_vector);
	}

	std::vector<PerimeterPos> perimeter_vector = heap_pool_[current_pool_count_++];

	int k = 0;
	for (auto itr = perimeter_list.begin(); itr != perimeter_list.end(); ++itr) {
		auto perimeter_grid_position = *itr;
		float grid_distance = glm::length(glm::vec3(perimeter_grid_position - robot_grid_position));

		if (range_min <= grid_distance && grid_distance < range_max) {
			perimeter_vector.push_back(PerimeterPos(grid_distance, perimeter_grid_position));
		}
	}

	std::stable_sort(perimeter_vector.begin(), perimeter_vector.end());
	glm::vec3 a = (robot_grid_position);
	bool perimeter_found = false;
	
	for (auto j = 0; j < perimeter_vector.size(); ++j) {
		glm::vec3 b = perimeter_vector[j].grid_position_;
		bool interior_found = going_through_interior_test(a, b);

		if (!interior_found) {
			explore_position = glm::ivec3(b);
			return true;
		}
	}
	
	return false;
}

bool SwarmOccupancyTree::find_closest_position_from_list_visibility_non_aware(const std::set<glm::ivec3, IVec3Comparator>& perimeter_list,
	const glm::ivec3& robot_grid_position,
	glm::ivec3& explore_position, float range_min, float range_max) {

	// create a pool, to stop micro memory allocations all the time
	//if (current_pool_count_ == pool_size_) {
	//	current_pool_count_ = 0;
	//	std::vector<PerimeterPos> sample_vector;
	//	sample_vector.reserve(perimeter_list.size());
	//	//heap_pool_.clear();
	//	//heap_pool_.resize(pool_size_);
	//	std::fill(heap_pool_.begin(), heap_pool_.end(), sample_vector);
	//}

	//std::vector<PerimeterPos> perimeter_vector = heap_pool_[current_pool_count_++];

	int valid_perimeter_locs = 0;
	for (auto itr = perimeter_list.begin(); itr != perimeter_list.end(); ++itr) {
		auto perimeter_grid_position = *itr;
		float grid_distance = glm::length(glm::vec3(perimeter_grid_position - robot_grid_position));

		if (range_min <= grid_distance && grid_distance < range_max) {
			perimeter_vector_[valid_perimeter_locs++] = (PerimeterPos(grid_distance, perimeter_grid_position));
		}
	}

	if (valid_perimeter_locs == 0) {
		return false;
	}

	std::stable_sort(perimeter_vector_.begin(), perimeter_vector_.begin() + valid_perimeter_locs);

	explore_position = perimeter_vector_[0].grid_position_;

	// try to get a position that is one step away from the wall, otherwise robot will always bounce, 
	// due to obstacle avoidance

	// check perimeter
	for (int nx = -1; nx <= 1; ++nx) {
		for (int ny = -1; ny <= 1; ++ny) {
			if (!(nx == 0 && ny == 0)) {
				//char loc;
				int next_x = explore_position.x + nx;
				int next_y = explore_position.z + ny;

				glm::ivec3 next_pos(next_x, 0, next_y);


				if (is_out_of_bounds(next_pos)) {
					continue;
				}

				int loc = at(next_x, next_y);
				if (!is_perimeter(next_pos) && !is_interior(next_pos)) {
					explore_position = next_pos;
					return true;
				}
			}
		}
	}

	return true;
}

bool SwarmOccupancyTree::find_closest_2_positions_from_list(const std::set<glm::ivec3, IVec3Comparator>& perimeter_list,
	const glm::ivec3& robot_grid_position,
	std::vector<glm::ivec3>& explore_positions, float range_min, float range_max, bool enable_interior_test) {

	// create a pool, to stop micro memory allocations all the time
	if (current_pool_count_ == pool_size_) {
		current_pool_count_ = 0;
		std::vector<PerimeterPos> sample_vector;
		sample_vector.reserve(perimeter_list.size());
		//heap_pool_.clear();
		//heap_pool_.resize(pool_size_);
		std::fill(heap_pool_.begin(), heap_pool_.end(), sample_vector);
	}


	std::vector<PerimeterPos> perimeter_vector = heap_pool_[current_pool_count_++];

	int k = 0;
	for (auto itr = perimeter_list.begin(); itr != perimeter_list.end(); ++itr) {
		auto perimeter_grid_position = *itr;
		float grid_distance = glm::length(glm::vec3(perimeter_grid_position - robot_grid_position));

		if (range_min <= grid_distance && grid_distance < range_max) {
			perimeter_vector.push_back(PerimeterPos(grid_distance, perimeter_grid_position));
		}
	}

	std::stable_sort(perimeter_vector.begin(), perimeter_vector.end());
	glm::vec3 a = (robot_grid_position);
	bool perimeter_found = false;
	
	int explore_positions_count = 0;

	for (auto j = 0; j < perimeter_vector.size(); ++j) {
		glm::vec3 b = perimeter_vector[j].grid_position_;
		bool interior_found = false;

		if (enable_interior_test) {
			interior_found = going_through_interior_test(a, b);
		}

		if (!interior_found) {
			explore_positions.push_back(glm::ivec3(b));
			explore_positions_count++;

			if (explore_positions_count == 2) {
				return true;
			}
		}
	}
	if (explore_positions_count > 0) {
		return true;
	}
	
	return false;
}

bool SwarmOccupancyTree::next_cell_to_explore(const glm::ivec3& robot_grid_position,
	glm::ivec3& explore_position) {
	return find_closest_position_from_list(empty_space_list_, robot_grid_position, explore_position);

}

bool SwarmOccupancyTree::next_cell_to_explore(const glm::ivec3& robot_grid_position,
	glm::ivec3& explore_position, float range_min, float range_max) {
	return find_closest_position_from_list(explore_perimeter_list_, robot_grid_position, explore_position, range_min, range_max);
}

bool SwarmOccupancyTree::next_cell_to_explore_visibility_non_aware(const glm::ivec3& robot_grid_position,
	glm::ivec3& explore_position, float range_min, float range_max) {
	return find_closest_position_from_list_visibility_non_aware(explore_perimeter_list_, robot_grid_position, explore_position, range_min, range_max);
}

bool SwarmOccupancyTree::closest_perimeter(const glm::ivec3& robot_grid_position,
	glm::ivec3& perimeter_position, float range_min, float range_max) {
	return find_closest_position_from_list(static_perimeter_list_, robot_grid_position, perimeter_position, range_min, range_max);
}

bool SwarmOccupancyTree::closest_2_interior_positions(const glm::ivec3& robot_grid_position,
	std::vector<glm::ivec3>& perimeter_position, float range_min, float range_max) {
	return find_closest_2_positions_from_list(interior_list_, robot_grid_position, perimeter_position, range_min, range_max, false);
}
