#include "edgequadtree.h"
#include <stdexcept>
#include <cmath>
#include <iostream>




EdgeQuadTree::~EdgeQuadTree()
{
}


EdgeQuadTree::EdgeQuadTree(unsigned width, unsigned height, float grid_square_length) : grid_width_(width), 
							grid_height_(height), grid_square_length_(grid_square_length) {

	if ((width < 1) 
		|| (height < 1)) {
		const char* error_msg = "Resolution cannot be zero or negative";
		print_error(error_msg);
		throw std::runtime_error(error_msg);
	}

	//create_grid(width, height);



	resolution_per_side_ = std::max(grid_width_, grid_height_);
	resolution_ = std::log(std::pow(resolution_per_side_, 2));
	//resolution_per_side_ = std::pow(std::pow(4, resolution), 0.5);
	init_root(root_);
	bool success = create_nodes(root_, 0, resolution_);
	if (!success) {
		const char* error_msg = "Error occurred during quad tree initialization...";
		print_error(error_msg);
		throw std::runtime_error("Error occurred during quad tree initialization...");
	}

}


bool EdgeQuadTree::insert_value(QuadNode* node, unsigned x, unsigned y, bool is_interior) {
	QuadNode* result_node;
	bool success =	find_node(root_, x, y, result_node);
	if (success) {
		//result_node->object_ = object;
		return true;
	}
	return false;
}


bool EdgeQuadTree::delete_value(QuadNode* node, unsigned x, unsigned y) {
	QuadNode* result_node;
	bool success =	find_node(root_, x, y, result_node);
	if (success) {
		//result_node->object_ = nullptr;
		return true;
	}
	return false;
}


bool EdgeQuadTree::create_nodes(QuadNode* node, int current_level, int resolution) {

	// leaf node case
	if (current_level == resolution) {
		for (char i = 0; i < 4; ++i) {
			node->children[i] = nullptr;
		}
		//node->object_ = empty_value_;
		return true;
	} else {
		// make sure there are no leaks
		for (char i = 0; i < 4; ++i) {
			//if (node->children[i]) {
			//	delete node->children[i];
			//}
			node->children[i] = nullptr;
		}

		QuadRect rect = node->rect_;
		// create NW
		QuadNode* nw_node = new QuadNode();
		QuadRect nw_rect;
		nw_rect.x1_ = rect.x1_;
		nw_rect.x2_ = rect.x_avg_;
		nw_rect.y1_ = rect.y1_;
		nw_rect.y2_ = rect.y_avg_;
		nw_rect.x_avg_ = nw_rect.get_x_avg();
		nw_rect.y_avg_ = nw_rect.get_y_avg();
		nw_node->rect_ = nw_rect;
		node->children[NW] = nw_node;

		// create NE
		QuadNode* ne_node = new QuadNode();
		QuadRect ne_rect;
		ne_rect.x1_ = rect.x_avg_;
		ne_rect.x2_ = rect.x2_;
		ne_rect.y1_ = rect.y1_;
		ne_rect.y2_ = rect.y_avg_;
		ne_rect.x_avg_ = ne_rect.get_x_avg();
		ne_rect.y_avg_ = ne_rect.get_y_avg();
		ne_node->rect_ = ne_rect;
		node->children[NE] = ne_node;

		// create SE
		QuadNode* se_node = new QuadNode();
		QuadRect se_rect;
		se_rect.x1_ = rect.x_avg_;
		se_rect.x2_ = rect.x2_;
		se_rect.y1_ = rect.y_avg_;
		se_rect.y2_ = rect.y2_;	
		se_rect.x_avg_ = se_rect.get_x_avg();
		se_rect.y_avg_= se_rect.get_y_avg();
		se_node->rect_ = se_rect;
		node->children[SE] = se_node;

		// create SW
		QuadNode* sw_node = new QuadNode();
		QuadRect sw_rect;
		sw_rect.x1_ = rect.x1_;
		sw_rect.x2_ = rect.x_avg_;
		sw_rect.y1_ = rect.y_avg_;
		sw_rect.y2_ = rect.y2_;
		sw_rect.x_avg_ = sw_rect.get_x_avg();
		sw_rect.y_avg_ = sw_rect.get_y_avg();
		sw_node->rect_ = sw_rect;
		node->children[SW] = sw_node;

		bool success = true;
		++current_level;
		for (int i = 0; i < 4; ++i) {
			success = create_nodes(node->children[i], current_level, resolution);
			if (!success) {
				return false;
			}
		}

		return true;
	}
}


bool EdgeQuadTree::destroy_nodes(QuadNode* node) {
	for (char i = 0; i < 4; ++i) {
		if (node->children[i] == nullptr) {
			// if even one is null, it's a leaf
			break;
		} else {
			destroy_nodes(node->children[i]);
		}
	}

	for (char i = 0; i < 4; ++i) {
		delete node->children[i];
		node->children[i] = nullptr;
	}
	return true;
}

bool EdgeQuadTree::GetQuadrant(QuadNode* node, unsigned x, unsigned y, EdgeQuadTree::QUADRANT& quadrant) const {

			if (node->rect_.x1_ <= x && x < node->rect_.x_avg_) {
				// west half, equal to handle the edge case of being on the edge
				if (node->rect_.y1_ <= y && y < node->rect_.y_avg_) {
					// NW quadrant, equal to handle the edge case of being on the edge
					quadrant = NW;
					return true;
				} else if (node->rect_.y_avg_ <= y && y <= node->rect_.y2_) {
					// SW quadrant
					quadrant = SW;
					return true;
				} else {
					// undefined situation
					return false;
				}
			} else if (node->rect_.x_avg_ <= x  && x  <= node->rect_.x2_) {
				// east half
				if (node->rect_.y1_ <= y && y < node->rect_.y_avg_) {
					// NE quadrant, equal to handle the edge case of being on the edge
					quadrant = NE;
					return true;
				} else if (node->rect_.y_avg_ <= y && y <= node->rect_.y2_) {
					// SE quadrant
					quadrant = SE;
					return true;
				} else {
					// undefined situation
					return false;
				}
			} else {
				// something wrong happened
				// out of bounds
				return false;
			}
			return false;
	
}

bool EdgeQuadTree::IsLeafNode(QuadNode* node) const {
	return (node->children[NE] == nullptr);
}

bool EdgeQuadTree::find_node(QuadNode* node, unsigned x, unsigned y, QuadNode*& result_node) const {
	// base case
	if (IsLeafNode(node)) {
		// this is the leaf node
		result_node = node;
		return true;
	} else {
		QUADRANT quadrant;
		bool quadrant_exists = GetQuadrant(node, x, y, quadrant);

		if (!quadrant_exists) return false;

		return find_node(node->children[quadrant], x, y, result_node);
	}

}

//bool EdgeQuadTree::find_node(QuadNode* node, unsigned x, unsigned y, QuadNode*& result_node) const {
//	// base case
//	if (node->children[QuadNode::NE] == nullptr) {
//		// this is the leaf node
//		result_node = node;
//		return true;
//	} else {
//		if (x < node->rect_.x1_ || x > node->rect_.x2_
//			|| y < node->rect_.y1_ || y > node->rect_.y2_) {
//			// something wrong
//			return false;
//		} else {
//			if (node->rect_.x1_ <= x && x < node->rect_.x_avg_) {
//				// west half, equal to handle the edge case of being on the edge
//				if (node->rect_.y1_ <= y && y < node->rect_.y_avg_) {
//					// NW quadrant, equal to handle the edge case of being on the edge
//					return find_node(node->children[QuadNode::NW], x, y, result_node);
//					
//				} else if (node->rect_.y_avg_ <= y && y <= node->rect_.y2_) {
//					// SW quadrant
//					return find_node(node->children[QuadNode::SW], x, y, result_node);
//					
//				} else {
//					// undefined situation
//					return false;
//				}
//			} else if (node->rect_.x_avg_ <= x  && x  <= node->rect_.x2_) {
//				// east half
//				if (node->rect_.y1_ <= y && y < node->rect_.y_avg_) {
//					// NE quadrant, equal to handle the edge case of being on the edge
//					return find_node(node->children[QuadNode::NE], x, y, result_node);
//					
//				} else if (node->rect_.y_avg_ <= y && y <= node->rect_.y2_) {
//					// SE quadrant
//					return find_node(node->children[QuadNode::SE], x, y, result_node);
//					
//				} else {
//					// undefined situation
//					return false;
//				}
//			} else {
//				// something wrong happened
//				return false;
//			}
//		}
//	}
//
//}


void EdgeQuadTree::print_error(const char* error_msg) const {
#ifdef DEBUG
		std::cout << error_msg << std::endl;
#endif
}


void EdgeQuadTree::init_root(QuadNode*& root) {
	root_ = new QuadNode();
	QuadRect rect;
	rect.x1_ = 0;
	rect.x2_ = resolution_per_side_;
	rect.y1_ = 0;
	rect.y2_ = resolution_per_side_;
	rect.x_avg_ = rect.get_x_avg();
	rect.y_avg_ = rect.get_y_avg();
	root_->rect_ = rect;
	for (char i = 0; i < 4; ++i) {
		root_->children[i] = nullptr;
	}
}
