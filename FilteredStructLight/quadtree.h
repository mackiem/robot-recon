#pragma once
#include <stdexcept>

namespace mm {
	template <class T> class Quadtree {
	public:
		bool set(unsigned int x, unsigned int y, T& object);
		T at(unsigned int x, unsigned int y) const;
		bool unset(unsigned int x, unsigned int y);
		void set_empty_value(T empty_object);
		Quadtree<T>(unsigned int resolution, T empty_value);
		virtual ~Quadtree<T>();
		int get_resolution_per_side() const;

	protected:
		unsigned int resolution_;
		T empty_value_;
		int resolution_per_side_;

		struct QuadRect {
			int x1_, x2_, y1_, y2_;
			int x_avg_, y_avg_;
			inline int get_x_avg() const { return ((x1_ + x2_) / 2); };
			inline int get_y_avg() const { return ((y1_ + y2_) / 2); };
		};
		struct QuadNode {
			QuadRect rect_;
			QuadNode* children[4];
			T object_;
			enum QUADRANT {
				NW = 0,
				NE = 1,
				SE = 2,
				SW = 3
			};
		};
		QuadNode* root_;
		bool insert_value(QuadNode* node, unsigned int x, unsigned int y, T& object);
		bool delete_value(QuadNode* node, unsigned int x, unsigned int y);
		bool create_nodes(QuadNode* node, int current_level, int resolution);
		bool destroy_nodes(QuadNode* node);
		bool find_node(QuadNode* node, unsigned int x, unsigned int y, QuadNode*& result_node) const;
		void print_error(const char* error_msg) const;
		void init_root(QuadNode*& root);
	};
};

template <class T>
bool mm::Quadtree<T>::set(unsigned x, unsigned y, T& object) {
	return insert_value(root_, x, y, object);
}

template <class T>
T mm::Quadtree<T>::at(unsigned x, unsigned y) const {
	QuadNode* result_node;
	bool success = find_node(root_, x, y, result_node);
	if (success) {
		T result_value = result_node->object_;
		return result_value;
	}
	//const char* error_msg = "Error state - no node found in location";
	//print_error(error_msg);
	return empty_value_;
}

template <class T>
bool mm::Quadtree<T>::unset(unsigned x, unsigned y) {
	QuadNode* result_node;
	bool success = find_node(root_, x, y, result_node);
	if (success) {
		result_node->object_ = empty_value_;
		return true;
	}
	return false;
}

template <class T>
void mm::Quadtree<T>::set_empty_value(T empty_object) {
	empty_value_ = empty_object;
}

template <class T>
mm::Quadtree<T>::Quadtree(unsigned resolution, T empty_value) : resolution_(resolution), empty_value_(empty_value) {
	if (resolution < 1) {
		const char* error_msg = "Resolution cannot be zero or negative";
		print_error(error_msg);
		throw std::runtime_error(error_msg);
	}
	resolution_per_side_ = std::pow(std::pow(4, resolution), 0.5);
	init_root(root_);
	bool success = create_nodes(root_, 0, resolution_);
	if (!success) {
		const char* error_msg = "Error occurred during quad tree initialization...";
		print_error(error_msg);
		throw std::runtime_error("Error occurred during quad tree initialization...");
	}

}

template <class T>
mm::Quadtree<T>::~Quadtree() {
	destroy_nodes(root_);
	delete root_;
}

template <class T>
int mm::Quadtree<T>::get_resolution_per_side() const {
	return resolution_per_side_;
}

template <class T>
bool mm::Quadtree<T>::insert_value(QuadNode* node, unsigned x, unsigned y, T& object) {
	QuadNode* result_node;
	bool success =	find_node(root_, x, y, result_node);
	if (success) {
		result_node->object_ = object;
		return true;
	}
	return false;
}

template <class T>
bool mm::Quadtree<T>::delete_value(QuadNode* node, unsigned x, unsigned y) {
	QuadNode* result_node;
	bool success =	find_node(root_, x, y, result_node);
	if (success) {
		result_node->object_ = nullptr;
		return true;
	}
	return false;
}

template <class T>
bool mm::Quadtree<T>::create_nodes(QuadNode* node, int current_level, int resolution) {

	// leaf node case
	if (current_level == resolution) {
		for (char i = 0; i < 4; ++i) {
			node->children[i] = nullptr;
		}
		node->object_ = empty_value_;
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
		node->children[QuadNode::NW] = nw_node;

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
		node->children[QuadNode::NE] = ne_node;

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
		node->children[QuadNode::SE] = se_node;

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
		node->children[QuadNode::SW] = sw_node;

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

template <class T>
bool mm::Quadtree<T>::destroy_nodes(QuadNode* node) {
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

template <class T>
bool mm::Quadtree<T>::find_node(QuadNode* node, unsigned x, unsigned y, QuadNode*& result_node) const {
	// base case
	if (node->children[QuadNode::NE] == nullptr) {
		// this is the leaf node
		result_node = node;
		return true;
	} else {
		if (x < node->rect_.x1_ || x > node->rect_.x2_
			|| y < node->rect_.y1_ || y > node->rect_.y2_) {
			// something wrong
			return false;
		} else {
			if (node->rect_.x1_ <= x && x < node->rect_.x_avg_) {
				// west half, equal to handle the edge case of being on the edge
				if (node->rect_.y1_ <= y && y < node->rect_.y_avg_) {
					// NW quadrant, equal to handle the edge case of being on the edge
					return find_node(node->children[QuadNode::NW], x, y, result_node);
					
				} else if (node->rect_.y_avg_ <= y && y <= node->rect_.y2_) {
					// SW quadrant
					return find_node(node->children[QuadNode::SW], x, y, result_node);
					
				} else {
					// undefined situation
					return false;
				}
			} else if (node->rect_.x_avg_ <= x  && x  <= node->rect_.x2_) {
				// east half
				if (node->rect_.y1_ <= y && y < node->rect_.y_avg_) {
					// NE quadrant, equal to handle the edge case of being on the edge
					return find_node(node->children[QuadNode::NE], x, y, result_node);
					
				} else if (node->rect_.y_avg_ <= y && y <= node->rect_.y2_) {
					// SE quadrant
					return find_node(node->children[QuadNode::SE], x, y, result_node);
					
				} else {
					// undefined situation
					return false;
				}
			} else {
				// something wrong happened
				return false;
			}
		}
	}

}

template <class T>
void mm::Quadtree<T>::print_error(const char* error_msg) const {
#ifdef DEBUG
		std::cout << error_msg << std::endl;
#endif
}

template <class T>
void mm::Quadtree<T>::init_root(QuadNode*& root) {
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
