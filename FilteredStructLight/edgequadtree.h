#pragma once
#include <type_traits>

class EdgeQuadTree
{
	enum QUADRANT {
		NW = 0,
		NE = 1,
		SE = 2,
		SW = 3
	};
		struct QuadRect {
			int x1_, x2_, y1_, y2_;
			int x_avg_, y_avg_;
			inline int get_x_avg() const { return ((x1_ + x2_) / 2); };
			inline int get_y_avg() const { return ((y1_ + y2_) / 2); };
		};
		struct QuadNode {
			QuadRect rect_;
			QuadNode* children[4];
			bool is_interior;
			bool is_leaf_node;
		};
		unsigned int grid_height_;
		unsigned int grid_width_;
		float grid_square_length_;
		unsigned resolution_per_side_;
		unsigned resolution_;

public:
	EdgeQuadTree(unsigned width, unsigned height, float grid_square_length);
	~EdgeQuadTree();
	QuadNode* root_;
	bool insert_value(QuadNode* node, unsigned int x, unsigned int y, bool is_interior);
	bool delete_value(QuadNode* node, unsigned int x, unsigned int y);
	bool create_nodes(QuadNode* node, int current_level, int resolution);
	bool destroy_nodes(QuadNode* node);
	bool GetQuadrant(QuadNode* node, unsigned x, unsigned y, EdgeQuadTree::QUADRANT& quadrant) const;
	bool IsLeafNode(QuadNode* node) const;
	bool find_node(QuadNode* node, unsigned int x, unsigned int y, QuadNode*& result_node) const;
	void print_error(const char* error_msg) const;
	void init_root(QuadNode*& root);
};

