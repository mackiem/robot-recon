#pragma once
#include <stlastar.h>
#include <deque>
#include "swarmtree.h"

// Definitions

typedef mm::Quadtree<int> Grid;

class MapSearchNode
{
public:
	int x;	 // the (x,y) positions of the node
	int y;
	Grid* grid_;
	

	MapSearchNode(): grid_(nullptr) { x = y = 0; } ;
	//MapSearchNode(int px, int py) { x = px; y = py; };

	MapSearchNode(Grid* grid) { x = y = 0; grid_ = grid; }
	MapSearchNode(Grid* grid, int px, int py) { x = px; y = py; grid_ = grid;}

	float GoalDistanceEstimate(MapSearchNode &nodeGoal);
	bool IsGoal(MapSearchNode &nodeGoal);
	bool GetSuccessors(AStarSearch<MapSearchNode> *astarsearch, MapSearchNode *parent_node);
	float GetCost(MapSearchNode &successor);
	bool IsSameState(MapSearchNode &rhs);

	void PrintNodeInfo();


};
class AStar
{
	Grid* grid_;
public:
	AStarSearch<MapSearchNode>* astarsearch_;
	std::deque<glm::ivec3> search(glm::vec3& source, glm::vec3& target);
	explicit AStar(Grid* grid);
	~AStar();
};

