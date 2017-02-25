#pragma once
#include <stlastar.h>
#include <deque>
#include "swarmtree.h"
#include "robot.h"

// Definitions

class ExperimentalRobot;
typedef mm::Quadtree<int> Grid;

class MapSearchNode
{
public:
	int x;	 // the (x,y) positions of the node
	int y;
	Grid* grid_;
	std::vector<int>* adjacent_robots_;
	int current_no_of_robots_;
	std::vector<Robot*>* robots_;
	ExperimentalRobot* robot_;
	

	MapSearchNode(): grid_(nullptr), current_no_of_robots_(0), robots_(nullptr), adjacent_robots_(nullptr) { x = y = 0; } ;
	//MapSearchNode(int px, int py) { x = px; y = py; };

	MapSearchNode(Grid* grid): adjacent_robots_(nullptr), current_no_of_robots_(0), robots_(nullptr) {
		x = y = 0;
		grid_ = grid;
	}

	MapSearchNode(Grid* grid, int px, int py) : adjacent_robots_(nullptr), current_no_of_robots_(0), robots_(nullptr) {
		x = px;
		y = py;
		grid_ = grid;
	}

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
	ExperimentalRobot* robot_;
	AStarSearch<MapSearchNode>* astarsearch_;

	void search(glm::vec3& source, glm::vec3& target, std::vector<Robot*>* robots,
		std::vector<int>* adjacent_robots, int current_no_of_robots, ExperimentalRobot* robot, std::vector<glm::ivec3>&path,
		int& no_of_steps) const;

	explicit AStar(Grid* grid);
	~AStar();
};

