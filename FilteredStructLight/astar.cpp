#include "astar.h"
#include "swarmtree.h"




#include "stlastar.h" // See header for copyright and usage information

#include <iostream>
#include <stdio.h>
#include <math.h>
#include "swarmutils.h"

#define DEBUG_LISTS 0
#define DEBUG_LIST_LENGTHS_ONLY 0

using namespace std;

// Global data

// The world map

//const int MAP_WIDTH = 20;
//const int MAP_HEIGHT = 20;
//
//int world_map[MAP_WIDTH * MAP_HEIGHT] =
//{
//
//	// 0001020304050607080910111213141516171819
//	1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,   // 00
//	1, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 1,   // 01
//	1, 9, 9, 1, 1, 9, 9, 9, 1, 9, 1, 9, 1, 9, 1, 9, 9, 9, 1, 1,   // 02
//	1, 9, 9, 1, 1, 9, 9, 9, 1, 9, 1, 9, 1, 9, 1, 9, 9, 9, 1, 1,   // 03
//	1, 9, 1, 1, 1, 1, 9, 9, 1, 9, 1, 9, 1, 1, 1, 1, 9, 9, 1, 1,   // 04
//	1, 9, 1, 1, 9, 1, 1, 1, 1, 9, 1, 1, 1, 1, 9, 1, 1, 1, 1, 1,   // 05
//	1, 9, 9, 9, 9, 1, 1, 1, 1, 1, 1, 9, 9, 9, 9, 1, 1, 1, 1, 1,   // 06
//	1, 9, 9, 9, 9, 9, 9, 9, 9, 1, 1, 1, 9, 9, 9, 9, 9, 9, 9, 1,   // 07
//	1, 9, 1, 1, 1, 1, 1, 1, 1, 1, 1, 9, 1, 1, 1, 1, 1, 1, 1, 1,   // 08
//	1, 9, 1, 9, 9, 9, 9, 9, 9, 9, 1, 1, 9, 9, 9, 9, 9, 9, 9, 1,   // 09
//	1, 9, 1, 1, 1, 1, 9, 1, 1, 9, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,   // 10
//	1, 9, 9, 9, 9, 9, 1, 9, 1, 9, 1, 9, 9, 9, 9, 9, 1, 1, 1, 1,   // 11
//	1, 9, 1, 9, 1, 9, 9, 9, 1, 9, 1, 9, 1, 9, 1, 9, 9, 9, 1, 1,   // 12
//	1, 9, 1, 9, 1, 9, 9, 9, 1, 9, 1, 9, 1, 9, 1, 9, 9, 9, 1, 1,   // 13
//	1, 9, 1, 1, 1, 1, 9, 9, 1, 9, 1, 9, 1, 1, 1, 1, 9, 9, 1, 1,   // 14
//	1, 9, 1, 1, 9, 1, 1, 1, 1, 9, 1, 1, 1, 1, 9, 1, 1, 1, 1, 1,   // 15
//	1, 9, 9, 9, 9, 1, 1, 1, 1, 1, 1, 9, 9, 9, 9, 1, 1, 1, 1, 1,   // 16
//	1, 1, 9, 9, 9, 9, 9, 9, 9, 1, 1, 1, 9, 9, 9, 1, 9, 9, 9, 9,   // 17
//	1, 9, 1, 1, 1, 1, 1, 1, 1, 1, 1, 9, 1, 1, 1, 1, 1, 1, 1, 1,   // 18
//	1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,   // 19
//
//};

AStar::~AStar()
{
	if (astarsearch_) {
		delete astarsearch_;
	}
}

// map helper functions

//int GetMap(int x, int y)
//{
//	//if (x < 0 ||
//	//	x >= MAP_WIDTH ||
//	//	y < 0 ||
//	//	y >= MAP_HEIGHT
//	//	)
//	//{
//	//	return 9;
//	//}
//
//	//return world_map[(y*MAP_WIDTH) + x];
//	if (grid_)
//}




bool MapSearchNode::IsSameState(MapSearchNode &rhs)
{

	// same state in a maze search is simply when (x,y) are the same
	if ((x == rhs.x) &&
		(y == rhs.y))
	{
		return true;
	}
	else
	{
		return false;
	}

}

void MapSearchNode::PrintNodeInfo()
{
	char str[100];
	sprintf(str, "Node position : (%d,%d)\n", x, y);

	std::cout << str;
}

// Here's the heuristic function that estimates the distance from a Node
// to the Goal. 

float MapSearchNode::GoalDistanceEstimate(MapSearchNode &nodeGoal)
{
	return fabsf(x - nodeGoal.x) + fabsf(y - nodeGoal.y);
}

bool MapSearchNode::IsGoal(MapSearchNode &nodeGoal)
{

	if ((x == nodeGoal.x) &&
		(y == nodeGoal.y))
	{
		return true;
	}

	return false;
}

// This generates the successors to the given Node. It uses a helper function called
// AddSuccessor to give the successors to the AStar class. The A* specific initialisation
// is done for each node internally, so here you just set the state information that
// is specific to the application
bool MapSearchNode::GetSuccessors(AStarSearch<MapSearchNode> *astarsearch, MapSearchNode *parent_node)
{

	int parent_x = -1;
	int parent_y = -1;

	if (parent_node)
	{
		parent_x = parent_node->x;
		parent_y = parent_node->y;
		//parent_node->PrintNodeInfo();
	}


	MapSearchNode NewNode(grid_);

	// push each possible move except allowing the search to go backwards

	//char loc;
	//grid_->get(x - 1, y, loc);
	//if ((loc < CityGrid::BUILDING)
	//	&& !((parent_x == x - 1) && (parent_y == y))
	//	)
	//{
	//	NewNode = MapSearchNode(grid_, x - 1, y);
	//	astarsearch->AddSuccessor(NewNode);
	//}

	//grid_->get(x, y - 1, loc);
	//if ((loc < CityGrid::BUILDING)
	//	&& !((parent_x == x) && (parent_y == y - 1))
	//	)
	//{
	//	NewNode = MapSearchNode(grid_, x, y - 1);
	//	astarsearch->AddSuccessor(NewNode);
	//}

	//grid_->get(x + 1, y, loc);
	//if ((loc < CityGrid::BUILDING)
	//	&& !((parent_x == x + 1) && (parent_y == y))
	//	)
	//{
	//	NewNode = MapSearchNode(grid_, x + 1, y);
	//	astarsearch->AddSuccessor(NewNode);
	//}


	//grid_->get(x, y + 1, loc);
	//if ((loc < CityGrid::BUILDING)
	//	&& !((parent_x == x) && (parent_y == y + 1))
	//	)
	//{
	//	NewNode = MapSearchNode(grid_, x, y + 1);
	//	astarsearch->AddSuccessor(NewNode);
	//}

	// add diagonal cases
	// (x+1, y+1), (x-1, y+1), (x+1, y-1), (x-1, y-1)

	for (int nx = -1; nx <= 1; ++nx) {
		for (int ny = -1; ny <= 1; ++ny) {
			if (!(nx == 0 && ny == 0)) {
				//char loc;
				int next_x = x + nx;
				int next_y = y + ny;

				if (grid_->is_out_of_bounds(next_x, next_y)) {
					continue;
				}

				int loc = grid_->at(next_x, next_y);
				if ((loc < SwarmOccupancyTree::INTERIOR_MARK)
					&& !((parent_x == next_x) && (parent_y == next_y))
					)
				{
						NewNode = MapSearchNode(grid_, next_x, next_y);
						astarsearch->AddSuccessor(NewNode);
					//if (nx == 0 || ny == 0) {
					//	NewNode = MapSearchNode(grid_, next_x, next_y);
					//	astarsearch->AddSuccessor(NewNode);
					//} else {
					//	// if diagonal we have to do additional checks, as check the locations next to it is occupied
					//	int next_to_loc[2];
					//	//grid_->get(x, next_y, next_to_loc[0]);
					//	//grid_->get(next_x, y, next_to_loc[1]);

					//	if (grid_->is_out_of_bounds(x, next_y)
					//		|| grid_->is_out_of_bounds(next_x, y)) {
					//		continue;
					//	}

					//	next_to_loc[0] = grid_->at(x, next_y);
					//	next_to_loc[1] = grid_->at(next_x, y);

					//	if (next_to_loc[0] < SwarmOccupancyTree::INTERIOR_MARK
					//		&& next_to_loc[1] < SwarmOccupancyTree::INTERIOR_MARK) {
					//		NewNode = MapSearchNode(grid_, next_x, next_y);
					//		astarsearch->AddSuccessor(NewNode);
					//	}
					//}
				}
			}

		}
	}


	return true;
}

// given this node, what does it cost to move to successor. In the case
// of our map the answer is the map terrain value at this node since that is 
// conceptually where we're moving

float MapSearchNode::GetCost(MapSearchNode &successor)
{
	if (grid_->is_out_of_bounds(x, y)) {
		return SwarmOccupancyTree::INTERIOR_MARK;
	}

	// check perimeter
	for (int nx = -1; nx <= 1; ++nx) {
		for (int ny = -1; ny <= 1; ++ny) {
			if (!(nx == 0 && ny == 0)) {
				//char loc;
				int next_x = x + nx;
				int next_y = y + ny;

				if (grid_->is_out_of_bounds(next_x, next_y)) {
					continue;
				}
				int loc = grid_->at(next_x, next_y);
				if (loc == SwarmOccupancyTree::INTERIOR_MARK) {
					return SwarmOccupancyTree::PERIMETER;
				}
			}
		}
	}

	int loc = grid_->at(x, y);
	if (loc == SwarmOccupancyTree::INTERIOR_MARK) {
		return loc;
	}

	// check for diagonal and return perimeter
	if (std::abs(successor.x - x) + std::abs(successor.y - y) == 2) {
		return SwarmOccupancyTree::PERIMETER;
	}

	return (float)loc;
}


// Main

#define DISPLAY_SOLUTION 0

std::deque<glm::ivec3> AStar::search(glm::vec3& source, glm::vec3& target)
{

	//std::cout << "STL A* Search implementation\n(C)2001 Justin Heyes-Jones\n";

	// Our sample problem defines the world as a 2d array representing a terrain
	// Each element contains an integer from 0 to 5 which indicates the cost 
	// of travel across the terrain. Zero means the least possible difficulty 
	// in travelling (think ice rink if you can skate) whilst 5 represents the 
	// most difficult. 9 indicates that we cannot pass.

	// Create an instance of the search class...
	std::deque< glm::ivec3 > path;

	//AStarSearch<MapSearchNode> astarsearch_;
	//AStarSearch<MapSearchNode> astarsearch(10000);

	unsigned int SearchCount = 0;

	const unsigned int NumSearches = 1;

	while (SearchCount < NumSearches)
	{

		// Create a start state
		MapSearchNode nodeStart(grid_);
		//grid_->convert_coords(source.x, source.z, nodeStart.x, nodeStart.y);
		grid_->map_to_grid(source.x, source.z, nodeStart.x, nodeStart.y);
		//nodeStart.x = source.x;
		//nodeStart.y = source.z;

		// Define the goal state
		MapSearchNode nodeEnd(grid_);
		grid_->map_to_grid(target.x, target.z, nodeEnd.x, nodeEnd.y);
		//nodeEnd.x = target.x;
		//nodeEnd.y = target.z;

		// Set Start and goal states

		AStarSearch<MapSearchNode> astarsearch(10000);
		astarsearch.SetStartAndGoalStates(nodeStart, nodeEnd);

		unsigned int SearchState;
		unsigned int SearchSteps = 0;

		do
		{
			SearchState = astarsearch.SearchStep();

			SearchSteps++;

#if DEBUG_LISTS

			cout << "Steps:" << SearchSteps << "\n";

			int len = 0;

			cout << "Open:\n";
			MapSearchNode *p = astarsearch.GetOpenListStart();
			while (p)
			{
				len++;
#if !DEBUG_LIST_LENGTHS_ONLY			
				((MapSearchNode *)p)->PrintNodeInfo();
#endif
				p = astarsearch.GetOpenListNext();

			}

			cout << "Open list has " << len << " nodes\n";

			len = 0;

			cout << "Closed:\n";
			p = astarsearch.GetClosedListStart();
			while (p)
			{
				len++;
#if !DEBUG_LIST_LENGTHS_ONLY			
				p->PrintNodeInfo();
#endif			
				p = astarsearch.GetClosedListNext();
			}

			cout << "Closed list has " << len << " nodes\n";
#endif

		} while (SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SEARCHING);

		if (SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SUCCEEDED)
		{
			//std::cout << "Search found goal state\n";

			MapSearchNode *node = astarsearch.GetSolutionStart();

#if DISPLAY_SOLUTION
			cout << "Displaying solution\n";
			node->PrintNodeInfo();
#endif

			int steps = 0;
			for (;;)
			{
				node = astarsearch.GetSolutionNext();

				if (!node)
				{
					break;
				}

				path.push_back(glm::vec3(node->x, 0, node->y));
#if DISPLAY_SOLUTION
				node->PrintNodeInfo();
#endif
				steps++;

			};

			//std::cout << "Solution steps " << steps << endl;

			// Once you're done with the solution you can free the nodes up
			astarsearch.FreeSolutionNodes();


		}
		else if (SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_FAILED)
		{
			SwarmUtils::print_vector("source", glm::ivec3(nodeStart.x, 0, nodeStart.y));
			SwarmUtils::print_vector("target", glm::ivec3(nodeEnd.x, 0, nodeEnd.y));
			std::cout << "source :  " << ((grid_->at(nodeStart.x, nodeStart.y) == SwarmOccupancyTree::INTERIOR_MARK) ? "Interior" : "Not Interior") << "\n";
			std::cout << "target:  " << ((grid_->at(nodeEnd.x, nodeEnd.y) == SwarmOccupancyTree::INTERIOR_MARK) ? "Interior" : "Not Interior") << "\n";
			std::cout << "Search terminated. Did not find goal state\n";

		}

		// Display the number of loops the search went through
		//std::cout << "SearchSteps : " << SearchSteps << "\n";

		SearchCount++;

		//astarsearch.EnsureMemoryFreed();
	}

	return path;
}

AStar::AStar(Grid* grid) : grid_(grid) {
	astarsearch_ = new AStarSearch<MapSearchNode>(100000);
}
