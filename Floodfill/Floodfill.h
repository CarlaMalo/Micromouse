// Floodfill.h

#ifndef _FLOODFILL_h
#define _FLOODFILL_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include <vector>
#include <queue>

static const int width = 7;
static const int height = 7;
static const int MAX_NUMBER_NODES = width * height;
static const int INF = INT_MAX;
using namespace std;


struct coordinate
{
public:
	int x, y;
	coordinate();
	coordinate(int x_, int y_);
	bool operator==(const coordinate & other) const;
	void update(int x, int y);
};


class Node {
public:
	int index;
	int cost;
	coordinate position;
	Node(coordinate& position, int cost);
	Node();
	void updateNode(coordinate& position, int cost);
};

class Floodfill {
private:
	int distance[MAX_NUMBER_NODES]; //shortest distances for each node (the node corresponds to the index)
	Node parent[MAX_NUMBER_NODES];  //parent node (that gives the shortest distance) for each node (the node corresponds to the index)
	bool closedSet[MAX_NUMBER_NODES]; //visited nodes with the least cost 
	class prioritize { public: bool operator ()(Node&p1, Node&p2) { return p1.cost > p2.cost; } };
	priority_queue<Node, vector<Node>, prioritize> openSet;  //reachable nodes
	vector<coordinate> finalpath;
	coordinate currentPosition;
	coordinate nextPosition;
	uint8_t minValue;
	Node startNode;
	Node targetNode;
public:
	Floodfill(coordinate startPosition, coordinate targetPosition);
	coordinate getBestNeighbour(int(&walls)[width*height][4], coordinate &currentPosition);
	void generateMaze(int(&maze)[width][height], int(&walls)[width*height][4]);
	vector<coordinate> getPath();
};

class MotionPlanner {
private:
	int8_t delta_x;
	int8_t delta_y;
	int8_t next_delta_x;
	int8_t next_delta_y;
public:
	MotionPlanner();
	char getMovement(coordinate &currentPosition, coordinate &nextPosition);
	char getOrientation();
};
#endif

