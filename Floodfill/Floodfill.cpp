// 
// 
// 

#include "Floodfill.h"

coordinate::coordinate() : x(0), y(0) {};
coordinate::coordinate(int x_, int y_) : x(x_), y(y_) {};
bool coordinate::operator==(const coordinate& other) const {
	return x == other.x && y == other.y;
}
void coordinate::update(int x, int y){
	this->x = x;
	this->y = y;
}

Node::Node(coordinate& position, int cost) :
	position(position), cost(cost) {
	index = width * position.x + position.y;
}

Node::Node() : position(), cost(0), index(0) {}

void Node::updateNode(coordinate & position, int cost)
{
	this->position = position;
	this->cost = cost;
	index = width * position.x + position.y;
}


Floodfill::Floodfill(coordinate startPosition, coordinate targetPosition) {
	//Initialize values of variables
	startNode.updateNode(startPosition, 0);
	targetNode.updateNode(targetPosition, 0);//the cost of this node is going to change
	fill(distance, distance + MAX_NUMBER_NODES, INF);
	distance[startNode.index] = 0;
	//fill(parent, parent + MAX_NUMBER_NODES, 0);
	fill(closedSet, closedSet + MAX_NUMBER_NODES, false);
	minValue = INF;
}

coordinate Floodfill::getBestNeighbour(int(&walls)[width*height][4], coordinate &currentPosition) {
	minValue = INF;
	char move = ' ';
	int neighbour[4][2] = { {0,1},{0,-1} ,{-1,0} ,{1,0} }; //F,D,L,R
	for (unsigned int i = 0; i < 4; i++) {
		coordinate neighbourPosition(currentPosition.x + neighbour[i][0], currentPosition.y + neighbour[i][1]);
		if (neighbourPosition.x > (width - 1) || neighbourPosition.x < 0 || neighbourPosition.y >(height - 1) || neighbourPosition.y < 0) continue;
		if (walls[width * currentPosition.x + currentPosition.y][i] == 1) continue;
		if (distance[width*neighbourPosition.x + neighbourPosition.y] < minValue) {
			minValue = distance[width*neighbourPosition.x + neighbourPosition.y];
			nextPosition.x = neighbourPosition.x;
			nextPosition.y = neighbourPosition.y;
		}
	}

	return nextPosition;
}


void Floodfill::generateMaze(int(&maze)[width][height], int(&walls)[width * height][4])
{
	fill(distance, distance + MAX_NUMBER_NODES, INF);
	fill(closedSet, closedSet + MAX_NUMBER_NODES, false);
	fill(parent, parent + MAX_NUMBER_NODES, Node());
	distance[startNode.index] = 0;


	openSet.push(startNode);
	//While there are neighbours to explore
	while (!openSet.empty()) {
		Node currentNode = openSet.top(); //the top node is the smallest weight of the openSet
		openSet.pop(); //Remove the node

		//Check if the currenNode is the target node, if it save the path and  stop the algorithm

		/*if (currentNode.index == targetNode.index) {
			break;
		}*/
		//Check if the currentNode is on the closed set already
		if (closedSet[currentNode.index]) continue;
		else closedSet[currentNode.index] = true;

		//Neighbors of the currentNode				
		int neighbour[4][2] = { {0,1},{0,-1} ,{-1,0} ,{1,0} };
		for (unsigned int i = 0; i < 4; i++) {
			//Check that it doesn't go beyond the maze boundaries
			coordinate position(currentNode.position.x + neighbour[i][0], currentNode.position.y + neighbour[i][1]);
			if (position.x > (width - 1) || position.x < 0 || position.y >(height - 1) || position.y < 0) continue;
			if (walls[width * currentNode.position.x + currentNode.position.y][i] == 1) continue;
			int cost = currentNode.cost + 1;
			Node neighbourNode(position, cost);
			//if Node is already on closedSet skip, continue
			if (closedSet[neighbourNode.index]) continue;

			//Update if the new value of the node is smaller
			if (cost > distance[neighbourNode.index]) continue;
			distance[neighbourNode.index] = cost;
			maze[neighbourNode.position.x][neighbourNode.position.y] = cost;
			parent[neighbourNode.index] = currentNode;

			// Set the new distance and add the Node to priority queue
			openSet.push(neighbourNode);

		}
	}

	openSet = priority_queue <Node, vector<Node>, prioritize>();
	//finalpath.clear();
	/*
	for (unsigned int i = 0; i < height; i++) {
		for (unsigned int j = 0; j < width; j++)
			cout << distance[width*j + i] << " ";
		cout << endl;
	}
	*/

}
vector<coordinate> Floodfill::getPath()
{
	Node current = targetNode;
	while (current.index != startNode.index) {
		finalpath.push_back(current.position);
		current = parent[current.index];
	}
	finalpath.push_back(startNode.position);
	reverse(finalpath.begin(), finalpath.end());
	/*
	for (unsigned int i = 0; i < finalpath.size(); i++)
		cout << "(" << finalpath[i].first << "," << finalpath[i].second << " ) ";
	cout << endl;
	*/
	return finalpath;
}

MotionPlanner::MotionPlanner() : delta_x(0), delta_y(1), next_delta_x(0), next_delta_y(0) {}

char MotionPlanner::getMovement(coordinate & currentPosition, coordinate & nextPosition)
{
	char move = ' ';
	next_delta_x = (nextPosition.x - currentPosition.x);
	next_delta_y = (nextPosition.y - currentPosition.y);

	int turn = delta_x * next_delta_y - delta_y * next_delta_x;
	switch (turn)
	{
	case 1: //turn left
		move = 'L';
		break;
	case -1: //turn right
		move = 'R';
		break;
	default:
		//going straight
		if (delta_x == next_delta_x && delta_y == next_delta_y) {
			move = 'F';
		}
		else { //180°
			move = 'B';
			//next_delta_x = delta_x;
			//next_delta_y = delta_y;
		}
	}

	delta_x = next_delta_x;
	delta_y = next_delta_y;
	currentPosition = nextPosition;
	return move;
}

char MotionPlanner::getOrientation() {
	char orientation = ' ';
	uint8_t x_global_ref = 0;
	uint8_t y_global_ref = 1;
	int direction = delta_x * y_global_ref - delta_y * x_global_ref;
	switch (direction)
	{
	case 1:
		orientation = 'E';
		break;
	case -1:
		orientation = 'W';
		break;
	default:

		if (delta_x == x_global_ref && delta_y == y_global_ref) {
			orientation = 'N';
		}
		else {
			orientation = 'S';
		}
	}
	return orientation;
}
