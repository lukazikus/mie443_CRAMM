#include<bits/stdc++.h>
using namespace std;

#define ROW 6
#define COL 6
#define SCALE 3.0
#define RES 0.1

struct cell{
    int parent_i, parent_j; // Row and Column index of its parent; Note that 0 <= i <= ROW-1 & 0 <= j <= COL-1
    double f, g, h;
};

typedef pair<int, int> Pair; // Creating a shortcut for int, int pair type
typedef pair<double, pair<int, int> > pPair; // Creating a shortcut for pair<int, pair<int, int>> type
bool isValid(int, int);
bool isUnBlocked(int[][COL], int, int);
bool isDestination(int, int, Pair);
double calculateHValue(int, int, Pair);
stack<Pair> path(cell, Pair);
void tracePath(stack<Pair>);
bool searchSuccessor(cell[][COL], bool[][COL], int [][COL], set<pPair> &, int, int, int, int, Pair, stack<Pair> &Path);
void printMap(int[][COL]);
void convertMap(char[][COL], int[][COL], Pair &);
float map_to_world(int);
int world_to_map(float);
Pair goal(char[][COL], int[][COL], Pair);
void aStarSearch(int[][COL], Pair, Pair, stack<Pair> &Path);
