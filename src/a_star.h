#include<bits/stdc++.h>
using namespace std;

#define ROW 9
#define COL 10

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
void tracePath(cell[][COL], Pair);
bool searchSuccessor(cell[][COL], bool[][COL], int [][COL], set<pPair> &, int, int, int, int, Pair);
void aStarSearch(int[][COL], Pair, Pair);
