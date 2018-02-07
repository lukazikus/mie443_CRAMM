#include <bits/stdc++.h>
#include "../include/a_star.h"
using namespace std;

// A Utility Function to check whether given cell (row, col) is a valid cell or not.
bool isValid(int row, int col){
    return (row >= 0) && (row < ROW) && (col >= 0) && (col < COL);
}

// A Utility Function to check whether the given cell is blocked or not
bool isUnBlocked(int grid[][COL], int row, int col){
    return grid[row][col] == 1 ? true : false;
}

// A Utility Function to check whether destination cell has been reached or not
bool isDestination(int row, int col, Pair dest){
    return row == dest.first && col == dest.second ? true : false;
}

// A Utility Function to calculate the 'h' heuristics.
double calculateHValue(int row, int col, Pair dest){
    // Return using the distance formula
    return (double)(abs(row-dest.first) + abs(col-dest.second));
}

stack<Pair> path(cell cellDetails[][COL], Pair dest){
    int row = dest.first;
    int col = dest.second;

    stack<Pair> Path;

    while (!(cellDetails[row][col].parent_i == row && cellDetails[row][col].parent_j == col )){
        Path.push (make_pair (row, col));
        int temp_row = cellDetails[row][col].parent_i;
        int temp_col = cellDetails[row][col].parent_j;
        row = temp_row;
        col = temp_col;
    }

    Path.push (make_pair (row, col));
    return Path;
}

// A Utility Function to trace the path from the source to destination
void tracePath(stack<Pair> Path){
    while (!Path.empty()){
        Pair p = Path.top();
        Path.pop();
        printf("-> (%d,%d) ",p.first,p.second);
    }
    printf("\n");
}

bool searchSuccessor(cell cellDetails[][COL], bool closedList[][COL], int grid[][COL], set<pPair> &openList, int i, int j, int k, int l, Pair dest, stack<Pair> &Path){
    // To store the 'g', 'h' and 'f' of the 8 successors
    double gNew, hNew, fNew;

    // Only process this cell if this is a valid one
    if (isValid(k, l) == true){
        // If the destination cell is the same as the current successor
        if (isDestination(k, l, dest) == true){
            // Set the Parent of the destination cell
            cellDetails[k][l].parent_i = i;
            cellDetails[k][l].parent_j = j;
            printf ("The destination cell is found\n");
            Path = path(cellDetails, dest);
            // tracePath (Path);
            return true;
        }
        // If the successor is already on the closed list or if it is blocked, then ignore it.
        else if (closedList[k][l] == false && isUnBlocked(grid, k, l) == true){
            gNew = cellDetails[i][j].g + 1.0;
            hNew = calculateHValue (k, l, dest);
            fNew = gNew + hNew;

            // If it isn’t on the open list, add it to the open list. Make the current square the parent of this square
            // If it is on the open list already, check to see if this path to that square is better
            if (cellDetails[k][l].f == FLT_MAX || cellDetails[k][l].f > fNew){
                openList.insert( make_pair(fNew, make_pair(k, l)));

                // Update the details of this cell
                cellDetails[k][l].f = fNew;
                cellDetails[k][l].g = gNew;
                cellDetails[k][l].h = hNew;
                cellDetails[k][l].parent_i = i;
                cellDetails[k][l].parent_j = j;
            }
        }
    }
    return false;
}

void printMap(int grid[][COL]){
    for(int i = 0; i < ROW; i++){
        for(int j = 0; j < COL; j++){
            printf("%d,", grid[i][j]);
        }
        printf("\n");
    }
}

void convertMap(char maze_input[][COL] , int grid[][COL], Pair &src){
    // Create grid map that can be parsed by A*
    for (int i = 0; i < ROW; i++){
        for (int j = 0; j < COL; j++){
            grid[i][j] = (maze_input[i][j] == 'B' || maze_input[i][j] == 'U') ? 0 : 1;
            if (maze_input[i][j] == 'W' || maze_input[i][j] == 'A' || maze_input[i][j] == 'S' || maze_input[i][j] == 'D'){
                src = make_pair(i,j);
            }
        }
    }
}

float map_to_world(int map_coord){
    return map_coord*SCALE;
}

int world_to_map(float world_coord){
    return int(world_coord/SCALE);
}

int check_num_u(char maze_input[][COL], Pair dest, Pair obs_dest, int rob_direction){
    int square_boundary[4];//(x,y,x,y) (top left, bottom right)
    int u_checking_boundary[4];
    int num_u = 0;
    //form robot region
    square_boundary[0] = dest.first - half_square >= 0 ? dest.first - half_square:0;
    square_boundary[1] = dest.second - half_square >= 0 ? dest.second - half_square:0;
    square_boundary[2] = dest.first + half_square < ROW ? dest.first + half_square:ROW-1;//size of the x input matrix
    square_boundary[3] = dest.second + half_square < COL ? dest.second + half_square:COL-1;//size of the y input matrix
    // check whether robot is safe to go there
    if (square_boundary[2] - square_boundary[0] < 2 || square_boundary[3] - square_boundary[1] < 2){
        //cout << "robot is too big";
        //cout << endl;
        return 0;
    }

    if (rob_direction == 1){//front
        u_checking_boundary[0] = obs_dest.first - u_rec_length >= 0 ? obs_dest.first - u_rec_length:0;
        u_checking_boundary[1] = obs_dest.second - u_rec_length >= 0 ? obs_dest.second - u_rec_length:0;
        u_checking_boundary[2] = obs_dest.first;
        u_checking_boundary[3] = obs_dest.second + u_rec_length <= COL-1 ? obs_dest.second + u_rec_length:COL-1;
    }else if (rob_direction == 2){//right
        u_checking_boundary[0] = obs_dest.first - u_rec_length >= 0 ? obs_dest.first - u_rec_length:0;
        u_checking_boundary[1] = obs_dest.second;
        u_checking_boundary[2] = obs_dest.first + u_rec_length <= ROW-1 ? obs_dest.first + u_rec_length:ROW-1;
        u_checking_boundary[3] = obs_dest.second + u_rec_length <= COL-1 ? obs_dest.second + u_rec_length:COL-1;
    }else if (rob_direction == 3){//left
        u_checking_boundary[0] = obs_dest.first - u_rec_length >= 0 ? obs_dest.first - u_rec_length:0;
        u_checking_boundary[1] = obs_dest.second - u_rec_length >= 0 ? obs_dest.second - u_rec_length:0;
        u_checking_boundary[2] = obs_dest.first + u_rec_length <= ROW-1 ? obs_dest.first + u_rec_length:ROW-1;
        u_checking_boundary[3] = obs_dest.second;//size of the y input matrix
    }else if (rob_direction == 4){//back
        u_checking_boundary[0] = obs_dest.first;
        u_checking_boundary[1] = obs_dest.second - u_rec_length >= 0 ? obs_dest.second - u_rec_length:0;
        u_checking_boundary[2] = obs_dest.first + u_rec_length <= ROW-1 ? obs_dest.first + u_rec_length:ROW-1;//size of the x input matrix
        u_checking_boundary[3] = obs_dest.second + u_rec_length <= COL-1 ? obs_dest.second + u_rec_length:COL-1;//size of the y input matrix
    }

    // Check the obstacle rectangle
    for (int r = u_checking_boundary[0]; r <= u_checking_boundary[2]; r++){
        for (int i = u_checking_boundary[1]; i <= u_checking_boundary[3]; i++){//go through first row
            if (maze_input[r][i] == 'U'){
                num_u += 1;
            }
        }
    }

    return num_u;
}
Pair goal(char maze_input[][COL], int grid[][COL], Pair cur_pos){
    int direction = 0;//1: forward, 2: right, 3: left, 4: back
    int max_u = 0;//maximum possible u
    int temp_u = 0; // Temporary
    Pair rob_dest; // Robot destination
    Pair obs_dest; // Obstacle destination
    Pair final_dest; // Can eventually be same as rob_dest
    // Only find the shortest U at the front
    for (int i = cur_pos.first; i >= 0; i--){
        if (maze_input[i][cur_pos.second] == 'U' || maze_input[i][cur_pos.second - 1] == 'U' || maze_input[i][cur_pos.second + 1] == 'U'){//first U at the front
            rob_dest = make_pair(i+2, cur_pos.second);
            obs_dest = make_pair(i, cur_pos.second);
            temp_u = check_num_u(maze_input, rob_dest, obs_dest, 1);
            if (temp_u > max_u){
                max_u = temp_u;
                direction = 1;
                final_dest = rob_dest;
            }

            break;
        }else if (maze_input[i][cur_pos.second] == 'B' || maze_input[i][cur_pos.second - 1] == 'B' || maze_input[i][cur_pos.second + 1] == 'B'){//if blocked at front, direction stay at 0
            break;
        }
    }
    // Only find the shortest U at the right
    for (int i = cur_pos.second; i < COL; i++){
        if (maze_input[cur_pos.first][i] == 'U'){//first U at the front
            rob_dest = make_pair(cur_pos.first, i-2);//robot needs to go to a 'O' that is 2 cells before the U in order to not crash
            obs_dest = make_pair(cur_pos.first, i);
            temp_u = check_num_u(maze_input, rob_dest, obs_dest, 2);
            if (temp_u > max_u){
                max_u = temp_u;
                direction = 2;
                final_dest = rob_dest;
            }

            break;
        }else if (maze_input[cur_pos.first][i] == 'B' || maze_input[cur_pos.first+1][i] == 'B' || maze_input[cur_pos.first-1][i] == 'B'){//if blocked at right, direction stay as before
            break;
        }
    }
    // Only find the shortest U at the left
    for (int i = cur_pos.second; i >= 0; i--){
        if (maze_input[cur_pos.first][i] == 'U'){//first U at the front
            rob_dest = make_pair(cur_pos.first, i+2);//robot needs to go to a 'O' that is 2 cells before the U in order to not crash
            obs_dest = make_pair(cur_pos.first, i);
            temp_u = check_num_u(maze_input, rob_dest, obs_dest, 3);
            if (temp_u > max_u){
                max_u = temp_u;
                direction = 3;
                final_dest = rob_dest;
            }

            break;
        }else if (maze_input[cur_pos.first][i] == 'B' || maze_input[cur_pos.first+1][i] == 'B' || maze_input[cur_pos.first-1][i] == 'B'){//if blocked at right, direction stay as before
            break;
        }
    }
    // Only find the shortest U at the back
    for (int i = cur_pos.first; i < ROW; i++){
        if (maze_input[i][cur_pos.second] == 'U'){//first U at the front
            rob_dest = make_pair(i-2, cur_pos.second);//robot needs to go to a 'O' that is 2 cells before the U in order to not crash
            obs_dest = make_pair(i, cur_pos.second);
            temp_u = check_num_u(maze_input, rob_dest, obs_dest, 4);
            if (temp_u > max_u){
                max_u = temp_u;
                direction = 3;
                final_dest = rob_dest;
            }

            break;
        }else if (maze_input[i][cur_pos.second] == 'B' || maze_input[i][cur_pos.second - 1] == 'B' || maze_input[i][cur_pos.second + 1] == 'B'){//if blocked at front, direction stay at 0
            break;
        }
    }

    if (direction == 0){//if everywhere around is explored
        if (maze_input[cur_pos.first - 2][cur_pos.second] == 'O'){//front is ok
            final_dest = make_pair(cur_pos.first - 2, cur_pos.second);
        }else if (maze_input[cur_pos.first][cur_pos.second]+2 == 'O'){//right is ok
            final_dest = make_pair(cur_pos.first, cur_pos.second+2);
        }else if (maze_input[cur_pos.first][cur_pos.second]-2 == 'O'){//left is ok
            final_dest = make_pair(cur_pos.first, cur_pos.second-2);
        }else if (maze_input[cur_pos.first + 2][cur_pos.second] == 'O'){//back is ok
            final_dest = make_pair(cur_pos.first + 2, cur_pos.second);
        }
    }

    return final_dest;
}

// A* Function to find the shortest path between a given source cell to a destination cell
void aStarSearch(int grid[][COL], Pair src, Pair dest, stack<Pair> &Path){
    // If the source is out of range
    if (isValid (src.first, src.second) == false){
        printf ("Source is invalid\n");
        return;
    }

    // If the destination is out of range
    if (isValid (dest.first, dest.second) == false){
        printf ("Destination is invalid\n");
        return;
    }

    // Either the source or the destination is blocked
    if (isUnBlocked(grid, src.first, src.second) == false || isUnBlocked(grid, dest.first, dest.second) == false){
        printf ("Source or the destination is blocked\n");
        return;
    }

    // If the destination cell is the same as source cell
    if (isDestination(src.first, src.second, dest) == true){
        printf ("We are already at the destination\n");
        return;
    }

    // Create a closed list and initialise it to false which means that no cell has been included yet
    bool closedList[ROW][COL];
    memset(closedList, false, sizeof (closedList));
    cell cellDetails[ROW][COL]; // Declare a 2D array of structs to hold the details of each grid cell

    // Set each cost node to the maximum float value
    int i = 0; int j = 0;
    for (i=0; i<ROW; i++){
        for (j=0; j<COL; j++){
            cellDetails[i][j].f = FLT_MAX;
            cellDetails[i][j].g = FLT_MAX;
            cellDetails[i][j].h = FLT_MAX;
            cellDetails[i][j].parent_i = -1;
            cellDetails[i][j].parent_j = -1;
        }
    }

    // Initialize the parameters of the starting node
    i = src.first, j = src.second;
    cellDetails[i][j].f = 0.0;
    cellDetails[i][j].g = 0.0;
    cellDetails[i][j].h = 0.0;
    cellDetails[i][j].parent_i = i;
    cellDetails[i][j].parent_j = j;

    set<pPair> openList; // Create an open list having information as- <f, <i, j>>, f=g+h, i, j are indices of that cell
    openList.insert(make_pair (0.0, make_pair (i, j))); // Put the starting cell on the open list and set its 'f' as 0
    bool foundDest = false; // We set this boolean value as false as initially the destination is not reached

    // Run through all elements of openList until the set is empty
    while (!openList.empty()){
        pPair p = *openList.begin();
        openList.erase(openList.begin()); // Remove this vertex from the open list

        // Add this vertex to the closed list
        i = p.second.first;
        j = p.second.second;
        closedList[i][j] = true;

        // To store the 'g', 'h' and 'f' of the 8 successors
        double gNew, hNew, fNew;

        // Assume only up, down, left, or right motions are possible
        if(searchSuccessor(cellDetails, closedList, grid, openList, i, j, i-1, j, dest, Path)) return;
        if(searchSuccessor(cellDetails, closedList, grid, openList, i, j, i+1, j, dest, Path)) return;
        if(searchSuccessor(cellDetails, closedList, grid, openList, i, j, i, j+1, dest, Path)) return;
        if(searchSuccessor(cellDetails, closedList, grid, openList, i, j, i, j-1, dest, Path)) return;
        // if(searchSuccessor(cellDetails, closedList, grid, openList, i, j, i-1, j+1, dest, Path)) return;
        // if(searchSuccessor(cellDetails, closedList, grid, openList, i, j, i-1, j-1, dest, Path)) return;
        // if(searchSuccessor(cellDetails, closedList, grid, openList, i, j, i+1, j+1, dest, Path)) return;
        // if(searchSuccessor(cellDetails, closedList, grid, openList, i, j, i+1, j+1, dest, Path)) return;
    }
    // When destination cell is not found and open list is empty, then we failed to reach the destination cell
    if (foundDest == false)
        printf("Failed to find the Destination Cell\n");

    return ;
}

// Test above function
int main(){
    /* Description of the Grid-
    1--> The cell is not blocked
    0--> The cell is blocked
    NOTE: Each grid cell represents where the robot's centre could potentially be
    When the map outputs B or O, it must account for the size of the robot
        */

    stack<Pair> Path;
    int grid[ROW][COL];
    Pair src, dest;

    char maze_input[ROW][COL] = {{'B','B','B','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','B','B','B','B'},
                               {'B','B','B','B','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','B','B','B'},
                               {'B','B','B','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','B','B'},
                               {'U','B','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','B'},
                               {'U','U','U','U','U','U','U','U','U','U','U','U','U','B','U','U','U','U','U','U','U','U','U'},
                               {'U','U','U','U','U','U','U','U','U','U','U','U','B','B','U','U','U','U','U','U','U','U','U'},
                               {'U','U','U','U','U','U','U','U','U','U','U','B','B','U','U','U','U','U','U','U','U','U','U'},
                               {'U','U','U','U','U','U','U','U','U','U','B','B','U','U','U','U','U','U','U','U','U','U','U'},
                               {'U','U','U','U','U','U','U','U','U','B','B','U','U','U','U','U','U','U','U','U','U','U','U'},
                               {'U','U','U','U','U','U','U','U','B','B','U','U','U','U','U','U','U','U','U','U','U','U','U'},
                               {'U','U','U','U','U','U','U','B','B','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
                               {'U','U','U','U','U','U','B','B','U','U','U','U','U','U','U','U','U','U','U','U','U','B','B'},
                               {'U','U','U','U','U','B','B','U','U','U','U','U','U','U','U','U','U','U','U','U','U','B','B'},
                               {'U','U','U','U','U','B','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','B','B'},
                               {'U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','B','B'},
                               {'U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','B','B'},
                               {'U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','B','B'},
                               {'U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','B','B'},
                               {'U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','B','B'},
                               {'U','U','U','U','U','B','B','B','B','B','B','B','B','B','B','B','B','U','U','U','U','U','U'},
                               {'U','U','U','U','U','B','B','B','B','B','B','B','B','B','B','B','B','U','U','U','U','U','U'},
                               {'U','U','U','U','U','B','B','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
                               {'U','U','U','U','U','B','B','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
                               {'U','U','U','U','U','B','B','U','U','U','U','U','U','U','B','U','U','U','U','U','U','U','U'},
                               {'U','U','U','U','U','B','B','U','U','U','U','U','U','B','B','B','U','U','U','U','U','U','U'},
                               {'U','U','U','U','U','U','U','U','U','U','U','U','B','B','B','B','B','U','U','U','U','U','U'},
                               {'O','O','O','U','U','U','U','U','U','U','U','U','U','B','B','B','U','U','U','U','U','U','U'},
                               {'O','O','O','U','U','U','U','U','U','U','U','U','U','U','B','U','U','U','U','U','U','U','U'},
                               {'T','T','T','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
                               {'T','W','T','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
                               {'T','T','T','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'}};

    // Create grid map that can be parsed by A*
    convertMap(maze_input, grid, src);
    dest = goal(maze_input, grid, src);

    printf("Source: (%d,%d)\n", src.first,src.second);
    printf("Destination: (%d,%d)\n", dest.first,dest.second);
    printMap(grid);

    aStarSearch(grid, src, dest, Path); // Run A*

    Pair p = Path.top();
    float WayX = p.first;
    float WayY = p.second;
    printf("(Way X,WayY) = (%d,%d)\n", WayX, WayY);

    return(0);
}
