#include <iostream>
#include <string>
#include <tuple>
#include <bits/stdc++.h>
using namespace std;
typedef pair<int, int> Pair;

#define COL 23
#define ROW 31
#define half_square 1
#define u_row_num 2
#define u_rec_length 1

//31x23

int check_num_u(char maze_input[][COL], Pair dest, Pair obs_dest, int rob_direction){
    int square_boundary[4];//(x,y,x,y) (top left, bottom right)
    int u_checking_boundary[4];
    int num_u = 0;
    //form robot region
    square_boundary[0] = dest.first - half_square > 0 ? dest.first - half_square:0;
    square_boundary[1] = dest.second - half_square > 0 ? dest.second - half_square:0;
    square_boundary[2] = dest.first + half_square < ROW ? dest.first + half_square:ROW-1;//size of the x input matrix
    square_boundary[3] = dest.second + half_square < COL ? dest.second + half_square:COL-1;//size of the y input matrix
    // check whether robot is safe to go there
    if (square_boundary[2] - square_boundary[0] < 2 || square_boundary[3] - square_boundary[1] < 2){
        cout << "robot is too big";
        cout << endl;
        return 0;
    }

    //form obstacle region
    cout <<"obs location";
    cout << obs_dest.first;
    cout << obs_dest.second;
    cout << endl;
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
    cout <<"u boundary";
    cout << u_checking_boundary[0];
    cout << u_checking_boundary[1];
    cout << u_checking_boundary[2];
    cout << u_checking_boundary[3];
    cout << endl;
    // Check robot to ensure the robot is safe to go there
    for (int i = square_boundary[1]; i <= square_boundary[3]; i++){//go through first row
        if (maze_input[square_boundary[0]][i] == 'B'){
            cout << "front big";
            return 0;//cannot reach there
        }
    }
    for (int i = square_boundary[0]+1; i <= square_boundary[2]; i++){//go through first column
        if (maze_input[i][square_boundary[1]] == 'B'){
            cout << "front right";
            return 0;//cannot reach there
        }
    }
    for (int i = square_boundary[1]+1; i <= square_boundary[3]; i++){//go through second row
        if (maze_input[square_boundary[2]][i] == 'B'){
            cout << "left big";
            return 0;//cannot reach there
        }
    }
    for (int i = square_boundary[0]+1; i <= square_boundary[2]-1; i++){
        if (maze_input[i][square_boundary[3]] == 'B'){
            cout << "back big";
            return 0;//cannot reach there
        }
    }

    // Check the obstacle rectangle
    for (int r = u_checking_boundary[0]; r <= u_checking_boundary[2]; r++){
        for (int i = u_checking_boundary[1]; i <= u_checking_boundary[3]; i++){//go through first row
            if (maze_input[r][i] == 'U'){
                num_u += 1;
            }
        }
    }

    cout << "num_u";
    cout << endl;
    cout << num_u;
    cout << endl;
    return num_u;
}
Pair goal(char maze_input[][COL], int grid[][COL], Pair cur_pos){

    int direction = 0;//1: forward, 2: right, 3: left, 4: back
    int max_u = 0;//minimum possible u
    int temp_u = 0;
    Pair rob_dest;
    Pair obs_dest;
    Pair final_dest;
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
            cout << direction;
            cout << "y: ";
            cout << rob_dest.first;
            cout << "x: ";
            cout << rob_dest.second;
            cout << endl;
            break;
        }else if (maze_input[i][cur_pos.second] == 'B'){//if blocked at front, direction stay at 0
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
            cout << direction;
            cout << "y: ";
            cout << rob_dest.first;
            cout << "x: ";
            cout << rob_dest.second;
            cout << endl;
            break;
        }else if (maze_input[i][cur_pos.second] == 'B'){//if blocked at right, direction stay as before
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
            cout << direction;
            cout << "y: ";
            cout << rob_dest.first;
            cout << "x: ";
            cout << rob_dest.second;
            cout << endl;
            break;
        }else if (maze_input[i][cur_pos.second] == 'B'){//if blocked at right, direction stay as before
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
            cout << direction;
            cout << "y: ";
            cout << rob_dest.first;
            cout << "x: ";
            cout << rob_dest.second;
            cout << endl;
            break;
        }else if (maze_input[i][cur_pos.second] == 'B'){//if blocked at front, direction stay at 0
            break;
        }
    }
    if (direction == 1){

    }
    cout << "final y: ";
    cout << final_dest.first;
    cout << " final x: ";
    cout << final_dest.second;
    cout << endl;

    return final_dest;
}

int main()
{
    /*char maze_input[ROW][COL] = {{'U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
                               {'U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
                               {'U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
                               {'U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
                               {'U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
                               {'U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
                               {'U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
                               {'U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
                               {'U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
                               {'U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
                               {'U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
                               {'U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
                               {'U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
                               {'U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
                               {'U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
                               {'U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
                               {'U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
                               {'U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
                               {'U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
                               {'U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
                               {'U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
                               {'U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
                               {'U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
                               {'U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
                               {'U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
                               {'U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
                               {'U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
                               {'U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
                               {'U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
                               {'U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
                               {'U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'}};*/

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

    char maze_input_big[ROW*2][COL*2];
    for (int i = 0; i < ROW * 2; i++){
        for (int j = 0; j < COL * 2; j++){
            if (i == ROW && j == COL){
                maze_input_big[i][j] = 'W';
            }else{
                maze_input_big[i][j] = 'U';
            }
        }
    }
    //char maze_input[6][6] = {{'B','B','B','U','U','U'},
    //                        {'U','U','T','T','T','U'},
    //                        {'U','O','T','W','T','U'},
    //                        {'U','O','T','T','T','U'},
    //                        {'U','U','U','U','U','U'},
    //                        {'U','U','U','U','U','U'}};
    int maze[ROW][COL];
    Pair start_position = make_pair(0,0);
    Pair final_position = make_pair(0,0);

    for (int i = 0; i < ROW; i++){
        for (int j = 0; j < COL; j++){
            maze[i][j] = (maze_input[i][j] == 'B'||maze_input[i][j] == 'U') ? 0:1;
            if (maze_input[i][j] == 'W' || maze_input[i][j] == 'A' || maze_input[i][j] == 'S' || maze_input[i][j] == 'D'){
                //start_position = "(" + std::to_string(i) + "," + std::to_string(j) + ")";
                start_position = make_pair(i,j);
            }
        }
    }
    cout << start_position.first;
    cout << endl;
    cout << endl;
    //This will output the array.
    for (int i = 0; i < ROW; i++)
    {
        for (int j = 0; COL < 6; j++){
            cout << maze_input[i][j];
        }
        cout << endl;
    }
    cout << endl;
    //This will output the array.
    for (int i = 0; i < ROW; i++)
    {
        for (int j = 0; j < COL; j++){
            cout << maze[i][j];
        }
        cout << endl;
    }
    cout << endl;
    // testing purpose
    /*int size_square = 3;
    int half_square = size_square/2;
    int square_boundary[4];//(x,y,x,y) (top left, bottom right)
    //checking boundaries
    square_boundary[0] = start_position.first - half_square > 0 ? start_position.first - half_square:0;
    square_boundary[1] = start_position.second - half_square > 0 ? start_position.second - half_square:0;
    square_boundary[2] = start_position.first + half_square < 6 ? start_position.first + half_square:6-1;//size of the x input matrix
    square_boundary[3] = start_position.second + half_square < 6 ? start_position.second + half_square:6-1;//size of the y input matrix

    //check
    for (int i = 0; i < 4; i++){
        cout << square_boundary[i];
        cout << " ";
    }
    cout << endl;*/

    // only find the shortest u at the front
    final_position = goal(maze_input, maze, start_position);
    cout << final_position.first;
    cout << final_position.second;
    /*for (int i = start_position.first; i > 0; i--){
        if (maze_input[i][start_position.second] == 'U'){
            final_position = make_pair(i, start_position.second);
            cout << i;
            cout << start_position.second;
            break;

        }
    }*/


    //myvector.push_back (myint);
    /*int dist_list[] = {};
    float shortest_dist = 999;
    int dist_index = 0;
    // find the shortest
    for (int i = square_boundary[1]; i <= square_boundary[3]; i++){//go through first row
        if (maze_input[square_boundary[0]][i] == 'U'){
            cout << 'U';
            if (shortest_dist > square_boundary[0]+i){
                shortest_dist = square_boundary[0]+i;
                //shortest_dist.push_front(i);
                //shortest_dist.push_front(square_boundary[0]);
            }else{

            }
        }
        else{
            cout << 'O';
        }
    }
    cout << endl;
    for (int i = square_boundary[0]+1; i <= square_boundary[2]; i++){//go through first column
        if (maze_input[i][square_boundary[1]] == 'U'){
            cout << 'U';
            if (shortest_dist > square_boundary[0]+i){
                shortest_dist = square_boundary[0]+i;
                //shortest_dist.push_front(i);
                //shortest_dist.push_front(square_boundary[0]);
            }
        }
        else{
            cout << 'O';
        }
    }
    cout << endl;
    for (int i = square_boundary[1]+1; i <= square_boundary[3]; i++){//go through second row
        if (maze_input[square_boundary[2]][i] == 'U'){
            cout << 'U';
        }
        else{
            cout << 'O';
        }
    }
    cout << endl;
    for (int i = square_boundary[0]+1; i <= square_boundary[2]-1; i++){
        if (maze_input[i][square_boundary[3]] == 'U'){
            cout << 'U';
        }
        else{
            cout << 'O';
        }
    }*/

    return 0;
}
