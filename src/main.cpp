#include <iostream>
#include <string>
#include <bits/stdc++.h>
using namespace std;
typedef pair<int, int> Pair;

int main()
{
    char maze_input[6][6] = {{'B','B','B','U','U','U'},{'O','U','O','U','U','U'},{'O','O','O', 'U','U','U'},{'T','T','T','O','O','O'},{'T','W','T','O','O','O'},{'T','T','T','O','O','O'}};
    int maze[6][6];
    Pair start_position = make_pair(0,0); // Initialize starting and final positions
    Pair final_position = make_pair(0,0);

    for (int i = 0; i < 6; i++){
        for (int j = 0; j < 6; j++){
            grid[i][j] = (maze_input[i][j] == 'B' || maze_input[i][j] == 'U') ? 0 : 1;
            if (maze_input[i][j] == 'W' || maze_input[i][j] == 'A' || maze_input[i][j] == 'S' || maze_input[i][j] == 'D'){
                start_position = make_pair(i,j);
            }
        }
    }

    // only find the shortest u at the front
    for (int i = start_position.first; i > 0; i--){
        if (maze_input[i][start_position.second] == 'U'){
            final_position = make_pair(i, start_position.second);
            break;
        }
    }
    return 0;
}
