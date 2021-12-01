#include "krembot.ino.h"
#include <cstdio>
#include <fstream>
using namespace std;


int col, row;

int **occupancyGrid;
Real resolution;
CVector2 origin;
int height, width;
CVector2 pos;
CDegrees degreeX;

enum State
{
    move,
    turn
} state = turn;

void walkingOnGrid_controller::setup()
{
    krembot.setup();
    krembot.Led.write(0, 255, 0);

    occupancyGrid = mapMsg.occupancyGrid;
    resolution = mapMsg.resolution;
    origin = mapMsg.origin;
    height = mapMsg.height;
    width = mapMsg.width;

    walkingOnGrid_controller::write_grid("grid.txt", occupancyGrid, height, width);
}

void walkingOnGrid_controller::loop()
{
    krembot.loop();

    pos = posMsg.pos;
    degreeX = posMsg.degreeX;
}

void walkingOnGrid_controller::write_grid(std::string filename, int** grid, int h, int w) {
    std::ofstream fid;
    // File Open
    fid.open(filename, std::ios_base::trunc);

    // Write to the file
    for (int col = w-1; col >= 0; col--)
    {
        for (int row = h-1; row >= 0; row--)
            fid << grid[row][col];
        fid << std::endl;   
    }

    // File Close
    fid.close();
}
