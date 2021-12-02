#include "krembot.ino.h"
#include <cstdio>
#include <fstream>

float radius = 0.1;
int col, row;

int **occupancyGrid;
int **coarseGrid;
Real resolution;
CVector2 origin;
int height, width;
CVector2 pos;
CDegrees degreeX;
Real coarseResolution;

int coraseWidth, coarseHeight;

enum State
{
    move,
    turn
} state = turn;

void PRM_controller::setup()
{
    krembot.setup();
    krembot.Led.write(0, 255, 0);

    occupancyGrid = mapMsg.occupancyGrid;
    resolution = mapMsg.resolution;
    origin = mapMsg.origin;
    height = mapMsg.height;
    width = mapMsg.width;

    // the size of each grid
    coarseResolution = (float)radius;
    // relative resolution indicates the number of cells which come into one in the coarse grid
    int relativeResolution = (int)(coarseResolution / resolution);

    // create coarse grid array
    coarseGrid = new int *[height];     // row = height
    for (int i = 0; i < height; i++)    
        coarseGrid[i] = new int[width]; // col = width


    PRM_controller::write_grid("grid.txt", occupancyGrid, height, width);
    PRM_controller::thickening_grid(occupancyGrid, coarseGrid, height, width, relativeResolution);
    PRM_controller::write_grid("coarse_grid.txt", coarseGrid, height, width);
    std::cout << "setup done!" << std::endl;
}

void PRM_controller::loop()
{
    krembot.loop();

    pos = posMsg.pos;
    degreeX = posMsg.degreeX;
}

void PRM_controller::write_grid(std::string filename, int **grid, int h, int w)
{
    std::ofstream fid;
    // File Open
    fid.open(filename, std::ios_base::trunc);

    // Write to the file
    for (int col = 0; col < w; col++)
    {
        for (int row = h - 1; row >= 0; row--)
            fid << grid[col][row];
        fid << std::endl;
    }

    // File Close
    fid.close();
}

void PRM_controller::thickening_grid(int **origGrid, int **newGrid, int height, int width, int resolution)
{
    // by default: fill with 0 = free
    int filling_value = 0;
    // iterate over grid cells
    for (int i = 0; i < height; i += resolution)
    {
        for (int j = 0; j < width; j += resolution)
        {
            filling_value = 0;
            // for each cells
            // iterate over old and find the right value
            for (int kheight = 0; kheight < resolution; kheight++)
            {
                for (int kwidth = 0; kwidth < resolution; kwidth++)
                {
                    if (origGrid[i + kheight][j + kwidth] == 1)
                    {
                        filling_value = 1;
                        break;
                    }
                }
                if (filling_value == 1)
                    break;
            }

            // fill in the value
            for (int kheight = 0; kheight < resolution; kheight++)
                for (int kwidth = 0; kwidth < resolution; kwidth++)
                    newGrid[i + kheight][j + kwidth] = filling_value;
        }
    }
}

static float random_float(float max) {
    return static_cast<float>(rand()) / static_cast<float>(RAND_MAX / max);
}

void PRM_controller::generate_random_point(int width, int height, CVector2 *point, int **grid) {
    float x = random_float(width);
    float y = random_float(height);
    while (is_point_occupied(x, y, grid, resolution) == 1) {
        x = random_float(width);
        y = random_float(height);
    }
    point->SetX(x);
    point->SetY(y);
    // return CVector2(x, y);
}

int PRM_controller::is_point_occupied(float x, float y, int **grid, Real resolution) {
    float x_in_grid = (x - origin.GetX()) / resolution;
    float y_in_grid = (y - origin.GetY()) / resolution;
    return grid[(int)x_in_grid][(int)y_in_grid];
}

void PRM_controller::pos_to_grid(CVector2 pos, CVector2 *grid_pos, Real resolution) {
    grid_pos->SetX((pos.GetX() - origin.GetX()) / resolution);
    grid_pos->SetY((pos.GetY() - origin.GetY()) / resolution);
}
