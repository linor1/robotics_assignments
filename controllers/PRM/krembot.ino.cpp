#include "krembot.ino.h"
#include <cstdio>
#include <fstream>

/*
TO ASK:
- what's the size of the robot?
- is there a way to release allocated memory?
- can I use std's containers?
- how can I use structures that are in ARGOS documentation?
*/

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
std::map<std::pair<float, float>,CVector2> milestones;
// std::set<CVector2> milestones;

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


    PRM_controller::write_grid("/home/einat/grid.txt", occupancyGrid, height, width);
    PRM_controller::thickening_grid(occupancyGrid, coarseGrid, height, width, relativeResolution);
    PRM_controller::write_grid("/home/einat/coarse_grid.txt", coarseGrid, height, width);

    // milestones = new std::set<CVector2>();
    
    // Serial.Print("--- TEST ---");
    PRM_controller::fill_milestones_set(&milestones, height, width, 10, coarseGrid, resolution);
    PRM_controller::write_grid_with_milestones("/home/einat/grid_with_ml.txt", coarseGrid, height, width);
    std::cout << "setup done! set size: " << milestones.size() << std::endl;
}

void PRM_controller::loop()
{
    krembot.loop();

    pos = posMsg.pos;
    degreeX = posMsg.degreeX;
}

void PRM_controller::write_grid(std::string filename, int **grid, int height, int width)
{
    std::ofstream fid;
    // File Open
    fid.open(filename, std::ios_base::trunc);

    // Write to the file
    for (int col = 0; col < width; col++)
    {
        for (int row = height - 1; row >= 0; row--)
            fid << grid[col][row];
        fid << std::endl;
    }

    // File Close
    fid.close();
}

void PRM_controller::write_grid_with_milestones(std::string filename, int **grid, int height, int width)
{
    std::ofstream fid;
    // File Open
    fid.open(filename, std::ios_base::trunc);
    int symbol;
    
    // Write to the file
    for (int col = 0; col < width; col++)
    {
        for (int row = height - 1; row >= 0; row--) {
            symbol = grid[col][row];
            std::map<std::pair<float, float>,CVector2>::iterator it;
            for (it = milestones.begin(); it != milestones.end(); it++)
            {
                // std::cout << it->first.first << ": " << it->first.second << std::endl;
                int pCol = (it->first.first - origin.GetX() * resolution) / resolution;
                int pRow = (it->first.second - origin.GetY() * resolution) / resolution;
                if ((col == pCol) && (row == pRow)) {
                                symbol = 2;
                                break;
                            }
            }
            fid << symbol;
        }      
        fid << std::endl;
    }

    // File Close
    fid.close();
}

void PRM_controller::thickening_grid(int **origGrid, int **newGrid, int height, int width, Real res)
{
    // by default: fill with 0 = free
    int filling_value = 0;
    // iterate over grid cells
    for (int i = 0; i < height; i += res)
    {
        for (int j = 0; j < width; j += res)
        {
            filling_value = 0;
            // for each cells
            // iterate over old and find the right value
            for (int kheight = 0; kheight < res; kheight++)
            {
                for (int kwidth = 0; kwidth < res; kwidth++)
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
            for (int kheight = 0; kheight < res; kheight++)
                for (int kwidth = 0; kwidth < res; kwidth++)
                    newGrid[i + kheight][j + kwidth] = filling_value;
        }
    }
}

static float random_float(float max) {
    return static_cast<float>(rand()) / static_cast<float>(RAND_MAX / max);
}

void PRM_controller::fill_milestones_set(std::map<std::pair<float, float>,CVector2>* milestones,
                                int height, int width, int nmilestones, int** grid, Real res) {
    std::pair<float, float> new_key;
    while (milestones->size() < nmilestones) {
        generate_random_point(width, height, grid, resolution, new_key);
        if (milestones->find(new_key) == milestones->end()) {
            milestones->insert(make_pair(new_key, CVector2(new_key.first, new_key.second)));
            std::cout << "(" << new_key.first << ", " << new_key.second << ")" << std::endl;
        }
    }
    int size = milestones->size();
    
    LOG << "milestones " << milestones->size() << std::endl;
    
}

void PRM_controller::generate_random_point(int width, int height, int **grid, Real res, std::pair<float,float> &oPair) {
    float x = random_float(width * resolution);
    float y = random_float(height * resolution);
    while (is_point_occupied(x, y, grid, resolution) == 1) {
        x = random_float(width * resolution);
        y = random_float(height * resolution);
    }

    oPair.first = x;
    oPair.second = y;
}

int PRM_controller::is_point_occupied(float x, float y, int **grid, Real res) {
    float origx = origin.GetX();
    float x_in_grid = x / resolution; // (x - origin.GetX() * res) / res;
    float y_in_grid = y / resolution;// (y - origin.GetY() * res) / res;
    return grid[(int)x_in_grid][(int)y_in_grid];
}

void PRM_controller::pos_to_grid(CVector2 pos, CVector2 *grid_pos, Real res) {
    grid_pos->SetX((pos.GetX() - origin.GetX()) / res);
    grid_pos->SetY((pos.GetY() - origin.GetY()) / res);
}
