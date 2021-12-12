#include "krembot.ino.h"
#include <cstdio>
#include <fstream>

// given initializations
int col, row;
int **occupancyGrid;
int **coarseGrid;
Real resolution;
CVector2 origin;
int height, width;
CVector2 pos;
CDegrees degreeX;

// my initializations
int NMILESTONES = 20;
Real coarseResolution;
std::map<std::pair<float, float>, CVector2> milestones;

enum State
{
    move,
    turn
} state;

void PRM_controller::setup()
{
    krembot.setup();
    krembot.Led.write(0, 255, 0);

    state = turn;
    occupancyGrid = mapMsg.occupancyGrid;
    resolution = mapMsg.resolution;
    origin = mapMsg.origin;
    height = mapMsg.height;
    width = mapMsg.width;

    // relative resolution indicates the number of cells which come into one in the coarse grid
    int relativeResolution = (int)(robotSize / resolution);

    // create coarse grid array and fill it with grid values
    coarseGrid = new int *[height]; // row = height
    for (int i = 0; i < height; i++)
        coarseGrid[i] = new int[width]; // col = width
    // re-size obstacles in grid
    PRM_controller::thickening_grid(occupancyGrid, coarseGrid, height, width, relativeResolution);

    // write both grids to files
    PRM_controller::write_grid("grid.txt", occupancyGrid, height, width);
    PRM_controller::write_grid("coarse_grid.txt", coarseGrid, height, width);

    // fill milestones with points, and write to file
    PRM_controller::fill_milestones_set(&milestones, height, width, NMILESTONES, coarseGrid);
    PRM_controller::write_grid_with_milestones("grid_with_ml.txt", coarseGrid, height, width);

    // this is a test: iterate over all milestones and determine whether there's a path between them
    std::map<std::pair<float, float>, CVector2>::iterator it_in;
    std::map<std::pair<float, float>, CVector2>::iterator it_out;
    for (it_out = milestones.begin(); it_out != milestones.end(); it_out++)
        for (it_in = milestones.begin(); it_in != milestones.end(); it_in++)
            std::cout << it_out->second << ", " << it_in->second << ": " << is_path_clear(it_out->second, it_in->second, coarseGrid) << std::endl;
        
    int i = 0;
    std::pair<float, float> new_key;
    Graph g(NMILESTONES);
    KdNodeVector nodes;
    std::vector<std::vector<float>> points(2);
    while (i < NMILESTONES) // fill map until it has NMILESTONES elements
    {
        //random point
        generate_random_point(100, 100 ,new_key);
        // insert new_key to k-d-tree
        std::vector<float> point(2);
        point[0]=new_key.first;
        point[1]=new_key.second;
        int_to_nodes_map[i] = point;
        //print_float_vector(new_keys_map[i]);
        //print("here");
        points.insert(points.end(),point);
        nodes.push_back(KdNode(point));
        i++;
    }
    KdTree tree(&nodes);
    cout << "Points in kd-tree:\n  ";
    print_nodes(tree.allnodes);
    KdNodeVector result;
    for(int l=0;l<NMILESTONES;l++){
        tree.k_nearest_neighbors(int_to_nodes_map[l], 4, &result);
        for(int b=0;b<3;b++){
            //todo: If there is an obstacle between the 2 vertices, do not insert the edge.
            g.addEdge(source(int_to_nodes_map,points,l),
                      destination(int_to_nodes_map,result,b));
        }
    }
    //example
    g.addEdge(0,1);
    g.addEdge(1,2);
    g.addEdge(2,4);
    g.addEdge(4,6);
    g.addEdge(6,7);
    g.addEdge(7,8);
    g.BFS(int_to_nodes_map, 0,6);

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
    // open file
    std::ofstream fid;
    fid.open(filename, std::ios_base::trunc);

    // write data to file
    for (int col = 0; col < width; col++)
    {
        for (int row = height - 1; row >= 0; row--)
            fid << grid[col][row];
        fid << std::endl;
    }

    fid.close();
}

void PRM_controller::write_grid_with_milestones(std::string filename, int **grid, int height, int width)
{
    // open file
    std::ofstream fid;
    fid.open(filename, std::ios_base::trunc);
    int symbol;

    // write data to file
    for (int col = 0; col < width; col++)
    {
        for (int row = height - 1; row >= 0; row--)
        {
            // symbol from grid
            symbol = grid[col][row];
            // find out if the cell has a point in it
            std::map<std::pair<float, float>, CVector2>::iterator it;
            for (it = milestones.begin(); it != milestones.end(); it++)
            {
                int pCol = it->first.first / resolution;
                int pRow = it->first.second / resolution;
                if ((col == pCol) && (row == pRow))
                {
                    symbol = 2;
                    break;
                }
            }
            fid << symbol;
        }
        fid << std::endl;
    }

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
            // iterate over coarseGrid cells and find out for each if it's occupied or not
            // note that the size of both grid (and cells in them) is the sanw
            filling_value = 0;
            for (int kheight = 0; kheight < res; kheight++)
            {
                for (int kwidth = 0; kwidth < res; kwidth++)
                {
                    // if any cell in the big cell is occupied, the whole cell would be occupied
                    if (origGrid[i + kheight][j + kwidth] == 1)
                    {
                        filling_value = 1;
                        break;
                    }
                }
                if (filling_value == 1)
                    break;
            }

            // fill in the value in the tested cell above
            for (int kheight = 0; kheight < res; kheight++)
                for (int kwidth = 0; kwidth < res; kwidth++)
                    newGrid[i + kheight][j + kwidth] = filling_value;
        }
    }
}

// generate random float in range [0-max]
static float random_float(float max)
{
    return static_cast<float>(rand()) / static_cast<float>(RAND_MAX / max);
}

void PRM_controller::fill_milestones_set(std::map<std::pair<float, float>, CVector2> *milestones,
                                         int height, int width, int nmilestones, int **grid)
{
    std::pair<float, float> new_key;
    while (milestones->size() < nmilestones) // fill map until it has nmilestones elements
    {
        generate_random_point(width, height, grid, new_key);    // fill the key with a new value
        if (milestones->find(new_key) == milestones->end())     // if the key doesn't exist already, insert it into the map
            milestones->insert(make_pair(new_key, CVector2(new_key.first, new_key.second)));
    }
}

void PRM_controller::generate_random_point(int width, int height, int **grid, std::pair<float, float> &oPair)
{
    float x = random_float(width * resolution);
    float y = random_float(height * resolution);
    while (is_point_occupied(x, y, grid) == 1)  // if (x, y) is occupied in the grid, generate new point
    {
        x = random_float(width * resolution);
        y = random_float(height * resolution);
    }

   //  fill the passed pair with the (x, y) values
    oPair.first = x;
    oPair.second = y;
}

int PRM_controller::is_point_occupied(float x, float y, int **grid)
{
    // transform (x, y) coordinates into grid cell
    float x_in_grid = x / resolution;
    float y_in_grid = y / resolution;
    return grid[(int)x_in_grid][(int)y_in_grid];
}

bool PRM_controller::is_path_clear(CVector2 startpoint, CVector2 endpoint, int **grid)
{
    // create a middle point which is exactly halfway between startpoint and endpoint
    CVector2 midpoint((startpoint.GetX() + endpoint.GetX()) / 2, (startpoint.GetY() + endpoint.GetY()) / 2);

    // if the mid-pointis occupied, the path is not clear
    if (is_point_occupied(midpoint.GetX(), midpoint.GetY(), grid))
        return false;

    bool flag = true;
    // transform each point into a grid cell
    int start_x = startpoint.GetX() / resolution;
    int start_y = startpoint.GetY() / resolution;
    int mid_x = midpoint.GetX() / resolution;
    int mid_y = midpoint.GetY() / resolution;
    int end_x = endpoint.GetX() / resolution;
    int end_y = endpoint.GetY() / resolution;

    // if the start point cell and the end point cell are identical, a path exists
    if ((start_x == end_x) && (start_y == end_y))
        return true;

    // if the mid cell is occupied, there is no path
    if (grid[mid_x][mid_y] == 1)
        return false;

    // if the start point and the end point are adjacent, check if they're both free
    // and return accordingly
    if (((start_x + 1 == end_x) || (start_x - 1 == end_x)) && (start_y == end_y)) {
        if ((grid[start_x][start_y] == 0 && (grid[end_x][end_y] == 0)))
            return true;
        return false;
    }
    if ((start_x == end_x) && ((start_y + 1 == end_y) || (start_y - 1 == end_y))) {
        if ((grid[start_x][start_y] == 0 && (grid[end_x][end_y] == 0)))
            return true;
        return false;
    }

    // If you got so far, run the same function on (start, mid) and (mid, end)
    // assuming none of them are identical
    if (((start_x != mid_x) || (start_y != mid_y)) && (flag == true))
        flag = flag && is_path_clear(startpoint, midpoint, grid);
    if (((end_x != mid_x) || (end_y != mid_y)) && (flag == true))
        flag = flag && is_path_clear(midpoint, endpoint, grid);
    return flag;
}
int PRM_controller::source(map<int,vector<float>> new_keys_map,vector<vector<float>>points, int l)
{
    for(auto it = new_keys_map.begin(); it != new_keys_map.end(); ++it){
        if (it->second == points[l])
            return it->first;
    }
    return -1;
}
int PRM_controller::destination(map<int,vector<float>> new_keys_map, KdNodeVector result,int b)
{
    for(auto it = new_keys_map.begin(); it != new_keys_map.end(); ++it){
        if (it->second == result[b].point)
            return it->first;
    }
    return -1;
}
void PRM_controller::print_float_vector(vector<float> const &vec)
{
     for (int i = 0; i < vec.size(); i++) {
        std::cout << vec.at(i) << ' ';
    }
}
void PRM_controller::print_nodes(const KdNodeVector &nodes) {
    size_t i,j;
    for (i = 0; i < nodes.size(); ++i) {
        if (i > 0)
            cout << " ";
        cout << "(";
        for (j = 0; j < nodes[i].point.size(); j++) {
            if (j > 0)
                cout << ",";
            cout << nodes[i].point[j];
        }
        cout << ")";
    }
    cout << endl;
}