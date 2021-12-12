#include <Krembot/controller/krembot_controller.h>
#include <queue>
#include <map>
#include <tuple>

struct MapMsg
{
    int **occupancyGrid;
    Real resolution;
    CVector2 origin;
    int height, width;
};

struct PosMsg
{
    CVector2 pos;
    CDegrees degreeX;
};

class PRM_controller : public KrembotController
{
private:
    Real robotSize = 0.20;
    bool isFirst = true;

public:
    MapMsg mapMsg;
    PosMsg posMsg;
    static map<int,vector<float>> int_to_nodes_map;
    static map<int,vector<float>> nodes_to_int_map;

    ParticleObserver Particle;
    ~PRM_controller() = default;
    void setup();
    void loop();

    // I added the following methods:
    int is_point_occupied(float x, float y, int **grid);
    bool is_path_clear(CVector2 x, CVector2 y, int **grid);
    void write_grid(std::string filename, int **grid, int height, int width);
    void thickening_grid(int **origGrid, int **newGrid, int height, int width, Real resolution);
    void generate_random_point(int width, int height, int **grid, std::pair<float, float> &oPair);
    void fill_milestones_set(std::map<std::pair<float, float>, CVector2> *milestones,
                             int height, int width, int nmilestones, int **grid);
    void write_grid_with_milestones(std::string filename, int **grid, int height, int width);
    void print_float_vector(vector<float> const &vec);
    void print_nodes(const KdNodeVector &nodes);
    int source(map<int,vector<float>> new_keys_map,vector<vector<float>>points, int l);
    int destination(map<int,vector<float>> new_keys_map, KdNodeVector result,int b);
    // from here and on those are methods which were already written

    void Init(TConfigurationNode &t_node) override
    {
        KrembotController::Init(t_node);
        if (!krembot.isInitialized())
        {
            throw std::runtime_error("krembot.ino.cpp: krembot wasn't initialized in controller");
        }
        Particle.setName(krembot.getName());
    }
    void ControlStep() override
    {
        if (isFirst)
        {
            setup();
            isFirst = false;
        }
        loop();
    }
};

REGISTER_CONTROLLER(PRM_controller, "PRM_controller")
