//
// Created by edenha on 24.11.2020.
//

#ifndef KREMBOT_WS_POS_AND_MAP_LOOP_FUNCTIONS_H
#define KREMBOT_WS_POS_AND_MAP_LOOP_FUNCTIONS_H


#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/simulator/entity/floor_entity.h>
#include <argos3/core/utility/math/range.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>

#define numOfCircularObstacles 6
#define halfX 2.5f
#define halfY  2.5f

using namespace argos;

struct circularObstacle {
    CVector2 pos;
    Real radious;
};

class CPosAndMapLoopFunctions : public CLoopFunctions {

public:

    CPosAndMapLoopFunctions();
    virtual ~CPosAndMapLoopFunctions() {}

    virtual void Init(TConfigurationNode& t_tree);
    virtual void Reset();
    virtual void Destroy();
    virtual CColor GetFloorColor(const CVector2& c_position_on_plane);
    virtual void PreStep();

    typedef std::map<CFootBotEntity*, std::vector<CVector3> > TWaypointMap;
    TWaypointMap m_tWaypoints;

    virtual void PostStep();
    inline const TWaypointMap& GetWaypoints() const {
        return m_tWaypoints;
    }

private:
    CRange<Real> m_cForagingArenaSideX, m_cForagingArenaSideY;
    CRange<Real> obstacleRadius;
    CFloorEntity* m_pcFloor;
    CRandom::CRNG* m_pcRNG;

    bool isFirst = true;

    circularObstacle obstacles[numOfCircularObstacles];
    int ** occupancyGrid;
    UInt32 pixels_per_meter;
    int height, width;
    Real resolution;

    CVector2 pos_ind_small;

    CVector2 m_cArenaCenter, m_cHalfArenaSize;


    std::ofstream m_cOutput;
};

#endif //KREMBOT_WS_POS_AND_MAP_LOOP_FUNCTIONS_H
