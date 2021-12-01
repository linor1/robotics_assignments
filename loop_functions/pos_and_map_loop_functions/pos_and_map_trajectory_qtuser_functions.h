//
// Created by edenha on 24.11.2020.
//

#ifndef KREMBOT_WS_POS_AND_MAP_TRAJECTORY_QTUSER_FUNCTIONS_H
#define KREMBOT_WS_POS_AND_MAP_TRAJECTORY_QTUSER_FUNCTIONS_H


#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>

using namespace argos;

class CPosAndMapLoopFunctions;

class CPosAndMapTrajectoryQTUserFunctions : public CQTOpenGLUserFunctions {

public:

    CPosAndMapTrajectoryQTUserFunctions();

    virtual ~CPosAndMapTrajectoryQTUserFunctions() {}

    virtual void DrawInWorld();

private:

    void DrawWaypoints(const std::vector<CVector3>& c_waypoints);

private:

    CPosAndMapLoopFunctions& m_cTrajLF;

};

#endif //KREMBOT_WS_POS_AND_MAP_TRAJECTORY_QTUSER_FUNCTIONS_H
