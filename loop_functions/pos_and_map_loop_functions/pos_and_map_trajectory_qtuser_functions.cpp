//
// Created by edenha on 24.11.2020.
//

#include "pos_and_map_trajectory_qtuser_functions.h"
#include "pos_and_map_loop_functions.h"

/****************************************/
/****************************************/

CPosAndMapTrajectoryQTUserFunctions::CPosAndMapTrajectoryQTUserFunctions() :
        m_cTrajLF(dynamic_cast<CPosAndMapLoopFunctions&>(CSimulator::GetInstance().GetLoopFunctions())) {
}

/****************************************/
/****************************************/

void CPosAndMapTrajectoryQTUserFunctions::DrawInWorld() {
    /* Go through all the robot waypoints and draw them */
    for(CPosAndMapLoopFunctions::TWaypointMap::const_iterator it = m_cTrajLF.GetWaypoints().begin();
        it != m_cTrajLF.GetWaypoints().end();
        ++it) {
        DrawWaypoints(it->second);
    }
}

/****************************************/
/****************************************/

void CPosAndMapTrajectoryQTUserFunctions::DrawWaypoints(const std::vector<CVector3>& c_waypoints) {
    /* Start drawing segments when you have at least two points */
    if(c_waypoints.size() > 1) {
        size_t unStart = 0;
        size_t unEnd = 1;
        while(unEnd < c_waypoints.size()) {
            DrawRay(CRay3(c_waypoints[unEnd],
                          c_waypoints[unStart]));
            ++unStart;
            ++unEnd;
        }
    }
}

/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CPosAndMapTrajectoryQTUserFunctions, "pos_and_map_trajectory_qtuser_functions")
