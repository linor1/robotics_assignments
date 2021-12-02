//
// Created by edenha on 24.11.2020.
//

#include "pos_and_map_loop_functions.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <Krembot/controller/krembot_controller.h>
//#include <argos3/core/simulator/entity/composable_entity.h>
//#include "Krembot/krembot.h"
#include <controllers/PRM/krembot.ino.cpp>


/*
 * To reduce the number of waypoints stored in memory,
 * consider two robot positions distinct if they are
 * at least MIN_DISTANCE away from each other
 * This constant is expressed in meters
 */
static const Real MIN_DISTANCE = 0.05f;
/* Convenience constant to avoid calculating the square root in PostStep() */
static const Real MIN_DISTANCE_SQUARED = MIN_DISTANCE * MIN_DISTANCE;

/****************************************/
/****************************************/

CPosAndMapLoopFunctions::CPosAndMapLoopFunctions() :
        m_cForagingArenaSideX(-halfX, halfX),
        m_cForagingArenaSideY(-halfY, halfY),
        obstacleRadius(0.005f, 0.05f),
        m_pcFloor(NULL),
        m_pcRNG(NULL){
}

/****************************************/
/****************************************/

void CPosAndMapLoopFunctions::Init(TConfigurationNode& t_node) {
    try {
        TConfigurationNode& tPosAndMap = GetNode(t_node, "pos_and_map");
        /* Get a pointer to the floor entity */
        m_pcFloor = &GetSpace().GetFloorEntity();
        m_pcRNG = CRandom::CreateRNG("argos");
        for(UInt32 i = 0; i < numOfCircularObstacles; ++i)
        {
            obstacles[i].pos = CVector2(m_pcRNG->Uniform(m_cForagingArenaSideX),
                                        m_pcRNG->Uniform(m_cForagingArenaSideY));
            obstacles[i].radious = m_pcRNG->Uniform(obstacleRadius);
        }
        TConfigurationNode* tfloor = m_pcFloor->GetConfigurationNode();
        GetNodeAttribute(*tfloor, "pixels_per_meter", pixels_per_meter);
        width = pixels_per_meter * halfX * 2;
        height = pixels_per_meter * halfY * 2;
        Real fFactor = 1.0f / static_cast<Real>(pixels_per_meter);
        resolution = fFactor;
        const CVector3& cArenaCenter = CSimulator::GetInstance().GetSpace().GetArenaCenter();
        m_cArenaCenter.Set(cArenaCenter.GetX(),
                           cArenaCenter.GetY());
        CVector2 cFloorPos;
        m_cHalfArenaSize.Set(halfX, halfY);

        occupancyGrid = new int*[height];
        for (int y = 0 ; y < height ; y++)
        {
            occupancyGrid[y] = new int [width];
            for ( int x=0; x < width; x++)
            {
                cFloorPos.Set(x * fFactor, y * fFactor);
                cFloorPos -= m_cHalfArenaSize;
                cFloorPos += m_cArenaCenter;
                CColor color = GetFloorColor(cFloorPos);
                if (color == CColor::WHITE)
                {
                    occupancyGrid[y][x] = 0;
                }
                else if (color == CColor::GRAY50)
                {
                    occupancyGrid[y][x] = 1;
                }
            }
        }

        // robot location
        CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");
        for(CSpace::TMapPerType::iterator it = m_cFootbots.begin();
            it != m_cFootbots.end();
            ++it) {

            // fprintf(stderr, "here3\n");

            /* Get handle to foot-bot entity and controller */
            CFootBotEntity& cFootBot = *any_cast<CFootBotEntity*>(it->second);
            CFootBotEntity* pcFB = any_cast<CFootBotEntity*>(it->second);
            CVector2 cPos;
            cPos.Set(cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                     cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
            CQuaternion cQuat = cFootBot.GetEmbodiedEntity().GetOriginAnchor().Orientation;
            CRadians cZAngle, cYAngle, cXAngle;
            CDegrees xDegree;
            cQuat.ToEulerAngles(cZAngle, cYAngle, cXAngle);
            xDegree = ToDegrees(cZAngle);
            LOG << cXAngle << "\n";
            LOG << xDegree << "\n";
            // from pos to ind
            cPos -= m_cArenaCenter;
            cPos += m_cHalfArenaSize;

            /* Create a waypoint vector */
            m_tWaypoints[pcFB] = std::vector<CVector3>();
            /* Add the initial position of the foot-bot */
            m_tWaypoints[pcFB].push_back(pcFB->GetEmbodiedEntity().GetOriginAnchor().Position);
        }



//        m_cOutput.open("pos_and_map/occupancyGrid.txt", std::ios_base::trunc | std::ios_base::out);
//        for (int y = height-1 ; y >=0 ; y--)
//        {
//            for ( int x=0; x < width; x++)
//            {
//                fprintf(stderr,"%d", occupancyGrid[y][x]);
//                m_cOutput << occupancyGrid[y][x];
//            }
//            fprintf(stderr,"\n");
//            m_cOutput << std::endl;
//        }
//
//        /* Close the file */
//        m_cOutput.close();


    }
    catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error parsing loop functions!", ex);
    }
}

/****************************************/
/****************************************/

void CPosAndMapLoopFunctions::Reset() {

}

/****************************************/
/****************************************/

void CPosAndMapLoopFunctions::Destroy() {
}

/****************************************/
/****************************************/

CColor CPosAndMapLoopFunctions::GetFloorColor(const CVector2& c_position_on_plane) {
    if ((c_position_on_plane.GetX() < m_cForagingArenaSideX.GetMin())||
        (c_position_on_plane.GetX() > m_cForagingArenaSideX.GetMax()) ||
        (c_position_on_plane.GetY() < m_cForagingArenaSideY.GetMin())||
        (c_position_on_plane.GetY() > m_cForagingArenaSideY.GetMax()))
    {
        return CColor::BLACK;
    }
    for(UInt32 i = 0; i < numOfCircularObstacles; ++i) {
        if((c_position_on_plane - obstacles[i].pos).SquareLength() < obstacles[i].radious) {
            return CColor::GRAY50;
        }
    }
    return CColor::WHITE;
}

/****************************************/
/****************************************/

void CPosAndMapLoopFunctions::PreStep() {
    /* Logic to pick and drop food items */
    /*
     * If a robot is in the nest, drop the food item
     * If a robot is on a food item, pick it
     * Each robot can carry only one food item per time
     *
    /* Check whether a robot is on a food item */
    CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");

    for(CSpace::TMapPerType::iterator it = m_cFootbots.begin();
        it != m_cFootbots.end();
        ++it) {

        // fprintf(stderr, "here3\n");

        /* Get handle to foot-bot entity and controller */
        CFootBotEntity& cFootBot = *any_cast<CFootBotEntity*>(it->second);
        PRM_controller& cController = dynamic_cast<PRM_controller&>(cFootBot.GetControllableEntity().GetController());

        /* Get the position of the foot-bot on the ground as a CVector2 */
        if(isFirst)
        {
            cController.mapMsg.occupancyGrid = occupancyGrid;
            cController.mapMsg.resolution = resolution;
            cController.mapMsg.height = height;
            cController.mapMsg.width = width;
            cController.mapMsg.origin.Set(-m_cHalfArenaSize.GetX(),-m_cHalfArenaSize.GetY());
        }

        CVector2 cPos;
        cPos.Set(cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                 cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
        Real orientation = cFootBot.GetEmbodiedEntity().GetOriginAnchor().Orientation.GetZ();
//      LOG << orientation << "\n";
        // from pos to ind
        cController.posMsg.pos = cPos;
        CQuaternion cQuat = cFootBot.GetEmbodiedEntity().GetOriginAnchor().Orientation;
        CRadians cZAngle, cYAngle, cXAngle;
        CDegrees xDegree;
        cQuat.ToEulerAngles(cZAngle, cYAngle, cXAngle);
        xDegree = ToDegrees(cZAngle);
        cController.posMsg.degreeX = xDegree;

        // I should calc the opposite

    }

    // if the robot is on an obstacle or out of borders

}

void CPosAndMapLoopFunctions::PostStep() {
    /* Get the map of all foot-bots from the space */
    CSpace::TMapPerType& tFBMap = GetSpace().GetEntitiesByType("foot-bot");
    /* Go through them */
    for(CSpace::TMapPerType::iterator it = tFBMap.begin();
        it != tFBMap.end();
        ++it) {
        /* Create a pointer to the current foot-bot */
        CFootBotEntity* pcFB = any_cast<CFootBotEntity*>(it->second);
        /* Add the current position of the foot-bot if it's sufficiently far from the last */
        if(SquareDistance(pcFB->GetEmbodiedEntity().GetOriginAnchor().Position,
                          m_tWaypoints[pcFB].back()) > MIN_DISTANCE_SQUARED) {
            m_tWaypoints[pcFB].push_back(pcFB->GetEmbodiedEntity().GetOriginAnchor().Position);
        }
    }
}


/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CPosAndMapLoopFunctions, "pos_and_map_loop_functions")
