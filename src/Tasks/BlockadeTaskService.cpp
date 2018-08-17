// ===============================================================================
// Authors: AFRL/RQQA
// Organization: Air Force Research Laboratory, Aerospace Systems Directorate, Power and Control Division
// 
// Copyright (c) 2017 Government of the United State of America, as represented by
// the Secretary of the Air Force.  No copyright is claimed in the United States under
// Title 17, U.S. Code.  All Other Rights Reserved.
// ===============================================================================

/* 
 * File:   Task_BlockadeTask.cpp
 * Author: Derek & Steve
 * 
 * Created on February 26, 2016, 6:17 PM
 */


#include "BlockadeTaskService.h"

#include "Position.h"
#include "UnitConversions.h"

#include "avtas/lmcp/LmcpXMLReader.h"
#include "afrl/cmasi/VehicleActionCommand.h"
#include "afrl/cmasi/GimbalStareAction.h"
#include "afrl/cmasi/LoiterAction.h"
#include "afrl/cmasi/MissionCommand.h"
#include "afrl/cmasi/GimbalConfiguration.h"
#include "afrl/cmasi/AirVehicleConfiguration.h"
#include "afrl/vehicles/GroundVehicleConfiguration.h"
#include "afrl/vehicles/SurfaceVehicleConfiguration.h"
#include "uxas/messages/task/TaskImplementationResponse.h"
#include "uxas/messages/task/TaskOption.h"
#include "uxas/messages/route/RouteRequest.h"
#include "uxas/messages/route/RouteResponse.h"
#include "uxas/messages/route/RouteConstraints.h"

#include "pugixml.hpp"
#include "Constants/Convert.h"
#include "DpssDataTypes.h"

#include <sstream>      //std::stringstream
#include <iostream>     // std::cout, cerr, etc


#define STRING_XML_ENTITY_STATES "EntityStates" //TODO:: define this in some global place

#define COUT_FILE_LINE_MSG(MESSAGE) std::cout << "CMPS-CMPS-CMPS-CMPS:: BlockadeTask:" << __FILE__ << ":" << __LINE__ << ":" << MESSAGE << std::endl;std::cout.flush();
#define CERR_FILE_LINE_MSG(MESSAGE) std::cerr << "CMPS-CMPS-CMPS-CMPS:: BlockadeTask:" << __FILE__ << ":" << __LINE__ << ":" << MESSAGE << std::endl;std::cerr.flush();


namespace uxas
{
namespace service
{
namespace task
{
BlockadeTaskService::ServiceBase::CreationRegistrar<BlockadeTaskService>
BlockadeTaskService::s_registrar(BlockadeTaskService::s_registryServiceTypeNames());

BlockadeTaskService::BlockadeTaskService()
: DynamicTaskServiceBase(BlockadeTaskService::s_typeName(), BlockadeTaskService::s_directoryName())
{
    m_taskCompletes = false;
    m_straightLineThreshold_m = 10;
    m_isMultiVehicle = true;
};

BlockadeTaskService::~BlockadeTaskService()
{
};

bool
BlockadeTaskService::configureDynamicTask(const pugi::xml_node& ndComponent)

{
    std::string strBasePath = m_workDirectoryPath;
    std::stringstream sstrErrors;


    bool isSuccessful(true);

    if (isSuccessful)
    {
        if (afrl::impact::isBlockadeTask(m_task.get()))
        {
            m_blockadeTask = std::static_pointer_cast<afrl::impact::BlockadeTask>(m_task);
            if (!m_blockadeTask)
            {
                sstrErrors << "ERROR:: **BlockadeTaskService::bConfigure failed to cast a BlockadeTask from the task pointer." << std::endl;
                CERR_FILE_LINE_MSG(sstrErrors.str())
                isSuccessful = false;
            }
        }
        else
        {
            sstrErrors << "ERROR:: **BlockadeTaskService::bConfigure failed: taskObject[" << m_task->getFullLmcpTypeName() << "] is not a BlockadeTask." << std::endl;
            CERR_FILE_LINE_MSG(sstrErrors.str())
            isSuccessful = false;
        }
    } //isSuccessful
    if (isSuccessful)
    {
        if (m_entityStates.find(m_blockadeTask->getBlockedEntityID()) != m_entityStates.end())
        {
            m_blockedEntityStateLast = m_entityStates[m_blockadeTask->getBlockedEntityID()];
        }
    } //if(isSuccessful)
    return (isSuccessful);
}

bool BlockadeTaskService::processRecievedLmcpMessageDynamicTask(std::shared_ptr<avtas::lmcp::Object>& receivedLmcpObject)
//example: if (afrl::cmasi::isServiceStatus(receivedLmcpMessage->m_object.get()))
{
    std::stringstream sstrError;

    auto entityState = std::dynamic_pointer_cast<afrl::cmasi::EntityState>(receivedLmcpObject);
    if (entityState)
    {
        if (entityState->getID() == m_blockadeTask->getBlockedEntityID())
        {
            m_blockedEntityStateLast = entityState;
        }
        m_entityStates[entityState->getID()] = entityState;
    }
    return (false); // always false implies never terminating service from here
};


std::shared_ptr<afrl::cmasi::Location3D> BlockadeTaskService::calculateTargetLocation(const std::shared_ptr<afrl::cmasi::EntityState> entityState)
{
    // we are at an active waypoint
    if (m_blockedEntityStateLast)
    {
        // fill in neighbor set
        std::vector< int64_t > neighborIds;
        for (auto activeEntity : m_activeEntities)
        {
            neighborIds.push_back(activeEntity);
        }

        // look up speed to use for commanding vehicle
        double speed = entityState->getGroundspeed();
        if (m_entityConfigurations.find(entityState->getID()) != m_entityConfigurations.end())
        {
            speed = m_entityConfigurations[entityState->getID()]->getNominalSpeed();
        }

        auto stareLocation = std::shared_ptr<afrl::cmasi::Location3D>(m_blockedEntityStateLast->getLocation()->clone());
        auto targetLocation = std::shared_ptr<afrl::cmasi::Location3D>(m_blockedEntityStateLast->getLocation()->clone());
        double enemyHeading = CalculateCenterBlockingPosition(entityState->getLocation(), targetLocation, m_blockadeTask);

        // calculate blocking assignment on wall (ordered by ID)
        size_t placement = 0;
        for (auto neighborId : neighborIds)
        {
            if (neighborId > entityState->getID())
            {
                placement++;
            }
        }

        double offsetDist = m_blockadeTask->getStandoffDistance() * (placement - neighborIds.size() / 2.0);
        double theta = n_Const::c_Convert::dPiO2() - (enemyHeading + 90.0) * n_Const::c_Convert::dDegreesToRadians();
        uxas::common::utilities::CUnitConversions flatEarth;
        double north, east;
        flatEarth.ConvertLatLong_degToNorthEast_m(targetLocation->getLatitude(), targetLocation->getLongitude(), north, east);
        north += sin(theta) * offsetDist;
        east += cos(theta) * offsetDist;

        double lat, lon;
        flatEarth.ConvertNorthEast_mToLatLong_deg(north, east, lat, lon);

        auto loc = std::make_shared<afrl::cmasi::Location3D>();
        loc->setLatitude(lat);
        loc->setLongitude(lon);
        loc->setAltitude(entityState->getLocation()->getAltitude());
        loc->setAltitudeType(entityState->getLocation()->getAltitudeType());

        return loc;
    }
    else
    {
        CERR_FILE_LINE_MSG("ERROR::Task_BlockadeTask:: no watchedEntityState found for Entity[" << m_blockadeTask->getBlockedEntityID() << "]")
            return std::shared_ptr<afrl::cmasi::Location3D>(entityState->getLocation()->clone());
    }
}
void BlockadeTaskService::processMissionCommand(std::shared_ptr<afrl::cmasi::MissionCommand> miss)
{

    //insert an extra wp for AMASEt
    if (miss->getWaypointList().empty())
        return;
    auto lastWp = miss->getWaypointList().back();

    auto newLastWp = lastWp->clone();
    newLastWp->setNumber(lastWp->getNumber() + 1);
    newLastWp->setNextWaypoint(lastWp->getNumber() + 1);
    lastWp->setNextWaypoint(newLastWp->getNumber());
    lastWp->getVehicleActionList().clear();

    miss->getWaypointList().push_back(newLastWp);
}


std::shared_ptr<afrl::cmasi::VehicleActionCommand> BlockadeTaskService::CalculateGimbalActions(const std::shared_ptr<afrl::cmasi::EntityState>& entityState, double lat, double lon)
{
    std::shared_ptr<afrl::cmasi::VehicleActionCommand> caction(new afrl::cmasi::VehicleActionCommand);

    double surveyRadius = 100.0; // default 100 meters, circular
    double surveySpeed = entityState->getGroundspeed();
    auto surveyType = afrl::cmasi::LoiterType::Circular;
    std::vector<int64_t> gimbalId;

    if (m_entityConfigurations.find(entityState->getID()) != m_entityConfigurations.end())
    {
        surveySpeed = m_entityConfigurations[entityState->getID()]->getNominalSpeed();
        // find all gimbals to steer
        for (size_t a = 0; a < m_entityConfigurations[entityState->getID()]->getPayloadConfigurationList().size(); a++)
        {
            auto payload = m_entityConfigurations[entityState->getID()]->getPayloadConfigurationList().at(a);
            if (afrl::cmasi::isGimbalConfiguration(payload))
            {
                gimbalId.push_back(payload->getPayloadID());
            }
        }

        // calculate proper radius
        if (afrl::vehicles::isGroundVehicleConfiguration(m_entityConfigurations[entityState->getID()].get()) ||
                afrl::vehicles::isSurfaceVehicleConfiguration(m_entityConfigurations[entityState->getID()].get()))
        {
            surveyRadius = 0.0;
            surveyType = afrl::cmasi::LoiterType::Hover;
        }
        else if (afrl::cmasi::isAirVehicleConfiguration(m_entityConfigurations[entityState->getID()].get()))
        {
            double speed = m_entityConfigurations[entityState->getID()]->getNominalSpeed();
            double bank = 25.0 * n_Const::c_Convert::dDegreesToRadians();
            // Note: R = V/omega for coordinated turn omega = g*tan(phi)/V
            // Therefore: R = V^2/(g*tan(phi))
            surveyRadius = speed * speed / (9.80665 * tan(bank));
            // round up to the nearest 100m
            surveyRadius = std::ceil(surveyRadius / 100.0)*100.0;

            // TODO: sometimes the loiter radius seems to change size, so hard-code
            surveyRadius = 300.0;
        }
    }

    afrl::cmasi::LoiterAction* surveyAction = new afrl::cmasi::LoiterAction;
    surveyAction->setLocation(new afrl::cmasi::Location3D());
    surveyAction->getLocation()->setLatitude(lat);
    surveyAction->getLocation()->setLongitude(lon);
    surveyAction->getLocation()->setAltitude(entityState->getLocation()->getAltitude());
    surveyAction->getLocation()->setAltitudeType(entityState->getLocation()->getAltitudeType());
    surveyAction->setAirspeed(surveySpeed);
    surveyAction->setRadius(surveyRadius);
    surveyAction->setDirection(afrl::cmasi::LoiterDirection::CounterClockwise);
    surveyAction->setDuration(-1);
    surveyAction->setLoiterType(surveyType);
    surveyAction->getAssociatedTaskList().push_back(m_task->getTaskID());
    caction->getVehicleActionList().push_back(surveyAction);

    // steer all gimbals
    for (size_t g = 0; g < gimbalId.size(); g++)
    {
        afrl::cmasi::GimbalStareAction* gimbalAction = new afrl::cmasi::GimbalStareAction;
        gimbalAction->setDuration(-1);
        gimbalAction->setPayloadID(gimbalId.at(g));
        gimbalAction->setStarepoint(m_blockedEntityStateLast->getLocation()->clone());
        gimbalAction->getAssociatedTaskList().push_back(m_task->getTaskID());
        caction->getVehicleActionList().push_back(gimbalAction->clone());
    }

    return caction;
}

double BlockadeTaskService::CalculateCenterBlockingPosition(afrl::cmasi::Location3D* vloc, std::shared_ptr<afrl::cmasi::Location3D>& targetLocation, std::shared_ptr<afrl::impact::BlockadeTask>& task)
{
    double enemyDirection = -90.0; // default assume heading west

    // linearize enemy and base locations
    uxas::common::utilities::CUnitConversions flatEarth;
    double north, east;

    if (task->getProtectedLocation() == nullptr)
    {
        flatEarth.ConvertLatLong_degToNorthEast_m(targetLocation->getLatitude(), targetLocation->getLongitude(), north, east);
        east -= 100.0; // assume heading west
    }
    else
    {
        flatEarth.ConvertLatLong_degToNorthEast_m(task->getProtectedLocation()->getLatitude(), task->getProtectedLocation()->getLongitude(), north, east);
    }
    Dpss_Data_n::xyPoint base(east, north);

    flatEarth.ConvertLatLong_degToNorthEast_m(targetLocation->getLatitude(), targetLocation->getLongitude(), north, east);
    Dpss_Data_n::xyPoint enemy(east, north);
    enemy -= base; // base is considered origin of coordinate transformation

    // calculate enemy likely direction of travel (towards base)
    // re-orient to standard north == 0 angle and with units of degrees
    enemyDirection = (n_Const::c_Convert::dPiO2() - atan2(enemy.y, enemy.x)) * n_Const::c_Convert::dRadiansToDegrees();

    double blockDist = 0.0;
    if (vloc == nullptr)
    {
        // just go 3/4 of the way
        blockDist = 0.75;
    }
    else if (enemy.len() > task->getStandoffDistance())
    {
        flatEarth.ConvertLatLong_degToNorthEast_m(vloc->getLatitude(), vloc->getLongitude(), north, east);
        Dpss_Data_n::xyPoint veh(east, north);
        veh -= base; // origin is base

        // rotate so that approach from enemy to base is along the x-axis
        double r = atan2(enemy.y, enemy.x);
        enemy.Rotate(-r);
        veh.Rotate(-r);

        // try to intercept 'standoff-distance' in front of enemy
        enemy.x -= task->getStandoffDistance();

        Dpss_Data_n::Segment q(enemy, veh);

        if (veh.len() > enemy.x)
        {
            blockDist = 1.0; // Still go towards enemy. TODO: enemy is closer to base, just make beeline for base
        }
        else if (q.len() < 10.0)
        {
            // already with in 10 meters of enemy location
            blockDist = 1.0;
        }
        else
        {
            // calculate intercept assuming both travel at same speed
            double delta = fabs(q.angle());
            // note: guaranteed that delta \in (pi/2,pi]
            //  veh is closer to base, so must be closer to origin,
            //  so angle from x-axis must be greater than pi/2
            double beta = n_Const::c_Convert::dPi() - 2 * delta; // beta \in [-pi,0)
            double xtra = fabs(veh.y);
            if (fabs(beta + n_Const::c_Convert::dPiO2()) < 1e-4)
            {
                // special case of right triangle
                xtra = 0.0;
            }
            else
            {
                double tbeta = tan(beta);
                if (fabs(tbeta) > 1e-4)
                {
                    xtra /= tan(beta);
                }
                else
                {
                    // tan(beta) near zero so vehicle is on
                    // approach line already --> meet halfway
                    xtra = (enemy.x - veh.x) / 2.0;
                }
            }
            double ell = xtra + veh.x;
            if (ell < 0.0)
            {
                ell = 0.0;
            }
            blockDist = ell / enemy.x; // normalized
        }

        // undo rotation to return to 'base'-centered coordinate system
        enemy.Rotate(r);
    }

    // move to intercept point
    enemy *= blockDist;
    enemy += base; // move back from 'base'-centered coordinate system to general

    // back to lat/lon
    double lat, lon;
    flatEarth.ConvertNorthEast_mToLatLong_deg(enemy.y, enemy.x, lat, lon);
    targetLocation->setLatitude(lat);
    targetLocation->setLongitude(lon);

    return enemyDirection;
}


}; //namespace task
}; //namespace service
}; //namespace uxas
