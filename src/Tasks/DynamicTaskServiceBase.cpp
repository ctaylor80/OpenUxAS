// ===============================================================================
// Authors: AFRL/RQQA
// Organization: Air Force Research Laboratory, Aerospace Systems Directorate, Power and Control Division
// 
// Copyright (c) 2017 Government of the United State of America, as represented by
// the Secretary of the Air Force.  No copyright is claimed in the United States under
// Title 17, U.S. Code.  All Other Rights Reserved.
// ===============================================================================

/*
* File:   Task_EscortTask.cpp
* Author: Colin
*/
#include "DynamicTaskServiceBase.h"
#include "afrl/vehicles/GroundVehicleState.h"
#include <afrl/cmasi/GimbalConfiguration.h>
#include <afrl/vehicles/GroundVehicleConfiguration.h>
#include <afrl/vehicles/SurfaceVehicleConfiguration.h>
#include <afrl/cmasi/AirVehicleConfiguration.h>


namespace uxas
{
namespace service
{
namespace task
{
    DynamicTaskServiceBase::DynamicTaskServiceBase(const std::string& typeName, const std::string& directoryName)
        : TaskServiceBase(typeName, directoryName)
    {
        m_isMakeTransitionWaypointsActive = true;
    }


    bool DynamicTaskServiceBase::configureTask(const pugi::xml_node& serviceXmlNode)
    {
        addSubscriptionAddress(messages::route::RoutePlanResponse::Subscription);
        configureDynamicTask(serviceXmlNode);
        return true;
    }

    bool DynamicTaskServiceBase::processReceivedLmcpMessageTask(std::shared_ptr<avtas::lmcp::Object>& receivedLmcpObject)
    {
        if (messages::route::isRoutePlanResponse(receivedLmcpObject))
        {
            auto response = std::dynamic_pointer_cast<messages::route::RoutePlanResponse>(receivedLmcpObject);
            if (response->getResponseID() == m_serviceId &&
                !response->getRouteResponses().empty() &&
                response->getRouteResponses().front()->getRouteID() == TaskOptionClass::m_routeIdFromLastTask &&
                !response->getRouteResponses().front()->getWaypoints().empty())
            {
                auto newRoute = response->getRouteResponses().front()->getWaypoints();

                auto mish = std::make_shared<afrl::cmasi::MissionCommand>();
                for (auto wp : newRoute)
                {
                    wp->getAssociatedTasks().push_back(m_task->getTaskID());
                    mish->getWaypointList().push_back(wp->clone());
                }
                auto back = mish->getWaypointList().back();
                back->setNextWaypoint(back->getNumber());

                m_entityIdVsLastWaypoint[response->getVehicleID()] = back->getNumber();

                mish->setVehicleID(response->getVehicleID());
                mish->setFirstWaypoint(mish->getWaypointList().front()->getNumber());

                //add gimbal and loiter commands
                if (m_entityConfigurations.find(mish->getVehicleID()) != m_entityConfigurations.end() && 
                    m_targetLocations.find(mish->getVehicleID()) != m_targetLocations.end())
                {
                    auto firstWaypoint = mish->getWaypointList().front();
                    auto lastWaypont = mish->getWaypointList().back();

                    auto config = std::shared_ptr<afrl::cmasi::EntityConfiguration>(m_entityConfigurations[mish->getVehicleID()]);
                    auto loc = std::shared_ptr<afrl::cmasi::Location3D>(m_targetLocations[mish->getVehicleID()]);

                    auto gimbalActions = calculateGimbalStareAction(config, loc);
                    auto loiterActions = calculateLoiterAction(config, loc);

                    for (auto gimbalAction : gimbalActions->getVehicleActionList())
                    {
                        firstWaypoint->getVehicleActionList().push_back(gimbalAction->clone());
                    }
                    for (auto loiterAction : loiterActions->getVehicleActionList())
                    {
                        lastWaypont->getVehicleActionList().push_back(loiterAction->clone());
                    }
                }


                processMissionCommand(mish);

                sendSharedLmcpObjectBroadcastMessage(mish);

            }
        }
        else
        {
            processRecievedLmcpMessageDynamicTask(receivedLmcpObject);
        }


        return false;
    }

    void DynamicTaskServiceBase::activeEntityState(const std::shared_ptr<afrl::cmasi::EntityState>& entityState)
    {
        if (m_throttle.find(entityState->getID()) == m_throttle.end())
        {
            m_throttle[entityState->getID()] = uxas::common::utilities::c_TimeUtilities::getTimeNow_ms();
        }

        // throttle the output. 
        if (m_throttle[entityState->getID()] + m_throttle_ms <= uxas::common::utilities::c_TimeUtilities::getTimeNow_ms())
        {
            m_throttle[entityState->getID()] = uxas::common::utilities::c_TimeUtilities::getTimeNow_ms();
        }
        else
        {
            return;
        }

        //ground vehicles are problematic. Wait until they finish a leg and then recalculate
        auto cast = static_cast<std::shared_ptr<avtas::lmcp::Object>>(entityState);
        if (afrl::vehicles::isGroundVehicleState(cast))
        {
            if (m_entityIdVsLastWaypoint.find(entityState->getID()) != m_entityIdVsLastWaypoint.end())
            {
                auto lastWaypoint = m_entityIdVsLastWaypoint[entityState->getID()];
                if (entityState->getCurrentWaypoint() != lastWaypoint)
                {
                    return;
                }
            }
        }

        auto loc = calculateTargetLocation(entityState);
        if (loc == nullptr)
        {
            return;
        }

        m_targetLocations[entityState->getID()] = loc;



        uxas::common::utilities::CUnitConversions flatEarth;
        auto dist = flatEarth.dGetLinearDistance_m_Lat1Long1_deg_To_Lat2Long2_deg(loc->getLatitude(), loc->getLongitude(),
            entityState->getLocation()->getLatitude(), entityState->getLocation()->getLongitude());

        if (dist < m_straightLineThreshold_m)
        {
            return;
        }

        auto request = std::make_shared<messages::route::RoutePlanRequest>();
        request->setVehicleID(entityState->getID());
        request->setAssociatedTaskID(0); //go behind TaskServiceBases' back. The routePlanner should do a direct reply ensuring this is the only instance that gets the response
        request->setRequestID(this->m_serviceId);
        request->setIsCostOnlyRequest(false);
        auto constraint = new messages::route::RouteConstraints;

        constraint->setRouteID(TaskOptionClass::m_routeIdFromLastTask); //include initial waypoint

        //add some lead distance
        double startNorth, startEast, endNorth, endEast;
        flatEarth.ConvertLatLong_degToNorthEast_m(entityState->getLocation()->getLatitude(), entityState->getLocation()->getLongitude(), startEast, startNorth);
        flatEarth.ConvertLatLong_degToNorthEast_m(loc->getLatitude(), loc->getLongitude(), endEast, endNorth);

        auto startPoint = VisiLibity::Point(startNorth, startEast);
        auto endPoint = VisiLibity::Point(endNorth, endEast);

        auto vec = VisiLibity::Point::normalize(endPoint - startPoint) * m_startPointLead_m;

        auto newStart = startPoint + vec;
        auto newStartPoint = std::make_shared<afrl::cmasi::Location3D>();
        flatEarth.ConvertNorthEast_mToLatLong_deg(newStart.y(), newStart.x(), startNorth, startEast);
        newStartPoint->setLatitude(startNorth);
        newStartPoint->setLongitude(startEast);
        newStartPoint->setAltitude(entityState->getLocation()->getAltitude());

        constraint->setStartLocation(newStartPoint->clone());
        constraint->setEndLocation(loc->clone());
        constraint->setUseStartHeading(false);
        constraint->setUseEndHeading(false);

        request->getRouteRequests().push_back(constraint);

        sendSharedLmcpObjectBroadcastMessage(request);
    }

    std::shared_ptr<afrl::cmasi::VehicleActionCommand> DynamicTaskServiceBase::calculateGimbalStareAction(const std::shared_ptr<afrl::cmasi::EntityConfiguration>& config, const std::shared_ptr<afrl::cmasi::Location3D> loc)
    {
        auto vehicleActionCommand = std::make_shared<afrl::cmasi::VehicleActionCommand>();

        for (auto payloadConfig : config->getPayloadConfigurationList())
        {
            if (afrl::cmasi::isGimbalConfiguration(payloadConfig))
            {
                afrl::cmasi::GimbalStareAction* gimbalAction = new afrl::cmasi::GimbalStareAction;
                gimbalAction->setDuration(-1);
                gimbalAction->setPayloadID(payloadConfig->getPayloadID());
                gimbalAction->setStarepoint(loc->clone());
                gimbalAction->getAssociatedTaskList().push_back(m_task->getTaskID());
                vehicleActionCommand->getVehicleActionList().push_back(gimbalAction->clone());
            }
        }
        return vehicleActionCommand;
    }
    std::shared_ptr<afrl::cmasi::VehicleActionCommand> DynamicTaskServiceBase::calculateLoiterAction(const std::shared_ptr<afrl::cmasi::EntityConfiguration>& config, const std::shared_ptr<afrl::cmasi::Location3D> loc)
    {
        auto vehicleActionCommand = std::make_shared<afrl::cmasi::VehicleActionCommand>();

        double surveyRadius = 0.0;
        double surveySpeed = config->getNominalSpeed();
        auto surveyType = afrl::cmasi::LoiterType::Circular;

        // calculate proper radius
        if (afrl::vehicles::isGroundVehicleConfiguration(config.get()) ||
            afrl::vehicles::isSurfaceVehicleConfiguration(config.get()))
        {
            surveyRadius = 0.0;
            surveyType = afrl::cmasi::LoiterType::Hover;
        }
        else if (afrl::cmasi::isAirVehicleConfiguration(config.get()))
        {
            double speed = config->getNominalSpeed();
            double bank = 25.0 * n_Const::c_Convert::dDegreesToRadians();
            // Note: R = V/omega for coordinated turn omega = g*tan(phi)/V
            // Therefore: R = V^2/(g*tan(phi))
            surveyRadius = speed * speed / (9.80665 * tan(bank));
            // round up to the nearest 100m
            surveyRadius = std::ceil(surveyRadius / 100.0)*100.0;
        }
        afrl::cmasi::LoiterAction* surveyAction = new afrl::cmasi::LoiterAction;
        surveyAction->setLocation(loc->clone());
        surveyAction->setAirspeed(surveySpeed);
        surveyAction->setRadius(surveyRadius);
        surveyAction->setDirection(afrl::cmasi::LoiterDirection::CounterClockwise);
        surveyAction->setDuration(-1);
        surveyAction->setLoiterType(surveyType);
        surveyAction->getAssociatedTaskList().push_back(m_task->getTaskID());
        vehicleActionCommand->getVehicleActionList().push_back(surveyAction);
        return vehicleActionCommand;
    }
}
}
}
