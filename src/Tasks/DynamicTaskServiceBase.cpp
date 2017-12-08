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

                mish->setVehicleID(response->getVehicleID());
                mish->setFirstWaypoint(mish->getWaypointList().front()->getNumber());

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

        //don't use ground vehicles dynamically. They are problematic
        auto cast = static_cast<std::shared_ptr<avtas::lmcp::Object>>(entityState);
        if (afrl::vehicles::isGroundVehicleState(cast))
        {
            return;
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
}
}
}
