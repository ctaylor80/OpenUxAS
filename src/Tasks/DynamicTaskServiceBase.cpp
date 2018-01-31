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
#include <afrl/famus/AirVehicleConfiguration.h>

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

        for (auto koz : m_keepOutZones)
        {
            auto poly = BatchSummaryService::FromAbstractGeometry(koz.second->getBoundary());
            m_KeepOutZoneIDVsPolygon[koz.second->getZoneID()] = poly;
        }

        configureDynamicTask(serviceXmlNode);
        return true;
    }

    void DynamicTaskServiceBase::buildTaskPlanOptions()
    {
        int64_t optionId(1);
        int64_t taskId(m_task->getTaskID());

        for (auto itEligibleEntities = m_speedAltitudeVsEligibleEntityIdsRequested.begin(); itEligibleEntities != m_speedAltitudeVsEligibleEntityIdsRequested.end(); itEligibleEntities++)
        {
            for (auto entity : itEligibleEntities->second)
            {
                if (m_entityStates.find(entity) == m_entityStates.end() ||
                    m_entityConfigurations.find(entity) == m_entityConfigurations.end())
                {
                    continue;
                }
                auto state = m_entityStates[entity];
                auto config = m_entityConfigurations[entity];
                auto targetLocation = calculateTargetLocation(state);
                if (afrl::cmasi::isAirVehicleConfiguration(config.get()))
                {
                    auto airVehicleConfig = std::static_pointer_cast<afrl::cmasi::AirVehicleConfiguration>(config);
                    auto radius = loiterRadiusFromConfig(airVehicleConfig);
                    AttemptMoveOutsideKoz(targetLocation, radius * 1.5);
                }
                auto taskOption = new uxas::messages::task::TaskOption;
                taskOption->setTaskID(taskId);
                taskOption->setOptionID(optionId);
                taskOption->getEligibleEntities() = m_task->getEligibleEntities();
                taskOption->setStartLocation(targetLocation->clone());
                taskOption->setEndLocation(targetLocation->clone());
                auto pTaskOption = std::shared_ptr<uxas::messages::task::TaskOption>(taskOption->clone());
                m_optionIdVsTaskOptionClass.insert(std::make_pair(optionId, std::make_shared<TaskOptionClass>(pTaskOption)));
                m_taskPlanOptions->getOptions().push_back(taskOption);

                optionId++;
            }
        }

        std::string compositionString("+(");
        for (auto itOption = m_taskPlanOptions->getOptions().begin(); itOption != m_taskPlanOptions->getOptions().end(); itOption++)
        {
            compositionString += "p";
            compositionString += std::to_string((*itOption)->getOptionID());
            compositionString += " ";
        }
        compositionString += ")";

        m_taskPlanOptions->setComposition(compositionString);

        // send out the options
        auto newResponse = std::static_pointer_cast<avtas::lmcp::Object>(m_taskPlanOptions);
        sendSharedLmcpObjectBroadcastMessage(newResponse);
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
        //check if the target location would intersect a KOZ. Attempt move. Only for Air Vehicles
        if (afrl::cmasi::isAirVehicleState(cast) &&
            m_entityConfigurations.find(entityState->getID()) != m_entityConfigurations.end())
        {

            auto config = m_entityConfigurations[entityState->getID()];
            auto airVehicleConfig = std::static_pointer_cast<afrl::cmasi::AirVehicleConfiguration>(config);
            //use an extra offset to avoid jagged KOZs
            auto bufferMultiplier = 1.5;
            auto loiterRadius = loiterRadiusFromConfig(airVehicleConfig);
            AttemptMoveOutsideKoz(loc, loiterRadius * bufferMultiplier);
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
    std::shared_ptr<afrl::cmasi::VehicleActionCommand> DynamicTaskServiceBase::calculateLoiterAction(const std::shared_ptr<afrl::cmasi::EntityConfiguration>& config, std::shared_ptr<afrl::cmasi::Location3D> loc)
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
            auto airVehicleConfig = std::static_pointer_cast<afrl::cmasi::AirVehicleConfiguration>(config);
            surveyRadius = loiterRadiusFromConfig(airVehicleConfig);
            AttemptMoveOutsideKoz(loc, surveyRadius * 1.5);
        }
        afrl::cmasi::LoiterAction* surveyAction = new afrl::cmasi::LoiterAction;
        surveyAction->setLocation(loc->clone());
        surveyAction->getLocation()->setAltitude(config->getNominalAltitude());
        surveyAction->setAirspeed(surveySpeed);
        surveyAction->setRadius(surveyRadius);
        surveyAction->setDirection(afrl::cmasi::LoiterDirection::CounterClockwise);
        surveyAction->setDuration(-1);
        surveyAction->setLoiterType(surveyType);
        surveyAction->getAssociatedTaskList().push_back(m_task->getTaskID());
        vehicleActionCommand->getVehicleActionList().push_back(surveyAction);
        return vehicleActionCommand;
    }

    double DynamicTaskServiceBase::loiterRadiusFromConfig(std::shared_ptr<afrl::cmasi::AirVehicleConfiguration> config)
    {
        double speed = config->getNominalSpeed();
        double bank = 25.0 * n_Const::c_Convert::dDegreesToRadians();
        // Note: R = V/omega for coordinated turn omega = g*tan(phi)/V
        // Therefore: R = V^2/(g*tan(phi))
        auto radius = speed * speed / (9.80665 * tan(bank));
        // round up to the nearest 100m
        radius = std::ceil(radius / 100.0)*100.0;
        return radius;
    }

    void DynamicTaskServiceBase::AttemptMoveOutsideKoz(std::shared_ptr<afrl::cmasi::Location3D>& loc, double offset)
    {
        uxas::common::utilities::CUnitConversions unitConversions;
        VisiLibity::Point newEnd;
        bool newEndSet = false;
        for (auto koz : m_KeepOutZoneIDVsPolygon)
        {
            VisiLibity::Point p;
            double north, east;
            unitConversions.ConvertLatLong_degToNorthEast_m(loc->getLatitude(), loc->getLongitude(), north, east);
            p.set_x(east);
            p.set_y(north);



            //check for point inside koz case
            if (p.in(*koz.second))
            {
                //move the location outside the koz
                auto bounderyPoint = p.projection_onto_boundary_of(*koz.second);
                auto vector = VisiLibity::Point::normalize(bounderyPoint - p) * offset;
                newEnd = bounderyPoint + vector;
                newEndSet = true;

                break;
            }
            afrl::cmasi::Polygon *poly = new afrl::cmasi::Polygon();
            auto length = offset;
            for (double rad = 0; rad < n_Const::c_Convert::dTwoPi(); rad += n_Const::c_Convert::dPiO10())
            {
                double lat_deg, lon_deg;

                unitConversions.ConvertNorthEast_mToLatLong_deg(north + length * sin(rad), east + length * cos(rad), lat_deg, lon_deg);
                auto loc = new afrl::cmasi::Location3D();
                loc->setLatitude(lat_deg);
                loc->setLongitude(lon_deg);
                poly->getBoundaryPoints().push_back(loc);
            }
            auto loiterArea = BatchSummaryService::FromAbstractGeometry(poly);

            //check if loiter intersects the perimiter of the koz case
            if (loiterArea->n() > 0 && koz.second->n() > 0 &&
                boundary_distance(*loiterArea, *koz.second) < .1)
            {
                //move the location outside the koz
                auto bounderyPoint = p.projection_onto_boundary_of(*koz.second);
                //the loiter center point is outside of the koz because of the checks above
                auto vector = VisiLibity::Point::normalize(p - bounderyPoint) * offset;
                newEnd = bounderyPoint + vector;
                newEndSet = true;

                break;
            }
        }
        if (newEndSet)
        {
            double latitude_deg(0.0);
            double longitude_deg(0.0);
            unitConversions.ConvertNorthEast_mToLatLong_deg(newEnd.y(), newEnd.x(), latitude_deg, longitude_deg);
            loc->setLatitude(latitude_deg);
            loc->setLongitude(longitude_deg);
        }
    }
}
}
}
