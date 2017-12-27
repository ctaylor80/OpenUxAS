// ===============================================================================
// Authors: AFRL/RQQA
// Organization: Air Force Research Laboratory, Aerospace Systems Directorate, Power and Control Division
// 
// Copyright (c) 2017 Government of the United State of America, as represented by
// the Secretary of the Air Force.  No copyright is claimed in the United States under
// Title 17, U.S. Code.  All Other Rights Reserved.
// ===============================================================================

/* 
 * File:   00_PayloadDropTask.cpp
 * Author: steve
 *
 * Created on March 22, 2017, 5:55 PM
 *
 * <Service Type="PayloadDropTask" OptionString="Option_01" OptionInt="36" />
 * 
 */

// include header for this service
#include "PayloadDropTaskService.h"

//include for KeyValuePair LMCP Message
#include "afrl/cmasi/KeyValuePair.h"

#include <iostream>     // std::cout, cerr, etc
#include <afrl/ttcp/Affiliation.h>
#include <afrl/ttcp/PayloadDrop.h>
#include <afrl/cmasi/AirVehicleConfiguration.h>
#include <afrl/cmasi/LoiterAction.h>
#include <UnitConversions.h>
#include <afrl/cmasi/GimbalStareAction.h>
#include <afrl/cmasi/GimbalConfiguration.h>

// convenience definitions for the option strings
#define STRING_XML_OPTION_STRING "OptionString"
#define STRING_XML_OPTION_INT "OptionInt"

// namespace definitions
namespace uxas  // uxas::
{
namespace service   // uxas::service::
{
namespace task
{

// this entry registers the service in the service creation registry
PayloadDropTaskService::ServiceBase::CreationRegistrar<PayloadDropTaskService>
PayloadDropTaskService::s_registrar(PayloadDropTaskService::s_registryServiceTypeNames());

// service constructor
PayloadDropTaskService::PayloadDropTaskService()
: TaskServiceBase(PayloadDropTaskService::s_typeName(), PayloadDropTaskService::s_directoryName()) { };

PayloadDropTaskService::~PayloadDropTaskService()
{
    //UXAS_LOG_INFORM_ASSIGNMENT(s_typeName(), "::~PayloadDropTask()");
};


bool
PayloadDropTaskService::configureTask(const pugi::xml_node& ndComponent)

{
    std::string strBasePath = m_workDirectoryPath;
    std::stringstream sstrErrors;

    bool isSuccessful(true);

    if (isSuccessful)
    {
        if (afrl::ttcp::isPayloadDrop(m_task.get()))
        {
            m_payloadDrop = std::static_pointer_cast<afrl::ttcp::PayloadDrop>(m_task);
            if (!m_payloadDrop)
            {
                /*sstrErrors << "ERROR:: **ImpactPointSearchTaskService::bConfigure failed to cast a ImpactPointSearchTask from the task pointer." << std::endl;
                CERR_FILE_LINE_MSG(sstrErrors.str())
                    isSuccessful = false;*/
            }
        }
        else
        {
            //sstrErrors << "ERROR:: **ImpactPointSearchTaskService::bConfigure failed: taskObject[" << m_task->getFullLmcpTypeName() << "] is not a ImpactPointSearchTask." << std::endl;
            //CERR_FILE_LINE_MSG(sstrErrors.str())
                isSuccessful = false;
        }
    } //isSuccessful
    return (isSuccessful);

}

bool task::PayloadDropTaskService::initializeTask()
{
    // perform any required initialization before the service is started
    //std::cout << "*** INITIALIZING:: Service[" << s_typeName() << "] Service Id[" << m_serviceId << "] with working directory [" << m_workDirectoryName << "] *** " << std::endl;
    
    return (true);
}

bool task::PayloadDropTaskService::startTask()
{
    // perform any actions required at the time the service starts
    //std::cout << "*** STARTING:: Service[" << s_typeName() << "] Service Id[" << m_serviceId << "] with working directory [" << m_workDirectoryName << "] *** " << std::endl;
    
    return (true);
};

bool task::PayloadDropTaskService::terminateTask()
{    
    return (true);
}

bool task::PayloadDropTaskService::processReceivedLmcpMessageTask(std::shared_ptr<avtas::lmcp::Object>& receivedLmcpObject)
{
    return false;
}

void PayloadDropTaskService::buildTaskPlanOptions()
{
    double wedgeDirectionIncrement(n_Const::c_Convert::dPiO8());

    auto optionId = 1;
    //approach from any angle
    double dHeadingCurrent_rad = 0.0;
    double dHeadingTarget_rad = n_Const::c_Convert::dTwoPi() - wedgeDirectionIncrement;
    double runway_m = 200;
    for (auto itEligibleEntities : m_speedAltitudeVsEligibleEntityIdsRequested)
    {
        for (auto entityId : itEligibleEntities.second)
        {
            while (n_Const::c_Convert::bCompareDouble(dHeadingTarget_rad, dHeadingCurrent_rad, n_Const::c_Convert::enGreaterEqual))
            {
                auto taskOption = new uxas::messages::task::TaskOption;
                auto startEndHeading_deg = n_Const::c_Convert::dNormalizeAngleRad((dHeadingCurrent_rad + n_Const::c_Convert::dPi()), 0.0) * n_Const::c_Convert::dRadiansToDegrees(); // [0,2PI) 
                taskOption->setStartHeading(startEndHeading_deg);
                taskOption->setEndHeading(startEndHeading_deg);

                taskOption->setTaskID(m_payloadDrop->getTaskID());
                taskOption->setOptionID(optionId);

                double north_m(0.0);
                double east_m(0.0);
                double newLat;
                double newLon;
                uxas::common::utilities::CUnitConversions unitConversions;

                auto runwayStart = m_payloadDrop->getDropLocation()->clone();
                unitConversions.ConvertLatLong_degToNorthEast_m(runwayStart->getLatitude(), runwayStart->getLongitude(), north_m, east_m);
                north_m += cos(dHeadingCurrent_rad) * runway_m;
                east_m += sin(dHeadingCurrent_rad) * runway_m;
                unitConversions.ConvertNorthEast_mToLatLong_deg(north_m, east_m, newLat, newLon);
                runwayStart->setLatitude(newLat);
                runwayStart->setLongitude(newLon);

                taskOption->setStartLocation(runwayStart->clone());

                taskOption->setEndLocation(runwayStart->clone());

                auto pTaskOption = std::shared_ptr<uxas::messages::task::TaskOption>(taskOption->clone());
                auto taskOptionClass = std::make_shared<TaskOptionClass>(pTaskOption);

                //add a routePlanRequest
                auto routePlanRequest = std::make_shared<messages::route::RoutePlanRequest>();
                routePlanRequest->setRequestID(getOptionIdFromRouteId(optionId));
                routePlanRequest->setAssociatedTaskID(m_task->getTaskID());
                routePlanRequest->setIsCostOnlyRequest(true);
                //TODO
                //routePlanRequest->setOperatingRegion(currentAutomationRequest->getOriginalRequest()->getOperatingRegion()) 
                routePlanRequest->setVehicleID(entityId);

                auto entryConstraint = std::make_shared<messages::route::RouteConstraints>();
                entryConstraint->setStartLocation(runwayStart->clone());
                entryConstraint->setStartHeading(startEndHeading_deg);
                entryConstraint->setEndLocation(m_payloadDrop->getDropLocation()->clone());
                entryConstraint->setEndHeading(startEndHeading_deg);

                routePlanRequest->getRouteRequests().push_back(entryConstraint->clone());

                taskOptionClass->m_routePlanRequest = routePlanRequest;

                    m_optionIdVsTaskOptionClass.insert(std::make_pair(optionId, taskOptionClass));
                m_taskPlanOptions->getOptions().push_back(taskOption);
                taskOption = nullptr; //just gave up ownership




                optionId++;
                dHeadingCurrent_rad += wedgeDirectionIncrement;
            }
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


bool PayloadDropTaskService::isProcessTaskImplementationRouteResponse(std::shared_ptr<uxas::messages::task::TaskImplementationResponse>& taskImplementationResponse, std::shared_ptr<TaskOptionClass>& taskOptionClass,
    int64_t& waypointId, std::shared_ptr<uxas::messages::route::RoutePlan>& route)
{
    auto config = m_entityConfigurations.find(taskImplementationResponse->getVehicleID());
    if (config == m_entityConfigurations.end())
    {
        return false;
    }

    //only use for air vehicles
    auto cast = static_cast<std::shared_ptr<avtas::lmcp::Object>>(config->second);
    if (!afrl::cmasi::isAirVehicleConfiguration(cast))
    {
        return false;
    }
    auto airVehicleConfig = std::dynamic_pointer_cast<afrl::cmasi::AirVehicleConfiguration>(cast);

    if (taskImplementationResponse->getTaskWaypoints().empty())
    {
        return false;
    }

    auto back = taskImplementationResponse->getTaskWaypoints().back();

    //clone the last waypoint to make sure the loiter happens before TaskComplete
    auto clone = back->clone();
    clone->setNumber(back->getNumber() + 1);
    back->setNextWaypoint(clone->getNumber());
    back->getAssociatedTasks().push_back(m_payloadDrop->getTaskID());
    taskImplementationResponse->getTaskWaypoints().push_back(clone);

    // compute turn radius for a "level turn" r = V^2/(g*tan(phi_max))
    double nominalMaxBankAngle_rad = n_Const::c_Convert::toRadians(airVehicleConfig->getNominalFlightProfile()->getMaxBankAngle());
    double nominalSpeed_mps = airVehicleConfig->getNominalSpeed();
    double dTanMaxBankAngle = tan(nominalMaxBankAngle_rad);
    double turnRadius_m = (dTanMaxBankAngle <= 0.0) ? (0.0) : (pow((nominalSpeed_mps), 2) / (n_Const::c_Convert::dGravity_mps2() * dTanMaxBankAngle));

    //make a loiter action for two rotation. We do this by estimating the time it will take
    auto loiterRadius = turnRadius_m * 2.0; //buffer
    auto duration = (2 * n_Const::c_Convert::dTwoPi() * loiterRadius) / airVehicleConfig->getNominalSpeed() * 1000;
    auto loiter = new afrl::cmasi::LoiterAction();
    loiter->setLocation(m_payloadDrop->getDropLocation()->clone());
    loiter->setAirspeed(airVehicleConfig->getNominalSpeed());
    loiter->setRadius(loiterRadius);
    loiter->setLoiterType(afrl::cmasi::LoiterType::Circular);
    loiter->getAssociatedTaskList().push_back(m_payloadDrop->getTaskID());
    loiter->setDuration(duration); //two rotations
    back->getVehicleActionList().push_back(loiter);

    for (auto payload : config->second->getPayloadConfigurationList())
    {
        if (afrl::cmasi::isGimbalConfiguration(payload))
        {
            auto stare = new afrl::cmasi::GimbalStareAction();
            stare->setDuration(duration);
            auto groundPoint = m_payloadDrop->getDropLocation()->clone();
            groundPoint->setAltitude(0);
            stare->setStarepoint(groundPoint);
            stare->setPayloadID(payload->getPayloadID());
            stare->getAssociatedTaskList().push_back(m_task->getTaskID());
            back->getVehicleActionList().push_back(stare);
        }
    }
    return false; //base class builds response
}


}; //namespace task
}; //namespace service
}; //namespace uxas
