// ===============================================================================
// Authors: AFRL/RQQA
// Organization: Air Force Research Laboratory, Aerospace Systems Directorate, Power and Control Division
// 
// Copyright (c) 2017 Government of the United State of America, as represented by
// the Secretary of the Air Force.  No copyright is claimed in the United States under
// Title 17, U.S. Code.  All Other Rights Reserved.
// ===============================================================================

/* 
 * File:   Task_WatchTask.cpp
 * Author: steve
 * 
 * Created on February 24, 2016, 6:17 PM
 */


#include "OverwatchTaskService.h"

#include "Position.h"
#include "UnitConversions.h"

#include "avtas/lmcp/LmcpXMLReader.h"
#include "afrl/cmasi/VehicleActionCommand.h"
#include "afrl/cmasi/GimbalStareAction.h"
#include "afrl/cmasi/LoiterAction.h"
#include "uxas/messages/task/TaskOption.h"


#include <sstream>      //std::stringstream
#include "afrl/cmasi/GimbalConfiguration.h"


namespace uxas
{
namespace service
{
namespace task
{
OverwatchTaskService::ServiceBase::CreationRegistrar<OverwatchTaskService>
OverwatchTaskService::s_registrar(OverwatchTaskService::s_registryServiceTypeNames());

OverwatchTaskService::OverwatchTaskService()
: DynamicTaskServiceBase(OverwatchTaskService::s_typeName(), OverwatchTaskService::s_directoryName())
{
};

OverwatchTaskService::~OverwatchTaskService()
{
};

bool
OverwatchTaskService::configureDynamicTask(const pugi::xml_node& ndComponent)

{
    std::string strBasePath = m_workDirectoryPath;
    std::stringstream sstrErrors;

    bool isSuccessful(true);

    if (isSuccessful)
    {
        if (afrl::impact::isWatchTask(m_task.get()))
        {
            m_watchTask = std::static_pointer_cast<afrl::impact::WatchTask>(m_task);
            if (!m_watchTask)
            {
				UXAS_LOG_ERROR("**OverwatchTaskService::bConfigure failed to cast a WatchTask from the task pointer.");
                isSuccessful = false;
            }
        }
        else
        {
            sstrErrors << "ERROR:: **OverwatchTaskService::bConfigure failed: taskObject[" << m_task->getFullLmcpTypeName() << "] is not a WatchTask." << std::endl;
			UXAS_LOG_ERROR("**OverwatchTaskService::bConfigure failed: taskObject[" + m_task->getFullLmcpTypeName() + "] is not a WatchTask.");
            isSuccessful = false;
        }
    } //isSuccessful

	if (m_entityStates.find(m_watchTask->getWatchedEntityID()) != m_entityStates.end())
	{
		m_watchedEntityStateLast = m_entityStates[m_watchTask->getWatchedEntityID()];
	}

    m_straightLineThreshold_m = m_loiterRadius_m * 1.5;

    return (isSuccessful);
}

bool
OverwatchTaskService::processRecievedLmcpMessageDynamicTask(std::shared_ptr<avtas::lmcp::Object>& receivedLmcpObject)
//example: if (afrl::cmasi::isServiceStatus(receivedLmcpObject))
{
    auto entityState = std::dynamic_pointer_cast<afrl::cmasi::EntityState>(receivedLmcpObject);
    if (entityState)
    {
        if (entityState->getID() == m_watchTask->getWatchedEntityID())
        {
            m_watchedEntityStateLast = entityState;
        }
    }

    return (false); // always false implies never terminating service from here
};

void OverwatchTprocessMissionCommand(std::shared_ptr<afrl::cmasi::MissionCommand>);

void OverwatchTaskService::processMissionCommand(std::shared_ptr<afrl::cmasi::MissionCommand> mish)
{

    auto lastWaypoint = mish->getWaypointList().back();
    // point the camera at the search point
    //vehicleActionCommand->setCommandID();
    //vehicleActionCommand->setStatus();
    auto gimbalStareAction = new afrl::cmasi::GimbalStareAction;
    gimbalStareAction->setStarepoint(m_watchedEntityStateLast->getLocation()->clone());
    if (m_entityConfigurations.find(mish->getVehicleID()) != m_entityConfigurations.end())
    {
        auto config = m_entityConfigurations[mish->getVehicleID()];
        for (auto payload : config->getPayloadConfigurationList())
        {
            if (afrl::cmasi::isGimbalConfiguration(payload))
            {
                gimbalStareAction->setPayloadID(payload->getPayloadID());
            }
        }
    }
    lastWaypoint->getVehicleActionList().push_back(gimbalStareAction);
    gimbalStareAction = nullptr; //gave up ownership
                                 // add the loiter
    auto loiterAction = new afrl::cmasi::LoiterAction;
    loiterAction->setLocation(m_watchedEntityStateLast->getLocation()->clone());
    if (loiterAction->getLocation()->getAltitude() < 5) //too close to ground
    {
        //TODO: use current entityStates altitude
    }
    if (m_entityConfigurations.find(mish->getVehicleID()) != m_entityConfigurations.end())
    {
        auto config = m_entityConfigurations[mish->getVehicleID()];
        loiterAction->setAirspeed(config->getNominalSpeed());
        loiterAction->getLocation()->setAltitude(config->getNominalAltitude());
    }
    else
    {
        UXAS_LOG_ERROR("ERROR::Task_WatchTask:: no EntityConfiguration found for Entity[" + std::to_string(mish->getVehicleID()) + "]");
    }
    loiterAction->setRadius(m_loiterRadius_m);
    loiterAction->setAxis(0.0);
    loiterAction->setDirection(afrl::cmasi::LoiterDirection::Clockwise);
    loiterAction->setDuration(-1.0);
    loiterAction->setLength(0.0);
    loiterAction->setLoiterType(afrl::cmasi::LoiterType::Circular);
    lastWaypoint->getVehicleActionList().push_back(loiterAction);
    loiterAction = nullptr; //gave up ownership
}

void OverwatchTaskService::buildTaskPlanOptions()
{
    bool isSuccessful{true};

    int64_t optionId(1);
    int64_t taskId(m_watchTask->getTaskID());

    if (isCalculateOption(taskId, optionId, m_watchTask->getEligibleEntities()))
    {
        optionId++;
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
    if (isSuccessful)
    {
        auto newResponse = std::static_pointer_cast<avtas::lmcp::Object>(m_taskPlanOptions);
        sendSharedLmcpObjectBroadcastMessage(newResponse);
    }
};

bool OverwatchTaskService::isCalculateOption(const int64_t& taskId, int64_t& optionId, const std::vector<int64_t>& eligibleEntities) {
    bool isSuccessful{true};

    if (m_watchedEntityStateLast)
    {
        auto taskOption = new uxas::messages::task::TaskOption;
        taskOption->setTaskID(taskId);
        taskOption->setOptionID(optionId);
        taskOption->getEligibleEntities() = eligibleEntities;
        taskOption->setStartLocation(m_watchedEntityStateLast->getLocation()->clone());
        taskOption->setStartHeading(m_watchedEntityStateLast->getHeading());
        taskOption->setEndLocation(m_watchedEntityStateLast->getLocation()->clone());
        taskOption->setEndHeading(m_watchedEntityStateLast->getHeading());
        auto pTaskOption = std::shared_ptr<uxas::messages::task::TaskOption>(taskOption->clone());
        m_optionIdVsTaskOptionClass.insert(std::make_pair(optionId, std::make_shared<TaskOptionClass>(pTaskOption)));
        m_taskPlanOptions->getOptions().push_back(taskOption);
        taskOption = nullptr; //just gave up ownership

    }
    else
    {
		UXAS_LOG_ERROR("ERROR::Task_WatchTask:: no watchedEntityState found for Entity[" + std::to_string(m_watchTask->getWatchedEntityID()) + "]");
        isSuccessful = false;
    }

    return (isSuccessful);
}

std::shared_ptr<afrl::cmasi::Location3D> OverwatchTaskService::calculateTargetLocation(const std::shared_ptr<afrl::cmasi::EntityState> entityState)
{
    if (m_watchedEntityStateLast)
    {
        return std::make_shared<afrl::cmasi::Location3D>(*m_watchedEntityStateLast->getLocation()->clone());
    }
    return nullptr;
}

}; //namespace task
}; //namespace service
}; //namespace uxas
