// ===============================================================================
// Authors: AFRL/RQQA
// Organization: Air Force Research Laboratory, Aerospace Systems Directorate, Power and Control Division
// 
// Copyright (c) 2017 Government of the United State of America, as represented by
// the Secretary of the Air Force.  No copyright is claimed in the United States under
// Title 17, U.S. Code.  All Other Rights Reserved.
// ===============================================================================

/* 
 * File:   Task_CommRelayTask.cpp
 * Author: Derek & steve
 *
 * Created on March 7, 2016, 6:17 PM
 */


#include "CommRelayTaskService.h"

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
#include "afrl/impact/RadioTowerConfiguration.h"
#include "afrl/impact/RadioTowerState.h"
#include "uxas/messages/task/TaskImplementationResponse.h"
#include "uxas/messages/task/TaskOption.h"


#include "Constants/Convert.h"
#include "DpssDataTypes.h"
#include "TimeUtilities.h"

#include <sstream>      //std::stringstream



namespace uxas
{
namespace service
{
namespace task
{
CommRelayTaskService::ServiceBase::CreationRegistrar<CommRelayTaskService>
CommRelayTaskService::s_registrar(CommRelayTaskService::s_registryServiceTypeNames());

CommRelayTaskService::CommRelayTaskService()
: DynamicTaskServiceBase(CommRelayTaskService::s_typeName(), CommRelayTaskService::s_directoryName())
{
};

CommRelayTaskService::~CommRelayTaskService()
{
};

bool
CommRelayTaskService::configureDynamicTask(const pugi::xml_node& ndComponent)

{
    std::string strBasePath = m_workDirectoryPath;
    std::stringstream sstrErrors;

    bool isSuccessful(true);

    if (isSuccessful)
    {
        if (afrl::impact::isCommRelayTask(m_task.get()))
        {
            m_CommRelayTask = std::static_pointer_cast<afrl::impact::CommRelayTask>(m_task);
            if (!m_CommRelayTask)
            {
				UXAS_LOG_ERROR("**CommRelayTaskService::bConfigure failed to cast a CommRelayTask from the task pointer.");
                isSuccessful = false;
            }
        }
        else
        {
			UXAS_LOG_ERROR("**CommRelayTaskService::bConfigure failed: taskObject[" + m_task->getFullLmcpTypeName() + "] is not a CommRelayTask.");
            isSuccessful = false;
        }
    } //isSuccessful
    if (isSuccessful)
    {
        if (m_CommRelayTask->getSupportedEntityID() == 0)
        {
            if (m_CommRelayTask->getDestinationLocation() != nullptr)
            {
                m_supportedEntityStateLast = std::shared_ptr<afrl::cmasi::Location3D>(m_CommRelayTask->getDestinationLocation()->clone());
            }
        }
        else
        {
            if (m_entityStates.find(m_CommRelayTask->getSupportedEntityID()) != m_entityStates.end())
            {
                m_supportedEntityStateLast = std::shared_ptr<afrl::cmasi::Location3D>(m_entityStates[m_CommRelayTask->getSupportedEntityID()]->getLocation()->clone());
            }
            else
            {
                UXAS_LOG_ERROR("**CommRelayTaskService: supportedEntityID ", m_CommRelayTask->getSupportedEntityID(), " does not exist");
                isSuccessful = false;
            }
        }

        auto towerId = m_CommRelayTask->getTowerID();
        if (m_entityStates.find(towerId) == m_entityStates.end())
        {
            UXAS_LOG_ERROR("**CommRelayTaskService: tower with ID ", towerId, " does not exist");
            isSuccessful = false;
        }

        if (m_entityConfigurations.find(towerId) != m_entityConfigurations.end())
        {
            if (!afrl::impact::isRadioTowerConfiguration(m_entityConfigurations[towerId].get()))
            {
                UXAS_LOG_ERROR("**CommRelayTaskService: entity  config with ID ", towerId, " is not a radio tower");
                isSuccessful = false;
            }
        }
        else
        {
            UXAS_LOG_ERROR("CommRelayTaskService: radio tower ID ", towerId, " does not exist");
            isSuccessful = false;
        }

    } //if(isSuccessful)

    return (isSuccessful);
}

bool
CommRelayTaskService::processRecievedLmcpMessageDynamicTask(std::shared_ptr<avtas::lmcp::Object>& receivedLmcpObject)
//example: if (afrl::cmasi::isServiceStatus(receivedLmcpObject))
{
    auto entityState = std::dynamic_pointer_cast<afrl::cmasi::EntityState>(receivedLmcpObject);
    if (entityState)
    {
        if (entityState->getID() == m_CommRelayTask->getSupportedEntityID())
        {
            m_supportedEntityStateLast = std::shared_ptr<afrl::cmasi::Location3D>(entityState->getLocation()->clone());
        }
    }
    return (false); // always false implies never terminating service from here
};

void CommRelayTaskService::buildTaskPlanOptions()
{
    bool isSuccessful{true};

    int64_t optionId(1);
    int64_t taskId(m_CommRelayTask->getTaskID());

    if (isCalculateOption(taskId, optionId))
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
}

std::shared_ptr<afrl::cmasi::Location3D> CommRelayTaskService::calculateTargetLocation(const std::shared_ptr<afrl::cmasi::EntityState> entityState)
{
    auto middle = std::shared_ptr<afrl::cmasi::Location3D>(entityState->getLocation()->clone());
    if (m_supportedEntityStateLast)
    {
        // look up speed to use for commanding vehicle
        double alt = entityState->getLocation()->getAltitude();
        if (m_entityConfigurations.find(entityState->getID()) != m_entityConfigurations.end())
        {
            alt = m_entityConfigurations[entityState->getID()]->getNominalAltitude();
        }

        // extract location of tower
        int64_t towerId = m_CommRelayTask->getTowerID();
        std::shared_ptr<afrl::cmasi::Location3D> towerLocation{ nullptr };
        if (m_entityStates.find(towerId) != m_entityStates.end())
        {
            towerLocation.reset(m_entityStates[towerId]->getLocation()->clone());
        }

        if (!towerLocation)
        {
            if (m_entityConfigurations.find(towerId) != m_entityConfigurations.end())
            {
                if (afrl::impact::isRadioTowerConfiguration(m_entityConfigurations[towerId].get()))
                {
                    auto config = std::static_pointer_cast<afrl::impact::RadioTowerConfiguration>(m_entityConfigurations[towerId]);
                    towerLocation.reset(config->getPosition()->clone());
                }
            }
        }

        if (!towerLocation) // don't care if not enabled, still attempt relay
            return middle;

        // determine destination location
        uxas::common::utilities::CUnitConversions flatEarth;
        double north, east;
        flatEarth.ConvertLatLong_degToNorthEast_m(towerLocation->getLatitude(), towerLocation->getLongitude(), north, east);
        Dpss_Data_n::xyPoint tower(east, north);
        flatEarth.ConvertLatLong_degToNorthEast_m(m_supportedEntityStateLast->getLatitude(), m_supportedEntityStateLast->getLongitude(), north, east);
        Dpss_Data_n::xyPoint target(east, north);

        // go halfway between 'targetLocation' and 'tower' TODO: make this more efficient?
        target.x = (tower.x + target.x) / 2;
        target.y = (tower.y + target.y) / 2;

        // back to lat/lon
        double lat, lon;
        flatEarth.ConvertNorthEast_mToLatLong_deg(target.y, target.x, lat, lon);

        //auto loc = std::make_shared<afrl::cmasi::Location3D>();
        middle->setLatitude(lat);
        middle->setLongitude(lon);
        middle->setAltitude(alt);

    }
    return middle;
}

bool CommRelayTaskService::isCalculateOption(const int64_t& taskId, int64_t & optionId)
{
    bool isSuccessful{true};

    if (m_supportedEntityStateLast)
    {
        auto taskOption = new uxas::messages::task::TaskOption;
        taskOption->setTaskID(taskId);
        taskOption->setOptionID(optionId);
        taskOption->getEligibleEntities() = m_CommRelayTask->getEligibleEntities();
        taskOption->setStartLocation(m_supportedEntityStateLast->clone());
        //taskOption->setStartHeading(m_supportedEntityStateLast->getHeading());
        taskOption->setEndLocation(m_supportedEntityStateLast->clone());
        //taskOption->setEndHeading(m_supportedEntityStateLast->getHeading());
        auto pTaskOption = std::shared_ptr<uxas::messages::task::TaskOption>(taskOption->clone());
        m_optionIdVsTaskOptionClass.insert(std::make_pair(optionId, std::make_shared<TaskOptionClass>(pTaskOption)));
        m_taskPlanOptions->getOptions().push_back(taskOption);
        taskOption = nullptr; //just gave up ownership

    }
    else
    {
		UXAS_LOG_ERROR("Task_CommRelayTask:: no watchedEntityState found for Entity[" + std::to_string(m_CommRelayTask->getSupportedEntityID()) + "]");
        isSuccessful = false;
    }

    return (isSuccessful);
}



void CommRelayTaskService::processMissionCommand(std::shared_ptr<afrl::cmasi::MissionCommand> mission)
{
    if (mission->getWaypointList().empty())
    {
        return;
    }

    auto targetLocation = m_targetLocations[mission->getVehicleID()];
    auto state = m_entityStates[mission->getVehicleID()];

    auto actionCommand = CalculateGimbalActions(state, targetLocation->getLatitude(), targetLocation->getLongitude());
    auto firstWaypoint = mission->getWaypointList().front();

    for (auto action : actionCommand->getVehicleActionList())
    {
        firstWaypoint->getVehicleActionList().push_back(action->clone());
    }
}

std::shared_ptr<afrl::cmasi::VehicleActionCommand> CommRelayTaskService::CalculateGimbalActions(const std::shared_ptr<afrl::cmasi::EntityState>& entityState, double lat, double lon)
{
    std::shared_ptr<afrl::cmasi::VehicleActionCommand> caction(new afrl::cmasi::VehicleActionCommand);
    caction->setVehicleID(entityState->getID());

    double surveyRadius = m_loiterRadius_m;
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
            surveyRadius = m_loiterRadius_m;
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
        gimbalAction->setStarepoint(m_supportedEntityStateLast->clone());
        gimbalAction->getAssociatedTaskList().push_back(m_task->getTaskID());
        caction->getVehicleActionList().push_back(gimbalAction->clone());
    }

    return caction;
}

}; //namespace task
}; //namespace service
}; //namespace uxas
