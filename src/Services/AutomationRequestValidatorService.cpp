// ===============================================================================
// Authors: AFRL/RQQA
// Organization: Air Force Research Laboratory, Aerospace Systems Directorate, Power and Control Division
// 
// Copyright (c) 2017 Government of the United State of America, as represented by
// the Secretary of the Air Force.  No copyright is claimed in the United States under
// Title 17, U.S. Code.  All Other Rights Reserved.
// ===============================================================================

/* 
 * File:   AutomationRequestValidatorService.cpp
 * Author: derek
 * 
 * Created on Aug 24, 2015, 9:31 AM
 */

#include "AutomationRequestValidatorService.h"

#include "TimeUtilities.h"
#include "UxAS_Log.h"
#include "UxAS_TimerManager.h"

#include "uxas/messages/task/TaskInitialized.h"
#include "uxas/messages/task/TaskAutomationRequest.h"
#include "uxas/messages/task/TaskAutomationResponse.h"
#include "uxas/messages/task/UniqueAutomationResponse.h"
#include "afrl/cmasi/AutomationRequest.h"
#include "afrl/cmasi/AutomationResponse.h"
#include "afrl/impact/ImpactAutomationRequest.h"
#include "afrl/impact/ImpactAutomationResponse.h"
#include "afrl/impact/PointOfInterest.h"
#include "afrl/impact/LineOfInterest.h"
#include "afrl/impact/AreaOfInterest.h"

#include "afrl/cmasi/ServiceStatus.h"
#include "afrl/cmasi/EntityConfiguration.h"
#include "afrl/cmasi/EntityConfigurationDescendants.h"
#include "afrl/cmasi/EntityState.h"
#include "afrl/cmasi/EntityStateDescendants.h"
#include "afrl/cmasi/RemoveTasks.h"
#include "afrl/cmasi/KeepInZone.h"
#include "afrl/cmasi/KeepOutZone.h"

#include "afrl/cmasi/Task.h"
#include "afrl/cmasi/TaskDescendants.h"    

#include "afrl/impact/PointOfInterest.h"
#include "afrl/impact/LineOfInterest.h"
#include "afrl/impact/AreaOfInterest.h"

#include "pugixml.hpp"

namespace uxas
{
namespace service
{
AutomationRequestValidatorService::ServiceBase::CreationRegistrar<AutomationRequestValidatorService>
AutomationRequestValidatorService::s_registrar(AutomationRequestValidatorService::s_registryServiceTypeNames());

AutomationRequestValidatorService::AutomationRequestValidatorService()
: ServiceBase(AutomationRequestValidatorService::s_typeName(), AutomationRequestValidatorService::s_directoryName())
{
};

AutomationRequestValidatorService::~AutomationRequestValidatorService()
{
    uint64_t delayTime_ms{10};
    if (m_responseTimerId && !uxas::common::TimerManager::getInstance().destroyTimer(m_responseTimerId, delayTime_ms))
    {
        UXAS_LOG_WARN("AutomationRequestValidatorService::~AutomationRequestValidatorService failed to destroy response timer "
                "(m_responseTimerId) with timer ID ", m_responseTimerId, " within ", delayTime_ms, " millisecond timeout");
    }
    if (m_taskInitTimerId && !uxas::common::TimerManager::getInstance().destroyTimer(m_taskInitTimerId, delayTime_ms))
    {
        UXAS_LOG_WARN("AutomationRequestValidatorService::~AutomationRequestValidatorService failed to destroy response timer "
                "(m_taskInitTimerId) with timer ID ", m_taskInitTimerId, " within ", delayTime_ms, " millisecond timeout");
    }
};

bool
AutomationRequestValidatorService::initialize()
{
    // create timers
    m_responseTimerId = uxas::common::TimerManager::getInstance().createTimer(
        std::bind(&AutomationRequestValidatorService::OnResponseTimeout, this),
        "AutomationRequestValidatorService::OnResponseTimeout()");
    m_taskInitTimerId = uxas::common::TimerManager::getInstance().createTimer(
        std::bind(&AutomationRequestValidatorService::OnTasksReadyTimeout, this),
        "AutomationRequestValidatorService::OnTasksReadyTimeout()");

    m_messageSender.initializePush(m_messageSourceGroup, 0, 0);

    return true;
}

bool
AutomationRequestValidatorService::configure(const pugi::xml_node & ndComponent)
{
    // configure response time parameter, ensure response time is reasonable
    m_maxResponseTime_ms = ndComponent.attribute("MaxResponseTime_ms").as_uint(m_maxResponseTime_ms);
    if(m_maxResponseTime_ms < 10) m_maxResponseTime_ms = 10;

    // translate regular, impact, and task automation requests to unique automation requests
    addSubscriptionAddress(afrl::cmasi::AutomationRequest::Subscription);
    addSubscriptionAddress(afrl::impact::ImpactAutomationRequest::Subscription);
    addSubscriptionAddress(uxas::messages::task::TaskAutomationRequest::Subscription);
    addSubscriptionAddress(afrl::cmasi::ServiceStatus::Subscription);
    
    // respond with appropriate automation response based on unique response
    addSubscriptionAddress(uxas::messages::task::UniqueAutomationResponse::Subscription);

    // track all entity configurations
    addSubscriptionAddress(afrl::cmasi::EntityConfiguration::Subscription);
    std::vector< std::string > childconfigs = afrl::cmasi::EntityConfigurationDescendants();
    for(auto child : childconfigs)
        addSubscriptionAddress(child);
    
    // track all entity states
    addSubscriptionAddress(afrl::cmasi::EntityState::Subscription);
    std::vector< std::string > childstates = afrl::cmasi::EntityStateDescendants();
    for(auto child : childstates)
        addSubscriptionAddress(child);
    
    // track airspace constraints
    addSubscriptionAddress(afrl::cmasi::OperatingRegion::Subscription);
    addSubscriptionAddress(afrl::cmasi::KeepInZone::Subscription);
    addSubscriptionAddress(afrl::cmasi::KeepOutZone::Subscription);

    // track indicated locations of interest
    addSubscriptionAddress(afrl::impact::AreaOfInterest::Subscription);
    addSubscriptionAddress(afrl::impact::LineOfInterest::Subscription);
    addSubscriptionAddress(afrl::impact::PointOfInterest::Subscription);
    
    // track all tasks
    addSubscriptionAddress(afrl::cmasi::Task::Subscription);
    std::vector< std::string > childtasks = afrl::cmasi::TaskDescendants();
    for(auto child : childtasks)
        addSubscriptionAddress(child);

    // task removal and initialization
    addSubscriptionAddress(afrl::cmasi::RemoveTasks::Subscription);
    addSubscriptionAddress(uxas::messages::task::TaskInitialized::Subscription);

    return true;
}

bool
AutomationRequestValidatorService::processReceivedLmcpMessage(std::unique_ptr<uxas::communications::data::LmcpMessage> receivedLmcpMessage)
{
    auto entityConfig = std::dynamic_pointer_cast<afrl::cmasi::EntityConfiguration>(receivedLmcpMessage->m_object);
    auto entityState = std::dynamic_pointer_cast<afrl::cmasi::EntityState>(receivedLmcpMessage->m_object);
    auto task = std::dynamic_pointer_cast<afrl::cmasi::Task>(receivedLmcpMessage->m_object);
    if ( entityConfig )
    {
        m_availableConfigurationEntityIds.insert(entityConfig->getID());
        IMPACT_INFORM(entityConfig->getLmcpTypeName(), " received for vehicle ", entityConfig->getID());
    }
    else if ( entityState )
    {
        m_availableStateEntityIds.insert(entityState->getID());
    }
    else if ( task )
    {
        m_availableInitializedTasks.erase(task->getTaskID());
        m_availableTasks[task->getTaskID()] = task;
    }
    else if (afrl::cmasi::isRemoveTasks(receivedLmcpMessage->m_object.get()))
    {
        auto removeTasks = std::static_pointer_cast<afrl::cmasi::RemoveTasks>(receivedLmcpMessage->m_object);
        for (auto& taskId : removeTasks->getTaskList())
        {
            m_availableTasks.erase(taskId);
            m_availableInitializedTasks.erase(taskId);
        }
    }
    else if (uxas::messages::task::isTaskInitialized(receivedLmcpMessage->m_object.get()))
    {
        auto taskInitialized = std::static_pointer_cast<uxas::messages::task::TaskInitialized>(receivedLmcpMessage->m_object);
        m_availableInitializedTasks.insert(taskInitialized->getTaskID());
        checkTasksInitialized();
    }
    else if (afrl::impact::isAreaOfInterest(receivedLmcpMessage->m_object.get()))
    {
        auto areaOfInterest = std::static_pointer_cast<afrl::impact::AreaOfInterest>(receivedLmcpMessage->m_object);
        m_availableAreaOfInterestIds.insert(areaOfInterest->getAreaID());
    }
    else if (afrl::impact::isLineOfInterest(receivedLmcpMessage->m_object.get()))
    {
        auto lineOfInterest = std::static_pointer_cast<afrl::impact::LineOfInterest>(receivedLmcpMessage->m_object);
        m_availableLineOfInterestIds.insert(lineOfInterest->getLineID());
    }
    else if (afrl::impact::isPointOfInterest(receivedLmcpMessage->m_object.get()))
    {
        auto pointOfInterest = std::static_pointer_cast<afrl::impact::PointOfInterest>(receivedLmcpMessage->m_object);
        m_availablePointOfInterestIds.insert(pointOfInterest->getPointID());
    }
    else if (afrl::cmasi::isKeepInZone(receivedLmcpMessage->m_object.get()))
    {
        auto keepInZone = std::static_pointer_cast<afrl::cmasi::KeepInZone>(receivedLmcpMessage->m_object);
        m_availableKeepInZoneIds.insert(keepInZone->getZoneID());
    }
    else if (afrl::cmasi::isKeepOutZone(receivedLmcpMessage->m_object.get()))
    {
        auto keepOutZone = std::static_pointer_cast<afrl::cmasi::KeepOutZone>(receivedLmcpMessage->m_object);
        m_availableKeepOutZoneIds.insert(keepOutZone->getZoneID());
    }
    else if (afrl::cmasi::isOperatingRegion(receivedLmcpMessage->m_object.get()))
    {
        auto operatingRegion = std::static_pointer_cast<afrl::cmasi::OperatingRegion>(receivedLmcpMessage->m_object);
        m_availableOperatingRegions[operatingRegion->getID()] = operatingRegion;
    }
    else if (afrl::cmasi::isServiceStatus(receivedLmcpMessage->m_object))
    {
        auto servStatus = std::static_pointer_cast<afrl::cmasi::ServiceStatus>(receivedLmcpMessage->m_object);
        if (servStatus->getStatusType() == afrl::cmasi::ServiceStatusType::Error)
        {
            for (auto kv : servStatus->getInfo())
            {
                if (kv->getKey().compare("No UniqueAutomationResponse") == 0)
                {
                    if (!m_pendingRequests.empty())
                    {
                        std::shared_ptr<uxas::messages::task::UniqueAutomationRequest> timedOut = m_pendingRequests.front();
                        m_pendingRequests.pop_front();

                        auto id = timedOut->getRequestID();
                        m_timedOutRequests.insert(id);
                        sendResponseError(timedOut, kv->getValue());
                        sendNextRequest();
                    }
                    else
                    {
                        UXAS_LOG_WARN("kvp error with no waiting response! multiple were probably sent for the same request.");
                    }
                }
            }
        }
    }
    else if (afrl::cmasi::isAutomationRequest(receivedLmcpMessage->m_object) ||
            afrl::impact::isImpactAutomationRequest(receivedLmcpMessage->m_object) ||
            uxas::messages::task::isTaskAutomationRequest(receivedLmcpMessage->m_object))
    {
        HandleAutomationRequest(receivedLmcpMessage->m_object);
    }
    else if (uxas::messages::task::isUniqueAutomationResponse(receivedLmcpMessage->m_object.get()))
    {
        HandleAutomationResponse(receivedLmcpMessage->m_object);
    }
    
    return false; // always false unless terminating
}

void AutomationRequestValidatorService::HandleAutomationRequest(std::shared_ptr<avtas::lmcp::Object>& autoRequest)
{
    auto uniqueAutomationRequest = std::shared_ptr<uxas::messages::task::UniqueAutomationRequest> (new uxas::messages::task::UniqueAutomationRequest);
    uniqueAutomationRequest->setRequestID(getUniqueEntitySendMessageId());

    if (afrl::impact::isImpactAutomationRequest(autoRequest))
    {
        auto sand = std::static_pointer_cast<afrl::impact::ImpactAutomationRequest>(autoRequest);
        m_sandboxMap[uniqueAutomationRequest->getRequestID()].requestType = SANDBOX_AUTOMATION_REQUEST;
        m_sandboxMap[uniqueAutomationRequest->getRequestID()].playId = sand->getPlayID();
        m_sandboxMap[uniqueAutomationRequest->getRequestID()].solnId = sand->getSolutionID();
        m_sandboxMap[uniqueAutomationRequest->getRequestID()].sandboxed = sand->getSandbox();
        m_sandboxMap[uniqueAutomationRequest->getRequestID()].taskRequestId = sand->getRequestID();
        m_sandboxMap[uniqueAutomationRequest->getRequestID()].originalRequest = std::shared_ptr<afrl::cmasi::AutomationRequest>(sand->getTrialRequest()->clone());
        uniqueAutomationRequest->setOriginalRequest(sand->getTrialRequest()->clone());
        uniqueAutomationRequest->setSandBoxRequest(sand->getSandbox());
        auto req = sand->getTrialRequest();
        auto desc = generateDescription(req->getEntityList(), req->getTaskList());
        IMPACT_INFORM("recieved impact request with id ", sand->getRequestID(), " ", desc);
    }
    else if (uxas::messages::task::isTaskAutomationRequest(autoRequest))
    {
        auto taskAutomationRequest = std::static_pointer_cast<uxas::messages::task::TaskAutomationRequest>(autoRequest);

        uniqueAutomationRequest->setOriginalRequest((afrl::cmasi::AutomationRequest*) taskAutomationRequest->getOriginalRequest()->clone());
        uniqueAutomationRequest->setSandBoxRequest(taskAutomationRequest->getSandBoxRequest());
        for(auto& planningState : taskAutomationRequest->getPlanningStates())
        {
            uniqueAutomationRequest->getPlanningStates().push_back(planningState->clone());
        }
        m_sandboxMap[uniqueAutomationRequest->getRequestID()].requestType = TASK_AUTOMATION_REQUEST;
        m_sandboxMap[uniqueAutomationRequest->getRequestID()].taskRequestId = taskAutomationRequest->getRequestID();
        m_sandboxMap[uniqueAutomationRequest->getRequestID()].sandboxed = false;
    }
    else
    {
        IMPACT_INFORM("Received CMASI automation request");
        uniqueAutomationRequest->setOriginalRequest((afrl::cmasi::AutomationRequest*) autoRequest->clone());
        m_sandboxMap[uniqueAutomationRequest->getRequestID()].requestType = AUTOMATION_REQUEST;
        m_sandboxMap[uniqueAutomationRequest->getRequestID()].sandboxed = true;

    }

    // queue a valid automation request
    if (isCheckAutomationRequestRequirements(uniqueAutomationRequest))
    {
        m_requestsWaitingForTasks.push_back(uniqueAutomationRequest);
        checkTasksInitialized();
    }
    else
    {
        //send out empty automation response
        m_pendingRequests.push_front(uniqueAutomationRequest);
        auto emptyResponse = std::make_shared<messages::task::UniqueAutomationResponse>();
        emptyResponse->setResponseID(uniqueAutomationRequest->getRequestID());
        auto emptyObjectResponse = std::shared_ptr<avtas::lmcp::Object>(emptyResponse);
        HandleAutomationResponse(emptyObjectResponse);
    }
}

void AutomationRequestValidatorService::HandleAutomationResponse(std::shared_ptr<avtas::lmcp::Object>& autoResponse)
{
    if(m_pendingRequests.empty()) return;

    auto resp = std::static_pointer_cast<uxas::messages::task::UniqueAutomationResponse>(autoResponse);

    if (m_timedOutRequests.find(resp->getResponseID()) != m_timedOutRequests.end())
    {
        UXAS_LOG_ERROR("Response generated after timeout occured. Consider increasing the MaxResponseTime_ms value");
    }

    if (m_pendingRequests.front()->getRequestID() == resp->getResponseID() &&
        m_sandboxMap.find(resp->getResponseID()) != m_sandboxMap.end())
    {
        if (m_sandboxMap[resp->getResponseID()].requestType == TASK_AUTOMATION_REQUEST)
        {
            auto taskResponse = std::make_shared<uxas::messages::task::TaskAutomationResponse>();
            taskResponse->setOriginalResponse(resp->getOriginalResponse()->clone());
            taskResponse->setResponseID(m_sandboxMap[resp->getResponseID()].taskRequestId);

            // add FinalStates to task responses
            for(auto st : resp->getFinalStates())
                taskResponse->getFinalStates().push_back(st->clone());
            sendSharedLmcpObjectBroadcastMessage(taskResponse);
        }
        else if (m_sandboxMap[resp->getResponseID()].requestType == AUTOMATION_REQUEST)
        {
            auto cleanResponse = std::shared_ptr<afrl::cmasi::AutomationResponse>(resp->getOriginalResponse()->clone());
            sendSharedLmcpObjectBroadcastMessage(cleanResponse);
            IMPACT_INFORM("Sent CMASI automation response");
        }
        else
        {
            // look up play and solution IDs
            auto sandResponse = std::shared_ptr<afrl::impact::ImpactAutomationResponse> (new afrl::impact::ImpactAutomationResponse);
            sandResponse->setPlayID(m_sandboxMap[resp->getResponseID()].playId);
            sandResponse->setSolutionID(m_sandboxMap[resp->getResponseID()].solnId);
            sandResponse->setTrialResponse(resp->getOriginalResponse()->clone());
            sandResponse->setSandbox(m_sandboxMap[resp->getResponseID()].sandboxed);
            sandResponse->setResponseID(m_sandboxMap[resp->getResponseID()].taskRequestId);

            //stub out task and vehicle summaries from missionCommands
            for (auto task : m_sandboxMap[resp->getResponseID()].originalRequest->getTaskList())
            {
                auto taskSummary = std::make_shared<afrl::impact::TaskSummary>();
                taskSummary->setTaskID(task);

                for (auto vehicle : m_sandboxMap[resp->getResponseID()].originalRequest->getEntityList())
                {
                    auto vehicleSummary = std::make_shared<afrl::impact::VehicleSummary>();
                    vehicleSummary->setVehicleID(vehicle);
                    vehicleSummary->setDestinationTaskID(task);
                    taskSummary->getPerformingVehicles().push_back(vehicleSummary->clone());
                }
                sandResponse->getSummaries().push_back(taskSummary->clone());
            }
            //run it
            BatchSummaryService::UpdateTaskSummariesUtil(sandResponse->getSummaries(), resp->getOriginalResponse()->getMissionCommandList());

            sendSharedLmcpObjectBroadcastMessage(sandResponse);
            IMPACT_INFORM("sent ImpactAutomationResponse ", sandResponse->getResponseID());

            if (!sandResponse->getSandbox())
            {
                auto tmpresp = std::shared_ptr<afrl::cmasi::AutomationResponse>(resp->getOriginalResponse()->clone());
                sendSharedLmcpObjectBroadcastMessage(tmpresp);
            }

        }
        m_sandboxMap.erase(resp->getResponseID());
        m_pendingRequests.pop_front();
        sendNextRequest();
    }
}

void AutomationRequestValidatorService::OnResponseTimeout()
{
    if(!m_pendingRequests.empty())
    {
        //Send to self. This will execute on the ZMQ processing thread
        auto err = std::make_shared<afrl::cmasi::ServiceStatus>();
        err->setStatusType(afrl::cmasi::ServiceStatusType::Error);
        auto kvp = new afrl::cmasi::KeyValuePair();
        kvp->setKey("No UniqueAutomationResponse");
        kvp->setValue("Timeout");
        err->getInfo().push_back(kvp);
        m_messageSender.sendSharedLimitedCastMessage(m_entityIdNetworkIdUnicastString, err);
    }
}

void AutomationRequestValidatorService::OnTasksReadyTimeout()
{
    if(!m_requestsWaitingForTasks.empty())
    {
        std::shared_ptr<uxas::messages::task::UniqueAutomationRequest> timedOut = m_requestsWaitingForTasks.front();
        m_requestsWaitingForTasks.pop_front();

        std::stringstream reasonForFailure;
        reasonForFailure << "- automation request ID[" << timedOut->getRequestID() << "] was not able to properly initialize all requested tasks" << std::endl;

        auto id = timedOut->getRequestID();
        m_timedOutRequests.insert(id);
        sendResponseError(timedOut, reasonForFailure.str());
    }
}

void AutomationRequestValidatorService::sendResponseError(std::shared_ptr<uxas::messages::task::UniqueAutomationRequest> errorRequest, std::string errStr)
{
    auto req = errorRequest->getOriginalRequest();
    auto reqID = errorRequest->getRequestID();
    auto desc = generateDescription(req->getEntityList(), req->getTaskList());
    UXAS_LOG_ERROR("Sending Error Response: ", errStr, " ", desc);    
    auto errorResponse = std::make_shared<afrl::cmasi::AutomationResponse>();

    auto keyValuePair = new afrl::cmasi::KeyValuePair;
    keyValuePair->setKey("ERROR");
    keyValuePair->setValue(std::string("RequestValidator: ") + errStr);
    errorResponse->getInfo().push_back(keyValuePair);
    if (m_sandboxMap.find(reqID) != m_sandboxMap.end())
    {
        if (m_sandboxMap[reqID].requestType == TASK_AUTOMATION_REQUEST)
        {
            auto taskResponse = std::make_shared<uxas::messages::task::TaskAutomationResponse>();
            taskResponse->setOriginalResponse(errorResponse->clone());
            taskResponse->setResponseID(m_sandboxMap[reqID].taskRequestId);
            sendSharedLmcpObjectBroadcastMessage(taskResponse);
        }
        else if (m_sandboxMap[reqID].requestType == AUTOMATION_REQUEST)
        {
            auto cleanResponse = std::shared_ptr<afrl::cmasi::AutomationResponse>(errorResponse->clone());
            sendSharedLmcpObjectBroadcastMessage(cleanResponse);
        }
        else
        {
            // look up play and solution IDs
            auto sandResponse = std::shared_ptr<afrl::impact::ImpactAutomationResponse>(new afrl::impact::ImpactAutomationResponse);
            sandResponse->setPlayID(m_sandboxMap[reqID].playId);
            sandResponse->setSolutionID(m_sandboxMap[reqID].solnId);
            sandResponse->setSandbox(false);
            sandResponse->setResponseID(m_sandboxMap[reqID].taskRequestId);
            sandResponse->setTrialResponse(errorResponse->clone());
            sendSharedLmcpObjectBroadcastMessage(sandResponse);
        }
        m_sandboxMap.erase(reqID);
    }
    else
    {
        UXAS_LOG_ERROR("ERROR reported for a response that has already been sent! ", errStr);
    }
}

void AutomationRequestValidatorService::sendNextRequest()
{
    if(m_pendingRequests.empty())
    {
        // no other requests in queue, disable timer
        uxas::common::TimerManager::getInstance().disableTimer(m_responseTimerId,0);
        return;
    }
    
    auto uniqueAutomationRequest = m_pendingRequests.front();

    //double check that tasks exist.
    if (!std::all_of(uniqueAutomationRequest->getOriginalRequest()->getTaskList().begin(),
        uniqueAutomationRequest->getOriginalRequest()->getTaskList().end(),
        [&](const int64_t task) {return m_availableInitializedTasks.find(task) != m_availableInitializedTasks.end(); }))
    {
        sendResponseError(uniqueAutomationRequest, "Tasks were killed");
        m_pendingRequests.pop_front();
        sendNextRequest();
        return;
    }

    sendSharedLmcpObjectBroadcastMessage(uniqueAutomationRequest);

    auto serviceStatus = std::make_shared<afrl::cmasi::ServiceStatus>();
    serviceStatus->setStatusType(afrl::cmasi::ServiceStatusType::Information);
    auto keyValuePair = new afrl::cmasi::KeyValuePair;
    std::string message = "UniqueAutomationRequest[" + std::to_string(uniqueAutomationRequest->getRequestID()) + "] - sent";
    keyValuePair->setKey(message);
    serviceStatus->getInfo().push_back(keyValuePair);
    keyValuePair = nullptr;
    sendSharedLmcpObjectBroadcastMessage(serviceStatus);
    
    // reset the timer
    uxas::common::TimerManager::getInstance().startSingleShotTimer(m_responseTimerId, m_maxResponseTime_ms);
}

void AutomationRequestValidatorService::checkTasksInitialized()
{
    // checks to ensure all tasks are initialized for the requests in the 'task wait' queue
    // if so, moves them to the 'pending' queue, otherwise sets a timer to wait
    
    bool areAllTasksReady = true;
    bool isNewPendingRequest = false;
    while(areAllTasksReady && !m_requestsWaitingForTasks.empty())
    {
        for (auto& taskId : m_requestsWaitingForTasks.front()->getOriginalRequest()->getTaskList())
        {
            auto itStartedTaskId = m_availableInitializedTasks.find(taskId);
            if (itStartedTaskId == m_availableInitializedTasks.end())
            {
                areAllTasksReady = false;
                break;
            }
        }
        
        if(areAllTasksReady)
        {
            // move from 'task wait' queue
            isNewPendingRequest = true;
            m_pendingRequests.push_back(m_requestsWaitingForTasks.front());
            m_requestsWaitingForTasks.pop_front();
            
            // re-set the task initialized check timer
            uxas::common::TimerManager::getInstance().startSingleShotTimer(m_taskInitTimerId, m_maxResponseTime_ms);
        }
    }
    
    if(isNewPendingRequest)
    {
        // if timer not started (i.e. not currently waiting for a response),
        // then send the one that just got added
        if(!uxas::common::TimerManager::getInstance().isTimerActive(m_responseTimerId))
        {
            sendNextRequest();
        }
    }
    else if(!uxas::common::TimerManager::getInstance().isTimerActive(m_taskInitTimerId) && !m_requestsWaitingForTasks.empty())
    {
        // top of task-init queue is still not ready, start timer if not already started
        uxas::common::TimerManager::getInstance().startSingleShotTimer(m_taskInitTimerId, m_maxResponseTime_ms);
    }
    
    if(m_requestsWaitingForTasks.empty())
    {
        // all tasks have been initialized, so disable timer
        uxas::common::TimerManager::getInstance().disableTimer(m_taskInitTimerId, 0);
    }
}

bool AutomationRequestValidatorService::isCheckAutomationRequestRequirements(const std::shared_ptr<uxas::messages::task::UniqueAutomationRequest>& uniqueAutomationRequest)
{
    bool isReady{true};
    std::stringstream reasonForFailure;
    reasonForFailure << "Automation Request ID[" << uniqueAutomationRequest->getRequestID() << "] Not Ready ::" << std::endl;

    if (!uniqueAutomationRequest->getOriginalRequest()->getEntityList().empty())
    {
        // check for required entity configurations, if none are required, make sure there is at least one
        if (!m_availableConfigurationEntityIds.empty())
        {
            if (!uniqueAutomationRequest->getOriginalRequest()->getEntityList().empty())
            {
                for (auto& id : uniqueAutomationRequest->getOriginalRequest()->getEntityList())
                {
                    if (m_availableConfigurationEntityIds.find(id) == m_availableConfigurationEntityIds.end())
                    {
                        reasonForFailure << "- EntityConfiguration for Entity Id[" << id << "] not available." << std::endl;
                        isReady = false;
                    }
                }
            }
        }
        else
        {
            reasonForFailure << "- No EntityConfigurations available." << std::endl;
            isReady = false;
        }

        // check for required entity states, if none are required, make sure there is at least one with matching configuration
        if (!m_availableStateEntityIds.empty())
        {
            for (auto& id : uniqueAutomationRequest->getOriginalRequest()->getEntityList())
            {
                bool isReadyLocal{false};
                if (m_availableStateEntityIds.find(id) != m_availableStateEntityIds.end())
                {
                    isReadyLocal = true;
                }
                if(!isReadyLocal)
                {
                    for(auto& planningState: uniqueAutomationRequest->getPlanningStates())
                    {
                        if(planningState->getEntityID() == id)
                        {
                            isReadyLocal = true;
                            break;
                        }
                    }
                }
                if(!isReadyLocal)
                {
                    isReady = false;
                    reasonForFailure << "- EntityState for Entity Id[" << id << "] not available." << std::endl;
                }
            }
        }
        else
        {
            reasonForFailure << "- No EntityStates available." << std::endl;
            isReady = false;
        }
    }
    else //if(!uniqueAutomationRequest->getOriginalRequest()->getEntityList().empty())
    {
        if (!m_availableConfigurationEntityIds.empty() && !m_availableStateEntityIds.empty())
        {
            bool isFoundAMatch{false};
            for (auto& id1 : m_availableConfigurationEntityIds)
            {
                for (auto& id2 : m_availableConfigurationEntityIds)
                {
                    if (id1 == id2)
                    {
                        isFoundAMatch = true;
                        break;
                    }
                }
                if (isFoundAMatch)
                {
                    break;
                }
            }
            if (!isFoundAMatch)
            {
                reasonForFailure << "- No EntityStates that match EntityConfigurations are available." << std::endl;
                isReady = false;
            }
        }
        else
        {
            if (m_availableConfigurationEntityIds.empty())
            {
                reasonForFailure << "- No EntityConfigurations available." << std::endl;
            }
            else
            {
                reasonForFailure << "- No EntityStates available." << std::endl;
            }
            isReady = false;
        }

    } //if(!uniqueAutomationRequest->getOriginalRequest()->getEntityList().empty())

    // check for required operating region and keepin/keepout zones
    if (uniqueAutomationRequest->getOriginalRequest()->getOperatingRegion() != 0)
    {
        auto itOperatingRegion = m_availableOperatingRegions.find(uniqueAutomationRequest->getOriginalRequest()->getOperatingRegion());
        if (itOperatingRegion != m_availableOperatingRegions.end())
        {
            for (auto & keepInArea : itOperatingRegion->second->getKeepInAreas())
            {
                if (m_availableKeepInZoneIds.find(keepInArea) == m_availableKeepInZoneIds.end())
                {
                    reasonForFailure << "- KeepInArea Id[" << keepInArea << "] not available." << std::endl;
                    isReady = false;
                }
            }
            for (auto & keepOutArea : itOperatingRegion->second->getKeepOutAreas())
            {
                if (m_availableKeepOutZoneIds.find(keepOutArea) == m_availableKeepOutZoneIds.end())
                {
                    reasonForFailure << "- KeepOutArea Id[" << keepOutArea << "] not available." << std::endl;
                    isReady = false;
                }
            }
        }
        else
        {
            reasonForFailure << "- OperatingRegion Id[" << uniqueAutomationRequest->getOriginalRequest()->getOperatingRegion() << "] not available." << std::endl;
            isReady = false;
        }
    }

    // check for required tasks and task requirements
    for (auto& taskId : uniqueAutomationRequest->getOriginalRequest()->getTaskList())
    {
        auto itTask = m_availableTasks.find(taskId);
        if (itTask != m_availableTasks.end())
        {
            // check for specific task requirements
            if (itTask->second->getFullLmcpTypeName() == afrl::impact::AngledAreaSearchTask::Subscription)
            {
                auto angledAreaSearchTask = std::static_pointer_cast<afrl::impact::AngledAreaSearchTask>(itTask->second);
                if (angledAreaSearchTask->getSearchAreaID() != 0)
                {
                    if (m_availableAreaOfInterestIds.find(angledAreaSearchTask->getSearchAreaID()) == m_availableAreaOfInterestIds.end())
                    {
                        reasonForFailure << "- AreaOfInterest Id[" << angledAreaSearchTask->getSearchAreaID() << "] not available." << std::endl;
                        isReady = false;
                    }
                }
            }
            else if (itTask->second->getFullLmcpTypeName() == afrl::impact::ImpactLineSearchTask::Subscription)
            {
                auto impactLineSearchTask = std::static_pointer_cast<afrl::impact::ImpactLineSearchTask>(itTask->second);
                if (impactLineSearchTask->getLineID() != 0)
                {
                    if (m_availableLineOfInterestIds.find(impactLineSearchTask->getLineID()) == m_availableLineOfInterestIds.end())
                    {
                        reasonForFailure << "- LineOfInterest Id[" << impactLineSearchTask->getLineID() << "] not available." << std::endl;
                        isReady = false;
                    }
                }
            }
            else if (itTask->second->getFullLmcpTypeName() == afrl::impact::ImpactPointSearchTask::Subscription)
            {
                auto impactPointSearchTask = std::static_pointer_cast<afrl::impact::ImpactPointSearchTask>(itTask->second);
                if (impactPointSearchTask->getSearchLocationID() != 0)
                {
                    if (m_availablePointOfInterestIds.find(impactPointSearchTask->getSearchLocationID()) == m_availablePointOfInterestIds.end())
                    {
                        reasonForFailure << "- LineOfInterest Id[" << impactPointSearchTask->getSearchLocationID() << "] not available." << std::endl;
                        isReady = false;
                    }
                }
            }
        }
        else
        {
            reasonForFailure << "- Task with the Id[" << taskId << "] is unknown. Ensure task description preceeds automation request." << std::endl;
            isReady = false;
        }
    }

    if (!isReady)
    {
        UXAS_LOG_ERROR("Dropping Request " + reasonForFailure.str());
        auto serviceStatus = std::make_shared<afrl::cmasi::ServiceStatus>();
        serviceStatus->setStatusType(afrl::cmasi::ServiceStatusType::Error);
        auto keyValuePair = new afrl::cmasi::KeyValuePair;
        keyValuePair->setKey(std::string("RequestValidator"));
        keyValuePair->setValue(reasonForFailure.str());
        serviceStatus->getInfo().push_back(keyValuePair);
        keyValuePair = nullptr;
        sendSharedLmcpObjectBroadcastMessage(serviceStatus);
    }

    return (isReady);
}
std::string AutomationRequestValidatorService::generateDescription(std::vector<int64_t> vehicles, std::vector<int64_t> tasks) {
    std::string vehiclesDesc = "[";
    for (auto vehicle : vehicles)
    {
        vehiclesDesc += std::to_string(vehicle) + " ";
    }
    vehiclesDesc += "]";

    std::string tasksDesc = "[";
    for (auto task : tasks)
    {
        if (m_availableTasks.find(task) != m_availableTasks.end())
        {
            tasksDesc += m_availableTasks.find(task)->second->getLmcpTypeName() + " " + std::to_string(task) + " ";
        }
        else
        {
            tasksDesc += "type not available " + std::to_string(task) + " ";
        }
    }
    tasksDesc += "]";

    return "vehicles: " + vehiclesDesc + " tasks: " + tasksDesc;
}


}; //namespace service
}; //namespace uxas
