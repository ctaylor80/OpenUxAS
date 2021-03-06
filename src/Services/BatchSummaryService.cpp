// ===============================================================================
// Authors: AFRL/RQQA
// Organization: Air Force Research Laboratory, Aerospace Systems Directorate, Power and Control Division
// 
// Copyright (c) 2017 Government of the United State of America, as represented by
// the Secretary of the Air Force.  No copyright is claimed in the United States under
// Title 17, U.S. Code.  All Other Rights Reserved.
// ===============================================================================

/* 
 * File:   Component_BatchSummary.cpp
 * Author: derek
 * 
 * Created on Oct 25, 2015, 3:56 PM
 */

#include "BatchSummaryService.h"

#include "UxAS_Log.h"
#include "UnitConversions.h"
#include "Permute.h"

#include "afrl/cmasi/Task.h"
#include "afrl/cmasi/TaskDescendants.h"     

#include "afrl/vehicles/VEHICLES.h"

#include <map>
#include <numeric>
#include <uxas/messages/task/TaskPlanOptions.h>
#include <Polygon.h>

#define STRING_COMPONENT_NAME "BatchSummary"
#define STRING_XML_COMPONENT_TYPE STRING_COMPONENT_NAME
#define STRING_XML_COMPONENT "Component"
#define STRING_XML_FAST_PLAN "FastPlan"
#define STRING_XML_LANE_SPACING "LaneSpacing"


namespace uxas
{
namespace service
{
BatchSummaryService::ServiceBase::CreationRegistrar<BatchSummaryService>
    BatchSummaryService::s_registrar(BatchSummaryService::s_registryServiceTypeNames());

BatchSummaryService::BatchSummaryService()
    : ServiceBase(BatchSummaryService::s_typeName(), BatchSummaryService::s_directoryName())
{
};

BatchSummaryService::~BatchSummaryService()
{
};

bool BatchSummaryService::initialize()
{

    return true;
}

bool
    BatchSummaryService::configure(const pugi::xml_node & ndComponent)

{
    std::string strFastPlan = ndComponent.attribute(STRING_XML_FAST_PLAN).value();
    if (!strFastPlan.empty())
    {
        m_fastPlan = ndComponent.attribute(STRING_XML_FAST_PLAN).as_bool();
    }


    addSubscriptionAddress(afrl::cmasi::EntityState::Subscription);
    for (auto descendant : afrl::cmasi::EntityStateDescendants())
        addSubscriptionAddress(descendant);
    
    addSubscriptionAddress(afrl::cmasi::EntityConfiguration::Subscription);
    for (auto descendant : afrl::cmasi::EntityConfigurationDescendants())
        addSubscriptionAddress(descendant);


    //assume all KOZs apply to all vehicles for checking conflictsWithROZ
    addSubscriptionAddress(afrl::cmasi::KeepOutZone::Subscription);

    // Primary messages for actual route construction
    addSubscriptionAddress(afrl::impact::BatchSummaryRequest::Subscription);
    addSubscriptionAddress(messages::task::TaskAutomationResponse::Subscription);

    addSubscriptionAddress(afrl::impact::AngledAreaSearchTask::Subscription);
    addSubscriptionAddress(afrl::impact::AreaOfInterest::Subscription);

    addSubscriptionAddress(afrl::impact::ImpactLineSearchTask::Subscription);
    addSubscriptionAddress(afrl::impact::LineOfInterest::Subscription);
    return true; // may not have the proper fast plan value, but proceed anyway
}

bool BatchSummaryService::processReceivedLmcpMessage(std::unique_ptr<uxas::communications::data::LmcpMessage> receivedLmcpMessage)
//example: if (afrl::cmasi::isServiceStatus(receivedLmcpMessage->m_object.get()))
{
    if (afrl::impact::isBatchSummaryRequest(receivedLmcpMessage->m_object.get()))
    {
        HandleBatchSummaryRequest(std::static_pointer_cast<afrl::impact::BatchSummaryRequest>(receivedLmcpMessage->m_object));
    }
    else if (std::dynamic_pointer_cast<afrl::cmasi::EntityConfiguration>(receivedLmcpMessage->m_object))
    {
        auto config = std::dynamic_pointer_cast<afrl::cmasi::EntityConfiguration>(receivedLmcpMessage->m_object);
        int64_t id = config->getID();
        m_entityConfigs[id] = config;

        if (afrl::impact::isRadioTowerConfiguration(receivedLmcpMessage->m_object.get()))
        {
            auto rconfig = std::static_pointer_cast<afrl::impact::RadioTowerConfiguration>(receivedLmcpMessage->m_object);
            int64_t id = rconfig->getID();
            m_towerLocations[id] = std::shared_ptr<afrl::cmasi::Location3D>(rconfig->getPosition()->clone());
            m_towerRanges[id] = std::make_pair(rconfig->getRange(), rconfig->getEnabled());
        }
    }
    else if (std::dynamic_pointer_cast<afrl::cmasi::EntityState>(receivedLmcpMessage->m_object))
    {
        auto state = std::dynamic_pointer_cast<afrl::cmasi::EntityState>(receivedLmcpMessage->m_object);
        int64_t id = state->getID();
        m_entityStates[id] = state;

   if (afrl::impact::isRadioTowerState(receivedLmcpMessage->m_object.get()))
   {
       m_towerLocations[id] = std::shared_ptr<afrl::cmasi::Location3D>(state->getLocation()->clone());
       auto rs = std::static_pointer_cast<afrl::impact::RadioTowerState>(receivedLmcpMessage->m_object);
       if (m_towerRanges.find(id) != m_towerRanges.end())
       {
           m_towerRanges[id].second = rs->getEnabled();
       }
   }
       }

       else if (messages::task::isTaskAutomationResponse(receivedLmcpMessage->m_object))
       {

           HandleTaskAutomationResponse(std::static_pointer_cast<messages::task::TaskAutomationResponse>(receivedLmcpMessage->m_object));
           //check if all have been received and send out the batchSumaryResponse.
       }
       else if (afrl::cmasi::isKeepOutZone(receivedLmcpMessage->m_object))
       {
           auto koz = std::static_pointer_cast<afrl::cmasi::AbstractZone>(receivedLmcpMessage->m_object);
           auto poly = FromAbstractGeometry(koz->getBoundary());

           auto pair = std::make_shared<ZonePair>();
           pair->VisiLibityZone = poly;
           pair->LmcpZone = koz;
           m_keepOutZones[koz->getZoneID()] = pair;
       }
       else if (afrl::impact::isAngledAreaSearchTask(receivedLmcpMessage->m_object))
       {
           auto areaTask = std::static_pointer_cast<afrl::impact::AngledAreaSearchTask>(receivedLmcpMessage->m_object);
           m_taskIdToCacheId[areaTask->getTaskID()] = areaTask->getSearchAreaID();
       }
       else if (afrl::impact::isAreaOfInterest(receivedLmcpMessage->m_object))
       {
           auto areaOfInterest = std::static_pointer_cast<afrl::impact::AreaOfInterest>(receivedLmcpMessage->m_object);
           auto id = areaOfInterest->getAreaID();
           for (auto it = m_taskIdToCacheId.begin(); it != m_taskIdToCacheId.end();)
           {
               if (it->second == id)
                   it = m_taskIdToCacheId.erase(it);
               else
                   ++it;
           }
       }
       else if (afrl::impact::isImpactLineSearchTask(receivedLmcpMessage->m_object))
       {
           auto lineTask = std::static_pointer_cast<afrl::impact::ImpactLineSearchTask>(receivedLmcpMessage->m_object);
           m_taskIdToCacheId[lineTask->getTaskID()] = lineTask->getLineID();
       }
       else if (afrl::impact::isLineOfInterest(receivedLmcpMessage->m_object))
       {
           auto lineOfInterest = std::static_pointer_cast<afrl::impact::LineOfInterest>(receivedLmcpMessage->m_object);
           auto id = lineOfInterest->getLineID();
           for (auto it = m_taskIdToCacheId.begin(); it != m_taskIdToCacheId.end();)
           {
               if (it->second == id)
                   it = m_taskIdToCacheId.erase(it);
               else
                   ++it;
           }
       }
       return (false); // always false implies never terminating service from here
}

void BatchSummaryService::HandleTaskAutomationResponse(const std::shared_ptr<messages::task::TaskAutomationResponse>& taskAutomationResponse)
{

    //auto taskAutomationRequest = m_pendingTaskAutomationRequests.find(taskAutomationResponse->getResponseID())->second;
    m_pendingTaskAutomationRequests.erase(taskAutomationResponse->getResponseID());

    //remove pending task automation response. If any are empty, finalize
    std::list<int64_t> finishedIds;
    auto batchSummary = std::find_if(m_batchSummaryRequestVsTaskAutomation.begin(), m_batchSummaryRequestVsTaskAutomation.end(),
        [&](const std::pair<int64_t, std::list<int64_t>> x)
    {return std::find(x.second.begin(), x.second.end(), taskAutomationResponse->getResponseID()) != x.second.end(); });

    if (batchSummary != m_batchSummaryRequestVsTaskAutomation.end())
    {
        auto batchSummaryResponse = m_workingResponse.find((*batchSummary).first);
        
        if (batchSummaryResponse != m_workingResponse.end())
        {
            UpdateTaskSummariesUtil(batchSummaryResponse->second->getSummaries(), taskAutomationResponse->getOriginalResponse()->getMissionCommandList());
            
            //iterate through vehicle summaries to set conflicts with ROZ and beyondCommRange
            for (auto taskSummary : batchSummaryResponse->second->getSummaries())
            {
                for (auto vehicleSummary : taskSummary->getPerformingVehicles())
                {
                    BatchSummaryService::UpdateVehicleSummary(vehicleSummary);
                }
            }
        }

        (*batchSummary).second.remove(taskAutomationResponse->getResponseID());
        if ((*batchSummary).second.empty())
        {
            finishedIds.push_back((*batchSummary).first);
        }

    }


    for (auto finishedId : finishedIds)
    {
        FinalizeBatchRequest(finishedId);
        m_batchSummaryRequestVsTaskAutomation.erase(finishedId);
    }
}

bool BatchSummaryService::FinalizeBatchRequest(int64_t responseId)
{
    // this function is only called when the summaries have been composed
    // only need to check for possible 'best efforts' regarding the tasks in the
    // current summary list

    if (m_workingResponse.find(responseId) == m_workingResponse.end())
        return false;
    auto resp = m_workingResponse[responseId];

    // Finally re-constructed a full batch response, send along to global
    sendSharedLmcpObjectBroadcastMessage(resp);
    IMPACT_INFORM("sent batch summary id ", resp->getResponseID());

    for (auto taskSummary : resp->getSummaries())
    {
        for (auto vehicleSummary : taskSummary->getPerformingVehicles())
        {
            //do not cache times with the vehicles position
            if (vehicleSummary->getInitialTaskID() == 0)
            {
                continue;
            }
            if (m_taskIdToCacheId.find(vehicleSummary->getInitialTaskID()) != m_taskIdToCacheId.end() &&
                m_taskIdToCacheId.find(vehicleSummary->getDestinationTaskID()) != m_taskIdToCacheId.end() )
            {
                auto key = std::make_tuple(vehicleSummary->getVehicleID(), m_taskIdToCacheId[vehicleSummary->getInitialTaskID()], m_taskIdToCacheId[vehicleSummary->getDestinationTaskID()]);
                if (m_vehicleWithCacheIdsToVehicleSummary.find(key) == m_vehicleWithCacheIdsToVehicleSummary.end())
                {
                    m_vehicleWithCacheIdsToVehicleSummary[key] = std::shared_ptr<afrl::impact::VehicleSummary>(vehicleSummary->clone());
                }
            }

        }
    }

    // clear out this working response from the map
    m_workingResponse.erase(responseId);
    m_workingRequests.erase(responseId);
    return true;
}


void BatchSummaryService::HandleBatchSummaryRequest(std::shared_ptr<afrl::impact::BatchSummaryRequest> request)
{

    //build TaskAutomationRequest messages

    int64_t responseId = m_responseId;
    m_responseId++;

    m_workingResponse[responseId].reset(new afrl::impact::BatchSummaryResponse);
    m_workingResponse[responseId]->setResponseID(request->getRequestID());

    m_workingRequests[responseId] = request;

    //go ahead and make TaskSummaries
    for (auto task : request->getTaskList())
    {
        auto workingSummary = std::make_shared<afrl::impact::TaskSummary>();
        workingSummary->setTaskID(task);
        workingSummary->setBestEffort(100);
        m_workingResponse[responseId]->getSummaries().push_back(workingSummary->clone());
    }
    auto summariesFromCacheCount = 0;

    std::vector<std::shared_ptr<afrl::cmasi::AutomationRequest>> requests;
    if (request->getTaskRelationships().empty()) //no relationship. create a request per each vehicle task pair
    {
        for (auto task : request->getTaskList())
        {
            auto workingTaskSummary = std::find_if(m_workingResponse[responseId]->getSummaries().begin(), m_workingResponse[responseId]->getSummaries().end(), [&](const afrl::impact::TaskSummary* x) { return x->getTaskID() == task; });

            for (auto vehicle : request->getVehicles())
            {
                auto automationRequest = std::make_shared<afrl::cmasi::AutomationRequest>();
                automationRequest->getTaskList().push_back(task);
                automationRequest->getEntityList().push_back(vehicle);
                requests.push_back(automationRequest);

                auto summary = std::make_shared<afrl::impact::VehicleSummary>();
                summary->setVehicleID(vehicle);
                summary->setDestinationTaskID(task);
                summary->setTimeOnTask(-1);
                summary->setTimeToArrive(-1);
                if (workingTaskSummary != m_workingResponse[responseId]->getSummaries().end())
                {
                    (*workingTaskSummary)->getPerformingVehicles().push_back(summary->clone());
                }
            }
        }
    }
    else //assume realtionship wants intertask information. Split accordingly
    {
        for (auto vehicle : request->getVehicles())
        {
            for (auto taski : request->getTaskList())
            {

                //vehicle to task summary
                auto workingTaskSummaryi = std::find_if(m_workingResponse[responseId]->getSummaries().begin(), m_workingResponse[responseId]->getSummaries().end(), [&](const afrl::impact::TaskSummary* x) { return x->getTaskID() == taski; });
                auto summaryVehicleToTask = std::make_shared<afrl::impact::VehicleSummary>();
                summaryVehicleToTask->setVehicleID(vehicle);
                summaryVehicleToTask->setDestinationTaskID(taski);
                summaryVehicleToTask->setTimeOnTask(-1);
                summaryVehicleToTask->setTimeToArrive(-1);

                if (workingTaskSummaryi != m_workingResponse[responseId]->getSummaries().end())
                {
                    (*workingTaskSummaryi)->getPerformingVehicles().push_back(summaryVehicleToTask->clone());
                }

                auto taskiRequestCount = 0;
                for (auto taskj : request ->getTaskList())
                {
                    if (taski == taskj)
                    {
                        continue;
                    }
                    auto automationRequest = std::make_shared<afrl::cmasi::AutomationRequest>();

                    automationRequest->getTaskList().push_back(taski);
                    automationRequest->getTaskList().push_back(taskj);
                    automationRequest->setTaskRelationships(".(p" + std::to_string(taski) + " p" + std::to_string(taskj) + ")");
                    automationRequest->getEntityList().push_back(vehicle);

                    //task to task summaries
                    auto workingTaskSummaryj = std::find_if(m_workingResponse[responseId]->getSummaries().begin(), m_workingResponse[responseId]->getSummaries().end(), [&](const afrl::impact::TaskSummary* x) { return x->getTaskID() == taskj; });

                    //check cache
                    std::shared_ptr<afrl::impact::VehicleSummary> summaryTaskToTask = nullptr;
                    if (m_taskIdToCacheId.find(taski) != m_taskIdToCacheId.end() &&
                        m_taskIdToCacheId.find(taskj) != m_taskIdToCacheId.end())
                    {
                        auto key = std::make_tuple(vehicle, m_taskIdToCacheId[taski], m_taskIdToCacheId[taskj]);
                        if (m_vehicleWithCacheIdsToVehicleSummary.find(key) != m_vehicleWithCacheIdsToVehicleSummary.end())
                        {
                            summaryTaskToTask = std::shared_ptr<afrl::impact::VehicleSummary>(m_vehicleWithCacheIdsToVehicleSummary[key]->clone());
                            summaryTaskToTask->setInitialTaskID(taski);
                            summaryTaskToTask->setDestinationTaskID(taskj);
                            summariesFromCacheCount += 1;
                        }
                    }
                    if (summaryTaskToTask == nullptr)
                    {
                        summaryTaskToTask = std::make_shared<afrl::impact::VehicleSummary>();
                        summaryTaskToTask->setVehicleID(vehicle);
                        summaryTaskToTask->setInitialTaskID(taski);
                        summaryTaskToTask->setDestinationTaskID(taskj);
                        summaryTaskToTask->setTimeOnTask(-1);
                        summaryTaskToTask->setTimeToArrive(-1);

                        requests.push_back(automationRequest);
                        taskiRequestCount += 1;
                    }
                    if (workingTaskSummaryj != m_workingResponse[responseId]->getSummaries().end())
                    {
                        (*workingTaskSummaryj)->getPerformingVehicles().push_back(summaryTaskToTask->clone());
                    }

                }
                if (taskiRequestCount == 0)
                {
                    //Always send individual vehicle to task times in case some tasks can't be completed.
                    auto automationRequest = std::make_shared<afrl::cmasi::AutomationRequest>();
                    automationRequest->getTaskList().push_back(taski);
                    automationRequest->getEntityList().push_back(vehicle);
                    requests.push_back(automationRequest);
                }
            }

        }
    }



    //wrap requests up to send into TaskAutomationRequests
    for (auto requestToSend : requests)
    {
        auto taskAutomationRequest = std::make_shared<messages::task::TaskAutomationRequest>();
        taskAutomationRequest->setSandBoxRequest(true);
        taskAutomationRequest->setRequestID(m_taskAutomationRequestId);
        m_taskAutomationRequestId++;
        taskAutomationRequest->setOriginalRequest(requestToSend->clone());

        std::shared_ptr<avtas::lmcp::Object> pRequest = std::static_pointer_cast<avtas::lmcp::Object>(taskAutomationRequest);
        m_pendingTaskAutomationRequests[taskAutomationRequest->getRequestID()] = taskAutomationRequest;
        m_batchSummaryRequestVsTaskAutomation[responseId].push_back(taskAutomationRequest->getRequestID());
        sendSharedLmcpObjectBroadcastMessage(pRequest);
    }
    IMPACT_INFORM("received batch request ", request->getRequestID(), ". split into ", requests.size(), " internal task Automation Requests. ", summariesFromCacheCount, " From Cache.");
    if (requests.empty())
    {
        FinalizeBatchRequest(responseId);
    }
}

void BatchSummaryService::UpdateSummaryUtil(afrl::impact::VehicleSummary * sum, const std::vector<afrl::cmasi::Waypoint*>::iterator& task_begin, const std::vector<afrl::cmasi::Waypoint*>::iterator& task_end)
{
    if (task_begin == task_end)
    {
        return;
    }

    uxas::common::utilities::CUnitConversions unitConversions;

    //clone waypoints
    for (auto wp = task_begin; wp != task_end; wp++)
    {
        sum->getWaypointList().push_back((*wp)->clone());
    }

    //set timeOnTask and timeToArrive
    afrl::cmasi::Waypoint* prev = nullptr;
    double timeOnTask = 0.0;
    double timeToArrive = 0.0;
    bool onTask;
    for (auto wp = task_begin; wp != task_end; wp++)
    {
      if (prev)
      {
        onTask = !prev->getAssociatedTasks().empty();

        auto timeFromPrev = unitConversions.dGetLinearDistance_m_Lat1Long1_deg_To_Lat2Long2_deg(prev->getLatitude(), prev->getLongitude(), (*wp)->getLatitude(), (*wp)->getLongitude()) / (*wp)->getSpeed();
        if (onTask)
        {
          timeOnTask += timeFromPrev;
        }
        else
        {
          timeToArrive += timeFromPrev;
        }
      }
        prev = *wp;
    }

    timeOnTask *= 1000;
    timeToArrive *= 1000;
    sum->setTimeOnTask(timeOnTask);
    sum->setTimeToArrive(timeToArrive);

    //set first waypoint
    if (sum->getWaypointList().size() > 0)
    {
        sum->setFirstWaypoint(sum->getWaypointList().front()->getNumber());
    }


}

void BatchSummaryService::UpdateTaskSummariesUtil(std::vector<afrl::impact::TaskSummary*> taskSummaries, std::vector<afrl::cmasi::MissionCommand*> missions)
{
    //iterate through waypoints to reconstruct tasks.
    //this gives ordering and lists of waypoints to reconstruct costs.
    for (auto missionCommand : missions)
    {
        auto vehicle = missionCommand->getVehicleID();
        auto taskStart = missionCommand->getWaypointList().begin();
        bool iteratingToTask = true;
        auto workingTask = 0;
        auto prevTask = 0;
        for (auto wpIter = missionCommand->getWaypointList().begin(); wpIter != missionCommand->getWaypointList().end(); wpIter++)
        {
            if (!(*wpIter)->getAssociatedTasks().empty() && *wpIter != missionCommand->getWaypointList().back())
            {
                if (iteratingToTask) //found first waypoint on task
                {
                    iteratingToTask = false;
                    prevTask = workingTask;
                    workingTask = (*wpIter)->getAssociatedTasks().front();
                }
            }
            else
            {
                if (!iteratingToTask) //found last waypoint
                {
                    iteratingToTask = true;
                    auto taskEnd = wpIter;
                    //get the right TaskSummary
                    auto workingTaskSummary = std::find_if(taskSummaries.begin(), taskSummaries.end(), [&](const afrl::impact::TaskSummary* x) { return x->getTaskID() == workingTask; });
                    if (workingTaskSummary != taskSummaries.end())
                    {
                        //get the right vehicleSummary
                        auto vehicleSummary = std::find_if((*workingTaskSummary)->getPerformingVehicles().begin(), (*workingTaskSummary)->getPerformingVehicles().end(), 
                            [&](const afrl::impact::VehicleSummary* x) {return x->getVehicleID() == vehicle && x->getDestinationTaskID() == workingTask && x->getInitialTaskID() == prevTask; });
                        if (vehicleSummary != (*workingTaskSummary)->getPerformingVehicles().end())
                        {
                            UpdateSummaryUtil(*vehicleSummary, taskStart, taskEnd + 1);
                            (*vehicleSummary)->setInitialTaskID(prevTask);
                        }
                    }
                    //set next taskStart
                    taskStart = wpIter;
                }
            }
        }
    }
}


void BatchSummaryService::UpdateVehicleSummary(afrl::impact::VehicleSummary * vehicleSum)
{
    uxas::common::utilities::CUnitConversions unitConversions;


    double north, east;
    auto config = m_entityConfigs[vehicleSum->getVehicleID()];
    //TODO: check conflicts with ROZ. Previously taken out because it took too long.

    // calculate 'EnergyRemaining'
    vehicleSum->setEnergyRemaining(100.0f);

    if (m_entityStates.find(vehicleSum->getVehicleID()) != m_entityStates.end())
    {
        // get current energy of vehicle and energy expenditure rate
        double e = m_entityStates[vehicleSum->getVehicleID()]->getEnergyAvailable(); // %
        double erate = m_entityStates[vehicleSum->getVehicleID()]->getActualEnergyRate(); // %/s

        int64_t time = vehicleSum->getTimeToArrive() + vehicleSum->getTimeOnTask();

        // linear approximation of energy remaining
        double eloss = time * erate / 1000.0; // time back to seconds from milliseconds
        e -= eloss;
        e = (e < 0) ? 0.0 : e;
        vehicleSum->setEnergyRemaining(e);
    }

    // check comm range
    bool inCommRange = false;
    for (auto t : m_towerLocations)
    {
        bool beyondThisTower = false;
        double tn, te;
        unitConversions.ConvertLatLong_degToNorthEast_m(t.second->getLatitude(), t.second->getLongitude(), tn, te);

        if (m_entityStates.find(vehicleSum->getVehicleID()) != m_entityStates.end() &&
            m_towerRanges.find(t.first) != m_towerRanges.end())
        {
            double vn, ve;
            double towerRange = m_towerRanges[t.first].first;
            if (!m_towerRanges[t.first].second)
            {
                towerRange = 1.0;
            }

            // set to max of vehicle, tower
            if (m_entityConfigs.find(vehicleSum->getVehicleID()) != m_entityConfigs.end())
            {
                for (auto pay : m_entityConfigs[vehicleSum->getVehicleID()]->getPayloadConfigurationList())
                {
                    if (afrl::impact::isRadioConfiguration(pay))
                    {
                        auto commpay = static_cast<afrl::impact::RadioConfiguration*>(pay);
                        if (commpay->getRange() > towerRange)
                        {
                            towerRange = commpay->getRange();
                        }
                    }
                }
            }

            unitConversions.ConvertLatLong_degToNorthEast_m(
                m_entityStates[vehicleSum->getVehicleID()]->getLocation()->getLatitude(),
                m_entityStates[vehicleSum->getVehicleID()]->getLocation()->getLongitude(), vn, ve);
            double vdist = sqrt((tn - vn) * (tn - vn) + (te - ve) * (te - ve));
            beyondThisTower = (vdist > towerRange);
            for (auto wp : vehicleSum->getWaypointList())
            {
                if (beyondThisTower)
                    break;
                double pn, pe;
                unitConversions.ConvertLatLong_degToNorthEast_m(wp->getLatitude(), wp->getLongitude(), pn, pe);
                double pdist = sqrt((tn - pn) * (tn - pn) + (te - pe) * (te - pe));
                for (auto a : wp->getVehicleActionList())
                {
                    if (afrl::cmasi::isLoiterAction(a))
                    {
                        auto la = static_cast<afrl::cmasi::LoiterAction*>(a);
                        double wn, we;
                        unitConversions.ConvertLatLong_degToNorthEast_m(
                            la->getLocation()->getLatitude(),
                            la->getLocation()->getLongitude(), wn, we);
                        double wpdist = sqrt((wn - tn) * (wn - tn) + (we - te) * (we - te));
                        if ((la->getRadius() + wpdist) > pdist)
                        {
                            // override distance if action is futher than waypoint
                            pdist = la->getRadius() + wpdist;
                        }
                    }
                }
                beyondThisTower |= (pdist > towerRange);
            }
        }
        if (!beyondThisTower)
        {
            inCommRange = true;
            break;
        }
    }

    vehicleSum->setBeyondCommRange(!inCommRange);
}


bool BatchSummaryService::LinearizeBoundary(afrl::cmasi::AbstractGeometry* boundary, std::shared_ptr<VisiLibity::Polygon>& poly)
{
    uxas::common::utilities::CUnitConversions flatEarth;
    bool isValid = false;
    poly->clear();

    if (afrl::cmasi::isPolygon(boundary))
    {
        afrl::cmasi::Polygon* boundaryPolygon = (afrl::cmasi::Polygon*) boundary;
        for (unsigned int k = 0; k < boundaryPolygon->getBoundaryPoints().size(); k++)
        {
            VisiLibity::Point pt;
            double north, east;
            flatEarth.ConvertLatLong_degToNorthEast_m(boundaryPolygon->getBoundaryPoints()[k]->getLatitude(), boundaryPolygon->getBoundaryPoints()[k]->getLongitude(), north, east);
            pt.set_x(east);
            pt.set_y(north);
            poly->push_back(pt);
        }
        isValid = true;
    }
    else if (afrl::cmasi::isRectangle(boundary))
    {
        afrl::cmasi::Rectangle* rectangle = (afrl::cmasi::Rectangle*) boundary;
        VisiLibity::Point c;
        double north, east;
        flatEarth.ConvertLatLong_degToNorthEast_m(rectangle->getCenterPoint()->getLatitude(), rectangle->getCenterPoint()->getLongitude(), north, east);
        c.set_x(east);
        c.set_y(north);

        VisiLibity::Point pt;
        // note: north-aligned positive rotation is opposite direction of x-aligned positive rotation
        double a = -rectangle->getRotation() * n_Const::c_Convert::dDegreesToRadians();
        double w = rectangle->getWidth() / 2;
        double h = rectangle->getHeight() / 2;

        poly->push_back(VisiLibity::Point::rotate(VisiLibity::Point(east + w, north + h) - c, a) + c);
        poly->push_back(VisiLibity::Point::rotate(VisiLibity::Point(east - w, north + h) - c, a) + c);
        poly->push_back(VisiLibity::Point::rotate(VisiLibity::Point(east - w, north - h) - c, a) + c);
        poly->push_back(VisiLibity::Point::rotate(VisiLibity::Point(east + w, north - h) - c, a) + c);

        isValid = true;
    }
    else if (afrl::cmasi::isCircle(boundary))
    {
        afrl::cmasi::Circle* circle = (afrl::cmasi::Circle*) boundary;
        VisiLibity::Point c;
        double north, east;
        flatEarth.ConvertLatLong_degToNorthEast_m(circle->getCenterPoint()->getLatitude(), circle->getCenterPoint()->getLongitude(), north, east);
        c.set_x(east);
        c.set_y(north);
        double r = circle->getRadius() / cos(10.0 * n_Const::c_Convert::dDegreesToRadians());

        for (uint32_t k = 0; k < 18; k++)
        {
            VisiLibity::Point pt;
            pt.set_x(c.x() + r * cos(n_Const::c_Convert::dTwoPi() * k / 18.0));
            pt.set_y(c.y() + r * sin(n_Const::c_Convert::dTwoPi() * k / 18.0));
            poly->push_back(pt);
        }

        isValid = true;
    }

    return isValid;
}

bool BatchSummaryService::AttemptMoveOutsideKoz(std::shared_ptr<afrl::cmasi::Location3D>& loc, double offset, std::shared_ptr<afrl::cmasi::EntityConfiguration> config, std::unordered_map < int64_t, std::shared_ptr< BatchSummaryService::ZonePair > > kozPairs)
{
    uxas::common::utilities::CUnitConversions unitConversions;
    VisiLibity::Point newEnd;
    bool newEndSet = false;
    for (auto kozID : kozPairs)
    {
        //enforce altitude ranges.
        auto lmcpZone = kozID.second->LmcpZone;
        auto epsilon = 1e-3;
        if (config != nullptr && 
            (lmcpZone->getMinAltitude() > config->getNominalAltitude() || lmcpZone->getMaxAltitude() < config->getNominalAltitude()) &&
            !(abs(lmcpZone->getMaxAltitude()) < epsilon && abs(lmcpZone->getMinAltitude()) < epsilon))
        {
            continue;
        }

        auto koz = kozID.second;

        VisiLibity::Point p;
        double north, east;
        unitConversions.ConvertLatLong_degToNorthEast_m(loc->getLatitude(), loc->getLongitude(), north, east);
        p.set_x(east);
        p.set_y(north);



        //check for point inside koz case
        if (p.in(*koz->VisiLibityZone))
        {
            //move the location outside the koz
            auto bounderyPoint = p.projection_onto_boundary_of(*koz->VisiLibityZone);
            auto vector = VisiLibity::Point::normalize(bounderyPoint - p) * offset;
            newEnd = bounderyPoint + vector;
            newEndSet = true;

            break;
        }
        afrl::cmasi::Polygon *poly = new afrl::cmasi::Polygon();
        auto length = offset;
        auto loiterIntersections = 0;
        VisiLibity::Point v(0,0);
        for (double rad = 0; rad < n_Const::c_Convert::dTwoPi(); rad += n_Const::c_Convert::dPiO10())
        {
            VisiLibity::Point p;
            p.set_x(east + length * cos(rad));
            p.set_y(north + length * sin(rad));
            if (p.in(*koz->VisiLibityZone))
            {
                loiterIntersections += 1;
                //move the location outside the koz
                auto bounderyPoint = p.projection_onto_boundary_of(*koz->VisiLibityZone);
                v = v + bounderyPoint;
            }
        }
        if (loiterIntersections > 0)
        {
            newEndSet = true;


            v = v * (1.0 / loiterIntersections); //division operator not supported in VisiLibity::Point
            auto vector = VisiLibity::Point::normalize(p - v) * offset;
            newEnd = v + vector;
        }
    }
    if (newEndSet)
    {
        double latitude_deg(0.0);
        double longitude_deg(0.0);
        unitConversions.ConvertNorthEast_mToLatLong_deg(newEnd.y(), newEnd.x(), latitude_deg, longitude_deg);
        loc->setLatitude(latitude_deg);
        loc->setLongitude(longitude_deg);
        return true;
    }
    return false;
}

bool BatchSummaryService::AttemptMoveOutsideKozIterate(std::shared_ptr<afrl::cmasi::Location3D>& loc, double offset, std::shared_ptr<afrl::cmasi::EntityConfiguration> config, std::unordered_map < int64_t, std::shared_ptr< BatchSummaryService::ZonePair > > kozPairs)
{
    auto maxIterations = 20;

    auto iterations = 0;
    auto fudge = .999;
    while(AttemptMoveOutsideKoz(loc, offset * fudge, config, kozPairs))
    {
        iterations += 1;
        fudge *= .999;
        if (iterations >= maxIterations)
        {
            return false;
        }

    }
    return true;
}

std::shared_ptr<VisiLibity::Polygon> BatchSummaryService::FromAbstractGeometry(afrl::cmasi::AbstractGeometry *geom)
{
    auto poly = std::shared_ptr<VisiLibity::Polygon>(new VisiLibity::Polygon);
    LinearizeBoundary(geom, poly);
    poly->eliminate_redundant_vertices(1.0);
    if (poly->area() < 0)
    {
        poly->reverse();
    }
    
    return poly;
}
}; //namespace service
}; //namespace uxas
