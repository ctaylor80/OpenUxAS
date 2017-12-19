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
* Author: colin
*/

#ifndef UXAS_TASK_DYNAMIC_TASK_SERVICE_BASE_H
#define UXAS_TASK_DYNAMIC_TASK_SERVICE_BASE_H

#include "TaskServiceBase.h"
#include "ServiceBase.h"
#include "TimeUtilities.h"
#include "visilibity.h"
#include "UnitConversions.h"

namespace uxas
{
namespace service
{
namespace task
{
    /**
     * \class DynamicTaskServiceBase
     * 
     * Builds on TaskServiecBase to provide functionality for tasks whose waypoints will change.
     * 
     * Dynamic Tasks are considered as immediately on active. DynamicTasks define a target location
     * per entity. The base class handles querying the route planner to honor keep in and keep out zones.
     */
    class DynamicTaskServiceBase : public TaskServiceBase
    {
    public :
        DynamicTaskServiceBase(const std::string& typeName, const std::string& directoryName);


    protected :
        int64_t m_straightLineThreshold_m = 400;
        double m_startPointLead_m = 50.0;        
        std::unordered_map<int64_t, std::shared_ptr<afrl::cmasi::Location3D>> m_targetLocations;
        int64_t m_throttle_ms = 5000;

    private :
        //methods overriden from TaskServiceBase
        void activeEntityState(const std::shared_ptr<afrl::cmasi::EntityState>& entityState) override;
        bool processReceivedLmcpMessageTask(std::shared_ptr<avtas::lmcp::Object>& receivedLmcpObject)  override;
        bool configureTask(const pugi::xml_node& serviceXmlNode) override;

        //methods for children to implement
        virtual std::shared_ptr<afrl::cmasi::Location3D> calculateTargetLocation(const std::shared_ptr<afrl::cmasi::EntityState> entityState) { return nullptr; };
        virtual bool processRecievedLmcpMessageDynamicTask(std::shared_ptr<avtas::lmcp::Object>& receivedLmcpObject) { return false; };
        virtual void processMissionCommand(std::shared_ptr<afrl::cmasi::MissionCommand>) {};
        virtual bool configureDynamicTask(const pugi::xml_node& serviceXmlNode) { return true; };

        std::unordered_map<int64_t, int64_t> m_throttle;
        std::unordered_map<int64_t, int64_t> m_entityIdVsLastWaypoint;
    };
}
}
}

#endif
