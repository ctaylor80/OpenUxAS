// ===============================================================================
// Authors: AFRL/RQQA
// Organization: Air Force Research Laboratory, Aerospace Systems Directorate, Power and Control Division
// 
// Copyright (c) 2017 Government of the United State of America, as represented by
// the Secretary of the Air Force.  No copyright is claimed in the United States under
// Title 17, U.S. Code.  All Other Rights Reserved.
// ===============================================================================

/*
 * File:   PayloadDropTaskService.h
 * Author: Colin
 *
 * Created on December 21, 2017
 */

#ifndef UXAS_TASK_PAYLOADDROPTASK_H
#define UXAS_TASK_PAYLOADDROPTASK_H

#include "TaskServiceBase.h"
#include "afrl/impact/PayloadDropTask.h"
#include "afrl/cmasi/AirVehicleConfiguration.h"
namespace uxas
{
namespace service
{
namespace task
{


/*! \class PayloadDropTask
    \brief A Service that implements a PayloadDropTask.


 *
 *
 *  @par To add a new task:
 * <ul style="padding-left:1em;margin-left:0">
 * <li>Make copies of the source and header files of this template.</li>
 * <li>Search for the string PayloadDropTask and Replace it with the new
 * service name.</li>
 * <li>Change the unique include guard entries, in the header file, i.e.
 * "UXAS_00_SERVICE_TEMPLATE_H" to match the new service name</li>
 * <li> include the new service header file in 00_ServiceList.h</li>
 * <li> add a dummy instance of the new service in 00_ServiceList.h, e.g.
 * {auto svc = uxas::stduxas::make_unique<uxas::service::MyNewService>();}
 * Note: this is required to register the new service when starting UxAS</li>
 *
 * </ul> @n
 *
 *
 *
 *
 * Options:
 *  - OptionString
 *  - OptionInt
 *
 * Subscribed Messages:
 *  - afrl::cmasi::KeyValuePair

 * Sent Messages:
 *  - NONE
 *
 * TASK: Subscribed Messages:
 *  - afrl::cmasi::EntityState
 *  - afrl::cmasi::EntityConfiguration
 *  - afrl::cmasi::AirVehicleState
 *  - afrl::cmasi::AirVehicleConfiguration
 *  - afrl::vehicles::GroundVehicleState
 *  - afrl::vehicles::GroundVehicleConfiguration
 *  - afrl::vehicles::SurfaceVehicleState
 *  - afrl::vehicles::SurfaceVehicleConfiguration
 *  - uxas::messages::task::UniqueAutomationRequest
 *  - uxas::messages::task::UniqueAutomationResponse
 *  - uxas::messages::route::RoutePlanResponse
 *  - uxas::messages::task::TaskImplementationRequest
 *
 * TASK: Sent Messages:
 *  - uxas::messages::task::TaskInitialized
 *  - uxas::messages::task::TaskActive
 *  - uxas::messages::task::TaskComplete
 *  - uxas::messages::route::RoutePlanRequest
 *  - uxas::messages::task::TaskPlanOptions
 *  - uxas::messages::task::TaskImplementationResponse
 */

class PayloadDropTaskService : public TaskServiceBase
{
public:

    /** \brief This string is used to identify this service in XML configuration
     * files, i.e. Service Type="YOUR_TASKS_FULL_LMCP_TYPE_NAME". It is also entered into
     * service registry and used to create new instances of this service. */
    static const std::string&
        s_typeName()
    {
        static std::string s_string("PayloadDropTask");
        return (s_string);
    };

    static const std::vector<std::string>
        s_registryServiceTypeNames()
    {
        std::vector<std::string> registryServiceTypeNames = { s_typeName(),"afrl.impact.PayloadDropTask" };
        return (registryServiceTypeNames);
    };

    /** \brief If this string is not empty, it is used to create a data
     * directory to be used by the service. The path to this directory is
     * accessed through the ServiceBase variable m_workDirectoryPath. */
    static const std::string&
        s_directoryName() { static std::string s_string(""); return (s_string); };

    static ServiceBase*
        create()
    {
        return new PayloadDropTaskService;
    };

    PayloadDropTaskService();

    virtual
        ~PayloadDropTaskService();

private:

    static
        ServiceBase::CreationRegistrar<PayloadDropTaskService> s_registrar;

    /** brief Copy construction not permitted */
    PayloadDropTaskService(PayloadDropTaskService const&) = delete;

    /** brief Copy assignment operation not permitted */
    void operator=(PayloadDropTaskService const&) = delete;

    bool configureTask(const pugi::xml_node& serviceXmlNode) override;

    bool initializeTask() override;

    bool startTask() override;

    bool terminateTask() override;

    bool processReceivedLmcpMessageTask(std::shared_ptr<avtas::lmcp::Object>& receivedLmcpObject) override;

    bool isProcessTaskImplementationRouteResponse(std::shared_ptr<uxas::messages::task::TaskImplementationResponse>& taskImplementationResponse, std::shared_ptr<TaskOptionClass>& taskOptionClass,
        int64_t& waypointId, std::shared_ptr<uxas::messages::route::RoutePlan>& route) override;

    void activeEntityState(const std::shared_ptr<afrl::cmasi::EntityState>& entityState)override { };

    void buildTaskPlanOptions()override;

    double getLevelTurnRadius(std::shared_ptr<afrl::cmasi::AirVehicleConfiguration>);

private:
    std::shared_ptr<afrl::impact::PayloadDropTask> m_payloadDrop;
    float m_radiusBufferMultiplier = 1.1;
};


}; //namespace task
}; //namespace service
}; //namespace uxas

#endif /* UXAS_TASK_PAYLOADDROPTASK_H */

