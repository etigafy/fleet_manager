#include "TaskPlanner.h"


// ------------------------------------------------------------------------------------------------
void TaskPlanner::callback_statusRequest(const std::shared_ptr<PlannerApi::Request> request, std::shared_ptr<PlannerApi::Response> response)
{
    switch (request->type)
    {
    case 1:
        // reset deliveryMap
        deliveryMap.clear();
        printf("-> deliveryMap reset!\n");
        break;

    case 2:
        // reset clientMap
        clientMap.clear();

        // reset landmark bookings
        for(auto e : landmarkMap)
        {
            for(auto u : e.second)
                u.second->isOccupiedBy = 0;
        }

        printf("-> clientMap and landmarks reset!\n");
        break;

    default:
        break;
    }

    response->taskcomplete = true;
}

// ------------------------------------------------------------------------------------------------
void TaskPlanner::callback_status()
{
    auto msg = h_interfaces::msg::Status();
    msg.num_deliveries = deliveryMap.size();
    msg.num_clients = clientMap.size();

    // push clientMap
    h_interfaces::msg::ClientObject client;
    for(auto e : clientMap)
    {
        client.id = e.first;
        client.status = e.second->status;
        client.isonline = e.second->isOnline;
        client.isidle = e.second->isIdle;
        client.ischarging = e.second->isCharging;
        client.batterypercent = e.second->batteryPercent;
        client.activedeliveryid = e.second->activeDeliveryID;
        client.activetaskgroupid = e.second->activeTaskGroupID;

        msg.clients.push_back(client);
    }

    // push deliveryMap
    h_interfaces::msg::DeliveryObject delivery;
    for(auto e : deliveryMap)
    {
        delivery.id = e.first;
        for(auto tg : e.second)
        {
            delivery.groupid.push_back(tg.second->groupID);
            delivery.iscomplete.push_back(tg.second->isComplete);
            delivery.ishandledby.push_back(tg.second->isHandledBy);
            delivery.isprocessing.push_back(tg.second->isProcessing);
            delivery.processingtime.push_back(tg.second->processingTime.count());
        }

        msg.deliveries.push_back(delivery);
    }

    publisher_status->publish(msg);
}