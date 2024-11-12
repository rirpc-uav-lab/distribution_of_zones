#include "rclcpp/rclcpp.hpp"
#include "uav_msgs/srv/zone.hpp"
#include "geometry_msgs/msg/point32.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class ClientDistribution : public rclcpp::Node
{
    public:
    ClientDistribution()
    : Node("client_distribution")
    {
        rclcpp::Client<uav_msgs::srv::Zone>::SharedPtr client = this->create_client<uav_msgs::srv::Zone>("zones");
        request = std::make_shared<uav_msgs::srv::Zone::Request>();

    ///////ЗОНА 0

        request->zones.emplace_back();

        ///////ТОЧКА А
        geometry_msgs::msg::Point32 point;
        point.x = 6.0;
        point.y = 16.0;
        request->zones.at(0).points.push_back(point);

        ///////ТОЧКА D
        
        point.x = 12.0;
        point.y = 16.0;
        request->zones.at(0).points.push_back(point);
        ///////ТОЧКА B
        
        point.x = 12.0;
        point.y = 10.0;
        request->zones.at(0).points.push_back(point);

        ///////ТОЧКА C
        
        point.x = 6.0;
        point.y = 10.0;
        request->zones.at(0).points.push_back(point);

    ///////ЗОНА 1

        request->zones.emplace_back();
        
        ///////ТОЧКА E
        
        point.x = 18.0;
        point.y = 16.0;
        request->zones.at(1).points.push_back(point);

        ///////ТОЧКА H
        
        point.x = 22.0;
        point.y = 18.0;
        request->zones.at(1).points.push_back(point);

        ///////ТОЧКА P1
        
        point.x = 26.0;
        point.y = 16.0;
        request->zones.at(1).points.push_back(point);

        ///////ТОЧКА G
        
        point.x = 26.0;
        point.y = 10.0;
        request->zones.at(1).points.push_back(point);

        ///////ТОЧКА F
        
        point.x = 18.0;
        point.y = 10.0;
        request->zones.at(1).points.push_back(point);

    ///////ЗОНА 2

        request->zones.emplace_back();
        ///////ТОЧКА I
        
        
        point.x = 30.0;
        point.y = 16.0;
        request->zones.at(2).points.push_back(point);

        ///////ТОЧКА K
        
        point.x = 38.0;
        point.y = 18.0;
        request->zones.at(2).points.push_back(point);

        ///////ТОЧКА L
        
        point.x = 34.0;
        point.y = 10.0;
        request->zones.at(2).points.push_back(point);

        ///////ТОЧКА J
        
        point.x = 30.0;
        point.y = 10.0;
        request->zones.at(2).points.push_back(point);

    // ///////ЗОНА 3 

    //     request->zones.emplace_back();
    //     ///////ТОЧКА T
        
        
    //     point.x = 12;
    //     point.y = 28;
    //     request->zones.at(3).points.push_back(point);

    //     ///////ТОЧКА W
        
    //     point.x = 18;
    //     point.y = 26;
    //     request->zones.at(3).points.push_back(point);

    //     ///////ТОЧКА V
        
    //     point.x = 18;
    //     point.y = 22;
    //     request->zones.at(3).points.push_back(point);

    //     ///////ТОЧКА U
        
    //     point.x = 12;
    //     point.y = 22;
    //     request->zones.at(3).points.push_back(point);  

    // ///////ЗОНА 4

    //     request->zones.emplace_back();
    //     ///////ТОЧКА Z
        
        
    //     point.x = 26;
    //     point.y = 24;
    //     request->zones.at(4).points.push_back(point);

    //     ///////ТОЧКА C1
        
    //     point.x = 28;
    //     point.y = 28;
    //     request->zones.at(4).points.push_back(point);

    //     ///////ТОЧКА B1
        
    //     point.x = 28;
    //     point.y = 22;
    //     request->zones.at(4).points.push_back(point);

    //     ///////ТОЧКА A1
        
    //     point.x = 26;
    //     point.y = 22;
    //     request->zones.at(4).points.push_back(point); 

    // ///////ЗОНА 5

    //     request->zones.emplace_back();
    //     ///////ТОЧКА R
        
        
    //     point.x = 38;
    //     point.y = 30;
    //     request->zones.at(5).points.push_back(point);

    //     ///////ТОЧКА Q
        
    //     point.x = 42;
    //     point.y = 30;
    //     request->zones.at(5).points.push_back(point);

    //     ///////ТОЧКА P
        
    //     point.x = 42;
    //     point.y = 26;
    //     request->zones.at(5).points.push_back(point);

    //     ///////ТОЧКА S
        
    //     point.x = 38;
    //     point.y = 26;
    //     request->zones.at(5).points.push_back(point); 

    // ///////ЗОНА 6

    //     request->zones.emplace_back();
    //     ///////ТОЧКА G1
        
        
    //     point.x = 12;
    //     point.y = 44;
    //     request->zones.at(6).points.push_back(point);

    //     ///////ТОЧКА Q1
        
    //     point.x = 18;
    //     point.y = 44;
    //     request->zones.at(6).points.push_back(point);

    //     ///////ТОЧКА R1
        
    //     point.x = 20;
    //     point.y = 40;
    //     request->zones.at(6).points.push_back(point);

    //     ///////ТОЧКА F1
        
    //     point.x = 18;
    //     point.y = 32;
    //     request->zones.at(6).points.push_back(point); 

    //     ///////ТОЧКА E1
        
    //     point.x = 10;
    //     point.y = 32;
    //     request->zones.at(6).points.push_back(point);

    //     ///////ТОЧКА D1
        
    //     point.x = 10;
    //     point.y = 40;
    //     request->zones.at(6).points.push_back(point);

    // ///////ЗОНА 7

    //     request->zones.emplace_back();
    //     ///////ТОЧКА K1
        
        
    //     point.x = 25;
    //     point.y = 40;
    //     request->zones.at(7).points.push_back(point);

    //     ///////ТОЧКА J1
        
    //     point.x = 30;
    //     point.y = 40;
    //     request->zones.at(7).points.push_back(point);

    //     ///////ТОЧКА I1
        
    //     point.x = 30;
    //     point.y = 35;
    //     request->zones.at(7).points.push_back(point);

    //     ///////ТОЧКА H1
        
    //     point.x = 25;
    //     point.y = 35;
    //     request->zones.at(7).points.push_back(point); 

    // ///////ЗОНА 8

    //     request->zones.emplace_back();
    //     ///////ТОЧКА L1
        
        
    //     point.x = 44;
    //     point.y = 36;
    //     request->zones.at(8).points.push_back(point);

    //     ///////ТОЧКА O1
        
    //     point.x = 48;
    //     point.y = 38;
    //     request->zones.at(8).points.push_back(point);

    //     ///////ТОЧКА N1
        
    //     point.x = 46;
    //     point.y = 34;
    //     request->zones.at(8).points.push_back(point);

    //     ///////ТОЧКА M1
        
    //     point.x = 44;
    //     point.y = 34;
    //     request->zones.at(8).points.push_back(point); 

        int drone_count = 3;
        for (int drone_number = 0; drone_number < drone_count; ++drone_number)
        {
            request->odom.emplace_back();
        }

        request->odom.at(0).header.frame_id = "0";
        request->odom.at(1).header.frame_id = "1";
        request->odom.at(2).header.frame_id = "2";

        request->odom.at(0).pose.pose.position.x = 18.0;
        request->odom.at(0).pose.pose.position.y = 2.0;
        
        request->odom.at(1).pose.pose.position.x = 21.0;
        request->odom.at(1).pose.pose.position.y = 2.0;

        request->odom.at(2).pose.pose.position.x = 24.0;
        request->odom.at(2).pose.pose.position.y = 2.0;

        while (!client->wait_for_service(1s)) {
            if (!rclcpp::ok()) 
            {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            // return 0;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
        }

        auto result = client->async_send_request(request);
        // Wait for the result.
        if (rclcpp::spin_until_future_complete(shared_from_this(), result) == rclcpp::FutureReturnCode::SUCCESS)
        // auto this_p = std::make_shared<ClientDistribution>(this);
        // if (rclcpp::spin_until_future_complete(this_p, result) == rclcpp::FutureReturnCode::SUCCESS)
        {
            for (int drone_number = 0; drone_number < result.get()->zones_for_drones_array.size(); ++drone_number)
            {

                RCLCPP_INFO(this->get_logger(), "Current drone number:%i", drone_number);
                for (int zone_number = 0; zone_number < result.get()->zones_for_drones_array.at(drone_number).zones.size(); ++zone_number)
                {

                    RCLCPP_INFO(this->get_logger(), "   Current zone number:%i", zone_number);
                    for (int point_number = 0; point_number < result.get()->zones_for_drones_array.at(drone_number).zones.at(zone_number).points.size(); ++point_number)
                    {
                        int i = 0;
                        
                        RCLCPP_INFO(this->get_logger(), "       Point:%i  %f", i, result.get()->zones_for_drones_array.at(drone_number).zones.at(zone_number).points.at(point_number).x);
                        RCLCPP_INFO(this->get_logger(), "       Point:%i  %f", i, result.get()->zones_for_drones_array.at(drone_number).zones.at(zone_number).points.at(point_number).y);
                        RCLCPP_INFO(this->get_logger(), "       Point:%i  %f", i, result.get()->zones_for_drones_array.at(drone_number).zones.at(zone_number).points.at(point_number).z);
                        RCLCPP_INFO_STREAM(this->get_logger(), "");

                        ++i;
                    }
                }
            }
        } 
        
        else 
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
        }

    }
    
    private:

    std::shared_ptr<uav_msgs::srv::Zone::Request> request;

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ClientDistribution>());
    rclcpp::shutdown();
    return 0;
}
