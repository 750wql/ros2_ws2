//
// Created by wuqilin on 25-2-12.
//
#include <rclcpp/rclcpp.hpp>
#include <transportation_msgs/msg/overall_info.hpp>
#include <transportation_msgs/srv/shortest_path.hpp>
#include <judger_interfaces/msg/overall_info.hpp>
#include <judger_interfaces/msg/road_info.hpp>
#include <judger_interfaces/srv/my_service.hpp>
#include <vector>
#include <map>
#include <queue>
#include <climits>

using namespace std;

class TransportationHubNode : public rclcpp::Node {
public:
    TransportationHubNode() : Node("transportation_hub_node")
    {
        // 订阅 `question` 话题，接收城市和道路信息
        subscription_ = this->create_subscription<transportation_msgs::msg::OverallInfo>(
            "question", 10, std::bind(&TransportationHubNode::handle_message, this, std::placeholders::_1));

        // 创建服务客户端
        client_ = this->create_client<transportation_msgs::srv::ShortestPath>("judger_server");

        RCLCPP_INFO(this->get_logger(), "Transportation Hub Node Started.");
    }

private:
    rclcpp::Subscription<transportation_msgs::msg::OverallInfo>::SharedPtr subscription_;
    rclcpp::Client<transportation_msgs::srv::ShortestPath>::SharedPtr client_;

    void handle_message(const transportation_msgs::msg::OverallInfo::SharedPtr msg)
    {
        int number_of_cities = msg->number_of_cities;
        int src_city = msg->src_city;
        int des_city = msg->des_city;

        RCLCPP_INFO(this->get_logger(), "Received map with %d cities and %lu roads.", number_of_cities, msg->infos.size());
        RCLCPP_INFO(this->get_logger(), "Finding shortest path from %d to %d.", src_city, des_city);

        map<int, vector<pair<int, int>>> graph;  // 存储无向图
        for (const auto &road : msg->infos)
        {
            graph[road.source].emplace_back(road.destination, road.length);
            graph[road.destination].emplace_back(road.source, road.length);
        }

        // Dijkstra 计算最短路径
        vector<int> dist(number_of_cities, INT_MAX);
        vector<int> prev(number_of_cities, -1);
        dist[src_city] = 0;
        priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;
        pq.push({0, src_city});

        while (!pq.empty())
        {
            int u = pq.top().second;
            int d = pq.top().first;
            pq.pop();

            if (d > dist[u]) continue;

            for (const auto &neighbor : graph[u])
            {
                int v = neighbor.first;
                int weight = neighbor.second;

                if (dist[u] + weight < dist[v])
                {
                    dist[v] = dist[u] + weight;
                    prev[v] = u;
                    pq.push({dist[v], v});
                }
            }
        }

        // 回溯路径
        vector<int> path;
        for (int at = des_city; at != -1; at = prev[at])
        {
            path.push_back(at);
        }
        reverse(path.begin(), path.end());

        if (path.empty() || path.front() != src_city)
        {
            RCLCPP_ERROR(this->get_logger(), "No valid path found from %d to %d.", src_city, des_city);
            return;
        }

        // 发送服务请求
        if (!client_->wait_for_service(std::chrono::seconds(2)))
        {
            RCLCPP_ERROR(this->get_logger(), "Judger server not available.");
            return;
        }

        auto request = std::make_shared<transportation_msgs::srv::ShortestPath_Request>();
        request->src_city = src_city;
        request->des_city = des_city;


        auto result_future = client_->async_send_request(request);

        try
        {
            auto result = result_future.get();
            if (!result->path.empty())
            {
                RCLCPP_INFO(this->get_logger(), "Shortest path found:");
                for (int city : result->path)
                {
                    RCLCPP_INFO(this->get_logger(), "%d", city);
                }
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Judger server did not return a valid path.");
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error during service request: %s", e.what());
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TransportationHubNode>());
    rclcpp::shutdown();
    return 0;
}