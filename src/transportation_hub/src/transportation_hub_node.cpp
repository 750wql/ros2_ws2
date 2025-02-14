//
// Created by wuqilin on 25-2-12.
//
#include <rclcpp/rclcpp.hpp>
#include <judger_interfaces/msg/overall_info.hpp>
#include <judger_interfaces/msg/road_info.hpp>
#include <judger_interfaces/srv/my_service.hpp>
#include <vector>
#include <map>
#include <queue>
#include <climits>
#include <chrono>
#include <memory>

using namespace std;

class TransportationHubNode : public rclcpp::Node {
public:
    TransportationHubNode() : Node("transportation_hub_node")
    {
        // 订阅 `question` 话题，接收城市和道路信息
        subscription_ = this->create_subscription<judger_interfaces::msg::OverallInfo>(
            "question", 10, std::bind(&TransportationHubNode::handle_message, this, std::placeholders::_1));

        // 创建服务客户端
        std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("my_service_client");
        rclcpp::Client<judger_interfaces::srv::MyService>::SharedPtr client=
            node->create_client<judger_interfaces::srv::MyService>("my_service");
        client = node->create_client<judger_interfaces::srv::MyService>("judger_server");
        RCLCPP_INFO(this->get_logger(), "Transportation Hub Node Started.");
    }

private:
    rclcpp::Subscription<judger_interfaces::msg::OverallInfo>::SharedPtr subscription_;
    rclcpp::Client<judger_interfaces::srv::MyService>::SharedPtr client_;

    void handle_message(const judger_interfaces::msg::OverallInfo::SharedPtr msg)
    {
        int number_of_cities = msg->number_of_cities;
        int src_city = msg->src_city;
        int des_city = msg->des_city;

        RCLCPP_INFO(this->get_logger(), "Received map with %d cities and %lu roads.", number_of_cities, msg->infos.size());
        RCLCPP_INFO(this->get_logger(), "Finding shortest path from %d to %d.", src_city, des_city);

        if (src_city < 0 || src_city >= number_of_cities || des_city < 0 || des_city >= number_of_cities)
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid source or destination city: src=%d, des=%d", src_city, des_city);
            return;
        }

        map<int, vector<pair<int, int>>> graph;
        for (const auto &road : msg->infos)
        {
            if (road.source < 0 || road.source >= number_of_cities ||
                road.destination < 0 || road.destination >= number_of_cities)
            {
                RCLCPP_ERROR(this->get_logger(), "Invalid road detected: source=%d, destination=%d, length=%d",
                         road.source, road.destination, road.length);
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Adding road from %d to %d with length %d",
                    road.source, road.destination, road.length);

            graph[road.source].emplace_back(road.destination, road.length);
            graph[road.destination].emplace_back(road.source, road.length);
            RCLCPP_INFO(this->get_logger(), "Graph constructed:");
            for (const auto &city : graph)
            {
                std::string roads;
                for (const auto &neighbor : city.second)
                {
                    roads += "(" + std::to_string(neighbor.first) + ", " + std::to_string(neighbor.second) + ") ";
                }
                RCLCPP_INFO(this->get_logger(), "City %d -> %s", city.first, roads.c_str());
            }

        }

        RCLCPP_INFO(this->get_logger(), "Initializing Dijkstra algorithm...");
        vector<int> dist(number_of_cities, INT_MAX);
        vector<int> prev(number_of_cities, -1);
        RCLCPP_INFO(this->get_logger(), "Prev array initialized, size = %lu", prev.size());

        dist[src_city] = 0;
        priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;
        pq.push({0, src_city});
        RCLCPP_INFO(this->get_logger(), "Priority queue initialized with first node: %d", pq.top().second);

        if (pq.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Priority queue is unexpectedly empty at start.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Priority queue initialized. First node: %d", pq.top().second);

        while (!pq.empty())
        {
            int u = pq.top().second;
            int d = pq.top().first;
            pq.pop();

            RCLCPP_INFO(this->get_logger(), "Processing node %d with current distance %d.", u, d);
            if (graph.count(u) == 0) continue;

            for (const auto &neighbor : graph[u])
            {
                int v = neighbor.first, weight = neighbor.second;
                if (dist[u] + weight < dist[v])
                {
                    dist[v] = dist[u] + weight;
                    prev[v] = u;
                    pq.push({dist[v], v});
                }
            }
        }

        if (prev[des_city] == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "No valid path found from %d to %d.", src_city, des_city);
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Reconstructing path from destination %d.", des_city);
        vector<int> path;
        for (int at = des_city; at != -1; at = prev[at]) path.push_back(at);
        reverse(path.begin(), path.end());

        if (!client_) {
            RCLCPP_WARN(this->get_logger(), "Service client was not initialized. Initializing now...");
            client_ = rclcpp::Node::make_shared("my_service_client")
                          ->create_client<judger_interfaces::srv::MyService>("my_service");
        }

        string path_str = "Shortest path: ";
        for (int city : path) {
            path_str += to_string(city) + " ";
        }
        RCLCPP_INFO(this->get_logger(), "%s", path_str.c_str());

        judger_interfaces::msg::MyAnswer answer;
        for (int city : path) {
            answer.my_answer.push_back(city);  // 将路径上的每个城市加入 `my_answer`
        }

        // 创建请求并将 `answer` 设置为请求内容
        auto request = std::make_shared<judger_interfaces::srv::MyService::Request>();
        request->answer = answer;  // 将 `Answer` 对象赋给请求的 `answer` 字段

        // 发送请求
        auto future = client_->async_send_request(request);



        RCLCPP_INFO(this->get_logger(), "Shortest path found. Sending to Judger...");
    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TransportationHubNode>());
    rclcpp::shutdown();
    return 0;
}
