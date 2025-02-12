//
// Created by wuqilin on 25-2-12.
//
#include <rclcpp/rclcpp.hpp>
#include <transportation_msgs/msg/overall_info.hpp>
#include <transportation_msgs/srv/shortest_path.hpp>
#include <vector>
#include <map>
#include <queue>
#include <climits>

using namespace std;

class TransportationHubNode : public rclcpp::Node
{
public:
    TransportationHubNode() : Node("transportation_hub_node")
    {
        // 订阅question话题，获取道路和城市信息
        subscription_ = this->create_subscription<transportation_msgs::msg::OverallInfo>(
            "question", 10, std::bind(&TransportationHubNode::handle_message, this, std::placeholders::_1));

        // 服务客户端，用于发送计算结果
        client_ = this->create_client<transportation_msgs::srv::ShortestPath>("judger_server");
    }

private:
    rclcpp::Subscription<transportation_msgs::msg::OverallInfo>::SharedPtr subscription_;
    rclcpp::Client<transportation_msgs::srv::ShortestPath>::SharedPtr client_;

    void handle_message(const transportation_msgs::msg::OverallInfo::SharedPtr msg)
    {
        // 解析道路信息并构建图
        int number_of_cities = msg->number_of_cities;
        map<int, vector<pair<int, int>>> graph;  // 存储图：城市 -> {目标城市, 路径长度}

        for (const auto &road : msg->road_info)
        {
            graph[road.source].push_back({road.destination, road.length});
            graph[road.destination].push_back({road.source, road.length});
        }

        int src_city = msg->src_city;
        int des_city = msg->des_city;

        // 使用Dijkstra算法找到最短路径
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

            if (d > dist[u])
                continue;

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

        // 通过服务发送结果
        if (client_->wait_for_service(std::chrono::seconds(1)))
        {
            auto request = std::make_shared<transportation_msgs::srv::ShortestPath::Request>();

            request->src_city = src_city;  // 设置源城市
            request->des_city = des_city; // 设置目标城市

            auto result_future = client_->async_send_request(request);
            try
            {
                auto result = result_future.get();
                if (!result->path.empty())  // 判断路径是否为空，表示成功
                {
                    RCLCPP_INFO(this->get_logger(), "Shortest path: ");
                    for (int city : result->path)
                    {
                        RCLCPP_INFO(this->get_logger(), "%d", city);
                    }
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "Failed to find a path.");
                }
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to send result: %s", e.what());
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Judger server not available.");
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
