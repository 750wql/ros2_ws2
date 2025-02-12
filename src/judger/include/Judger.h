//
// Created by lbw on 25-1-4.
//

#ifndef JUDGER_JUDGER_H
#define JUDGER_JUDGER_H
//std
#include <iostream>
#include <ctime>
#include <chrono>
#include <queue>
#include <set>
#include <algorithm>
#include <thread>

//ros
#include <rclcpp/rclcpp.hpp>

//project
#include "AnswerAlogrithm.h"
#include "Details.h"
#include "judger_interfaces/msg/overall_info.hpp"
#include "judger_interfaces/msg/my_answer.hpp"
#include "judger_interfaces/srv/my_service.hpp"

class Judger
    : public rclcpp::Node
{
    using Question = judger_interfaces::msg::OverallInfo;
    using Answer = judger_interfaces::msg::MyAnswer;
    using Road = judger_interfaces::msg::RoadInfo;
    using Server = judger_interfaces::srv::MyService;

private:
    int score_;
    Question question_;
    std::queue<Question> quiz_library_;
    Answer answer_;
    AnswerAlgorithm solver_;

    std::vector<CityDetails> cities_;

    int quz_count_;

    rclcpp::Publisher<Question>::SharedPtr question_pub_;
    rclcpp::Service<Server>::SharedPtr question_service_;
    rclcpp::TimerBase::SharedPtr question_timer_;

    void service_callback(const Server::Request::SharedPtr request,
                          const Server::Response::SharedPtr response);
    void publish_question();

    Question genQuestion();
    static inline float distance(CityDetails c1y, CityDetails c2);

public:
    Judger();
    Judger(Judger& judger) = delete;
    Judger& operator=(Judger& judger) = delete;
};


#endif //JUDGER_JUDGER_H
