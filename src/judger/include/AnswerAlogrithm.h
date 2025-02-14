//
// Created by lbw on 25-1-4.
//
#ifndef ANSWER_ALOGRITHM_H
#define ANSWER_ALOGRITHM_H
//std
#include <iostream>
//project
#include "judger_interfaces/msg/overall_info.hpp"
#include "judger_interfaces/msg/my_answer.hpp"

class AnswerAlgorithm {
    using Question = judger_interfaces::msg::OverallInfo;
    using Answer = judger_interfaces::msg::MyAnswer;
public:
    AnswerAlgorithm() = default;

    ~AnswerAlgorithm() = default;

    bool solve(Question& question,const Answer& listened_answer);
};

#endif //ANSWER_ALOGRITHM_H