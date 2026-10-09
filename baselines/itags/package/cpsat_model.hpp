#pragma once
#include <cstdint>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>
namespace grstapse {
struct TimedEdge { int from, to; int64_t travel; };
struct TimedMutex { int first, second; int64_t forward, reverse; };
struct NumericScheduleProblem {
    std::vector<int64_t> durations, releases;
    std::vector<TimedEdge> precedences;
    std::vector<TimedMutex> mutexes;
    double timeout = 10.0, relative_gap = 0.1;
    int workers = 4;
    bool hierarchical = true;
};
struct NumericScheduleResult {
    bool accepted = false;
    std::string primary_status, secondary_status = "not_requested";
    int64_t makespan = 0;
    double primary_bound = 0;
    std::vector<int64_t> starts, ends;
    std::vector<std::pair<unsigned int,unsigned int>> orderings;
    int solver_calls = 0;
};
NumericScheduleResult solveNumericSchedule(const NumericScheduleProblem& p);
NumericScheduleProblem numericFromJson(const nlohmann::json& j);
nlohmann::json numericResultJson(const NumericScheduleResult& r);
}
