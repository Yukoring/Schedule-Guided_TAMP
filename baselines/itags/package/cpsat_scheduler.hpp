#pragma once
#include <chrono>
#include <stdexcept>
#include <grstapse/scheduling/scheduler_base.hpp>
#include <grstapse/scheduling/scheduler_parameters.hpp>
namespace grstapse {
struct CpSatSchedulerParameters : SchedulerParameters {
    CpSatSchedulerParameters():SchedulerParameters(SchedulerType::e_cpsat){}
    double timeout=10.0, relative_gap=0.1;
    int workers=4;
    int64_t time_scale=1000;
    bool hierarchical=true, transition_heuristics=false;
    static std::shared_ptr<const CpSatSchedulerParameters> fromJson(const nlohmann::json& j);
};
struct CpSatBudgetExceeded : std::runtime_error { CpSatBudgetExceeded():std::runtime_error("Overall wall-time budget exhausted"){} };
class CpSatScheduler : public SchedulerBase {
public:
    explicit CpSatScheduler(const std::shared_ptr<const SchedulerProblemInputs>& p):SchedulerBase(p){}
    static void setBudget(double seconds);
    static double remaining();
    static unsigned int numIterations();
    static nlohmann::json diagnostics();
protected:
    std::shared_ptr<const ScheduleBase> computeSchedule() override;
};
}
