#include <chrono>
#include <fstream>
#include <iostream>
#include <numeric>
#include <thread>
#include <ompl/util/RandomNumbers.h>
#include <grstapse/common/utilities/custom_json_conversions.hpp>
#include <grstapse/task_allocation/itags/itags.hpp>
#include "cpsat_scheduler.hpp"
int main(int argc,char** argv) {
    if(argc<3 || argc>4){std::cerr<<"Usage: itags_cpsat INPUT.json RESULT.json [SECONDS]\n";return 2;}
    nlohmann::json out;int code=0;
    const auto started=std::chrono::steady_clock::now();
    try {
        const double budget=argc==4?std::stod(argv[3]):100;
        grstapse::CpSatScheduler::setBudget(budget);
        ompl::RNG::setSeed(1);
        out["stage"]="loading_inputs";
        std::ifstream in(argv[1]);if(!in)throw std::runtime_error("Cannot open input");
        nlohmann::json j;in>>j;
        auto inputs=j.get<std::shared_ptr<grstapse::ItagsProblemInputs>>();
        inputs->validate();
        out["schedule_best"]=inputs->scheduleBestMakespan();
        out["schedule_worst"]=inputs->scheduleWorstMakespan();
        if(inputs->scheduleWorstMakespan()<=inputs->scheduleBestMakespan())throw std::invalid_argument("Invalid NSQ normalization range");
        grstapse::Itags<> itags(inputs);
        out["stage"]="search";
        auto result=itags.search();
        result.statistics()->serializeToJson(out["search_statistics"]);
        if(result.foundGoal() && result.goal()->schedule()) {
            auto node=result.goal();
            auto schedule=std::dynamic_pointer_cast<const grstapse::DeterministicSchedule>(node->schedule());
            out["status"]="schedule_found_requires_motion_validation";
            out["makespan"]=schedule->makespan();out["allocation"]=node->allocation();
            out["timepoints"]=schedule->timepoints();out["mutex_orderings"]=schedule->precedenceSetMutexConstraints();
            out["robot_plans"]=nlohmann::json::array();
            for(unsigned int r=0;r<inputs->numberOfRobots();++r) {
                std::vector<unsigned int> tasks;
                for(unsigned int t=0;t<inputs->numberOfPlanTasks();++t)if(node->allocation()(t,r))tasks.push_back(t);
                std::sort(tasks.begin(),tasks.end(),[&](auto a,auto b){return std::pair(schedule->timepoints()[a].first,a)<std::pair(schedule->timepoints()[b].first,b);});
                out["stage"]="export_robot";
                nlohmann::json rp=nlohmann::json::object();
                rp["robot"]=r;rp["name"]=inputs->robot(r)->name();rp["tasks"]=tasks;
                rp["transitions"]=nlohmann::json::array();
                auto previous=inputs->robot(r)->initialConfiguration();
                for(auto t:tasks) {
                    auto task=inputs->planTask(t);
                    auto path=inputs->robot(r)->motionPlanningQuery(previous,task->initialConfiguration());
                    out["stage"]="export_waypoints";
                    nlohmann::json waypoints;path->serializeToJson(waypoints);
                    out["stage"]="export_leg";
                    nlohmann::json leg={{"waypoints",waypoints},{"to_task",t},
                                       {"duration",path->duration(inputs->robot(r)->speed())}};
                    rp["transitions"].push_back(leg);previous=task->terminalConfiguration();
                }
                out["robot_plans"].push_back(rp);
                out["stage"]="complete";
            }
        } else {out["status"]="no_schedule_returned";code=3;}
    } catch(const grstapse::CpSatBudgetExceeded& e) {out["status"]="wall_budget_exhausted";out["error"]=e.what();code=4;}
      catch(const std::exception& e) {out["status"]="exception";out["error"]=e.what();code=5;}
    out["scheduler_statistics"]=grstapse::CpSatScheduler::diagnostics();
    out["elapsed_seconds"]=std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count();
    out["motion_validated"]=false;
    out["implementation"]="Upstream C++ Itags allocation search + native C++ OR-Tools CP-SAT";
    std::ofstream result(argv[2]);result<<out.dump(2)<<'\n';
    std::cout<<out.value("status",std::string("unknown"))<<" "<<out["elapsed_seconds"]<<std::endl;
    return code;
}
