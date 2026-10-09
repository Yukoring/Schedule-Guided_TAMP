#include "cpsat_scheduler.hpp"
#include "cpsat_model.hpp"
#include <algorithm>
#include <cmath>
#include <map>
#include <numeric>
#include <limits>
#include <set>
#include <grstapse/scheduling/scheduler_problem_inputs.hpp>
#include <grstapse/scheduling/milp/deterministic/deterministic_schedule.hpp>
#include <grstapse/geometric_planning/configuration_base.hpp>
#include <grstapse/robot.hpp>
#include <grstapse/task.hpp>
namespace grstapse {
namespace {
auto deadline=std::chrono::steady_clock::time_point::max();
unsigned int evaluations=0,rounds=0,solver_calls=0,accepted=0;
std::map<std::string,unsigned int> statuses;
// Separate unknown, successful, and failed transitions; never treat a failed
// motion query's -1 return value as a valid negative travel duration.
struct Transition { double duration=0; bool exact=false, failed=false; };
}
std::shared_ptr<const CpSatSchedulerParameters> CpSatSchedulerParameters::fromJson(const nlohmann::json& j) {
    auto p=std::make_shared<CpSatSchedulerParameters>();
    p->timeout=j.value("timeout",10.0); p->workers=j.value("threads",4);
    p->relative_gap=j.value("relative_gap",0.1); p->time_scale=j.value("time_scale",1000LL);
    p->hierarchical=j.value("use_hierarchical_objective",true);
    p->transition_heuristics=j.value("compute_transition_duration_heuristic",false);
    if(p->timeout<=0 || p->workers<1 || p->time_scale<1 || p->relative_gap<0) throw std::invalid_argument("Invalid CP-SAT parameters");
    return p;
}
void CpSatScheduler::setBudget(double seconds) { deadline=std::chrono::steady_clock::now()+std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(seconds)); }
double CpSatScheduler::remaining() {
    double r=std::chrono::duration<double>(deadline-std::chrono::steady_clock::now()).count();
    if(r<=0) throw CpSatBudgetExceeded();
    return r;
}
unsigned int CpSatScheduler::numIterations(){return rounds;}
nlohmann::json CpSatScheduler::diagnostics(){return {{"schedule_evaluations",evaluations},{"refinement_rounds",rounds},{"cp_sat_calls",solver_calls},{"accepted_schedules",accepted},{"solver_statuses",statuses}};}
std::shared_ptr<const ScheduleBase> CpSatScheduler::computeSchedule() {
    ++evaluations; remaining();
    auto params=std::dynamic_pointer_cast<const CpSatSchedulerParameters>(m_problem_inputs->schedulerParameters());
    if(!params) throw std::invalid_argument("CP-SAT parameters required");
    const int n=m_problem_inputs->numberOfPlanTasks(), nr=m_problem_inputs->numberOfRobots();
    const auto& allocation=m_problem_inputs->allocation();
    std::vector<std::vector<unsigned int>> owners(n);
    std::vector<int64_t> durations;
    auto ticks=[&](double x)->int64_t {
        if(!std::isfinite(x) || x<0) throw std::invalid_argument("Invalid duration");
        return static_cast<int64_t>(std::ceil(x*params->time_scale));
    };
    for(int i=0;i<n;++i) {
        remaining();
        std::vector<std::shared_ptr<const Robot>> coalition;
        for(int r=0;r<nr;++r) if(allocation(i,r)!=0) {owners[i].push_back(r); coalition.push_back(m_problem_inputs->robot(r));}
        double d=m_problem_inputs->planTask(i)->computeDuration(coalition);
        if(d<0 || !std::isfinite(d)) {++s_num_failures;return nullptr;}
        durations.push_back(ticks(d));
    }
    std::vector<std::vector<Transition>> initial(n,std::vector<Transition>(nr));
    std::vector<std::vector<std::vector<Transition>>> transit(n,std::vector<std::vector<Transition>>(n,std::vector<Transition>(nr)));
    auto init_transition=[&](Transition& t,int r,const auto& from,const auto& to) {
        if(!params->transition_heuristics) return;
        auto robot=m_problem_inputs->robot(r);
        if(robot->isMemoized(from,to)) { t.duration=robot->durationQuery(from,to); t.exact=true; t.failed=t.duration<0; }
        else t.duration=from->euclideanDistance(to)/robot->speed();
    };
    for(int i=0;i<n;++i) for(int r:owners[i]) {
        init_transition(initial[i][r],r,m_problem_inputs->robot(r)->initialConfiguration(),m_problem_inputs->planTask(i)->initialConfiguration());
        for(int j=0;j<n;++j) if(i!=j && allocation(j,r)!=0)
            init_transition(transit[i][j][r],r,m_problem_inputs->planTask(i)->terminalConfiguration(),m_problem_inputs->planTask(j)->initialConfiguration());
    }
    auto edge=[&](int i,int j)->std::pair<bool,int64_t> {
        double d=0;
        for(int r:owners[i]) if(allocation(j,r)!=0) {const auto& t=transit[i][j][r]; if(t.failed)return {false,0};d=std::max(d,t.duration);}
        return {true,ticks(d)};
    };
    std::set<std::pair<unsigned int,unsigned int>> precedence;
    for(auto e:m_problem_inputs->precedenceConstraints()) precedence.insert(e);
    while(true) {
        remaining(); ++rounds;
        NumericScheduleProblem p; p.durations=durations; p.releases.resize(n);
        p.timeout=std::min(params->timeout,remaining()); p.workers=params->workers;
        p.relative_gap=params->relative_gap; p.hierarchical=params->hierarchical;
        for(int i=0;i<n;++i) {
            double d=0;
            for(int r:owners[i]) {auto t=initial[i][r];if(t.failed){++s_num_failures;return nullptr;}d=std::max(d,t.duration);}
            p.releases[i]=ticks(d);
        }
        for(auto [i,j]:precedence) {auto [ok,d]=edge(i,j);if(!ok){++s_num_failures;return nullptr;}p.precedences.push_back({int(i),int(j),d});}
        for(auto [i,j]:m_problem_inputs->mutexConstraints()) {
            if(precedence.contains({i,j}) || precedence.contains({j,i}))continue;
            auto [a,ab]=edge(i,j);auto [b,ba]=edge(j,i);
            if(!a && !b){++s_num_failures;return nullptr;}
            if(!a)p.precedences.push_back({int(j),int(i),ba});
            else if(!b)p.precedences.push_back({int(i),int(j),ab});
            else p.mutexes.push_back({int(i),int(j),ab,ba});
        }
        auto result=solveNumericSchedule(p);solver_calls+=result.solver_calls;
        ++statuses["primary_"+result.primary_status];
        if(result.secondary_status!="not_requested")++statuses["secondary_"+result.secondary_status];
        if(!result.accepted){++s_num_failures;remaining();return nullptr;}
        std::vector<int> order(n);std::iota(order.begin(),order.end(),0);
        std::sort(order.begin(),order.end(),[&](int a,int b){return std::pair(result.starts[a],a)<std::pair(result.starts[b],b);});
        std::vector<int> previous(nr,-1);bool refined=false;
        // Preserve the upstream schedule -> selected transitions -> reschedule loop.
        for(int i:order) for(int r:owners[i]) {
            remaining();int prior=previous[r];
            auto& t=prior<0?initial[i][r]:transit[prior][i][r];
            if(!t.exact) {
                const auto from=prior<0?m_problem_inputs->robot(r)->initialConfiguration():m_problem_inputs->planTask(prior)->terminalConfiguration();
                t.duration=m_problem_inputs->robot(r)->durationQuery(from,m_problem_inputs->planTask(i)->initialConfiguration());
                t.failed=t.duration<0 || !std::isfinite(t.duration); t.exact=true; refined=true;
            }
            previous[r]=i;
        }
        if(refined)continue;
        std::vector<std::pair<float,float>> timepoints;
        for(int i=0;i<n;++i)timepoints.emplace_back(double(result.starts[i])/params->time_scale,double(result.ends[i])/params->time_scale);
        ++accepted;
        return std::make_shared<const DeterministicSchedule>(double(result.makespan)/params->time_scale,timepoints,result.orderings);
    }
}
}
