#include <cstdlib>
#include "cpsat_model.hpp"
#include <algorithm>
#include <chrono>
#include <numeric>
#include <stdexcept>
#include <ortools/sat/cp_model.h>
#include <ortools/sat/cp_model_solver.h>
namespace grstapse {
using namespace operations_research;
using namespace operations_research::sat;
NumericScheduleResult solveNumericSchedule(const NumericScheduleProblem& p) {
    NumericScheduleResult r;
    const int n=p.durations.size();
    if(n==0 || p.releases.size()!=n) throw std::invalid_argument("Invalid numeric scheduling dimensions");
    int64_t max_arc=0, horizon=0;
    for(auto d:p.durations) { if(d<0) throw std::invalid_argument("Negative duration"); horizon+=d; }
    for(auto d:p.releases) { if(d<0) throw std::invalid_argument("Negative release"); max_arc=std::max(max_arc,d); }
    auto edge_check=[&](int a,int b){ if(a<0 || b<0 || a>=n || b>=n || a==b) throw std::invalid_argument("Invalid scheduling edge"); };
    for(auto e:p.precedences) { edge_check(e.from,e.to); if(e.travel<0) throw std::invalid_argument("Negative transition"); max_arc=std::max(max_arc,e.travel); }
    for(auto e:p.mutexes) { edge_check(e.first,e.second); if(e.forward<0 || e.reverse<0) throw std::invalid_argument("Negative transition"); max_arc=std::max({max_arc,e.forward,e.reverse}); }
    horizon+=(n+1)*max_arc+1;
    if(horizon<=0 || horizon>1000000000000LL) throw std::invalid_argument("Unsafe scheduling horizon");
    CpModelBuilder cp;
    std::vector<IntVar> starts,ends;
    for(int i=0;i<n;++i) {
        starts.push_back(cp.NewIntVar(Domain(0,horizon)));
        ends.push_back(cp.NewIntVar(Domain(0,horizon)));
        cp.AddEquality(ends[i],starts[i]+p.durations[i]);
        cp.AddGreaterOrEqual(starts[i],p.releases[i]);
    }
    for(auto e:p.precedences) cp.AddGreaterOrEqual(starts[e.to],ends[e.from]+e.travel);
    std::vector<BoolVar> ordering;
    for(auto e:p.mutexes) {
        auto b=cp.NewBoolVar(); ordering.push_back(b);
        cp.AddGreaterOrEqual(starts[e.second],ends[e.first]+e.forward).OnlyEnforceIf(b);
        cp.AddGreaterOrEqual(starts[e.first],ends[e.second]+e.reverse).OnlyEnforceIf(Not(b));
    }
    auto makespan=cp.NewIntVar(Domain(0,horizon));
    cp.AddMaxEquality(makespan,ends);
    cp.Minimize(makespan);
    SatParameters params;
    params.set_max_time_in_seconds(p.timeout);
    params.set_num_search_workers(p.workers);
    params.set_relative_gap_limit(p.relative_gap);
    { const char* s=std::getenv("ITAGS_CPSAT_SEED"); params.set_random_seed(s?std::atoi(s):0); }   // seed repeats 2026-10-07 (default 0 = frozen)
    const auto started=std::chrono::steady_clock::now();
    auto response=SolveWithParameters(cp.Build(),params); ++r.solver_calls;
    r.primary_status=CpSolverStatus_Name(response.status());
    // As in the upstream scheduler: a time-limited feasible incumbent is not
    // accepted as a completed scheduling evaluation. OPTIMAL obeys gap settings.
    if(response.status()!=CpSolverStatus::OPTIMAL) return r;
    r.makespan=SolutionIntegerValue(response,makespan);
    r.primary_bound=response.best_objective_bound();
    if(p.hierarchical) {
        const double elapsed=std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count();
        if(elapsed>=p.timeout) { r.secondary_status="budget_exhausted"; return r; }
        cp.AddEquality(makespan,r.makespan);
        cp.Minimize(LinearExpr::Sum(starts));
        params.set_max_time_in_seconds(p.timeout-elapsed);
        params.set_relative_gap_limit(0.0);
        response=SolveWithParameters(cp.Build(),params); ++r.solver_calls;
        r.secondary_status=CpSolverStatus_Name(response.status());
        if(response.status()!=CpSolverStatus::OPTIMAL) return r;
    }
    for(int i=0;i<n;++i) { r.starts.push_back(SolutionIntegerValue(response,starts[i])); r.ends.push_back(SolutionIntegerValue(response,ends[i])); }
    for(int k=0;k<p.mutexes.size();++k) {
        const auto e=p.mutexes[k];
        r.orderings.emplace_back(SolutionBooleanValue(response,ordering[k])?e.first:e.second,
                                 SolutionBooleanValue(response,ordering[k])?e.second:e.first);
    }
    r.accepted=true;
    return r;
}
NumericScheduleProblem numericFromJson(const nlohmann::json& j) {
    NumericScheduleProblem p;
    p.durations=j.at("durations").get<std::vector<int64_t>>();
    p.releases=j.at("releases").get<std::vector<int64_t>>();
    for(auto e:j.at("precedences")) p.precedences.push_back({e[0],e[1],e[2]});
    for(auto e:j.at("mutexes")) p.mutexes.push_back({e[0],e[1],e[2],e[3]});
    p.timeout=j.value("timeout",10.0); p.workers=j.value("workers",1);
    p.relative_gap=j.value("relative_gap",0.0); p.hierarchical=j.value("hierarchical",true);
    return p;
}
nlohmann::json numericResultJson(const NumericScheduleResult& r) {
    return {{"accepted",r.accepted},{"primary_status",r.primary_status},{"secondary_status",r.secondary_status},
            {"makespan",r.makespan},{"primary_bound",r.primary_bound},{"starts",r.starts},{"ends",r.ends},
            {"orderings",r.orderings},{"solver_calls",r.solver_calls}};
}
}
