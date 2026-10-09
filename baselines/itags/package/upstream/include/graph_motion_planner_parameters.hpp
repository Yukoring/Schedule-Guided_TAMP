#pragma once
// PRM point-graph extension (AAMAS_ITAGS_CPP_MATCHED_20261003): motion-planner parameters for graph
// configurations. The upstream commit left the graph branch of MotionPlannerParametersBase::loadJson
// unimplemented; this struct only carries the base fields (configuration_type, timeout).
#include <grstapse/geometric_planning/motion_planner_parameters_base.hpp>
namespace grstapse {
struct GraphMotionPlannerParameters : MotionPlannerParametersBase {
    static std::shared_ptr<const GraphMotionPlannerParameters> loadJson(const nlohmann::json& j) {
        auto rv=std::make_shared<GraphMotionPlannerParameters>(); rv->internalLoadJson(j); return rv;
    }
};
}
