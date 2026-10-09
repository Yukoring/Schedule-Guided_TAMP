// Diagnostic interface for the point-graph motion query (AAMAS_ITAGS_PointGraphFix, 2026-10-04).
// Loads the SAME point-graph environment JSON that the ITAGS inputs carry (motion_planners[0].environment_parameters),
// builds the real PointGraphMotionPlanner with the same parameter type/timeout, and answers (from_id, to_id, speed)
// queries through MotionPlannerBase::query / durationQuery. Output: per query status, vertex-id path, length, duration.
// It is a separate executable; the experiment worker never reads its results.
#include <fstream>
#include <iostream>
#include <memory>
#include <nlohmann/json.hpp>
#include <Eigen/Core>
#include <grstapse/species.hpp>
#include <grstapse/geometric_planning/graph/point/point_graph_environment.hpp>
#include <grstapse/geometric_planning/graph/point/point_graph_motion_planner.hpp>
#include <grstapse/geometric_planning/graph/point/point_graph_motion_planning_query_result.hpp>
#include "../prm_graph_ext/graph_motion_planner_parameters.hpp"
int main(int argc,char** argv){
    if(argc!=3){std::cerr<<"Usage: pointgraph_query INPUT.json RESULT.json\n  INPUT: {environment_parameters:{vertices,edges}, mp_parameters:{configuration_type,timeout}, queries:[{from,to,speed}]}\n";return 2;}
    nlohmann::json out;
    try{
        std::ifstream in(argv[1]);if(!in)throw std::runtime_error("cannot open input");nlohmann::json j;in>>j;
        auto env=std::make_shared<grstapse::PointGraphEnvironment>();grstapse::from_json(j.at("environment_parameters"),*env);
        auto params=grstapse::GraphMotionPlannerParameters::loadJson(j.at("mp_parameters"));
        auto planner=std::make_shared<grstapse::PointGraphMotionPlanner>(params,env);
        out["vertices"]=env->vertices().size();out["timeout"]=params->timeout;out["queries"]=nlohmann::json::array();
        std::map<float,std::shared_ptr<grstapse::Species>> species_by_speed;
        for(const auto& q: j.at("queries")){
            const unsigned int a=q.at("from").get<unsigned int>(),b=q.at("to").get<unsigned int>();const float speed=q.value("speed",1.0f);
            auto& sp=species_by_speed[speed];
            if(!sp)sp=std::make_shared<grstapse::Species>("diag_speed_"+std::to_string(speed),Eigen::VectorXf::Zero(1),0.3f,speed,planner);
            auto va=env->vertices().find(a),vb=env->vertices().find(b);
            nlohmann::json r={{"from",a},{"to",b},{"speed",speed}};
            if(va==env->vertices().end()||vb==env->vertices().end()){r["status"]="unknown_vertex";out["queries"].push_back(r);continue;}
            auto res=planner->query(sp,va->second->payload(),vb->second->payload());
            auto pg=std::dynamic_pointer_cast<const grstapse::PointGraphMotionPlanningQueryResult>(res);
            r["status"]=res->status();r["length"]=res->length();r["duration"]=planner->durationQuery(sp,va->second->payload(),vb->second->payload());
            nlohmann::json path=nlohmann::json::array();if(pg)for(const auto& c:pg->path())path.push_back(c->id());r["path"]=path;
            out["queries"].push_back(r);
        }
        out["ok"]=true;
    }catch(const std::exception& e){out["ok"]=false;out["error"]=e.what();}
    std::ofstream o(argv[2]);o<<out.dump(1)<<"\n";return out.value("ok",false)?0:1;
}
