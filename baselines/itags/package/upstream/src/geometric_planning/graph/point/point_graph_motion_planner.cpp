/*
 * Graphically Recursive Simultaneous Task Allocation, Planning,
 * Scheduling, and Execution
 *
 * Copyright (C) 2020-2022
 *
 * Author: Andrew Messing
 * Author: Glen Neville
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
#include "grstapse/geometric_planning/graph/point/point_graph_motion_planner.hpp"

// Local
#include "grstapse/common/search/undirected_graph/undirected_graph_path_cost.hpp"
#include "grstapse/common/search/undirected_graph/undirected_graph_successor_generator.hpp"
#include "grstapse/common/utilities/constants.hpp"
#include "grstapse/geometric_planning/graph/point/equal_point_graph_configuration_goal_check.hpp"
#include "grstapse/geometric_planning/graph/point/point_graph_a_star.hpp"
#include "grstapse/geometric_planning/graph/point/point_graph_configuration.hpp"
#include "grstapse/geometric_planning/graph/point/point_graph_configuration_euclidean_distance_heuristic.hpp"
#include "grstapse/geometric_planning/graph/point/point_graph_environment.hpp"
#include "grstapse/geometric_planning/graph/point/point_graph_motion_planning_query_result.hpp"
#include "grstapse/geometric_planning/motion_planner_parameters_base.hpp"

// Global
#include <chrono>
#include <queue>
#include <unordered_map>

namespace grstapse
{
    PointGraphMotionPlanner::PointGraphMotionPlanner(
        const std::shared_ptr<const MotionPlannerParametersBase>& parameters,
        const std::shared_ptr<PointGraphEnvironment>& graph)
        : GraphMotionPlannerBase(parameters, graph)
        , m_search_parameters(
              new BestFirstSearchParameters{parameters->timeout > 0.0f,
                                            parameters->timeout,
                                            constants::k_motion_planning_time + std::string("a_star"),
                                            false,
                                            false})
        , m_astar_functors({
              std::make_shared<const UndirectedGraphPathCost<SearchNode>>(),
              nullptr,  // Gets set for each A* search individually
              std::make_shared<const UndirectedGraphSuccessorGenerator<SearchNode>>(graph),
              nullptr  // Gets set for each A* search individually
          })
        , m_graph(graph)
    {}

    std::shared_ptr<const MotionPlanningQueryResultBase> PointGraphMotionPlanner::computeMotionPlan(
        const std::shared_ptr<const Species>& species,
        const std::shared_ptr<const ConfigurationBase>& initial_configuration,
        const std::shared_ptr<const ConfigurationBase>& goal_configuration)
    {
        // POINT-GRAPH SHORTEST-PATH CORRECTION (AAMAS_ITAGS_PointGraphFix, 2026-10-04).
        // The shared A* path (UndirectedGraphPathCost / UndirectedGraphEdgeApplier / AStarSearchNodeBase) does not
        // accumulate the parent's cost (root g = NaN, child cost = node->g() + lastEdge cost evaluated on the parent),
        // and the shared best-first search replaces duplicate open nodes without a cost test. The point graph exported by
        // the adapter is a static PRM with non-negative edge costs, so the query is answered here with a graph-specific
        // Dijkstra over exactly the exported vertices/edges/costs: start distance 0, others infinity, a predecessor and
        // distance are updated only for a strictly smaller accumulated distance, stale queue entries are skipped, ties
        // are ordered by (distance, vertex id). Start == goal returns the normal length-0 path; an unreachable goal or a
        // timeout returns a non-success status (never a length-0 or negative travel time).
        auto ic = std::dynamic_pointer_cast<const PointGraphConfiguration>(initial_configuration);
        auto gc = std::dynamic_pointer_cast<const PointGraphConfiguration>(goal_configuration);
        if(!ic || !gc)
        {
            return std::make_shared<PointGraphMotionPlanningQueryResult>(MotionPlannerQueryStatus::e_unknown);
        }
        const auto start = m_graph->findVertex(ic);
        const auto goal  = m_graph->findVertex(gc);
        if(!start || !goal)
        {
            return std::make_shared<PointGraphMotionPlanningQueryResult>(MotionPlannerQueryStatus::e_unknown);
        }
        const bool has_timeout = m_search_parameters->has_timeout;
        const double timeout_s = m_search_parameters->timeout;
        const auto t0          = std::chrono::steady_clock::now();

        using Vertex = UndirectedGraph<PointGraphConfiguration>::Vertex;
        std::unordered_map<unsigned int, double> dist;
        std::unordered_map<unsigned int, unsigned int> pred;
        std::unordered_map<unsigned int, std::shared_ptr<Vertex>> seen;
        using Item = std::pair<double, unsigned int>;  // (distance, vertex id): ties by smaller id
        std::priority_queue<Item, std::vector<Item>, std::greater<Item>> heap;
        dist[start->id()] = 0.0;
        seen[start->id()] = start;
        heap.emplace(0.0, start->id());
        bool found = start->id() == goal->id();
        unsigned int settled = 0;
        while(!heap.empty() && !found)
        {
            const auto [d, uid] = heap.top();
            heap.pop();
            const auto it_d = dist.find(uid);
            if(it_d == dist.end() || d > it_d->second)
            {
                continue;  // stale queue entry
            }
            if(uid == goal->id())
            {
                found = true;
                break;
            }
            if(has_timeout && (++settled % 256u) == 0u &&
               std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() > timeout_s)
            {
                return std::make_shared<PointGraphMotionPlanningQueryResult>(MotionPlannerQueryStatus::e_timeout);
            }
            const auto& u = seen.at(uid);
            for(const auto& edge: u->edges())
            {
                const auto& v   = edge->nodeA() == u ? edge->nodeB() : edge->nodeA();
                const double nd = d + static_cast<double>(edge->cost());
                auto it         = dist.find(v->id());
                if(it == dist.end() || nd < it->second)
                {
                    dist[v->id()] = nd;
                    pred[v->id()] = uid;
                    seen[v->id()] = v;
                    heap.emplace(nd, v->id());
                }
            }
        }
        if(!found)
        {
            return std::make_shared<PointGraphMotionPlanningQueryResult>(MotionPlannerQueryStatus::e_unknown);
        }
        std::vector<std::shared_ptr<PointGraphConfiguration>> path;
        unsigned int cur = goal->id();
        path.push_back(seen.at(cur)->payload());
        while(cur != start->id())
        {
            cur = pred.at(cur);
            path.push_back(seen.at(cur)->payload());
        }
        std::reverse(path.begin(), path.end());
        return std::make_shared<PointGraphMotionPlanningQueryResult>(MotionPlannerQueryStatus::e_success,
                                                                     path,
                                                                     static_cast<float>(dist.at(goal->id())));
    }
}  // namespace grstapse
