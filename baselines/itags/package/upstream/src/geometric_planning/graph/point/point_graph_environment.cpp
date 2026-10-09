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
#include "grstapse/geometric_planning/graph/point/point_graph_environment.hpp"

// Global
#include <queue>
#include <unordered_map>
#include <algorithm>

// Local
#include "grstapse/common/utilities/constants.hpp"
#include "grstapse/common/utilities/error.hpp"

namespace grstapse
{
    PointGraphEnvironment::PointGraphEnvironment()
        : GraphEnvironment(GraphType::e_point)
    {}

    std::shared_ptr<UndirectedGraph<PointGraphConfiguration>::Vertex> PointGraphEnvironment::findVertex(
        const std::shared_ptr<const PointGraphConfiguration>& configuration) const
    {
        for(auto& [key, vertex]: UndirectedGraph<PointGraphConfiguration>::m_vertices)
        {
            if(*configuration == *(vertex->payload()))
            {
                return vertex;
            }
        }

        throw createLogicError("Cannot find vertex");
    }

    float PointGraphEnvironment::longestPath() const
    {
        // PRM point-graph extension: graph diameter = longest shortest path between any two vertices (Dijkstra from
        // every vertex over edge costs). Used by the upstream worst-makespan normalisation; the adapter supplies an
        // explicit worst_makespan in the JSON, which the loader applies afterwards.
        float longest = 0.0f;
        for(const auto& [sid, source]: UndirectedGraph<PointGraphConfiguration>::m_vertices)
        {
            std::unordered_map<unsigned int, float> dist;
            std::priority_queue<std::pair<float, unsigned int>, std::vector<std::pair<float, unsigned int>>, std::greater<>> heap;
            dist[sid] = 0.0f;
            heap.emplace(0.0f, sid);
            while(!heap.empty())
            {
                auto [d, uid] = heap.top();
                heap.pop();
                if(d > dist[uid])
                {
                    continue;
                }
                const auto& u = UndirectedGraph<PointGraphConfiguration>::m_vertices.at(uid);
                for(const auto& edge: u->edges())
                {
                    const auto& v  = edge->nodeA() == u ? edge->nodeB() : edge->nodeA();
                    const float nd = d + edge->cost();
                    auto it        = dist.find(v->id());
                    if(it == dist.end() || nd < it->second)
                    {
                        dist[v->id()] = nd;
                        heap.emplace(nd, v->id());
                    }
                }
            }
            for(const auto& [vid, d]: dist)
            {
                longest = std::max(longest, d);
            }
        }
        return longest;
    }

    std::shared_ptr<PointGraphEnvironment> PointGraphEnvironment::shallowCopy() const
    {
        auto rv        = std::make_shared<PointGraphEnvironment>();
        rv->m_vertices = m_vertices;
        rv->m_edges    = m_edges;
        return rv;
    }

    void from_json(const nlohmann::json& j, PointGraphEnvironment& environment)
    {
        for(const nlohmann::json& vertex_j: j[constants::k_vertices])
        {
            environment.addVertex(vertex_j[constants::k_id],
                                  std::make_shared<PointGraphConfiguration>(vertex_j[constants::k_id],
                                                                            vertex_j[constants::k_x],
                                                                            vertex_j[constants::k_y]));
        }

        for(const nlohmann::json& edge_j: j[constants::k_edges])
        {
            environment.addEdge(edge_j[constants::k_vertex_a],
                                edge_j[constants::k_vertex_b],
                                edge_j[constants::k_cost]);
        }
    }
}  // namespace grstapse