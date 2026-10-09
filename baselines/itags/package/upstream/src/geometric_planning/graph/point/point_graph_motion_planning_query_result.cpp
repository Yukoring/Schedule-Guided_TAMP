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
#include "grstapse/geometric_planning/graph/point/point_graph_motion_planning_query_result.hpp"

// Local
#include "grstapse/geometric_planning/graph/point/point_graph_configuration.hpp"

namespace grstapse
{
    PointGraphMotionPlanningQueryResult::PointGraphMotionPlanningQueryResult(
        MotionPlannerQueryStatus status,
        const std::vector<std::shared_ptr<PointGraphConfiguration>>& path,
        float edge_cost_length)
        : GraphMotionPlanningQueryResult(status)
        , m_path(path)
        , m_edge_cost_length(edge_cost_length)
    {}

    float PointGraphMotionPlanningQueryResult::duration(const float speed) const
    {
        if(status() != MotionPlannerQueryStatus::e_success)
        {
            return -1.0f;  // failure marker: unreachable / timeout / unknown must not become a travel time
        }
        return length() / speed;
    }

    float PointGraphMotionPlanningQueryResult::length() const
    {
        if(m_path.size() < 2)
        {
            return 0.0f;
        }
        if(m_edge_cost_length >= 0.0f)
        {
            return m_edge_cost_length;  // sum of the exported edge costs along the path
        }

        float rv                                          = 0.0f;
        std::shared_ptr<PointGraphConfiguration> previous = m_path[0];
        for(unsigned int i = 1, end = m_path.size(); i < end; ++i)
        {
            std::shared_ptr<PointGraphConfiguration> current = m_path[i];
            rv += current->euclideanDistance(previous);
            previous = current;
        }
        return rv;
    }

    void PointGraphMotionPlanningQueryResult::serializeToJson(nlohmann::json& j) const
    {
        // PRM point-graph extension: the path as graph vertices
        j = nlohmann::json::array();
        for(const auto& c: m_path)
        {
            j.push_back({{"id", c->id()}, {"x", c->x()}, {"y", c->y()}});
        }
    }
}  // namespace grstapse