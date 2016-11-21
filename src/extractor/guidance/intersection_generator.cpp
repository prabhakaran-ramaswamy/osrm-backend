#include "extractor/guidance/intersection_generator.hpp"
#include "extractor/guidance/constants.hpp"
#include "extractor/guidance/toolkit.hpp"

#include "util/bearing.hpp"
#include "util/guidance/toolkit.hpp"

#include <algorithm>
#include <functional>
#include <iomanip>
#include <iterator>
#include <limits>
#include <unordered_set>
#include <utility>

#include <boost/range/algorithm/count_if.hpp>

namespace osrm
{
namespace extractor
{
namespace guidance
{

IntersectionGenerator::IntersectionGenerator(
    const util::NodeBasedDynamicGraph &node_based_graph,
    const RestrictionMap &restriction_map,
    const std::unordered_set<NodeID> &barrier_nodes,
    const std::vector<QueryNode> &node_info_list,
    const CompressedEdgeContainer &compressed_edge_container)
    : node_based_graph(node_based_graph), restriction_map(restriction_map),
      barrier_nodes(barrier_nodes), node_info_list(node_info_list),
      coordinate_extractor(node_based_graph, compressed_edge_container, node_info_list)
{
}

Intersection IntersectionGenerator::operator()(const NodeID from_node, const EdgeID via_eid) const
{
    return GetConnectedRoads(from_node, via_eid);
}

Intersection
IntersectionGenerator::ComputeIntersectionShape(const NodeID node_at_center_of_intersection) const
{
    Intersection intersection;
    // reserve enough items (+ the possibly missing u-turn edge)
    intersection.reserve(node_based_graph.GetOutDegree(node_at_center_of_intersection));
    const util::Coordinate turn_coordinate = node_info_list[node_at_center_of_intersection];

    // number of lanes at the intersection changes how far we look down the road
    const auto intersection_lanes =
        getLaneCountAtIntersection(node_at_center_of_intersection, node_based_graph);
    for (const EdgeID edge_connected_to_intersection :
         node_based_graph.GetAdjacentEdgeRange(node_at_center_of_intersection))
    {
        BOOST_ASSERT(edge_connected_to_intersection != SPECIAL_EDGEID);
        const NodeID to_node = node_based_graph.GetTarget(edge_connected_to_intersection);
        double bearing = 0.;
        // the default distance we lookahead on a road. This distance prevents small mapping
        // errors to impact the turn angles.
        const auto coordinate_along_edge_leaving =
            coordinate_extractor.GetCoordinateAlongRoad(node_at_center_of_intersection,
                                                        edge_connected_to_intersection,
                                                        !INVERT,
                                                        to_node,
                                                        intersection_lanes);

        bearing =
            util::coordinate_calculation::bearing(turn_coordinate, coordinate_along_edge_leaving);

        auto coords = coordinate_extractor.GetCoordinatesAlongRoad(
            node_at_center_of_intersection, edge_connected_to_intersection, !INVERT, to_node);

        intersection.push_back(
            ConnectedRoad(TurnOperation{edge_connected_to_intersection,
                                        0,
                                        bearing,
                                        {TurnType::Invalid, DirectionModifier::UTurn},
                                        INVALID_LANE_DATAID},
                          false));
    }
    return intersection;
}

Intersection IntersectionGenerator::AssignTurnAnglesAndValidTags(const NodeID previous_node,
                                                                 const EdgeID entering_via_edge,
                                                                 Intersection intersection) const
{
    BOOST_ASSERT(intersection.valid());
    const auto node_at_intersection = node_based_graph.GetTarget(entering_via_edge);

    // check if there is a single valid turn entering the current intersection
    const auto only_valid_turn = GetOnlyAllowedTurnIfExistent(previous_node, node_at_intersection);

    // barriers change our behaviour regarding u-turns
    const bool is_barrier_node = barrier_nodes.find(node_at_intersection) != barrier_nodes.end();

    const auto connecects_to_previous_node = [this, previous_node](const ConnectedRoad &road) {
        return node_based_graph.GetTarget(road.eid) == previous_node;
    };

    // check which of the edges is the u-turn edge
    const auto uturn_edge_itr =
        std::find_if(intersection.begin(), intersection.end(), connecects_to_previous_node);

    // there needs to be a connection, otherwise stuff went seriously wrong. Note that this is not
    // necessarily the same id as `entering_via_edge`.
    // In cases where parallel edges are present, we only remember the minimal edge. Both share
    // exactly the same coordinates, so the u-turn is still the best choice here.
    BOOST_ASSERT(uturn_edge_itr != intersection.end());

    const auto is_restricted = [&](const NodeID destination) {
        // check if we have a dedicated destination
        if (only_valid_turn && *only_valid_turn != destination)
            return true;

        // not explicitly forbidden
        return restriction_map.CheckIfTurnIsRestricted(
            previous_node, node_at_intersection, destination);
    };

    const auto uturn_bearing = util::bearing::reverseBearing(uturn_edge_itr->bearing);
    for (auto &road : intersection)
    {
        const auto &road_data = node_based_graph.GetEdgeData(road.eid);
        const NodeID road_destination_node = node_based_graph.GetTarget(road.eid);

        road.entry_allowed =
            // reverse edges are never valid turns because the resulting turn would look like this:
            // from_node --via_edge--> node_at_intersection <--onto_edge-- to_node
            // however we need this for capture intersection shape for incoming one-ways
            !road_data.reversed &&
            // we are not turning over a barrier
            (!is_barrier_node || road_destination_node == previous_node) &&
            // don't allow restricted turns
            !is_restricted(road_destination_node);

        road.angle = util::bearing::angleBetweenBearings(uturn_bearing, road.bearing);
    }

    // number of found valid exit roads
    const auto valid_count =
        std::count_if(intersection.begin(), intersection.end(), [](const ConnectedRoad &road) {
            return road.entry_allowed;
        });

    // in general, we don't wan't to allow u-turns. If we don't look at a barrier, we have to check
    // for dead end streets. These are the only ones that we allow uturns for, next to barriers
    // (which are also kind of a dead end, but we don't have to check these again :))
    if ((uturn_edge_itr->entry_allowed && !is_barrier_node && valid_count != 1) || valid_count == 0)
    {
        const auto allow_uturn_at_dead_end = [&]() {
            const auto &uturn_data = node_based_graph.GetEdgeData(uturn_edge_itr->eid);

            // we can't turn back onto oneway streets
            if (uturn_data.reversed)
                return false;

            // don't allow explicitly restricted turns
            if (is_restricted(previous_node))
                return false;

            // we define dead ends as roads that can only be entered via the possible u-turn
            const auto is_bidirectional = [&](const EdgeID via_eid) {
                const auto to_node = node_based_graph.GetTarget(via_eid);
                const auto reverse_edge = node_based_graph.FindEdge(to_node, node_at_intersection);
                BOOST_ASSERT(reverse_edge != SPECIAL_EDGEID);
                return !node_based_graph.GetEdgeData(reverse_edge).reversed;
            };

            const auto bidirectional_edges = [&]() {
                std::uint32_t count = 0;
                for (const auto eid : node_based_graph.GetAdjacentEdgeRange(node_at_intersection))
                    if (is_bidirectional(eid))
                        ++count;
                return count;
            }();

            // Checking for dead-end streets is kind of difficult. There is obvious dead ends
            // (single road connected)
            return bidirectional_edges <= 1;
        }();
        uturn_edge_itr->entry_allowed = allow_uturn_at_dead_end;
    }

    std::sort(std::begin(intersection),
              std::end(intersection),
              std::mem_fn(&ConnectedRoad::compareByAngle));

    BOOST_ASSERT(intersection[0].angle >= 0. &&
                 intersection[0].angle < std::numeric_limits<double>::epsilon());

    return intersection;
}

boost::optional<NodeID>
IntersectionGenerator::GetOnlyAllowedTurnIfExistent(const NodeID coming_from_node,
                                                    const NodeID node_at_intersection) const
{
    // If only restrictions refer to invalid ways somewhere far away, we rather ignore the
    // restriction than to not route over the intersection at all.
    const auto only_restriction_to_node =
        restriction_map.CheckForEmanatingIsOnlyTurn(coming_from_node, node_at_intersection);
    if (only_restriction_to_node != SPECIAL_NODEID)
    {
        // if the mentioned node does not exist anymore, we don't return it. This checks for broken
        // turn restrictions
        for (const auto onto_edge : node_based_graph.GetAdjacentEdgeRange(node_at_intersection))
            if (only_restriction_to_node == node_based_graph.GetTarget(onto_edge))
                return {only_restriction_to_node};
    }
    // Ignore broken only restrictions.
    return boost::none;
}

//                                               a
//                                               |
//                                               |
//                                               v
// For an intersection from_node --via_edi--> turn_node ----> c
//                                               ^
//                                               |
//                                               |
//                                               b
// This functions returns _all_ turns as if the graph was undirected.
// That means we not only get (from_node, turn_node, c) in the above example
// but also (from_node, turn_node, a), (from_node, turn_node, b). These turns are
// marked as invalid and only needed for intersection classification.
Intersection IntersectionGenerator::GetConnectedRoads(const NodeID from_node,
                                                      const EdgeID via_eid) const
{
    BOOST_ASSERT([this](const NodeID from_node, const EdgeID via_eid) {
        const auto range = node_based_graph.GetAdjacentEdgeRange(from_node);
        return range.front() <= via_eid && via_eid <= range.back();
    }(from_node, via_eid));

    Intersection intersection;
    const NodeID turn_node = node_based_graph.GetTarget(via_eid);
    // reserve enough items (+ the possibly missing u-turn edge)
    intersection.reserve(node_based_graph.GetOutDegree(turn_node) + 1);
    const NodeID only_restriction_to_node = [&]() {
        // If only restrictions refer to invalid ways somewhere far away, we rather ignore the
        // restriction than to not route over the intersection at all.
        const auto only_restriction_to_node =
            restriction_map.CheckForEmanatingIsOnlyTurn(from_node, turn_node);
        if (only_restriction_to_node != SPECIAL_NODEID)
        {
            // check if we can find an edge in the edge-rage
            for (const auto onto_edge : node_based_graph.GetAdjacentEdgeRange(turn_node))
                if (only_restriction_to_node == node_based_graph.GetTarget(onto_edge))
                    return only_restriction_to_node;
        }
        // Ignore broken only restrictions.
        return SPECIAL_NODEID;
    }();
    const bool is_barrier_node = barrier_nodes.find(turn_node) != barrier_nodes.end();

    bool has_uturn_edge = false;
    bool uturn_could_be_valid = false;
    const util::Coordinate turn_coordinate = node_info_list[turn_node];

    const auto intersection_lanes = getLaneCountAtIntersection(turn_node, node_based_graph);

    // The first coordinate (the origin) can depend on the number of lanes turning onto,
    // just as the target coordinate can. Here we compute the corrected coordinate for the
    // incoming edge.
    const auto first_coordinate = coordinate_extractor.GetCoordinateAlongRoad(
        from_node, via_eid, INVERT, turn_node, intersection_lanes);

    for (const EdgeID onto_edge : node_based_graph.GetAdjacentEdgeRange(turn_node))
    {
        BOOST_ASSERT(onto_edge != SPECIAL_EDGEID);
        const NodeID to_node = node_based_graph.GetTarget(onto_edge);
        const auto &onto_data = node_based_graph.GetEdgeData(onto_edge);

        bool turn_is_valid =
            // reverse edges are never valid turns because the resulting turn would look like this:
            // from_node --via_edge--> turn_node <--onto_edge-- to_node
            // however we need this for capture intersection shape for incoming one-ways
            !onto_data.reversed &&
            // we are not turning over a barrier
            (!is_barrier_node || from_node == to_node) &&
            // We are at an only_-restriction but not at the right turn.
            (only_restriction_to_node == SPECIAL_NODEID || to_node == only_restriction_to_node) &&
            // the turn is not restricted
            !restriction_map.CheckIfTurnIsRestricted(from_node, turn_node, to_node);

        auto angle = 0.;
        double bearing = 0.;

        if (from_node == to_node)
        {
            bearing = util::coordinate_calculation::bearing(turn_coordinate, first_coordinate);
            uturn_could_be_valid = turn_is_valid;
            if (turn_is_valid && !is_barrier_node)
            {
                // we only add u-turns for dead-end streets.
                if (node_based_graph.GetOutDegree(turn_node) > 1)
                {
                    auto number_of_emmiting_bidirectional_edges = 0;
                    for (auto edge : node_based_graph.GetAdjacentEdgeRange(turn_node))
                    {
                        auto target = node_based_graph.GetTarget(edge);
                        auto reverse_edge = node_based_graph.FindEdge(target, turn_node);
                        BOOST_ASSERT(reverse_edge != SPECIAL_EDGEID);
                        if (!node_based_graph.GetEdgeData(reverse_edge).reversed)
                        {
                            ++number_of_emmiting_bidirectional_edges;
                        }
                    }
                    // is a dead-end, only possible road is to go back
                    turn_is_valid = number_of_emmiting_bidirectional_edges <= 1;
                }
            }

            has_uturn_edge = true;
            BOOST_ASSERT(angle >= 0. && angle < std::numeric_limits<double>::epsilon());
        }
        else
        {
            const auto third_coordinate = coordinate_extractor.GetCoordinateAlongRoad(
                turn_node, onto_edge, !INVERT, to_node, intersection_lanes);

            angle = util::coordinate_calculation::computeAngle(
                first_coordinate, turn_coordinate, third_coordinate);

            bearing = util::coordinate_calculation::bearing(turn_coordinate, third_coordinate);

            if (std::abs(angle) < std::numeric_limits<double>::epsilon())
                has_uturn_edge = true;
        }
        intersection.push_back(
            ConnectedRoad(TurnOperation{onto_edge,
                                        angle,
                                        bearing,
                                        {TurnType::Invalid, DirectionModifier::UTurn},
                                        INVALID_LANE_DATAID},
                          turn_is_valid));
    }

    // We hit the case of a street leading into nothing-ness. Since the code here assumes
    // that this will never happen we add an artificial invalid uturn in this case.
    if (!has_uturn_edge)
    {
        const auto first_coordinate = coordinate_extractor.GetCoordinateAlongRoad(
            from_node,
            via_eid,
            INVERT,
            turn_node,
            node_based_graph.GetEdgeData(via_eid).road_classification.GetNumberOfLanes());
        const double bearing =
            util::coordinate_calculation::bearing(turn_coordinate, first_coordinate);

        intersection.push_back({TurnOperation{via_eid,
                                              0.,
                                              bearing,
                                              {TurnType::Invalid, DirectionModifier::UTurn},
                                              INVALID_LANE_DATAID},
                                false});
    }

    std::sort(std::begin(intersection),
              std::end(intersection),
              std::mem_fn(&ConnectedRoad::compareByAngle));

    BOOST_ASSERT(intersection[0].angle >= 0. &&
                 intersection[0].angle < std::numeric_limits<double>::epsilon());

    const auto valid_count =
        boost::count_if(intersection, [](const ConnectedRoad &road) { return road.entry_allowed; });
    if (0 == valid_count && uturn_could_be_valid)
    {
        // after intersections sorting by angles, find the u-turn with (from_node ==
        // to_node)
        // that was inserted together with setting uturn_could_be_valid flag
        std::size_t self_u_turn = 0;
        while (self_u_turn < intersection.size() &&
               intersection[self_u_turn].angle < std::numeric_limits<double>::epsilon() &&
               from_node != node_based_graph.GetTarget(intersection[self_u_turn].eid))
        {
            ++self_u_turn;
        }

        BOOST_ASSERT(from_node == node_based_graph.GetTarget(intersection[self_u_turn].eid));
        intersection[self_u_turn].entry_allowed = true;
    }
    return intersection;
}

Intersection
IntersectionGenerator::GetActualNextIntersection(const NodeID starting_node,
                                                 const EdgeID via_edge,
                                                 NodeID *resulting_from_node = nullptr,
                                                 EdgeID *resulting_via_edge = nullptr) const
{
    // This function skips over traffic lights/graph compression issues and similar to find the next
    // actual intersection
    Intersection result = GetConnectedRoads(starting_node, via_edge);

    // Skip over stuff that has not been compressed due to barriers/parallel edges
    NodeID node_at_intersection = starting_node;
    EdgeID incoming_edge = via_edge;

    // to prevent endless loops
    const auto termination_node = node_based_graph.GetTarget(via_edge);

    // using a maximum lookahead, we make sure not to end up in some form of loop
    std::unordered_set<NodeID> visited_nodes;
    while (visited_nodes.count(node_at_intersection) == 0 &&
           (result.size() == 2 &&
            node_based_graph.GetEdgeData(via_edge).IsCompatibleTo(
                node_based_graph.GetEdgeData(result[1].eid))))
    {
        visited_nodes.insert(node_at_intersection);
        node_at_intersection = node_based_graph.GetTarget(incoming_edge);
        incoming_edge = result[1].eid;
        result = GetConnectedRoads(node_at_intersection, incoming_edge);

        // When looping back to the original node, we obviously are in a loop. Stop there.
        if (termination_node == node_based_graph.GetTarget(incoming_edge))
            break;
    }

    // return output if requested
    if (resulting_from_node)
        *resulting_from_node = node_at_intersection;
    if (resulting_via_edge)
        *resulting_via_edge = incoming_edge;

    return result;
}

const CoordinateExtractor &IntersectionGenerator::GetCoordinateExtractor() const
{
    return coordinate_extractor;
}

} // namespace guidance
} // namespace extractor
} // namespace osrm
