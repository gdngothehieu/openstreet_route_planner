#include "route_planner.h"
#include <algorithm>
using std::sort;

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}


float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    float HValue = node->distance(*end_node);
    return HValue;
}


void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    for (auto node: current_node->neighbors){
        node->parent = current_node;
        node->h_value = CalculateHValue(node);
        node->g_value = current_node->g_value + current_node->distance(*node);
        node->visited = true;

        open_list.emplace_back(node);
    }
}


// FOUND SORT FUNCTION SOLUTION ON STACK OVERFLOW
// https://stackoverflow.com/questions/16894700/c-custom-compare-function-for-stdsort
// accessed October 2nd, 2022

RouteModel::Node *RoutePlanner::NextNode() {
    sort(open_list.begin(), open_list.end(), [](const auto &a, const auto &b){
      return(a->g_value + a->h_value > b->g_value + b->h_value);
    });
    auto current = open_list.back();
    open_list.pop_back();
    return current;
}


std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
    
  	auto parent_node = current_node->parent;
    while(parent_node){
        path_found.emplace_back(*current_node);
        distance += current_node->distance(*parent_node);
        current_node = parent_node;
		parent_node = current_node->parent;
    }
    path_found.emplace_back(*current_node);
  	std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = start_node;
  open_list.emplace_back(current_node);
  current_node->visited = true;
  
    while(!open_list.empty()){
      if(current_node == end_node){
        break;
      }
        AddNeighbors(current_node);
        current_node = NextNode();
    }
    m_Model.path = ConstructFinalPath(current_node);
}