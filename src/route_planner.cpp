#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    this->start_node = &m_Model.FindClosestNode(start_x, start_y);
    this->end_node = &m_Model.FindClosestNode(end_x, end_y);
}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
	return node->distance(*end_node);
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
  	current_node->FindNeighbors();
    for (RouteModel::Node * node : current_node->neighbors){
    	node->parent = current_node;
        node->h_value = CalculateHValue(node);
        node->g_value = current_node->g_value + current_node->distance(*node);
        if (node->visited == false){
          node->visited = true;
          this->open_list.emplace_back(node);
        }
    }
}

RouteModel::Node *RoutePlanner::NextNode() {
    auto compare = [](RouteModel::Node* node1, RouteModel::Node* node2){
    	return node1->h_value + node1->g_value > node2->h_value + node2->g_value;
    };
  	std::sort(this->open_list.begin(), this->open_list.end(), compare);
  	RouteModel::Node* lowest_node = (this->open_list.back());
  	this->open_list.pop_back();
  	return lowest_node;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    while(current_node->parent != nullptr){
        path_found.emplace_back(*current_node);
        distance += current_node->distance(*(current_node->parent));
        current_node = current_node->parent;
    }
    path_found.emplace_back(*current_node);
    std::reverse(path_found.begin(), path_found.end());
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
}

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    // TODO: Implement your solution here.
    this->open_list.emplace_back(this->start_node);
  	this->start_node->visited = true;

  	while(!this->open_list.empty()){
        current_node = NextNode();
        if (current_node == this->end_node){
          std::cout << "Goal reached!" << "\n";
    	  m_Model.path = ConstructFinalPath(current_node);
          std::cout << "Path size is: " << m_Model.path.size() << "\n";
          break;
        }
        AddNeighbors(current_node);
    }
}
