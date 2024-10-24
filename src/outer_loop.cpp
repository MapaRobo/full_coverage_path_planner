//
// Copyright [2024] MapaRobo [legal/copyright]
//
#include <algorithm>
#include <iostream>
#include <list>
#include <string>
#include <vector>

#include "full_coverage_path_planner/outer_loop.h"
#include <pluginlib/class_list_macros.h>

// register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(full_coverage_path_planner::OuterLoop, nav_core::BaseGlobalPlanner)

namespace full_coverage_path_planner
{
void OuterLoop::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
  if (!initialized_)
  {
    // Create a publisher to visualize the plan
    ros::NodeHandle private_nh("~/");
    ros::NodeHandle nh, private_named_nh("~/" + name);

    plan_pub_ = private_named_nh.advertise<nav_msgs::Path>("plan", 1);
    // Try to request the cpp-grid from the cpp_grid map_server
    cpp_grid_client_ = nh.serviceClient<nav_msgs::GetMap>("static_map");
    // Get the cost map:
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros->getCostmap();

    // Define  robot radius (radius) parameter
    float robot_radius_default = 0.5f;
    private_named_nh.param<float>("robot_radius", robot_radius_, robot_radius_default);
    // Define  tool radius (radius) parameter
    float tool_radius_default = 0.5f;
    private_named_nh.param<float>("tool_radius", tool_radius_, tool_radius_default);
    initialized_ = true;
  }
}

bool is_edge_node(int x, int y, std::vector<std::vector<bool> > const& grid) {
  int dx = 0, dy = 0, n_rows = grid.size(), n_cols = grid[0].size();
  if (
    (x >= 0 && x < n_cols && y >= 0 && y < n_rows)
    && grid[x][y] == eNodeOpen
  ) {
    for (dx = -1; dx <= 1;dx++) {
      for (dy = -1; dy <= 1;dy++) {
        if (
          !(dx == 0 && dy == 0)
          && (x + dx >= 0 && x + dx < n_cols && y + dy >= 0 && y + dy < n_rows)
          && grid[x + dx][y + dy] == eNodeVisited
        ) {
       	  return true;
        }
      }
    }
  }
  return false;
}

std::list<Point_t> OuterLoop::outer_loop(std::vector<std::vector<bool> > const& grid, Point_t& init)
{
  int dx = 0, dy = 0, x, y, n_rows = grid.size(), n_cols = grid[0].size();

  std::vector<std::vector<bool>> visited = grid;  // Copy grid matrix

  // Copy incoming list to 'end'
  std::list<gridNode_t> path_nodes;
  // Set starting pos
  x = init.x;
  y = init.y;
  addNodeToList(x, y, path_nodes, visited);

  // set initial direction based on space visible from initial pos
  int robot_dir = dirWithLeastSpace(x, y, n_cols, n_rows, grid, visited, point);
  if (robot_dir == east) {
    dx = 1;
  } else if (robot_dir == west) {
    dx = -1;
  } else if (robot_dir == north) {
    dy = 1;
  } else if (robot_dir == south) {
    dy = -1;
  } else {
    ROS_ERROR("Full Coverage Path Planner: NO INITIAL ROBOT DIRECTION CALCULATED. This is a logic error that must be fixed by editing outer_loop function in outer_loop.cpp. Will travel east for now.");
    robot_dir = east;
    dx = +1;
  }

  bool done = false;
  while (!done) {
    // 1. drive straight until not a valid move (hit occupied cell or at end of map)
    while(validMove(x + dx, y + dy, n_cols, n_rows, grid, visited)) {
      x += dx;
      y += dy;
      addNodeToList(x, y, path_nodes, visited);
    }

    // 2. connect the adjasen edge points to form a outer loop
    int move_dir = 1;
    if (dx == -1 || (dx == 0 && dy == 0)) {
      move_dir = -1;
    }

    bool found_connecting_edge_node = false;
    for (int it = 0; it < 8; it++) {
      if (move_dir < 0) {
        if (dx > -1) {
          dx += move_dir;
        } else if (dy > -1) {
          dy += move_dir;
        } else {
          move_dir = move_dir * -1;
        }
      } else {
        if (dx < 1) {
          dx += move_dir;
        } else if (dy < 1) {
          dy += move_dir;
        } else {
          move_dir = move_dir * -1;
        }
      }

      if (
        (x + dx >= 0 && x + dx < n_cols && y + dy >= 0 && y + dy < n_rows)
        && validMove(x + dx, y + dy, n_cols, n_rows, grid, visited)
        && is_edge_node(x + dx, y + dy, grid)
      ) {
        addNodeToList(x, y, path_nodes, visited);
        found_connecting_edge_node = true;
        break;
      }
    }

    if (!found_connecting_edge_node) {
      done = true;
    }
  }

  // Log
  // printPathNodes(pathNodes);
  std::list<Point_t> full_path;
  for (std::list<gridNode_t>::iterator it = path_nodes.begin(); it != path_nodes.end(); ++it){
    Point_t new_point = { it->pos.x, it->pos.y };
    full_path.push_back(new_point);
  }

  return full_path;
}


bool OuterLoop::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                         std::vector<geometry_msgs::PoseStamped>& plan)
{
  if (!initialized_)
  {
    ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
    return false;
  }
  else
  {
    ROS_INFO("Initialized!");
  }

  //clear the plan, just in case
  plan.clear();
  costmap_ = costmap_ros_->getCostmap();

  clock_t begin = clock();
  Point_t startPoint;

  /********************** Get grid from server **********************/
  std::vector<std::vector<bool> > grid;
  nav_msgs::GetMap grid_req_srv;
  ROS_INFO("Requesting grid...");
  if (!cpp_grid_client_.call(grid_req_srv))
  {
    ROS_ERROR("Could not retrieve grid from map_server");
    return false;
  }
  ROS_INFO("grid recieved!!");

  ROS_INFO("Parsing grid to internal representation...");
  if (!parseCostmap(costmap_, grid, robot_radius_ * 2, tool_radius_ * 2, start, startPoint))
  {
    ROS_ERROR("Could not parse retrieved grid");
    return false;
  }
  ROS_INFO("grid parsed!!");

#ifdef DEBUG_PLOT
  ROS_INFO("Start grid is:");
  std::list<Point_t> printPath;
  printPath.push_back(startPoint);
  printGrid(grid, grid, printPath);
#endif

  std::list<Point_t> outer_loop_goal_points = outer_loop(grid,
                                              startPoint);

  std::list<Point_t> full_path;
  full_path.splice(full_path.end(), outer_loop_goal_points);

  ROS_INFO("naive cpp completed!");
  ROS_INFO("Converting path to plan");

  parsePointlist2Plan(start, full_path, plan);

  // TODO(CesarLopez): Check if global path should be calculated repetitively or just kept
  // (also controlled by planner_frequency parameter in move_base namespace)

  ROS_INFO("Publishing outer loop plan!");
  publishPlan(plan);
  ROS_INFO("Outer loop plan published!");
  ROS_DEBUG("Outer loop plan published!");

  clock_t end = clock();
  double elapsed_secs = static_cast<double>(end - begin) / CLOCKS_PER_SEC;
  std::cout << "elapsed time: " << elapsed_secs << "\n";

  return true;
}
}  // namespace full_coverage_path_planner
