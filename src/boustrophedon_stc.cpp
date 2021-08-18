//
// Copyright [2020] Nobleo Technology"  [legal/copyright]
//
#include <algorithm>
#include <iostream>
#include <list>
#include <string>
#include <vector>

#include "full_coverage_path_planner/boustrophedon_stc.h"
#include <pluginlib/class_list_macros.h>

// register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(full_coverage_path_planner::BoustrophedonSTC, nav_core::BaseGlobalPlanner)

int pattern_dir_ = point;
int robot_dir_ = point;
// save plan
std::vector<geometry_msgs::PoseStamped> prev_plan_;

namespace full_coverage_path_planner
{

void BoustrophedonSTC::coverage_grid_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
  visited_grid_ = *msg;
}

void BoustrophedonSTC::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
  if (!initialized_)
  {
    ros::NodeHandle private_nh("~/");
    ros::NodeHandle nh, private_named_nh("~/" + name);

    // Create a publisher to visualize the plan
    plan_pub_ = private_named_nh.advertise<nav_msgs::Path>("plan", 1);
    // Try to request the cpp-grid from the cpp_grid map_server
    cpp_grid_client_ = nh.serviceClient<nav_msgs::GetMap>("static_map");
    // Get the cost map:
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros->getCostmap();
    visited_grid_ = nav_msgs::OccupancyGrid();

    // Define  robot radius (radius) parameter
    float robot_radius_default = 0.5f;
    private_named_nh.param<float>("robot_radius", robot_radius_, robot_radius_default);
    // Define  tool radius (radius) parameter
    float tool_radius_default = 0.5f;
    private_named_nh.param<float>("tool_radius", tool_radius_, tool_radius_default);
    initialized_ = true;

    string coverage_grid_topic_default = "/coverage_grid";
    private_named_nh.param<string>("coverage_grid", coverage_grid_topic_, coverage_grid_topic_default);
    // Get the visited occupancy grid
    coverage_grid_sub = nh.subscribe(coverage_grid_topic_, 0, &BoustrophedonSTC::coverage_grid_callback, this);
  }
}

std::list<gridNode_t> BoustrophedonSTC::boustrophedon(std::vector<std::vector<bool> > const& grid,
                                                      std::list<gridNode_t>& init,
                                                      std::vector<std::vector<bool> >& visited)
{
  int dx, dy, x2, y2, i, nRows = grid.size(), nCols = grid[0].size();
  // Mountain pattern filling of the open space
  // Copy incoming list to 'end'
  std::list<gridNode_t> pathNodes(init);

  // Set starting pos
  x2 = pathNodes.back().pos.x;
  y2 = pathNodes.back().pos.y;
  // set initial direction based on...
  // 1. current robot heading (robot_dir_)
  // 2. pattern direction (pattern_dir_)
  // 3. space visible from initial pos
  ROS_INFO("patern_dir_: %u", pattern_dir_);
  ROS_INFO("robot_dir_: %u", robot_dir_);

  if (pattern_dir_ == north || pattern_dir_ == south) {
    robot_dir_ = dirWithMostSpace(x2, y2, nCols, nRows, grid, visited, north);
    if (robot_dir_ == south) {
      robot_dir_ = dirWithMostSpace(x2, y2, nCols, nRows, grid, visited, south);
    }
  } else if (pattern_dir_ == east || pattern_dir_ == west) {
    robot_dir_ = dirWithMostSpace(x2, y2, nCols, nRows, grid, visited, east);
    if (robot_dir_ == west) {
      robot_dir_ = dirWithMostSpace(x2, y2, nCols, nRows, grid, visited, west);
    }
  } else { // pattern_dir_ == point
    // keep robot_dir_ as robot's initial heading
  } 

  // set dx and dy based on robot_dir_
  switch(robot_dir_) {
    case east: // 1
      dx = +1;
      dy = 0;
      break;
    case west: // 2
      dx = -1;
      dy = 0;
      break;
    case north: // 3
      dx = 0;
      dy = +1;
      break;
    case south: // 4
      dx = 0;
      dy = -1;
      break;
    default:
      ROS_ERROR("Full Coverage Path Planner: NO INITIAL ROBOT DIRECTION CALCULATED. This is a logic error that must be fixed by editing boustrophedon_stc.cpp. Will travel east for now.");
      robot_dir_ = east;
      dx = +1;
      dy = 0;
      break;
  }

  bool done = false;
  while (!done)
  {
    // 1. drive straight until not a valid move (hit occupied cell or at end of map)
    bool hitWall = false;
    while(!hitWall) {
      x2 += dx;
      y2 += dy;
      if (!validMove(x2, y2, nCols, nRows, grid, visited))
      {
        hitWall = true;
        x2 = pathNodes.back().pos.x;
        y2 = pathNodes.back().pos.y;
        break;
      }
      if (!hitWall) {
        addNodeToList(x2, y2, pathNodes, visited);
      }
    }

    // 2. check left and right after hitting wall, then change direction
    if (robot_dir_ == north || robot_dir_ == south)
    {
      if (!validMove(x2 + 1, y2, nCols, nRows, grid, visited)
        && !validMove(x2 - 1, y2, nCols, nRows, grid, visited)) {
        // dead end, exit
        done = true;
        break;
      } else if (!validMove(x2 + 1, y2, nCols, nRows, grid, visited)) {
        // east is occupied, travel towards west
        x2--;
        pattern_dir_ = west;
      } else if (!validMove(x2 - 1, y2, nCols, nRows, grid, visited)) {
        // west is occupied, travel towards east
        x2++;
        pattern_dir_ = east;
      } else {
        // both sides are opened. If don't have a prefered turn direction, travel towards most open direction
        if (!(pattern_dir_ == east || pattern_dir_ == west)) {
          if (validMove(x2, y2 + 1, nCols, nRows, grid, visited)) {
            pattern_dir_ = dirWithMostSpace(x2, y2, nCols, nRows, grid, visited, north);        
          } else {
            pattern_dir_ = dirWithMostSpace(x2, y2, nCols, nRows, grid, visited, south);
          }
          ROS_INFO("rotation dir with most space successful");
        }
        if (pattern_dir_ == east) {
            x2++;
        } else if (pattern_dir_ == west) {
            x2--;
        }
      }

      // add Node to List
      addNodeToList(x2, y2, pathNodes, visited);

      // change direction 180 deg
      if (robot_dir_ == north) {
        robot_dir_ = south;
        dy = -1;
      } else if (robot_dir_ == south) {
        robot_dir_ = north;
        dy = 1;
      }
    }
    else if (robot_dir_ == east || robot_dir_ == west)
    {
      if (!validMove(x2, y2 + 1, nCols, nRows, grid, visited)
        && !validMove(x2, y2 - 1, nCols, nRows, grid, visited)) {
        // dead end, exit
        done = true;
        break;
      } else if (!validMove(x2, y2 + 1, nCols, nRows, grid, visited)) {
        // north is occupied, travel towards south
        y2--;
        pattern_dir_ = south;
      } else if (!validMove(x2, y2 - 1, nCols, nRows, grid, visited)) {
        // south is occupied, travel towards north
        y2++;
        pattern_dir_ = north;
      } else {
        // both sides are opened. If don't have a prefered turn direction, travel towards farthest edge
        if (!(pattern_dir_ == north || pattern_dir_ == south)) {
          if (validMove(x2 + 1, y2, nCols, nRows, grid, visited)) {
            pattern_dir_ = dirWithMostSpace(x2, y2, nCols, nRows, grid, visited, east);        
          } else {
            pattern_dir_ = dirWithMostSpace(x2, y2, nCols, nRows, grid, visited, west);
          }
        }
        if (pattern_dir_ == north) {
            y2++;
        } else if (pattern_dir_ == south) {
            y2--;
        }
      }

      // add Node to List
      addNodeToList(x2, y2, pathNodes, visited);
      
      // change direction 180 deg
      if (robot_dir_ == east) {
        robot_dir_ = west;
        dx = -1;
      } else if (robot_dir_ == west) {
        robot_dir_ = east;
        dx = 1;
      }
    }
  }
  // Log
  // printPathNodes(pathNodes);
  return pathNodes;
}

std::list<Point_t> BoustrophedonSTC::boustrophedon_stc(std::vector<std::vector<bool> > const& grid,
                                          std::vector<std::vector<bool> > const& visited_grid,
                                          geometry_msgs::PoseStamped& init,
                                          int &multiple_pass_counter,
                                          int &visited_counter)
{
  int x, y, nRows = grid.size(), nCols = grid[0].size();
  double yaw;
  // Initial node is initially set as visited so it does not count
  multiple_pass_counter = 0;
  visited_counter = 0;

  std::vector<std::vector<bool>> visited;
  visited = visited_grid;  // Copy visited grid
  x = init.pose.position.x;
  y = init.pose.position.y;
  tf2::Quaternion heading_quaternion(init.pose.orientation.x,
                                     init.pose.orientation.y,
                                     init.pose.orientation.z,
                                     init.pose.orientation.w);
  heading_quaternion.normalize();
  yaw = heading_quaternion.getAngle();

  // Set initial robot_dir:
  ROS_INFO("initial robot angle: %f", yaw);
  if (M_PI*(1.0f/4.0f) <= yaw && yaw < M_PI*(3.0f/4.0f)) {
    robot_dir_ = north;
  } else if (M_PI*(3.0f/4.0f) <= yaw && yaw < M_PI*(5.0f/4.0f)) {
    robot_dir_ = west;
  } else if (M_PI*(5.0f/4.0f) <= yaw && yaw < M_PI*(7.0f/4.0f)) {
    robot_dir_ = south;
  } else {
    robot_dir_ = east;
  }
  // add initial point to pathNodes
  std::list<gridNode_t> pathNodes;
  std::list<Point_t> fullPath;
  addNodeToList(x, y, pathNodes, visited);

  std::list<Point_t> goals = map_2_goals(visited, eNodeOpen);  // Retrieve all goalpoints (Cells not visited)
  std::list<gridNode_t>::iterator it;

#ifdef DEBUG_PLOT
  ROS_INFO("Grid before walking is: ");
  printGrid(grid, visited, fullPath);
#endif

  while (goals.size() != 0)
  {
    // boustrophedon pattern from current position
    pathNodes = boustrophedon(grid, pathNodes, visited);
#ifdef DEBUG_PLOT
    ROS_INFO("Visited grid updated after boustrophedon:");
    printGrid(grid, visited, pathNodes, PatternStart, pathNodes.back());
#endif

    for (it = pathNodes.begin(); it != pathNodes.end(); ++it)
    {
      Point_t newPoint = { it->pos.x, it->pos.y };
      visited_counter++;
      fullPath.push_back(newPoint);
    }

    goals = map_2_goals(visited, eNodeOpen);  // Retrieve remaining goalpoints

    // Remove all elements from pathNodes list except last element.
    // The last point is the starting point for a new search and A* extends the path from there on
    pathNodes.erase(pathNodes.begin(), --(pathNodes.end()));
    visited_counter--;  // First point is already counted as visited
    // Plan to closest open Node using A*
    // `goals` is essentially the map, so we use `goals` to determine the distance from the end of a potential path
    //    to the nearest free space
    bool resign = a_star_to_open_space(grid, pathNodes.back(), 1, visited, goals, pathNodes);
    if (resign)
    {
      ROS_WARN("A_star_to_open_space is resigning! This may be due to the open cells outside of the obstacle boundary. Goals Left: %u", goals.size());
      break;
    }

    // Update visited grid
    for (it = pathNodes.begin(); it != pathNodes.end(); ++it)
    {
      if (visited[it->pos.y][it->pos.x])
      {
        multiple_pass_counter++;
      }
      visited[it->pos.y][it->pos.x] = eNodeVisited;
    }
    if (pathNodes.size() > 0)
    {
      multiple_pass_counter--;  // First point is already counted as visited
    }

#ifdef DEBUG_PLOT
    ROS_INFO("Grid with path marked as visited is:");
    gridNode_t PatternStart = pathNodes.back();
    printGrid(grid, visited, pathNodes, pathNodes.front(), pathNodes.back());
#endif

  }

  return fullPath;
}

bool BoustrophedonSTC::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                         std::vector<geometry_msgs::PoseStamped>& plan)
{
  if (!initialized_)
  {
    ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
    return false;
  }

  costmap_ = costmap_ros_->getCostmap();

  clock_t begin = clock();
  geometry_msgs::PoseStamped startPose;

  std::vector<std::vector<bool> > grid;
  std::vector<std::vector<bool> > visited_grid;
  ROS_INFO("Parsing costmap to internal representation...");
  if (!parseCostmap(costmap_, grid, robot_radius_ * 2, tool_radius_ * 2, start, startPose))
  {
    ROS_ERROR("Could not parse costmap");
    return false;
  }
  ROS_INFO("costmap parsed!!");

  ROS_INFO("Parsing visited areas to internal representation...");
  if (!parseGrid(visited_grid_, visited_grid, robot_radius_ * 2, tool_radius_ * 2))
  {
    ROS_ERROR("Could not parse visited grid");
    return false;
  }
  ROS_INFO("grid parsed!!");

#ifdef DEBUG_PLOT
  ROS_INFO("Start grid is:");
  std::list<Point_t> printPath;
  printPath.push_back(startPoint);
  printGrid(grid, grid, printPath);
#endif

  std::list<Point_t> goalPoints = boustrophedon_stc(grid,
                                              visited_grid,
                                              startPose,
                                              boustrophedon_cpp_metrics_.multiple_pass_counter,
                                              boustrophedon_cpp_metrics_.visited_counter);
  ROS_INFO("naive cpp completed!");
  ROS_INFO("Converting path to plan");

  parsePointlist2Plan(start, goalPoints, plan);
  // Print some metrics:
  boustrophedon_cpp_metrics_.accessible_counter = boustrophedon_cpp_metrics_.visited_counter
                                            - boustrophedon_cpp_metrics_.multiple_pass_counter;
  boustrophedon_cpp_metrics_.total_area_covered = (4.0 * tool_radius_ * tool_radius_) * boustrophedon_cpp_metrics_.accessible_counter;
  ROS_INFO("Total visited: %d", boustrophedon_cpp_metrics_.visited_counter);
  ROS_INFO("Total re-visited: %d", boustrophedon_cpp_metrics_.multiple_pass_counter);
  ROS_INFO("Total accessible cells: %d", boustrophedon_cpp_metrics_.accessible_counter);
  ROS_INFO("Total accessible area: %f", boustrophedon_cpp_metrics_.total_area_covered);

  // TODO(CesarLopez): Check if global path should be calculated repetitively or just kept
  // (also controlled by planner_frequency parameter in move_base namespace)

  ROS_INFO("Publishing plan!");
  publishPlan(plan);
  ROS_INFO("Plan published!");
  ROS_DEBUG("Plan published");

  clock_t end = clock();
  double elapsed_secs = static_cast<double>(end - begin) / CLOCKS_PER_SEC;
  std::cout << "elapsed time: " << elapsed_secs << "\n";

  return true;
}
}  // namespace full_coverage_path_planner
