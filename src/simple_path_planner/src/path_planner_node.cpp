// Include necessary ROS and standard library headers

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <set>

// Global flags to track validity of start, goal and map data
bool is_start_valid = false;
bool is_goal_valid = false;
bool is_map_valid = false;



// Add flags to track if the messages have been printed
bool path_created = false;
bool goal_reached = false;

// Subscriber (start_position,goal_position, map_data)
geometry_msgs::Point start_position;
geometry_msgs::Point goal_position;
nav_msgs::OccupancyGrid map_data;
nav_msgs::OccupancyGrid::ConstPtr global_map_ptr;

// Publisher for visualization markers
ros::Publisher marker_publisher;

// Parameters for heuristic type and obstacle penalty
std::string heuristic_type;



int penalty;

/**
 * Convert world coordinates to grid cell indices
 * Uses map origin and resolution for conversion
 */
std::pair<int, int> worldToGridCoords(const geometry_msgs::Point& point)
{
    int grid_x = (point.x - map_data.info.origin.position.x) / map_data.info.resolution;
    int grid_y = (point.y - map_data.info.origin.position.y) / map_data.info.resolution;
    return {grid_x, grid_y};
}

/**
 * Check if a point is in free space (not obstacle/unknown)
 * Verifies: 1. Map validity 2. Within bounds 3. Cell value < 80 (free)
 */
bool checkIfFreeSpace(const geometry_msgs::Point& point)
{
    if (!is_map_valid)
    {
        ROS_WARN("Map data not available.");
        return false;
    }

    auto [grid_x, grid_y] = worldToGridCoords(point);

    // Check if the point is within the map bounds
    if (grid_x < 0 || grid_x >= map_data.info.width || grid_y < 0 || grid_y >= map_data.info.height)
    {
        ROS_WARN("Point is out of map bounds.");
        return false;
    }

    int map_index = grid_y * map_data.info.width + grid_x;
    int cell_value = map_data.data[map_index];
    
    // Check for unknown or obscticles cells
    if (cell_value == -1)  
    {
        ROS_WARN("Point lies in an unknown area.");
        return false;
    }
    else if (cell_value >= 80)  // Blocked (black)
    {
        ROS_WARN("Point lies in an obstacle.");
        return false;
    }
    
    return true;
}

/**
 * Callback for map data updates
 * Stores map information and resets path status flags
 */
void mapDataCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    map_data = *msg;
    global_map_ptr = msg;
    is_map_valid = true;
    
    path_created = false;
    goal_reached = false;
    
    ROS_INFO("Map received with resolution: %f", map_data.info.resolution);
}


/**
 * Publish a 3D sphere marker at specified coordinates
 * Used for visualizing start/goal points
 */
void publishPointMarker(const geometry_msgs::Point& point, int id, float r, float g, float b)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "points";
    marker.id = id;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0;
    marker.pose.position = point;
    marker_publisher.publish(marker);
}

/**
 * Callback for initial pose (start position)
 * Validates position and publishes green marker
 */
void startPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    start_position = msg->pose.pose.position;
    is_start_valid = checkIfFreeSpace(start_position);
    
    path_created = false;
    goal_reached = false;
    
    
    if (is_start_valid == true){
    publishPointMarker(start_position, 1, 0.0, 1.0, 0.0); // Green sphere for start point
    }
    //std::cout << is_start_valid << std::endl;
    
    ROS_INFO("Created Start Point: x=%f, y=%f", start_position.x, start_position.y);
}

/**
 * Callback for goal pose updates
 * Validates position and publishes blue marker
 */
void goalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    goal_position = msg->pose.position;
    is_goal_valid = checkIfFreeSpace(goal_position);
    
    
    path_created = false;
    goal_reached = false;
    
    
    if (is_goal_valid == true){
    publishPointMarker(goal_position, 2, 0.0, 0.0, 1.0); // Blue sphere for goal point
    }
    //std::cout << is_goal_valid << std::endl;
    
    ROS_INFO("Created Goal Point: x=%f, y=%f", goal_position.x, goal_position.y);
}

// Struct representing a node in the A* search
struct PathNode
{
    int x, y;
    double cost, heuristic;
    bool operator>(const PathNode& other) const { 
    //ROS_INFO("STARTING OPEN SET");
    //ROS_INFO("Current Point Values is x:%d, y:%d, cost:%lf, h:%lf  --> Goal Point Values is x:%d, y:%d, cost: %lf, h: %lf, Boolean:%d", x,y,cost,heuristic,other.x, other.y, other.cost, other.heuristic, (cost + heuristic) > (other.cost + other.heuristic));
    //ROS_INFO("ENDING OPEN SET");
    return (cost + heuristic) > (other.cost + other.heuristic); 
    
    }
};


/**
 * Calculate heuristic distance between two nodes
 * Supports Euclidean, Chebyshev, and Manhattan distances
 */
 
double calculateHeuristic(const PathNode& a, const PathNode& b)

{
    //ROS_INFO("My Hurastic is  %s", heuristic_type.c_str());
    
    if(heuristic_type =="e"){ // Euclieden Distance
    //ROS_INFO("heuristic type is Euclieden Distance");
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));    
    
    }
    else if (heuristic_type=="c"){ // Chebyshev  Distance
    //ROS_INFO("heuristic type is Chebyshev  Distance");
    return std::max(std::abs(a.x - b.x), std::abs(a.y - b.y));
    }
    else if(heuristic_type=="m"){ // Manhattan Distance
    //ROS_INFO("heuristic type is Manhattan Distance");
    return std::abs(a.x - b.x) + std::abs(a.y - b.y);
    }
    else {
        //ROS_WARN("Unsupported heuristic type '%s', defaulting to Manhattan distance.", heuristic_type.c_str());
        return std::abs(a.x - b.x) + std::abs(a.y - b.y);  // Default to Manhattan
    }
    //return std::abs(a.x - b.x) + std::abs(a.y - b.y);
}



/**
 * Calculate obstacle proximity penalty for a grid cell
 * Checks neighboring cells and returns penalty based on distance
 */
double getObstaclePenalty(int x, int y)
{
    int map_index = y * map_data.info.width + x;
    int cell_value = map_data.data[map_index];

    if (cell_value >= 80)  // Direct obstacle
      
        //ROS_INFO("We get Penalty Sameer!: %d", penalty);
        return penalty;

    // Check 5x5 neighborhood for obstacles
    double min_distance = std::numeric_limits<double>::max();
    for (int dy = -2; dy <= 2; ++dy)
    {
        for (int dx = -2; dx <= 2; ++dx)
        {
            int nx = x + dx;
            int ny = y + dy;
            if (nx >= 0 && nx < map_data.info.width && ny >= 0 && ny < map_data.info.height)
            {
                int n_index = ny * map_data.info.width + nx;
                if (map_data.data[n_index] >= 80)  // Obstacle detected
                {
                    double distance = std::sqrt(dx * dx + dy * dy);
                    min_distance = std::min(min_distance, distance);
                }
            }
        }
    }

    return (min_distance < std::numeric_limits<double>::max()) ? (penalty / min_distance) : 0.0;
}

/**
 * A* pathfinding algorithm implementation
 * Returns path from start to goal as world coordinates
 */
std::vector<geometry_msgs::Point> findOptimalPath()
{
    // Convert start/goal to grid coordinates
    auto [start_x, start_y] = worldToGridCoords(start_position);
    auto [goal_x, goal_y] = worldToGridCoords(goal_position);
    
    // Priority queue for open set (sorted by total cost)
    std::priority_queue<PathNode, std::vector<PathNode>, std::greater<PathNode>> open_set;
    std::unordered_map<int, int> came_from;
    std::unordered_map<int, double> cost_so_far;
    
    int map_width = map_data.info.width;
    
    // Initialize open set with start node
    open_set.push({start_x, start_y, 0, calculateHeuristic({start_x, start_y}, {goal_x, goal_y})});
    cost_so_far[start_y * map_width + start_x] = 0;

      // 8-directional movement (includes diagonals)
    std::vector<std::pair<int, int>> directions = {{0, 1}, {0, -1}, {1, 0}, {-1, 0}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};
    std::set<std::pair<int, int>> closed_set; // Visited nodes

    while (!open_set.empty())
    {	
    
        PathNode current = open_set.top();
        
        open_set.pop();
	
	// Check if goal reached
        if (current.x == goal_x && current.y == goal_y)
        {
            std::vector<geometry_msgs::Point> path;
            int index = goal_y * map_width + goal_x;
            
            if(goal_reached == false){
            	ROS_INFO("Reached Goal Successfully");
            	goal_reached == true;
            }
            
            // Reconstruct path from goal to start
            while (came_from.count(index))
            {
                int prev_x = index % map_width;
                int prev_y = index / map_width;
                geometry_msgs::Point point;
                point.x = prev_x * map_data.info.resolution + map_data.info.origin.position.x;
                point.y = prev_y * map_data.info.resolution + map_data.info.origin.position.y;
                path.push_back(point);
                index = came_from[index];
            }
            return path;
        }

        closed_set.insert({current.x, current.y});
        
        // Explore neighbors
        for (auto [dx, dy] : directions)
        {
            int next_x = current.x + dx;
            int next_y = current.y + dy;
            
            // Convert to world coordinates for validation
            geometry_msgs::Point next_point;
            next_point.x = next_x * map_data.info.resolution + map_data.info.origin.position.x;
            next_point.y = next_y * map_data.info.resolution + map_data.info.origin.position.y;
            next_point.z = 0;
	     
	     // Check validity and closed set
            if (next_x >= 0 && next_x < map_width && next_y >= 0 && next_y < map_data.info.height &&
                checkIfFreeSpace(next_point) && closed_set.find({next_x, next_y}) == closed_set.end())
            {
                double obstacle_penalty = getObstaclePenalty(next_x, next_y);
                double new_cost = cost_so_far[current.y * map_width + current.x] + 1 + obstacle_penalty;
                int next_index = next_y * map_width + next_x;
                
                
                // Update cost and path if better
                
                if (!cost_so_far.count(next_index) || new_cost < cost_so_far[next_index])
                {
                    cost_so_far[next_index] = new_cost;
                    double h = calculateHeuristic({next_x, next_y}, {goal_x, goal_y});
                    
                    open_set.push({next_x, next_y, new_cost, h});
                    //std::cout << open_set << std::endl;
                    //ROS_INFO(open_set);
                    came_from[next_index] = current.y * map_width + current.x;
                }
            }
        }
    }
    return {};  // Return empty path if no valid path is found
}


/**
 * Publish path visualization markers
 * Uses LINE_STRIP marker to show planned path
 */
void publishPathMarkers()
{
    if (!is_start_valid || !is_goal_valid || !is_map_valid)
        return;

    visualization_msgs::Marker path_marker;
    path_marker.header.frame_id = "map";
    path_marker.header.stamp = ros::Time::now();
    path_marker.ns = "path";
    path_marker.id = 3;
    path_marker.type = visualization_msgs::Marker::LINE_STRIP;
    path_marker.scale.x = 0.1;
    // Bright Orange Color
    path_marker.color.r = 1.0;
    path_marker.color.g = 0.2;
    path_marker.color.b = 0.0;
    path_marker.color.a = 1.0;

    auto path = findOptimalPath();
    
    if (goal_reached ==false){
	    if (!path.empty()) {
		    for (const auto& point : path)
			path_marker.points.push_back(point);
		    
		    marker_publisher.publish(path_marker);
		    ROS_INFO("Path Created");
		    }
		    
	    else{
		ROS_WARN("No valid path found.");
		}
	    //std::cout << "Path Created" << std::endl;
	    goal_reached = true;
	   }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "Simple_path_planner");
    ros::NodeHandle nh;
    
    
    // Print the heuristic type
    ros::NodeHandle node_handle("~");  // Use private NodeHandle
    
    node_handle.getParam("heuristic_type", heuristic_type);

    ROS_INFO("Heuristic Type is: %s", heuristic_type.c_str());
    
    
    // Print the panalty type
    ros::NodeHandle node_handle_p("~");  // Private namespace
    
    node_handle_p.getParam("penalty", penalty);

    ROS_INFO("Penalty is: %d", penalty);

    // Subscribe to required topics
    ros::Subscriber start_sub = nh.subscribe("initialpose", 10, startPoseCallback);
    ros::Subscriber goal_sub = nh.subscribe("move_base_simple/goal", 10, goalPoseCallback);
    ros::Subscriber map_sub = nh.subscribe("map", 1, mapDataCallback);

    // Advertise marker publisher
    marker_publisher = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    ros::Rate rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        publishPathMarkers();
        rate.sleep();
    }
    return 0;
}

