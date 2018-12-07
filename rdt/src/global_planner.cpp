#include <pluginlib/class_list_macros.h>
#include <rdt/init.h>
#include <rdt/make_plan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <string>
#include "global_planner.h"

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(global_planner::GlobalPlanner, nav_core::BaseGlobalPlanner)

using namespace std;

//Default Constructor
namespace global_planner {

    ros::NodeHandle n;

GlobalPlanner::GlobalPlanner (){
}

GlobalPlanner::GlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    //ros::init(nullptr, nullptr, "rdt_server_client");
    initialize(name, costmap_ros);
}

nav_msgs::OccupancyGrid GlobalPlanner::costmap_2d_to_grid(costmap_2d::Costmap2DROS* costmap_ros){
    //code taken from costmap_2d_publisher
    static char* cost_translation_table_;
    /////transaltion table 
    cost_translation_table_ = new char[256];

    // special values:
    cost_translation_table_[0] = 0;  // NO obstacle
    cost_translation_table_[253] = 99;  // INSCRIBED obstacle
    cost_translation_table_[254] = 100;  // LETHAL obstacle
    cost_translation_table_[255] = -1;  // UNKNOWN

    // regular cost values scale the range 1 to 252 (inclusive) to fit
    // into 1 to 98 (inclusive).
    for (int i = 1; i < 253; i++)
    {
      cost_translation_table_[ i ] = char(1 + (97 * (i - 1)) / 251);
    }
    ////prepare grid
    nav_msgs::OccupancyGrid grid_;
    double resolution = costmap_ros->getCostmap()->getResolution();

    grid_.header.frame_id = std::string("/map"); //FIXME
    grid_.header.stamp = ros::Time::now();
    grid_.info.resolution = resolution;

    grid_.info.width = costmap_ros->getCostmap()->getSizeInCellsX();
    grid_.info.height = costmap_ros->getCostmap()->getSizeInCellsY();

    double wx, wy;
    costmap_ros->getCostmap()->mapToWorld(0, 0, wx, wy);
    grid_.info.origin.position.x = wx - resolution / 2;
    grid_.info.origin.position.y = wy - resolution / 2;
    grid_.info.origin.position.z = 0.0;
    grid_.info.origin.orientation.w = 1.0;
    //saved_origin_x_ = costmap_ros->getCostmap()->getOriginX();
    //saved_origin_y_ = costmap_ros->getCostmap()->getOriginY();

    grid_.data.resize(grid_.info.width * grid_.info.height);

    unsigned char* data = costmap_ros->getCostmap()->getCharMap();
    for (unsigned int i = 0; i < grid_.data.size(); i++)
    {
        grid_.data[i] = cost_translation_table_[ data[ i ]];
    }

    return grid_;    
}

void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    //create service client
    ros::ServiceClient client = n.serviceClient<rdt::init>("rdt_server_init");
    //get grid
    nav_msgs::OccupancyGrid grid = costmap_2d_to_grid(costmap_ros);
    //make message
    rdt::init srv;
    srv.request.grid = grid;
    //call service
    if (client.call(srv)) {
        //do nothing
    }
    else {
        ROS_ERROR("Failed to call service rdt_server (global_planner)");
        return;
    }

}

bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan ){
    //create service client
    ros::ServiceClient client = n.serviceClient<rdt::make_plan>("rdt_server_make_plan");
    //make message
    rdt::make_plan srv;
    srv.request.start = start;
    srv.request.goal = goal;
    //call service
    if (client.call(srv)) {
        //check if no path found
        if (srv.response.path.size() == 1) {
            ROS_ERROR("No path found");
            return false;
        }
        for (std::vector<geometry_msgs::PoseStamped>::iterator it = srv.response.path.begin() ; it != srv.response.path.end(); ++it) {
            plan.push_back(*it);
        }
        plan.push_back(goal);
    }
    else {
        ROS_ERROR("Failed to call service rdt_server (global_planner)");
        return false;
    }

    return true;
}
};