/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/
#include <lnglobal_planner/planner_core.h>
#include <pluginlib/class_list_macros.h>
#include <tf/transform_listener.h>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>

#include <lnglobal_planner/dijkstra.h>
#include <lnglobal_planner/astar.h>
#include <lnglobal_planner/grid_path.h>
#include <lnglobal_planner/gradient_path.h>
#include <lnglobal_planner/quadratic_calculator.h>

//register this planner as a BaseGlobalPlanner plugin
//将此计划者注册为BaseGlobalPlanner插件
PLUGINLIB_EXPORT_CLASS(lnglobal_planner::GlobalPlanner, nav_core::BaseGlobalPlanner)

namespace lnglobal_planner {

//将地图外围轮廓设为障碍
void GlobalPlanner::outlineMap(unsigned char* costarr, int nx, int ny, unsigned char value) {
    unsigned char* pc = costarr;
    for (int i = 0; i < nx; i++)
        *pc++ = value;
    pc = costarr + (ny - 1) * nx;
    for (int i = 0; i < nx; i++)
        *pc++ = value;
    pc = costarr;
    for (int i = 0; i < ny; i++, pc += nx)
        *pc = value;
    pc = costarr + nx - 1;
    for (int i = 0; i < ny; i++, pc += nx)
        *pc = value;
}

GlobalPlanner::GlobalPlanner() :
        costmap_(NULL), initialized_(false), allow_unknown_(true) {
}

GlobalPlanner::GlobalPlanner(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id) :
        costmap_(NULL), initialized_(false), allow_unknown_(true) {
    //initialize the planner
    //初始化路径规划期
    initialize(name, costmap, frame_id);
}

GlobalPlanner::~GlobalPlanner() {
    if (p_calc_)
        delete p_calc_;
    if (planner_)
        delete planner_;
    if (path_maker_)
        delete path_maker_;
    if (dsrv_)
        delete dsrv_;
}

void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
    initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
}

void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id) {
    if (!initialized_) {
        ros::NodeHandle private_nh("~/" + name);
        costmap_ = costmap;
        frame_id_ = frame_id;

        unsigned int cx = costmap->getSizeInCellsX(), cy = costmap->getSizeInCellsY();
        //是否使用navfn的规划方式。
        private_nh.param("old_navfn_behavior", old_navfn_behavior_, false);
        if(!old_navfn_behavior_)
            convert_offset_ = 0.5;
        else
            convert_offset_ = 0.0;

        bool use_quadratic;
        //如果设置为true，则使用二次函数近似函数，否则使用更简单的计算方式，这样更节省硬件计算资源
        private_nh.param("use_quadratic", use_quadratic, true);
        if (use_quadratic)
            p_calc_ = new QuadraticCalculator(cx, cy);
        else
            //计算一个点的可行性
            p_calc_ = new PotentialCalculator(cx, cy);

        bool use_dijkstra;
        //如果为true，则使用Dijkstra算法，否则使用A*算法
        private_nh.param("use_dijkstra", use_dijkstra, true);
        if (use_dijkstra)
        {
            DijkstraExpansion* de = new DijkstraExpansion(p_calc_, cx, cy);
            if(!old_navfn_behavior_)
                de->setPreciseStart(true);
            planner_ = de;
        }
        else
            planner_ = new AStarExpansion(p_calc_, cx, cy);

        bool use_grid_path;
        //如果为true，则沿栅格边界创建路径。否则，使用梯度下降的方法。
        private_nh.param("use_grid_path", use_grid_path, false);
        if (use_grid_path)
            //栅格路径，从终点开始找上下或者左右四个中最小的栅格直到起点。
            path_maker_ = new GridPath(p_calc_);
        else
            //梯度路径，从周围八个栅格中找到下降梯度最大的点。
            path_maker_ = new GradientPath(p_calc_);

        //给路径加方向实例化
        orientation_filter_ = new OrientationFilter();

        //发布话题  plan   potential
        plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
        potential_pub_ = private_nh.advertise<nav_msgs::OccupancyGrid>("potential", 1);

        //允许规划unknown中的全局路径
        private_nh.param("allow_unknown", allow_unknown_, true);
        planner_->setHasUnknown(allow_unknown_);
        //指定可选窗口的x,y大小以限定规划器的工作空间
        private_nh.param("planner_window_x", planner_window_x_, 0.0);
        private_nh.param("planner_window_y", planner_window_y_, 0.0);
        //允许的目标点误差范围
        private_nh.param("default_tolerance", default_tolerance_, 0.0);
        private_nh.param("publish_scale", publish_scale_, 100);

        //get the tf prefix
        ros::NodeHandle prefix_nh;
        tf_prefix_ = tf::getPrefixParam(prefix_nh);

        //发布一个make_plan的service
        make_plan_srv_ = private_nh.advertiseService("make_plan", &GlobalPlanner::makePlanService, this);

        //动态调参
        dsrv_ = new dynamic_reconfigure::Server<lnglobal_planner::GlobalPlannerConfig>(ros::NodeHandle("~/" + name));
        dynamic_reconfigure::Server<lnglobal_planner::GlobalPlannerConfig>::CallbackType cb = boost::bind(
                &GlobalPlanner::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);

        initialized_ = true;
    } else
        ROS_WARN("This planner has already been initialized, you can't call it twice, doing nothing");

}

void GlobalPlanner::reconfigureCB(lnglobal_planner::GlobalPlannerConfig& config, uint32_t level) {
    planner_->setLethalCost(config.lethal_cost);
    path_maker_->setLethalCost(config.lethal_cost);
    //
    planner_->setNeutralCost(config.neutral_cost);
    planner_->setFactor(config.cost_factor);
    publish_potential_ = config.publish_potential;
    orientation_filter_->setMode(config.orientation_mode);
    orientation_filter_->setWindowSize(config.orientation_window_size);
}

//将某个costmap栅格设置为free
void GlobalPlanner::clearRobotCell(const tf::Stamped<tf::Pose>& global_pose, unsigned int mx, unsigned int my) {
    if (!initialized_) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }

    //将一个栅格内的cost设为free
    //set the associated costs in the cost map to be free
    costmap_->setCost(mx, my, costmap_2d::FREE_SPACE);
}

//makeplan回调函数
bool GlobalPlanner::makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp) {
    //调用函数makeplan
    makePlan(req.start, req.goal, resp.plan.poses);

    resp.plan.header.stamp = ros::Time::now();
    resp.plan.header.frame_id = frame_id_;

    return true;
}

//将map上的坐标mx,my转化成world上的坐标wx,wy
void GlobalPlanner::mapToWorld(double mx, double my, double& wx, double& wy) {
    wx = costmap_->getOriginX() + (mx+convert_offset_) * costmap_->getResolution();
    wy = costmap_->getOriginY() + (my+convert_offset_) * costmap_->getResolution();
}

//将world上的坐标wx,wy转化为map上的坐标mx,my
bool GlobalPlanner::worldToMap(double wx, double wy, double& mx, double& my) {
    //地图的长宽
    double origin_x = costmap_->getOriginX(), origin_y = costmap_->getOriginY();
    //分辨率
    double resolution = costmap_->getResolution();

    //如果超过地图范围
    if (wx < origin_x || wy < origin_y)
        return false;

    // x = (实际点 - 长度) / 分辨率 - 转换偏移
    mx = (wx - origin_x) / resolution - convert_offset_;
    my = (wy - origin_y) / resolution - convert_offset_;

    if (mx < costmap_->getSizeInCellsX() && my < costmap_->getSizeInCellsY())
        return true;

    return false;
}

//调用下面的makePlan函数
bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                           std::vector<geometry_msgs::PoseStamped>& plan) {
    return makePlan(start, goal, default_tolerance_, plan);
}

//主要函数
//start 起始点位姿   goal 目标点位姿  tolerance 路径规划器规划处的终点容错距离   plan 路径，刚开始会清除路径
bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                           double tolerance, std::vector<geometry_msgs::PoseStamped>& plan) {
    boost::mutex::scoped_lock lock(mutex_);
    if (!initialized_) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return false;
    }

    //clear the plan, just in case
    //初始化清除路径
    plan.clear();

    ros::NodeHandle n;
    std::string global_frame = frame_id_;

    //判断起始点及目标点是否在该环境下
    //until tf can handle transforming things that are way in the past... we'll require the goal to be in our global frame
    if (tf::resolve(tf_prefix_, goal.header.frame_id) != tf::resolve(tf_prefix_, global_frame)) {
        ROS_ERROR(
                "The goal pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", tf::resolve(tf_prefix_, global_frame).c_str(), tf::resolve(tf_prefix_, goal.header.frame_id).c_str());
        return false;
    }

    if (tf::resolve(tf_prefix_, start.header.frame_id) != tf::resolve(tf_prefix_, global_frame)) {
        ROS_ERROR(
                "The start pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", tf::resolve(tf_prefix_, global_frame).c_str(), tf::resolve(tf_prefix_, start.header.frame_id).c_str());
        return false;
    }

    //获取起始点的世界坐标系坐标
    double wx = start.pose.position.x;
    double wy = start.pose.position.y;

    unsigned int start_x_i, start_y_i, goal_x_i, goal_y_i;
    double start_x, start_y, goal_x, goal_y;

    //起始点世界坐标系转map坐标系(栅格)
    if (!costmap_->worldToMap(wx, wy, start_x_i, start_y_i)) {
        ROS_WARN(
                "The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?");
        return false;
    }
    if(old_navfn_behavior_){
        start_x = start_x_i;
        start_y = start_y_i;
    }else{
        //世界坐标系转map坐标系(单位:米)
        worldToMap(wx, wy, start_x, start_y);
    }

    //获取目标坐标点
    wx = goal.pose.position.x;
    wy = goal.pose.position.y;

    //世界坐标系转map坐标系(栅格)
    if (!costmap_->worldToMap(wx, wy, goal_x_i, goal_y_i)) {
        ROS_WARN_THROTTLE(1.0,
                "The goal sent to the global planner is off the global costmap. Planning will always fail to this goal.");
        return false;
    }
    if(old_navfn_behavior_){
        goal_x = goal_x_i;
        goal_y = goal_y_i;
    }else{
        //世界坐标系转map坐标系(单位:米)
        worldToMap(wx, wy, goal_x, goal_y);
    }

    //clear the starting cell within the costmap because we know it can't be an obstacle
    tf::Stamped<tf::Pose> start_pose;
    tf::poseStampedMsgToTF(start, start_pose);
    //清除起始点单元格的代价,因为不知道是否为障碍
    clearRobotCell(start_pose, start_x_i, start_y_i);

    //获取代价地图的大小  x,y
    int nx = costmap_->getSizeInCellsX(), ny = costmap_->getSizeInCellsY();

    //make sure to resize the underlying array that Navfn uses
    //分配空间,确保使用基础数组的大小
    //QP优化设置长宽
    p_calc_->setSize(nx, ny);
    //astar设置长宽
    planner_->setSize(nx, ny);
    //梯度路径设置长宽
    path_maker_->setSize(nx, ny);
    //float指针，存储potential可行点
    potential_array_ = new float[nx * ny];

    //地图外轮廓设为障碍
    outlineMap(costmap_->getCharMap(), nx, ny, costmap_2d::LETHAL_OBSTACLE);

    //计算代价
    //主要步骤1:
    //调用A star 算法进行计算
    //计算potential_array_寻找所有的可行点,调用的是Expander::virtual bool calculatePotentials() = 0; 函数
	//calculatePotentials的方法有两种（A*、Dijkstra）,由use_dijkstra参数决定：class AStarExpansion : public Expander 、class DijkstraExpansion : public Expander 
    bool found_legal = planner_->calculatePotentials(costmap_->getCharMap(), start_x, start_y, goal_x, goal_y,
                                                    nx * ny * 2, potential_array_);

    if(!old_navfn_behavior_)
        planner_->clearEndpoint(costmap_->getCharMap(), potential_array_, goal_x_i, goal_y_i, 2);
    if(publish_potential_)
        //发布可行性点到话题"potential"
        publishPotential(potential_array_);

    //判断calculatePotentials()函数是否成功找到可行点矩阵potential_array_
    if (found_legal) {
        //extract the plan
        //makeplan主要步骤2:
        //提取plan,调用path_maker_-getPath()实例,从所有可行性点中获取路径
        //通过代价地图path_maker_->getPath得到路径plan,调用的是Traceback::virtual bool getPath();
        if (getPlanFromPotential(start_x, start_y, goal_x, goal_y, goal, plan)) {
            //make sure the goal we push on has the same timestamp as the rest of the plan
            //确保目标点与计划的其余部分具有相同的时间戳
            geometry_msgs::PoseStamped goal_copy = goal;
            goal_copy.header.stamp = ros::Time::now();
            plan.push_back(goal_copy);
            this->optimizationPath(plan,M_PI/10); //此处调用路径平滑函数
        } else {
            ROS_ERROR("Failed to get a plan from potential when a legal potential was found. This shouldn't happen.");
        }
    }else{
        ROS_ERROR("Failed to get a plan.");
    }

    // add orientations if needed
    //给路径加方向
    orientation_filter_->processPath(start, plan);

    // filtePath(plan, 0.2);
    //publish the plan for visualization purposes
    //发布可视化路径
    publishPlan(plan);
    delete potential_array_;
    //如果可以规划出非空路径则返回
    return !plan.empty();
}

void GlobalPlanner::filtePath(std::vector<geometry_msgs::PoseStamped> &plan,double safe_distance)
  {
    if(plan.empty())
    {
      ROS_INFO("PurePlannerROS::filtePath: plan is empty.");
      return;
    }
    int safe_cell = (int)( safe_distance / this->costmap_->getResolution() );
    if(safe_cell < 1)
    {
      ROS_INFO("The safety distance is too small.");
      return;
    }
    size_t point_size = plan.size();
    geometry_msgs::PoseStamped tem_point;
    geometry_msgs::PoseStamped before_point;
    geometry_msgs::PoseStamped next_point;
    geometry_msgs::PoseStamped nearest_obstacle;
    unsigned int mx_min,mx_max,my_min,my_max,mx,my;
    for(size_t i=0;i<point_size;i++)
    {
      tem_point = plan[i];
      before_point = i>0?plan[i-1]:plan[i];
      next_point   = i<point_size-1?plan[i+1]:plan[i];

      this->costmap_->worldToMap(tem_point.pose.position.x,tem_point.pose.position.y,mx,my);
      mx_min = mx>safe_cell?mx-safe_cell:mx;
      mx_max = mx+safe_cell<this->costmap_->getSizeInCellsX()?mx+safe_cell:mx;
      my_min = my>safe_cell?my-safe_cell:my;
      my_max = my+safe_cell<this->costmap_->getSizeInCellsY()?my+safe_cell:my;
      std::vector<geometry_msgs::Point> obstacle_vec;
      geometry_msgs::Point obstacle;
      obstacle_vec.clear();
      for(unsigned int j=mx_min;j<mx_max;j++) //Find all obstacles within a safe distance.
      {
        for(unsigned int k=my_min;k<my_max;k++)
        {
          if(this->costmap_->getCost(j,k) != costmap_2d::FREE_SPACE)
          {
            this->costmap_->mapToWorld(j,k,obstacle.x,obstacle.y);
            obstacle_vec.push_back(obstacle);
          }
        }
      }

      if(obstacle_vec.empty() != true)
      {
         //Check if the points are on the same side.
        bool same_side_flag = false;
        if(next_point.pose.position.x != before_point.pose.position.x)
        {
          double lk = 0,lb = 0,ly = 0,num = 0;
          lk = (next_point.pose.position.y-before_point.pose.position.y) / (next_point.pose.position.x-before_point.pose.position.x);
          lb = next_point.pose.position.y - lk * next_point.pose.position.x;

          for(size_t m=0;m<obstacle_vec.size();m++)
          {
            ly = lk * obstacle_vec[m].x + lb;
            if(ly != 0)
              break;
          }

          for(size_t m=0;m<obstacle_vec.size();m++)
          {
            num = ly*(lk * obstacle_vec[m].x + lb);
            if(num < 0)
            {
              same_side_flag = true;
              break;
            }
          }
        }
        else
        {
          double const_x = next_point.pose.position.x;
          double err = 0,num = 0;
          for(size_t m=0;m<obstacle_vec.size();m++)
          {
            err = const_x - obstacle_vec[m].x;
            if(err != 0)
              break;
          }
          for(size_t m=0;m<obstacle_vec.size();m++)
          {
            num = err*(const_x - obstacle_vec[m].x);
            if(num < 0)
            {
              same_side_flag = true;
              break;
            }
          }
        }

        if(same_side_flag == true)
        {
          ROS_INFO("These points are not on the same side.");
          continue;
        }

        double distance=0,min_distance_obst = 1000.0;
        size_t min_obst_index = 0;
        double diff_x,diff_y;
        for(size_t l=0;l<obstacle_vec.size();l++) //find nearest obstacle
        {
          diff_x = obstacle_vec[l].x - tem_point.pose.position.x;
          diff_y = obstacle_vec[l].y - tem_point.pose.position.y;
          distance = sqrt(diff_x*diff_x+diff_y*diff_y);
          if(min_distance_obst > distance)
          {
            min_distance_obst = distance;
            min_obst_index = l;
          }
        }

        if(safe_distance - min_distance_obst < 0.0)
        {
          continue;
        }

        nearest_obstacle.pose.position.x = obstacle_vec[min_obst_index].x;
        nearest_obstacle.pose.position.y = obstacle_vec[min_obst_index].y;

        distance =  safe_distance - min_distance_obst;
        double err_x,err_y,theta,finally_x,finally_y;
        theta = atan2(tem_point.pose.position.y-nearest_obstacle.pose.position.y,tem_point.pose.position.x-nearest_obstacle.pose.position.x);
        err_x = distance*cos(theta);
        err_y = distance*sin(theta);
        finally_x = tem_point.pose.position.x + err_x;
        finally_y = tem_point.pose.position.y + err_y;
        this->costmap_->worldToMap(finally_x,finally_y,mx,my);
        if(this->costmap_->getCost(mx,my) == costmap_2d::FREE_SPACE)
        {
          plan[i].pose.position.x = finally_x;
          plan[i].pose.position.y = finally_y;
        }
      }
    }
  }


//发布可视化路径
void GlobalPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path) {
    if (!initialized_) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }

    //create a message for the plan
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());

    gui_path.header.frame_id = frame_id_;
    gui_path.header.stamp = ros::Time::now();

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for (unsigned int i = 0; i < path.size(); i++) {
        gui_path.poses[i] = path[i];
    }

    plan_pub_.publish(gui_path);
}

//从potential中提取路径plan ,调用path_maker_->getPath()实例:
bool GlobalPlanner::getPlanFromPotential(double start_x, double start_y, double goal_x, double goal_y,
                                      const geometry_msgs::PoseStamped& goal,
                                       std::vector<geometry_msgs::PoseStamped>& plan) {
    if (!initialized_) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return false;
    }

    std::string global_frame = frame_id_;

    //clear the plan, just in case
    plan.clear();

    std::vector<std::pair<float, float> > path;
    //调用path_maker_->getPath()实例提取路径
    if (!path_maker_->getPath(potential_array_, start_x, start_y, goal_x, goal_y, path)) {
        ROS_ERROR("NO PATH!");
        return false;
    }

    ros::Time plan_time = ros::Time::now();
////////////////////////
    int path_size_num = path.size() -1;
////////////////////////
    for (int i = path.size() -1; i>=0; i--) {
        //
        std::pair<float, float> point = path[i];
        //convert the plan to world coordinates
        double world_x, world_y;
        //将提取的路径从map坐标系转换为世界坐标系
        mapToWorld(point.first, point.second, world_x, world_y);

        geometry_msgs::PoseStamped pose;
        pose.header.stamp = plan_time;
        pose.header.frame_id = global_frame;
        pose.pose.position.x = world_x;
        pose.pose.position.y = world_y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;
        plan.push_back(pose);
//////////////
        if(i != path_size_num)
        {
          double cx,cy,px,py;
          cx = pose.pose.position.x;
          cy = pose.pose.position.y;
          px = plan.back().pose.position.x;
          py = plan.back().pose.position.y;
          if( sqrt( (cx-px)*(cx-px) + (cy-py)*(cy-py) ) > 0.05)
          {
            geometry_msgs::PoseStamped pose_insert = pose;
            pose_insert.pose.position.x = (cx+px)/2;
            pose_insert.pose.position.y = (cy+py)/2;
            plan.push_back(pose_insert);
          }
        }
////////////////
        plan.push_back(pose);
    }
    if(old_navfn_behavior_){
            plan.push_back(goal);
    }
    return !plan.empty();
}

//发布可行性点
void GlobalPlanner::publishPotential(float* potential)
{
    int nx = costmap_->getSizeInCellsX(), ny = costmap_->getSizeInCellsY();
    double resolution = costmap_->getResolution();
    nav_msgs::OccupancyGrid grid;
    // Publish Whole Grid
    grid.header.frame_id = frame_id_;
    grid.header.stamp = ros::Time::now();
    grid.info.resolution = resolution;

    grid.info.width = nx;
    grid.info.height = ny;

    double wx, wy;
    costmap_->mapToWorld(0, 0, wx, wy);
    grid.info.origin.position.x = wx - resolution / 2;
    grid.info.origin.position.y = wy - resolution / 2;
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.w = 1.0;

    grid.data.resize(nx * ny);

    float max = 0.0;
    for (unsigned int i = 0; i < grid.data.size(); i++) {
        float potential = potential_array_[i];
        if (potential < POT_HIGH) {
            if (potential > max) {
                max = potential;
            }
        }
    }

    for (unsigned int i = 0; i < grid.data.size(); i++) {
        if (potential_array_[i] >= POT_HIGH) {
            grid.data[i] = -1;
        } else
            grid.data[i] = potential_array_[i] * publish_scale_ / max;
    }
    potential_pub_.publish(grid);
}
////////////////////////////////////////////////
int GlobalPlanner::optimizationPath(std::vector<geometry_msgs::PoseStamped>& plan,double movement_angle_range)
{
  if(plan.empty())
    return 0;
  size_t pose_size = plan.size() - 1;
  double px,py,cx,cy,nx,ny,a_p,a_n;
  bool is_run = false;
  int ci = 0;
  for(ci=0;ci<1000;ci++)
  {
    is_run = false;
    for(size_t i=1;i<pose_size;i++)
    {
      px = plan[i-1].pose.position.x;
      py = plan[i-1].pose.position.y;

      cx = plan[i].pose.position.x;
      cy = plan[i].pose.position.y;

      nx = plan[i+1].pose.position.x;
      ny = plan[i+1].pose.position.y;

      a_p = normalizeAngle(atan2(cy-py,cx-px),0,2*M_PI);
      a_n = normalizeAngle(atan2(ny-cy,nx-cx),0,2*M_PI);

      if(std::max(a_p,a_n)-std::min(a_p,a_n) > movement_angle_range)
      {
        plan[i].pose.position.x = (px + nx)/2;
        plan[i].pose.position.y = (py + ny)/2;
        is_run = true;
      }
    }
    if(!is_run)
      return ci;
  }
  return ci;
}


} //end namespace lnglobal_planner
