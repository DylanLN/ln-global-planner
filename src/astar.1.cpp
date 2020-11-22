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
#include<lnglobal_planner/astar.h>
#include<costmap_2d/cost_values.h>

namespace lnglobal_planner {

AStarExpansion::AStarExpansion(PotentialCalculator* p_calc, int xs, int ys) :
        Expander(p_calc, xs, ys) {
}

//queue_即为openlist
//costs为地图指针，potential为代价数组，cycles为循环次数，代码里值为2*nx*ny为地图栅格数的两倍
bool AStarExpansion::calculatePotentials(unsigned char* costs, double start_x, double start_y, double end_x, double end_y,
                                        int cycles, float* potential) {
    //queue 为启发式搜索到的向量队列。：<i , cost>
    queue_.clear();
    //将坐标转换成地图栅格信号
    int start_i = toIndex(start_x, start_y);
    //将起始点放入队列
    //Index自定义的一个类，第一个元素代表栅格地图的序号，第二个代表f，cost
    /****************************************************************
        astar 第一步,将出发点加入openlist
    ****************************************************************/
    queue_.push_back(Index(start_i, 0));

    //所有位置的poteneial都设置为最大值1e10
    std::fill(potential, potential + ns_, POT_HIGH);
    //起始点设为0,poteneial就是估计值g，f=g+h
    potential[start_i] = 0;

    //获取终点对应的序号
    int goal_i = toIndex(end_x, end_y);
    int cycle = 0;

    //进入循环,继续循环条件为队列不为空且循环次数小于格子总数的2倍
    while (queue_.size() > 0 && cycle < cycles) {
        /****************************************************************
            astar 第二步,取出openlist中代价最小点
        ****************************************************************/
        Index top = queue_[0];
        //把首元素放到最后，其他元素按照cost值从小到大排列（很重要）
        std::pop_heap(queue_.begin(), queue_.end(), greater1());
        //因为每次拓展都是从这个元素的首元素pop，h选不好的话，会多拓展节点。
        //删除最小代价点
        queue_.pop_back();

        int i = top.i;
        //计算到goal就结束了，可能会有很多点没有计算，所以速度快（到达目标点）
        if (i == goal_i)
            return true;

        /****************************************************************
            astar 第三步,检查最小点的前后左右邻接点
        ****************************************************************/
        //将代价最小点i周围点加入搜索队列幷更新代价值。
        add(costs, potential, potential[i], i + 1, end_x, end_y);
        add(costs, potential, potential[i], i - 1, end_x, end_y);
        add(costs, potential, potential[i], i + nx_, end_x, end_y);
        add(costs, potential, potential[i], i - nx_, end_x, end_y);

        cycle++;
    }
    return false;
}

//添加点幷更新代价函数,如果是已经添加的的点则忽略，根据costmap的值如果是障碍物的点也忽略。
void AStarExpansion::add(unsigned char* costs, float* potential, float prev_potential, int next_i, int end_x,
                         int end_y) {
    //超出范围，ns_为栅格总数
    if (next_i < 0 || next_i >= ns_)
        return;

    //没有搜索过的元素f都是POT_HIGH，不是则为close表中的元素，不再搜索。
    /****************************************************************
        astar 检查节点是否在closedlist中
    ****************************************************************/
    if (potential[next_i] < POT_HIGH)
        return;

    //如果改点已经大于我们设置的障碍成本,则退出
    if(costs[next_i]>=lethal_cost_ && !(unknown_ && costs[next_i]==costmap_2d::NO_INFORMATION))
        return;

    potential[next_i] = p_calc_->calculatePotential(potential, costs[next_i] + neutral_cost_, next_i, prev_potential);

    /*************************************************
    potential[]存储所有点的g(n)是在状态空间中从初始节点到n节点的实际代价。
    costs[next_i] + neutral_cost_：
        costs[next_i]，next_i这几个点在costmap上的cost值
        neutral_cost_，设定的一个值默认50
    prev_potential，当前点的f

    calculatePotential根据use_quadratic的不同有两个选择
    默认true，QuadraticCalculator 二次曲线计算
    false 简单的计算，virtual float calculatePotential：return prev_potential+cost
    *************************************************/
    int x = next_i % nx_, y = next_i / nx_;
    //选取很重要，启发式函数，代价h(n)是从n节点到目标节点最佳路径的估计代价，此处用的是曼哈顿距离
    float distance = abs(end_x - x) + abs(end_y - y);

    //f= potential[next_i] + distance * neutral_cost_
    queue_.push_back(Index(next_i, potential[next_i] + distance * neutral_cost_));
    //对刚插入的(尾部)元素做堆排序，把最小代价点放到front队头queue_[0]
    std::push_heap(queue_.begin(), queue_.end(), greater1());
}

} //end namespace lnglobal_planner
