#include "lane_node.h"

LaneNode::LaneNode(MLaneUId lane_uid_, MXYZ xyz_, double length_, double speed_)
{
	lane_uid = lane_uid_;
	xyz = xyz_;
	length = length_;
	speed = speed_;

	g_cost = h_cost = f_cost = 0;
	father = nullptr;
}

void LaneNode::UpdateCost(MXYZ target_xyz, bool is_neighbor)
{
	if (father == nullptr)
		g_cost = 0;
	// 邻居车道
	else if (is_neighbor == true)
	{
		g_cost = father->g_cost + father->g_cost * 0.2;
	}
	else
	{
		// 车道长度
		g_cost = length + father->g_cost;
	}
	// 车道终点到目标点的欧氏距离
	h_cost = (target_xyz.x - xyz.x) * (target_xyz.x - xyz.x) + (target_xyz.y - xyz.y) * (target_xyz.y - xyz.y);

	f_cost = g_cost + h_cost;
}

bool LaneNode::IsSameLane(MLaneUId lane_uid_)
{
	if (lane_uid.road_id == lane_uid_.road_id && lane_uid.section_index == lane_uid_.section_index && lane_uid.local_id == lane_uid_.local_id)
		return true;
	else
		return false;
}
