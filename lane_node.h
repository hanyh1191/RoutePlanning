#pragma once
#include "common_define.h"
class LaneNode
{
public:

	LaneNode(MLaneUId lane_uid_, MXYZ xyz_, double length_, double speed_);

	void UpdateCost(MXYZ target_xyz, bool is_neighbor);

	bool IsSameLane(MLaneUId lane_uid_);

	MLaneUId lane_uid;
	MXYZ xyz;
	double length;
	double speed;
	std::vector<MLaneUId> to_laneuid;
	std::vector<MLaneUId> from_laneuid;

	std::vector<LaneNode*> to_lanenode;
	std::vector<LaneNode*> from_lanenode;

	double g_cost, h_cost, f_cost;

	LaneNode* father;
};

