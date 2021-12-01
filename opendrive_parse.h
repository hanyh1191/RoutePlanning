#pragma once
#include<string>
#include<vector>
#include<unordered_map>
#include<functional>
#include "common_define.h"
#include "visualize_map.h"
#include "lane_node.h"

class OpenDriveParse
{
public:

	/* 加载地图文件 */
	bool LoadMap(const std::string& file_path);

	/* XYZ 转 SLZ 坐标，优先搜索hint road附近的道路 */
	MErrorCode MapFindSLZ(MXYZ xyz, MLaneUId hint, MSLZ* slz);

	/* XYZ 转 SLZ 坐标，直接全局搜索 */
	MErrorCode MapFindSLZWithOutHInt(MXYZ xyz, MSLZ* slz);

	/* SLZ 转 XYZ 坐标 */
	MErrorCode MapCalcXYZ(MSLZ slz, MXYZ* xyz);

	/* 创建一个锚点 */
	MAnchor CreateAnchor(MAnchorId id, MSLZ pos);

	/* 创建一个车道全局ID */
	MLaneUId CreateLaneUId(const MRoadId road_id, MSectionIndex section_index, MLaneLocalId lane_id);

	/* 创建一个MSLZ */
	MSLZ CreateSLZ(MLaneUId lane_uid, double s, double l);

	/* 规划路径函数 */
	MErrorCode MapPlanRoute(MAnchorArray anchor_list, MRoute* route);

	/* 查询车道的基本信息 */
	MErrorCode MapQueryLaneInfo(MLaneUId lane_uid, MLaneInfo* lane_info);

	/* 查询车道上某一个位置的最大限速 km/h */
	MErrorCode MapQueryLaneSpeedAt(MLaneUId lane_uid, MRoadS s, double* speed_limit);

	/* 查询车道上某一个位置的变道类型 */
	MErrorCode MapQueryLaneChangeTypeAt(MLaneUId lane_uid, MRoadS s, MLaneChangeType* lane_change_type);

	/* 计算得到车道左右边界线 */
	MErrorCode MapQueryLaneBoundaries(MLaneUId lane_uid, double sampling_spacing, MSLZArray* left_boundary, MSLZArray* right_boundary);

	/* 计算得到车道中心线采样点 */
	MErrorCode MapCalcLaneCenterLine(MLaneUId lane_uid, double sampling_spacing, MSLZArray* result);

	/* 计算得到车道中心线的某一段（s1~s2）中心线采样点集 */
	MErrorCode MapCalcLaneCenterLineInterval(MLaneUId lane_uid, double s1, double s2, double sampling_spacing, MSLZArray* result);

	/* 查看道路是否属于一个Junction的内部道路 */
	MErrorCode MapQueryRoadJunctionId(const std::string road_id, std::string* junction_id);

	/* 确认指定点是否在某一道路边界范围内 */
	bool IsPointOnRoad(MXYZ& xyz, Road& road, int& idx, double& s, double& l);


	/*************************************/
	/* 计算离散的道路中心线坐标点集和离散的车道中心线坐标点集 */
	void ParseToDiscretePoints();

	/* 可视化道路中心线和车道中心线 */
	void VisualizePoints();


	double CalcLaneWidth(const double& s, const double& s_section, const std::vector<LaneWidth>& width);

	void CreateLaneNodes();

	bool CheckFrenetSL(double& s, double& l, int& idx, Road& road, int& section_idx, std::string& lane_id);

	std::string ContactLaneUId(const MLaneUId& lane_uid);

	double f_value(double g, MLaneUId cur_laneuid, MLaneUId target_laneuid, MXYZ target_xyz);

private:
	std::vector<Road> roads;	// roads
	std::vector<Junction> junctions;	// junctions

	std::vector<LaneNode*> nodes;	// lane nodes

	std::vector<MLaneUId> lane_nodes;
	std::vector<std::vector<int>> lane_nodes_index;
	std::vector<std::vector<MLaneUId>> pre_nodes;
	std::vector<std::vector<MLaneUId>> suc_nodes;
	std::vector<std::vector<MLaneUId>> parall_nodes;

	VisualizeMap visiualize;		// visiualize

};

