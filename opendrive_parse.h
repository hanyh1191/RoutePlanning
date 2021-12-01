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

	/* ���ص�ͼ�ļ� */
	bool LoadMap(const std::string& file_path);

	/* XYZ ת SLZ ���꣬��������hint road�����ĵ�· */
	MErrorCode MapFindSLZ(MXYZ xyz, MLaneUId hint, MSLZ* slz);

	/* XYZ ת SLZ ���ֱ꣬��ȫ������ */
	MErrorCode MapFindSLZWithOutHInt(MXYZ xyz, MSLZ* slz);

	/* SLZ ת XYZ ���� */
	MErrorCode MapCalcXYZ(MSLZ slz, MXYZ* xyz);

	/* ����һ��ê�� */
	MAnchor CreateAnchor(MAnchorId id, MSLZ pos);

	/* ����һ������ȫ��ID */
	MLaneUId CreateLaneUId(const MRoadId road_id, MSectionIndex section_index, MLaneLocalId lane_id);

	/* ����һ��MSLZ */
	MSLZ CreateSLZ(MLaneUId lane_uid, double s, double l);

	/* �滮·������ */
	MErrorCode MapPlanRoute(MAnchorArray anchor_list, MRoute* route);

	/* ��ѯ�����Ļ�����Ϣ */
	MErrorCode MapQueryLaneInfo(MLaneUId lane_uid, MLaneInfo* lane_info);

	/* ��ѯ������ĳһ��λ�õ�������� km/h */
	MErrorCode MapQueryLaneSpeedAt(MLaneUId lane_uid, MRoadS s, double* speed_limit);

	/* ��ѯ������ĳһ��λ�õı������ */
	MErrorCode MapQueryLaneChangeTypeAt(MLaneUId lane_uid, MRoadS s, MLaneChangeType* lane_change_type);

	/* ����õ��������ұ߽��� */
	MErrorCode MapQueryLaneBoundaries(MLaneUId lane_uid, double sampling_spacing, MSLZArray* left_boundary, MSLZArray* right_boundary);

	/* ����õ����������߲����� */
	MErrorCode MapCalcLaneCenterLine(MLaneUId lane_uid, double sampling_spacing, MSLZArray* result);

	/* ����õ����������ߵ�ĳһ�Σ�s1~s2�������߲����㼯 */
	MErrorCode MapCalcLaneCenterLineInterval(MLaneUId lane_uid, double s1, double s2, double sampling_spacing, MSLZArray* result);

	/* �鿴��·�Ƿ�����һ��Junction���ڲ���· */
	MErrorCode MapQueryRoadJunctionId(const std::string road_id, std::string* junction_id);

	/* ȷ��ָ�����Ƿ���ĳһ��·�߽緶Χ�� */
	bool IsPointOnRoad(MXYZ& xyz, Road& road, int& idx, double& s, double& l);


	/*************************************/
	/* ������ɢ�ĵ�·����������㼯����ɢ�ĳ�������������㼯 */
	void ParseToDiscretePoints();

	/* ���ӻ���·�����ߺͳ��������� */
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

