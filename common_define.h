#pragma once
#include <vector>
#include <string>

typedef int MErrorCode;

typedef std::string MRoadId;
typedef int MSectionIndex;
typedef int MLaneLocalId;
typedef std::string MJunctionId;
typedef std::string MAnchorId;	// 锚点Id
typedef double MRoadS;	// 道路S坐标
typedef double MRouteS;	// 路径S坐标

/* 车道的全局Id */
typedef struct MLaneUId {
	MRoadId road_id;
	MSectionIndex section_index;
	MLaneLocalId local_id;	// 在道路参考线左侧为正，右侧为负
}MLaneUId;

/* 笛卡尔空间XYZ坐标系下表示的位置 */
typedef struct MXYZ {
	double x;
	double y;
	double z; 
}MXYZ;

/* 基于道路参考线的Frenet坐标系下表示的位置 */
typedef struct MSLZ {
	MLaneUId lane_uid;
	MRoadS s;
	double l;
	double z;
}MSLZ;

/* 锚点结构体 */
typedef struct MAnchor
{
	MAnchorId anchor_id; // 用户自定义锚点ID
	MSLZ slz; // 锚点SLZ坐标
}MAnchor;

/* 车道基本信息结构体 */
typedef struct MLaneInfo {
	MLaneUId lane_uid;
	MRoadS begin;
	MRoadS end;
	MRoadS length;
}MLaneInfo;

/* 车道ID数组 */
typedef struct MLaneUIdArray
{
	uint32_t length; // 数组长度
	std::vector<MLaneUId> array;
}MLaneUIdArray;

/* MXYZ数组 */
typedef struct MXYZArray
{
	uint32_t length;
	std::vector<MXYZ> array;
}MXYZArray;

/* MSLZ数组 */
typedef struct MSLZArray
{
	uint32_t length;
	std::vector<MSLZ> array;
}MSLZArray;

/* MAnchor数组 */
typedef struct MAnchorArray
{
	uint32_t length; // 数组长度
	std::vector<MAnchor> array; 
}MAnchorArray;

/* 路径信息，用于存储路径规划算法返回的路径信息 */
typedef struct MRoute
{
	MLaneUIdArray lane_uids;
	MSLZ begin;
	MSLZ end;
	MRouteS distance;
}MRoute;

/* 车道允许的变道类型 */
typedef enum MLaneChangeType {
	NoChange = 0,
	LeftChange = 1,
	RightChange = 2,
	BothChange = 4,
	UnknownChangeType = -1
}MLaneChangeType;

const MLaneUId EmptyLaneUId = { "", INT_MAX, 0 };
const MSLZ EmptySLZ = { EmptyLaneUId, DBL_MIN, DBL_MAX };
const MAnchor EmptyAnchor = { "", EmptySLZ};
const MSLZArray EmptySLZArray = { 0, {} };
const MXYZArray EmptyXYZArray = { 0, {} };
const MAnchorArray EmptyAnchorArray = { 0, {} };

// ******************************************* //
// ******************************************* //
// ******************************************* //
typedef std::string Type;

/* 地理参考信息 */
struct geoReference
{

};

/* 地图头部信息 */
struct Header {
	std::string revMajor;
	std::string revMinor;
	std::string name;
	std::string version;
	std::string date;
	double north;
	double south;
	double east;
	double west;
	std::string vendor;
	geoReference geo;
};

/* 单个坐标点信息 */
struct Point {
	double x = 0;
	double y = 0;
	double hdg = 0;
	double s = 0;
	double width = 0;
	double l = 0;
};

/********************************************/
/* 三次多项式 */
struct Poly3 {
	double a;
	double b;
	double c;
	double d;
};

/* 车道宽度信息 */
struct LaneWidth {
	double sOffset;
	Poly3 poly3;	// 三次多项式 局部坐标系下
};

/* 车道信息 */
struct Lane {
	std::string road_id = "NONE";
	std::string road_section = "NONE";
	std::string id = "NONE";
	std::string predecessor_id = "NONE";	// 车道连接—前驱
	std::string successor_id = "NONE";	// 车道连接—后继
	std::string type;	// 车道类型
	std::string lane_change;	// 变道类型
	std::vector<LaneWidth> width;	// 车道宽度
	std::vector<Point> center_line_points;
	double length = 0;
};
/********************************************/

/********************************************/
/* 道路段信息 */
struct RoadSection {
	MRoadS s;		// 道路段的起始S值
	std::string id;
	std::vector<Lane> left_lanes;	// 道路中心线左侧车道集合
	std::vector<Lane> right_lanes;	// 道路中心线右侧车道集合
};

/* 道路类型和限速 */
struct RoadType {
	MRoadS s;
	Type type;		// 道路类型
	double speed;	// 道路限速
	std::string speed_unit;	// 道路限速单位
};

/* 参数形式的三次多项式 */
struct ParamPoly3 {
	double aU = 0;
	double bU = 0;
	double cU = 0;
	double dU = 0;
	double aV = 0;
	double bV = 0;
	double cV = 0;
	double dV = 0;
};

/* 道路中心线的几何形状信息 */
struct Geometry {
	MRoadS s;
	double x;
	double y;
	double hdg;
	double length;
	Type type;	// 几何形状类型
	ParamPoly3 param_poly3;	// 参数三次多项式 局部坐标系下
};

/* 道路前驱和道路后继 用于道路连接 */
typedef struct {
	std::string element_type;	// 道路类型 Junction or Road
	std::string element_id;
	std::string contact_point;
}Predecessor, Successor;

/* 道路连接 */
struct RoadLink {
	Predecessor predecessor = { "NONE", "NONE", "NONE" };	// 道路前驱
	Successor successor = { "NONE", "NONE", "NONE" };	// 道路后继
};

/* 道路信息 */
struct Road {
	std::string id;
	std::string junction_id;
	double length;
	RoadLink link;		// 道路连接
	std::vector<Geometry> reference_line;	// 道路中心线的几何形状
	std::vector<Point> reference_line_points;
	std::vector<RoadType> type;	// 道路类型

	std::vector<RoadSection> road_sections;
};
/********************************************/

/* 用于交叉口内的车道连接 */
struct LaneLink {
	std::string from;
	std::string to;
};

/* 交叉口内的连接 */
struct Connection {
	std::string id;
	std::string incomingRoad;
	std::string connectingRoad;
	std::string contactPoint;
	LaneLink lane_link;	// 与交叉口有关的车道连接
};

/* 交叉口信息 */
struct Junction {
	std::string id;
	std::vector<Connection> connection;	// 道路连接
};