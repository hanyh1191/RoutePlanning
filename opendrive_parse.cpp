#include "opendrive_parse.h"
#include "roads_parse.h"
#include "junctions_parse.h"
#include "util.h"
#include <iomanip>
#include <iostream>
#include <fstream>
#include <streambuf>


bool OpenDriveParse::LoadMap(const std::string& file_path)
{
    tinyxml2::XMLDocument document;
    if (document.LoadFile(file_path.c_str()) != tinyxml2::XML_SUCCESS) {
        ERROR("load failed");
        return false;
    }
    // root node
    const tinyxml2::XMLElement* root_node = document.RootElement();
    auto header_node = root_node->FirstChildElement("header");
    if (!header_node) {
        ERROR("xml data missing header");
        return false;
    }
    else
    {
        // Header
        std::string map_name;
        QueryStringAttribute(*header_node, "name", &map_name);
        std::cout << ">>>> map file: " << file_path 
            << " map name: " << map_name << std::endl;

        // Parse Roads and Lanes
        bool state = RoadsParse::Parse(*root_node, &roads);
        if (!state)
        {
            ERROR("Road Parse Error");
            return false;
        }

        // Parse Junctions
        state = JunctionsParse::Parse(*root_node, &junctions);
        if (!state)
        {
            ERROR("Junction Parse Error");
            return false;
        }
    }
    return true;
}

MErrorCode OpenDriveParse::MapFindSLZ(MXYZ xyz, MLaneUId hint, MSLZ* slz)
{
    bool get_slz = false;
    Road road = roads[std::stoi(hint.road_id)];

    double min_dot_product = double(INT_MAX);
    int idx = -1;

    double s, l;
    for (int j = 0; j < road.reference_line_points.size(); j++)
    {
        Point point = road.reference_line_points[j];
        double dot_product = std::abs((xyz.x - point.x) * (std::cos(point.hdg)) + (xyz.y - point.y) * (std::sin(point.hdg)));
        if (dot_product < min_dot_product)
        {
            min_dot_product = dot_product;
            idx = j;
        }
    }
    Point rpoint = road.reference_line_points[idx];
    CalculateFrenetPoint(rpoint.s, rpoint.x, rpoint.y, rpoint.hdg, xyz.x, xyz.y, &s, &l);

    // CheckSL
    int section_idx;
    std::string lane_id;
    if (CheckFrenetSL(s, l, idx, road, section_idx, lane_id) == true)
    {
        // Get SLZ
        slz->l = l;
        slz->s = s;
        slz->z = 0;
        slz->lane_uid.road_id = hint.road_id;
        slz->lane_uid.section_index = section_idx;
        slz->lane_uid.local_id = stoi(lane_id);
        get_slz = true;
    }
    if (get_slz)
        return 0;
    else
        return MapFindSLZWithOutHInt(xyz, slz);
}

MErrorCode OpenDriveParse::MapFindSLZWithOutHInt(MXYZ xyz, MSLZ* slz)
{
    bool get_slz = false;
    for (int i = 0; i < roads.size(); i++)
    {
        Road road = roads[i];
        double min_dot_product = double(INT_MAX);
        int idx = -1;

        double s, l;
        for (int j = 0; j < road.reference_line_points.size(); j++)
        {
            Point point = road.reference_line_points[j];
            double dot_product = std::abs((xyz.x - point.x) * (std::cos(point.hdg)) + (xyz.y - point.y) * (std::sin(point.hdg)));
            if (dot_product < min_dot_product)
            {
                min_dot_product = dot_product;
                idx = j;
            }
        }
        Point rpoint = road.reference_line_points[idx];
        CalculateFrenetPoint(rpoint.s, rpoint.x, rpoint.y, rpoint.hdg, xyz.x, xyz.y, &s, &l);

        // CheckSL
        int section_idx;
        std::string lane_id;
        if (CheckFrenetSL(s, l, idx, roads[i], section_idx, lane_id) == true)
        {
            // Get SLZ
            slz->l = l;
            slz->s = s;
            slz->z = 0;
            slz->lane_uid.road_id = std::to_string(i);
            slz->lane_uid.section_index = section_idx;
            slz->lane_uid.local_id = stoi(lane_id);
            get_slz = true;
            break;
        }
    }
    if (get_slz)
        return 0;
    else
        return -1;
}

MErrorCode OpenDriveParse::MapCalcXYZ(MSLZ slz, MXYZ* xyz)
{
    Road road = roads[std::stoi(slz.lane_uid.road_id)];

    double start_s = road.road_sections[slz.lane_uid.section_index].s;
    int idx = -1;

    if (slz.s >= road.reference_line_points[road.reference_line_points.size() - 1].s)
        idx = road.reference_line_points.size() - 1;
    else
        for (int i = 0; i < road.reference_line_points.size() - 1; i++)
        {
            Point point = road.reference_line_points[i];
            //std::cout << slz.s << " " << point.s << " " << road.reference_line_points[i + 1].s << std::endl;

            if (point.s < start_s)
                continue;
            if (slz.s >= point.s && slz.s < road.reference_line_points[i + 1].s)
            {
                idx = i;
                break;
            }
        }
    //std::cout << "Idx: " << idx << std::endl;
    Point rpoint = road.reference_line_points[idx];
    Point point = CalculateCartesianPoint(rpoint, slz.l);

    xyz->x = point.x;
    xyz->y = point.y;
    xyz->z = 0;
    //std::cout << xyz->x << " " << xyz->y << std::endl;
    return 0;
}

MAnchor OpenDriveParse::CreateAnchor(MAnchorId id, MSLZ pos)
{
    return MAnchor();
}

MLaneUId OpenDriveParse::CreateLaneUId(const MRoadId road_id, MSectionIndex section_index, MLaneLocalId lane_id)
{
    MLaneUId lane_uid = { road_id, section_index, lane_id };

    return lane_uid;
}

MSLZ OpenDriveParse::CreateSLZ(MLaneUId lane_uid, double s, double l)
{
    MSLZ slz;
    slz.lane_uid = lane_uid;
    slz.s = s;
    slz.l = l;
    slz.z = 0;

    return slz;
}

MErrorCode OpenDriveParse::MapPlanRoute(MAnchorArray anchor_list, MRoute* route)
{
    std::cout << "Begin Plan Route" << std::endl;
    if (anchor_list.length == 0)
        return -1;

    MSLZ begin_slz = anchor_list.array[0].slz;
    MSLZ target_slz = anchor_list.array[1].slz;
    MXYZ target_xyz;
    MapCalcXYZ(target_slz, &target_xyz);

    MLaneUId begin_laneuid = begin_slz.lane_uid;
    MLaneUId target_laneuid = target_slz.lane_uid;

    std::vector<MLaneUId> route_lane_uids;  // 保存最后寻路的车道序列
    std::vector<int> route_lane_indexs;

    std::cout << "Search Start Lane: " << begin_laneuid.road_id << " " << begin_laneuid.section_index << " "
        << begin_laneuid.local_id << std::endl;
    std::cout << "Search End Lane: " << target_laneuid.road_id << " " << target_laneuid.section_index << " "
        << target_laneuid.local_id << std::endl;
    
    bool find_road = false;

    // 起点和终点在同一条道路上
    if (begin_laneuid.road_id == target_laneuid.road_id && begin_laneuid.section_index == target_laneuid.section_index
        && begin_laneuid.local_id == target_laneuid.local_id) {
        find_road = true;
        route_lane_uids.push_back(begin_laneuid);

        for (int i = 0; i < lane_nodes.size(); i++)
        {
            if (lane_nodes[i].road_id == begin_laneuid.road_id &&
                lane_nodes[i].section_index == begin_laneuid.section_index &&
                lane_nodes[i].local_id == begin_laneuid.local_id)
            {
                route_lane_indexs.push_back(i);
                break;
            }
        }
    }
    // 不在同一条道路上 Astar算法
    else 
    {
        struct pair {
            MLaneUId lane_uid;
            int index;
            double f;
        };
        struct node {
            MLaneUId lane_uid;
            int index = -1;
            std::string type;
        };
        struct cmp {
            bool operator()(pair a, pair b) {
                return  a.f > b.f;  //小顶堆
            }
        };
        std::priority_queue< pair, std::vector<pair>, cmp > OPEN;
        std::vector<pair> CLOSE;

        std::map<int, int> PARENT; // 索引
        std::map<int, double> g;

        int begin_index, target_index;
        for (int i = 0; i < lane_nodes.size(); i++)
        {
            if (lane_nodes[i].road_id == begin_laneuid.road_id &&
                lane_nodes[i].section_index == begin_laneuid.section_index &&
                lane_nodes[i].local_id == begin_laneuid.local_id)
            {
                begin_index = i;
            }
            if (lane_nodes[i].road_id == target_laneuid.road_id &&
                lane_nodes[i].section_index == target_laneuid.section_index &&
                lane_nodes[i].local_id == target_laneuid.local_id)
            {
                target_index = i;
            }
        }
        PARENT[begin_index] = begin_index;
        g[begin_index] = 0;
        g[target_index] = INT_MAX;

        OPEN.push(pair{begin_laneuid, begin_index, f_value(g[begin_index], begin_laneuid, target_laneuid, target_xyz)});

        std::cout << "A star" << std::endl;
        while(!OPEN.empty())
        {
            pair cur = OPEN.top();
            OPEN.pop();
            CLOSE.push_back(cur);
            std::cout << "current lane uid: " << cur.lane_uid.road_id << " " << cur.lane_uid.section_index
                << " " << cur.lane_uid.local_id << " " << cur.index << std::endl;
            if (cur.index == target_index) {
                find_road = true;
                break;
            }

            std::vector<node> nodes;

            // 该车道的所有并行车道
            for(MLaneUId lane_uid : parall_nodes[cur.index]){
                nodes.push_back(node{lane_uid, -1, "parall"});
            }
            // 该车道的所有后继车道
            for(MLaneUId lane_uid : suc_nodes[cur.index]){
                if(lane_uid.road_id != "")
                    nodes.push_back(node{lane_uid, -1, "suc"});
            }
            for(int i = 0; i < lane_nodes.size(); i++){
                for(int j = 0; j < nodes.size(); j++)
                {
                    if(lane_nodes[i].road_id == nodes[j].lane_uid.road_id &&
                lane_nodes[i].section_index == nodes[j].lane_uid.section_index &&
                lane_nodes[i].local_id == nodes[j].lane_uid.local_id)
                    {
                        nodes[j].index = i;
                        break;
                    }
                }
            }

            // 对该车道的所有邻居
            std::cout << "nodes size: " << nodes.size() << std::endl;
            for(int i = 0; i < nodes.size(); i++)
            {
                double new_cost = 0;
                if(nodes[i].type == "parall")
                    new_cost = g[cur.index] + 0;
                if (nodes[i].type == "suc") {
                    MLaneInfo lane_info;
                    MErrorCode code = MapQueryLaneInfo(nodes[i].lane_uid, &lane_info);
                    new_cost = g[cur.index] + lane_info.length;
                }
                if(g.find(nodes[i].index) == g.end())
                    g[nodes[i].index] = INT_MAX;
                if(new_cost < g[nodes[i].index])
                {
                    g[nodes[i].index] = new_cost;
                    PARENT[nodes[i].index] = cur.index;
                    OPEN.push(pair{nodes[i].lane_uid, nodes[i].index, f_value(g[nodes[i].index], nodes[i].lane_uid, target_laneuid, target_xyz)});
                }
                std::cout << "neighbor lane uid: " << nodes[i].lane_uid.road_id << " " << nodes[i].lane_uid.section_index
                    << " " << nodes[i].lane_uid.local_id << " " << nodes[i].index << " " << f_value(g[nodes[i].index], nodes[i].lane_uid, target_laneuid, target_xyz) << std::endl;
            }
            std::cout << "quit " << OPEN.size() << std::endl;
        } //while
        if (find_road == true)
        {
            route_lane_uids.push_back(target_laneuid);
            int cur = target_index;
            route_lane_indexs.push_back(cur);
            while (true) {
                MLaneUId lane_uid = lane_nodes[PARENT[cur]];
                route_lane_uids.push_back(lane_uid);

                cur = PARENT[cur];
                route_lane_indexs.push_back(cur);

                if (cur == begin_index)
                    break;
            }
            std::reverse(route_lane_uids.begin(), route_lane_uids.end());
            std::reverse(route_lane_indexs.begin(), route_lane_indexs.end());
        }
        
    } //else
   
    // 输出
    if (!route_lane_uids.empty())
    {
        route->lane_uids.array = route_lane_uids;
        route->lane_uids.length = route_lane_uids.size();
        route->begin = begin_slz;
        route->end = target_slz;
        route->distance = 1000;

        // visiualize
        std::cout << "Visiualize" << std::endl;
        std::map<std::string, std::vector<Point>> vis;
        std::vector<Point> all_points;
        for (int i = 0; i < route_lane_indexs.size(); i++)
        {
            std::vector<int> indexs = lane_nodes_index[route_lane_indexs[i]];
            std::vector<Point> points;
            if (indexs[2] == 1)
                points = roads[indexs[0]].road_sections[indexs[1]].left_lanes[indexs[3]].center_line_points;
            else
                points = roads[indexs[0]].road_sections[indexs[1]].right_lanes[indexs[3]].center_line_points;
            all_points.insert(all_points.end(), points.begin(), points.end());
        }
        vis.insert(std::pair<std::string, std::vector<Point>>("test_points", all_points));

        std::vector<Point> other_points;
        for (int i = 0; i < roads.size(); i++) {
            for (int j = 0; j < roads[i].road_sections.size(); j++) {
                for (int k = 0; k < roads[i].road_sections[j].left_lanes.size(); k++)
                {
                    bool ishave = false;
                    for (auto route_lane_index : route_lane_indexs) {
                        std::vector<int> indexs = lane_nodes_index[route_lane_index];
                        if (i == indexs[0] && j == indexs[1] && indexs[2] == 1 && k == indexs[3]) {
                            ishave = true;
                            break;
                        }
                    }
                    if (ishave == false)
                        other_points.insert(other_points.end(), roads[i].road_sections[j].left_lanes[k].center_line_points.begin(), 
                            roads[i].road_sections[j].left_lanes[k].center_line_points.end());
                }
                for (int k = 0; k < roads[i].road_sections[j].right_lanes.size(); k++)
                {
                    bool ishave = false;
                    for (auto route_lane_index : route_lane_indexs) {
                        std::vector<int> indexs = lane_nodes_index[route_lane_index];
                        if (i == indexs[0] && j == indexs[1] && indexs[2] == -1 && k == indexs[3]) {
                            ishave = true;
                            break;
                        }
                    }
                    if (ishave == false)
                        other_points.insert(other_points.end(), roads[i].road_sections[j].right_lanes[k].center_line_points.begin(),
                            roads[i].road_sections[j].right_lanes[k].center_line_points.end());
                }
            }
        }
        vis.insert(std::pair<std::string, std::vector<Point>>("all_lane_center_points", other_points));

        visiualize.Visualize(vis);

        return 0;
    }
    else
        return -1;
}

MErrorCode OpenDriveParse::MapQueryLaneInfo(MLaneUId lane_uid, MLaneInfo* lane_info)
{
    lane_info->lane_uid = lane_uid;

    RoadSection section = roads[std::stoi(lane_uid.road_id)].road_sections[lane_uid.section_index];
    int lane_id = lane_uid.local_id;

    Lane lane;
    if (lane_id > 0)
        lane = section.left_lanes[section.left_lanes.size() - lane_id];
    else
        lane = section.right_lanes[int(-lane_id - 1)];

    lane_info->length = lane.length;
    lane_info->begin = section.s;
    lane_info->end = section.s + lane.length;

    return 0;
}

MErrorCode OpenDriveParse::MapQueryLaneSpeedAt(MLaneUId lane_uid, MRoadS s, double* speed_limit)
{
    Road road = roads[std::stoi(lane_uid.road_id)];
    if (s > road.type.back().s)
        *speed_limit = road.type.back().speed;
    else
        for (int i = 0; i < road.type.size() - 1; i++)
        {
            RoadType type = road.type[i];
            if (s >= type.s && s < road.type[i + 1].s)
            {
                *speed_limit = type.speed;
                break;
            }
        }
    *speed_limit *= 0.447;
    return 0;
}

MErrorCode OpenDriveParse::MapQueryLaneChangeTypeAt(MLaneUId lane_uid, MRoadS s, MLaneChangeType* lane_change_type)
{
    RoadSection section = roads[std::stoi(lane_uid.road_id)].road_sections[lane_uid.section_index];
    int lane_id = lane_uid.local_id;
    //if (lane_id > 0)
    //    *lane_change_type = section.left_lanes[section.left_lanes.size() - lane_id].lane_change;
    //else
    //    *lane_change_type = section.right_lanes[int(-lane_id - 1)].lane_change;
    return NoChange;
}

MErrorCode OpenDriveParse::MapQueryLaneBoundaries(MLaneUId lane_uid, double sampling_spacing, MSLZArray* left_boundary, MSLZArray* right_boundary)
{
    std::vector<MSLZ>& lborder_slz_points = left_boundary->array;
    std::vector<MSLZ>& rborder_slz_points = right_boundary->array;

    RoadSection section = roads[stoi(lane_uid.road_id)].road_sections[lane_uid.section_index];
    Lane lane;
    int lane_id = lane_uid.local_id;
    int lane_idx;
    if (lane_id > 0)
    {
        lane_idx = section.left_lanes.size() - lane_id;
        lane = section.left_lanes[section.left_lanes.size() - lane_id];
    }
    else
    {
        lane_idx = int(-lane_id - 1);
        lane = section.right_lanes[int(-lane_id - 1)];
    }
    int sampling_num = int(lane.length / sampling_spacing);
    for (int i = 0; i < sampling_num + 1; i++)
    {
        double s = section.s + i * sampling_spacing;
        double width = CalcLaneWidth(s, section.s, lane.width);
        double in_l = 0, out_l = 0;
        if (lane_id > 0)
        {
            for (int j = lane_idx + 1; j < section.left_lanes.size(); j++)
                in_l += CalcLaneWidth(s, section.s, section.left_lanes[j].width);
            out_l = in_l + width;
        }
        else
        {
            for (int j = lane_idx - 1; j >= 0; j--)
                in_l -= CalcLaneWidth(s, section.s, section.right_lanes[j].width);
            out_l = in_l - width;
        }
        MSLZ in_slz = CreateSLZ(lane_uid, s, in_l);
        MSLZ out_slz = CreateSLZ(lane_uid, s, out_l);
        if (lane_id > 0)
        {
            lborder_slz_points.push_back(out_slz);
            rborder_slz_points.push_back(in_slz);
        }
        else
        {
            lborder_slz_points.push_back(in_slz);
            rborder_slz_points.push_back(out_slz);
        }
    }
    left_boundary->length = lborder_slz_points.size();
    right_boundary->length = rborder_slz_points.size();

    return 0;
}

MErrorCode OpenDriveParse::MapCalcLaneCenterLine(MLaneUId lane_uid, double sampling_spacing, MSLZArray* result)
{
    MLaneInfo lane_info;
    MapQueryLaneInfo(lane_uid, &lane_info);

    return MapCalcLaneCenterLineInterval(lane_uid, lane_info.begin, lane_info.end, sampling_spacing, result);
}

MErrorCode OpenDriveParse::MapCalcLaneCenterLineInterval(MLaneUId lane_uid, double s1, double s2, double sampling_spacing, MSLZArray* result)
{
    std::vector<MSLZ>& slz_points = result->array;
    RoadSection section = roads[stoi(lane_uid.road_id)].road_sections[lane_uid.section_index];
    Lane lane;
    int lane_id = lane_uid.local_id;
    int lane_idx;
    if (lane_id > 0)
    {
        lane_idx = section.left_lanes.size() - lane_id;
        lane = section.left_lanes[section.left_lanes.size() - lane_id];
    }
    else
    {
        lane_idx = int(-lane_id - 1);
        lane = section.right_lanes[int(-lane_id - 1)];
    }
    int sampling_num = int((s2 - s1) / sampling_spacing);
    for (int i = 0; i < sampling_num + 1; i++)
    {
        double s = s1 + i * sampling_spacing;
        double width = CalcLaneWidth(s, section.s, lane.width);
        double in_l = 0, out_l = 0;
        if (lane_id > 0)
        {
            for (int j = lane_idx + 1; j < section.left_lanes.size(); j++)
                in_l += CalcLaneWidth(s, section.s, section.left_lanes[j].width);
            out_l = in_l + width;
        }
        else
        {
            for (int j = lane_idx - 1; j >= 0; j--)
                in_l -= CalcLaneWidth(s, section.s, section.right_lanes[j].width);
            out_l = in_l - width;
        }
        MSLZ slz = CreateSLZ(lane_uid, s, (in_l + out_l) / 2);
        slz_points.push_back(slz);
    }
    //result->array = &slz_points;
    result->length = slz_points.size();
    return 0;
}

MErrorCode OpenDriveParse::MapQueryRoadJunctionId(const std::string road_id, std::string* junction_id)
{
    *junction_id = roads[std::stoi(road_id)].junction_id;

    return 0;
}

bool OpenDriveParse::IsPointOnRoad(MXYZ& xyz, Road& road, int& idx, double& s, double& l)
{
    Point start = road.reference_line_points[0];
    Point end = road.reference_line_points[road.reference_line_points.size() - 1];

    double vector_a[2] = { end.x - start.x, end.y - start.y };
    double vector_b[2] = { xyz.x - start.x, xyz.y - start.y };
    double vector_c[2] = { xyz.x - end.x, xyz.y - end.y };
    double length_a = std::sqrtf(vector_a[0] * vector_a[0] + vector_a[1] * vector_a[1]);
    double length_b = std::sqrtf(vector_b[0] * vector_b[0] + vector_b[1] * vector_b[1]);
    double length_c = std::sqrtf(vector_c[0] * vector_c[0] + vector_c[1] * vector_c[1]);

    double cos_ab = (vector_a[0] * vector_b[0] + vector_a[1] * vector_b[1]) / (length_a * length_b);
    double cos_ac = (vector_a[0] * vector_c[0] + vector_a[1] * vector_c[1]) / (length_a * length_c);
    //std::cout << cos_ab << " " << cos_ac << std::endl;

    if (cos_ab * cos_ac <= 0)
    {
        double min_dot_product = std::abs((xyz.x - start.x) * (std::cos(start.hdg)) + (xyz.y - start.y) * (std::sin(start.hdg)));
        idx = 0;

        for (int j = 0; j < road.reference_line_points.size(); j++)
        {
            Point point = road.reference_line_points[j];
            double dot_product = std::abs((xyz.x - point.x) * (std::cos(point.hdg)) + (xyz.y - point.y) * (std::sin(point.hdg)));
            if (dot_product < min_dot_product)
            {
                min_dot_product = dot_product;
                idx = j;
            }
        }
        Point rpoint = road.reference_line_points[idx];
        CalculateFrenetPoint(rpoint.s, rpoint.x, rpoint.y, rpoint.hdg, xyz.x, xyz.y, &s, &l);
        return true;
    }
    else
        return false;
}


void OpenDriveParse::ParseToDiscretePoints()
{
    std::cout << std::setprecision(std::numeric_limits<long double>::digits10 + 1);

    for (int i = 0; i < roads.size(); i++)
    {
        // 解析道路，获得道路参考线的离散坐标点
        std::vector<Point> road_reference_points;
        double delta_s = 0.2;
        //std::cout << "road id: " << roads[i].id << std::endl;
        for (int j = 0; j < roads[i].reference_line.size(); j++)
        {
            Geometry geometry = roads[i].reference_line[j];
            double s = geometry.s;
            if (geometry.type == "Line")
            {
                int sample_num = int(geometry.length / delta_s);
                for (int k = 0; k < sample_num + 1; k++)
                {
                    double x = geometry.x + (delta_s * k) * std::cos(geometry.hdg);
                    double y = geometry.y + (delta_s * k) * std::sin(geometry.hdg);
                    road_reference_points.push_back(Point{ x, y, geometry.hdg, s, 0 });

                    s += delta_s; 
                }
            }
            else if (geometry.type == "Parampoly3")
            {
                ParamPoly3 parampoly3 = geometry.param_poly3;
                int sample_num = int(geometry.length / delta_s);
                double delta_normalize_s = 1.0 / sample_num;
                for (int k = 0; k < sample_num + 1; k++)
                {
                    double t = delta_normalize_s * k;
                    // 局部坐标
                    double u = parampoly3.aU + parampoly3.bU * t + parampoly3.cU * t * t + parampoly3.dU * t * t * t;
                    double v = parampoly3.aV + parampoly3.bV * t + parampoly3.cV * t * t + parampoly3.dV * t * t * t;
                    double du_dt = parampoly3.bU + 2 * parampoly3.cU * t + 3 * parampoly3.dU * t * t;
                    double dv_dt = parampoly3.bV + 2 * parampoly3.cV * t + 3 * parampoly3.dV * t * t;
                    double hdg = std::atan2(dv_dt, du_dt);

                    Point front = Point{ geometry.x, geometry.y, geometry.hdg, 0, 0 };
                    Point colc = Point{ u, v, hdg, 0, 0 };
                    Point next;   // 全局坐标
                    Local_To_Global(front, colc, next);
                    road_reference_points.push_back(Point{ next.x, next.y, next.hdg, s, 0 });

                    s += delta_s;
                }
            }
            else
            {
                ERROR("Con't find geomery in common define");
                return ;
            }
        }
        roads[i].reference_line_points = road_reference_points;

        // 解析车道，获得车道中心线离散坐标点
        int section_idx = 0;
        for (int j = 0; j < road_reference_points.size(); j++)
        {
            Point rpoint = road_reference_points[j];
            // 确定Section
            for (int k = 0; k < roads[i].road_sections.size(); k++)
            {
                if (rpoint.s >= roads[i].road_sections[roads[i].road_sections.size() - 1].s)
                {
                    section_idx = roads[i].road_sections.size() - 1;
                    break;
                }
                else if (rpoint.s >= roads[i].road_sections[k].s && rpoint.s < roads[i].road_sections[k + 1].s)
                {
                    section_idx = k;
                    break;
                }
            }
            RoadSection &road_section = roads[i].road_sections[section_idx];
            double s_section = road_section.s;
            // 解析左侧车道
            double l_left = 0;
            for (int idx = road_section.left_lanes.size() - 1; idx >= 0; idx--)
            {
                Lane &lane = road_section.left_lanes[idx];

                double width = CalcLaneWidth(rpoint.s, s_section, lane.width);
                Point point = CalculateCartesianPoint(rpoint, l_left + width / 2);
                lane.center_line_points.push_back(Point{ point.x, point.y, 0, rpoint.s, width, l_left + width / 2 });
                if (lane.road_id == "NONE")
                {
                    lane.road_id = roads[i].id;
                    lane.road_section = std::to_string(section_idx);
                    if(section_idx == roads[i].road_sections.size() - 1)
                        lane.length = roads[i].length - road_section.s;
                    else
                        lane.length = roads[i].road_sections[section_idx + 1].s - road_section.s;
                }
                l_left += width;
            }

            // 解析右侧车道
            double l_right = 0;
            for (int idx = 0; idx < road_section.right_lanes.size(); idx++)
            {
                Lane &lane = road_section.right_lanes[idx];

                double width = CalcLaneWidth(rpoint.s, s_section, lane.width);
                Point point = CalculateCartesianPoint(rpoint, l_right - width / 2);
                lane.center_line_points.push_back(Point{ point.x, point.y, 0, rpoint.s, width, l_right - width / 2 });
                if (lane.road_id == "NONE")
                {
                    lane.road_id = roads[i].id;
                    lane.road_section = std::to_string(section_idx);
                    if (section_idx == roads[i].road_sections.size() - 1)
                        lane.length = roads[i].length - road_section.s;
                    else
                        lane.length = roads[i].road_sections[section_idx + 1].s - road_section.s;
                }
                l_right -= width;
            }
        }
    }
}

void OpenDriveParse::CreateLaneNodes()
{
    for (int i = 0; i < roads.size(); i++) {
        std::string road_id = roads[i].id;
        std::string road_pre = roads[i].link.predecessor.element_id;
        std::string road_suc = roads[i].link.successor.element_id;

        for (int j = 0; j < roads[i].road_sections.size(); j++) {
            std::string section_index = roads[i].road_sections[j].id;
            for (int k = 0; k < roads[i].road_sections[j].left_lanes.size(); k++) {
                std::string local_id = roads[i].road_sections[j].left_lanes[k].id;
                std::string local_pre = roads[i].road_sections[j].left_lanes[k].predecessor_id;
                std::string local_suc = roads[i].road_sections[j].left_lanes[k].successor_id;

                lane_nodes.push_back({ road_id, std::stoi(section_index), std::stoi(local_id) });
                lane_nodes_index.push_back({ i, j, 1, k }); // 1代表左侧车道
                if (road_pre != "NONE" && local_pre != "NONE")
                    pre_nodes.push_back({ { road_pre, 0, stoi(local_pre) } });  // 假设车道段都为0
                else
                    pre_nodes.push_back({ EmptyLaneUId });
                if (road_suc != "NONE" && local_suc != "NONE")
                    suc_nodes.push_back({ { road_suc, 0, stoi(local_suc) } });  // 假设车道段都为0
                else
                    suc_nodes.push_back({ EmptyLaneUId });

                std::vector<MLaneUId> parall;
                // 不允许在交叉口平行变道
                if (roads[i].junction_id == "-1") {
                    for (int n = 0; n < roads[i].road_sections[j].left_lanes.size(); n++) {
                        if (n == k)
                            continue;
                        std::string parall_id = roads[i].road_sections[j].left_lanes[n].id;
                        parall.push_back({ road_id, stoi(section_index), stoi(parall_id) });
                    }
                }
                parall_nodes.push_back(parall);
            }
            for (int k = 0; k < roads[i].road_sections[j].right_lanes.size(); k++) {
                std::string local_id = roads[i].road_sections[j].right_lanes[k].id;
                std::string local_pre = roads[i].road_sections[j].right_lanes[k].predecessor_id;
                std::string local_suc = roads[i].road_sections[j].right_lanes[k].successor_id;

                lane_nodes.push_back({ road_id, std::stoi(section_index), std::stoi(local_id) });
                lane_nodes_index.push_back({ i, j, -1, k }); // -1代表右侧车道
                if (road_pre != "NONE" && local_pre != "NONE")
                    pre_nodes.push_back({ { road_pre, 0, stoi(local_pre) } });  // 假设车道段都为0
                else
                    pre_nodes.push_back({ EmptyLaneUId });
                if (road_suc != "NONE" && local_suc != "NONE")
                    suc_nodes.push_back({ { road_suc, 0, stoi(local_suc) } });  // 假设车道段都为0
                else
                    suc_nodes.push_back({ EmptyLaneUId });

                std::vector<MLaneUId> parall;
                // 不允许在交叉口平行变道
                if (roads[i].junction_id == "-1") {
                    for (int n = 0; n < roads[i].road_sections[j].right_lanes.size(); n++) {
                        if (n == k)
                            continue;
                        std::string parall_id = roads[i].road_sections[j].right_lanes[n].id;
                        parall.push_back({ road_id, stoi(section_index), stoi(parall_id) });
                    }
                }
                parall_nodes.push_back(parall);
            }
        }
    }

    for (int i = 0; i < junctions.size(); i++) {
        for (int j = 0; j < junctions[i].connection.size(); j++) {
            Connection connection = junctions[i].connection[j];
            if (connection.contactPoint == "start") {
                MLaneUId lane_pre = { connection.incomingRoad, 0, stoi(connection.lane_link.from) };
                MLaneUId lane_junc = { connection.connectingRoad, 0, stoi(connection.lane_link.to) };
                for (int k = 0; k < lane_nodes.size(); k++) {
                    if (lane_nodes[k].road_id == lane_junc.road_id &&
                        lane_nodes[k].section_index == lane_junc.section_index &&
                        lane_nodes[k].local_id == lane_junc.local_id) {
                        bool flag = false;
                        for (int n = 0; n < pre_nodes[k].size(); n++) {
                            if (pre_nodes[k][n].road_id == lane_pre.road_id &&
                                pre_nodes[k][n].section_index == lane_pre.section_index &&
                                pre_nodes[k][n].local_id == lane_pre.local_id)
                                flag = true;
                        }
                        if (flag == false)
                            pre_nodes[k].push_back({ connection.incomingRoad, 0, stoi(connection.lane_link.from) });
                    }
                    if (lane_nodes[k].road_id == lane_pre.road_id &&
                        lane_nodes[k].section_index == lane_pre.section_index &&
                        lane_nodes[k].local_id == lane_pre.local_id) {
                        bool flag = false;
                        for (int n = 0; n < suc_nodes[k].size(); n++) {
                            if (suc_nodes[k][n].road_id == lane_junc.road_id &&
                                suc_nodes[k][n].section_index == lane_junc.section_index &&
                                suc_nodes[k][n].local_id == lane_junc.local_id)
                                flag = true;
                        }
                        if (flag == false)
                            suc_nodes[k].push_back({ connection.connectingRoad, 0, stoi(connection.lane_link.to) });
                    }
                }
            }
            else {
                MLaneUId lane_suc = { connection.incomingRoad, 0, stoi(connection.lane_link.from) };
                MLaneUId lane_junc = { connection.connectingRoad, 0, stoi(connection.lane_link.to) };
                for (int k = 0; k < lane_nodes.size(); k++) {
                    if (lane_nodes[k].road_id == lane_junc.road_id &&
                        lane_nodes[k].section_index == lane_junc.section_index &&
                        lane_nodes[k].local_id == lane_junc.local_id) {
                        bool flag = false;
                        for (int n = 0; n < suc_nodes[k].size(); n++) {
                            if (suc_nodes[k][n].road_id == lane_suc.road_id &&
                                suc_nodes[k][n].section_index == lane_suc.section_index &&
                                suc_nodes[k][n].local_id == lane_suc.local_id)
                                flag = true;
                        }
                        if (flag == false)
                            suc_nodes[k].push_back({ connection.incomingRoad, 0, stoi(connection.lane_link.from) });
                    }
                    if (lane_nodes[k].road_id == lane_suc.road_id &&
                        lane_nodes[k].section_index == lane_suc.section_index &&
                        lane_nodes[k].local_id == lane_suc.local_id) {
                        bool flag = false;
                        for (int n = 0; n < pre_nodes[k].size(); n++) {
                            if (pre_nodes[k][n].road_id == lane_junc.road_id &&
                                pre_nodes[k][n].section_index == lane_junc.section_index &&
                                pre_nodes[k][n].local_id == lane_junc.local_id)
                                flag = true;
                        }
                        if (flag == false)
                            pre_nodes[k].push_back({ connection.connectingRoad, 0, stoi(connection.lane_link.to) });
                    }
                }
            }
        }
    }
    for (int i = 0; i < lane_nodes.size(); i++) 
    {
        MLaneUId cur_lane_uid = lane_nodes[i];
        for (int j = 0; j < suc_nodes[i].size(); j++)
        {
            if (suc_nodes[i][j].road_id != "")
            {
                MLaneUId lane_uid = suc_nodes[i][j];
                for (int k = 0; k < lane_nodes.size(); k++)
                {
                    if (lane_nodes[k].road_id == lane_uid.road_id
                        && lane_nodes[k].section_index == lane_uid.section_index
                        && lane_nodes[k].local_id == lane_uid.local_id)
                    {
                        bool ishave = false;
                        for (int m = 0; m < pre_nodes[k].size(); m++) 
                        {
                            if (pre_nodes[k][m].road_id == cur_lane_uid.road_id
                                && pre_nodes[k][m].section_index == cur_lane_uid.section_index
                                && pre_nodes[k][m].local_id == cur_lane_uid.local_id)
                            {
                                ishave = true;
                                break;
                            }
                        }
                        if (ishave == false)
                            pre_nodes[k].push_back(cur_lane_uid);
                        break;
                    }
                }
            }
        }
        for (int j = 0; j < pre_nodes[i].size(); j++)
        {
            if (pre_nodes[i][j].road_id != "")
            {
                MLaneUId lane_uid = pre_nodes[i][j];
                for (int k = 0; k < lane_nodes.size(); k++)
                {
                    if (lane_nodes[k].road_id == lane_uid.road_id
                        && lane_nodes[k].section_index == lane_uid.section_index
                        && lane_nodes[k].local_id == lane_uid.local_id)
                    {
                        bool ishave = false;
                        for (int m = 0; m < suc_nodes[k].size(); m++)
                        {
                            if (suc_nodes[k][m].road_id == cur_lane_uid.road_id
                                && suc_nodes[k][m].section_index == cur_lane_uid.section_index
                                && suc_nodes[k][m].local_id == cur_lane_uid.local_id)
                            {
                                ishave = true;
                                break;
                            }
                        }
                        if (ishave == false)
                            suc_nodes[k].push_back(cur_lane_uid);
                        break;
                    }
                }
            }
        }
    }
    std::cout << " Test Lane Node" << std::endl;
    std::cout << "********************************" << std::endl;
    for (int i = 0; i < lane_nodes.size(); i++) {
        std::cout << "cur node: " << lane_nodes[i].road_id << " " << lane_nodes[i].section_index << " " << lane_nodes[i].local_id << std::endl;
        int count = 0;

        for(int j = 0; j < parall_nodes[i].size(); j++)
            if (parall_nodes[i][j].road_id != "") {
                count++;
                std::cout << "parall node: " << parall_nodes[i][j].road_id << " " << parall_nodes[i][j].section_index << " " << parall_nodes[i][j].local_id << std::endl;
            }
        // std::cout << "parall nodes: " << count << std::endl;

        count = 0;
        for (int j = 0; j < pre_nodes[i].size(); j++)
            if (pre_nodes[i][j].road_id != "") {
                count++;
                std::cout << "pre node: " << pre_nodes[i][j].road_id << " " << pre_nodes[i][j].section_index << " " << pre_nodes[i][j].local_id << std::endl;
            }
        // std::cout << "pre nodes: " << count << std::endl;

        count = 0;
        for (int j = 0; j < suc_nodes[i].size(); j++)
            if (suc_nodes[i][j].road_id != "") {
                count++;
                std::cout << "suc node: " << suc_nodes[i][j].road_id << " " << suc_nodes[i][j].section_index << " " << suc_nodes[i][j].local_id << std::endl;
            }
        // std::cout << "suc nodes: " << count << std::endl;  
        count = 0;
    }
    std::cout << "********************************" << std::endl;
}

void OpenDriveParse::VisualizePoints()
{
    for (int index = 0; index < roads.size(); index++) {
        std::map<std::string, std::vector<Point>> visualize_points;

        std::vector<std::vector<std::string>> dont_draw;

        std::vector<Point> all_road_reference_points;
        std::vector<Point> all_lane_center_points;

        std::string road_id = roads[index].id;
        std::string pre_road_id = roads[index].link.predecessor.element_id;
        std::string suc_road_id = roads[index].link.successor.element_id;

        for (int j = 0; j < roads[index].road_sections.size(); j++)
        {
            std::vector<Point> left_lane_points;
            std::vector<Point> right_lane_points;
            std::vector<Point> pre_points;
            std::vector<Point> suc_points;
            for (int k = 0; k < roads[index].road_sections[j].left_lanes.size(); k++)
            {
                Lane lane = roads[index].road_sections[j].left_lanes[k];
                std::string local_id = lane.id;
                std::string pre_local_id = lane.predecessor_id;
                std::string suc_local_id = lane.successor_id;
                std::string pre_section_id = "0";
                std::string suc_section_id = "0";
                left_lane_points.insert(left_lane_points.end(), lane.center_line_points.begin(), lane.center_line_points.end());
                if (pre_road_id != "NONE" && pre_local_id != "NONE") {
                    dont_draw.push_back({ pre_road_id, pre_local_id });
                    for (int x = 0; x < roads.size(); x++)
                        if (roads[x].id != pre_road_id)
                            continue;
                        else
                            for (int y = 0; y < roads[x].road_sections.size(); y++)
                                if (roads[x].road_sections[y].id != pre_section_id)
                                    continue;
                                else {
                                    Lane pre_lane;
                                    if (stoi(pre_local_id) > 0) {
                                        std::vector<Lane> lanes = roads[x].road_sections[y].left_lanes;
                                        pre_lane = lanes[lanes.size() - stoi(pre_local_id)];
                                    }
                                    else {
                                        std::vector<Lane> lanes = roads[x].road_sections[y].right_lanes;
                                        pre_lane = lanes[abs(stoi(pre_local_id)) - 1];
                                    }
                                    pre_points.insert(pre_points.end(), pre_lane.center_line_points.begin(), pre_lane.center_line_points.end());
                                }
                }
                if (suc_road_id != "NONE" && suc_local_id != "NONE") {
                    dont_draw.push_back({ suc_road_id, suc_local_id });
                    for (int x = 0; x < roads.size(); x++)
                        if (roads[x].id != suc_road_id)
                            continue;
                        else
                            for (int y = 0; y < roads[x].road_sections.size(); y++)
                                if (roads[x].road_sections[y].id != suc_section_id)
                                    continue;
                                else {
                                    Lane suc_lane;
                                    if (stoi(suc_local_id) > 0) {
                                        std::vector<Lane> lanes = roads[x].road_sections[y].left_lanes;
                                        suc_lane = lanes[lanes.size() - stoi(suc_local_id)];
                                    }
                                    else {
                                        std::vector<Lane> lanes = roads[x].road_sections[y].right_lanes;
                                        suc_lane = lanes[abs(stoi(suc_local_id)) - 1];
                                    }
                                    suc_points.insert(suc_points.end(), suc_lane.center_line_points.begin(), suc_lane.center_line_points.end());
                                }
                }
            }
            for (int k = 0; k < roads[index].road_sections[j].right_lanes.size(); k++)
            {
                Lane lane = roads[index].road_sections[j].right_lanes[k];
                std::string local_id = lane.id;
                std::string pre_local_id = lane.predecessor_id;
                std::string suc_local_id = lane.successor_id;
                std::string pre_section_id = "0";
                std::string suc_section_id = "0";
                right_lane_points.insert(right_lane_points.end(), lane.center_line_points.begin(), lane.center_line_points.end());
                if (pre_road_id != "NONE" && pre_local_id != "NONE") {
                    dont_draw.push_back({ pre_road_id, pre_local_id });
                    for (int x = 0; x < roads.size(); x++)
                        if (roads[x].id != pre_road_id)
                            continue;
                        else
                            for (int y = 0; y < roads[x].road_sections.size(); y++)
                                if (roads[x].road_sections[y].id != pre_section_id)
                                    continue;
                                else {
                                    Lane pre_lane;
                                    if (stoi(pre_local_id) > 0) {
                                        std::vector<Lane> lanes = roads[x].road_sections[y].left_lanes;
                                        pre_lane = lanes[lanes.size() - stoi(pre_local_id)];
                                    }
                                    else {
                                        std::vector<Lane> lanes = roads[x].road_sections[y].right_lanes;
                                        pre_lane = lanes[abs(stoi(pre_local_id)) - 1];
                                    }
                                    pre_points.insert(pre_points.end(), pre_lane.center_line_points.begin(), pre_lane.center_line_points.end());
                                }
                }
                if (suc_road_id != "NONE" && suc_local_id != "NONE") {
                    dont_draw.push_back({ suc_road_id, suc_local_id });
                    for (int x = 0; x < roads.size(); x++)
                        if (roads[x].id != suc_road_id)
                            continue;
                        else
                            for (int y = 0; y < roads[x].road_sections.size(); y++)
                                if (roads[x].road_sections[y].id != suc_section_id)
                                    continue;
                                else {
                                    Lane suc_lane;
                                    if (stoi(suc_local_id) > 0) {
                                        std::vector<Lane> lanes = roads[x].road_sections[y].left_lanes;
                                        suc_lane = lanes[lanes.size() - stoi(suc_local_id)];
                                    }
                                    else {
                                        std::vector<Lane> lanes = roads[x].road_sections[y].right_lanes;
                                        suc_lane = lanes[abs(stoi(suc_local_id)) - 1];
                                    }
                                    suc_points.insert(suc_points.end(), suc_lane.center_line_points.begin(), suc_lane.center_line_points.end());
                                }
                }
            }
            visualize_points.insert(std::pair<std::string, std::vector<Point>>("1" + road_id, left_lane_points));
            visualize_points.insert(std::pair<std::string, std::vector<Point>>("2" + road_id, right_lane_points));
            visualize_points.insert(std::pair<std::string, std::vector<Point>>("pre_points", pre_points));
            visualize_points.insert(std::pair<std::string, std::vector<Point>>("suc_points", suc_points));
        }
        visualize_points.insert(std::pair<std::string, std::vector<Point>>("all_road_reference_points", roads[index].reference_line_points));

        // 获得所有的道路和车道中心线坐标点集
        for (int i = 0; i < roads.size(); i++)
        {
            if (i == index)
                continue;
            all_road_reference_points.insert(all_road_reference_points.end(), roads[i].reference_line_points.begin(), roads[i].reference_line_points.end());
            for (int j = 0; j < roads[i].road_sections.size(); j++)
            {
                for (int m = 0; m < roads[i].road_sections[j].left_lanes.size(); m++)
                {
                    bool draw = true;
                    for (auto dont : dont_draw)
                        if (dont[0] == roads[i].id && dont[1] == roads[i].road_sections[j].left_lanes[m].id)
                        {
                            draw = false;
                            break;
                        }
                    if (draw == false)
                        continue;
                    all_lane_center_points.insert(all_lane_center_points.end(), roads[i].road_sections[j].left_lanes[m].center_line_points.begin(),
                        roads[i].road_sections[j].left_lanes[m].center_line_points.end());
                }
                for (int m = 0; m < roads[i].road_sections[j].right_lanes.size(); m++)
                {
                    bool draw = true;
                    for (auto dont : dont_draw)
                        if (dont[0] == roads[i].id && dont[1] == roads[i].road_sections[j].right_lanes[m].id)
                        {
                            draw = false;
                            break;
                        }
                    if (draw == false)
                        continue;
                    all_lane_center_points.insert(all_lane_center_points.end(), roads[i].road_sections[j].right_lanes[m].center_line_points.begin(),
                        roads[i].road_sections[j].right_lanes[m].center_line_points.end());
                }
            }
        }

        //// Test
        //int nums[] = { 17, 14, 12, 6, 32, 31};
        //for (auto j : nums)
        //{
        //    int i = j;
        //    all_road_reference_points.insert(all_road_reference_points.end(), roads[i].reference_line_points.begin(), roads[i].reference_line_points.end());
        //    for (int j = 0; j < roads[i].road_sections.size(); j++)
        //    {
        //        for (int m = 0; m < roads[i].road_sections[j].left_lanes.size(); m++)
        //        {
        //            all_lane_center_points.insert(all_lane_center_points.end(), roads[i].road_sections[j].left_lanes[m].center_line_points.begin(),
        //                roads[i].road_sections[j].left_lanes[m].center_line_points.end());
        //        }
        //        for (int m = 0; m < roads[i].road_sections[j].right_lanes.size(); m++)
        //        {
        //            all_lane_center_points.insert(all_lane_center_points.end(), roads[i].road_sections[j].right_lanes[m].center_line_points.begin(),
        //                roads[i].road_sections[j].right_lanes[m].center_line_points.end());
        //        }
        //    }
        //}

        //visualize_points.insert(std::pair<std::string, std::vector<Point>>("all_road_reference_points", all_road_reference_points));
        visualize_points.insert(std::pair<std::string, std::vector<Point>>("all_lane_center_points", all_lane_center_points));

        // 额外加入测试点
        //std::vector<Point> test_points;
        //test_points.push_back({ -50.3, -9.0, 0 });
        //test_points.push_back({ 324.3, -121.9, 0 });
        ///*test_points.push_back({ 343.3, -104.4, 0 });
        //test_points.push_back({  -26.9, -57.0, 0 });*/
        //test_points.push_back({ -150.2, -59.3, 0 });

        //test_points.push_back({ 27.6, 1.8, 0 });

        //visualize_points.insert(std::pair<std::string, std::vector<Point>>("test_points", test_points));

        // 可视化
        visiualize.Visualize(visualize_points);
        std::cout << "Write OK " << std::endl;
    }
}

double OpenDriveParse::CalcLaneWidth(const double& s, const double& s_section, const std::vector<LaneWidth>& width)
{
    int idx_poly3 = 0;
    for (int k = 0; k < width.size(); k++)
    {
        if (k == width.size() - 1)
        {
            idx_poly3 = k;
            break;
        }
        if (s >= width[k].sOffset)
            continue;
        idx_poly3 = k - 1;
        break;
    }
    double sOffset = width[idx_poly3].sOffset;
    Poly3 poly3 = width[idx_poly3].poly3;
    double ds = s - s_section - sOffset;
    double calc_width = poly3.a + poly3.b * ds + poly3.c * ds * ds + poly3.d * ds * ds * ds;

    //std::cout << lane.id << " " << ds << " " << s << " " << s_section << " " << sOffset <<
    //    " width: " << width << std::endl;

    return calc_width;
}

bool OpenDriveParse::CheckFrenetSL(double& s, double& l, int& idx, Road& road, int& section_idx, std::string& lane_id)
{
    // Check S
    if (s < road.road_sections[0].s || s - road.road_sections[0].s > road.length)
        return false;
    if (s >= road.road_sections[road.road_sections.size() - 1].s)
    {
        section_idx = road.road_sections.size() - 1;
    }
    for (int j = 0; j < road.road_sections.size() - 1; j++)
    {
        if (s >= road.road_sections[j].s && s < road.road_sections[j + 1].s)
        {
            section_idx = j;
            break;
        }
    }
    //std::cout << "Section idx " << section_idx << std::endl;
    // Check L
    std::vector<Lane> lanes;
    if (l > 0)
    {
        lanes = road.road_sections[section_idx].left_lanes;
        if (lanes.size() == 0 || l > lanes[0].center_line_points[idx].l + lanes[0].center_line_points[idx].width / 2)
            return false;
    }
    else
    {
        lanes = road.road_sections[section_idx].right_lanes;
        if (lanes.size() == 0 || l < lanes[lanes.size() - 1].center_line_points[idx].l - lanes[lanes.size() - 1].center_line_points[idx].width / 2)
            return false;
    }
    for (int j = 0; j < lanes.size(); j++)
    {
        Point lane_point = lanes[j].center_line_points[idx];
        if (l > 0 && (l >= lane_point.l - lane_point.width / 2 && l < lane_point.l + lane_point.width / 2)
            ||
            (l < 0 && (l <= lane_point.l + lane_point.width / 2 && l > lane_point.l - lane_point.width / 2)))
        {
            lane_id = lanes[j].id;
            break;
        }
    }
    //std::cout << "lane id " << lane_id << std::endl;
    return true;
}

std::string OpenDriveParse::ContactLaneUId(const MLaneUId& lane_uid)
{
    return lane_uid.road_id + "#" + std::to_string(lane_uid.section_index) + "#" + std::to_string(lane_uid.local_id);
}

double OpenDriveParse::f_value(double g, MLaneUId cur_laneuid, MLaneUId target_laneuid, MXYZ target_xyz)
{
    double h = 0;
    if (cur_laneuid.road_id == target_laneuid.road_id)
        h = 0;
    else {
        MSLZArray cur_result;
        MErrorCode error = MapCalcLaneCenterLine(cur_laneuid, 0.4, &cur_result);
        MXYZ xyz;
        error = MapCalcXYZ(cur_result.array[cur_result.length - 1], &xyz);
        h = pow(xyz.x - target_xyz.x, 2) + pow(xyz.y - target_xyz.y, 2);
    }
    return g + h;
}







