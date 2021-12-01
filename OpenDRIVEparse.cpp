// OpenDRIVEparse.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <iostream>
#include <string>
#include "opendrive_parse.h"
#include "roads_parse.h"
#include "lanes_parse.h"
#include "junctions_parse.h"

int main()
{
    // IS DEBUG
    RoadsParse::IS_DEBUG_ROAD = false;
    LanesParse::IS_DEBUG_LANE = false;
    JunctionsParse::IS_DEBUG_JUNCTION = false;

    std::string filename = "borregasave.xodr";
    //std::string filename = "AITownReconstructed_V0103_200518.xodr";
    //std::string filename = "sanfrancisco.xodr";
    //std::string filename = "cubetown.xodr";

    // Load map and parse roads, lanes, junctions.
    OpenDriveParse opendrive_parse;
    opendrive_parse.LoadMap(filename);

    // Discrete Points of road reference line and lane reference line
    opendrive_parse.ParseToDiscretePoints();

    // Create Lane Nodes used for A star algorithm planning
    opendrive_parse.CreateLaneNodes();

    // Visualize
    //opendrive_parse.VisualizeConnection();
    //opendrive_parse.VisualizePoints();

    bool test_api = true;
    if (test_api == true)
    {
        /*
        // Test MapFindSLZWithOutHInt API
        std::cout << "Test MapFindSLZWithOutHInt API" << std::endl;
        XYZ xyz = { -3.2, 37.8, 0 };
        SLZ slz;
        if (opendrive_parse.MapFindSLZWithOutHInt(xyz, &slz) == 0)
            std::cout << "(Road_id, Section_idx, Local_id) " << slz.lane_uid.road_id << " " << slz.lane_uid.section_idx << " " << slz.lane_uid.local_id
            << "   " << "(s, l, z) " << slz.s << " " << slz.l << " " << slz.z << std::endl;
        else
            std::cout << "Point Not in Road" << std::endl;

        // Test MapCalcXYZ API
        std::cout << "Test MapCalcXYZ API" << std::endl;
        XYZ calc_xyz;
        if (opendrive_parse.MapCalcXYZ(slz, &calc_xyz) == 0)
            std::cout << "(x, y, z) " << calc_xyz.x << " " << calc_xyz.y << " " << calc_xyz.z << std::endl;

        // Test MapQueryLaneInfo API
        std::cout << "Test MapQueryLaneInfo API" << std::endl;
        LaneUId lane_uid = slz.lane_uid;
        LaneInfo lane_info;
        if (opendrive_parse.MapQueryLaneInfo(lane_uid, &lane_info) == 0)
            std::cout << "(begin, end, length) " << lane_info.begin << " " << lane_info.end << " " << lane_info.length << std::endl;

        // Test MapQueryLaneSpeedAt API
        std::cout << "Test MapQueryLaneSpeedAt API" << std::endl;
        double speed_limit;
        if (opendrive_parse.MapQueryLaneSpeedAt(lane_uid, 20, &speed_limit) == 0)
            std::cout << "Speed " << speed_limit << std::endl;  // 单位 m/s
        std::cout << "Hello World!\n";

        // Test MapQueryLaneChangeTypeAt API
        std::cout << "Test MapQueryLaneChangeTypeAt API" << std::endl;
        std::string lane_change_type;
        if (opendrive_parse.MapQueryLaneChangeTypeAt({ "13", "0", "-1" }, 20, &lane_change_type) == 0)
            std::cout << "Lane change type " << lane_change_type << std::endl;

        // Test MapCalcLaneCenterLine API
        // Test MapCalcLaneCenterLineInterval API
        std::cout << "Test MapCalcLaneCenterLine API" << std::endl;
        SLZArray result;
        //if(opendrive_parse.MapCalcLaneCenterLineInterval(lane_uid, 20, 40, 0.1, &result) == 0)
        if (opendrive_parse.MapCalcLaneCenterLine(lane_uid, 0.1, &result) == 0)
        {
            std::cout << "Center points num " << result.length << std::endl;
            for (int i = 0; i < result.length; i++)
            {
                SLZ slz = result.array.at(i);
                std::cout << "(Road_id, Section_idx, Local_id) " << slz.lane_uid.road_id << " " << slz.lane_uid.section_idx << " " << slz.lane_uid.local_id
                    << "   " << "(s, l, z) " << slz.s << " " << slz.l << " " << slz.z << std::endl;
            }
        }

        // Test MapQueryLaneBoundaries API
        std::cout << "Test MapQueryLaneBoundaries API" << std::endl;
        SLZArray left_boundary, right_boundary;
        if (opendrive_parse.MapQueryLaneBoundaries(lane_uid, 0.1, &left_boundary, &right_boundary) == 0)
        {
            std::cout << "Left boundary points num " << left_boundary.length << std::endl;
            for (auto slz : left_boundary.array)
            {
                std::cout << "(Road_id, Section_idx, Local_id) " << slz.lane_uid.road_id << " " << slz.lane_uid.section_idx << " " << slz.lane_uid.local_id
                    << "   " << "(s, l, z) " << slz.s << " " << slz.l << " " << slz.z << std::endl;
            }
            std::cout << "Right boundary points num " << right_boundary.length << std::endl;
            for (auto slz : right_boundary.array)
            {
                std::cout << "(Road_id, Section_idx, Local_id) " << slz.lane_uid.road_id << " " << slz.lane_uid.section_idx << " " << slz.lane_uid.local_id
                    << "   " << "(s, l, z) " << slz.s << " " << slz.l << " " << slz.z << std::endl;
            }
        }

        // Test MapQueryRoadJunctionId API
        std::cout << "Test MapQueryRoadJunctionId API" << std::endl;
        std::string junction_id;
        if (opendrive_parse.MapQueryRoadJunctionId("10", &junction_id) == 0)
            std::cout << "Junction id " << junction_id << std::endl;

        */

        // Test MapPlanRoute API
        std::cout << std::endl << std::endl;
        MXYZ start_xyz = { -50.3, -9.0, 0 };
        MXYZ end_xyz = { 324.3, -121.9, 0 };
        MSLZ start_slz;


        MSLZ end_slz;
        if (opendrive_parse.MapFindSLZWithOutHInt(start_xyz, &start_slz) == 0)
        {
            MXYZ calc_xyz;
            std::cout << "Start Point ==== (Road_id, Section_idx, Local_id) " << start_slz.lane_uid.road_id << " " << start_slz.lane_uid.section_index << " " << start_slz.lane_uid.local_id
                << "   " << "(s, l, z) " << start_slz.s << " " << start_slz.l << " " << start_slz.z << std::endl;
                opendrive_parse.MapCalcXYZ(start_slz, &calc_xyz);
                std::cout << "(x, y, z) " << calc_xyz.x << " " << calc_xyz.y << " " << calc_xyz.z << std::endl;
        }

        else
            std::cout << "Point Not in Road" << std::endl;
        if (opendrive_parse.MapFindSLZWithOutHInt(end_xyz, &end_slz) == 0)
        {
            MXYZ calc_xyz;
            std::cout << "End Point ==== (Road_id, Section_idx, Local_id) " << end_slz.lane_uid.road_id << " " << end_slz.lane_uid.section_index << " " << end_slz.lane_uid.local_id
                << "   " << "(s, l, z) " << end_slz.s << " " << end_slz.l << " " << end_slz.z << std::endl;
                opendrive_parse.MapCalcXYZ(end_slz, &calc_xyz);
                std::cout << "(x, y, z) " << calc_xyz.x << " " << calc_xyz.y << " " << calc_xyz.z << std::endl;
        }

        else
            std::cout << "Point Not in Road" << std::endl;

        MAnchorArray array;
        array.array.push_back(MAnchor{ "start", start_slz });
        array.array.push_back(MAnchor{ "end", end_slz });
        array.length = 2;
        MRoute route;
        if (opendrive_parse.MapPlanRoute(array, &route) == 0)
        {
            std::cout << "Plan OK" << std::endl;
            for (int i = 0; i < route.lane_uids.array.size(); i++)
            {
                MLaneUId lane_uid = route.lane_uids.array[i];
                std::cout << lane_uid.road_id << " " << lane_uid.section_index << " " << lane_uid.local_id << std::endl;
            }
        }
        struct pair {
            int index;
            double f;
        };
        //pair p = { -12, 0.32 };
        //p.index = 10;
        //std::cout << p.index << std::endl;
    }
}

// 运行程序: Ctrl + F5 或调试 >“开始执行(不调试)”菜单
// 调试程序: F5 或调试 >“开始调试”菜单

// 入门使用技巧: 
//   1. 使用解决方案资源管理器窗口添加/管理文件
//   2. 使用团队资源管理器窗口连接到源代码管理
//   3. 使用输出窗口查看生成输出和其他消息
//   4. 使用错误列表窗口查看错误
//   5. 转到“项目”>“添加新项”以创建新的代码文件，或转到“项目”>“添加现有项”以将现有代码文件添加到项目
//   6. 将来，若要再次打开此项目，请转到“文件”>“打开”>“项目”并选择 .sln 文件
