#include "lanes_parse.h"
#include <iostream>

bool LanesParse::IS_DEBUG_LANE = false;

bool LanesParse::Parse(const tinyxml2::XMLElement& xml_node, std::vector<RoadSection>* sections)
{
    auto section_node = xml_node.FirstChildElement("laneSection");
    int section_id = 0;
    while (section_node)
    {
        if(LanesParse::IS_DEBUG_LANE)
            std::cout << "########" << std::endl;
        int checker = 0;
        RoadSection section;
        checker = section_node->QueryDoubleAttribute("s", &section.s);
        section.id = std::to_string(section_id);
        if (checker != tinyxml2::XML_SUCCESS) {
            ERROR("Error parsing section attributes");
            return false;
        }
        if(LanesParse::IS_DEBUG_LANE)
            std::cout << "section.id: " << section_id << " section.s:" << section.s << std::endl;

        // 道路中心线左侧车道
        auto left_node = section_node->FirstChildElement("left");
        if (left_node)
        {
            auto lane_node = left_node->FirstChildElement("lane");
            while (lane_node)
            {
                Lane lane;
                if(!Parse_lane(*lane_node, &lane))
                    ERROR("Parse left lane error");

                section.left_lanes.push_back(lane);
                lane_node = lane_node->NextSiblingElement("lane");
            }
        }
        if (LanesParse::IS_DEBUG_LANE)
        {
            for (int i = 0; i < section.left_lanes.size(); i++)
            {
                std::cout << i << " left_lanes: " << section.left_lanes[i].id << " predecessor id: " 
                    << section.left_lanes[i].predecessor_id << " successor id: " << section.left_lanes[i].successor_id << " lane type: "
                    << section.left_lanes[i].type << " lane changetype: " << section.left_lanes[i].lane_change << std::endl;
                for(int j = 0; j < section.left_lanes[i].width.size(); j++)
                    std::cout << "   " << j << " sOffset, a,b,c,d: " << section.left_lanes[i].width[j].sOffset << " "
                        << section.left_lanes[i].width[j].poly3.a << " "
                        << section.left_lanes[i].width[j].poly3.b << " "
                        << section.left_lanes[i].width[j].poly3.c << " "
                        << section.left_lanes[i].width[j].poly3.d << std::endl;
            }
        }

        // 中心车道 即道路参考线、道路中心线
        auto center_node = section_node->FirstChildElement("center");
        if (center_node)
        {
            //
        }

        // 道路中心线右侧车道
        auto right_node = section_node->FirstChildElement("right");
        if (right_node)
        {
            auto lane_node = right_node->FirstChildElement("lane");
            while (lane_node)
            {
                Lane lane;
                if (!Parse_lane(*lane_node, &lane))
                    ERROR("Parse right lane error");
                section.right_lanes.push_back(lane);
                lane_node = lane_node->NextSiblingElement("lane");
            }
        }
        if(LanesParse::IS_DEBUG_LANE)
        {
            for (int i = 0; i < section.right_lanes.size(); i++)
            {
                std::cout << i << " left_lanes: " << section.right_lanes[i].id << " predecessor id: "
                    << section.right_lanes[i].predecessor_id << " successor id: " << section.right_lanes[i].successor_id << " lane type: "
                    << section.right_lanes[i].type << " lane changetype: " << section.right_lanes[i].lane_change << std::endl;
                for(int j = 0; j < section.right_lanes[i].width.size(); j++)
                    std::cout << "   " << j << " sOffset, a,b,c,d: " << section.right_lanes[i].width[j].sOffset << " "
                        << section.right_lanes[i].width[j].poly3.a << " "
                        << section.right_lanes[i].width[j].poly3.b << " "
                        << section.right_lanes[i].width[j].poly3.c << " "
                        << section.right_lanes[i].width[j].poly3.d << std::endl;
            }
        }
        
        sections->push_back(section);
        section_node = section_node->NextSiblingElement("laneSection");
        section_id++;

        if (LanesParse::IS_DEBUG_LANE)
            std::cout << "########" << std::endl;
    }

    return true;
}

bool LanesParse::Parse_lane(const tinyxml2::XMLElement& xml_node, Lane* lane)
{
    int checker = 0;
    checker = QueryStringAttribute(xml_node, "id", &lane->id);
    checker += QueryStringAttribute(xml_node, "type", &lane->type);
    auto link_node = xml_node.FirstChildElement("link");
    if (link_node)
    {
        auto predecessor_node = link_node->FirstChildElement("predecessor");
        if (predecessor_node)
            checker += QueryStringAttribute(*predecessor_node, "id", &lane->predecessor_id);

        auto successor_node = link_node->FirstChildElement("successor");
        if (successor_node)
            checker += QueryStringAttribute(*successor_node, "id", &lane->successor_id);

    }

    auto roadmark_node = xml_node.FirstChildElement("roadMark");
    if(roadmark_node)
        checker += QueryStringAttribute(*roadmark_node, "laneChange", &lane->lane_change);

    auto width_node = xml_node.FirstChildElement("width");
    while (width_node)
    {
        LaneWidth width;
        checker += width_node->QueryDoubleAttribute("sOffset", &width.sOffset);
        checker += width_node->QueryDoubleAttribute("a", &width.poly3.a);
        checker += width_node->QueryDoubleAttribute("b", &width.poly3.b);
        checker += width_node->QueryDoubleAttribute("c", &width.poly3.c);
        checker += width_node->QueryDoubleAttribute("d", &width.poly3.d);

        lane->width.push_back(width);
        width_node = width_node->NextSiblingElement("width");
    }
    if (checker != tinyxml2::XML_SUCCESS) {
        return false;
    }
    return true;
}
