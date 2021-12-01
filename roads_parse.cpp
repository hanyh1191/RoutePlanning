#include "roads_parse.h"
#include "lanes_parse.h"
#include <iostream>

bool RoadsParse::IS_DEBUG_ROAD = false;

bool RoadsParse::Parse(const tinyxml2::XMLElement& xml_node, std::vector<Road>* roads)
{
    auto road_node = xml_node.FirstChildElement("road");
    while (road_node) {

        if(RoadsParse::IS_DEBUG_ROAD)
            std::cout << "*************" << std::endl;

        int checker;

        Road road;

        // 道路属性
        checker = QueryStringAttribute(*road_node, "id", &road.id);
        checker += QueryStringAttribute(*road_node, "junction", &road.junction_id);
        checker += road_node->QueryDoubleAttribute("length", &road.length);
        if (checker != tinyxml2::XML_SUCCESS) {
            ERROR("Error parsing road attributes");
            return false;
        }
        if (RoadsParse::IS_DEBUG_ROAD)
        {
            std::cout << "id: " << road.id << " junction_id: " << road.junction_id
                << " length: " << road.length << std::endl;
        }
        
        // 道路连接
        auto link_node = road_node->FirstChildElement("link");
        if (link_node)
            if (!Parse_Road_Link(*link_node, &road.link))
            {
                ERROR("Error parsing road link");
                return false;
            }
        if (RoadsParse::IS_DEBUG_ROAD)
        {
            std::cout << "predecessor.element_type: " << road.link.predecessor.element_type
                << " predecessor.element_id: " << road.link.predecessor.element_id
                << " predecessor.contact_point: " << road.link.predecessor.contact_point << std::endl;
            std::cout << "successor.element_type: " << road.link.successor.element_type
                << " successor.element_id: " << road.link.successor.element_id
                << " successor.contact_point: " << road.link.successor.contact_point << std::endl;
        }
        
        // 道路类型
        if (road_node)
            if (!Parse_Road_Type(*road_node, &road.type))
            {
                ERROR("Error parsing road type");
                return false;
            }
        if (RoadsParse::IS_DEBUG_ROAD)
            for(int i = 0; i < road.type.size(); i++)
                std::cout << "type " << i << " s: " << road.type[i].s << " type: " << road.type[i].type 
                    << " speed: " << road.type[i].speed << " unit: " << road.type[i].speed_unit << std::endl;

        // 道路中心线
        auto planView_node = road_node->FirstChildElement("planView");
        if (planView_node)
            if (!Parse_Road_ReferenceLine(*planView_node, &road.reference_line))
            {
                ERROR("Error parsing road referenceLine");
                return false;
            }  
        if (RoadsParse::IS_DEBUG_ROAD)
        {
            for (int i = 0; i < road.reference_line.size(); i++)
            {
                std::cout << i << " reference_line(s,x,y,hdg,length,type): " << road.reference_line[i].s << " "
                    << road.reference_line[i].x << " " << road.reference_line[i].y << " " << road.reference_line[i].hdg
                    << " " << road.reference_line[i].length << " " << road.reference_line[i].type << std::endl;
                if (road.reference_line[i].type == "Parampoly3")
                    std::cout << "paramPoly3:" << road.reference_line[i].param_poly3.aU << " " << road.reference_line[i].param_poly3.bU << " "
                    << road.reference_line[i].param_poly3.cU << " " << road.reference_line[i].param_poly3.dU << " "
                    << road.reference_line[i].param_poly3.aV << " " << road.reference_line[i].param_poly3.bV << " "
                    << road.reference_line[i].param_poly3.cV << " " << road.reference_line[i].param_poly3.dV << std::endl;
            }
        }
        
        // 车道
        auto lanes_node = road_node->FirstChildElement("lanes");
        if(lanes_node)
            if (!LanesParse::Parse(*lanes_node, &road.road_sections))
            {
                ERROR("Error parsing lanes");
                return false;
            }

        roads->push_back(road);
        road_node = road_node->NextSiblingElement("road");

        if(RoadsParse::IS_DEBUG_ROAD)
            std::cout << "*************" << std::endl;
    }
    return true;
}

bool RoadsParse::Parse_Road_Link(const tinyxml2::XMLElement& xml_node, RoadLink* link)
{
    int checker = 0;
    auto predecessor_node = xml_node.FirstChildElement("predecessor");
    if (predecessor_node) {
        checker = QueryStringAttribute(*predecessor_node, "elementType", &(link->predecessor.element_type));
        checker += QueryStringAttribute(*predecessor_node, "elementId", &(link->predecessor.element_id));
        if(predecessor_node->FindAttribute("contactPoint") != nullptr)
            checker += QueryStringAttribute(*predecessor_node, "contactPoint", &(link->predecessor.contact_point));
    }
    auto successor_node = xml_node.FirstChildElement("successor");
    if (successor_node) {
        checker = QueryStringAttribute(*successor_node, "elementType", &(link->successor.element_type));
        checker += QueryStringAttribute(*successor_node, "elementId", &(link->successor.element_id));
        if (successor_node->FindAttribute("contactPoint") != nullptr)
            checker += QueryStringAttribute(*successor_node, "contactPoint", &(link->successor.contact_point));
    }
    if (checker != tinyxml2::XML_SUCCESS) {
        return false;
    }
    return true;
}

bool RoadsParse::Parse_Road_ReferenceLine(const tinyxml2::XMLElement& xml_node, std::vector<Geometry>* referenceline)
{
    int checker = 0;
    auto geometry_node = xml_node.FirstChildElement("geometry");
    while (geometry_node)
    {
        Geometry geometry;
        checker = geometry_node->QueryDoubleAttribute("s", &geometry.s);
        checker += geometry_node->QueryDoubleAttribute("x", &geometry.x);
        checker += geometry_node->QueryDoubleAttribute("y", &geometry.y);
        checker += geometry_node->QueryDoubleAttribute("hdg", &geometry.hdg);
        checker += geometry_node->QueryDoubleAttribute("length", &geometry.length);

        auto geometry_param_node = geometry_node->FirstChildElement();
        if(strcmp(geometry_param_node->Value(), "line") == 0)
            geometry.type = "Line";
        else if(strcmp(geometry_param_node->Value(), "paramPoly3") == 0){
            geometry.type = "Parampoly3";

            auto parampoly3_node = geometry_param_node;
            checker += parampoly3_node->QueryDoubleAttribute("aU", &geometry.param_poly3.aU);
            checker += parampoly3_node->QueryDoubleAttribute("bU", &geometry.param_poly3.bU);
            checker += parampoly3_node->QueryDoubleAttribute("cU", &geometry.param_poly3.cU);
            checker += parampoly3_node->QueryDoubleAttribute("dU", &geometry.param_poly3.dU);
            checker += parampoly3_node->QueryDoubleAttribute("aV", &geometry.param_poly3.aV);
            checker += parampoly3_node->QueryDoubleAttribute("bV", &geometry.param_poly3.bV);
            checker += parampoly3_node->QueryDoubleAttribute("cV", &geometry.param_poly3.cV);
            checker += parampoly3_node->QueryDoubleAttribute("dV", &geometry.param_poly3.dV);
        }
        else
        {
            ERROR("Con't find geomery in common define");
            return false;
        }

        referenceline->push_back(geometry);
        geometry_node = geometry_node->NextSiblingElement("geometry");
    }

    if (checker != tinyxml2::XML_SUCCESS) {
        return false;
    }
    return true;
}

bool RoadsParse::Parse_Road_Type(const tinyxml2::XMLElement& xml_node, std::vector<RoadType>* types)
{
    int checker = 0;
    auto type_node = xml_node.FirstChildElement("type");
    while (type_node)
    {
        RoadType type;

        checker = type_node->QueryDoubleAttribute("s", &type.s);
        checker += QueryStringAttribute(*type_node, "type", &type.type);
        auto speed_node = type_node->FirstChildElement("speed");
        if (speed_node) {
            checker += speed_node->QueryDoubleAttribute("max", &type.speed);    // 1mph=1 609.344m/3600s=0.447m/s
            //type.speed *= 0.447;
            checker += QueryStringAttribute(*speed_node, "unit", &type.speed_unit);

        }
        types->push_back(type);
        type_node = type_node->NextSiblingElement("type");
    }
    if (checker != tinyxml2::XML_SUCCESS) {
        return false;
    }
    return true;
}
