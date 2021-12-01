#pragma once
#include "common_define.h"
#include "util.h"

class RoadsParse
{
public:
	static bool Parse(const tinyxml2::XMLElement& xml_node, std::vector<Road>* roads);

	static bool IS_DEBUG_ROAD;

private:
	static bool Parse_Road_Link(const tinyxml2::XMLElement& xml_node, RoadLink* link);

	static bool Parse_Road_ReferenceLine(const tinyxml2::XMLElement& xml_node, std::vector<Geometry>* referenceline);

	static bool Parse_Road_Type(const tinyxml2::XMLElement& xml_node, std::vector<RoadType>* types);

	
};

