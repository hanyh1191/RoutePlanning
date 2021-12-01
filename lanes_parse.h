#pragma once
#include "common_define.h"
#include "util.h"

class LanesParse
{
public:
	static bool Parse(const tinyxml2::XMLElement& xml_node, std::vector<RoadSection>* sections);

	static bool IS_DEBUG_LANE;
private:
	static bool Parse_lane(const tinyxml2::XMLElement& xml_node, Lane* lane);

};

