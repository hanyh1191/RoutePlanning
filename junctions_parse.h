#pragma once
#include "common_define.h"
#include "util.h"

class JunctionsParse
{
public:
	static bool Parse(const tinyxml2::XMLElement& xml_node, std::vector<Junction>* junctions);

	static bool IS_DEBUG_JUNCTION;
};

