#include "junctions_parse.h"
#include <iostream>

bool JunctionsParse::IS_DEBUG_JUNCTION = false;

bool JunctionsParse::Parse(const tinyxml2::XMLElement& xml_node, std::vector<Junction>* junctions)
{
    int checker = 0;
    auto junction_node = xml_node.FirstChildElement("junction");
    while (junction_node)
    {
        Junction junction;
        
        checker = QueryStringAttribute(*junction_node, "id", &junction.id);
        auto connection_node = junction_node->FirstChildElement("connection");
        while (connection_node)
        {
            Connection connection;
            checker += QueryStringAttribute(*connection_node, "id", &connection.id);
            checker += QueryStringAttribute(*connection_node, "incomingRoad", &connection.incomingRoad);
            checker += QueryStringAttribute(*connection_node, "connectingRoad", &connection.connectingRoad);
            checker += QueryStringAttribute(*connection_node, "contactPoint", &connection.contactPoint);
            auto lanelink_node = connection_node->FirstChildElement("laneLink");
            if (lanelink_node)
            {
                checker += QueryStringAttribute(*lanelink_node, "from", &connection.lane_link.from);
                checker += QueryStringAttribute(*lanelink_node, "to", &connection.lane_link.to);
            }
            junction.connection.push_back(connection);
            connection_node = connection_node->NextSiblingElement("connection");
        }
        junctions->push_back(junction);
        junction_node = junction_node->NextSiblingElement("junction");
    }
    if (checker != tinyxml2::XML_SUCCESS) {
        ERROR("Error parsing junction");
        return false;
    }
    if (JunctionsParse::IS_DEBUG_JUNCTION)
    {
        std::cout << "&&&&&&&&&" << std::endl;
        for (int i = 0; i < junctions->size(); i++)
        {
            std::cout << "Junction id: " << junctions->at(i).id << std::endl;
            for (int j = 0; j < junctions->at(i).connection.size(); j++)
            {
                std::cout << "   connection: " << junctions->at(i).connection.at(j).id << " incoming road: " <<
                    junctions->at(i).connection.at(j).incomingRoad << " connecting road: " <<
                    junctions->at(i).connection.at(j).connectingRoad << " contact point: " <<
                    junctions->at(i).connection.at(j).contactPoint << " lane link from: " <<
                    junctions->at(i).connection.at(j).lane_link.from << " lane link to: " <<
                    junctions->at(i).connection.at(j).lane_link.to << std::endl;
            }
        }
        std::cout << "&&&&&&&&&" << std::endl;
    }
    return true;
}
