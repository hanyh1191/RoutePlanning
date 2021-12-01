#pragma once
#include <string>
#include "tinyxml2.h"
#include "common_define.h"

void ERROR(const std::string& error_msg);

tinyxml2::XMLError QueryStringAttribute(const tinyxml2::XMLElement& xml_node, const std::string& name, std::string* value);

double NormalizeAngle(const double angle);

void Local_To_Global(Point& front, Point& colc, Point& next);

Point CalculateCartesianPoint(const Point& rpoint, const double l);

void CalculateFrenetPoint(
    const double rs, const double rx, const double ry, const double rtheta,
    const double x, const double y, double* ptr_s, double* ptr_l);

double CalcSquareDistance(Point a, Point b);
