#include <iostream>
#include <corecrt_math_defines.h>
#include "util.h"

void ERROR(const std::string& error_msg)
{
    std::cout << error_msg << std::endl;
    //exit(0);
}

tinyxml2::XMLError QueryStringAttribute(const tinyxml2::XMLElement& xml_node, const std::string& name, std::string* value)
{
    const char* val = xml_node.Attribute(name.c_str());
    if (val == nullptr)
    {
        return tinyxml2::XML_NO_ATTRIBUTE;
    }
    *value = val;
    return tinyxml2::XML_SUCCESS;
}

// 归一化角度到 [-pi, pi]
double NormalizeAngle(const double angle)
{
    double a = fmod(angle + M_PI, 2.0 * M_PI);
    if (a < 0.0) {
        a += (2.0 * M_PI);
    }
    return a - M_PI;
}

void Local_To_Global(Point& front, Point& colc, Point& next)
{
    next.x = front.x + colc.x * std::cos(front.hdg) - colc.y * std::sin(front.hdg);
    next.y = front.y + colc.x * std::sin(front.hdg) + colc.y * std::cos(front.hdg);
    next.hdg = NormalizeAngle(colc.hdg + front.hdg);
}

Point CalculateCartesianPoint(const Point& rpoint, const double l)
{
    const double x = rpoint.x - l * std::sin(rpoint.hdg);
    const double y = rpoint.y + l * std::cos(rpoint.hdg);
    return Point{ x, y };
}

void CalculateFrenetPoint(
    const double rs, const double rx, const double ry, const double rtheta,
    const double x, const double y, double* ptr_s, double* ptr_l) {
    const double dx = x - rx;
    const double dy = y - ry;

    const double cos_theta_r = std::cos(rtheta);
    const double sin_theta_r = std::sin(rtheta);

    const double cross_rd_nd = cos_theta_r * dy - sin_theta_r * dx;
    *ptr_l = std::copysign(std::sqrt(dx * dx + dy * dy), cross_rd_nd);
    *ptr_s = rs;
}

double CalcSquareDistance(Point a, Point b)
{
    return (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y);
}