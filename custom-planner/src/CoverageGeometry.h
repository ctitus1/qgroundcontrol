#pragma once

#include <QPointF>
#include <QList>
#include <QString>
#include <algorithm>
#include <cmath>
#include <limits>

namespace CoverageGeometry {
struct Camera {
    double width, height, fx, fy, cx, cy;
};
struct Footprint {
    QList<QPointF> corners;
    QPointF center;
    double spacing = 0;
    QString error;
};

inline Footprint project(const Camera& camera, double altitude, double yaw, double pitch,
                         double trackAngle, double overlap)
{
    Footprint result;
    for (double value : {camera.width, camera.height, camera.fx, camera.fy, camera.cx, camera.cy,
                         altitude, yaw, pitch, trackAngle, overlap}) {
        if (!std::isfinite(value)) {
            result.error = "Enter finite camera and flight settings.";
            return result;
        }
    }
    if (camera.width <= 0 || camera.height <= 0 || camera.fx <= 0 || camera.fy <= 0 ||
        camera.cx < 0 || camera.cx > camera.width || camera.cy < 0 || camera.cy > camera.height ||
        altitude <= 0 || pitch < -90 || pitch >= 0 || overlap < 0 || overlap >= 95) {
        result.error = "Check the camera dimensions, height, pitch (-90 to below 0), and overlap (0 to below 95%).";
        return result;
    }
    constexpr double radians = 3.14159265358979323846 / 180.0;
    const double tilt = -pitch * radians;
    const double bearing = yaw * radians;
    const QPointF right(std::cos(bearing), -std::sin(bearing));
    const QPointF forward(std::sin(bearing), std::cos(bearing));
    for (const auto& pixel : {QPointF(0, 0), QPointF(camera.width, 0),
                             QPointF(camera.width, camera.height), QPointF(0, camera.height)}) {
        const double x = (pixel.x() - camera.cx) / camera.fx;
        const double y = (camera.cy - pixel.y()) / camera.fy;
        const double down = std::sin(tilt) - y * std::cos(tilt);
        if (down <= 0.02) {
            result.error = "The camera footprint reaches the horizon. Increase the downward pitch.";
            return result;
        }
        const QPointF point = altitude / down * (x * right +
            (std::cos(tilt) + y * std::sin(tilt)) * forward);
        result.corners.append(point);
        result.center += point / 4;
    }
    const QPointF along(std::sin(trackAngle * radians), std::cos(trackAngle * radians));
    const QPointF across(along.y(), -along.x());
    double halfSide = std::numeric_limits<double>::max();
    for (int i = 0; i < 4; ++i) {
        const QPointF edge = result.corners[(i + 1) % 4] - result.corners[i];
        const QPointF normal(-edge.y(), edge.x());
        const double distance = std::abs(QPointF::dotProduct(normal, result.center - result.corners[i]));
        const double extent = std::abs(QPointF::dotProduct(normal, along)) +
                              std::abs(QPointF::dotProduct(normal, across));
        halfSide = std::min(halfSide, distance / extent);
    }
    // A track-aligned square inside the trapezoid bounds both spacing and capture distance.
    result.spacing = 2 * halfSide * (1 - overlap / 100);
    if (!std::isfinite(result.spacing) || result.spacing < 0.5) {
        result.error = "The calculated spacing is below 0.5 m. Adjust the camera settings or overlap.";
    }
    return result;
}
}
