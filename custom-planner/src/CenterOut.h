#pragma once
#include <QGeoCoordinate>
#include <QPointF>
#include <QList>
#include <algorithm>
#include <cmath>

namespace CenterOut {
inline double normalizeAngle(double angle)
{
    const double normalized = std::fmod(angle, 360.);
    return normalized < 0 ? normalized + 360. : normalized;
}

inline double inwardHeading(const QList<QGeoCoordinate>& sector, const QGeoCoordinate& center)
{
    if (sector.size() < 3 || !center.isValid()) return 0;
    const double start = center.azimuthTo(sector[1]);
    const double sweep = normalizeAngle(start - center.azimuthTo(sector.last()));
    return normalizeAngle(start - sweep / 2 + 180.);
}

struct Pass { QGeoCoordinate start, end; double distance; };
inline QList<Pass> passes(const QList<QPair<QGeoCoordinate, QGeoCoordinate>>& legs,
                         const QGeoCoordinate& center, double heading, bool reverseEntry = false)
{
    QList<Pass> result;
    const double radians = heading * M_PI / 180;
    const QPointF along(std::sin(radians), std::cos(radians));
    auto local = [&](const QGeoCoordinate& coordinate) {
        const double azimuth = center.azimuthTo(coordinate) * M_PI / 180;
        return QPointF(center.distanceTo(coordinate) * std::sin(azimuth),
                       center.distanceTo(coordinate) * std::cos(azimuth));
    };
    for (const auto& leg : legs) {
        if (leg.first.distanceTo(leg.second) < 0.1) continue;
        const auto start=local(leg.first), end=local(leg.second), delta=end-start;
        const double t=std::clamp(-QPointF::dotProduct(start,delta)/QPointF::dotProduct(delta,delta),0.,1.);
        const auto nearest=start+t*delta;
        result.append({leg.first,leg.second,std::hypot(nearest.x(),nearest.y())});
    }
    std::stable_sort(result.begin(), result.end(), [](const Pass& a, const Pass& b) {
        return a.distance < b.distance;
    });
    bool forward = result.isEmpty() || center.distanceTo(result.first().start) <= center.distanceTo(result.first().end);
    if (!result.isEmpty() && QPointF::dotProduct(local(result.first().end) - local(result.first().start), along) < 0)
        forward = !forward;
    if (reverseEntry) forward = !forward;
    for (auto& pass : result) {
        const bool aligned = QPointF::dotProduct(local(pass.end) - local(pass.start), along) > 0;
        if (aligned != forward) std::swap(pass.start, pass.end);
        forward = !forward;
    }
    return result;
}
}
