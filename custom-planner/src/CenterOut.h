#pragma once
#include <QGeoCoordinate>
#include <QPointF>
#include <QList>
#include <algorithm>
#include <cmath>

namespace CenterOut {
struct Pass { QGeoCoordinate start, end; double across; };
inline QList<Pass> passes(const QList<QPair<QGeoCoordinate, QGeoCoordinate>>& legs,
                         const QGeoCoordinate& center, double heading)
{
    QList<Pass> result;
    const double radians = heading * M_PI / 180;
    const QPointF across(std::cos(radians), -std::sin(radians));
    const QPointF along(std::sin(radians), std::cos(radians));
    auto local = [&](const QGeoCoordinate& coordinate) {
        const double azimuth = center.azimuthTo(coordinate) * M_PI / 180;
        return QPointF(center.distanceTo(coordinate) * std::sin(azimuth),
                       center.distanceTo(coordinate) * std::cos(azimuth));
    };
    for (const auto& leg : legs) {
        if (leg.first.distanceTo(leg.second) < 0.1) continue;
        const auto midpoint = (local(leg.first) + local(leg.second)) / 2;
        result.append({leg.first, leg.second, QPointF::dotProduct(midpoint, across)});
    }
    std::stable_sort(result.begin(), result.end(), [](const Pass& a, const Pass& b) {
        return std::abs(a.across) < std::abs(b.across);
    });
    bool forward = result.isEmpty() || center.distanceTo(result.first().start) <= center.distanceTo(result.first().end);
    if (!result.isEmpty() && QPointF::dotProduct(local(result.first().end) - local(result.first().start), along) < 0)
        forward = !forward;
    for (auto& pass : result) {
        const bool aligned = QPointF::dotProduct(local(pass.end) - local(pass.start), along) > 0;
        if (aligned != forward) std::swap(pass.start, pass.end);
        forward = !forward;
    }
    return result;
}
}
