#pragma once
#include <QGeoCoordinate>
#include <QPointF>
#include <QList>
#include <algorithm>
#include <cmath>

namespace CenterOut {
struct Pass { QGeoCoordinate start, end; int ring; double across; };
inline QList<Pass> passes(const QList<QPair<QGeoCoordinate, QGeoCoordinate>>& legs,
                         const QGeoCoordinate& center, double step, double heading)
{
    QList<Pass> result;
    auto local = [&](const QGeoCoordinate& c) {
        const double a = center.azimuthTo(c) * M_PI / 180;
        return QPointF(center.distanceTo(c) * std::sin(a), center.distanceTo(c) * std::cos(a));
    };
    auto geo = [&](const QPointF& p) {
        return center.atDistanceAndAzimuth(std::hypot(p.x(), p.y()), std::atan2(p.x(), p.y()) * 180 / M_PI);
    };
    const QPointF across(std::cos(heading * M_PI / 180), -std::sin(heading * M_PI / 180));
    for (const auto& leg : legs) {
        const QPointF a = local(leg.first), b = local(leg.second), d = b - a;
        const double aa = QPointF::dotProduct(d,d), bb = 2 * QPointF::dotProduct(a,d);
        if (aa < 0.01) continue;
        const double radius = std::max(std::hypot(a.x(),a.y()), std::hypot(b.x(),b.y()));
        QList<double> cuts{0,1};
        for (double r = step; r < radius; r += step) {
            const double disc = bb*bb - 4*aa*(QPointF::dotProduct(a,a)-r*r);
            if (disc <= 0) continue;
            for (double t : {(-bb-std::sqrt(disc))/(2*aa), (-bb+std::sqrt(disc))/(2*aa)}) if (t > 0 && t < 1) cuts.append(t);
        }
        std::sort(cuts.begin(),cuts.end());
        for (int i=1; i<cuts.size(); ++i) {
            const QPointF start = a + cuts[i-1]*d, end = a + cuts[i]*d, mid = (start+end)/2;
            result.append({geo(start),geo(end),int(std::floor(std::hypot(mid.x(),mid.y())/step)),QPointF::dotProduct(mid,across)});
        }
    }
    std::stable_sort(result.begin(),result.end(),[](const Pass& a,const Pass& b) {
        return a.ring != b.ring ? a.ring < b.ring : a.across < b.across;
    });
    QGeoCoordinate previous = center;
    for (auto& pass : result) {
        if (previous.distanceTo(pass.end) < previous.distanceTo(pass.start)) std::swap(pass.start,pass.end);
        previous = pass.end;
    }
    return result;
}
}
