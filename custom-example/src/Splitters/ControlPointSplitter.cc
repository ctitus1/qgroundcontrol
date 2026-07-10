/****************************************************************************
 *
 * (c) 2009-2024 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

#include "ControlPointSplitter.h"

#include "QGCGeo.h"

#include <QtCore/QPointF>
#include <QtCore/QObject>

#include <algorithm>
#include <cmath>
#include <limits>

namespace {

constexpr double kEps = 1e-6;

inline double cross(const QPointF& a, const QPointF& b)
{
    return (a.x() * b.y()) - (a.y() * b.x());
}

double signedArea(const QList<QPointF>& poly)
{
    double area = 0.0;
    const int n = poly.size();
    for (int i = 0; i < n; ++i) {
        const QPointF& a = poly[i];
        const QPointF& b = poly[(i + 1) % n];
        area += (a.x() * b.y()) - (b.x() * a.y());
    }
    return area * 0.5;
}

void ensureCCW(QList<QPointF>& poly)
{
    if (signedArea(poly) < 0.0) {
        std::reverse(poly.begin(), poly.end());
    }
}

bool pointInPolygon(const QList<QPointF>& poly, const QPointF& p)
{
    bool inside = false;
    const int n = poly.size();
    for (int i = 0, j = n - 1; i < n; j = i++) {
        const QPointF& a = poly[i];
        const QPointF& b = poly[j];
        const bool straddles = (a.y() > p.y()) != (b.y() > p.y());
        if (straddles) {
            const double xCross = ((b.x() - a.x()) * (p.y() - a.y()) / (b.y() - a.y())) + a.x();
            if (p.x() < xCross) {
                inside = !inside;
            }
        }
    }
    return inside;
}

// A cut point projected onto the polygon boundary.
struct Cut {
    int     segment  = 0;    ///< boundary edge index [V[segment], V[segment+1]]
    double  t        = 0.0;  ///< position along that edge [0,1]
    QPointF point;           ///< the projected point
    double  perimPos = 0.0;  ///< segment + t, for ordering around the boundary
};

// Project p onto the nearest point of the closed polygon boundary.
Cut projectToBoundary(const QList<QPointF>& V, const QPointF& p)
{
    const int n = V.size();
    Cut best;
    double bestDist = std::numeric_limits<double>::max();
    for (int j = 0; j < n; ++j) {
        const QPointF A = V[j];
        const QPointF B = V[(j + 1) % n];
        const QPointF ab = B - A;
        const double len2 = (ab.x() * ab.x()) + (ab.y() * ab.y());
        double t = (len2 > 0.0) ? (((p.x() - A.x()) * ab.x()) + ((p.y() - A.y()) * ab.y())) / len2 : 0.0;
        t = std::clamp(t, 0.0, 1.0);
        const QPointF proj(A.x() + (t * ab.x()), A.y() + (t * ab.y()));
        const double dist = std::hypot(p.x() - proj.x(), p.y() - proj.y());
        if (dist < bestDist) {
            bestDist = dist;
            best.segment = j;
            best.t = t;
            best.point = proj;
            best.perimPos = j + t;
        }
    }
    return best;
}

QList<QPointF> dedupe(const QList<QPointF>& poly)
{
    QList<QPointF> out;
    for (const QPointF& p : poly) {
        if (out.isEmpty() || std::hypot(p.x() - out.last().x(), p.y() - out.last().y()) > kEps) {
            out.append(p);
        }
    }
    while (out.size() >= 2 &&
           std::hypot(out.first().x() - out.last().x(), out.first().y() - out.last().y()) <= kEps) {
        out.removeLast();
    }
    return out;
}

} // anonymous namespace

QList<SplitRegion> ControlPointSplitter::split(const SplitInput& input, QString& errorString) const
{
    errorString.clear();

    if (input.masterPolygon.size() < 3) {
        errorString = QObject::tr("Survey polygon needs at least 3 vertices.");
        return {};
    }
    if (input.edgePoints.size() < 2) {
        errorString = QObject::tr("At least 2 control points are required to divide the area.");
        return {};
    }

    const QGeoCoordinate origin = input.masterPolygon.first();

    auto toLocal = [&origin](const QGeoCoordinate& c) {
        double north, east, down;
        QGCGeo::convertGeoToNed(c, origin, north, east, down);
        return QPointF(east, north);
    };

    QList<QPointF> V;
    V.reserve(input.masterPolygon.size());
    for (const QGeoCoordinate& c : input.masterPolygon) {
        V.append(toLocal(c));
    }
    ensureCCW(V);

    const QPointF center = toLocal(input.center);
    if (!pointInPolygon(V, center)) {
        errorString = QObject::tr("The center control point must be inside the survey area. Drag it inward.");
        return {};
    }

    // Each edge control point becomes a cut where it projects onto the boundary.
    QList<Cut> cuts;
    for (const QGeoCoordinate& e : input.edgePoints) {
        cuts.append(projectToBoundary(V, toLocal(e)));
    }
    std::sort(cuts.begin(), cuts.end(), [](const Cut& a, const Cut& b) {
        return a.perimPos < b.perimPos;
    });

    const int n = V.size();
    const int cutCount = cuts.size();

    QList<SplitRegion> regions;
    regions.reserve(cutCount);

    for (int i = 0; i < cutCount; ++i) {
        const Cut& a = cuts[i];
        const Cut& b = cuts[(i + 1) % cutCount];

        // Wedge = center -> cut(a) -> boundary vertices a..b -> cut(b).
        QList<QPointF> wedge;
        wedge.append(center);
        wedge.append(a.point);

        int k = a.segment;
        while (k != b.segment) {
            k = (k + 1) % n;
            wedge.append(V[k]);
        }
        wedge.append(b.point);

        wedge = dedupe(wedge);
        if (wedge.size() < 3 || std::abs(signedArea(wedge)) < 1.0 /* m^2 */) {
            continue;
        }

        SplitRegion region;
        region.polygon.reserve(wedge.size());
        for (const QPointF& p : wedge) {
            QGeoCoordinate c;
            QGCGeo::convertNedToGeo(p.y() /*north*/, p.x() /*east*/, 0.0, origin, c);
            region.polygon.append(c);
        }
        regions.append(region);
    }

    if (regions.isEmpty()) {
        errorString = QObject::tr("Could not generate any sub-regions from the current control points.");
    }
    return regions;
}
