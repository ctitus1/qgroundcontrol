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

// 2D cross product (z component of a x b).
inline double cross(const QPointF& a, const QPointF& b)
{
    return (a.x() * b.y()) - (a.y() * b.x());
}

// Signed area of a planar polygon (positive == counter-clockwise).
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

// Even-odd point-in-polygon test in planar coordinates.
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

// Intersect ray (origin C, direction d) with segment [A,B].
// On success returns true and sets P (hit point) and tRay (distance param along d, >= 0).
bool rayEdgeIntersect(const QPointF& C, const QPointF& d,
                      const QPointF& A, const QPointF& B,
                      QPointF& P, double& tRay)
{
    const QPointF ab = B - A;
    const double denom = cross(d, ab);
    if (std::abs(denom) < 1e-12) {
        return false;   // ray parallel to edge
    }
    const QPointF ac = A - C;
    const double t = cross(ac, ab) / denom;   // distance along ray
    const double u = cross(ac, d)  / denom;   // position along edge [0,1]
    if (t < kEps) {
        return false;   // behind or at the ray origin
    }
    if (u < -kEps || u > 1.0 + kEps) {
        return false;   // outside the segment
    }
    P = C + (t * d);
    tRay = t;
    return true;
}

struct RayHit {
    double  angle    = 0.0;   ///< direction angle around center (radians)
    int     edgeIndex = -1;   ///< index j of boundary edge [V[j], V[j+1]] containing the exit
    QPointF point;            ///< exit point on the boundary
};

// Nearest boundary exit of a ray cast from C in direction d. Returns false if
// the ray never crosses the boundary (center cannot "see" out in that direction).
bool nearestExit(const QList<QPointF>& V, const QPointF& C, const QPointF& d, RayHit& hit)
{
    const int n = V.size();
    double bestT = std::numeric_limits<double>::max();
    bool found = false;
    for (int j = 0; j < n; ++j) {
        QPointF P;
        double t;
        if (rayEdgeIntersect(C, d, V[j], V[(j + 1) % n], P, t)) {
            if (t < bestT) {
                bestT = t;
                hit.edgeIndex = j;
                hit.point = P;
                found = true;
            }
        }
    }
    return found;
}

// Drop consecutive near-duplicate vertices (and a duplicated closing vertex).
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

    // Project polygon, center and edge points onto a local tangent plane
    // (x == East, y == North) so we can do planar geometry.
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

    // One ray per edge control point, sorted CCW by direction around the center.
    QList<RayHit> hits;
    for (const QGeoCoordinate& e : input.edgePoints) {
        const QPointF ep = toLocal(e);
        const QPointF d(ep.x() - center.x(), ep.y() - center.y());
        if (std::hypot(d.x(), d.y()) < kEps) {
            continue;   // control point sits on the center; ignore
        }
        RayHit hit;
        hit.angle = std::atan2(d.y(), d.x());
        if (!nearestExit(V, center, d, hit)) {
            errorString = QObject::tr("A control point does not project onto the survey boundary. Move the center toward the middle of the area.");
            return {};
        }
        hits.append(hit);
    }

    if (hits.size() < 2) {
        errorString = QObject::tr("At least 2 valid control points are required.");
        return {};
    }

    std::sort(hits.begin(), hits.end(), [](const RayHit& a, const RayHit& b) {
        return a.angle < b.angle;
    });

    const int n = V.size();
    const int rayCount = hits.size();

    QList<SplitRegion> regions;
    regions.reserve(rayCount);

    for (int i = 0; i < rayCount; ++i) {
        const RayHit& a = hits[i];
        const RayHit& b = hits[(i + 1) % rayCount];

        // Wedge = center -> exit(a) -> (real boundary vertices a..b) -> exit(b).
        QList<QPointF> wedge;
        wedge.append(center);
        wedge.append(a.point);

        int k = a.edgeIndex;
        while (k != b.edgeIndex) {
            k = (k + 1) % n;
            wedge.append(V[k]);
        }
        wedge.append(b.point);

        wedge = dedupe(wedge);
        if (wedge.size() < 3 || std::abs(signedArea(wedge)) < 1.0 /* m^2 */) {
            continue;
        }

        // Back to geographic coordinates.
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
