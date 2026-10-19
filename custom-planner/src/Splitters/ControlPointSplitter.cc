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

// Keep the part of `poly` on the inward side of the line through `linePoint`
// with unit inward normal `nrm`. Sutherland-Hodgman half-plane clip — robust at
// any orientation (never flips or overshoots).
QList<QPointF> clipByHalfPlane(const QList<QPointF>& poly, const QPointF& linePoint, const QPointF& nrm)
{
    QList<QPointF> out;
    const int m = poly.size();
    if (m < 3) {
        return out;
    }
    auto side = [&](const QPointF& p) {
        return ((p.x() - linePoint.x()) * nrm.x()) + ((p.y() - linePoint.y()) * nrm.y());
    };
    QPointF s = poly.last();
    double ss = side(s);
    for (const QPointF& e : poly) {
        const double se = side(e);
        if (se >= 0.0) {
            if (ss < 0.0) {
                const double t = ss / (ss - se);
                out.append(QPointF(s.x() + (t * (e.x() - s.x())), s.y() + (t * (e.y() - s.y()))));
            }
            out.append(e);
        } else if (ss >= 0.0) {
            const double t = ss / (ss - se);
            out.append(QPointF(s.x() + (t * (e.x() - s.x())), s.y() + (t * (e.y() - s.y()))));
        }
        s = e;
        ss = se;
    }
    return out;
}

// Intersect line (o1 + t*dir1) with line (o2 + s*dir2). false if parallel.
bool lineIntersect(const QPointF& o1, const QPointF& dir1, const QPointF& o2, const QPointF& dir2, QPointF& out)
{
    const double denom = (dir1.x() * dir2.y()) - (dir1.y() * dir2.x());
    if (std::abs(denom) < 1e-12) {
        return false;
    }
    const QPointF diff = o2 - o1;
    const double t = ((diff.x() * dir2.y()) - (diff.y() * dir2.x())) / denom;
    out = QPointF(o1.x() + (t * dir1.x()), o1.y() + (t * dir1.y()));
    return true;
}

// A wedge is [center, cutA, boundary..., cutB], with the boundary walked CCW so
// the region occupies the CCW sweep from ray A (center->cutA) to ray B
// (center->cutB). Separate it from its neighbors by pulling ONLY its two ray
// edges inward by `d`, leaving the survey-boundary edges in place.
//
// We offset each ray *line* inward and reconnect (apex = the two offset lines'
// intersection; each cut slides along its boundary edge). Deriving the inward
// normals from the CCW sweep (+90 deg off ray A, -90 deg off ray B) keeps the
// apex on the correct side even when the sweep exceeds 180 deg — a half-plane
// clip would instead eat the far side of the region "past the center".
QList<QPointF> insetRayEdges(const QList<QPointF>& wedge, double d)
{
    const int n = wedge.size();
    if (n < 3 || d <= 0.0) {
        return wedge;
    }

    const QPointF C = wedge[0];
    auto unit = [](const QPointF& v) {
        const double len = std::hypot(v.x(), v.y());
        return (len > 1e-9) ? QPointF(v.x() / len, v.y() / len) : QPointF(0.0, 0.0);
    };
    const QPointF ua = unit(wedge[1] - C);       // ray A direction (center -> cutA)
    const QPointF ub = unit(wedge[n - 1] - C);   // ray B direction (center -> cutB)

    const QPointF na(-ua.y(), ua.x());   // +90 deg into the CCW sweep, off ray A
    const QPointF nb(ub.y(), -ub.x());   // -90 deg into the CCW sweep, off ray B
    const QPointF pa(C.x() + (d * na.x()), C.y() + (d * na.y()));   // point on offset ray-A line
    const QPointF pb(C.x() + (d * nb.x()), C.y() + (d * nb.y()));   // point on offset ray-B line

    QPointF apex, newA, newB;
    if (lineIntersect(pa, ua, pb, ub, apex) &&                                    // inset apex
        lineIntersect(pa, ua, wedge[1], wedge[2] - wedge[1], newA) &&             // ray A x first boundary edge
        lineIntersect(pb, ub, wedge[n - 2], wedge[n - 1] - wedge[n - 2], newB)) { // ray B x last boundary edge
        QList<QPointF> out;
        out.append(apex);
        out.append(newA);
        for (int i = 2; i <= n - 2; ++i) {
            out.append(wedge[i]);
        }
        out.append(newB);
        out = dedupe(out);
        const double a0 = signedArea(wedge);
        const double a1 = signedArea(out);
        if (out.size() >= 3 && (a0 > 0.0) == (a1 > 0.0) && std::abs(a1) >= 1.0) {
            return out;
        }
    }

    // Degenerate primary (rays ~colinear => ~180 deg straight divider). Shifting
    // a straight divider is a single half-plane clip and is safe for a convex
    // sweep; for a reflex sweep, clipping would eat the far side, so leave the
    // region un-inset instead.
    const double crossAB = (ua.x() * ub.y()) - (ua.y() * ub.x());   // >0: sweep<180, <0: >180
    if (crossAB < 0.0) {
        return wedge;
    }
    QList<QPointF> clipped = clipByHalfPlane(wedge, pa, na);
    clipped = clipByHalfPlane(clipped, pb, nb);
    return dedupe(clipped);
}

// True if the simple polygon is convex (all turns the same way).
bool isConvex(const QList<QPointF>& poly)
{
    const int m = poly.size();
    if (m < 3) {
        return false;
    }
    int sign = 0;
    for (int i = 0; i < m; ++i) {
        const QPointF u = poly[(i + 1) % m] - poly[i];
        const QPointF v = poly[(i + 2) % m] - poly[(i + 1) % m];
        const double cross = (u.x() * v.y()) - (u.y() * v.x());
        if (std::abs(cross) < 1e-9) {
            continue;   // collinear vertex
        }
        const int s = (cross > 0.0) ? 1 : -1;
        if (sign == 0) {
            sign = s;
        } else if (s != sign) {
            return false;
        }
    }
    return true;
}

// Clip `subject` to the interior of a CONVEX, CCW-wound polygon `clip`
// (Sutherland-Hodgman against every clip edge). Guarantees the result never
// extends outside `clip`. Works for an arbitrary (even reflex) subject.
QList<QPointF> clipToConvexPolygon(QList<QPointF> subject, const QList<QPointF>& clip)
{
    const int m = clip.size();
    for (int j = 0; j < m && subject.size() >= 3; ++j) {
        const QPointF a = clip[j];
        const QPointF edge = clip[(j + 1) % m] - a;
        subject = clipByHalfPlane(subject, a, QPointF(-edge.y(), edge.x()));   // left = interior for CCW
    }
    return subject;
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
    // Offsetting can push the reconnected vertices onto extended edges, i.e.
    // outside the survey area. Clip each offset region back to the master so it
    // can never exceed it. Only valid (exact) for a convex master.
    const bool masterConvex = isConvex(V);

    QList<SplitRegion> regions;
    regions.reserve(cutCount);

    for (int i = 0; i < cutCount; ++i) {
        const Cut& a = cuts[i];
        const Cut& b = cuts[(i + 1) % cutCount];

        // Wedge = center -> cut(a) -> boundary vertices on the CCW arc a..b -> cut(b).
        QList<QPointF> wedge;
        wedge.append(center);
        wedge.append(a.point);

        // Number of boundary vertices to append walking CCW from a to b. The
        // final (wrap) region has a AFTER b in perimeter order; when a and b fall
        // on the SAME segment that wrap must still traverse the entire boundary
        // rather than collapse to a sliver (the "last region flips" bug).
        const bool wrap = a.perimPos > b.perimPos;
        int steps = (((b.segment - a.segment) % n) + n) % n;
        if (wrap && steps == 0) {
            steps = n;
        }
        for (int s = 1; s <= steps; ++s) {
            wedge.append(V[(a.segment + s) % n]);
        }
        wedge.append(b.point);

        wedge = dedupe(wedge);
        if (input.regionSeparation > 0.0) {
            wedge = insetRayEdges(wedge, input.regionSeparation);
            if (masterConvex) {
                wedge = dedupe(clipToConvexPolygon(wedge, V));
            }
        }
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
