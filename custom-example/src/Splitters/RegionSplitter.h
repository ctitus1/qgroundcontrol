/****************************************************************************
 *
 * (c) 2009-2024 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

#pragma once

#include <QtCore/QList>
#include <QtCore/QString>
#include <QtPositioning/QGeoCoordinate>

/*
 * RegionSplitter
 *
 * Decoupled, swappable strategy for dividing one survey polygon into
 * N sub-region polygons. Implementations are PURE GEOMETRY: they take a
 * polygon plus a set of control points and return sub-polygons. They must
 * have NO dependency on QGC UI, mission items, or the manager, so a new
 * splitter can be dropped into this folder and selected in ActiveSplitter.h
 * by changing a single line, without touching the rest of the system.
 */

/// Input to a split operation. All coordinates are geographic (WGS84).
struct SplitInput {
    QList<QGeoCoordinate>   masterPolygon;      ///< The traced survey area polygon (>= 3 vertices)
    QGeoCoordinate          center;             ///< Interior control point the division radiates from
    QList<QGeoCoordinate>   edgePoints;         ///< Draggable control points; one region per point
    double                  regionSeparation = 0.0; ///< Inset each region inward from its shared (ray) edges by this many meters; the survey-boundary edges are NOT inset. 0 = touching.
};

/// A single generated sub-region.
struct SplitRegion {
    QList<QGeoCoordinate>   polygon;        ///< Sub-region boundary (geographic)
};

/// Abstract splitter strategy. See ControlPointSplitter for the default.
class RegionSplitter
{
public:
    virtual ~RegionSplitter() = default;

    /// Divide the master polygon into sub-regions.
    ///     @param input        Master polygon + control points
    ///     @param errorString  Set to a user-facing message on failure (return is then empty)
    /// @return One polygon per region; empty on error.
    virtual QList<SplitRegion> split(const SplitInput& input, QString& errorString) const = 0;

    /// Human readable name of this strategy (for logging / diagnostics).
    virtual QString name() const = 0;
};
