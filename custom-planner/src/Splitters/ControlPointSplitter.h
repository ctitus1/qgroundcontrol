/****************************************************************************
 *
 * (c) 2009-2024 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

#pragma once

#include "RegionSplitter.h"

/*
 * ControlPointSplitter
 *
 * Default region splitter. The user positions one interior "center" control
 * point plus a ring of "edge" control points. A ray is cast from the center
 * through each edge point to its exit on the polygon boundary; the polygon is
 * then partitioned into fan wedges between consecutive rays, each wedge closed
 * by walking the real polygon boundary between the two exits. N edge points
 * therefore produce N sub-regions, fully covering a star-shaped polygon.
 *
 * The algorithm is intentionally self-contained (only QGCGeo is reused for the
 * geo<->local planar transform) so it can be unit tested and replaced freely.
 */
class ControlPointSplitter : public RegionSplitter
{
public:
    QList<SplitRegion> split(const SplitInput& input, QString& errorString) const override;
    QString name() const override { return QStringLiteral("control-point"); }
};
