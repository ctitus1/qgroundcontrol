/****************************************************************************
 *
 * (c) 2009-2024 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

#include "CustomSurveyManager.h"

#include "CameraCalc.h"
#include "Fact.h"
#include "FactMetaData.h"
#include "JsonHelper.h"
#include "MissionController.h"
#include "MissionItem.h"
#include "PlanMasterController.h"
#include "QGCGeo.h"
#include "QGCMAVLink.h"
#include "QGCMapPolygon.h"
#include "QGroundControlQmlGlobal.h"
#include "QmlObjectListModel.h"
#include "SurveyComplexItem.h"
#include "VisualMissionItem.h"

#include <QtCore/QDir>
#include <QtCore/QFile>
#include <QtCore/QFileInfo>
#include <QtCore/QJsonArray>
#include <QtCore/QJsonDocument>
#include <QtCore/QPointF>
#include <QtCore/QStringList>

#include <cmath>
#include <limits>

CustomSurveyManager::CustomSurveyManager(QObject* parent)
    : QObject(parent)
{
    _createCaptureFacts();
}

//=============================================================================
// Capture-setting Facts (individual-waypoint export)
//=============================================================================

void CustomSurveyManager::_createCaptureFacts()
{
    // Gimbal pitch mirrors CameraSection's "GimbalPitch": the "gimbal-degrees"
    // unit installs FactMetaData's built-in translator so the field shows +90 for
    // straight-down (cooked) while rawValue()/the plan store -90 (raw). Reusing
    // the unit reproduces that exact behavior with no manual sign flip.
    FactMetaData* pitchMeta = new FactMetaData(FactMetaData::valueTypeDouble, QStringLiteral("GimbalPitch"), this);
    pitchMeta->setRawUnits(QStringLiteral("gimbal-degrees"));
    pitchMeta->setRawMin(-90.0);
    pitchMeta->setRawMax(0.0);
    pitchMeta->setDecimalPlaces(0);
    _captureGimbalPitchFact = new Fact(0, QStringLiteral("GimbalPitch"), FactMetaData::valueTypeDouble, this);
    _captureGimbalPitchFact->setMetaData(pitchMeta);
    _captureGimbalPitchFact->setRawValue(-90.0);   // UI shows +90 == straight down

    FactMetaData* yawMeta = new FactMetaData(FactMetaData::valueTypeDouble, QStringLiteral("GimbalYaw"), this);
    yawMeta->setRawUnits(QStringLiteral("deg"));
    yawMeta->setRawMin(-180.0);
    yawMeta->setRawMax(180.0);
    yawMeta->setDecimalPlaces(0);
    _captureGimbalYawFact = new Fact(0, QStringLiteral("GimbalYaw"), FactMetaData::valueTypeDouble, this);
    _captureGimbalYawFact->setMetaData(yawMeta);
    _captureGimbalYawFact->setRawValue(0.0);

    FactMetaData* holdMeta = new FactMetaData(FactMetaData::valueTypeDouble, QStringLiteral("Hold"), this);
    holdMeta->setRawUnits(QStringLiteral("secs"));
    holdMeta->setRawMin(0.0);
    holdMeta->setDecimalPlaces(0);
    _captureHoldFact = new Fact(0, QStringLiteral("Hold"), FactMetaData::valueTypeDouble, this);
    _captureHoldFact->setMetaData(holdMeta);
    _captureHoldFact->setRawValue(2.0);

    // Editing any capture value dirties open custom-survey plans so it persists.
    connect(_captureGimbalPitchFact, &Fact::rawValueChanged, this, [this](const QVariant&) { _markAllCustomSurveysDirty(); });
    connect(_captureGimbalYawFact,   &Fact::rawValueChanged, this, [this](const QVariant&) { _markAllCustomSurveysDirty(); });
    connect(_captureHoldFact,        &Fact::rawValueChanged, this, [this](const QVariant&) { _markAllCustomSurveysDirty(); });
}

void CustomSurveyManager::_markAllCustomSurveysDirty()
{
    if (_suppressCaptureDirty) {
        return;     // restoring capture facts from a plan load must not dirty the plan
    }
    for (QObject* item : std::as_const(_customSurveyItems)) {
        if (PlanMasterController* controller = _itemController(item)) {
            controller->setDirty(true);
        }
    }
}

bool CustomSurveyManager::exportAsWaypoints(QObject* item)
{
    if (!_surveyItem(item)) {
        return false;
    }
    return _stateFor(item).exportAsWaypoints;
}

void CustomSurveyManager::setExportAsWaypoints(QObject* item, bool enabled)
{
    if (!_surveyItem(item)) {
        return;
    }
    ControlState& state = _stateFor(item);
    if (state.exportAsWaypoints == enabled) {
        return;
    }
    state.exportAsWaypoints = enabled;
    if (PlanMasterController* controller = _itemController(item)) {
        controller->setDirty(true);
    }
    emit customSurveyChanged(item);
}

//=============================================================================
// Identity helpers
//=============================================================================

SurveyComplexItem* CustomSurveyManager::_surveyItem(QObject* item) const
{
    return qobject_cast<SurveyComplexItem*>(item);
}

VisualMissionItem* CustomSurveyManager::_visualItem(QObject* item) const
{
    return qobject_cast<VisualMissionItem*>(item);
}

PlanMasterController* CustomSurveyManager::_itemController(QObject* item) const
{
    if (VisualMissionItem* visual = _visualItem(item)) {
        return visual->masterController();
    }
    return nullptr;
}

int CustomSurveyManager::_sequenceNumber(QObject* item) const
{
    if (VisualMissionItem* visual = _visualItem(item)) {
        return visual->sequenceNumber();
    }
    return -1;
}

//=============================================================================
// Ray -> boundary cut
//=============================================================================

QGeoCoordinate CustomSurveyManager::_rayBoundaryIntersection(const QList<QGeoCoordinate>& polygon,
                                                             const QGeoCoordinate& center,
                                                             double azimuthDeg)
{
    const int n = polygon.size();
    if (n < 3 || !center.isValid()) {
        return QGeoCoordinate();
    }

    const QGeoCoordinate origin = polygon.first();
    auto toLocal = [&origin](const QGeoCoordinate& c) {
        double north, east, down;
        QGCGeo::convertGeoToNed(c, origin, north, east, down);
        return QPointF(east, north);
    };

    QList<QPointF> V;
    V.reserve(n);
    for (const QGeoCoordinate& c : polygon) {
        V.append(toLocal(c));
    }
    const QPointF C = toLocal(center);

    // Bearing: 0 = north, clockwise. In (east, north): (sin, cos).
    const double rad = azimuthDeg * M_PI / 180.0;
    const QPointF dir(std::sin(rad), std::cos(rad));

    double bestT = std::numeric_limits<double>::max();
    QPointF bestPoint;
    bool found = false;
    for (int j = 0; j < n; ++j) {
        const QPointF a = V[j];
        const QPointF ab = V[(j + 1) % n] - a;
        const double denom = (dir.x() * ab.y()) - (dir.y() * ab.x());   // cross(dir, ab)
        if (std::abs(denom) < 1e-12) {
            continue;   // parallel
        }
        const QPointF ac = a - C;
        const double t = ((ac.x() * ab.y()) - (ac.y() * ab.x())) / denom;   // distance along ray
        const double u = ((ac.x() * dir.y()) - (ac.y() * dir.x())) / denom; // position along edge
        if (t > 1e-6 && u >= -1e-6 && u <= 1.0 + 1e-6 && t < bestT) {
            bestT = t;
            bestPoint = C + (t * dir);
            found = true;
        }
    }
    if (!found) {
        return QGeoCoordinate();
    }

    QGeoCoordinate cut;
    QGCGeo::convertNedToGeo(bestPoint.y() /*north*/, bestPoint.x() /*east*/, 0.0, origin, cut);
    return cut;
}

//=============================================================================
// State access
//=============================================================================

void CustomSurveyManager::_attachSurvey(QObject* survey)
{
    if (!survey || _stateBySurvey.contains(survey)) {
        return;
    }

    ControlState state;
    const int seq = _sequenceNumber(survey);
    if (_pendingState.contains(seq)) {
        state = _pendingState.take(seq);
    }
    _stateBySurvey.insert(survey, state);
    // If this survey carried restored control points, snap them to their ray
    // midpoints now that the polygon exists (no-op for a fresh, untraced survey).
    _snapEdgeVerticesToMidpoints(survey, _stateBySurvey[survey]);

    connect(survey, &QObject::destroyed, this, [this](QObject* obj) {
        _stateBySurvey.remove(obj);
        _customSurveyItems.remove(obj);
        _lastParamSig.remove(obj);
        _cachedRegions.remove(obj);
        _cachedRegionSig.remove(obj);
        const QList<SurveyComplexItem*> shadows = _regionSurveys.take(obj);
        for (SurveyComplexItem* shadow : shadows) {
            if (shadow) {
                shadow->deleteLater();
            }
        }
    });
}

CustomSurveyManager::ControlState& CustomSurveyManager::_stateFor(QObject* item)
{
    _attachSurvey(item);
    return _stateBySurvey[item];
}

bool CustomSurveyManager::markCustomSurvey(QObject* item)
{
    return _markCustomSurvey(item, true);
}

bool CustomSurveyManager::_markCustomSurvey(QObject* item, bool setDirty)
{
    SurveyComplexItem* survey = _surveyItem(item);
    if (!survey) {
        _setLastError(tr("Only Survey items can become custom surveys."));
        return false;
    }

    if (!_customSurveyItems.contains(survey)) {
        _customSurveyItems.insert(survey);
        _attachSurvey(survey);

        // Follow whole-survey moves/edits by translating the center and control
        // vertices by the survey polygon's centroid delta. We listen to
        // pathChanged (not centerChanged): during a center-drag translation
        // QGCMapPolygon sets _center to the drag target itself, so the recomputed
        // centroid matches and centerChanged is suppressed — but pathChanged
        // still fires, so it's the reliable trigger for both moves and reshapes.
        if (QGCMapPolygon* polygon = survey->surveyAreaPolygon()) {
            _stateBySurvey[survey].lastPolygonCenter = polygon->center();
            connect(polygon, &QGCMapPolygon::pathChanged, this, [this, survey, polygon]() {
                // Core-bug workaround (no source mods): QGCMapPolygon::setCenter
                // gates its centerChanged emit behind the SAME _deferredPathChanged
                // flag it already raised for pathChanged, so during a center drag
                // centerChanged never fires. The core centroid marker binds
                // `coordinate: mapPolygon.center`, so its binding freezes even
                // though center() is already up to date. pathChanged DOES fire, so
                // re-emit centerChanged here to drive the real marker's binding.
                if (polygon->centerDrag()) {
                    QMetaObject::invokeMethod(polygon, "centerChanged",
                                              Q_ARG(QGeoCoordinate, polygon->center()));
                }
                _onSurveyPolygonMoved(survey);
            });
        }

        emit customSurveyChanged(survey);
    }

    if (setDirty) {
        if (PlanMasterController* controller = _itemController(survey)) {
            controller->setDirty(true);
        }
    }
    return true;
}

bool CustomSurveyManager::isCustomSurvey(QObject* item) const
{
    return item && _customSurveyItems.contains(_surveyItem(item));
}

void CustomSurveyManager::_onSurveyPolygonMoved(QObject* survey)
{
    if (!_stateBySurvey.contains(survey)) {
        return;
    }
    SurveyComplexItem* surveyItem = _surveyItem(survey);
    if (!surveyItem) {
        return;
    }
    const QGeoCoordinate newCenter = surveyItem->surveyAreaPolygon()->center();
    if (!newCenter.isValid()) {
        return;
    }

    ControlState& state = _stateBySurvey[survey];
    const QGeoCoordinate oldCenter = state.lastPolygonCenter;
    state.lastPolygonCenter = newCenter;

    // Only a rigid whole-survey CENTER drag carries the control points along.
    // Any OTHER polygon edit (a vertex reshape/resize, KML load, ...) must leave
    // the center and every control vertex exactly where they are: the rays just
    // re-project through the (fixed) vertices onto the new boundary, so the
    // regions update but the handles do not move. So recompute and return
    // without translating anything.
    if (!surveyItem->surveyAreaPolygon()->centerDrag()) {
        emit customSurveyChanged(survey);
        return;
    }

    if (!oldCenter.isValid()) {
        return;                 // first observation: nothing to translate against yet
    }
    const double distance = oldCenter.distanceTo(newCenter);
    if (distance < 0.01) {
        return;                 // negligible movement
    }

    // Rigid translation: shift the center AND every control vertex by the same
    // geodesic delta the polygon applied to its own vertices, so the whole
    // division rides along. The regions/transects then shift by exactly this
    // delta too — signal the visuals to translate the already-built lines
    // (cheap) rather than recompute.
    const double azimuth = oldCenter.azimuthTo(newCenter);
    if (state.center.isValid()) {
        state.center = state.center.atDistanceAndAzimuth(distance, azimuth);
    }
    for (int i = 0; i < state.edgeVertices.size(); ++i) {
        if (state.edgeVertices[i].isValid()) {
            state.edgeVertices[i] = state.edgeVertices[i].atDistanceAndAzimuth(distance, azimuth);
        }
    }
    emit customSurveyTranslated(survey, distance, azimuth);
}

//=============================================================================
// Control points (rays)
//=============================================================================

void CustomSurveyManager::_seedControlPoints(QObject* item, ControlState& state, int count)
{
    SurveyComplexItem* survey = _surveyItem(item);
    if (!survey) {
        return;
    }
    if (survey->surveyAreaPolygon()->coordinateList().size() < 3) {
        return;
    }

    if (!state.center.isValid()) {
        state.center = survey->surveyAreaPolygon()->center();
    }
    const QGeoCoordinate center = state.center;
    const QList<QGeoCoordinate> polygon = survey->surveyAreaPolygon()->coordinateList();

    // Place a control vertex at the midpoint of each evenly-spaced ray.
    state.edgeVertices.clear();
    for (int i = 0; i < count; ++i) {
        const double azimuth = (360.0 * i) / static_cast<double>(count);
        const QGeoCoordinate cut = _rayBoundaryIntersection(polygon, center, azimuth);
        if (cut.isValid()) {
            state.edgeVertices.append(center.atDistanceAndAzimuth(center.distanceTo(cut) * 0.5, azimuth));
        } else {
            state.edgeVertices.append(center);
        }
    }
    state.regionCount = count;
    state.seeded = true;
    state.lastPolygonCenter = survey->surveyAreaPolygon()->center();
}

void CustomSurveyManager::_snapEdgeVerticesToMidpoints(QObject* item, ControlState& state)
{
    SurveyComplexItem* survey = _surveyItem(item);
    if (!survey) {
        return;
    }
    const QList<QGeoCoordinate> polygon = survey->surveyAreaPolygon()->coordinateList();
    if (polygon.size() < 3 || !state.center.isValid()) {
        return;
    }
    // Only the ray AZIMUTH defines a region cut (see _computeRegions), so moving
    // each vertex along its own ray to the center<->boundary midpoint leaves the
    // division untouched while putting the handle back where the user expects it.
    for (int i = 0; i < state.edgeVertices.size(); ++i) {
        if (!state.edgeVertices[i].isValid()) {
            continue;
        }
        const double azimuth = state.center.azimuthTo(state.edgeVertices[i]);
        const QGeoCoordinate cut = _rayBoundaryIntersection(polygon, state.center, azimuth);
        if (cut.isValid()) {
            state.edgeVertices[i] = state.center.atDistanceAndAzimuth(state.center.distanceTo(cut) * 0.5, azimuth);
        }
    }
}

int CustomSurveyManager::regionCount(QObject* item)
{
    if (!_surveyItem(item)) {
        return 1;
    }
    return _stateFor(item).regionCount;
}

void CustomSurveyManager::setRegionCount(QObject* item, int count)
{
    if (!_surveyItem(item)) {
        return;
    }
    count = qBound(1, count, 64);
    ControlState& state = _stateFor(item);
    state.regionCount = count;
    if (count < 2) {
        state.edgeVertices.clear();
        state.seeded = true;   // undivided: nothing to seed
    } else {
        _seedControlPoints(item, state, count);
    }
    if (PlanMasterController* controller = _itemController(item)) {
        controller->setDirty(true);
    }
    emit customSurveyChanged(item);
}

QGeoCoordinate CustomSurveyManager::centerControlPoint(QObject* item)
{
    SurveyComplexItem* survey = _surveyItem(item);
    if (!survey) {
        return QGeoCoordinate();
    }
    ControlState& state = _stateFor(item);
    if (!state.center.isValid()) {
        state.center = survey->surveyAreaPolygon()->center();
    }
    return state.center;
}

QVariantList CustomSurveyManager::edgeControlPoints(QObject* item)
{
    QVariantList out;
    SurveyComplexItem* survey = _surveyItem(item);
    if (!survey) {
        return out;
    }
    ControlState& state = _stateFor(item);
    if (state.regionCount >= 2 && (!state.seeded || state.edgeVertices.size() != state.regionCount)) {
        _seedControlPoints(item, state, state.regionCount);
    }
    // Control vertices are stored as absolute positions, so they stay put when
    // the center is dragged (the rays re-project through them).
    for (const QGeoCoordinate& vertex : std::as_const(state.edgeVertices)) {
        out << QVariant::fromValue(vertex);
    }
    return out;
}

double CustomSurveyManager::regionOffset(QObject* item)
{
    if (!_surveyItem(item)) {
        return 0.0;
    }
    return _stateFor(item).regionOffset;
}

void CustomSurveyManager::setRegionOffset(QObject* item, double meters)
{
    if (!_surveyItem(item)) {
        return;
    }
    if (!(meters >= 0.0)) {   // clamp negatives and NaN to 0
        meters = 0.0;
    }
    ControlState& state = _stateFor(item);
    state.regionOffset = meters;
    if (PlanMasterController* controller = _itemController(item)) {
        controller->setDirty(true);
    }
    emit customSurveyChanged(item);
}

void CustomSurveyManager::setCenterControlPoint(QObject* item, const QGeoCoordinate& coordinate)
{
    if (!_surveyItem(item) || !coordinate.isValid()) {
        return;
    }
    ControlState& state = _stateFor(item);
    state.center = coordinate;
    state.seeded = true;
    if (PlanMasterController* controller = _itemController(item)) {
        controller->setDirty(true);
    }
    emit customSurveyChanged(item);
}

void CustomSurveyManager::setEdgeControlPoint(QObject* item, int index, const QGeoCoordinate& coordinate)
{
    SurveyComplexItem* survey = _surveyItem(item);
    if (!survey || !coordinate.isValid()) {
        return;
    }
    ControlState& state = _stateFor(item);
    if (index < 0 || index >= state.edgeVertices.size()) {
        return;
    }
    const QGeoCoordinate center = state.center.isValid() ? state.center : survey->surveyAreaPolygon()->center();
    if (!center.isValid()) {
        return;
    }
    // The dragged handle defines the ray direction; snap the vertex to the
    // midpoint between the center and where that ray meets the boundary.
    const double azimuth = center.azimuthTo(coordinate);
    const QList<QGeoCoordinate> polygon = survey->surveyAreaPolygon()->coordinateList();
    const QGeoCoordinate cut = _rayBoundaryIntersection(polygon, center, azimuth);
    state.edgeVertices[index] = cut.isValid()
        ? center.atDistanceAndAzimuth(center.distanceTo(cut) * 0.5, azimuth)
        : coordinate;
    state.seeded = true;
    if (PlanMasterController* controller = _itemController(item)) {
        controller->setDirty(true);
    }
    emit customSurveyChanged(item);
}

//=============================================================================
// Region generation
//=============================================================================

QList<SplitRegion> CustomSurveyManager::_computeRegions(QObject* item, QString& errorString)
{
    errorString.clear();

    SurveyComplexItem* survey = _surveyItem(item);
    if (!survey) {
        errorString = tr("Invalid survey.");
        return {};
    }

    const QList<QGeoCoordinate> polygon = survey->surveyAreaPolygon()->coordinateList();
    if (polygon.size() < 3) {
        errorString = tr("Trace the survey polygon first.");
        return {};
    }

    ControlState& state = _stateFor(item);
    if (state.regionCount < 2) {
        SplitRegion whole;
        whole.polygon = polygon;
        return { whole };
    }

    if (!state.seeded || state.edgeVertices.size() != state.regionCount) {
        _seedControlPoints(item, state, state.regionCount);
    }

    const QGeoCoordinate center = state.center.isValid() ? state.center : survey->surveyAreaPolygon()->center();

    // Memoization: the split is comparatively expensive (ray casts + wedge walk
    // + inset + clip) and every live frame asks for regions from several places
    // (map overlays, per-region flight paths, the editor's region list). Build a
    // signature of everything the split depends on; return the cached result
    // unless one of those inputs actually changed.
    QStringList sigParts;
    sigParts << QString::number(state.regionCount)
             << QString::number(state.regionOffset, 'f', 3);
    for (const QGeoCoordinate& c : polygon) {
        sigParts << QString::number(c.latitude(), 'f', 9) << QString::number(c.longitude(), 'f', 9);
    }
    sigParts << QString::number(center.latitude(), 'f', 9) << QString::number(center.longitude(), 'f', 9);
    for (const QGeoCoordinate& v : std::as_const(state.edgeVertices)) {
        sigParts << QString::number(v.latitude(), 'f', 9) << QString::number(v.longitude(), 'f', 9);
    }
    const QString sig = sigParts.join(QLatin1Char('|'));
    if (_cachedRegionSig.value(item) == sig && _cachedRegions.contains(item)) {
        return _cachedRegions.value(item);
    }

    // Cut = where the ray from the center through each (fixed) control vertex
    // meets the boundary.
    SplitInput input;
    input.masterPolygon    = polygon;
    input.center           = center;
    input.regionSeparation = state.regionOffset;
    for (const QGeoCoordinate& vertex : std::as_const(state.edgeVertices)) {
        const QGeoCoordinate cut = _rayBoundaryIntersection(polygon, center, center.azimuthTo(vertex));
        if (cut.isValid()) {
            input.edgePoints.append(cut);
        }
    }

    const QList<SplitRegion> regions = _splitter.split(input, errorString);
    _cachedRegionSig[item] = sig;
    _cachedRegions[item]   = regions;
    return regions;
}

QString CustomSurveyManager::_masterParamSignature(SurveyComplexItem* survey) const
{
    if (!survey) {
        return QString();
    }
    CameraCalc* cc = survey->cameraCalc();
    const QStringList parts = {
        // Survey-level params. Grid angle is deliberately EXCLUDED — it is
        // mirrored to the shadows directly on every sync (cheap), so dragging
        // the angle slider never triggers the heavy save()/load() reconfigure.
        survey->flyAlternateTransects()->rawValue().toString(),
        survey->splitConcavePolygons()->rawValue().toString(),
        // Transect-style params.
        survey->turnAroundDistance()->rawValue().toString(),
        survey->cameraTriggerInTurnAround()->rawValue().toString(),
        survey->hoverAndCapture()->rawValue().toString(),
        survey->refly90Degrees()->rawValue().toString(),
        survey->terrainAdjustTolerance()->rawValue().toString(),
        survey->terrainAdjustMaxDescentRate()->rawValue().toString(),
        survey->terrainAdjustMaxClimbRate()->rawValue().toString(),
        // Camera calc: the adjusted footprints capture the net transect spacing
        // regardless of which input (overlap / distance / camera) produced it.
        QString::number(static_cast<int>(cc->distanceMode())),
        cc->adjustedFootprintSide()->rawValue().toString(),
        cc->adjustedFootprintFrontal()->rawValue().toString(),
        cc->distanceToSurface()->rawValue().toString(),
        cc->valueSetIsDistance()->rawValue().toString(),
    };
    return parts.join(QLatin1Char('|'));
}

QVariantList CustomSurveyManager::regionPolygons(QObject* item)
{
    QString errorString;
    const QList<SplitRegion> regions = _computeRegions(item, errorString);
    _setLastError(errorString);

    QVariantList out;
    for (int i = 0; i < regions.size(); ++i) {
        QVariantMap map;
        map[QStringLiteral("name")]        = tr("Region %1").arg(i + 1);
        map[QStringLiteral("vertexCount")] = regions[i].polygon.size();
        map[QStringLiteral("area")]        = _polygonArea(regions[i].polygon);
        map[QStringLiteral("polygon")]     = _coordinatesToVariantList(regions[i].polygon);
        out << map;
    }
    return out;
}

void CustomSurveyManager::_syncRegionSurveys(QObject* master, const QList<SplitRegion>& regions)
{
    SurveyComplexItem* masterSurvey = _surveyItem(master);
    PlanMasterController* controller = _itemController(master);
    if (!masterSurvey || !controller) {
        return;
    }

    QList<SurveyComplexItem*>& shadows = _regionSurveys[master];

    // Undivided: no per-region shadows.
    if (regions.size() < 2) {
        for (SurveyComplexItem* shadow : std::as_const(shadows)) {
            if (shadow) {
                shadow->deleteLater();
            }
        }
        shadows.clear();
        _lastParamSig.remove(master);
        return;
    }

    // Grow / shrink the shadow list to match the region count.
    const int oldCount = shadows.size();
    while (shadows.size() < regions.size()) {
        shadows.append(new SurveyComplexItem(controller, /*flyView*/ false, QString()));
    }
    while (shadows.size() > regions.size()) {
        if (SurveyComplexItem* extra = shadows.takeLast()) {
            extra->deleteLater();
        }
    }

    // Reconfigure shadow PARAMETERS only on a genuine parameter edit (or for a
    // newly created shadow). save() regenerates every mission item and load()
    // reparses it, so doing this per frame is what made dragging laggy. The
    // parameter signature excludes the grid angle and the polygon, so neither a
    // control-point / whole-survey drag nor an angle-slider drag lands here.
    const QString paramSig      = _masterParamSignature(masterSurvey);
    const bool    paramsChanged = (_lastParamSig.value(master) != paramSig);
    const bool    grew          = regions.size() > oldCount;
    if (paramsChanged || grew) {
        QJsonArray masterArray;
        masterSurvey->save(masterArray);
        const QJsonObject masterSurveyJson = masterArray.isEmpty() ? QJsonObject() : masterArray.first().toObject();
        for (int i = 0; i < shadows.size(); ++i) {
            if (paramsChanged || i >= oldCount) {
                QString errorString;
                shadows[i]->load(masterSurveyJson, 1, errorString);
            }
        }
        _lastParamSig[master] = paramSig;
    }

    // Mirror the grid angle live. It is the one continuously-dragged parameter,
    // so rather than the heavy reconfigure path above we copy the single fact
    // directly (guarded, so an unchanged angle never forces a needless grid
    // rebuild). The shadow rebuilds its transects synchronously on the change.
    const QVariant masterAngle = masterSurvey->gridAngle()->rawValue();
    for (SurveyComplexItem* shadow : std::as_const(shadows)) {
        if (shadow->gridAngle()->rawValue() != masterAngle) {
            shadow->gridAngle()->setRawValue(masterAngle);
        }
    }

    // Set each region's sub-polygon, but only when it actually changed, so an
    // idle sync (or one where only some regions moved) never rebuilds a grid
    // needlessly. Setting the polygon rebuilds that region's transects
    // synchronously (for fixed-altitude modes).
    for (int i = 0; i < regions.size(); ++i) {
        QGCMapPolygon* polygon = shadows[i]->surveyAreaPolygon();
        if (polygon->coordinateList() != regions[i].polygon) {
            polygon->beginReset();
            polygon->clear();
            polygon->appendVertices(regions[i].polygon);
            polygon->endReset();
        }
    }
}

QVariantList CustomSurveyManager::regionFlightPaths(QObject* item)
{
    QString errorString;
    const QList<SplitRegion> regions = _computeRegions(item, errorString);
    _syncRegionSurveys(item, regions);

    QVariantList out;
    for (SurveyComplexItem* shadow : std::as_const(_regionSurveys[item])) {
        if (shadow) {
            out << shadow->property("visualTransectPoints");
        }
    }
    return out;
}

//=============================================================================
// Export
//=============================================================================

SurveyComplexItem* CustomSurveyManager::_buildRegionSurveyObject(PlanMasterController* controller,
                                                                 const QJsonObject& masterSurveyJson,
                                                                 const QList<QGeoCoordinate>& regionPolygon) const
{
    // Build the region's survey from a genuine, freshly-configured survey
    // object so its transects are recomputed for the sub-polygon. We never
    // touch the live survey and never block its signals (that was the bug in
    // the prior attempt which left every export with the parent's grid).
    SurveyComplexItem* temp = new SurveyComplexItem(controller, /*flyView*/ false, QString());

    QString errorString;
    if (!temp->load(masterSurveyJson, 1, errorString)) {
        temp->deleteLater();
        return nullptr;
    }

    QGCMapPolygon* polygon = temp->surveyAreaPolygon();
    polygon->beginReset();
    polygon->clear();
    polygon->appendVertices(regionPolygon);
    polygon->endReset();     // emits pathChanged -> synchronous transect rebuild

    return temp;
}

QJsonObject CustomSurveyManager::_buildRegionSurveyJson(PlanMasterController* controller,
                                                        const QJsonObject& masterSurveyJson,
                                                        const QList<QGeoCoordinate>& regionPolygon,
                                                        bool& terrainPending) const
{
    SurveyComplexItem* temp = _buildRegionSurveyObject(controller, masterSurveyJson, regionPolygon);
    if (!temp) {
        return {};
    }

    if (temp->readyForSaveState() != VisualMissionItem::ReadyForSave) {
        terrainPending = true;
    }

    QJsonArray items;
    temp->save(items);
    temp->deleteLater();

    return items.isEmpty() ? QJsonObject() : items.first().toObject();
}

//=============================================================================
// Individual-waypoint expansion
//=============================================================================

QJsonObject CustomSurveyManager::_missionItemToJson(const MissionItem& item, bool withAltitude) const
{
    QJsonObject obj;
    item.save(obj);   // type/command/frame/autoContinue/doJumpId/params[7] (NaN -> null)

    // Coordinate items also carry the altitude trio the way SimpleMissionItem::save()
    // writes it, so QGC shows the right altitude mode on reload.
    if (withAltitude) {
        int altMode;
        switch (item.frame()) {
        case MAV_FRAME_GLOBAL:              altMode = QGroundControlQmlGlobal::AltitudeModeAbsolute;     break;
        case MAV_FRAME_GLOBAL_TERRAIN_ALT:  altMode = QGroundControlQmlGlobal::AltitudeModeTerrainFrame; break;
        case MAV_FRAME_GLOBAL_RELATIVE_ALT:
        default:                            altMode = QGroundControlQmlGlobal::AltitudeModeRelative;     break;
        }
        obj[QStringLiteral("AltitudeMode")]        = altMode;
        obj[QStringLiteral("Altitude")]            = item.param7();
        obj[QStringLiteral("AMSLAltAboveTerrain")] = QJsonValue::Null;   // recomputed on load if terrain-relative
    }
    return obj;
}

QJsonArray CustomSurveyManager::_buildRegionWaypointItems(PlanMasterController* controller,
                                                          const QJsonObject& masterSurveyJson,
                                                          const QList<QGeoCoordinate>& regionPolygon,
                                                          double transectYawDeg,
                                                          int& photoPointCount) const
{
    photoPointCount = 0;
    QJsonArray out;

    // Build the region survey, but force Hover & Capture ON *before* the polygon
    // reset so the synchronous transect rebuild lays down one discrete waypoint at
    // every photo location (NAV_WAYPOINT immediately followed by IMAGE_START_CAPTURE).
    // This reuses QGC's own photo-point spacing (camera trigger distance) instead of
    // reimplementing survey geometry in the plugin.
    SurveyComplexItem* temp = new SurveyComplexItem(controller, /*flyView*/ false, QString());
    QString errorString;
    if (!temp->load(masterSurveyJson, 1, errorString)) {
        temp->deleteLater();
        return out;
    }
    temp->hoverAndCapture()->setRawValue(true);

    QGCMapPolygon* polygon = temp->surveyAreaPolygon();
    polygon->beginReset();
    polygon->clear();
    polygon->appendVertices(regionPolygon);
    polygon->endReset();     // synchronous transect rebuild, now including photo points

    // Enumerate the survey's generated mission items and pull out the photo points.
    QObject itemParent;
    QList<MissionItem*> genItems;
    temp->appendMissionItems(genItems, &itemParent);

    const double pitchRaw = _captureGimbalPitchFact->rawValue().toDouble();   // e.g. -90 (straight down)
    const double gimbalYaw = _captureGimbalYawFact->rawValue().toDouble();
    const double holdSecs  = _captureHoldFact->rawValue().toDouble();

    int seqNum = 1;   // planned-home is item 0; real items start at doJumpId 1
    for (int i = 0; i < genItems.count(); ++i) {
        const MissionItem* capture = genItems[i];
        if (capture->command() != MAV_CMD_IMAGE_START_CAPTURE) {
            continue;
        }
        // The photo location is the NAV_WAYPOINT immediately preceding the capture.
        if (i == 0 || genItems[i - 1]->command() != MAV_CMD_NAV_WAYPOINT) {
            continue;
        }
        const MissionItem* wp = genItems[i - 1];
        const MAV_FRAME frame = wp->frame();
        const double lat = wp->param5();
        const double lon = wp->param6();
        const double alt = wp->param7();

        // 1. Arrive: NAV_WAYPOINT, yaw = transect angle, hold 0.
        MissionItem arrive(seqNum++, MAV_CMD_NAV_WAYPOINT, frame,
                           0.0,             // param1 hold
                           0.0, 0.0,        // param2 acceptance, param3 pass-through
                           transectYawDeg,  // param4 yaw
                           lat, lon, alt,
                           true, false, nullptr);
        out.append(_missionItemToJson(arrive, /*withAltitude*/ true));

        // 2. Gimbal: DO_MOUNT_CONTROL, pitch(raw)/yaw from the capture panel.
        MissionItem gimbal(seqNum++, MAV_CMD_DO_MOUNT_CONTROL, MAV_FRAME_MISSION,
                           pitchRaw,        // param1 pitch (raw; -90 == straight down)
                           0.0,             // param2 roll
                           gimbalYaw,       // param3 yaw
                           0.0, 0.0, 0.0,   // param4-6 unused
                           MAV_MOUNT_MODE_MAVLINK_TARGETING,  // param7 mount mode
                           true, false, nullptr);
        out.append(_missionItemToJson(gimbal, /*withAltitude*/ false));

        // 3. Capture: IMAGE_START_CAPTURE, take 1 photo.
        MissionItem shoot(seqNum++, MAV_CMD_IMAGE_START_CAPTURE, MAV_FRAME_MISSION,
                          0.0,                          // param1 reserved
                          0.0,                          // param2 interval (none)
                          1.0,                          // param3 take 1 photo
                          photoPointCount + 1,          // param4 capture sequence number
                          qQNaN(), qQNaN(), qQNaN(),    // param5-7 reserved
                          true, false, nullptr);
        out.append(_missionItemToJson(shoot, /*withAltitude*/ false));

        // 4. Dwell: NAV_WAYPOINT, yaw = transect angle, hold from the capture panel.
        MissionItem dwell(seqNum++, MAV_CMD_NAV_WAYPOINT, frame,
                          holdSecs,        // param1 hold
                          0.0, 0.0,        // param2 acceptance, param3 pass-through
                          transectYawDeg,  // param4 yaw
                          lat, lon, alt,
                          true, false, nullptr);
        out.append(_missionItemToJson(dwell, /*withAltitude*/ true));

        ++photoPointCount;
    }

    temp->deleteLater();
    return out;
}

bool CustomSurveyManager::saveRegionWaypointPlans(QObject* item, const QString& folder)
{
    SurveyComplexItem* survey = _surveyItem(item);
    if (!survey) {
        _setLastError(tr("Invalid survey."));
        return false;
    }
    if (!isCustomSurvey(survey)) {
        _setLastError(tr("Only custom surveys can be exported."));
        return false;
    }
    PlanMasterController* controller = _itemController(survey);
    if (!controller) {
        _setLastError(tr("No plan controller available."));
        return false;
    }

    const QDir dir(folder);
    if (folder.isEmpty() || !dir.exists()) {
        _setLastError(tr("Output folder does not exist: %1").arg(folder));
        return false;
    }

    QString errorString;
    const QList<SplitRegion> regions = _computeRegions(survey, errorString);
    if (!errorString.isEmpty()) {
        _setLastError(errorString);
        return false;
    }
    if (regions.isEmpty()) {
        _setLastError(tr("There are no regions to export."));
        return false;
    }

    QJsonArray masterArray;
    survey->save(masterArray);
    if (masterArray.isEmpty()) {
        _setLastError(tr("Unable to serialize the survey."));
        return false;
    }
    const QJsonObject masterSurveyJson = masterArray.first().toObject();

    // Yaw stamped on every generated waypoint is the survey's transect angle (the
    // "Angle" set in the panel) — a single fixed heading regardless of the
    // alternating physical direction of each pass.
    const double transectYawDeg = survey->gridAngle()->rawValue().toDouble();

    const QJsonObject templateRoot = controller->saveToJson().object();
    QString baseName = QFileInfo(controller->currentPlanFile()).completeBaseName();
    if (baseName.isEmpty()) {
        baseName = QStringLiteral("custom-survey");
    }

    int saved = 0;
    int totalPhotoPoints = 0;
    for (int i = 0; i < regions.size(); ++i) {
        int photoPoints = 0;
        const QJsonArray waypointItems = _buildRegionWaypointItems(controller, masterSurveyJson, regions[i].polygon, transectYawDeg, photoPoints);
        if (waypointItems.isEmpty()) {
            continue;
        }
        totalPhotoPoints += photoPoints;

        QJsonObject root = templateRoot;
        QJsonObject mission = root.value(QStringLiteral("mission")).toObject();
        mission[QStringLiteral("items")] = waypointItems;
        root[QStringLiteral("mission")] = mission;

        const QString filename = dir.absoluteFilePath(QStringLiteral("%1-region_%2-waypoints.plan").arg(baseName).arg(i + 1));
        if (_writePlanFile(QJsonDocument(root), filename)) {
            ++saved;
        }
    }

    if (saved == 0 || totalPhotoPoints == 0) {
        _setLastError(tr("No photo points were generated. Configure a camera and a non-zero trigger distance on the survey so it produces discrete capture points."));
        return false;
    }

    _setLastError(tr("Exported %1 waypoint plan(s), %2 photo point(s) total, to %3.").arg(saved).arg(totalPhotoPoints).arg(folder));
    return true;
}

bool CustomSurveyManager::saveRegionPlans(QObject* item, const QString& folder)
{
    SurveyComplexItem* survey = _surveyItem(item);
    if (!survey) {
        _setLastError(tr("Invalid survey."));
        return false;
    }
    if (!isCustomSurvey(survey)) {
        _setLastError(tr("Only custom surveys can be exported."));
        return false;
    }
    PlanMasterController* controller = _itemController(survey);
    if (!controller) {
        _setLastError(tr("No plan controller available."));
        return false;
    }

    const QDir dir(folder);
    if (folder.isEmpty() || !dir.exists()) {
        _setLastError(tr("Output folder does not exist: %1").arg(folder));
        return false;
    }

    QString errorString;
    const QList<SplitRegion> regions = _computeRegions(survey, errorString);
    if (!errorString.isEmpty()) {
        _setLastError(errorString);
        return false;
    }
    if (regions.isEmpty()) {
        _setLastError(tr("There are no regions to export."));
        return false;
    }

    // Snapshot the master survey's parameters once.
    QJsonArray masterArray;
    survey->save(masterArray);
    if (masterArray.isEmpty()) {
        _setLastError(tr("Unable to serialize the survey."));
        return false;
    }
    const QJsonObject masterSurveyJson = masterArray.first().toObject();

    // Reuse the full plan envelope (header, planned home, vehicle profile,
    // empty geofence/rally) exactly as core would write it.
    const QJsonObject templateRoot = controller->saveToJson().object();

    QString baseName = QFileInfo(controller->currentPlanFile()).completeBaseName();
    if (baseName.isEmpty()) {
        baseName = QStringLiteral("custom-survey");
    }

    int saved = 0;
    bool terrainPending = false;

    for (int i = 0; i < regions.size(); ++i) {
        QJsonObject regionSurveyJson = _buildRegionSurveyJson(controller, masterSurveyJson, regions[i].polygon, terrainPending);
        if (regionSurveyJson.isEmpty()) {
            continue;
        }

        // Each exported region is a standalone, single-region custom survey.
        QJsonObject customSurvey;
        customSurvey[QStringLiteral("version")]     = 5;
        customSurvey[QStringLiteral("regionCount")] = 1;
        regionSurveyJson[QStringLiteral("customSurvey")] = customSurvey;

        QJsonObject root = templateRoot;
        QJsonObject mission = root.value(QStringLiteral("mission")).toObject();
        mission[QStringLiteral("items")] = QJsonArray{ regionSurveyJson };
        root[QStringLiteral("mission")] = mission;

        const QString filename = dir.absoluteFilePath(QStringLiteral("%1-region_%2.plan").arg(baseName).arg(i + 1));
        if (_writePlanFile(QJsonDocument(root), filename)) {
            ++saved;
        }
    }

    if (saved == 0) {
        _setLastError(tr("Failed to export any region plans."));
        return false;
    }

    QString message = tr("Exported %1 region plan(s) to %2.").arg(saved).arg(folder);
    if (terrainPending) {
        message += QStringLiteral(" ") + tr("Terrain-follow altitudes will be recomputed when a region plan is opened.");
    }
    _setLastError(message);
    return true;
}

bool CustomSurveyManager::_writePlanFile(const QJsonDocument& document, const QString& filename)
{
    QFile file(filename);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text)) {
        _setLastError(tr("Unable to write %1").arg(filename));
        return false;
    }
    file.write(document.toJson(QJsonDocument::Indented));
    return true;
}

//=============================================================================
// Mission JSON save / restore (plan-file plugin hooks)
//=============================================================================

int CustomSurveyManager::_sequenceNumberFromMissionObject(const QJsonObject& itemObject) const
{
    static constexpr const char* kDoJumpIdKey = "doJumpId";
    static constexpr const char* kTransectStyleKey = "TransectStyleComplexItem";
    static constexpr const char* kTransectItemsKey = "Items";

    if (itemObject.contains(kDoJumpIdKey)) {
        return itemObject[kDoJumpIdKey].toInt(-1);
    }

    const QJsonArray items = itemObject[kTransectStyleKey].toObject()[kTransectItemsKey].toArray();
    if (!items.isEmpty()) {
        return items.first().toObject()[kDoJumpIdKey].toInt(-1);
    }
    return -1;
}

bool CustomSurveyManager::_isSurveyMissionObject(const QJsonObject& itemObject) const
{
    return itemObject[QStringLiteral("type")].toString() == QStringLiteral("ComplexItem") &&
           itemObject[QStringLiteral("complexItemType")].toString() == QStringLiteral("survey");
}

QJsonObject CustomSurveyManager::_metadataForItem(QObject* item)
{
    ControlState& state = _stateFor(item);

    QJsonObject obj;
    obj[QStringLiteral("version")]           = 6;
    obj[QStringLiteral("regionCount")]       = state.regionCount;
    obj[QStringLiteral("regionOffset")]      = state.regionOffset;
    obj[QStringLiteral("sourceSequence")]    = _sequenceNumber(item);

    // Individual-waypoint export settings. exportAsWaypoints is per-survey; the
    // gimbal/hold capture values are shared (manager-level Facts) but written on
    // each custom survey so a saved plan reopens with them.
    obj[QStringLiteral("exportAsWaypoints")] = state.exportAsWaypoints;
    obj[QStringLiteral("captureGimbalPitch")] = _captureGimbalPitchFact->rawValue().toDouble();
    obj[QStringLiteral("captureGimbalYaw")]   = _captureGimbalYawFact->rawValue().toDouble();
    obj[QStringLiteral("captureHold")]        = _captureHoldFact->rawValue().toDouble();

    // Control points are 2D map positions (no meaningful altitude). Save WITHOUT
    // altitude so the element count matches the loader, which reads them with
    // altitudeRequired=false. JsonHelper::_loadGeoCoordinate enforces an EXACT
    // element count (2 vs 3), so a writeAltitude/altitudeRequired mismatch makes
    // the load silently fail — which previously wiped the restored vertices and
    // forced a re-seed to defaults.
    if (state.center.isValid()) {
        QJsonValue value;
        JsonHelper::saveGeoCoordinate(state.center, false /*writeAltitude*/, value);
        obj[QStringLiteral("center")] = value;
    }
    if (!state.edgeVertices.isEmpty()) {
        QJsonValue value;
        JsonHelper::saveGeoCoordinateArray(state.edgeVertices, false /*writeAltitude*/, value);
        obj[QStringLiteral("edgeVertices")] = value;
    }
    return obj;
}

void CustomSurveyManager::decorateMissionJson(PlanMasterController* controller, QJsonObject& missionJson)
{
    if (!controller || _customSurveyItems.isEmpty()) {
        return;
    }

    QHash<int, QObject*> bySequence;
    for (QObject* item : std::as_const(_customSurveyItems)) {
        if (!item || _itemController(item) != controller) {
            continue;
        }
        const int seq = _sequenceNumber(item);
        if (seq >= 0) {
            bySequence.insert(seq, item);
        }
    }

    QJsonArray items = missionJson[QStringLiteral("items")].toArray();
    for (int i = 0; i < items.size(); ++i) {
        QJsonObject obj = items[i].toObject();
        if (!_isSurveyMissionObject(obj)) {
            continue;
        }
        QObject* custom = bySequence.value(_sequenceNumberFromMissionObject(obj), nullptr);
        if (!custom) {
            continue;
        }
        obj[QStringLiteral("customSurvey")] = _metadataForItem(custom);
        items[i] = obj;
    }
    missionJson[QStringLiteral("items")] = items;
}

void CustomSurveyManager::restoreFromPlanJson(PlanMasterController* controller, const QJsonObject& planJson)
{
    if (!controller) {
        return;
    }

    const QJsonArray items = planJson[QStringLiteral("mission")].toObject()[QStringLiteral("items")].toArray();

    QHash<int, QJsonObject> customBySequence;
    for (const QJsonValue& value : items) {
        const QJsonObject obj = value.toObject();
        if (!obj.contains(QStringLiteral("customSurvey"))) {
            continue;
        }
        const int seq = _sequenceNumberFromMissionObject(obj);
        if (seq >= 0) {
            customBySequence.insert(seq, obj[QStringLiteral("customSurvey")].toObject());
        }
    }
    if (customBySequence.isEmpty()) {
        return;
    }

    // Capture settings are shared (manager-level Facts); restore them from the
    // first custom survey that carries them. Suppress the dirty side effect so a
    // freshly-loaded plan is not marked modified.
    _suppressCaptureDirty = true;
    for (auto it = customBySequence.constBegin(); it != customBySequence.constEnd(); ++it) {
        const QJsonObject& cs = it.value();
        if (cs.contains(QStringLiteral("captureGimbalPitch")) ||
            cs.contains(QStringLiteral("captureGimbalYaw")) ||
            cs.contains(QStringLiteral("captureHold"))) {
            _captureGimbalPitchFact->setRawValue(cs.value(QStringLiteral("captureGimbalPitch")).toDouble(-90.0));
            _captureGimbalYawFact->setRawValue(cs.value(QStringLiteral("captureGimbalYaw")).toDouble(0.0));
            _captureHoldFact->setRawValue(cs.value(QStringLiteral("captureHold")).toDouble(2.0));
            break;
        }
    }
    _suppressCaptureDirty = false;

    auto parse = [](const QJsonObject& cs) {
        ControlState state;
        state.regionCount = cs.value(QStringLiteral("regionCount")).toInt(1);
        state.regionOffset = cs.value(QStringLiteral("regionOffset")).toDouble(0.0);
        state.exportAsWaypoints = cs.value(QStringLiteral("exportAsWaypoints")).toBool(false);
        QString errorString;
        if (cs.contains(QStringLiteral("center"))) {
            QGeoCoordinate center;
            JsonHelper::loadGeoCoordinate(cs.value(QStringLiteral("center")), false, center, errorString);
            state.center = center;
        }
        if (cs.contains(QStringLiteral("edgeVertices"))) {
            QList<QGeoCoordinate> vertices;
            JsonHelper::loadGeoCoordinateArray(cs.value(QStringLiteral("edgeVertices")), false, vertices, errorString);
            state.edgeVertices = vertices;
        }
        state.seeded = true;
        return state;
    };

    QmlObjectListModel* visualItems = controller->missionController()->visualItems();
    QSet<int> matched;
    if (visualItems) {
        for (int i = 0; i < visualItems->count(); ++i) {
            QObject* item = visualItems->get(i);
            const int seq = _sequenceNumber(item);
            if (!customBySequence.contains(seq)) {
                continue;
            }
            _markCustomSurvey(item, false /*setDirty*/);
            ControlState& state = _stateFor(item);
            state = parse(customBySequence.value(seq));
            if (SurveyComplexItem* survey = _surveyItem(item)) {
                state.lastPolygonCenter = survey->surveyAreaPolygon()->center();
            }
            _snapEdgeVerticesToMidpoints(item, state);
            matched.insert(seq);
            emit customSurveyChanged(item);
        }
    }

    // Anything not yet materialized is stashed and transferred on attach.
    for (auto it = customBySequence.constBegin(); it != customBySequence.constEnd(); ++it) {
        if (!matched.contains(it.key())) {
            _pendingState.insert(it.key(), parse(it.value()));
        }
    }
}

//=============================================================================
// Misc helpers
//=============================================================================

double CustomSurveyManager::_polygonArea(const QList<QGeoCoordinate>& coordinates) const
{
    if (coordinates.size() < 3) {
        return 0.0;
    }
    const QGeoCoordinate origin = coordinates.first();
    QList<QPointF> local;
    for (const QGeoCoordinate& c : coordinates) {
        double n, e, d;
        QGCGeo::convertGeoToNed(c, origin, n, e, d);
        local.append(QPointF(e, n));
    }
    double area = 0.0;
    for (int i = 0; i < local.size(); ++i) {
        const QPointF& p0 = local[i];
        const QPointF& p1 = local[(i + 1) % local.size()];
        area += (p0.x() * p1.y()) - (p1.x() * p0.y());
    }
    return std::abs(area) * 0.5;
}

QVariantList CustomSurveyManager::_coordinatesToVariantList(const QList<QGeoCoordinate>& coordinates) const
{
    QVariantList out;
    for (const QGeoCoordinate& c : coordinates) {
        out << QVariant::fromValue(c);
    }
    return out;
}

void CustomSurveyManager::_setLastError(const QString& errorString)
{
    if (_lastError == errorString) {
        return;
    }
    _lastError = errorString;
    emit lastErrorChanged();
}
