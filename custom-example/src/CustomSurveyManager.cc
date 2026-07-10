/****************************************************************************
 *
 * (c) 2009-2024 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

#include "CustomSurveyManager.h"

#include "JsonHelper.h"
#include "MissionController.h"
#include "PlanMasterController.h"
#include "QGCGeo.h"
#include "QGCMapPolygon.h"
#include "QmlObjectListModel.h"
#include "SurveyComplexItem.h"
#include "VisualMissionItem.h"

#include <QtCore/QDir>
#include <QtCore/QFile>
#include <QtCore/QFileInfo>
#include <QtCore/QJsonArray>
#include <QtCore/QJsonDocument>
#include <QtCore/QPointF>

#include <cmath>

CustomSurveyManager::CustomSurveyManager(QObject* parent)
    : QObject(parent)
{
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

    connect(survey, &QObject::destroyed, this, [this](QObject* obj) {
        _stateBySurvey.remove(obj);
        _customSurveyItems.remove(obj);
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

        // Follow whole-survey moves. When the survey polygon is translated (its
        // center is dragged), QGCMapPolygon shifts every vertex by the same
        // geodesic delta and emits centerChanged; we mirror that shift onto the
        // control points so the division travels with the survey.
        if (QGCMapPolygon* polygon = survey->surveyAreaPolygon()) {
            _stateBySurvey[survey].lastPolygonCenter = polygon->center();
            connect(polygon, &QGCMapPolygon::centerChanged, this, [this, survey]() {
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

    if (!oldCenter.isValid()) {
        return;                 // first observation: nothing to translate against yet
    }
    const double distance = oldCenter.distanceTo(newCenter);
    if (distance < 0.01) {
        return;                 // negligible movement
    }
    const double azimuth = oldCenter.azimuthTo(newCenter);

    // Same rigid geodesic translation QGCMapPolygon applies to its vertices.
    if (state.center.isValid()) {
        state.center = state.center.atDistanceAndAzimuth(distance, azimuth);
    }
    for (int i = 0; i < state.edgePoints.size(); ++i) {
        state.edgePoints[i] = state.edgePoints[i].atDistanceAndAzimuth(distance, azimuth);
    }

    emit customSurveyChanged(survey);
}

//=============================================================================
// Control points
//=============================================================================

void CustomSurveyManager::_seedControlPoints(QObject* item, ControlState& state, int count)
{
    SurveyComplexItem* survey = _surveyItem(item);
    if (!survey) {
        return;
    }
    const QList<QGeoCoordinate> polygon = survey->surveyAreaPolygon()->coordinateList();
    if (polygon.size() < 3) {
        return;
    }

    const QGeoCoordinate origin = polygon.first();

    QGeoCoordinate centerGeo = state.center.isValid() ? state.center : survey->surveyAreaPolygon()->center();
    if (!centerGeo.isValid()) {
        centerGeo = origin;
    }

    double cn, ce, cd;
    QGCGeo::convertGeoToNed(centerGeo, origin, cn, ce, cd);
    const QPointF center(ce, cn);

    double radius = 0.0;
    for (const QGeoCoordinate& c : polygon) {
        double n, e, d;
        QGCGeo::convertGeoToNed(c, origin, n, e, d);
        radius = std::max(radius, std::hypot(e - center.x(), n - center.y()));
    }
    if (radius < 1.0) {
        radius = 1.0;
    }

    state.center = centerGeo;
    state.edgePoints.clear();
    for (int i = 0; i < count; ++i) {
        const double angle = (2.0 * M_PI * i) / static_cast<double>(count);
        const double east  = center.x() + (0.6 * radius * std::cos(angle));
        const double north = center.y() + (0.6 * radius * std::sin(angle));
        QGeoCoordinate edge;
        QGCGeo::convertNedToGeo(north, east, 0.0, origin, edge);
        state.edgePoints.append(edge);
    }
    state.regionCount = count;
    state.seeded = true;
    state.lastPolygonCenter = survey->surveyAreaPolygon()->center();
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
        state.edgePoints.clear();
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
    if (!_surveyItem(item)) {
        return QGeoCoordinate();
    }
    ControlState& state = _stateFor(item);
    if (!state.center.isValid()) {
        state.center = _surveyItem(item)->surveyAreaPolygon()->center();
    }
    return state.center;
}

QVariantList CustomSurveyManager::edgeControlPoints(QObject* item)
{
    QVariantList out;
    if (!_surveyItem(item)) {
        return out;
    }
    ControlState& state = _stateFor(item);
    if (state.regionCount >= 2 && (!state.seeded || state.edgePoints.size() != state.regionCount)) {
        _seedControlPoints(item, state, state.regionCount);
    }
    for (const QGeoCoordinate& c : std::as_const(state.edgePoints)) {
        out << QVariant::fromValue(c);
    }
    return out;
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
    if (!_surveyItem(item) || !coordinate.isValid()) {
        return;
    }
    ControlState& state = _stateFor(item);
    if (index < 0 || index >= state.edgePoints.size()) {
        return;
    }
    state.edgePoints[index] = coordinate;
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

    if (!state.seeded || state.edgePoints.size() != state.regionCount) {
        _seedControlPoints(item, state, state.regionCount);
    }

    SplitInput input;
    input.masterPolygon = polygon;
    input.center        = state.center;
    input.edgePoints    = state.edgePoints;

    return _splitter.split(input, errorString);
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

//=============================================================================
// Export
//=============================================================================

QJsonObject CustomSurveyManager::_buildRegionSurveyJson(PlanMasterController* controller,
                                                        const QJsonObject& masterSurveyJson,
                                                        const QList<QGeoCoordinate>& regionPolygon,
                                                        bool& terrainPending) const
{
    // Build the region's survey from a genuine, freshly-configured survey
    // object so its transects are recomputed for the sub-polygon. We never
    // touch the live survey and never block its signals (that was the bug in
    // the prior attempt which left every export with the parent's grid).
    SurveyComplexItem* temp = new SurveyComplexItem(controller, /*flyView*/ false, QString());

    QString errorString;
    if (!temp->load(masterSurveyJson, 1, errorString)) {
        temp->deleteLater();
        return {};
    }

    QGCMapPolygon* polygon = temp->surveyAreaPolygon();
    polygon->beginReset();
    polygon->clear();
    polygon->appendVertices(regionPolygon);
    polygon->endReset();     // emits pathChanged -> synchronous transect rebuild

    if (temp->readyForSaveState() != VisualMissionItem::ReadyForSave) {
        terrainPending = true;
    }

    QJsonArray items;
    temp->save(items);
    temp->deleteLater();

    return items.isEmpty() ? QJsonObject() : items.first().toObject();
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
        customSurvey[QStringLiteral("version")]     = 2;
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
    obj[QStringLiteral("version")]        = 2;
    obj[QStringLiteral("regionCount")]    = state.regionCount;
    obj[QStringLiteral("sourceSequence")] = _sequenceNumber(item);

    if (state.center.isValid()) {
        QJsonValue value;
        JsonHelper::saveGeoCoordinate(state.center, true /*writeAltitude*/, value);
        obj[QStringLiteral("center")] = value;
    }
    if (!state.edgePoints.isEmpty()) {
        QJsonValue value;
        JsonHelper::saveGeoCoordinateArray(state.edgePoints, true /*writeAltitude*/, value);
        obj[QStringLiteral("controlPoints")] = value;
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

    auto parse = [](const QJsonObject& cs) {
        ControlState state;
        state.regionCount = cs.value(QStringLiteral("regionCount")).toInt(1);
        QString errorString;
        if (cs.contains(QStringLiteral("center"))) {
            QGeoCoordinate center;
            JsonHelper::loadGeoCoordinate(cs.value(QStringLiteral("center")), false, center, errorString);
            state.center = center;
        }
        if (cs.contains(QStringLiteral("controlPoints"))) {
            QList<QGeoCoordinate> points;
            JsonHelper::loadGeoCoordinateArray(cs.value(QStringLiteral("controlPoints")), false, points, errorString);
            state.edgePoints = points;
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
