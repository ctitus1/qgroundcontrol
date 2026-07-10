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
#include <QtCore/QDebug>

#include <cmath>
#include <limits>

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
// Boundary <-> perimeter-fraction helpers
//=============================================================================

QGeoCoordinate CustomSurveyManager::_boundaryCoordinate(const QList<QGeoCoordinate>& polygon, double param)
{
    const int n = polygon.size();
    if (n == 0) {
        return QGeoCoordinate();
    }
    if (n < 2) {
        return polygon.first();
    }

    QList<double> segLength;
    segLength.reserve(n);
    double total = 0.0;
    for (int i = 0; i < n; ++i) {
        const double length = polygon[i].distanceTo(polygon[(i + 1) % n]);
        segLength.append(length);
        total += length;
    }
    if (total <= 0.0) {
        return polygon.first();
    }

    double fraction = param - std::floor(param);   // wrap into [0,1)
    double target = fraction * total;
    for (int i = 0; i < n; ++i) {
        if (target <= segLength[i] || i == n - 1) {
            const QGeoCoordinate& a = polygon[i];
            const QGeoCoordinate& b = polygon[(i + 1) % n];
            if (segLength[i] <= 0.0) {
                return a;
            }
            return a.atDistanceAndAzimuth(qMin(target, segLength[i]), a.azimuthTo(b));
        }
        target -= segLength[i];
    }
    return polygon.first();
}

double CustomSurveyManager::_boundaryParam(const QList<QGeoCoordinate>& polygon, const QGeoCoordinate& coordinate)
{
    const int n = polygon.size();
    if (n < 2) {
        return 0.0;
    }

    const QGeoCoordinate origin = polygon.first();
    auto toLocal = [&origin](const QGeoCoordinate& c) {
        double north, east, down;
        QGCGeo::convertGeoToNed(c, origin, north, east, down);
        return QPointF(east, north);
    };

    QList<QPointF> local;
    local.reserve(n);
    for (const QGeoCoordinate& c : polygon) {
        local.append(toLocal(c));
    }
    const QPointF point = toLocal(coordinate);

    QList<double> segLength;
    segLength.reserve(n);
    double total = 0.0;
    for (int i = 0; i < n; ++i) {
        const QPointF d = local[(i + 1) % n] - local[i];
        const double length = std::hypot(d.x(), d.y());
        segLength.append(length);
        total += length;
    }
    if (total <= 0.0) {
        return 0.0;
    }

    double bestDistance = std::numeric_limits<double>::max();
    double bestPerim = 0.0;
    double cumulative = 0.0;
    for (int i = 0; i < n; ++i) {
        const QPointF a = local[i];
        const QPointF ab = local[(i + 1) % n] - a;
        const double len2 = (ab.x() * ab.x()) + (ab.y() * ab.y());
        double t = (len2 > 0.0) ? (((point.x() - a.x()) * ab.x()) + ((point.y() - a.y()) * ab.y())) / len2 : 0.0;
        t = qBound(0.0, t, 1.0);
        const QPointF proj(a.x() + (t * ab.x()), a.y() + (t * ab.y()));
        const double distance = std::hypot(point.x() - proj.x(), point.y() - proj.y());
        if (distance < bestDistance) {
            bestDistance = distance;
            bestPerim = cumulative + (t * segLength[i]);
        }
        cumulative += segLength[i];
    }
    return bestPerim / total;
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
        // interior center control point (edge cuts are perimeter fractions and
        // follow the boundary on their own).
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
    qWarning() << "CSDBG _onSurveyPolygonMoved old=" << oldCenter << "new=" << newCenter << "dist=" << distance << "center=" << state.center;
    if (distance < 0.01) {
        return;                 // negligible movement
    }

    // Same rigid geodesic translation QGCMapPolygon applies to its vertices.
    if (state.center.isValid()) {
        state.center = state.center.atDistanceAndAzimuth(distance, oldCenter.azimuthTo(newCenter));
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
    if (survey->surveyAreaPolygon()->coordinateList().size() < 3) {
        return;
    }

    if (!state.center.isValid()) {
        state.center = survey->surveyAreaPolygon()->center();
    }
    qWarning() << "CSDBG _seedControlPoints count=" << count << "center=" << state.center;

    // Distribute the boundary cuts evenly around the perimeter.
    state.edgeParams.clear();
    for (int i = 0; i < count; ++i) {
        state.edgeParams.append(static_cast<double>(i) / static_cast<double>(count));
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
        state.edgeParams.clear();
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
    qWarning() << "CSDBG centerControlPoint ->" << state.center
               << "inside=" << survey->surveyAreaPolygon()->containsCoordinate(state.center);
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
    if (state.regionCount >= 2 && (!state.seeded || state.edgeParams.size() != state.regionCount)) {
        _seedControlPoints(item, state, state.regionCount);
    }
    const QList<QGeoCoordinate> polygon = survey->surveyAreaPolygon()->coordinateList();
    for (const double param : std::as_const(state.edgeParams)) {
        out << QVariant::fromValue(_boundaryCoordinate(polygon, param));
    }
    return out;
}

void CustomSurveyManager::setCenterControlPoint(QObject* item, const QGeoCoordinate& coordinate)
{
    qWarning() << "CSDBG setCenterControlPoint coord=" << coordinate << "valid=" << coordinate.isValid();
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
    qWarning() << "CSDBG setEdgeControlPoint idx=" << index << "coord=" << coordinate;
    SurveyComplexItem* survey = _surveyItem(item);
    if (!survey || !coordinate.isValid()) {
        return;
    }
    ControlState& state = _stateFor(item);
    if (index < 0 || index >= state.edgeParams.size()) {
        return;
    }
    const QList<QGeoCoordinate> polygon = survey->surveyAreaPolygon()->coordinateList();
    if (polygon.size() < 3) {
        return;
    }
    // Constrain the cut to the survey boundary: store where the dragged point
    // projects onto the perimeter.
    state.edgeParams[index] = _boundaryParam(polygon, coordinate);
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

    if (!state.seeded || state.edgeParams.size() != state.regionCount) {
        _seedControlPoints(item, state, state.regionCount);
    }

    SplitInput input;
    input.masterPolygon = polygon;
    input.center        = state.center.isValid() ? state.center : survey->surveyAreaPolygon()->center();
    for (const double param : std::as_const(state.edgeParams)) {
        input.edgePoints.append(_boundaryCoordinate(polygon, param));
    }

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
        customSurvey[QStringLiteral("version")]     = 3;
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
    obj[QStringLiteral("version")]        = 3;
    obj[QStringLiteral("regionCount")]    = state.regionCount;
    obj[QStringLiteral("sourceSequence")] = _sequenceNumber(item);

    if (state.center.isValid()) {
        QJsonValue value;
        JsonHelper::saveGeoCoordinate(state.center, true /*writeAltitude*/, value);
        obj[QStringLiteral("center")] = value;
    }
    if (!state.edgeParams.isEmpty()) {
        QJsonArray params;
        for (const double param : std::as_const(state.edgeParams)) {
            params.append(param);
        }
        obj[QStringLiteral("edgeParams")] = params;
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
        if (cs.contains(QStringLiteral("edgeParams"))) {
            const QJsonArray params = cs.value(QStringLiteral("edgeParams")).toArray();
            for (const QJsonValue& value : params) {
                state.edgeParams.append(value.toDouble());
            }
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
