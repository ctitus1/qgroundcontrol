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
#include <QtCore/QHash>
#include <QtCore/QIODevice>
#include <QtCore/QJsonArray>
#include <QtCore/QJsonDocument>
#include <QtCore/QRegularExpression>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace {
constexpr const char* kItemsKey = "items";
constexpr const char* kTypeKey = "type";
constexpr const char* kTypeComplexItemValue = "ComplexItem";
constexpr const char* kComplexItemTypeKey = "complexItemType";
constexpr const char* kSurveyComplexItemTypeValue = "survey";
constexpr const char* kTransectStyleKey = "TransectStyleComplexItem";
constexpr const char* kTransectItemsKey = "Items";
constexpr const char* kDoJumpIdKey = "doJumpId";
constexpr const char* kCustomSurveyKey = "customSurvey";
constexpr const char* kCustomSurveyTypeKey = "surveyType";
constexpr const char* kCustomSurveyTypeValue = "custom";
constexpr const char* kCustomSurveyVersionKey = "version";
constexpr const char* kSourceSequenceKey = "sourceSequence";
constexpr const char* kQuadrantsKey = "quadrants";
constexpr const char* kQuadrantNameKey = "name";
constexpr const char* kQuadrantPolygonKey = "polygon";
constexpr const char* kQuadrantAreaKey = "area";
constexpr const char* kExportedQuadrantKey = "exportedQuadrant";
constexpr double kPointEpsilonMeters = 0.05;
constexpr int kDivisionCount = 3;    // Change this to 2,3,4,...

bool pointsClose(const QPointF& a, const QPointF& b)
{
    return std::hypot(a.x() - b.x(), a.y() - b.y()) < kPointEpsilonMeters;
}

void appendUnique(QList<QPointF>& points, const QPointF& point)
{
    if (points.isEmpty() || !pointsClose(points.last(), point)) {
        points.append(point);
    }
}

void closeAndClean(QList<QPointF>& points)
{
    if (points.count() > 1 && pointsClose(points.first(), points.last())) {
        points.removeLast();
    }

    for (int i = points.count() - 1; i > 0; --i) {
        if (pointsClose(points[i], points[i - 1])) {
            points.removeAt(i);
        }
    }
}

QString sanitizedBaseName(QString baseName)
{
    baseName = baseName.trimmed();
    if (baseName.isEmpty()) {
        baseName = QStringLiteral("custom-survey");
    }
    baseName.replace(QRegularExpression(QStringLiteral("[^A-Za-z0-9._-]+")), QStringLiteral("-"));
    baseName.replace(QRegularExpression(QStringLiteral("-+")), QStringLiteral("-"));
    return baseName;
}
}

CustomSurveyManager::CustomSurveyManager(QObject* parent)
    : QObject(parent)
{
}

bool CustomSurveyManager::markCustomSurvey(QObject* item)
{
    return _markCustomSurvey(item, true /* setDirty */);
}

bool CustomSurveyManager::_markCustomSurvey(QObject* item, bool setDirty)
{
    SurveyComplexItem* surveyItem = _surveyItem(item);
    if (!surveyItem) {
        _setLastError(tr("Custom surveys can only be created from Survey items."));
        return false;
    }

    if (!_customSurveyItems.contains(surveyItem)) {
        _customSurveyItems.insert(surveyItem);
        connect(surveyItem, &QObject::destroyed, this, [this](QObject* destroyedItem) {
            _customSurveyItems.remove(destroyedItem);
        });
        emit customSurveyChanged(surveyItem);
    }

    if (setDirty) {
        if (PlanMasterController* masterController = _itemPlanMasterController(surveyItem)) {
            masterController->setDirty(true);
        }
    }

    _setLastError(QString());
    return true;
}

bool CustomSurveyManager::isCustomSurvey(QObject* item) const
{
    return item && _customSurveyItems.contains(_surveyItem(item));
}

QVariantList CustomSurveyManager::quadrantPolygons(QObject* item)
{
    QString errorString;
    const QList<QuadrantInfo> quadrants = _buildQuadrants(item, errorString);
    if (!errorString.isEmpty()) {
        _setLastError(errorString);
    } else {
        _setLastError(QString());
    }

    QVariantList quadrantList;
    for (const QuadrantInfo& quadrant: quadrants) {
        QVariantMap quadrantMap;
        quadrantMap[QStringLiteral("name")] = quadrant.name;
        quadrantMap[QStringLiteral("fileSuffix")] = quadrant.fileSuffix;
        quadrantMap[QStringLiteral("vertexCount")] = quadrant.polygon.count();
        quadrantMap[QStringLiteral("area")] = _polygonArea(quadrant.polygon);
        quadrantMap[QStringLiteral("polygon")] = _coordinatesToVariantList(quadrant.polygon);
        quadrantList.append(quadrantMap);
    }

    return quadrantList;
}

bool CustomSurveyManager::saveQuadrantPlans(QObject* planMasterControllerObject, QObject* item, const QString& folder)
{
    PlanMasterController* planMasterController = qobject_cast<PlanMasterController*>(planMasterControllerObject);
    SurveyComplexItem* surveyItem = _surveyItem(item);
    if (!planMasterController || !surveyItem) {
        _setLastError(tr("Unable to save quadrant plans: missing plan or survey."));
        return false;
    }
    if (!isCustomSurvey(surveyItem)) {
        _setLastError(tr("Only custom surveys can be saved as quadrant plans."));
        return false;
    }

    QDir outputDir(folder);
    if (!outputDir.exists()) {
        _setLastError(tr("Output folder does not exist: %1").arg(folder));
        return false;
    }

    QString errorString;
    const QList<QuadrantInfo> quadrants = _buildQuadrants(surveyItem, errorString);
    if (!errorString.isEmpty()) {
        _setLastError(errorString);
        return false;
    }
    if (quadrants.isEmpty()) {
        _setLastError(tr("No non-empty quadrants were generated."));
        return false;
    }

    QGCMapPolygon* surveyPolygon = surveyItem->surveyAreaPolygon();
    const QList<QGeoCoordinate> originalPolygon = surveyPolygon->coordinateList();
    const bool wasDirty = planMasterController->dirty();
    const QString currentPlan = planMasterController->currentPlanFile();
    const QString baseName = sanitizedBaseName(currentPlan.isEmpty() ? QStringLiteral("custom-survey") : QFileInfo(currentPlan).completeBaseName());

    int savedCount = 0;
    for (const QuadrantInfo& quadrant: quadrants) {
        if (!_applyPolygon(surveyPolygon, quadrant.polygon)) {
            continue;
        }

        QJsonDocument quadrantPlan = planMasterController->saveToJson();
        QJsonObject quadrantRoot = quadrantPlan.object();
        QJsonObject missionObject = quadrantRoot[PlanMasterController::kJsonMissionObjectKey].toObject();
        QJsonArray items = missionObject[kItemsKey].toArray();
        int itemIndex = -1;
        if (_findMissionObjectBySequence(items, _sequenceNumber(surveyItem), itemIndex)) {
            QJsonObject itemObject = items[itemIndex].toObject();
            QJsonObject customObject = itemObject[kCustomSurveyKey].toObject();
            customObject[kExportedQuadrantKey] = quadrant.name;
            itemObject[kCustomSurveyKey] = customObject;
            items[itemIndex] = itemObject;
            missionObject[kItemsKey] = items;
            quadrantRoot[PlanMasterController::kJsonMissionObjectKey] = missionObject;
            quadrantPlan = QJsonDocument(quadrantRoot);
        }

        const QString filename = outputDir.absoluteFilePath(QStringLiteral("%1-%2.plan").arg(baseName, quadrant.fileSuffix));
        if (!_writePlanFile(quadrantPlan, filename)) {
            _applyPolygon(surveyPolygon, originalPolygon);
            planMasterController->setDirty(wasDirty);
            return false;
        }
        ++savedCount;
    }

    _applyPolygon(surveyPolygon, originalPolygon);
    planMasterController->setDirty(wasDirty);

    _setLastError(tr("Saved %1 quadrant plan(s).").arg(savedCount));
    return savedCount > 0;
}

void CustomSurveyManager::decorateMissionJson(PlanMasterController* planMasterController, QJsonObject& missionJson)
{
    if (!planMasterController || _customSurveyItems.isEmpty()) {
        return;
    }

    QHash<int, QObject*> customItemsBySequence;
    for (QObject* item: std::as_const(_customSurveyItems)) {
        if (!item) {
            continue;
        }
        if (_itemPlanMasterController(item) == planMasterController) {
            const int sequenceNumber = _sequenceNumber(item);
            if (sequenceNumber >= 0) {
                customItemsBySequence[sequenceNumber] = item;
            }
        }
    }
    if (customItemsBySequence.isEmpty()) {
        return;
    }

    QJsonArray missionItems = missionJson[kItemsKey].toArray();
    for (int i = 0; i < missionItems.count(); ++i) {
        QJsonObject itemObject = missionItems[i].toObject();
        if (!_isSurveyMissionObject(itemObject)) {
            continue;
        }

        QObject* customItem = customItemsBySequence.value(_sequenceNumberFromMissionObject(itemObject), nullptr);
        if (customItem) {
            itemObject[kCustomSurveyKey] = _metadataForItem(customItem);
            missionItems[i] = itemObject;
        }
    }
    missionJson[kItemsKey] = missionItems;
}

void CustomSurveyManager::restoreFromPlanJson(PlanMasterController* planMasterController, const QJsonObject& planJson)
{
    if (!planMasterController) {
        return;
    }

    const QJsonObject missionObject = planJson[PlanMasterController::kJsonMissionObjectKey].toObject();
    const QJsonArray missionItems = missionObject[kItemsKey].toArray();
    QSet<int> customSequences;
    for (const QJsonValue& missionItemValue: missionItems) {
        const QJsonObject itemObject = missionItemValue.toObject();
        const QJsonObject customObject = itemObject[kCustomSurveyKey].toObject();
        if (customObject[kCustomSurveyTypeKey].toString() == QLatin1StringView(kCustomSurveyTypeValue)) {
            const int sequenceNumber = _sequenceNumberFromMissionObject(itemObject);
            if (sequenceNumber >= 0) {
                customSequences.insert(sequenceNumber);
            }
        }
    }

    if (customSequences.isEmpty()) {
        return;
    }

    QmlObjectListModel* visualItems = planMasterController->missionController()->visualItems();
    for (int i = 0; i < visualItems->count(); ++i) {
        QObject* visualItem = visualItems->get(i);
        if (customSequences.contains(_sequenceNumber(visualItem))) {
            _markCustomSurvey(visualItem, false /* setDirty */);
        }
    }
}

SurveyComplexItem* CustomSurveyManager::_surveyItem(QObject* item) const
{
    return qobject_cast<SurveyComplexItem*>(item);
}

VisualMissionItem* CustomSurveyManager::_visualItem(QObject* item) const
{
    return qobject_cast<VisualMissionItem*>(item);
}

PlanMasterController* CustomSurveyManager::_itemPlanMasterController(QObject* item) const
{
    if (VisualMissionItem* visualItem = _visualItem(item)) {
        return visualItem->masterController();
    }
    return nullptr;
}

int CustomSurveyManager::_sequenceNumber(QObject* item) const
{
    if (VisualMissionItem* visualItem = _visualItem(item)) {
        return visualItem->sequenceNumber();
    }
    return -1;
}

int CustomSurveyManager::_sequenceNumberFromMissionObject(const QJsonObject& itemObject) const
{
    if (itemObject.contains(kDoJumpIdKey)) {
        return itemObject[kDoJumpIdKey].toInt(-1);
    }

    const QJsonObject transectObject = itemObject[kTransectStyleKey].toObject();
    const QJsonArray transectItems = transectObject[kTransectItemsKey].toArray();
    if (!transectItems.isEmpty()) {
        return transectItems.first().toObject()[kDoJumpIdKey].toInt(-1);
    }

    return -1;
}

bool CustomSurveyManager::_isSurveyMissionObject(const QJsonObject& itemObject) const
{
    return itemObject[kTypeKey].toString() == QLatin1StringView(kTypeComplexItemValue) &&
       itemObject[kComplexItemTypeKey].toString() == QLatin1StringView(kSurveyComplexItemTypeValue);
}

bool CustomSurveyManager::_findMissionObjectBySequence(const QJsonArray& items, int sequenceNumber, int& itemIndex) const
{
    for (int i = 0; i < items.count(); ++i) {
        const QJsonObject itemObject = items[i].toObject();
        if (_isSurveyMissionObject(itemObject) && _sequenceNumberFromMissionObject(itemObject) == sequenceNumber) {
            itemIndex = i;
            return true;
        }
    }
    return false;
}

QList<CustomSurveyManager::QuadrantInfo> CustomSurveyManager::_buildQuadrants(QObject* item, QString& errorString) const
{
    errorString.clear();

    SurveyComplexItem* surveyItem = _surveyItem(item);
    if (!surveyItem) {
        errorString = tr("Custom survey quadrants require a Survey item.");
        return {};
    }

    const QList<QGeoCoordinate> polygon = surveyItem->surveyAreaPolygon()->coordinateList();
    if (polygon.count() < 3) {
        errorString = tr("Create a valid survey polygon before generating quadrants.");
        return {};
    }

    const QGeoCoordinate origin = polygon.first();
    QList<QPointF> localPolygon;
    localPolygon.reserve(polygon.count());
    for (const QGeoCoordinate& coordinate: polygon) {
        double north = 0.0;
        double east = 0.0;
        double down = 0.0;
        QGCGeo::convertGeoToNed(coordinate, origin, north, east, down);
        localPolygon.append(QPointF(east, north));
    }

    qreal minX = std::numeric_limits<qreal>::max();
    qreal maxX = std::numeric_limits<qreal>::lowest();
    qreal minY = std::numeric_limits<qreal>::max();
    qreal maxY = std::numeric_limits<qreal>::lowest();
    for (const QPointF& point: localPolygon) {
        minX = std::min(minX, point.x());
        maxX = std::max(maxX, point.x());
        minY = std::min(minY, point.y());
        maxY = std::max(maxY, point.y());
    }

    if (maxX - minX < kPointEpsilonMeters || maxY - minY < kPointEpsilonMeters) {
        errorString = tr("Survey polygon is too small to split into quadrants.");
        return {};
    }

    
    const int regions = qMax(1, _regionCount);

    if (regions == 1) {
        return { QuadrantInfo{
            tr("Survey"),
            QStringLiteral("survey"),
            surveyItem->surveyAreaPolygon()->coordinateList()
        } };
    }
    const qreal cellWidth  = (maxX - minX) / regions;
    const qreal cellHeight = (maxY - minY) / regions;

    QList<QuadrantInfo> quadrants;

    for (int row = 0; row < regions; ++row) {
        for (int col = 0; col < regions; ++col) {

            QRectF rect(
                QPointF(minX + col * cellWidth,
                        minY + row * cellHeight),
                QSizeF(cellWidth, cellHeight));

            QList<QPointF> clipped = _clipPolygonToRect(localPolygon, rect);

            closeAndClean(clipped);

            if (clipped.count() >= 3) {
                QuadrantInfo q;
                q.name = QString("Cell %1,%2").arg(row + 1).arg(col + 1);
                q.fileSuffix = QString("r%1c%2").arg(row + 1).arg(col + 1);
                q.polygon = _pointsToCoordinates(clipped, origin);
                quadrants.append(q);
            }
        }
    }


    if (quadrants.isEmpty()) {
        errorString = tr("No non-empty quadrants were generated.");
    }

    return quadrants;
}

QList<QPointF> CustomSurveyManager::_clipPolygonToRect(const QList<QPointF>& polygon, const QRectF& rect) const
{
    enum class Boundary {
        Left,
        Right,
        Bottom,
        Top,
    };

    auto inside = [rect](const QPointF& point, Boundary boundary) {
        switch (boundary) {
        case Boundary::Left:
            return point.x() >= rect.left() - kPointEpsilonMeters;
        case Boundary::Right:
            return point.x() <= rect.right() + kPointEpsilonMeters;
        case Boundary::Bottom:
            return point.y() >= rect.top() - kPointEpsilonMeters;
        case Boundary::Top:
            return point.y() <= rect.bottom() + kPointEpsilonMeters;
        }
        return false;
    };

    auto intersection = [rect](const QPointF& start, const QPointF& end, Boundary boundary) {
        qreal value = 0.0;
        qreal t = 0.0;
        switch (boundary) {
        case Boundary::Left:
            value = rect.left();
            t = qFuzzyIsNull(end.x() - start.x()) ? 0.0 : (value - start.x()) / (end.x() - start.x());
            return QPointF(value, start.y() + t * (end.y() - start.y()));
        case Boundary::Right:
            value = rect.right();
            t = qFuzzyIsNull(end.x() - start.x()) ? 0.0 : (value - start.x()) / (end.x() - start.x());
            return QPointF(value, start.y() + t * (end.y() - start.y()));
        case Boundary::Bottom:
            value = rect.top();
            t = qFuzzyIsNull(end.y() - start.y()) ? 0.0 : (value - start.y()) / (end.y() - start.y());
            return QPointF(start.x() + t * (end.x() - start.x()), value);
        case Boundary::Top:
            value = rect.bottom();
            t = qFuzzyIsNull(end.y() - start.y()) ? 0.0 : (value - start.y()) / (end.y() - start.y());
            return QPointF(start.x() + t * (end.x() - start.x()), value);
        }
        return end;
    };

    auto clipAgainst = [&](const QList<QPointF>& input, Boundary boundary) {
        QList<QPointF> output;
        if (input.isEmpty()) {
            return output;
        }

        QPointF start = input.last();
        bool startInside = inside(start, boundary);
        for (const QPointF& end: input) {
            const bool endInside = inside(end, boundary);
            if (endInside) {
                if (!startInside) {
                    appendUnique(output, intersection(start, end, boundary));
                }
                appendUnique(output, end);
            } else if (startInside) {
                appendUnique(output, intersection(start, end, boundary));
            }
            start = end;
            startInside = endInside;
        }
        closeAndClean(output);
        return output;
    };

    QList<QPointF> output = polygon;
    output = clipAgainst(output, Boundary::Left);
    output = clipAgainst(output, Boundary::Right);
    output = clipAgainst(output, Boundary::Bottom);
    output = clipAgainst(output, Boundary::Top);
    return output;
}

QList<QGeoCoordinate> CustomSurveyManager::_pointsToCoordinates(const QList<QPointF>& points, const QGeoCoordinate& origin) const
{
    QList<QGeoCoordinate> coordinates;
    coordinates.reserve(points.count());
    for (const QPointF& point: points) {
        QGeoCoordinate coordinate;
        QGCGeo::convertNedToGeo(point.y(), point.x(), 0.0, origin, coordinate);
        coordinates.append(coordinate);
    }
    return coordinates;
}

QVariantList CustomSurveyManager::_coordinatesToVariantList(const QList<QGeoCoordinate>& coordinates) const
{
    QVariantList variants;
    for (const QGeoCoordinate& coordinate: coordinates) {
        variants.append(QVariant::fromValue(coordinate));
    }
    return variants;
}

QJsonArray CustomSurveyManager::_coordinatesToJson(const QList<QGeoCoordinate>& coordinates) const
{
    QJsonValue jsonValue;
    JsonHelper::saveGeoCoordinateArray(coordinates, false /* writeAltitude */, jsonValue);
    return jsonValue.toArray();
}

QJsonObject CustomSurveyManager::_metadataForItem(QObject* item) const
{
    QJsonObject customObject;
    customObject[kCustomSurveyVersionKey] = 1;
    customObject[kCustomSurveyTypeKey] = QString::fromLatin1(kCustomSurveyTypeValue);
    customObject[kSourceSequenceKey] = _sequenceNumber(item);

    QString errorString;
    const QList<QuadrantInfo> quadrants = _buildQuadrants(item, errorString);
    QJsonArray quadrantArray;
    for (const QuadrantInfo& quadrant: quadrants) {
        QJsonObject quadrantObject;
        quadrantObject[kQuadrantNameKey] = quadrant.name;
        quadrantObject[kQuadrantPolygonKey] = _coordinatesToJson(quadrant.polygon);
        quadrantObject[kQuadrantAreaKey] = _polygonArea(quadrant.polygon);
        quadrantArray.append(quadrantObject);
    }
    customObject[kQuadrantsKey] = quadrantArray;

    return customObject;
}

double CustomSurveyManager::_polygonArea(const QList<QGeoCoordinate>& coordinates) const
{
    if (coordinates.count() < 3) {
        return 0.0;
    }

    const QGeoCoordinate origin = coordinates.first();
    QList<QPointF> points;
    for (const QGeoCoordinate& coordinate: coordinates) {
        double north = 0.0;
        double east = 0.0;
        double down = 0.0;
        QGCGeo::convertGeoToNed(coordinate, origin, north, east, down);
        points.append(QPointF(east, north));
    }

    double area = 0.0;
    for (int i = 0; i < points.count(); ++i) {
        const QPointF& point = points[i];
        const QPointF& nextPoint = points[(i + 1) % points.count()];
        area += point.x() * nextPoint.y() - nextPoint.x() * point.y();
    }
    return std::abs(area) / 2.0;
}

bool CustomSurveyManager::_applyPolygon(QGCMapPolygon* mapPolygon, const QList<QGeoCoordinate>& coordinates)
{
    if (!mapPolygon || coordinates.count() < 3) {
        return false;
    }

    mapPolygon->beginReset();
    mapPolygon->clear();
    mapPolygon->appendVertices(coordinates);
    mapPolygon->endReset();
    return true;
}

bool CustomSurveyManager::_writePlanFile(const QJsonDocument& planDocument, const QString& filename)
{
    QFile file(filename);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text)) {
        _setLastError(tr("Unable to write %1: %2").arg(filename, file.errorString()));
        return false;
    }

    if (file.write(planDocument.toJson(QJsonDocument::Indented)) < 0) {
        _setLastError(tr("Unable to write %1: %2").arg(filename, file.errorString()));
        return false;
    }

    return true;
}

void CustomSurveyManager::_setLastError(const QString& errorString)
{
    if (_lastError != errorString) {
        _lastError = errorString;
        emit lastErrorChanged();
    }
}
