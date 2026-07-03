/****************************************************************************
 *
 * Rewritten CustomSurveyManager
 * Radial partition implementation (work in progress)
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

#include <cmath>

namespace {

constexpr const char* kItemsKey = "items";
constexpr const char* kTypeKey = "type";
constexpr const char* kTypeComplexItemValue = "ComplexItem";
constexpr const char* kComplexItemTypeKey = "complexItemType";
constexpr const char* kSurveyComplexItemTypeValue = "survey";
constexpr const char* kCustomSurveyKey = "customSurvey";

}



int
CustomSurveyManager::_regionCountForSurvey(
    QObject* survey) const
{
    return _regionCountBySurvey.value(survey, 1);
}

void
CustomSurveyManager::_setRegionCountForSurvey(
    QObject* survey,
    int count)
{
    if (!survey)
        return;

    _regionCountBySurvey[survey] =
        qMax(1, count);

    emit customSurveyChanged(survey);
}

CustomSurveyManager::CustomSurveyManager(QObject* parent)
    : QObject(parent)
{
}



struct ClipEdge {
    QPointF a;
    QPointF b;
};

static inline double cross(const QPointF& a,const QPointF& b)
{
    return a.x()*b.y()-a.y()*b.x();
}







static QPointF lineIntersection(
    const QPointF& p1,
    const QPointF& p2,
    const QPointF& q1,
    const QPointF& q2)
{
    QPointF r = p2-p1;
    QPointF s = q2-q1;

    double t =
        cross(q1-p1,s) /
        cross(r,s);

    return p1 + t*r;
}

static bool insideEdge(
    const QPointF& p,
    const ClipEdge& e)
{
    return cross(e.b-e.a,p-e.a) >= -1e-9;
}


static QList<QPointF> clipAgainstEdge(
    const QList<QPointF>& poly,
    const ClipEdge& edge)
{
    QList<QPointF> out;

    if (poly.isEmpty())
        return out;

    QPointF S = poly.last();

    for (const QPointF& E : poly) {

        bool Sin = insideEdge(S, edge);
        bool Ein = insideEdge(E, edge);

        if (Ein) {
            if (!Sin)
                out.append(lineIntersection(S,E,edge.a,edge.b));

            out.append(E);

        } else if (Sin) {

            out.append(lineIntersection(S,E,edge.a,edge.b));
        }

        S = E;
    }

    return out;
}

static double signedArea(const QList<QPointF>& poly)
{
    double area = 0.0;

    for (int i=0; i<poly.size(); ++i) {

        const QPointF& a = poly[i];
        const QPointF& b = poly[(i+1)%poly.size()];

        area += a.x()*b.y() - b.x()*a.y();
    }

    return area * 0.5;
}

static void ensureCCW(QList<QPointF>& poly)
{
    if (signedArea(poly) < 0.0)
        std::reverse(poly.begin(), poly.end());
}

static QList<QPointF> clipAgainstConvexPolygon(
    QList<QPointF> poly,
    const QList<QPointF>& clip)
{
    for (int i=0; i<clip.size(); ++i) {

        ClipEdge edge{
            clip[i],
            clip[(i+1)%clip.size()]
        };

        poly = clipAgainstEdge(poly, edge);

        if (poly.isEmpty())
            break;
    }

    return poly;
}

static QList<QPointF> makeSectorPolygon(
    const QPointF& center,
    double startAngle,
    double endAngle,
    double radius)
{
    QList<QPointF> poly;

    const double R = radius * 8.0;

    QPointF d0(
        std::cos(startAngle),
        std::sin(startAngle));

    QPointF d1(
        std::cos(endAngle),
        std::sin(endAngle));

    QPointF mid =
        QPointF(
            std::cos((startAngle+endAngle)*0.5),
            std::sin((startAngle+endAngle)*0.5));

    poly
        << center
        << center + R*d0
        << center + R*mid
        << center + R*d1;

    ensureCCW(poly);

    return poly;
}

static double polygonRadius(
    const QList<QPointF>& poly,
    const QPointF& center)
{
    double r = 0.0;

    for (const QPointF& p : poly) {
        r = std::max(
            r,
            std::hypot(
                p.x()-center.x(),
                p.y()-center.y()));
    }

    return r * 2.0;
}


static QPointF polygonCentroid(
    const QList<QPointF>& poly)
{
    if (poly.isEmpty())
        return QPointF();

    double area = 0.0;
    double cx = 0.0;
    double cy = 0.0;

    for (int i = 0; i < poly.size(); ++i) {

        const QPointF& p0 = poly[i];
        const QPointF& p1 = poly[(i + 1) % poly.size()];

        const double a =
            p0.x() * p1.y() -
            p1.x() * p0.y();

        area += a;
        cx += (p0.x() + p1.x()) * a;
        cy += (p0.y() + p1.y()) * a;
    }

    area *= 0.5;

    if (std::abs(area) < 1e-9) {
        QPointF avg;

        for (const QPointF& p : poly)
            avg += p;

        return avg / double(poly.size());
    }

    return QPointF(
        cx / (6.0 * area),
        cy / (6.0 * area));
}



// ---------------------------------------------------------------------
// Geometry
// ---------------------------------------------------------------------



QList<CustomSurveyManager::RegionInfo>
CustomSurveyManager::_buildRegions(
    QObject* item,
    QString& errorString) const
{
    errorString.clear();

    SurveyComplexItem* surveyItem = _surveyItem(item);

    if (!surveyItem) {
        errorString = tr("Invalid survey.");
        return {};
    }

    QList<QGeoCoordinate> geo =
        surveyItem->surveyAreaPolygon()->coordinateList();

    if (geo.size() < 3) {
        errorString = tr("Survey polygon invalid.");
        return {};
    }

    const int regionCount =
        _regionCountForSurvey(item);

    if (regionCount == 1) {

        return {{
            tr("Survey"),
            QStringLiteral("survey"),
            geo
        }};
    }

    //------------------------------------------------------------------
    // Convert to local coordinates
    //------------------------------------------------------------------

    const QGeoCoordinate origin = geo.first();

    QList<QPointF> local;

    for (const auto& c : geo) {

        double n,e,d;

        QGCGeo::convertGeoToNed(
            c,
            origin,
            n,e,d);

        local.append(QPointF(e,n));
    }

    QPointF center =
        polygonCentroid(local);

    double radius =
        polygonRadius(
            local,
            center);

    QList<RegionInfo> regions;

    const double step =
        2.0*M_PI /
        double(regionCount);

    for(int i=0;i<regionCount;i++){

        double a0 = i*step;
        double a1 = (i+1)*step;

        QList<QPointF> wedge =
            makeSectorPolygon(
                center,
                a0,
                a1,
                radius);

        ensureCCW(wedge);

        QList<QPointF> clipped =
            clipAgainstConvexPolygon(
                local,
                wedge);

        if(clipped.size()<3)
            continue;

        RegionInfo q;

        q.name =
            QString("Region %1")
            .arg(i+1);

        q.fileSuffix =
            QString("region_%1")
            .arg(i+1);

        q.polygon =
            _pointsToCoordinates(
                clipped,
                origin);

        regions.append(q);
    }

    return regions;
}


// ---------------------------------------------------------------------
// Coordinate conversion helpers
// ---------------------------------------------------------------------

QList<QGeoCoordinate>
CustomSurveyManager::_pointsToCoordinates(
    const QList<QPointF>& points,
    const QGeoCoordinate& origin) const
{
    QList<QGeoCoordinate> out;

    out.reserve(points.size());

    for (const QPointF& p : points) {

        QGeoCoordinate c;

        QGCGeo::convertNedToGeo(
            p.y(),
            p.x(),
            0.0,
            origin,
            c);

        out.append(c);
    }

    return out;
}

QVariantList
CustomSurveyManager::_coordinatesToVariantList(
    const QList<QGeoCoordinate>& coords) const
{
    QVariantList out;

    for (const auto& c : coords)
        out << QVariant::fromValue(c);

    return out;
}

QJsonArray
CustomSurveyManager::_coordinatesToJson(
    const QList<QGeoCoordinate>& coords) const
{
    QJsonValue value;

    JsonHelper::saveGeoCoordinateArray(
        coords,
        false,
        value);

    return value.toArray();
}








// ---------------------------------------------------------------------
// Mission JSON helper functions
// ---------------------------------------------------------------------

int
CustomSurveyManager::_sequenceNumberFromMissionObject(
    const QJsonObject& itemObject) const
{
    static constexpr const char* kDoJumpIdKey = "doJumpId";
    static constexpr const char* kTransectStyleKey = "TransectStyleComplexItem";
    static constexpr const char* kTransectItemsKey = "Items";

    if (itemObject.contains(kDoJumpIdKey))
        return itemObject[kDoJumpIdKey].toInt(-1);

    const QJsonObject transect =
        itemObject[kTransectStyleKey].toObject();

    const QJsonArray items =
        transect[kTransectItemsKey].toArray();

    if (!items.isEmpty()) {
        return items.first()
            .toObject()[kDoJumpIdKey]
            .toInt(-1);
    }

    return -1;
}

bool
CustomSurveyManager::_isSurveyMissionObject(
    const QJsonObject& itemObject) const
{
    static constexpr const char* kTypeKey = "type";
    static constexpr const char* kComplexItemTypeKey = "complexItemType";

    return
        itemObject[kTypeKey].toString() ==
            QStringLiteral("ComplexItem") &&

        itemObject[kComplexItemTypeKey].toString() ==
            QStringLiteral("survey");
}

bool
CustomSurveyManager::_findMissionObjectBySequence(
    const QJsonArray& items,
    int sequenceNumber,
    int& itemIndex) const
{
    for (int i = 0; i < items.size(); ++i) {

        const QJsonObject obj =
            items[i].toObject();

        if (_isSurveyMissionObject(obj) &&
            _sequenceNumberFromMissionObject(obj) == sequenceNumber)
        {
            itemIndex = i;
            return true;
        }
    }

    itemIndex = -1;
    return false;
}


// ---------------------------------------------------------------------
// Utility helpers
// ---------------------------------------------------------------------

double
CustomSurveyManager::_polygonArea(
    const QList<QGeoCoordinate>& coords) const
{
    if (coords.size() < 3)
        return 0.0;

    const QGeoCoordinate origin = coords.first();

    QList<QPointF> local;

    for (const auto& c : coords) {

        double n,e,d;

        QGCGeo::convertGeoToNed(
            c,
            origin,
            n,e,d);

        local.append(QPointF(e,n));
    }

    double area = 0.0;

    for (int i=0;i<local.size();++i) {

        const QPointF& p0 = local[i];
        const QPointF& p1 =
            local[(i+1)%local.size()];

        area +=
            p0.x()*p1.y() -
            p1.x()*p0.y();
    }

    return std::abs(area)*0.5;
}

bool
CustomSurveyManager::_applyPolygon(
    QGCMapPolygon* polygon,
    const QList<QGeoCoordinate>& coords)
{
    if (!polygon || coords.size() < 3)
        return false;

    polygon->beginReset();
    polygon->clear();
    polygon->appendVertices(coords);
    polygon->endReset();

    return true;
}

bool
CustomSurveyManager::_writePlanFile(
    const QJsonDocument& doc,
    const QString& filename)
{
    QFile file(filename);

    if (!file.open(
            QIODevice::WriteOnly |
            QIODevice::Truncate |
            QIODevice::Text)) {

        _setLastError(
            tr("Unable to write %1")
            .arg(filename));

        return false;
    }

    file.write(
        doc.toJson(
            QJsonDocument::Indented));

    return true;
}

void
CustomSurveyManager::_setLastError(
    const QString& err)
{
    if (_lastError == err)
        return;

    _lastError = err;

    emit lastErrorChanged();
}



// ---------------------------------------------------------------------
// Core manager helpers
// ---------------------------------------------------------------------

SurveyComplexItem*
CustomSurveyManager::_surveyItem(QObject* item) const
{
    return qobject_cast<SurveyComplexItem*>(item);
}

VisualMissionItem*
CustomSurveyManager::_visualItem(QObject* item) const
{
    return qobject_cast<VisualMissionItem*>(item);
}

PlanMasterController*
CustomSurveyManager::_itemPlanMasterController(QObject* item) const
{
    if (auto* visual = _visualItem(item))
        return visual->masterController();

    return nullptr;
}

int
CustomSurveyManager::_sequenceNumber(QObject* item) const
{
    if (auto* visual = _visualItem(item))
        return visual->sequenceNumber();

    return -1;
}



// =========================================================
// Per-survey region API
// =========================================================

int
CustomSurveyManager::regionCountForSurvey(QObject* survey) const
{
    if (!survey)
        return 1;

    auto it =
        _regionCountBySurvey.find(survey);

    if (it != _regionCountBySurvey.end())
        return it.value();

    return 1;
}

void
CustomSurveyManager::setRegionCountForSurvey(
    QObject* survey,
    int count)
{
    if (!survey)
        return;

    count = qMax(1,count);

    _attachSurvey(survey);

    _regionCountBySurvey[survey] = count;

    emit regionCountChanged();
    emit customSurveyChanged(survey);
}



void
CustomSurveyManager::_attachSurvey(QObject* survey)
{
    if (!survey)
        return;

    if (_regionCountBySurvey.contains(survey))
        return;

    const int seq = _sequenceNumber(survey);

    int count = 1;

    auto it = _pendingRegionCounts.find(seq);
    if (it != _pendingRegionCounts.end()) {
        count = it.value();
        _pendingRegionCounts.erase(it);
    }

    _regionCountBySurvey.insert(survey, count);

    connect(
        survey,
        &QObject::destroyed,
        this,
        [this](QObject* obj)
        {
            _regionCountBySurvey.remove(obj);
        });
}

bool
CustomSurveyManager::markCustomSurvey(QObject* item)
{
    return _markCustomSurvey(item, true);
}

bool
CustomSurveyManager::_markCustomSurvey(
    QObject* item,
    bool setDirty)
{
    SurveyComplexItem* survey = _surveyItem(item);

    if (!survey) {
        _setLastError(
            tr("Only Survey items may become custom surveys."));
        return false;
    }

    if (!_customSurveyItems.contains(survey)) {

        _customSurveyItems.insert(survey);

        connect(
            survey,
            &QObject::destroyed,
            this,
            [this](QObject* obj)
            {
                _customSurveyItems.remove(obj);
            });

        if(_regionCountForSurvey(survey)<1)
        {
            _setRegionCountForSurvey(
                survey,
                1);
        }

        emit customSurveyChanged(survey);
    }

    if (setDirty) {

        if (auto* master =
                _itemPlanMasterController(survey))
        {
            master->setDirty(true);
        }
    }

    return true;
}

bool
CustomSurveyManager::isCustomSurvey(QObject* item) const
{
    if (!item)
        return false;

    
    return
        _customSurveyItems.contains(
            _surveyItem(item));
}



// ---------------------------------------------------------------------
// Region access (QML)
// ---------------------------------------------------------------------

QVariantList
CustomSurveyManager::regionPolygons(QObject* item)
{
    
    QString error;

    _attachSurvey(item);

    QList<RegionInfo> regions =
        _buildRegions(item, error);

    if (!error.isEmpty())
        _setLastError(error);
    else
        _setLastError(QString());

    QVariantList out;

    for (const RegionInfo& region : regions) {

        QVariantMap map;

        map["name"] =
            region.name;

        map["fileSuffix"] =
            region.fileSuffix;

        map["vertexCount"] =
            region.polygon.size();

        map["area"] =
            _polygonArea(region.polygon);

        map["polygon"] =
            _coordinatesToVariantList(
                region.polygon);

        out << map;
    }

    return out;
}

QJsonObject
CustomSurveyManager::_metadataForItem(QObject* item) const
{
    QJsonObject obj;

    obj["version"] = 1;
    obj["regionCount"] =
        _regionCountForSurvey(item);
    obj["sourceSequence"] =
        _sequenceNumber(item);

    return obj;
}








// =========================================================
// PATCH 6 TODO
//
// Remaining work:
//
// SurveyItemEditor.qml
//
//      regionCount()
//          ↓
//
//      selected survey
//          ↓
//
//      sequence number
//          ↓
//
//      _regionCountBySurvey
//
// SurveyMapVisual.qml
//
//      same migration.
//
// =========================================================

// PATCH 1 END




// =====================================================================

// =====================================================================
// REGION IMPLEMENTATION ROADMAP
//
// Plugin-only implementation.
// Do NOT modify QGroundControl core classes.
//
// Remaining implementation order:
//
// [ ] 1. Per-survey region storage
//       - Replace global /*removed_global_regionCount*/ usage
//       - Introduce sequenceNumber -> regionCount map
//       - Default missing entries to 1
//
// [ ] 2. Region accessors
//       - regionCount(QObject*)
//       - setRegionCount(QObject*, int)
//
// [ ] 3. Serialization
//       - decorateMissionJson()
//       - restoreFromPlanJson()
//       - Persist regionCount with each survey
//
// [ ] 4. Export
//       - saveRegionPlans()
//       - Exported surveys become regionCount = 1
//
// [ ] 5. Geometry
//       - Finish radial region clipping
//       - Remove duplicate vertices
//       - Remove tiny regions
//       - Ensure CCW winding
//
// [ ] 6. UI
//       - SurveyItemEditor uses selected survey regionCount
//       - SurveyMapVisual refreshes selected survey only
//
// [ ] 7. Cleanup
//       - Remove obsolete rectangle helpers
//       - Rename remaining quadrant references
//       - Final compile/review
//
// =====================================================================


// ---------------------------------------------------------------------
// TODO: Restore full implementations.
// These stubs allow the project to link while the radial implementation
// is completed.
// ---------------------------------------------------------------------



int
CustomSurveyManager::_surveyOrdinal(QObject* survey) const
{
    if (!survey)
        return 1;

    QList<QObject*> surveys =
        _customSurveyItems.values();

    std::sort(
        surveys.begin(),
        surveys.end(),
        [this](QObject* a, QObject* b)
        {
            return _sequenceNumber(a) <
                   _sequenceNumber(b);
        });

    for (int i = 0; i < surveys.size(); ++i) {
        if (surveys[i] == survey)
            return i + 1;
    }

    return 1;
}

bool
CustomSurveyManager::saveRegionPlans(
    QObject* planMasterControllerObject,
    QObject* item,
    const QString& folder)
{
    PlanMasterController* planMasterController =
        qobject_cast<PlanMasterController*>(planMasterControllerObject);

    SurveyComplexItem* surveyItem =
        _surveyItem(item);

    if (!planMasterController || !surveyItem) {
        _setLastError(tr("Unable to save region plans."));
        return false;
    }

    if (!isCustomSurvey(surveyItem)) {
        _setLastError(tr("Only custom surveys may be exported."));
        return false;
    }

    QDir outputDir(folder);

    if (!outputDir.exists()) {
        _setLastError(
            tr("Output folder does not exist: %1").arg(folder));
        return false;
    }

    QString errorString;

    QList<RegionInfo> regions =
        _buildRegions(surveyItem, errorString);

    if (!errorString.isEmpty()) {
        _setLastError(errorString);
        return false;
    }

    if (regions.isEmpty()) {
        _setLastError(tr("No regions generated."));
        return false;
    }

    QGCMapPolygon* surveyPolygon =
        surveyItem->surveyAreaPolygon();

    const QList<QGeoCoordinate> originalPolygon =
        surveyPolygon->coordinateList();

    const bool wasDirty =
        planMasterController->dirty();

    QString currentPlan =
        planMasterController->currentPlanFile();

    QString baseName =
        QFileInfo(
            currentPlan.isEmpty()
                ? QStringLiteral("custom-survey.plan")
                : currentPlan).completeBaseName();

    int savedCount = 0;

    const int surveyIndex =
        _surveyOrdinal(surveyItem);

    for (const RegionInfo& region : regions) {

        if (!_applyPolygon(
                surveyPolygon,
                region.polygon))
            continue;

        QJsonDocument plan =
            planMasterController->saveToJson();

        //------------------------------------------------------------
        // Exported regions become independent custom surveys.
        // Reset their region count to 1 so reopening the plan does
        // not immediately regenerate additional regions.
        //------------------------------------------------------------

        QJsonObject root = plan.object();

        if (root.contains("mission")) {

            QJsonObject mission =
                root["mission"].toObject();

            QJsonArray items =
                mission["items"].toArray();

            int missionIndex = -1;

            if (_findMissionObjectBySequence(
                    items,
                    _sequenceNumber(surveyItem),
                    missionIndex))
            {
                QJsonObject item =
                    items[missionIndex].toObject();

                QJsonObject custom =
                    item["customSurvey"].toObject();

                custom["regionCount"] = 1;

                _setRegionCountForSurvey(
                    surveyItem,
                    1);

                // This exported plan now represents only one region.
                custom["sourceSequence"] = 0;
                custom["regionIndex"] = savedCount;

                item["customSurvey"] = custom;

                items[missionIndex] = item;
            }

            mission["items"] = items;
            root["mission"] = mission;

            plan = QJsonDocument(root);
        }

        const QString filename =
            outputDir.absoluteFilePath(
                QString(
                    "custom-survey_%1-region_%2.plan")
                    .arg(surveyIndex)
                    .arg(savedCount + 1));

        if (_writePlanFile(plan, filename))
            ++savedCount;
    }

    _applyPolygon(
        surveyPolygon,
        originalPolygon);

    planMasterController->setDirty(wasDirty);

    _setLastError(
        tr("Saved %1 region plan(s).")
            .arg(savedCount));

    return savedCount > 0;
}

void
CustomSurveyManager::decorateMissionJson(
    PlanMasterController* planMasterController,
    QJsonObject& missionJson)
{
    if (!planMasterController || _customSurveyItems.isEmpty())
        return;

    static constexpr const char* kItemsKey = "items";
    static constexpr const char* kCustomSurveyKey = "customSurvey";

    QHash<int, QObject*> customItems;

    for (QObject* item : std::as_const(_customSurveyItems)) {

        if (!item)
            continue;

        if (_itemPlanMasterController(item) != planMasterController)
            continue;

        const int seq = _sequenceNumber(item);

        if (seq >= 0)
            customItems.insert(seq, item);
    }

    QJsonArray items = missionJson[kItemsKey].toArray();

    for (int i = 0; i < items.size(); ++i) {

        QJsonObject obj = items[i].toObject();

        if (!_isSurveyMissionObject(obj))
            continue;

        QObject* custom =
            customItems.value(
                _sequenceNumberFromMissionObject(obj),
                nullptr);

        if (!custom)
            continue;

        _attachSurvey(custom);

        QJsonObject metadata =
            _metadataForItem(custom);

        metadata["regionCount"] =
            _regionCountBySurvey.value(
                custom,
                1);

        obj[kCustomSurveyKey] =
            metadata;

        items[i] = obj;
    }

    missionJson[kItemsKey] = items;
}

void
CustomSurveyManager::restoreFromPlanJson(
    PlanMasterController* planMasterController,
    const QJsonObject& planJson)
{
    if (!planMasterController)
        return;

    static constexpr const char* kMissionKey = "mission";
    static constexpr const char* kItemsKey = "items";
    static constexpr const char* kCustomSurveyKey = "customSurvey";

    const QJsonObject mission =
        planJson[kMissionKey].toObject();

    const QJsonArray items =
        mission[kItemsKey].toArray();

    QSet<int> sequences;

    for (const QJsonValue& value : items) {

        const QJsonObject obj = value.toObject();

        if (!obj.contains(kCustomSurveyKey))
            continue;

        const int seq =
            _sequenceNumberFromMissionObject(obj);

        if (seq >= 0)
            sequences.insert(seq);
    }

    if (sequences.isEmpty())
        return;

    QmlObjectListModel* visualItems =
        planMasterController
            ->missionController()
            ->visualItems();

    for (int i = 0; i < visualItems->count(); ++i) {

        QObject* item = visualItems->get(i);

        if (!sequences.contains(
                _sequenceNumber(item)))
            continue;

        _markCustomSurvey(
            item,
            false);

        
        int itemIndex = -1;

        if (!_findMissionObjectBySequence(
                items,
                _sequenceNumber(item),
                itemIndex))
        {
            continue;
        }

        const QJsonObject mission =
            items[itemIndex].toObject();


        const QJsonObject custom =
            mission["customSurvey"].toObject();

        _setRegionCountForSurvey(
            item,
            custom["regionCount"].toInt(1));
    }
}

