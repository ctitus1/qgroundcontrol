/****************************************************************************
 *
 * (c) 2009-2024 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

/*
 * CustomSurveyManager
 *
 * Plugin-only orchestrator for "Custom Survey" support. A custom survey is a
 * stock QGC SurveyComplexItem that this manager flags and augments with a set
 * of draggable control points. From those control points a swappable
 * RegionSplitter divides the survey polygon into N sub-regions, live, for map
 * preview. On export each sub-region is written out as its own real survey
 * .plan file.
 *
 * IMPORTANT: This lives entirely in the custom plugin. It never modifies QGC
 * core; it integrates only through QGCCorePlugin hooks and a QML context
 * property named "customSurveyManager".
 */

#pragma once

#include <QtCore/QHash>
#include <QtCore/QJsonObject>
#include <QtCore/QList>
#include <QtCore/QObject>
#include <QtCore/QSet>
#include <QtCore/QString>
#include <QtCore/QVariantList>
#include <QtPositioning/QGeoCoordinate>

#include "ActiveSplitter.h"

class PlanMasterController;
class SurveyComplexItem;
class VisualMissionItem;
class QJsonDocument;

class CustomSurveyManager : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QString customSurveyName READ customSurveyName CONSTANT)
    Q_PROPERTY(QString lastError        READ lastError        NOTIFY lastErrorChanged)

public:
    explicit CustomSurveyManager(QObject* parent = nullptr);

    /// User-visible name shown in the Plan view "Pattern" menu. Must differ from
    /// the stock Survey name so PlanView can dispatch custom creation.
    QString customSurveyName() const { return QStringLiteral("Custom Survey"); }
    QString lastError()        const { return _lastError; }

    // ---- Custom-survey flagging -------------------------------------------
    Q_INVOKABLE bool markCustomSurvey(QObject* item);
    Q_INVOKABLE bool isCustomSurvey  (QObject* item) const;

    // ---- Region count / control points (live editing) ---------------------
    Q_INVOKABLE int            regionCount        (QObject* item);
    Q_INVOKABLE void           setRegionCount     (QObject* item, int count);
    Q_INVOKABLE QGeoCoordinate centerControlPoint (QObject* item);
    Q_INVOKABLE QVariantList   edgeControlPoints  (QObject* item);
    Q_INVOKABLE void           setCenterControlPoint(QObject* item, const QGeoCoordinate& coordinate);
    Q_INVOKABLE void           setEdgeControlPoint  (QObject* item, int index, const QGeoCoordinate& coordinate);

    // Inward separation (meters) between adjacent regions; default 0, n>1 only.
    Q_INVOKABLE double         regionOffset       (QObject* item);
    Q_INVOKABLE void           setRegionOffset    (QObject* item, double meters);

    // ---- Regions (computed live from control points) ----------------------
    Q_INVOKABLE QVariantList regionPolygons(QObject* item);

    // Per-region flight paths. Each region is backed by its own SurveyComplexItem
    // (a manager-owned "shadow" survey) so it computes a real transect grid.
    // Returns one entry per region: the transect polyline (list of coordinates).
    Q_INVOKABLE QVariantList regionFlightPaths(QObject* item);

    // ---- Export -----------------------------------------------------------
    Q_INVOKABLE bool saveRegionPlans(QObject* item, const QString& folder);

    // ---- Plan-file hooks (called from CustomPlugin) -----------------------
    void decorateMissionJson(PlanMasterController* controller, QJsonObject& missionJson);
    void restoreFromPlanJson(PlanMasterController* controller, const QJsonObject& planJson);

signals:
    void lastErrorChanged();
    void customSurveyChanged(QObject* item);

private:
    struct ControlState {
        int                     regionCount = 1;        ///< 1 == undivided
        QGeoCoordinate          center;                 ///< interior apex the division rays radiate from
        QList<QGeoCoordinate>   edgeVertices;           ///< one control vertex per region, stored as an absolute position; the cut is where the ray center->vertex meets the boundary
        bool                    seeded = false;         ///< true once center/vertices are populated
        QGeoCoordinate          lastPolygonCenter;      ///< survey polygon centroid last seen; used to translate center+vertices when the whole survey is moved
        double                  regionOffset = 0.0;     ///< meters each region is inset inward from its shared (ray) edges to separate neighbors (n>1 only)
    };

    // Casting / identity helpers
    SurveyComplexItem*    _surveyItem   (QObject* item) const;
    VisualMissionItem*    _visualItem   (QObject* item) const;
    PlanMasterController* _itemController(QObject* item) const;
    int                   _sequenceNumber(QObject* item) const;

    // State access
    void                  _attachSurvey(QObject* survey);
    bool                  _markCustomSurvey(QObject* item, bool setDirty);
    ControlState&         _stateFor(QObject* item);
    void                  _seedControlPoints(QObject* item, ControlState& state, int count);

    // Translate the center when the whole survey polygon is moved, so the
    // division rides along with the survey. Edge cuts are perimeter fractions
    // and follow the boundary automatically.
    void                  _onSurveyPolygonMoved(QObject* survey);

    // Cast a ray from center at the given bearing (deg) and return where it
    // first meets the survey polygon boundary (the region cut). Invalid if the
    // ray never exits (e.g. center outside the polygon).
    static QGeoCoordinate _rayBoundaryIntersection(const QList<QGeoCoordinate>& polygon, const QGeoCoordinate& center, double azimuthDeg);

    // Region generation
    QList<SplitRegion>    _computeRegions(QObject* item, QString& errorString);

    // Keep one shadow SurveyComplexItem per region, in sync with the region
    // polygons (params mirrored from the master for now). These are what render
    // per-region flight paths and, later, carry per-region parameters.
    void                  _syncRegionSurveys(QObject* master, const QList<SplitRegion>& regions);

    // Mission-JSON matching (adapted from the prior working implementation)
    int                   _sequenceNumberFromMissionObject(const QJsonObject& itemObject) const;
    bool                  _isSurveyMissionObject(const QJsonObject& itemObject) const;
    QJsonObject           _metadataForItem(QObject* item);

    // Export helpers (robust: builds each region from a genuine survey object)
    QJsonObject           _buildRegionSurveyJson(PlanMasterController* controller,
                                                 const QJsonObject& masterSurveyJson,
                                                 const QList<QGeoCoordinate>& regionPolygon,
                                                 bool& terrainPending) const;
    bool                  _writePlanFile(const QJsonDocument& document, const QString& filename);

    // Misc
    double                _polygonArea(const QList<QGeoCoordinate>& coordinates) const;
    QVariantList          _coordinatesToVariantList(const QList<QGeoCoordinate>& coordinates) const;
    void                  _setLastError(const QString& errorString);

    QHash<QObject*, ControlState>              _stateBySurvey;     ///< live per-survey control state
    QHash<int, ControlState>                   _pendingState;      ///< restored-by-sequence before the survey object exists
    QSet<QObject*>                             _customSurveyItems; ///< which surveys are custom
    QHash<QObject*, QList<SurveyComplexItem*>> _regionSurveys;     ///< per-master shadow surveys, one per region
    QHash<QObject*, QJsonObject>               _lastMasterJson;    ///< cached master-survey JSON per master; skip shadow reloads when unchanged
    ActiveRegionSplitter                       _splitter;          ///< swappable division strategy
    QString                                    _lastError;
};
