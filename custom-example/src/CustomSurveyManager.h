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
#include "Fact.h"   // full type required: exposed via Q_PROPERTY(Fact*) below (moc needs a complete pointer metatype)

class PlanMasterController;
class SurveyComplexItem;
class VisualMissionItem;
class MissionItem;
class FactMetaData;
class QJsonDocument;

class CustomSurveyManager : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QString customSurveyName READ customSurveyName CONSTANT)
    Q_PROPERTY(QString lastError        READ lastError        NOTIFY lastErrorChanged)

    // Per-photo-point capture settings for the "individual waypoints" export.
    // These reuse the SAME Fact-backed UI controls as the core mission panels:
    //  - captureGimbalPitch uses units "gimbal-degrees", so the field shows +90
    //    for straight-down while rawValue()/the plan record -90 (identical to
    //    CameraSection's gimbal pitch). See CameraSection.FactMetaData.json.
    //  - captureGimbalYaw   uses units "deg" (same as CameraSection gimbal yaw).
    //  - captureHold        uses units "secs" (same as NAV_WAYPOINT param1 "Hold").
    Q_PROPERTY(Fact* captureGimbalPitch READ captureGimbalPitch CONSTANT)
    Q_PROPERTY(Fact* captureGimbalYaw   READ captureGimbalYaw   CONSTANT)
    Q_PROPERTY(Fact* captureHold        READ captureHold        CONSTANT)

public:
    explicit CustomSurveyManager(QObject* parent = nullptr);

    /// User-visible name shown in the Plan view "Pattern" menu. Must differ from
    /// the stock Survey name so PlanView can dispatch custom creation.
    QString customSurveyName() const { return QStringLiteral("Custom Survey"); }
    QString lastError()        const { return _lastError; }

    Fact* captureGimbalPitch() const { return _captureGimbalPitchFact; }
    Fact* captureGimbalYaw()   const { return _captureGimbalYawFact; }
    Fact* captureHold()        const { return _captureHoldFact; }

    // ---- Custom-survey flagging -------------------------------------------
    Q_INVOKABLE bool markCustomSurvey(QObject* item);
    Q_INVOKABLE bool isCustomSurvey  (QObject* item) const;

    // ---- Individual-waypoint export toggle (persisted per survey) ----------
    Q_INVOKABLE bool exportAsWaypoints   (QObject* item);
    Q_INVOKABLE void setExportAsWaypoints(QObject* item, bool enabled);

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
    // Writes one .plan per region, each region a stock survey ComplexItem.
    Q_INVOKABLE bool saveRegionPlans(QObject* item, const QString& folder);

    // Writes one .plan per region, but each region's survey is EXPANDED into
    // individual simple mission items. At every discrete photo point the survey
    // would capture at (driven via Hover & Capture), we emit the 4-item sequence:
    //   1. NAV_WAYPOINT  (yaw = transect angle, hold 0)          -> arrive
    //   2. DO_MOUNT_CONTROL (pitch/yaw from the capture panel)   -> aim gimbal
    //   3. IMAGE_START_CAPTURE                                   -> take photo
    //   4. NAV_WAYPOINT  (yaw = transect angle, hold from panel) -> dwell
    Q_INVOKABLE bool saveRegionWaypointPlans(QObject* item, const QString& folder);

    // ---- Plan-file hooks (called from CustomPlugin) -----------------------
    void decorateMissionJson(PlanMasterController* controller, QJsonObject& missionJson);
    void restoreFromPlanJson(PlanMasterController* controller, const QJsonObject& planJson);

signals:
    void lastErrorChanged();
    void customSurveyChanged(QObject* item);
    // Emitted when the whole survey is translated (center-handle drag): the
    // division is rigid, so the visuals can shift the existing region outlines
    // and transects by this geodesic delta instead of recomputing them.
    void customSurveyTranslated(QObject* item, double distance, double azimuth);

private:
    struct ControlState {
        int                     regionCount = 1;        ///< 1 == undivided
        QGeoCoordinate          center;                 ///< interior apex the division rays radiate from
        QList<QGeoCoordinate>   edgeVertices;           ///< one control vertex per region, stored as an absolute position; the cut is where the ray center->vertex meets the boundary
        bool                    seeded = false;         ///< true once center/vertices are populated
        QGeoCoordinate          lastPolygonCenter;      ///< survey polygon centroid last seen; used to translate center+vertices when the whole survey is moved
        double                  regionOffset = 0.0;     ///< meters each region is inset inward from its shared (ray) edges to separate neighbors (n>1 only)
        bool                    exportAsWaypoints = false; ///< export this survey expanded into individual mission items instead of a survey ComplexItem
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

    // Re-place each edge control vertex at the midpoint of its ray (center ->
    // boundary), preserving the ray azimuth (so the division is unchanged). Used
    // after a load so the blue handles sit at their proper midpoints even if the
    // saved vertices were left off-midpoint by a prior center move.
    void                  _snapEdgeVerticesToMidpoints(QObject* item, ControlState& state);

    // Translate the center when the whole survey polygon is moved, so the
    // division rides along with the survey. Edge cuts are perimeter fractions
    // and follow the boundary automatically.
    void                  _onSurveyPolygonMoved(QObject* survey);

    // Cast a ray from center at the given bearing (deg) and return where it
    // first meets the survey polygon boundary (the region cut). Invalid if the
    // ray never exits (e.g. center outside the polygon).
    static QGeoCoordinate _rayBoundaryIntersection(const QList<QGeoCoordinate>& polygon, const QGeoCoordinate& center, double azimuthDeg);

    // Region generation. Memoized on an input signature (polygon + control
    // points + offset + count) so the splitter runs once per actual change no
    // matter how many callers ask (map overlays, flight paths, editor list).
    QList<SplitRegion>    _computeRegions(QObject* item, QString& errorString);

    // Cheap signature of the master's transect PARAMETERS (excludes grid angle,
    // which is mirrored live, and the polygon, which is geometry). Used to decide
    // when shadows must be reconfigured — avoids a per-frame save()/load().
    QString               _masterParamSignature(SurveyComplexItem* survey) const;

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

    // Builds a throwaway, fully-configured SurveyComplexItem for one region from
    // the master survey JSON (transects recomputed for the sub-polygon). Caller
    // owns the returned object (deleteLater). Returns nullptr on failure.
    SurveyComplexItem*    _buildRegionSurveyObject(PlanMasterController* controller,
                                                   const QJsonObject& masterSurveyJson,
                                                   const QList<QGeoCoordinate>& regionPolygon) const;

    // Expands one region survey into the per-photo-point 4-item sequences and
    // returns them as a JSON "items" array (simple items, doJumpId from 1). The
    // transect yaw applied to every waypoint is passed in (survey grid angle).
    QJsonArray            _buildRegionWaypointItems(PlanMasterController* controller,
                                                    const QJsonObject& masterSurveyJson,
                                                    const QList<QGeoCoordinate>& regionPolygon,
                                                    double transectYawDeg,
                                                    int& photoPointCount) const;

    // Serializes one MissionItem (owned/temporary) to a plan "items" JSON object.
    // For coordinate/waypoint items, appends the Altitude/AltitudeMode/
    // AMSLAltAboveTerrain trio the way SimpleMissionItem::save() does.
    QJsonObject           _missionItemToJson(const MissionItem& item, bool withAltitude) const;

    bool                  _writePlanFile(const QJsonDocument& document, const QString& filename);

    void                  _createCaptureFacts();
    void                  _markAllCustomSurveysDirty();

    // Misc
    double                _polygonArea(const QList<QGeoCoordinate>& coordinates) const;
    QVariantList          _coordinatesToVariantList(const QList<QGeoCoordinate>& coordinates) const;
    void                  _setLastError(const QString& errorString);

    QHash<QObject*, ControlState>              _stateBySurvey;     ///< live per-survey control state
    QHash<int, ControlState>                   _pendingState;      ///< restored-by-sequence before the survey object exists
    QSet<QObject*>                             _customSurveyItems; ///< which surveys are custom
    QHash<QObject*, QList<SurveyComplexItem*>> _regionSurveys;     ///< per-master shadow surveys, one per region
    QHash<QObject*, QString>                   _lastParamSig;      ///< last master-parameter signature per master; reconfigure shadows only when it changes
    QHash<QObject*, QList<SplitRegion>>        _cachedRegions;     ///< memoized region result per master
    QHash<QObject*, QString>                   _cachedRegionSig;   ///< input signature the memoized regions were computed from
    ActiveRegionSplitter                       _splitter;          ///< swappable division strategy
    QString                                    _lastError;

    // Capture settings for the individual-waypoint export. Manager-owned Facts so
    // the QML panel can bind the exact same FactControls the core panels use.
    Fact*        _captureGimbalPitchFact = nullptr;   ///< units "gimbal-degrees": UI +90 == raw -90 (straight down)
    Fact*        _captureGimbalYawFact   = nullptr;   ///< units "deg"
    Fact*        _captureHoldFact        = nullptr;   ///< units "secs" (NAV_WAYPOINT hold)
    bool         _suppressCaptureDirty   = false;     ///< true while restoring facts from a plan load, so it doesn't dirty the plan
};
