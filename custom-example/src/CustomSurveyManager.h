/****************************************************************************
 *
 * (c) 2009-2024 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

/*
 * Custom survey manager.
 *
 * Owns all plugin-only survey state including:
 *   - custom survey identification
 *   - per-survey region counts
 *   - region generation
 *   - save/load/export
 */


#pragma once

#include <QtCore/QJsonArray>
#include <QtCore/QJsonDocument>
#include <QtCore/QJsonObject>
#include <QtCore/QObject>
#include <QtCore/QPointF>
#include <QtCore/QRectF>
#include <QtCore/QSet>
#include <QtCore/QVariantList>
#include <QtPositioning/QGeoCoordinate>

class PlanMasterController;
class QGCMapPolygon;
class SurveyComplexItem;
class VisualMissionItem;

class CustomSurveyManager : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QString customSurveyName READ customSurveyName CONSTANT)
    Q_PROPERTY(QString lastError READ lastError NOTIFY lastErrorChanged)

public:
    explicit CustomSurveyManager(QObject* parent = nullptr);

    QString customSurveyName() const { return QStringLiteral("custom"); }
    QString lastError() const { return _lastError; }



    Q_INVOKABLE int regionCountForSurvey(QObject* survey) const;
    Q_INVOKABLE void setRegionCountForSurvey(QObject* survey, int count);

    Q_INVOKABLE bool markCustomSurvey(QObject* item);
    Q_INVOKABLE bool isCustomSurvey(QObject* item) const;
    Q_INVOKABLE QVariantList regionPolygons(QObject* item);
    Q_INVOKABLE bool saveRegionPlans(QObject* planMasterController, QObject* item, const QString& folder);

    void decorateMissionJson(PlanMasterController* planMasterController, QJsonObject& missionJson);
    void restoreFromPlanJson(PlanMasterController* planMasterController, const QJsonObject& planJson);

signals:
    void lastErrorChanged();
    void regionCountChanged();
    void customSurveyChanged(QObject* item);

private:
    struct RegionInfo {
        QString name;
        QString fileSuffix;
        QList<QGeoCoordinate> polygon;
    };


    int _regionCountForSurvey(QObject* survey) const;

    void _setRegionCountForSurvey(QObject* survey, int count);

    // Attach a runtime Survey QObject to the per-survey
    // region bookkeeping. If a pending value was restored
    // from JSON, transfer it into the runtime map.
    void _attachSurvey(QObject* survey);


    bool _markCustomSurvey(QObject* item, bool setDirty);
    SurveyComplexItem* _surveyItem(QObject* item) const;
    VisualMissionItem* _visualItem(QObject* item) const;
    PlanMasterController* _itemPlanMasterController(QObject* item) const;
    int _sequenceNumber(QObject* item) const;
    int _sequenceNumberFromMissionObject(const QJsonObject& itemObject) const;
    bool _isSurveyMissionObject(const QJsonObject& itemObject) const;
    bool _findMissionObjectBySequence(const QJsonArray& items, int sequenceNumber, int& itemIndex) const;

    // Returns the 1-based ordinal of this custom survey
    // within the current mission.
    int _surveyOrdinal(QObject* survey) const;

    QList<RegionInfo> _buildRegions(QObject* item, QString& errorString) const;
    QList<QPointF> _clipPolygonToRect(const QList<QPointF>& polygon, const QRectF& rect) const;
    QList<QGeoCoordinate> _pointsToCoordinates(const QList<QPointF>& points, const QGeoCoordinate& origin) const;
    QVariantList _coordinatesToVariantList(const QList<QGeoCoordinate>& coordinates) const;
    QJsonArray _coordinatesToJson(const QList<QGeoCoordinate>& coordinates) const;
    QJsonObject _metadataForItem(QObject* item) const;
    double _polygonArea(const QList<QGeoCoordinate>& coordinates) const;
    bool _applyPolygon(QGCMapPolygon* mapPolygon, const QList<QGeoCoordinate>& coordinates);
    bool _writePlanFile(const QJsonDocument& planDocument, const QString& filename);
    void _setLastError(const QString& errorString);


    QHash<QObject*, int> _regionCountBySurvey;

// Pending region counts loaded from JSON before
// Survey QObject instances have been reconstructed.
QHash<int, int> _pendingRegionCounts;


    QSet<QObject*> _customSurveyItems;
    QString _lastError;
};
