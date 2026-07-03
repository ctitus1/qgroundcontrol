/****************************************************************************
 *
 * (c) 2009-2024 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

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
    Q_PROPERTY(int regionCount READ regionCount WRITE setDivisionCount NOTIFY regionCountChanged)

public:
    explicit CustomSurveyManager(QObject* parent = nullptr);

    QString customSurveyName() const { return QStringLiteral("custom"); }
    QString lastError() const { return _lastError; }

    int regionCount() const { return _regionCount; }
    void setDivisionCount(int count) {
        count = qMax(1, count);
        if (_regionCount != count) {
            _regionCount = count;
            emit regionCountChanged();
            for (QObject* item : std::as_const(_customSurveyItems)) {
                emit customSurveyChanged(item);
            }
        }
    }

    Q_INVOKABLE bool markCustomSurvey(QObject* item);
    Q_INVOKABLE bool isCustomSurvey(QObject* item) const;
    Q_INVOKABLE QVariantList quadrantPolygons(QObject* item);
    Q_INVOKABLE bool saveQuadrantPlans(QObject* planMasterController, QObject* item, const QString& folder);

    void decorateMissionJson(PlanMasterController* planMasterController, QJsonObject& missionJson);
    void restoreFromPlanJson(PlanMasterController* planMasterController, const QJsonObject& planJson);

signals:
    void lastErrorChanged();
    void regionCountChanged();
    void customSurveyChanged(QObject* item);

private:
    struct QuadrantInfo {
        QString name;
        QString fileSuffix;
        QList<QGeoCoordinate> polygon;
    };

    bool _markCustomSurvey(QObject* item, bool setDirty);
    SurveyComplexItem* _surveyItem(QObject* item) const;
    VisualMissionItem* _visualItem(QObject* item) const;
    PlanMasterController* _itemPlanMasterController(QObject* item) const;
    int _sequenceNumber(QObject* item) const;
    int _sequenceNumberFromMissionObject(const QJsonObject& itemObject) const;
    bool _isSurveyMissionObject(const QJsonObject& itemObject) const;
    bool _findMissionObjectBySequence(const QJsonArray& items, int sequenceNumber, int& itemIndex) const;
    QList<QuadrantInfo> _buildQuadrants(QObject* item, QString& errorString) const;
    QList<QPointF> _clipPolygonToRect(const QList<QPointF>& polygon, const QRectF& rect) const;
    QList<QGeoCoordinate> _pointsToCoordinates(const QList<QPointF>& points, const QGeoCoordinate& origin) const;
    QVariantList _coordinatesToVariantList(const QList<QGeoCoordinate>& coordinates) const;
    QJsonArray _coordinatesToJson(const QList<QGeoCoordinate>& coordinates) const;
    QJsonObject _metadataForItem(QObject* item) const;
    double _polygonArea(const QList<QGeoCoordinate>& coordinates) const;
    bool _applyPolygon(QGCMapPolygon* mapPolygon, const QList<QGeoCoordinate>& coordinates);
    bool _writePlanFile(const QJsonDocument& planDocument, const QString& filename);
    void _setLastError(const QString& errorString);

    QSet<QObject*> _customSurveyItems;
    QString _lastError;
    int _regionCount = 1;
};
