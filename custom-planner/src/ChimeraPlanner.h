#pragma once

#include <QObject>
#include <QVariantMap>
#include <QJsonObject>
#include <QPointer>
#include <QGeoCoordinate>
#include "CoverageGeometry.h"

class PlanMasterController;
class SurveyComplexItem;
class MissionItem;

class ChimeraPlanner : public QObject
{
    Q_OBJECT
    Q_PROPERTY(bool showSurveyDetails MEMBER _showSurveyDetails NOTIFY displayChanged)
    Q_PROPERTY(bool showContext MEMBER _showContext NOTIFY displayChanged)
    Q_PROPERTY(QObject* placementSource READ placementSource NOTIFY displayChanged)
    Q_PROPERTY(QString placementPrompt READ placementPrompt NOTIFY displayChanged)
    Q_PROPERTY(QGeoCoordinate pauseCoordinate READ pauseCoordinate NOTIFY displayChanged)
public:
    QObject* placementSource() const { return _placementSource.data(); }
    QString placementPrompt() const;
    QGeoCoordinate pauseCoordinate() const { return _pause; }
    Q_INVOKABLE void beginInspection(QObject* source);
    Q_INVOKABLE void beginSplit(QObject* source);
    Q_INVOKABLE void cancelPlacement();
    Q_INVOKABLE bool handleMapClick(const QGeoCoordinate& coordinate, QObject* controller);
    explicit ChimeraPlanner(QObject* parent = nullptr) : QObject(parent) {}
    Q_INVOKABLE QVariantMap settings(QObject* item) const;
    Q_INVOKABLE void setSetting(QObject* item, const QString& key, const QVariant& value);
    Q_INVOKABLE QVariantMap preview(QObject* item);
    Q_INVOKABLE QString exportPlan(QObject* item, const QString& filename);
    void saveSettings(PlanMasterController* controller, QJsonObject& mission);
    void loadSettings(PlanMasterController* controller, const QJsonObject& plan);
    bool buildSurvey(SurveyComplexItem* source, QList<MissionItem*>& items, QObject* owner,
                     QVariantList& path, QString& error, SurveyComplexItem* cached = nullptr, bool configure = true);
    QVariantMap preview(SurveyComplexItem* survey, const QVariantList& path) const;
signals:
    void displayChanged();
    void settingsChanged(QObject* item);
private:
    bool _showSurveyDetails=false, _showContext=false;
    QPointer<QObject> _placementSource;
    QString _placementMode;
    QGeoCoordinate _pause;
    CoverageGeometry::Footprint footprint(SurveyComplexItem* survey) const;
};
