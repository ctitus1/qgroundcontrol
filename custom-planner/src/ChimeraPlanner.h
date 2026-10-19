#pragma once

#include <QObject>
#include <QVariantMap>
#include <QJsonObject>
#include "CoverageGeometry.h"

class PlanMasterController;
class SurveyComplexItem;
class MissionItem;

class ChimeraPlanner : public QObject
{
    Q_OBJECT
public:
    explicit ChimeraPlanner(QObject* parent = nullptr) : QObject(parent) {}
    Q_INVOKABLE QVariantMap settings(QObject* item) const;
    Q_INVOKABLE void setSetting(QObject* item, const QString& key, const QVariant& value);
    Q_INVOKABLE QVariantMap preview(QObject* item);
    Q_INVOKABLE QString exportMission(QObject* item, const QString& filename);
    Q_INVOKABLE QString exportPlan(QObject* item, const QString& filename);
    void saveSettings(PlanMasterController* controller, QJsonObject& mission);
    void loadSettings(PlanMasterController* controller, const QJsonObject& plan);
    bool buildSurvey(SurveyComplexItem* source, QList<MissionItem*>& items, QObject* owner,
                     QVariantList& path, QString& error, SurveyComplexItem* cached = nullptr, bool configure = true);
    QVariantMap preview(SurveyComplexItem* survey, const QVariantList& path) const;
signals:
    void settingsChanged(QObject* item);
private:
    CoverageGeometry::Footprint footprint(SurveyComplexItem* survey) const;
};
