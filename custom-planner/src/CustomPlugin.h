#pragma once

#include "QGCCorePlugin.h"
#include "ChimeraPlanner.h"

class CustomPlugin : public QGCCorePlugin
{
    Q_OBJECT
public:
    explicit CustomPlugin(QObject* parent = nullptr) : QGCCorePlugin(parent), _planner(this) {}
    static QGCCorePlugin* instance();
    QQmlApplicationEngine* createQmlApplicationEngine(QObject* parent) override;
    void postSaveToMissionJson(PlanMasterController* controller, QJsonObject& mission) override;
    void postLoadFromJson(PlanMasterController* controller, QJsonObject& plan) override;
    QString missionUploadError(PlanMasterController* controller) const override;
private:
    ChimeraPlanner _planner;
};
