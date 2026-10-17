#pragma once

#include "QGCCorePlugin.h"
#include "ChimeraPlanner.h"
#include "FleetPlanner.h"

class CustomPlugin : public QGCCorePlugin
{
    Q_OBJECT
public:
    explicit CustomPlugin(QObject* parent = nullptr) : QGCCorePlugin(parent), _planner(this), _fleet(this) {}
    static QGCCorePlugin* instance();
    QQmlApplicationEngine* createQmlApplicationEngine(QObject* parent) override;
    void postSaveToMissionJson(PlanMasterController* controller, QJsonObject& mission) override;
    void postLoadFromJson(PlanMasterController* controller, QJsonObject& plan) override;
    QString missionUploadError(PlanMasterController* controller) const override;
    ComplexMissionItem* createComplexMissionItem(const QString& type, PlanMasterController* controller, bool flyView) override;
    QStringList complexMissionItemNames(Vehicle* vehicle, const QStringList& names) override;
private:
    ChimeraPlanner _planner;
    FleetPlanner _fleet;
};
