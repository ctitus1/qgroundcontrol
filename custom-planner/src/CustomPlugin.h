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
    ComplexMissionItem* createComplexMissionItem(const QString& type, PlanMasterController* controller, bool flyView, const QString& file = QString()) override;
    QVariantList complexMissionItemNames(Vehicle* vehicle) override;
private:
    ChimeraPlanner _planner;
    FleetPlanner _fleet;
};
