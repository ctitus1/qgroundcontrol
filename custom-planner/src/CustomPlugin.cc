#include "CustomPlugin.h"
#include "QGCApplication.h"
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QApplicationStatic>
#include "PlanMasterController.h"
#include "MissionController.h"
#include "QmlObjectListModel.h"
#include "ChimeraMissionItem.h"
#include "MissionItem.h"

Q_APPLICATION_STATIC(CustomPlugin, plannerPlugin)

QGCCorePlugin* CustomPlugin::instance()
{
    return plannerPlugin();
}

QQmlApplicationEngine* CustomPlugin::createQmlApplicationEngine(QObject* parent)
{
    auto* engine = QGCCorePlugin::createQmlApplicationEngine(parent);
    engine->rootContext()->setContextProperty("chimeraPlanner", &_planner);
    engine->rootContext()->setContextProperty("chimeraFleet", &_fleet);
#ifdef QGC_PLANNER_SMOKE_TEST
    extern void schedulePlannerSmoke(QQmlApplicationEngine*, ChimeraPlanner*);
    schedulePlannerSmoke(engine, &_planner);
#endif
    return engine;
}

void CustomPlugin::postSaveToMissionJson(PlanMasterController* controller, QJsonObject& mission)
{
    _planner.saveSettings(controller, mission);
    _fleet.save(controller, mission);
}

void CustomPlugin::postLoadFromJson(PlanMasterController* controller, QJsonObject& plan)
{
    _planner.loadSettings(controller, plan);
    _fleet.load(controller, plan);
    if (!_fleet.error().isEmpty()) qgcApp()->showAppMessage(_fleet.error());
    auto* items = controller->missionController()->visualItems();
    for (int i = 1; i < items->count(); ++i) {
        if (auto* block = qobject_cast<ChimeraMissionItem*>(items->get(i)); block && block->splitPart()) block->upgradeFrozenSurvey();
    }
}

QString CustomPlugin::missionUploadError(PlanMasterController* controller) const
{
    if (controller == _fleet.activeController() && !_fleet.error().isEmpty()) return _fleet.error();
    if (_fleet.owns(controller)) return tr("Export the selected drone plan and open it before upload. The master contains multiple missions.");
    auto* items = controller->missionController()->visualItems();
    bool regionStarted = false;
    QObject owner;
    for (int i = 0; i < items->count(); ++i) {
        auto* visual = qobject_cast<VisualMissionItem*>(items->get(i));
        if (auto* block = qobject_cast<ChimeraMissionItem*>(visual)) {
            if (!block->error().isEmpty()) return block->error();
            const auto groupError = block->linkedSurveyError();
            if (!groupError.isEmpty()) return groupError;
            if (block->kind() == "fiducial" && regionStarted)
                return tr("Move fiducial blocks before the first mission arm or vantage marker.");
        }
        QList<MissionItem*> commands;
        visual->appendMissionItems(commands, &owner);
        for (auto* command : commands) {
            if (command->command() == MAV_CMD_NAV_DELAY && (command->param1() == 0 || command->param1() == 1)) regionStarted = true;
        }
        if (_planner.settings(items->get(i))["enabled"].toBool()) {
            return tr("Export and open the editable Chimera mission before upload. This plan contains survey source polygons.");
        }
    }
    return {};
}

ComplexMissionItem* CustomPlugin::createComplexMissionItem(const QString& type, PlanMasterController* controller, bool flyView)
{
    const QStringList kinds{"fiducial", "survey", "vantage", "inspection"};
    for (const auto& kind : kinds) {
        if (type == "chimera-" + kind || type == "Chimera " + kind) {
            return new ChimeraMissionItem(controller, flyView, kind, &_planner);
        }
    }
    return nullptr;
}

QStringList CustomPlugin::complexMissionItemNames(Vehicle*, const QStringList& names)
{
    return QStringList{"Chimera fiducial", "Chimera survey", "Chimera vantage", "Chimera inspection"} + names;
}
