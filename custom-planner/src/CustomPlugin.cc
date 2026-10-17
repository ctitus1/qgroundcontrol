#include "CustomPlugin.h"
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QApplicationStatic>
#include "PlanMasterController.h"
#include "MissionController.h"
#include "QmlObjectListModel.h"

Q_APPLICATION_STATIC(CustomPlugin, plannerPlugin)

QGCCorePlugin* CustomPlugin::instance()
{
    return plannerPlugin();
}

QQmlApplicationEngine* CustomPlugin::createQmlApplicationEngine(QObject* parent)
{
    auto* engine = QGCCorePlugin::createQmlApplicationEngine(parent);
    engine->rootContext()->setContextProperty("chimeraPlanner", &_planner);
#ifdef QGC_PLANNER_SMOKE_TEST
    extern void schedulePlannerSmoke(QQmlApplicationEngine*, ChimeraPlanner*);
    schedulePlannerSmoke(engine, &_planner);
#endif
    return engine;
}

void CustomPlugin::postSaveToMissionJson(PlanMasterController* controller, QJsonObject& mission)
{
    _planner.saveSettings(controller, mission);
}

void CustomPlugin::postLoadFromJson(PlanMasterController* controller, QJsonObject& plan)
{
    _planner.loadSettings(controller, plan);
}

QString CustomPlugin::missionUploadError(PlanMasterController* controller) const
{
    auto* items = controller->missionController()->visualItems();
    for (int i = 0; i < items->count(); ++i) {
        if (_planner.settings(items->get(i))["enabled"].toBool()) {
            return tr("Export and open the editable Chimera mission before upload. This plan contains survey source polygons.");
        }
    }
    return {};
}
