#include "ChimeraPlanner.h"
#include "PlanMasterController.h"
#include "MissionController.h"
#include "SurveyComplexItem.h"
#include "SimpleMissionItem.h"
#include "CameraCalc.h"
#include "QGCMapPolygon.h"
#include "QGCCorePlugin.h"
#include "QmlObjectListModel.h"
#include <QQmlApplicationEngine>
#include <QQmlComponent>
#include <QQmlContext>
#include <QCoreApplication>
#include <QTimer>
#include <QTemporaryDir>
#include <QDebug>
#include <QFile>
#include <QJsonDocument>
#include <QJsonArray>
#include <stdexcept>

namespace {
void check(bool passed, const QString& message)
{
    if (!passed) throw std::runtime_error(message.toStdString());
}
}

void schedulePlannerSmoke(QQmlApplicationEngine* engine, ChimeraPlanner* planner)
{
    if (!qEnvironmentVariableIsSet("QGC_PLANNER_SMOKE")) return;
    QTimer::singleShot(2500, engine, [engine, planner] {
        try {
            QTemporaryDir folder;
            check(folder.isValid(), "Temporary directory");
            PlanMasterController plan;
            plan.start();
            auto* mission = plan.missionController();
            QGeoCoordinate origin(38.313294, -76.55221, 40);
            mission->insertTakeoffItem(origin, 1);
            auto* survey = qobject_cast<SurveyComplexItem*>(mission->insertComplexMissionItem("Survey", origin, 2));
            check(survey, "Survey creation");
            survey->cameraCalc()->setDistanceMode(QGroundControlQmlGlobal::AltitudeModeRelative);
            survey->cameraCalc()->distanceToSurface()->setRawValue(40);
            QList<QGeoCoordinate> polygon{origin, origin.atDistanceAndAzimuth(100, 90),
                origin.atDistanceAndAzimuth(141.421, 45), origin.atDistanceAndAzimuth(100, 0)};
            survey->surveyAreaPolygon()->setPath(polygon);
            survey->setWizardMode(false);
            planner->setSetting(survey, "enabled", true);
            planner->setSetting(survey, "yaw", 37.0);
            planner->setSetting(survey, "pitch", -50.0);
            const auto preview = planner->preview(survey);
            check(preview["error"].toString().isEmpty(), preview["error"].toString());
            check(preview["path"].toList().size() >= 2, "Preview flight path");
            check(preview["offset"].toDouble() > 10, "Oblique footprint offset");
            check(!QGCCorePlugin::instance()->missionUploadError(&plan).isEmpty(), "Source upload guard");
            const auto source = plan.saveToJson().object();
            const QString exported = folder.filePath("export.plan");
            const auto error = planner->exportPlan(survey, exported);
            check(error.isEmpty(), error);
            QFile output(exported);
            check(output.open(QIODevice::ReadOnly), "Export exists");
            const auto saved = QJsonDocument::fromJson(output.readAll()).object();
            const auto items = saved["mission"].toObject()["items"].toArray();
            check(items.size() > 5, "Expanded mission");
            int sequence = 1, waypointCount = 0;
            for (const auto& entry : items) {
                auto item = entry.toObject();
                check(item["doJumpId"].toInt() == sequence++, "Sequential IDs");
                if (item["command"].toInt() == MAV_CMD_NAV_WAYPOINT) {
                    check(item["params"].toArray()[3].toDouble() == 37, "Fixed waypoint yaw");
                    ++waypointCount;
                }
            }
            check(waypointCount >= 2, "Generated waypoints");
            plan.loadFromFile(exported);
            check(QGCCorePlugin::instance()->missionUploadError(&plan).isEmpty(), "Export upload guard");
            auto* model = mission->visualItems();
            SimpleMissionItem* checkpoint = nullptr;
            for (int i = 1; i < model->count(); ++i) {
                auto* simple = qobject_cast<SimpleMissionItem*>(model->get(i));
                if (simple && simple->command() == MAV_CMD_NAV_WAYPOINT) { checkpoint = simple; break; }
            }
            check(checkpoint, "Editable checkpoint");
            mission->setCurrentPlanViewSeqNum(checkpoint->sequenceNumber(), true);
            check(planner->insertBlock(mission, "inspection").isEmpty(), "Inspection insertion");
            int roi = 0, clear = 0;
            bool pitchRestored = false;
            for (int i = 1; i < model->count(); ++i) {
                auto* simple = qobject_cast<SimpleMissionItem*>(model->get(i));
                if (!simple) continue;
                roi += simple->command() == MAV_CMD_DO_SET_ROI_LOCATION;
                clear += simple->command() == MAV_CMD_DO_SET_ROI_NONE;
                if (roi && clear && simple->command() == MAV_CMD_DO_MOUNT_CONTROL) {
                    pitchRestored = simple->missionItem().param1() == -50;
                }
                if (simple->command() == MAV_CMD_NAV_WAYPOINT) {
                    check(simple->missionItem().param4() == 37, "Restored yaw");
                }
            }
            check(roi == 1 && clear >= 1 && pitchRestored, "ROI pair and restored pitch");
            QString loadError;
            check(mission->load(source["mission"].toObject(), loadError), loadError);
            planner->loadSettings(&plan, source);
            survey = nullptr;
            model = mission->visualItems();
            for (int i = 0; i < model->count(); ++i) {
                if (auto* found = qobject_cast<SurveyComplexItem*>(model->get(i))) survey = found;
            }
            check(survey && planner->settings(survey)["yaw"].toDouble() == 37, "Settings round trip");
            for (const QString& filename : {"SurveyEditor.qml", "SurveyOverlay.qml", "WorkflowTools.qml"}) {
                QQmlComponent component(engine, QUrl("qrc:/Planner/" + filename));
                check(component.isReady(), component.errorString());
                auto* widget = component.create();
                check(widget, component.errorString());
                if (filename == "WorkflowTools.qml") widget->setProperty("controller", QVariant::fromValue(mission));
                else widget->setProperty("survey", QVariant::fromValue(survey));
                delete widget;
            }
            qInfo() << "PLANNER_SMOKE_PASS: preview, export, reload, yaw, ROI insertion, persistence, QML";
            QCoreApplication::exit(0);
        } catch (const std::exception& error) {
            qCritical() << "PLANNER_SMOKE_FAIL:" << error.what();
            QCoreApplication::exit(1);
        }
    });
}
