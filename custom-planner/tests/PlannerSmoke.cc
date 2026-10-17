#include "ChimeraPlanner.h"
#include "ChimeraMissionItem.h"
#include "PlanMasterController.h"
#include "MissionController.h"
#include "MissionItem.h"
#include "QGCMapPolygon.h"
#include "QGCCorePlugin.h"
#include "QmlObjectListModel.h"
#include <QQmlApplicationEngine>
#include <QQmlComponent>
#include <QQmlContext>
#include <QCoreApplication>
#include <QTimer>
#include <QJsonArray>
#include <QJsonDocument>
#include <stdexcept>

namespace {
void check(bool passed, const QString& message)
{
    if (!passed) throw std::runtime_error(message.toStdString());
}
}
void schedulePlannerSmoke(QQmlApplicationEngine* engine, ChimeraPlanner*)
{
    if (!qEnvironmentVariableIsSet("QGC_PLANNER_SMOKE")) return;
    QTimer::singleShot(2500, engine, [engine] {
        try {
            PlanMasterController plan; plan.start();
            auto* mission = plan.missionController();
            QGeoCoordinate origin(38.313294, -76.55221, 40);
            mission->insertTakeoffItem(origin, 1);
            auto* fiducial = qobject_cast<ChimeraMissionItem*>(mission->insertComplexMissionItem("Chimera fiducial", origin, 2));
            auto* survey = qobject_cast<ChimeraMissionItem*>(mission->insertComplexMissionItem("Chimera survey", origin, 3));
            check(fiducial && survey, "Native block creation");
            fiducial->setValue("delay", 45); fiducial->setValue("yaw", 23);
            survey->setValue("yaw", 37); survey->setValue("pitch", -50);
            check(survey->error().isEmpty(), survey->error());
            check(survey->preview()["offset"].toDouble() > 10, "Offset path");
            const auto checkpoint = survey->preview()["path"].toList()[1].value<QGeoCoordinate>();
            survey->addInspection(1);
            survey->setInspectionPoint(0, 1, "latitude", checkpoint.atDistanceAndAzimuth(20, 0).latitude());
            survey->setInspectionPoint(0, 1, "yaw", 90);
            survey->addInspectionPoint(0, true);
            survey->addInspectionPoint(0, false);
            check(survey->error().isEmpty(), survey->error());
            QObject owner;
            QList<MissionItem*> commands;
            fiducial->appendMissionItems(commands, &owner);
            check(commands.last()->command() == MAV_CMD_NAV_DELAY && commands.last()->param1() == 45, "Fiducial delay until");
            for (auto* command : commands) if (command->command() == MAV_CMD_NAV_WAYPOINT)
                check(command->param1() == 0 && command->param4() == 23, "Fiducial waypoint has no hold");
            commands.clear(); survey->appendMissionItems(commands, &owner);
            check(commands.size() == survey->lastSequenceNumber() - survey->sequenceNumber() + 1, "Sequence count");
            int seq = survey->sequenceNumber(), roiCount = 0, armCount = 0;
            bool inspecting = false, returned = false;
            for (int i = 0; i < commands.size(); ++i) {
                auto* c = commands[i];
                check(c->sequenceNumber() == seq++, "Sequential command IDs");
                if (c->command() == MAV_CMD_DO_SET_ROI_LOCATION) { inspecting = true; ++roiCount; }
                if (c->command() == MAV_CMD_DO_SET_ROI_NONE && inspecting) { inspecting = false; returned = true; }
                if (c->command() == MAV_CMD_NAV_DELAY && c->param1() == 0) {
                    ++armCount;
                    check(i > 0 && commands[i-1]->command() == MAV_CMD_DO_MOUNT_CONTROL && commands[i-1]->param1() == -50, "Pitch before mission arm");
                }
                if (c->command() == MAV_CMD_NAV_WAYPOINT && !inspecting) {
                    check(c->param4() == 37, "Survey and return heading");
                    if (returned) {
                        check(QGeoCoordinate(c->param5(), c->param6()).distanceTo(checkpoint) < 0.01, "Return checkpoint");
                        returned = false;
                    }
                }
            }
            check(roiCount == 2 && armCount == 2, "Multiple ROI targets and survey resume");
            auto before = survey->inspections()[0].toMap()["points"].toList()[0].toMap();
            survey->setCoordinate(survey->centerCoordinate().atDistanceAndAzimuth(30, 90));
            auto after = survey->inspections()[0].toMap()["points"].toList()[0].toMap();
            check(QGeoCoordinate(before["latitude"].toDouble(), before["longitude"].toDouble()).distanceTo(QGeoCoordinate(after["latitude"].toDouble(), after["longitude"].toDouble())) > 29, "Move inspection with survey");
            auto saved = plan.saveToJson().object();
            QString error;
            check(mission->load(saved["mission"].toObject(), error), error);
            check(mission->visualItems()->count() == 4, "Block count after reload");
            survey = qobject_cast<ChimeraMissionItem*>(mission->visualItems()->get(3));
            check(survey && survey->inspections().size() == 1 && survey->values()["yaw"].toDouble() == 37, "Native persistence");
            QQmlContext context(engine->rootContext());
            context.setContextProperty("missionItem", survey);
            context.setContextProperty("availableWidth", 420);
            context.setContextProperty("object", survey);
            QQmlComponent editor(engine, QUrl("qrc:/Planner/BlockEditor.qml"));
            check(editor.isReady(), editor.errorString());
            auto* widget = editor.create(&context); check(widget, editor.errorString()); delete widget;
            QQmlComponent map(engine, QUrl("qrc:/Planner/BlockMap.qml"));
            check(map.isReady(), map.errorString());
            QQmlComponent canvas(engine);
            canvas.setData("import QtQuick; import QtLocation; Map { width: 600; height: 400; property bool planView: true; property rect centerViewport: Qt.rect(0, 0, 600, 400) }", QUrl());
            auto* mapObject = canvas.create(&context); check(mapObject, canvas.errorString());
            survey->setIsCurrentItem(true);
            auto* mapWidget = map.createWithInitialProperties({{"map", QVariant::fromValue(mapObject)}}, &context);
            check(mapWidget, map.errorString()); delete mapWidget;
            qobject_cast<VisualMissionItem*>(mission->visualItems()->get(2))->setIsCurrentItem(true);
            context.setContextProperty("object", mission->visualItems()->get(2));
            mapWidget = map.createWithInitialProperties({{"map", QVariant::fromValue(mapObject)}}, &context);
            check(mapWidget, map.errorString()); delete mapWidget; delete mapObject;
            check(QGCCorePlugin::instance()->missionUploadError(&plan).isEmpty(), "Native upload validation");
            mission->insertComplexMissionItem("Chimera fiducial", origin, 4);
            check(!QGCCorePlugin::instance()->missionUploadError(&plan).isEmpty(), "Fiducial order validation");
            survey->setValue("pitch", -5);
            check(survey->readyForSaveState() == VisualMissionItem::NotReadyForSaveData, "Invalid geometry blocked");
            qInfo() << "PLANNER_SMOKE_PASS: grouped blocks, delay, offset route, ROI resume, move, persistence, QML";
            QCoreApplication::exit(0);
        } catch (const std::exception& error) {
            qCritical() << "PLANNER_SMOKE_FAIL:" << error.what(); QCoreApplication::exit(1);
        }
    });
}
