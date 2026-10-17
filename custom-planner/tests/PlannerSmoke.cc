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
#include <QQuickWindow>
#include <QQuickItem>
#include <QEventLoop>
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
            for (auto* command : commands) check(command->command() != MAV_CMD_DO_MOUNT_CONTROL && command->command() != MAV_CMD_DO_SET_ROI_NONE, "No fiducial gimbal command");
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
            survey->clearInspections();
            const auto unsplitPath = survey->preview()["path"].toList();
            const auto splitFrom = unsplitPath[1].value<QGeoCoordinate>(), splitTo = unsplitPath[2].value<QGeoCoordinate>();
            const auto splitPoint = splitFrom.atDistanceAndAzimuth(splitFrom.distanceTo(splitTo)/2, splitFrom.azimuthTo(splitTo));
            check(survey->addSplitPoint(splitPoint).isEmpty(), "Split survey");
            auto* continuation = qobject_cast<ChimeraMissionItem*>(mission->visualItems()->get(4));
            check(continuation && continuation->linkedSurvey() && survey->linkedSurvey() && !survey->splitPart(), "Two native survey parts");
            check(survey->exitCoordinate().distanceTo(splitPoint) < 0.01 && continuation->coordinate().distanceTo(splitPoint) < 0.01, "Shared split endpoint");
            check(continuation->exitCoordinate().distanceTo(unsplitPath.last().value<QGeoCoordinate>()) < 0.01, "Original survey end preserved");
            commands.clear(); continuation->appendMissionItems(commands, &owner);
            int firstWaypoint = -1, arm = -1;
            for (int i = 0; i < commands.size(); ++i) {
                auto* c = commands[i];
                if (c->command() == MAV_CMD_NAV_WAYPOINT) { if (firstWaypoint < 0) firstWaypoint = i; check(c->param4() == 37, "Continuation heading"); }
                if (c->command() == MAV_CMD_NAV_DELAY && c->param1() == 0) arm = i;
            }
            check(firstWaypoint >= 0 && arm > firstWaypoint && commands[arm-1]->command() == MAV_CMD_DO_MOUNT_CONTROL, "Return before rearming search");
            check(survey->removeEndSplit().isEmpty(), "Remove split and recover full source survey");
            check(!survey->linkedSurvey() && survey->preview()["path"].toList().size() == unsplitPath.size(), "Unsplit route restored");
            check(survey->addSplitPoint(splitPoint).isEmpty(), "Add split from map coordinate");
            mission->insertSimpleMissionItem(splitPoint.atDistanceAndAzimuth(20, 90), 4);
            auto* vantage = qobject_cast<ChimeraMissionItem*>(mission->insertComplexMissionItem("Chimera vantage", origin, 6));
            commands.clear(); vantage->appendMissionItems(commands, &owner);
            check(commands.first()->command() == MAV_CMD_NAV_DELAY && commands.first()->param1() == 1, "Vantage starts with autonomy marker");
            for (auto* c : commands) check(c->command() != MAV_CMD_DO_MOUNT_CONTROL && c->command() != MAV_CMD_DO_SET_ROI_NONE, "No vantage gimbal command");
            saved = plan.saveToJson().object();
            check(mission->load(saved["mission"].toObject(), error), error);
            check(mission->visualItems()->count() == 7, "Split and manual waypoint round trip");
            survey = qobject_cast<ChimeraMissionItem*>(mission->visualItems()->get(3));
            continuation = qobject_cast<ChimeraMissionItem*>(mission->visualItems()->get(5));
            check(survey && continuation && survey->exitCoordinate().distanceTo(continuation->coordinate()) < 0.01, "Split boundary after reload");
            continuation->setValue("altitude", 45);
            continuation->setValue("yaw", 42);
            continuation->setValue("angle", 25);
            continuation->setValue("overlap", 65);
            check(survey->values() == continuation->values(), "Shared flight and camera settings");
            check(survey->exitCoordinate().distanceTo(continuation->coordinate()) < 0.01, "Regenerated split boundary");
            const auto moved = survey->centerCoordinate().atDistanceAndAzimuth(30, 0);
            continuation->setCoordinate(moved);
            check(survey->surveyAreaPolygon()->coordinateList() == continuation->surveyAreaPolygon()->coordinateList(), "Shared polygon movement");
            const auto editedVertex = survey->surveyAreaPolygon()->vertexCoordinate(0).atDistanceAndAzimuth(15, 270);
            survey->surveyAreaPolygon()->adjustVertex(0, editedVertex);
            QEventLoop polygonEvents; QTimer::singleShot(100, &polygonEvents, &QEventLoop::quit); polygonEvents.exec();
            check(survey->surveyAreaPolygon()->vertexCoordinate(0).distanceTo(editedVertex) < 0.01, "Deferred edits preserve the latest polygon");
            check(survey->surveyAreaPolygon()->coordinateList() == continuation->surveyAreaPolygon()->coordinateList(), "Shared polygon edit");
            survey->moveEndSplit(survey->splitHandles().first().value<QGeoCoordinate>());
            check(survey->exitCoordinate().distanceTo(continuation->coordinate()) < 0.01, "Movable split point");
            check(!survey->removeEndSplit().isEmpty() && mission->visualItems()->count() == 7, "Manual inspection items prevent unsafe merge");
            check(survey->linkedSurveyError().isEmpty(), "Complete linked survey");
            check(continuation->addSplitPoint(continuation->splitHandles().first().value<QGeoCoordinate>()).isEmpty(), "Multiple split points");
            check(continuation->removeEndSplit().isEmpty(), "Remove one of multiple split points");
            saved = plan.saveToJson().object();
            check(mission->load(saved["mission"].toObject(), error), error);
            survey = qobject_cast<ChimeraMissionItem*>(mission->visualItems()->get(3));
            continuation = qobject_cast<ChimeraMissionItem*>(mission->visualItems()->get(5));
            check(survey->linkedSurveyError().isEmpty() && survey->values()["yaw"].toDouble() == 42, "Edited source persists");
            auto legacyMission = saved["mission"].toObject();
            auto legacyItems = legacyMission["items"].toArray();
            int legacyPart = 0;
            for (int i = 0; i < legacyItems.size(); ++i) {
                auto json = legacyItems[i].toObject();
                if (json["complexItemType"].toString() != "chimera-survey") continue;
                auto* part = legacyPart++ == 0 ? survey : continuation;
                commands.clear(); part->appendMissionItems(commands, &owner);
                QJsonArray frozen;
                for (auto* c : commands) if (c->command() == MAV_CMD_NAV_WAYPOINT || c->command() == MAV_CMD_DO_SET_CAM_TRIGG_DIST) { QJsonObject item; c->save(item); frozen.append(item); }
                json["version"] = 2; json["frozenRoute"] = frozen;
                json.remove("surveyGroup"); json.remove("rangeStart"); json.remove("rangeEnd");
                legacyItems[i] = json;
            }
            legacyMission["items"] = legacyItems;
            check(mission->load(legacyMission, error), error);
            survey = qobject_cast<ChimeraMissionItem*>(mission->visualItems()->get(3));
            check(survey->upgradeFrozenSurvey().isEmpty(), "Restore legacy fixed-route parts to shared source");
            check(survey->linkedSurvey() && !survey->splitPart() && survey->linkedSurveyError().isEmpty(), "Restored source is complete");
            context.setContextProperty("object", survey);
            survey->setIsCurrentItem(true);
            mapObject = canvas.create(&context);
            mapWidget = map.createWithInitialProperties({{"map", QVariant::fromValue(mapObject)}}, &context);
            check(mapWidget, map.errorString()); delete mapWidget; delete mapObject;
            for (int itemIndex : {2, 3, 6}) {
                auto* item = qobject_cast<ChimeraMissionItem*>(mission->visualItems()->get(itemIndex));
                context.setContextProperty("missionItem", item);
                context.setContextProperty("availableWidth", 240);
                auto* panel = qobject_cast<QQuickItem*>(editor.create(&context)); check(panel, editor.errorString());
                if (auto* header = panel->findChild<QObject*>("cameraSection")) header->setProperty("checked", true);
                QQuickWindow window;
                window.setColor(QColor("#303030"));
                panel->setParentItem(window.contentItem());
                window.resize(240, 1800); window.show();
                QEventLoop events; QTimer::singleShot(100, &events, &QEventLoop::quit); events.exec();
                window.resize(240, int(panel->implicitHeight()) + 10);
                QTimer::singleShot(100, &events, &QEventLoop::quit); events.exec();
                window.grabWindow().save("/tmp/qgc-panel-" + item->kind() + ".png");
                int fieldCount = 0;
                QList<QQuickItem*> visualItems = panel->childItems();
                for (int visualIndex = 0; visualIndex < visualItems.size(); ++visualIndex) {
                    auto* field = visualItems[visualIndex];
                    visualItems.append(field->childItems());
                    if (!field->objectName().startsWith("field_")) continue;
                    ++fieldCount;
                    const auto position = field->mapToItem(panel, QPointF());
                    check(position.x() >= -1 && position.x() + field->width() <= 241 && field->width() >= 90, "Input fits narrow panel: " + field->objectName() + QString(" x=%1 width=%2 panel=%3").arg(position.x()).arg(field->width()).arg(panel->width()));
                }
                check(fieldCount == (item->kind() == "survey" ? 14 : 6), "All input fields rendered: " + item->kind() + " count=" + QString::number(fieldCount));
                check(window.grabWindow().save("/tmp/qgc-panel-" + item->kind() + ".png"), "Panel screenshot");
                delete panel;
            }
            mission->insertComplexMissionItem("Chimera fiducial", origin, 7);
            check(!QGCCorePlugin::instance()->missionUploadError(&plan).isEmpty(), "Fiducial order validation");
            auto* invalidSurvey = qobject_cast<ChimeraMissionItem*>(mission->insertComplexMissionItem("Chimera survey", origin, 8));
            invalidSurvey->setValue("pitch", -5);
            check(invalidSurvey->readyForSaveState() == VisualMissionItem::NotReadyForSaveData, "Invalid geometry blocked");
            qInfo() << "PLANNER_SMOKE_PASS: blocks, no-gimbal markers, linked split edit/remove/resume, persistence, Fact panels, map handles";
            QCoreApplication::exit(0);
        } catch (const std::exception& error) {
            qCritical() << "PLANNER_SMOKE_FAIL:" << error.what(); QCoreApplication::exit(1);
        }
    });
}
