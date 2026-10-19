#include "ChimeraPlanner.h"
#include "FleetPlanner.h"
#include "CenterOut.h"
#include "SettingsManager.h"
#include "AppSettings.h"
#include "Fact.h"
#include "ChimeraMissionItem.h"
#include "PlanMasterController.h"
#include "MissionController.h"
#include "MissionItem.h"
#include "MissionSettingsItem.h"
#include "SimpleMissionItem.h"
#include "SpeedSection.h"
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
#include <QFile>
#include <stdexcept>

namespace {
void check(bool passed, const QString& message)
{
    if (!passed) throw std::runtime_error(message.toStdString());
}
void checkFleetCreationUi(QQmlApplicationEngine* engine, PlanMasterController* master)
{
    auto* fleet=qobject_cast<FleetPlanner*>(engine->rootContext()->contextProperty("chimeraFleet").value<QObject*>());
    auto* mainWindow=qobject_cast<QQuickWindow*>(engine->rootObjects().first());
    check(QMetaObject::invokeMethod(mainWindow,"showPlanView"),"Show creation panel");
    QEventLoop viewEvents;
    master->loadFromFile("/tmp/qgc-small-ui.plan");
    auto* smallUiMission=master->missionController();
    smallUiMission->setCurrentPlanViewSeqNum(qobject_cast<VisualMissionItem*>(smallUiMission->visualItems()->get(3))->sequenceNumber(),true);
    QTimer::singleShot(300,&viewEvents,&QEventLoop::quit); viewEvents.exec();
    QObject* createButton=nullptr;
    QList<QQuickItem*> creationItems{mainWindow->contentItem()};
    for (int i=0;i<creationItems.size();++i) {
        auto* item=creationItems[i]; creationItems.append(item->childItems());
        if (item->objectName()=="fleetMinimum") item->setProperty("text","18");
        if (item->objectName()=="fleetMaximum") item->setProperty("text","30");
        if (item->objectName()=="fleetCreate") createButton=item;
    }
    qInfo() << "Create small fleet through UI at 18-30 m";
    check(createButton && QMetaObject::invokeMethod(createButton,"clicked"),"Create fleet through its QML button");
    QTimer::singleShot(1500,&viewEvents,&QEventLoop::quit); viewEvents.exec();
    check(fleet->enabled() && fleet->validate().isEmpty(),fleet->validate());
}

}
void schedulePlannerSmoke(QQmlApplicationEngine* engine, ChimeraPlanner*)
{
    if (!qEnvironmentVariableIsSet("QGC_PLANNER_SMOKE")) return;
    SettingsManager::instance()->appSettings()->firstRunPromptIdsShown()->setRawValue("1,2");
    QTimer::singleShot(2500, engine, [engine] {
        try {
            PlanMasterController plan; plan.start();
            auto* mission = plan.missionController();
            SettingsManager::instance()->appSettings()->defaultMissionItemAltitude()->setRawValue(53.);
            QGeoCoordinate origin(38.313294, -76.55221, 40);
            mission->insertTakeoffItem(origin, 1);
            auto* fiducial = qobject_cast<ChimeraMissionItem*>(mission->insertComplexMissionItem("Chimera fiducial", origin, 2));
            auto* survey = qobject_cast<ChimeraMissionItem*>(mission->insertComplexMissionItem("Chimera survey", origin, 3));
            check(fiducial && survey, "Native block creation");
            if (qEnvironmentVariableIsSet("QGC_PLANNER_CREATE_ONLY")) {
                QList<QGeoCoordinate> polygon;
                for (double bearing : {45.,135.,225.,315.}) polygon.append(origin.atDistanceAndAzimuth(20,bearing));
                survey->surveyAreaPolygon()->setPath(polygon); survey->setValue("altitude",18);
                plan.saveToFile("/tmp/qgc-small-ui.plan");
                auto* fleet=qobject_cast<FleetPlanner*>(engine->rootContext()->contextProperty("chimeraFleet").value<QObject*>());
                checkFleetCreationUi(engine,fleet->activeController());
                qInfo() << "PLANNER_CREATE_PASS";
                QCoreApplication::exit(0); return;
            }
            check(survey->values()["altitude"].toDouble() == 53 && survey->values()["speed"].toDouble() == 1 &&
                  survey->values()["pitch"].toDouble() == -70 && survey->values()["overlap"].toDouble() == 25, "New survey defaults");
            check(survey->cameraPresets().size() == 9, "Eight aircraft camera presets and Custom");
            for (int preset = 1; preset <= 8; ++preset) {
                survey->setCameraPreset(preset);
                const auto config = survey->cameraPresets()[preset].toMap();
                const auto matrix = config["camera_matrix"].toMap()["data"].toList();
                check(survey->cameraPreset() == preset && survey->values()["width"] == config["image_width"] &&
                      survey->values()["fx"] == matrix[0] && survey->values()["cy"] == matrix[5] && survey->error().isEmpty(), "Camera preset uses source intrinsics");
            }
            survey->setValue("fx", 1200);
            check(survey->cameraPreset() == 0, "Manual camera edit selects Custom");
            survey->setCameraPreset(5);
            survey->setValue("angle", 19);
            check(survey->values()["yaw"].toDouble() == 19, "Track angle changes heading");
            fiducial->setValue("delay", 45); fiducial->setValue("yaw", 23);
            survey->setValue("yaw", 37); survey->setValue("pitch", -50);
            check(survey->values()["angle"].toDouble() == 37, "Heading changes track angle");
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
            for (auto* c : commands) check(c->command()!=MAV_CMD_DO_SET_CAM_TRIGG_DIST,"Survey has no camera trigger commands");
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
            check(survey->cameraPreset() == 5, "Camera preset persists");
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
            const auto entryRoute=survey->route();
            survey->rotateEntryPoint();
            check(survey->route()!=entryRoute,"Normal survey entry rotates through QGC locations");
            for (int turn=0;turn<3;++turn) survey->rotateEntryPoint();
            check(survey->route()==entryRoute,"Four normal entry rotations restore the route");
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
            vantage->setValue("speed",1.4);
            vantage->setValue("transitSpeed",6);
            auto* secondVantage=qobject_cast<ChimeraMissionItem*>(mission->insertComplexMissionItem("Chimera vantage",origin.atDistanceAndAzimuth(30,90),7));
            secondVantage->setValue("speed",1.6); secondVantage->setValue("transitSpeed",6);
            check(ChimeraPlanner().exportPlan(&plan,"/tmp/qgc-vantage-transits.plan").isEmpty(),"Export survey and vantage transfers");
            QFile vantageExport("/tmp/qgc-vantage-transits.plan"); check(vantageExport.open(QIODevice::ReadOnly),"Read vantage transfers");
            const auto transferItems=QJsonDocument::fromJson(vantageExport.readAll()).object()["mission"].toObject()["items"].toArray();
            double transferSpeed=0; int vantageEntries=0;
            for (int i=0;i<transferItems.size();++i) {
                const auto item=transferItems[i].toObject(); const auto params=item["params"].toArray();
                if (item["command"].toInt()==MAV_CMD_DO_CHANGE_SPEED) transferSpeed=params[1].toDouble();
                if (item["command"].toInt()!=MAV_CMD_NAV_DELAY || params[0].toDouble()!=1) continue;
                check(i>=3 && transferItems[i-2].toObject()["command"].toInt()==MAV_CMD_NAV_WAYPOINT,"Vantage transit reaches destination before arming");
                check(transferItems[i-3].toObject()["params"].toArray()[1].toDouble()==6,"Vantage transfers use rapid speed");
                check(transferSpeed==(vantageEntries==0?1.4:1.6),"Vantage restores its mission speed on arrival");
                ++vantageEntries;
            }
            check(vantageEntries==2,"Both vantage transfers use rapid entry and speed restoration");
            mission->removeVisualItem(7);
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
            continuation->setCameraPreset(2);
            check(survey->cameraPreset() == 2, "Camera preset applies to all linked parts");
            check(survey->values() == continuation->values(), "Shared flight and camera settings");
            check(survey->exitCoordinate().distanceTo(continuation->coordinate()) < 0.01, "Regenerated split boundary");
            const auto moved = survey->centerCoordinate().atDistanceAndAzimuth(30, 0);
            continuation->setCoordinate(moved);
            check(survey->surveyAreaPolygon()->coordinateList() == continuation->surveyAreaPolygon()->coordinateList(), "Shared polygon movement");
            const auto beforeDrag = survey->route();
            int translations = 0;
            const auto dragConnection = QObject::connect(survey, &ChimeraMissionItem::translated, survey, [&](double, double) { ++translations; });
            survey->surveyAreaPolygon()->setCenterDrag(true);
            survey->surveyAreaPolygon()->setCenter(survey->centerCoordinate().atDistanceAndAzimuth(20, 90));
            QEventLoop dragEvents; QTimer::singleShot(100, &dragEvents, &QEventLoop::quit); dragEvents.exec();
            check(translations > 0 && survey->route() == beforeDrag, "Center drag translates visuals without rebuilding commands");
            survey->surveyAreaPolygon()->setCenterDrag(false);
            QObject::disconnect(dragConnection);
            check(survey->route() != beforeDrag && survey->linkedSurveyError().isEmpty(), "Center release rebuilds linked survey commands");
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
            check(survey->linkedSurveyError().isEmpty() && survey->values()["yaw"].toDouble() == 25, "Edited source persists");
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
            check(mapWidget, map.errorString());
            const auto mapItems = mapObject->children();
            check(QMetaObject::invokeMethod(mapWidget, "refresh"), "Refresh pooled map visuals");
            check(mapObject->children() == mapItems, "Unchanged map visuals reuse existing objects");
            check(QMetaObject::invokeMethod(mapWidget, "translate", Q_ARG(QVariant, 10.), Q_ARG(QVariant, 90.)), "Translate pooled map visuals");
            delete mapWidget; delete mapObject;
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
                check(fieldCount == (item->kind() == "survey" ? 15 : 6), "All input fields rendered: " + item->kind() + " count=" + QString::number(fieldCount));
                check(window.grabWindow().save("/tmp/qgc-panel-" + item->kind() + ".png"), "Panel screenshot");
                delete panel;
            }
            mission->insertComplexMissionItem("Chimera fiducial", origin, 7);
            check(!QGCCorePlugin::instance()->missionUploadError(&plan).isEmpty(), "Fiducial order validation");
            auto* invalidSurvey = qobject_cast<ChimeraMissionItem*>(mission->insertComplexMissionItem("Chimera survey", origin, 8));
            invalidSurvey->setValue("pitch", -5);
            check(invalidSurvey->readyForSaveState() == VisualMissionItem::NotReadyForSaveData, "Invalid geometry blocked");
            for (double radius : {5.,20.}) {
                qInfo() << "Small fleet at 18-30 m, radius" << radius;
                PlanMasterController smallMaster; smallMaster.setFlyView(false); smallMaster.start(true);
                auto* smallMission=smallMaster.missionController();
                smallMission->insertTakeoffItem(origin,1);
                smallMission->insertComplexMissionItem("Chimera fiducial",origin,2);
                auto* smallSurvey=qobject_cast<ChimeraMissionItem*>(smallMission->insertComplexMissionItem("Chimera survey",origin,3));
                QList<QGeoCoordinate> smallPolygon;
                for (double bearing : {45.,135.,225.,315.}) smallPolygon.append(origin.atDistanceAndAzimuth(radius,bearing));
                smallSurvey->surveyAreaPolygon()->setPath(smallPolygon);
                smallSurvey->setValue("altitude",18);
                smallMaster.saveToFile("/tmp/qgc-small-ui.plan");
                FleetPlanner smallFleet;
                check(smallFleet.create(smallSurvey,4,18,30,0).isEmpty(),"Create small fleet at 18-30 m");
                smallFleet.synchronize();
                check(smallFleet.validate().isEmpty(),smallFleet.validate());
                for (int count : {2,8,4}) check(smallFleet.setCount(count).isEmpty(),"Resize small low-altitude fleet");
            }
            PlanMasterController fleetMaster; fleetMaster.setFlyView(false); fleetMaster.start(true);
            auto* fleetMission = fleetMaster.missionController();
            fleetMission->insertTakeoffItem(origin, 1);
            fleetMission->insertComplexMissionItem("Chimera fiducial", origin, 2);
            auto* domain = qobject_cast<ChimeraMissionItem*>(fleetMission->insertComplexMissionItem("Chimera survey", origin, 3));
            FleetPlanner fleet;
            check(fleet.minimum()==20 && fleet.maximum()==30,"Default altitude layers are 20-30 m");
            check(fleet.create(domain, 3, 30, 60, 10).isEmpty(), "Create three drone missions");
            fleet.synchronize();
            check(fleet.validate().isEmpty(), fleet.validate());
            check(fleet.separation() == 10 && fleet.drones()[2].toMap()["altitude"].toDouble() == 50, "Maximum vertical gap caps assigned layers");
            for (int drone = 0; drone < 3; ++drone) {
                fleet.setSelected(drone + 1);
                auto* items = fleet.activeController()->missionController()->visualItems();
                auto* droneFiducial=qobject_cast<ChimeraMissionItem*>(items->get(2));
                const auto expectedFiducial=origin.atDistanceAndAzimuth(2,120*drone+180);
                check(droneFiducial->coordinate().distanceTo(expectedFiducial)<0.01,"Fiducial offset follows the drone heading");
                check(droneFiducial->coordinate().altitude()==30+10*drone,"Fiducial uses the survey altitude layer");
                for (int phase = 0; phase < 3; ++phase) {
                    auto* region = qobject_cast<ChimeraMissionItem*>(items->get(3 + phase));
                    check(region && region->fleetRegion() == (drone + phase) % 3 && region->centerOut(), "Rotated region assignment");
                    check(region->values()["yaw"].toDouble() == 120 * drone && region->values()["altitude"].toDouble() == 30 + 10 * drone, "Independent heading and automatic altitude");
                    check(region->route().size() >= 3, "Center-first route generated");
                    check(region->searchCenter().distanceTo(fleet.controlPoints().first().value<QGeoCoordinate>())<0.01,"Every region uses the shared partition center");
                    auto* firstRegion=qobject_cast<ChimeraMissionItem*>(items->get(3));
                    check(region->coordinate().distanceTo(firstRegion->coordinate())<0.01,"Each drone starts all regions at the same camera-offset shared center");
                }
            }
            domain->setValue("yaw",30); fleet.synchronize();
            check(fleet.drones()[0].toMap()["heading"].toDouble()==30 && fleet.drones()[1].toMap()["heading"].toDouble()==150,"Master heading rotates all drone paths");
            fleet.setSelected(1);
            auto* headingRegion=qobject_cast<ChimeraMissionItem*>(fleet.activeController()->missionController()->visualItems()->get(3));
            headingRegion->setValue("yaw",45); fleet.synchronize();
            check(fleet.drones()[0].toMap()["heading"].toDouble()==45 && fleet.drones()[1].toMap()["heading"].toDouble()==150,"Drone editor changes heading independently");
            domain->setValue("yaw",60); fleet.synchronize();
            check(headingRegion->values()["yaw"].toDouble()==75,"Master rotation preserves a drone heading offset");
            headingRegion->setValue("altitude",32); fleet.synchronize();
            check(fleet.drones()[0].toMap()["altitude"].toDouble()==32 && fleet.separation()==8,"Drone altitude edits update the layer gap");
            fleet.setLayers(29,60,10); fleet.setLayers(30,60,10);
            domain->setValue("yaw",0); fleet.synchronize();
            headingRegion->setValue("yaw",0); fleet.synchronize();
            fleet.setProfile(0,"heading",12,true);
            check(fleet.drones()[0].toMap()["heading"].toDouble()==12 && headingRegion->values()["yaw"].toDouble()==0,"Slider edits publish immediately and batch route work");
            QEventLoop sliderEvents; QTimer::singleShot(200,&sliderEvents,&QEventLoop::quit); sliderEvents.exec();
            check(headingRegion->values()["yaw"].toDouble()==12,"Slider route updates run while the drag remains active");
            fleet.setProfile(0,"heading",0);

            fleet.setFiducialRadius(3.5);
            auto* radiusMarker=qobject_cast<ChimeraMissionItem*>(fleet.activeController()->missionController()->visualItems()->get(2));
            check(std::abs(radiusMarker->coordinate().distanceTo(origin)-3.5)<0.01,"Changing radius updates drone positions");
            fleet.setRapidTransitSpeed(6);
            fleet.setSelected(1);
            check(qobject_cast<ChimeraMissionItem*>(fleet.activeController()->missionController()->visualItems()->get(3))->values()["transitSpeed"].toDouble()==6,"Master transit speed reaches survey blocks");
            fleet.setNight(true);
            for (int drone=0;drone<3;++drone) {
                fleet.setSelected(drone+1);
                auto* items=fleet.activeController()->missionController()->visualItems();
                for (int phase=0;phase<3;++phase)
                    check(qobject_cast<ChimeraMissionItem*>(items->get(3+phase))->cameraPreset()==2*(drone+1), "Night mode applies aircraft thermal intrinsics to all regions");
            }
            fleet.setNight(false);
            fleet.setProfile(0,"speed",1.7);
            fleet.setSelected(1);
            for (int phase=0;phase<3;++phase)
                check(qobject_cast<ChimeraMissionItem*>(fleet.activeController()->missionController()->visualItems()->get(3+phase))->values()["speed"].toDouble()==1.7, "Master settings apply across a drone's regions");
            const auto controls=fleet.controlPoints();
            check(controls.size()==4,"Master has a center and one ray handle per region");
            fleet.setPartitionPoint(0,controls[0].value<QGeoCoordinate>().atDistanceAndAzimuth(5,90));
            fleet.setPartitionPoint(1,controls[1].value<QGeoCoordinate>().atDistanceAndAzimuth(8,90));
            fleet.finishPartitionDrag();
            const auto partitionCenter=fleet.controlPoints()[0].value<QGeoCoordinate>();
            const auto originalPolygon=domain->surveyAreaPolygon()->coordinateList();
            auto resizedPolygon=originalPolygon; resizedPolygon[0]=resizedPolygon[0].atDistanceAndAzimuth(4,270);
            domain->surveyAreaPolygon()->setPath(resizedPolygon); fleet.synchronize();
            check(fleet.controlPoints()[0].value<QGeoCoordinate>().distanceTo(partitionCenter)<0.01,"Boundary resizing leaves the partition center fixed");
            domain->surveyAreaPolygon()->setPath(originalPolygon); fleet.synchronize();
            check(fleet.controlPoints()[0].value<QGeoCoordinate>().distanceTo(controls[0].value<QGeoCoordinate>())>4,"Move the partition center independently");
            check(fleet.validate().isEmpty(),fleet.validate());
            fleet.setSelected(1);
            auto* childMission = fleet.activeController()->missionController();
            auto* region = qobject_cast<ChimeraMissionItem*>(childMission->visualItems()->get(3));
            check(region->searchCenter().distanceTo(partitionCenter)<0.01,"Survey origin follows the shared junction when it moves");
            const auto originalRoute=region->route();
            const auto originalYaw=region->values()["yaw"];
            region->rotateEntryPoint();
            check(region->route()!=originalRoute && region->values()["yaw"]==originalYaw,"Rotate fleet entry without changing heading");
            region->rotateEntryPoint();
            check(region->route()==originalRoute,"Entry rotation returns to original route");
            check(region->addSplitPoint(region->splitHandles().first().value<QGeoCoordinate>()).isEmpty(), "Fleet region supports survey interruptions");
            region->rotateEntryPoint();
            auto* rotatedPart=qobject_cast<ChimeraMissionItem*>(childMission->visualItems()->get(4));
            check(rotatedPart->values()["entryPoint"]==region->values()["entryPoint"] && region->linkedSurveyError().isEmpty(),"Entry rotation applies to the complete linked survey");
            auto* manualWaypoint=qobject_cast<SimpleMissionItem*>(childMission->insertSimpleMissionItem(origin.atDistanceAndAzimuth(10, 90), 4));
            fleet.synchronize();
            manualWaypoint->speedSection()->setSpecifyFlightSpeed(true);
            manualWaypoint->speedSection()->flightSpeed()->setRawValue(1.2);
            fleet.synchronize();
            check(manualWaypoint->speedSection()->flightSpeed()->rawValue().toDouble()==1.2,"Manual speed overrides remain editable");
            const int countBefore = childMission->visualItems()->count();
            region->setCameraPreset(2);
            domain->setCoordinate(domain->centerCoordinate().atDistanceAndAzimuth(20, 90));
            fleet.setProfile(0, "heading", 45);
            fleet.setProfile(0, "latitude", origin.latitude() + 0.001);
            fleet.synchronize();
            check(childMission->visualItems()->count() == countBefore && region->cameraPreset() == 2 && region->values()["yaw"].toDouble() == 45, "Shared edits preserve drone inspection items and camera settings");
            check(fleet.validate().isEmpty(), fleet.validate());
            for (int count : {2, 4, 3, 2, 3}) {
                check(fleet.setCount(count).isEmpty(), "Change drone count without locking the master");
                QEventLoop settle; QTimer::singleShot(300, &settle, &QEventLoop::quit); settle.exec();
                check(fleet.drones().size() == count && fleet.validate().isEmpty(), fleet.validate());
            }
            fleet.setSelected(1);
            childMission = fleet.activeController()->missionController();
            region = qobject_cast<ChimeraMissionItem*>(childMission->visualItems()->get(3));
            check(childMission->visualItems()->count() == countBefore && region->cameraPreset() == 2 && region->linkedSurvey(), "Count changes preserve splits, inspection items, and camera overrides");
            check(fleet.drones()[0].toMap()["heading"].toDouble() == 45 && fleet.drones()[1].toMap()["heading"].toDouble() == 120, "Count changes preserve custom headings and redistribute default headings");
            fleet.setSelected(3);
            auto* archivedRegion = qobject_cast<ChimeraMissionItem*>(fleet.activeController()->missionController()->visualItems()->get(3));
            archivedRegion->setValue("speed", 2.5);
            check(fleet.setCount(2).isEmpty(), "Archive a removed drone before save");
            const auto savedControls=fleet.controlPoints();
            auto fleetJson = fleetMaster.saveToJson().object();
            auto fleetMissionJson = fleetJson["mission"].toObject();
            fleet.save(&fleetMaster, fleetMissionJson); fleetJson["mission"] = fleetMissionJson;
            PlanMasterController restoredFleet; restoredFleet.setFlyView(false); restoredFleet.start(true);
            check(restoredFleet.missionController()->load(fleetMissionJson, error), error);
            fleet.attach(&restoredFleet); fleet.load(&restoredFleet, fleetJson);
            check(fleet.enabled() && fleet.validate().isEmpty(), "Master restores all drone missions");
            check(fleet.fiducialRadius()==3.5 && fleet.rapidTransitSpeed()==6,"Master restores fiducial radius and rapid transit speed");
            check(fleet.controlPoints().size()==savedControls.size() && fleet.controlPoints()[0].value<QGeoCoordinate>().distanceTo(savedControls[0].value<QGeoCoordinate>())<0.01,"Master restores partition controls");
            check(fleet.setCount(3).isEmpty(), "Restore an archived drone after reload");
            fleet.setSelected(3);
            archivedRegion = qobject_cast<ChimeraMissionItem*>(fleet.activeController()->missionController()->visualItems()->get(3));
            check(archivedRegion->searchCenter().distanceTo(fleet.controlPoints()[0].value<QGeoCoordinate>())<0.01,"Reloaded regions retain the shared junction");
            check(archivedRegion->values()["speed"].toDouble() == 2.5, "Saved archive preserves removed drone settings");
            fleet.setSelected(1);
            check(fleet.activeController()->missionController()->visualItems()->count() == countBefore, "Master preserves manual drone mission items");
            check(qobject_cast<ChimeraMissionItem*>(fleet.activeController()->missionController()->visualItems()->get(3))->values()["entryPoint"].toInt()==1,"Master reload preserves rotated entry");
            check(fleet.exportDrone(0, "/tmp/qgc-fleet-drone.plan").isEmpty(), "Export a single drone plan");
            check(fleet.exportDrone(0,"/tmp/qgc-fleet-plain.plan",true).isEmpty(),"Export standard QGC mission");
            QFile plainFile("/tmp/qgc-fleet-plain.plan"); check(plainFile.open(QIODevice::ReadOnly),"Read standard mission");
            const auto plainMission=QJsonDocument::fromJson(plainFile.readAll()).object()["mission"].toObject();
            check(!plainMission.contains("chimeraFleet"),"Standard mission has no fleet metadata");
            const auto plainItems=plainMission["items"].toArray();
            QObject exportOwner;
            QList<MissionItem*> uploadItems;
            auto* exportVisuals=fleet.activeController()->missionController()->visualItems();
            for (int i=0;i<exportVisuals->count();++i)
                qobject_cast<VisualMissionItem*>(exportVisuals->get(i))->appendMissionItems(uploadItems,&exportOwner);
            qobject_cast<MissionSettingsItem*>(exportVisuals->get(0))->addMissionEndAction(uploadItems,uploadItems.size(),&exportOwner);
            int expectedIndex=1, transitWaypoints=0, surveyEntries=0;
            double entrySpeed=0;
            for (int i=0;i<plainItems.size();++i) {
                const auto item=plainItems[i].toObject();
                check(item["type"]=="SimpleItem" && !item.contains("complexItemType"),"Standard export uses only simple items");
                check(item["command"].toInt()!=MAV_CMD_DO_SET_CAM_TRIGG_DIST,"Standard export has no camera trigger commands");
                if (item["command"].toInt()==MAV_CMD_DO_CHANGE_SPEED) entrySpeed=item["params"].toArray()[1].toDouble();
                if (item["command"].toInt()==MAV_CMD_NAV_DELAY && item["params"].toArray()[0].toDouble()==0) {
                    check(entrySpeed==1.7,"Every survey restores mission speed after its rapid transfer");
                    ++surveyEntries;
                }
                if (item["command"].toInt()==MAV_CMD_NAV_WAYPOINT) {
                    const auto speedCommand=plainItems[i-1].toObject();
                    check(i>0 && speedCommand["command"].toInt()==MAV_CMD_DO_CHANGE_SPEED,"Plain waypoints have explicit speeds");
                    if (speedCommand["params"].toArray()[1].toDouble()==6) ++transitWaypoints;
                }
                while (expectedIndex<uploadItems.size() && uploadItems[expectedIndex]->command()==MAV_CMD_DO_CHANGE_SPEED) ++expectedIndex;
                if (expectedIndex>=uploadItems.size()) continue;
                QJsonObject expected; uploadItems[expectedIndex]->save(expected);
                expected=QJsonDocument::fromJson(QJsonDocument(expected).toJson()).object();
                if (item["command"]==expected["command"] && item["frame"]==expected["frame"] && item["params"]==expected["params"]) ++expectedIndex;
            }
            check(expectedIndex==uploadItems.size(),"Plain export preserves original commands and adds transit legs");
            check(transitWaypoints>=3 && transitWaypoints==surveyEntries,"Every survey entry has a rapid transfer");
            PlanMasterController plainController; plainController.setFlyView(false); plainController.start(true);
            check(plainController.missionController()->load(plainMission,error),error);
            auto* plainVisuals=plainController.missionController()->visualItems();
            for (int i=1;i<plainVisuals->count();++i)
                check(!qobject_cast<ChimeraMissionItem*>(plainVisuals->get(i)),"Standard mission loads without custom blocks");
            fleet.setProfile(0,"startRegion",1);
            fleet.setSelected(1);
            check(qobject_cast<ChimeraMissionItem*>(fleet.activeController()->missionController()->visualItems()->get(3))->fleetRegion()==1, "Master region assignment reorders complete survey groups");
            check(fleet.drones()[1].toMap()["startRegion"].toInt()==0 && fleet.validate().isEmpty(), "Region assignment swaps owners without losing coverage or splits");
            fleet.setProfile(0,"startRegion",0);
            fleet.setProfile(0,"uas",2);
            check(fleet.drones()[0].toMap()["uas"].toInt()==2 && fleet.drones()[1].toMap()["uas"].toInt()==1,"Aircraft identities remain unique");
            fleet.setProfile(0,"uas",1);
            for (int uas=1;uas<=3;++uas) QFile::remove(QString("/tmp/chimera-master-smoke-uas%1.plan").arg(uas));
            check(fleet.exportAll("/tmp/chimera-master-smoke.plan").isEmpty(),"Export all drones from a mission name");
            check(fleet.exportAll("/tmp/chimera-plain-smoke.plan").isEmpty(),"Export all drones as standard missions");
            check(QFile::exists("/tmp/chimera-plain-smoke-uas1-plain.plan"),"Standard fleet export uses distinct filenames");
            QFile exportedMaster("/tmp/chimera-master-smoke.plan"); check(exportedMaster.open(QIODevice::ReadOnly),"Batch export saves the editable master");
            check(QJsonDocument::fromJson(exportedMaster.readAll()).object()["mission"].toObject()["chimeraFleet"].toObject()["plans"].toArray().size()==3,"Exported master contains all drone plans");
            for (int uas=1;uas<=3;++uas) {
                check(QFile::exists(QString("/tmp/chimera-master-smoke-uas%1.plan").arg(uas)),"Bundle saves editable drone plans");
                check(QFile::exists(QString("/tmp/chimera-master-smoke-uas%1-plain.plan").arg(uas)),"Bundle saves standard drone plans");
            }
            check(ChimeraPlanner().exportMission(&plainController,"/tmp/chimera-single-bundle.plan").isEmpty(),"Export standalone editable and plain plans");
            check(QFile::exists("/tmp/chimera-single-bundle.plan") && QFile::exists("/tmp/chimera-single-bundle-plain.plan"),"Standalone export saves both formats");
            context.setContextProperty("chimeraFleet", &fleet);
            QQmlComponent fleetBar(engine, QUrl("qrc:/Planner/FleetBar.qml"));
            check(fleetBar.isReady(), fleetBar.errorString());
            auto* fleetWidget = fleetBar.create(&context); check(fleetWidget, fleetBar.errorString()); delete fleetWidget;
            QQmlComponent fleetOverlay(engine, QUrl("qrc:/Planner/FleetOverlay.qml"));
            check(fleetOverlay.isReady(), fleetOverlay.errorString());
            auto* fleetMap = canvas.create(&context);
            fleetWidget = fleetOverlay.createWithInitialProperties({{"map", QVariant::fromValue(fleetMap)}}, &context);
            check(fleetWidget, fleetOverlay.errorString()); delete fleetWidget; delete fleetMap;
            const auto testCenter = origin;
            QList<QPair<QGeoCoordinate,QGeoCoordinate>> testLegs;
            for (double x : {-10.,0.,10.}) {
                auto middle=testCenter.atDistanceAndAzimuth(std::abs(x),x<0?270:90);
                testLegs.append({middle.atDistanceAndAzimuth(40,180),middle.atDistanceAndAzimuth(40,0)});
            }
            const auto passes=CenterOut::passes(testLegs,testCenter,0);
            check(passes.size()==testLegs.size(),"Center-out keeps complete tracks");
            double length=0, previousOffset=-1, previousDirection=0;
            for (const auto& pass : passes) {
                check(pass.distance>=previousOffset,"Tracks advance away from the common center");
                const double direction=std::cos(pass.start.azimuthTo(pass.end)*M_PI/180);
                if (previousDirection) check(direction*previousDirection<0,"Successive tracks zigzag in opposite directions");
                previousDirection=direction; previousOffset=pass.distance;
                length+=pass.start.distanceTo(pass.end);
            }
            check(std::abs(length-240)<0.1,"Center-out preserves full survey coverage");
            QFile masterFile("/tmp/qgc-fleet-master.plan"); check(masterFile.open(QIODevice::WriteOnly),"Write fleet fixture");
            masterFile.write(QJsonDocument(fleetJson).toJson()); masterFile.close();
            auto* liveFleet=qobject_cast<FleetPlanner*>(engine->rootContext()->contextProperty("chimeraFleet").value<QObject*>());
            check(liveFleet && liveFleet->activeController(),"Fleet attached to native Plan View");
            auto* liveMaster=liveFleet->activeController();
            liveMaster->loadFromFile("/tmp/qgc-fleet-master.plan");
            check(liveFleet->enabled(),"Native master load restores fleet");
            auto* mainWindow=qobject_cast<QQuickWindow*>(engine->rootObjects().first());
            check(QMetaObject::invokeMethod(mainWindow,"showPlanView"),"Show native master view");
            check(QMetaObject::invokeMethod(liveMaster,"fitViewportToItems"),"Fit fleet master map");
            QEventLoop viewEvents; QTimer::singleShot(250,&viewEvents,&QEventLoop::quit); viewEvents.exec();
            mainWindow->grabWindow().save("/tmp/qgc-fleet-master.png");
            liveFleet->setProperty("showRoutes",false);
            QTimer::singleShot(150,&viewEvents,&QEventLoop::quit); viewEvents.exec();
            mainWindow->grabWindow().save("/tmp/qgc-fleet-sectors.png");
            liveFleet->setProperty("showRoutes",true);
            liveFleet->setSelected(1);
            auto* liveMission=liveFleet->activeController()->missionController();
            liveMission->setCurrentPlanViewSeqNum(qobject_cast<VisualMissionItem*>(liveMission->visualItems()->get(3))->sequenceNumber(),true);
            QTimer::singleShot(250,&viewEvents,&QEventLoop::quit); viewEvents.exec();
            mainWindow->grabWindow().save("/tmp/qgc-fleet-drone.png");
            for (int repeat=0; repeat<4; ++repeat) {
                auto* active = liveFleet->activeController()->missionController();
                auto* part = qobject_cast<ChimeraMissionItem*>(active->visualItems()->get(3));
                QObject* splitButton=nullptr;
                for (auto* candidate : mainWindow->findChildren<QObject*>("chimeraSplitButton"))
                    if (candidate->property("missionBlock").value<QObject*>()==part) { splitButton=candidate; break; }
                check(splitButton && QMetaObject::invokeMethod(splitButton,"clicked"),"Split through the map control");
                QTimer::singleShot(150,&viewEvents,&QEventLoop::quit); viewEvents.exec();
                QObject* splitDragger=nullptr;
                QList<QQuickItem*> visualChildren{mainWindow->contentItem()};
                for (int i=0;i<visualChildren.size();++i) {
                    auto* candidate=visualChildren[i]; visualChildren.append(candidate->childItems());
                    if (candidate->objectName()=="chimeraSplitDragger" && candidate->property("missionBlock").value<QObject*>()==part) { splitDragger=candidate; break; }
                }
                check(splitDragger,QString("Split drag control exists: current=%1 split=%2 controls=%3").arg(part->isCurrentItem()).arg(part->hasEndSplit()).arg(mainWindow->findChildren<QObject*>("chimeraSplitDragger").size()));
                const auto previousExit=part->exitCoordinate();
                const auto dragTarget=part->splitHandles().first().value<QGeoCoordinate>();
                check(QMetaObject::invokeMethod(splitDragger,"dragStart"),"Start split drag");
                splitDragger->setProperty("itemCoordinate",QVariant::fromValue(dragTarget));
                check(part->exitCoordinate()==previousExit,"Dragging does not rebuild the survey");
                check(QMetaObject::invokeMethod(splitDragger,"dragStop"),"Commit split drag");
                check(part->exitCoordinate().distanceTo(dragTarget)<0.05,"Split drag commits its new endpoint");
                QTimer::singleShot(150,&viewEvents,&QEventLoop::quit); viewEvents.exec();
                check(part->removeEndSplit().isEmpty(), "Remove split with the native editor open");
                liveFleet->setLayers(30 + repeat, 60 + repeat, 10);
                QTimer::singleShot(150,&viewEvents,&QEventLoop::quit); viewEvents.exec();
            }
            for (int count : {2, 3, 4, 2, 3}) {
                check(liveFleet->setCount(count).isEmpty(), "Change count with the native drone editor visible");
                QTimer::singleShot(300,&viewEvents,&QEventLoop::quit); viewEvents.exec();
                check(liveFleet->validate().isEmpty(),liveFleet->validate());
            }
            QTimer::singleShot(3000,&viewEvents,&QEventLoop::quit); viewEvents.exec();
            liveFleet->setSelected(0);
            QTimer::singleShot(200,&viewEvents,&QEventLoop::quit); viewEvents.exec();
            QQuickItem* fleetSettings=nullptr;
            QList<QQuickItem*> settingsItems{mainWindow->contentItem()};
            for (int i=0;i<settingsItems.size();++i) {
                auto* item=settingsItems[i]; settingsItems.append(item->childItems());
                if (item->objectName()=="fleetSettings") fleetSettings=item;
            }
            check(fleetSettings,"Fleet setup remains in the survey editor");
            check(fleetSettings->mapToItem(mainWindow->contentItem(),QPointF()).x()>mainWindow->width()/2,"Fleet setup is in the right panel");
            mainWindow->grabWindow().save("/tmp/qgc-fleet-settings.png");
            liveFleet->setSelected(1);
            qInfo() << "Native low-altitude fleet";
            liveFleet->setLayers(18,30,0);
            QTimer::singleShot(1500,&viewEvents,&QEventLoop::quit); viewEvents.exec();
            check(liveFleet->validate().isEmpty(),liveFleet->validate());
            liveMaster->saveToFile("/tmp/qgc-fleet-native-save.plan");
            check(liveFleet->activeController()!=liveMaster,"Native editor switches to the selected drone");
            checkFleetCreationUi(engine,liveMaster);
            qInfo() << "PLANNER_SMOKE_PASS: blocks, no-gimbal markers, linked split edit/remove/resume, persistence, Fact panels, map handles";
            QCoreApplication::exit(0);
        } catch (const std::exception& error) {
            qCritical() << "PLANNER_SMOKE_FAIL:" << error.what(); QCoreApplication::exit(1);
        }
    });
}
