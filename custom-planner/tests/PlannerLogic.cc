#include "ChimeraPlanner.h"
#include "ChimeraMissionItem.h"
#include "FleetPlanner.h"
#include "CenterOut.h"
#include "Fact.h"
#include "MissionController.h"
#include "MissionItem.h"
#include "MissionSettingsItem.h"
#include "PlanMasterController.h"
#include "QGCMapPolygon.h"
#include "QmlObjectListModel.h"
#include "SimpleMissionItem.h"
#include "SpeedSection.h"
#include <QCoreApplication>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QTemporaryDir>
#include <QQmlApplicationEngine>
#include <QQmlComponent>
#include <QScopedPointer>
#include <stdexcept>

namespace {
void require(bool condition, const char* message)
{
    if (!condition) throw std::runtime_error(message);
}
ChimeraMissionItem* survey(MissionController* mission, const QGeoCoordinate& center, int index)
{
    auto* block = qobject_cast<ChimeraMissionItem*>(mission->insertComplexMissionItem("Chimera survey", center, index));
    require(block, "Create survey");
    QList<QGeoCoordinate> polygon;
    for (double bearing : {45., 135., 225., 315.}) polygon.append(center.atDistanceAndAzimuth(80, bearing));
    block->surveyAreaPolygon()->setPath(polygon);
    block->setValue("altitude", 25);
    require(block->error().isEmpty(), "Build survey geometry");
    return block;
}
QList<MissionItem*> commands(PlanMasterController& plan, QObject* owner)
{
    QList<MissionItem*> items;
    auto* visuals = plan.missionController()->visualItems();
    for (int i = 0; i < visuals->count(); ++i)
        qobject_cast<VisualMissionItem*>(visuals->get(i))->appendMissionItems(items, owner);
    qobject_cast<MissionSettingsItem*>(visuals->get(0))->addMissionEndAction(items, items.size(), owner);
    return items;
}
QVariantList previewSegments(FleetPlanner& fleet, ChimeraPlanner& planner)
{
    QVariantList segments;
    for (const auto& value : fleet.previewMissions()) {
        const auto preview = value.toMap();
        segments.append(planner.speedSegments(preview["controller"].value<PlanMasterController*>(), preview["itemCount"].toInt()));
    }
    return segments;
}
bool containsCoordinate(const QVariantList& segments, const QGeoCoordinate& coordinate)
{
    for (const auto& value : segments)
        for (const auto& point : value.toMap()["path"].toList())
            if (point.value<QGeoCoordinate>().distanceTo(coordinate) < 0.01) return true;
    return false;
}
void checkSurvey(ChimeraMissionItem* block)
{
    QObject owner;
    QList<MissionItem*> items;
    block->appendMissionItems(items, &owner);
    double speed = 0;
    bool armed = false;
    int entries = 0, tracks = 0;
    for (auto* item : items) {
        if (item->command() == MAV_CMD_DO_CHANGE_SPEED) speed = item->param2();
        if (item->command() == MAV_CMD_NAV_DELAY) { require(speed == 1, "Arm at survey speed"); armed = true; }
        if (item->command() != MAV_CMD_NAV_WAYPOINT) continue;
        if (!armed) { require(speed == 5, "Reach survey entry at transit speed"); ++entries; }
        else { require(speed == 1, "Complete survey tracks at survey speed"); ++tracks; }
    }
    require(entries == 1 && tracks > 0, "One transit entry before survey tracks");
    require(items.last()->command() == MAV_CMD_DO_CHANGE_SPEED && speed == 5, "Leave survey at transit speed");
}
}

void runPlannerLogic(QQmlApplicationEngine* engine)
{
    try {
        for (const auto& name : {"SpeedOverlay.qml", "FleetOverlay.qml"}) {
            QQmlComponent component(engine, QUrl(QString("qrc:/qml/Chimera/Planner/%1").arg(name)));
            QScopedPointer<QObject> item(component.create());
            if (!item) throw std::runtime_error(component.errorString().toStdString());
        }
        const QGeoCoordinate origin(38, -77, 0);
        PlanMasterController plan;
        plan.start(true);
        auto* mission = plan.missionController();
        mission->insertTakeoffItem(origin, 1);
        auto* fiducial = qobject_cast<ChimeraMissionItem*>(mission->insertComplexMissionItem("Chimera fiducial", origin, 2));
        auto* first = survey(mission, origin, 3);
        require(first->values()["speed"].toDouble() == 1 && first->values()["transitSpeed"].toDouble() == 5, "Speed defaults");
        auto* roi = qobject_cast<SimpleMissionItem*>(mission->insertROIMissionItem(origin, 4));
        require(roi && roi->missionItem().param7() == 0 && roi->altitude()->rawValue().toDouble() == 0, "ROI defaults to ground height");
        auto* manual = qobject_cast<SimpleMissionItem*>(mission->insertSimpleMissionItem(origin.atDistanceAndAzimuth(140, 90), 5));
        manual->speedSection()->setSpecifyFlightSpeed(true);
        manual->speedSection()->flightSpeed()->setRawValue(2.2);
        const auto manualDestination = origin.atDistanceAndAzimuth(160, 90);
        mission->insertSimpleMissionItem(manualDestination, 6);
        auto* second = survey(mission, origin.atDistanceAndAzimuth(300, 90), 7);
        auto* vantage = qobject_cast<ChimeraMissionItem*>(mission->insertComplexMissionItem("Chimera vantage", origin, 8));
        checkSurvey(first);
        checkSurvey(second);
        QObject owner;
        QList<MissionItem*> hold;
        fiducial->appendMissionItems(hold, &owner);
        require(hold.first()->param2() == 5 && hold.last()->command() == MAV_CMD_NAV_LOITER_TIME, "Fiducial rapid transit and loiter");
        hold.clear(); vantage->appendMissionItems(hold, &owner);
        require(hold.size() == 3 && hold[1]->param2() == 5 && hold.last()->command() == MAV_CMD_NAV_LOITER_TIME, "Vantage rapid transit and loiter");
        ChimeraPlanner planner;
        auto segments = planner.speedSegments(&plan);
        int slow = 0, rapid = 0;
        bool manualGreen = false;
        for (const auto& value : segments) {
            const auto segment = value.toMap();
            const double speed = segment["speed"].toDouble();
            const auto tint = segment["color"].toString();
            if (speed == 1) { ++slow; require(tint == "#27ae60", "Survey overlay is green"); }
            if (speed == 5) { ++rapid; require(tint == "#eb5757", "Transit overlay is red"); }
            for (const auto& point : segment["path"].toList())
                if (point.value<QGeoCoordinate>().distanceTo(manualDestination) < 0.01 && speed == 2.2) manualGreen = tint == "#27ae60";
        }
        require(slow >= 2 && rapid >= 3 && manualGreen, "Overlay includes surveys, transfers, and operator speed");
        manual->speedSection()->flightSpeed()->setRawValue(4.);
        bool manualRed = false;
        for (const auto& value : planner.speedSegments(&plan)) {
            const auto segment = value.toMap();
            const auto path = segment["path"].toList();
            if (segment["speed"].toDouble() == 4 && path.last().value<QGeoCoordinate>().distanceTo(manualDestination) < 0.01)
                manualRed = segment["color"] == "#eb5757";
        }
        require(manualRed, "Manual speed changes update the overlay classification");
        QTemporaryDir directory;
        const auto filename = directory.filePath("mission.plan");
        require(planner.exportPlan(&plan, filename).isEmpty(), "Export plain mission");
        QFile file(filename); require(file.open(QIODevice::ReadOnly), "Read plain mission");
        const auto json = QJsonDocument::fromJson(file.readAll()).object()["mission"].toObject();
        const auto exported = json["items"].toArray();
        const auto upload = commands(plan, &owner);
        require(exported.size() == upload.size() - 1, "Export adds no speed commands");
        for (int i = 0; i < exported.size(); ++i) {
            QJsonObject expected; upload[i + 1]->save(expected);
            expected = QJsonDocument::fromJson(QJsonDocument(expected).toJson()).object();
            const auto actual = exported[i].toObject();
            require(actual["command"] == expected["command"] && actual["params"] == expected["params"], "Export preserves upload commands and manual speeds");
        }
        PlanMasterController restored;
        restored.start(true);
        QString error;
        require(restored.missionController()->load(json, error), "Load plain mission through native QGC sections");
        const auto roundTrip = commands(restored, &owner);
        require(roundTrip.size() == upload.size(), "Round trip preserves command count");
        for (int i = 0; i < upload.size(); ++i)
            require(roundTrip[i]->command() == upload[i]->command() && roundTrip[i]->param2() == upload[i]->param2(), "Round trip preserves speed commands");
        const auto path = first->preview()["path"].toList();
        const auto from = path[1].value<QGeoCoordinate>(), to = path[2].value<QGeoCoordinate>();
        require(first->addSplitPoint(from.atDistanceAndAzimuth(from.distanceTo(to) / 2, from.azimuthTo(to))).isEmpty(), "Split survey for manual inspection");
        checkSurvey(first);
        auto* continuation = qobject_cast<ChimeraMissionItem*>(mission->visualItems()->get(4));
        require(continuation, "Find linked continuation");
        checkSurvey(continuation);
        first->addInspection(1);
        QList<MissionItem*> inspection;
        first->appendMissionItems(inspection, &owner);
        double speed = 0;
        for (auto* item : inspection) {
            if (item->command() == MAV_CMD_DO_CHANGE_SPEED) speed = item->param2();
            if (item->command() == MAV_CMD_DO_SET_ROI_LOCATION) require(speed == 5, "ROI interruption uses rapid transit speed");
            if (item->command() == MAV_CMD_NAV_DELAY) require(speed == 1, "Inspection returns to survey speed");
        }
        PlanMasterController master;
        master.start(true);
        master.missionController()->insertTakeoffItem(origin, 1);
        master.missionController()->insertComplexMissionItem("Chimera fiducial", origin, 2);
        auto* domain = survey(master.missionController(), origin, 3);
        FleetPlanner fleet;
        auto* home = qobject_cast<MissionSettingsItem*>(master.missionController()->visualItems()->get(0));
        home->setCoordinate(origin);
        home->plannedHomePositionAltitude()->setRawValue(0.);
        const auto fleetError = fleet.create(domain, 3, 20, 30, 0);
        if (!fleetError.isEmpty()) throw std::runtime_error(fleetError.toStdString());
        fleet.synchronize();
        const auto before = fleet.fiducialPositions();
        require(before.size() == 3, "Show every drone at the fiducial");
        fleet.setProfile(0, "heading", 75);
        require(fleet.fiducialPositions()[0].toMap()["coordinate"] == before[0].toMap()["coordinate"], "Heading edits do not reorder fiducial positions");
        fleet.setFiducialRotation(45);
        const auto rotated = fleet.fiducialPositions();
        for (int i = 0; i < before.size(); ++i) {
            const auto previous = before[i].toMap()["coordinate"].value<QGeoCoordinate>();
            const auto current = rotated[i].toMap()["coordinate"].value<QGeoCoordinate>();
            require(current.distanceTo(origin.atDistanceAndAzimuth(2, origin.azimuthTo(previous) + 45)) < 0.01, "Rotate drone positions as one ordered group");
            require(current.altitude() == fleet.drones()[i].toMap()["altitude"].toDouble(), "Fiducial retains survey altitude");
            require(rotated[i].toMap()["heading"] == fleet.drones()[i].toMap()["heading"], "Fiducial retains survey heading");
        }
        require(rotated[0].toMap()["heading"].toDouble() == 75, "Rotation preserves a manual survey heading");
        require(fleet.validate().isEmpty(), "Fleet remains valid after rotation");
        fleet.setSelected(0);
        require(fleet.previewMissions().size() == 3, "Master speed overlay includes all drones");
        fleet.setSelected(1);
        require(fleet.previewMissions().size() == 1 && fleet.previewMissions()[0].toMap()["controller"].value<PlanMasterController*>() == fleet.activeController(), "Drone selector limits the speed overlay");
        auto* childMission = fleet.activeController()->missionController();
        auto* sector = qobject_cast<ChimeraMissionItem*>(childMission->visualItems()->get(3));
        const auto sectorPath = sector->preview()["path"].toList();
        const auto splitFrom = sectorPath[0].value<QGeoCoordinate>(), splitTo = sectorPath[1].value<QGeoCoordinate>();
        require(sector->addSplitPoint(splitFrom.atDistanceAndAzimuth(splitFrom.distanceTo(splitTo) / 2, splitFrom.azimuthTo(splitTo))).isEmpty(), "Split a sector for custom inspection waypoints");
        const auto customPosition = origin.atDistanceAndAzimuth(170, 32);
        const auto removedPosition = origin.atDistanceAndAzimuth(195, 32);
        auto* custom = qobject_cast<SimpleMissionItem*>(childMission->insertSimpleMissionItem(customPosition, 4));
        custom->altitude()->setRawValue(21.5);
        custom->missionItem().setParam4(123);
        custom->speedSection()->setSpecifyFlightSpeed(true);
        custom->speedSection()->flightSpeed()->setRawValue(2.4);
        auto* removable = childMission->insertSimpleMissionItem(removedPosition, 5);
        childMission->insertROIMissionItem(origin, 6);
        fleet.synchronize();
        require(custom->altitude()->rawValue().toDouble() == 21.5 && custom->missionItem().param4() == 123 && custom->speedSection()->flightSpeed()->rawValue().toDouble() == 2.4, "Fleet updates preserve custom altitude, heading, and speed");
        fleet.setPathDisplay(FleetPlanner::FirstSectorPath);
        const int firstSectorEnd = fleet.previewMissions()[0].toMap()["itemCount"].toInt();
        auto* fleetContinuation = qobject_cast<ChimeraMissionItem*>(childMission->visualItems()->get(7));
        require(fleetContinuation && fleetContinuation->fleetRegion() == sector->fleetRegion() && firstSectorEnd == 8, "Sector filtering includes split continuations and custom items");
        const auto laterPosition = origin.atDistanceAndAzimuth(650, 233);
        childMission->insertSimpleMissionItem(laterPosition, firstSectorEnd + 1);
        fleet.synchronize();
        auto filtered = previewSegments(fleet, planner);
        require(containsCoordinate(filtered, customPosition) && containsCoordinate(filtered, removedPosition) && !containsCoordinate(filtered, laterPosition), "First sector overlay includes its inspection path and hides later sectors");
        fleet.setPathDisplay(FleetPlanner::WholePath);
        require(containsCoordinate(previewSegments(fleet, planner), laterPosition), "Whole path includes later custom waypoints");
        fleet.setPathDisplay(FleetPlanner::NoPaths);
        require(fleet.previewMissions().isEmpty() && previewSegments(fleet, planner).isEmpty(), "No paths hides the speed overlay");
        fleet.setPathDisplay(FleetPlanner::FirstSectorPath);
        const auto editedPosition = origin.atDistanceAndAzimuth(180, 43);
        custom->setCoordinate(editedPosition);
        childMission->removeVisualItem(childMission->visualItems()->indexOf(removable));
        fleet.synchronize();
        filtered = previewSegments(fleet, planner);
        require(containsCoordinate(filtered, editedPosition) && !containsCoordinate(filtered, customPosition) && !containsCoordinate(filtered, removedPosition), "Overlay follows custom waypoint moves and removals");
        auto* inherited = qobject_cast<SimpleMissionItem*>(childMission->insertSimpleMissionItem(origin.atDistanceAndAzimuth(155, 24), 4));
        inherited->altitude()->setRawValue(20.);
        fleet.setProfile(0, "altitude", 22);
        require(inherited->altitude()->rawValue().toDouble() == 22 && custom->altitude()->rawValue().toDouble() == 21.5, "Layer changes update inherited heights and retain explicit waypoint heights");
        require(planner.exportPlan(fleet.activeController(), directory.filePath("custom-fleet.plan")).isEmpty(), "Filtered views still export the complete custom mission");
        QFile customFile(directory.filePath("custom-fleet.plan"));
        require(customFile.open(QIODevice::ReadOnly), "Read custom fleet mission");
        const auto customItems = QJsonDocument::fromJson(customFile.readAll()).object()["mission"].toObject()["items"].toArray();
        bool exportedLater = false;
        for (const auto& value : customItems) {
            const auto params = value.toObject()["params"].toArray();
            if (QGeoCoordinate(params[4].toDouble(), params[5].toDouble()).distanceTo(laterPosition) < 0.01) exportedLater = true;
        }
        require(exportedLater, "Sector display filters never truncate exported missions");
        fleet.setSelected(0);
        require(fleet.previewMissions().size() == 3, "Master restores all drone overlays");
        QJsonArray savedFiducial;
        for (int i = 1; i < master.missionController()->visualItems()->count(); ++i) {
            auto* block = qobject_cast<ChimeraMissionItem*>(master.missionController()->visualItems()->get(i));
            if (block && block->kind() == "fiducial") block->save(savedFiducial);
        }
        auto saved = savedFiducial.first().toObject();
        require(fiducial->load(saved, 2, error) && fiducial->values()["fiducialRotation"].toDouble() == 45, "Save and restore fiducial rotation");
        auto values = saved["values"].toObject(); values.remove("fiducialRotation"); saved["values"] = values;
        require(fiducial->load(saved, 2, error) && fiducial->values()["fiducialRotation"].toDouble() == 0, "Older plans keep their default fiducial rotation");
        require(fleet.setCount(2).isEmpty() && fleet.setCount(3).isEmpty(), "Resize fleet after fiducial rotation");
        fleet.synchronize();
        require(fleet.fiducialPositions().size() == 3 && fleet.fiducialRotation() == 45 && fleet.validate().isEmpty(), "Fleet changes retain rotation and valid altitude layers");
        fleet.setSelected(1);
        bool restoredCustom = false;
        auto* restoredItems = fleet.activeController()->missionController()->visualItems();
        for (int i = 1; i < restoredItems->count(); ++i) {
            auto* waypoint = qobject_cast<SimpleMissionItem*>(restoredItems->get(i));
            if (!waypoint || waypoint->coordinate().distanceTo(editedPosition) >= 0.01) continue;
            restoredCustom = waypoint->altitude()->rawValue().toDouble() == 21.5 && waypoint->missionItem().param4() == 123 && waypoint->speedSection()->flightSpeed()->rawValue().toDouble() == 2.4;
        }
        require(restoredCustom, "Fleet resizing preserves custom waypoints and their settings");
        require(!engine->rootObjects().isEmpty() && QMetaObject::invokeMethod(engine->rootObjects().first(), "finishCloseProcess"), "Close through the standard QGC shutdown pipeline");
        qInfo("PLANNER_LOGIC_PASS");
    } catch (const std::exception& error) {
        qCritical("PLANNER_LOGIC_FAIL: %s", error.what());
        QCoreApplication::exit(1);
    }
}
