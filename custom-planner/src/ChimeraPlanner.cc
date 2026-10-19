#include <algorithm>
#include <memory>
#include "ChimeraPlanner.h"
#include "ChimeraMissionItem.h"
#include "CameraCalc.h"
#include "Vehicle.h"
#include "MissionController.h"
#include "MissionItem.h"
#include "MissionSettingsItem.h"
#include "PlanMasterController.h"
#include "QGCMapPolygon.h"
#include "QmlObjectListModel.h"
#include "SimpleMissionItem.h"
#include "SurveyComplexItem.h"
#include "PlannerSurvey.h"
#include "CenterOut.h"
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QSaveFile>
#include <QUrl>

namespace {
constexpr auto settingsKey = "chimeraSurvey";

QGeoCoordinate translate(const QGeoCoordinate& coordinate, const QPointF& en)
{
    return coordinate.atDistanceAndAzimuth(std::hypot(en.x(), en.y()),
                                           std::atan2(en.x(), en.y()) * 180 / M_PI);
}

MissionItem* command(QObject* owner, MAV_CMD cmd, double p1 = 0, double p2 = 0, double p3 = 0)
{
    return new MissionItem(0, cmd, MAV_FRAME_MISSION, p1, p2, p3, 0, 0, 0,
        cmd == MAV_CMD_DO_MOUNT_CONTROL ? MAV_MOUNT_MODE_MAVLINK_TARGETING : 0,
        true, false, owner);
}

QJsonObject serialize(MissionItem* item, int sequence)
{
    item->setSequenceNumber(sequence);
    QJsonObject json;
    item->save(json);
    if (item->frame() == MAV_FRAME_GLOBAL || item->frame() == MAV_FRAME_GLOBAL_RELATIVE_ALT ||
        item->frame() == MAV_FRAME_GLOBAL_TERRAIN_ALT) {
        const auto mode = item->frame() == MAV_FRAME_GLOBAL ? QGroundControlQmlGlobal::AltitudeModeAbsolute :
            item->frame() == MAV_FRAME_GLOBAL_TERRAIN_ALT ? QGroundControlQmlGlobal::AltitudeModeTerrainFrame :
                                                         QGroundControlQmlGlobal::AltitudeModeRelative;
        json["AltitudeMode"] = mode;
        json["Altitude"] = item->param7();
        json["AMSLAltAboveTerrain"] = QJsonValue::Null;
    }
    return json;
}
}

QVariantMap ChimeraPlanner::settings(QObject* item) const
{
    QVariantMap values{{"enabled", false}, {"yaw", 0.0}, {"pitch", -70.0}, {"overlap", 25.0},
        {"width", 1920.0}, {"height", 1080.0}, {"fx", 1200.0}, {"fy", 1200.0},
        {"cx", 960.0}, {"cy", 540.0}};
    if (item) {
        const auto saved = item->property(settingsKey).toMap();
        if (saved.contains("entryPoint")) values["entryPoint"] = saved["entryPoint"];
        for (auto it = saved.begin(); it != saved.end(); ++it) {
            if (values.contains(it.key())) values[it.key()] = it.value();
        }
    }
    return values;
}

void ChimeraPlanner::setSetting(QObject* item, const QString& key, const QVariant& value)
{
    auto* survey = qobject_cast<SurveyComplexItem*>(item);
    auto values = settings(item);
    if (!survey || !values.contains(key)) return;
    values[key] = value;
    survey->setProperty(settingsKey, values);
    survey->setDirty(true);
    emit settingsChanged(item);
}

CoverageGeometry::Footprint ChimeraPlanner::footprint(SurveyComplexItem* survey) const
{
    const auto s = settings(survey);
    const CoverageGeometry::Camera camera{s["width"].toDouble(), s["height"].toDouble(),
        s["fx"].toDouble(), s["fy"].toDouble(), s["cx"].toDouble(), s["cy"].toDouble()};
    return CoverageGeometry::project(camera, survey->cameraCalc()->distanceToSurface()->rawValue().toDouble(),
        s["yaw"].toDouble(), s["pitch"].toDouble(), survey->gridAngle()->rawValue().toDouble(),
        s["overlap"].toDouble());
}

bool ChimeraPlanner::buildSurvey(SurveyComplexItem* source, QList<MissionItem*>& items, QObject* owner,
                                QVariantList& path, QString& error, SurveyComplexItem* cached, bool configure)
{
    const auto shape = footprint(source);
    error = shape.error;
    if (!error.isEmpty()) return false;
    if (!source->surveyAreaPolygon()->isValid() ||
        source->cameraCalc()->distanceMode() != QGroundControlQmlGlobal::AltitudeModeRelative) {
        error = tr("Use a valid polygon and altitude relative to home. This draft assumes level ground at home height.");
        return false;
    }
    if (source->surveyAreaPolygon()->area() / (shape.spacing * shape.spacing) > 10000) {
        error = tr("The survey is too dense. Increase the spacing or reduce the area.");
        return false;
    }
    std::unique_ptr<SurveyComplexItem> temporary;
    if (!cached) {
        temporary = std::make_unique<SurveyComplexItem>(source->masterController(), false, QString());
        cached = temporary.get();
        configure = true;
    }
    auto& generated = *cached;
    if (configure) {
        QJsonArray saved;
        source->save(saved);
        auto json = saved.first().toObject();
        if (source->property(settingsKey).toMap().contains("entryPoint"))
            json["entryLocation"] = source->property("chimeraCenterOut").toBool() ? 0 : settings(source).value("entryPoint", 0).toInt();
        auto transect = json["TransectStyleComplexItem"].toObject();
        auto calc = transect["CameraCalc"].toObject();
        calc["CameraName"] = CameraCalc::canonicalManualCameraName();
        calc["AdjustedFootprintSide"] = shape.spacing;
        calc["AdjustedFootprintFrontal"] = shape.captureDistance;
        transect["CameraCalc"] = calc;
        transect["Items"] = QJsonArray();
        json["TransectStyleComplexItem"] = transect;
        if (!generated.load(json, 1, error)) return false;
        generated.setWizardMode(false);
        generated.hoverAndCapture()->setRawValue(false);
        generated.refly90Degrees()->setRawValue(false);
    }
    generated.cameraCalc()->adjustedFootprintSide()->setRawValue(shape.spacing);
    generated.cameraCalc()->adjustedFootprintFrontal()->setRawValue(shape.captureDistance);
    generated.gridAngle()->setRawValue(source->gridAngle()->rawValue());
    QList<QGeoCoordinate> polygon;
    for (const auto& point : source->surveyAreaPolygon()->coordinateList()) {
        polygon.append(translate(point, -shape.center));
    }
    if (generated.surveyAreaPolygon()->coordinateList() != polygon) generated.surveyAreaPolygon()->setPath(polygon);
    if (source->property("chimeraCenterOut").toBool()) {
        auto* grid = dynamic_cast<PlannerSurvey*>(&generated);
        if (!grid) { error = tr("Center-first survey grid is unavailable."); return false; }
        const auto searchCenter=source->property("chimeraSearchCenter").value<QGeoCoordinate>();
        const auto center = searchCenter.isValid() ? translate(searchCenter,-shape.center) : generated.surveyAreaPolygon()->center();
        const auto passes = CenterOut::passes(grid->legs(), center, source->gridAngle()->rawValue().toDouble(), settings(source).value("entryPoint", 0).toInt() % 2);
        if (passes.size() > 2000) { error = tr("Center-first survey exceeds 2000 tracks. Increase spacing or reduce the area."); return false; }
        auto waypoint = [&](const QGeoCoordinate& c) {
            items.append(new MissionItem(0, MAV_CMD_NAV_WAYPOINT, MAV_FRAME_GLOBAL_RELATIVE_ALT, 0,0,0,0,
                c.latitude(), c.longitude(), source->cameraCalc()->distanceToSurface()->rawValue().toDouble(), true,false,owner));
        };
        waypoint(center);
        items.append(command(owner, MAV_CMD_DO_SET_CAM_TRIGG_DIST, shape.captureDistance, 0, 1));
        items.append(command(owner, MAV_CMD_DO_SET_CAM_TRIGG_DIST, 0));
        for (const auto& pass : passes) {
            waypoint(pass.start);
            items.append(command(owner, MAV_CMD_DO_SET_CAM_TRIGG_DIST, shape.captureDistance));
            waypoint(pass.end);
            items.append(command(owner, MAV_CMD_DO_SET_CAM_TRIGG_DIST, 0));
        }
    } else generated.appendMissionItems(items, owner);
    const double yaw = settings(source)["yaw"].toDouble();
    for (auto* item : std::as_const(items)) {
        if (item->command() == MAV_CMD_NAV_WAYPOINT) {
            item->setParam4(yaw);
            path.append(QVariant::fromValue(QGeoCoordinate(item->param5(), item->param6(), item->param7())));
        }
    }
    if (path.size() < 2) {
        error = tr("No flight legs were generated. Enlarge the polygon or adjust the settings.");
        return false;
    }
    return true;
}

QVariantMap ChimeraPlanner::preview(QObject* item)
{
    auto* survey = qobject_cast<SurveyComplexItem*>(item);
    if (!survey || !settings(item)["enabled"].toBool()) return {};
    QObject owner;
    QList<MissionItem*> items;
    QVariantList path;
    QString error;
    if (!buildSurvey(survey, items, &owner, path, error)) return {{"error", error}};
    return preview(survey, path);
}

QVariantMap ChimeraPlanner::preview(SurveyComplexItem* survey, const QVariantList& path) const
{
    const auto shape = footprint(survey);
    QVariantList footprints;
    const int stride = std::max(1, int(path.size() / 80));
    for (int i = 0; i < path.size(); i += stride) {
        QVariantList corners;
        for (const auto& corner : shape.corners) {
            corners.append(QVariant::fromValue(translate(path[i].value<QGeoCoordinate>(), corner)));
        }
        footprints.append(QVariant(corners));
    }
    return {{"path", path}, {"footprints", footprints}, {"spacing", shape.spacing},
            {"offset", std::hypot(shape.center.x(), shape.center.y())}, {"error", ""}};
}

QString ChimeraPlanner::exportPlan(QObject* item, const QString& filename)
{
    auto* controller = qobject_cast<PlanMasterController*>(item);
    if (auto* visual = qobject_cast<VisualMissionItem*>(item)) controller = visual->masterController();
    if (!controller) return tr("Select a mission first.");
    QString path = QUrl(filename).isLocalFile() ? QUrl(filename).toLocalFile() : filename;
    if (!path.endsWith(".plan", Qt::CaseInsensitive)) path += ".plan";
    if (QFileInfo(path).absoluteFilePath() == QFileInfo(controller->currentPlanFile()).absoluteFilePath()) {
        return tr("Choose a new filename to keep the editable survey source.");
    }
    QObject owner;
    QList<MissionItem*> compiled;
    auto* visualItems = controller->missionController()->visualItems();
    for (int i = 0; i < visualItems->count(); ++i) {
        const int firstCommand = compiled.size();
        auto* visual = qobject_cast<VisualMissionItem*>(visualItems->get(i));
        if (auto* block = qobject_cast<ChimeraMissionItem*>(visual)) {
            if (!block->error().isEmpty()) return block->error();
            const auto error = block->linkedSurveyError();
            if (!error.isEmpty()) return error;
        }
        auto* candidate = qobject_cast<SurveyComplexItem*>(visual);
        if (candidate && settings(candidate)["enabled"].toBool()) {
            QList<MissionItem*> generated;
            QVariantList points;
            QString error;
            if (!buildSurvey(candidate, generated, &owner, points, error)) return error;
            compiled.append(command(&owner, MAV_CMD_DO_SET_ROI_NONE));
            compiled.append(command(&owner, MAV_CMD_DO_CHANGE_SPEED, 1, settings(candidate)["speed"].toDouble(), -1));
            compiled.append(command(&owner, MAV_CMD_DO_MOUNT_CONTROL, settings(candidate)["pitch"].toDouble()));
            compiled.append(command(&owner, MAV_CMD_NAV_DELAY, 0));
            compiled.append(generated);
        } else {
            if (visual->readyForSaveState() != VisualMissionItem::ReadyForSave) {
                return tr("Complete all mission items before export.");
            }
            visual->appendMissionItems(compiled, &owner);
        }
        auto* block = qobject_cast<ChimeraMissionItem*>(visual);
        if (block && (block->kind() == "survey" || block->kind() == "vantage")) {
            int firstWaypoint = -1, firstArm = -1, firstSpeed = -1;
            for (int j = firstCommand; j < compiled.size(); ++j) {
                const auto command = compiled[j]->command();
                if (firstWaypoint < 0 && command == MAV_CMD_NAV_WAYPOINT) firstWaypoint = j;
                if (firstArm < 0 && command == MAV_CMD_NAV_DELAY && compiled[j]->param1() == (block->kind() == "vantage" ? 1 : 0)) firstArm = j;
                if (firstSpeed < 0 && command == MAV_CMD_DO_CHANGE_SPEED) firstSpeed = j;
            }
            const double transitSpeed = block->values()["transitSpeed"].toDouble();
            if (firstWaypoint >= 0 && firstArm > firstWaypoint && firstSpeed >= firstCommand && firstSpeed < firstWaypoint) {
                compiled[firstSpeed]->setParam2(transitSpeed);
            } else if (firstWaypoint >= 0) {
                auto* entry = new MissionItem(*compiled[firstWaypoint], &owner);
                const int insertion = firstSpeed >= 0 && firstArm >= 0 ? std::min(firstSpeed, firstArm) : firstCommand;
                compiled.insert(insertion, command(&owner, MAV_CMD_DO_CHANGE_SPEED, 1, transitSpeed, -1));
                compiled.insert(insertion + 1, entry);
                if (firstArm >= 0 && firstArm < firstSpeed)
                    compiled.insert(insertion + 2, command(&owner, MAV_CMD_DO_CHANGE_SPEED, 1, block->values()["speed"].toDouble(), -1));
            }
        }
    }
    if (auto* home = qobject_cast<MissionSettingsItem*>(visualItems->get(0))) {
        home->addMissionEndAction(compiled, compiled.size(), &owner);
    }
    const auto* vehicle = controller->controllerVehicle();
    const double defaultSpeed = vehicle->multiRotor() || vehicle->vtol() ? vehicle->defaultHoverSpeed() : vehicle->defaultCruiseSpeed();
    double flightSpeed = defaultSpeed;
    QJsonArray items;
    int sequence = 1;
    for (int i = 1; i < compiled.size(); ++i) {
        auto* missionItem = compiled[i];
        if (missionItem->command() == MAV_CMD_DO_JUMP || missionItem->command() == MAV_CMD_DO_JUMP_TAG) {
            return tr("Remove mission jumps before export. Expansion changes their target sequences.");
        }
        if (missionItem->command() == MAV_CMD_DO_CHANGE_SPEED) {
            if (missionItem->param2() > 0) flightSpeed = missionItem->param2();
            else if (missionItem->param2() == -1) flightSpeed = defaultSpeed;
        }
        if (missionItem->command() == MAV_CMD_NAV_WAYPOINT &&
            (i == 1 || compiled[i-1]->command() != MAV_CMD_DO_CHANGE_SPEED || compiled[i-1]->param2() <= 0))
            items.append(serialize(command(&owner, MAV_CMD_DO_CHANGE_SPEED, 1, flightSpeed, -1), sequence++));
        items.append(serialize(missionItem, sequence++));
    }
    auto root = controller->saveToJson().object();
    auto mission = root["mission"].toObject();
    mission["items"] = items;
    mission.remove("chimeraFleet");
    root["mission"] = mission;
    QSaveFile file(path);
    if (!file.open(QIODevice::WriteOnly)) return file.errorString();
    const auto bytes = QJsonDocument(root).toJson();
    if (file.write(bytes) != bytes.size() || !file.commit()) return file.errorString();
    return {};
}

void ChimeraPlanner::saveSettings(PlanMasterController* controller, QJsonObject& mission)
{
    QList<QVariantMap> surveys;
    auto* model = controller->missionController()->visualItems();
    for (int i = 0; i < model->count(); ++i) {
        if (auto* survey = qobject_cast<SurveyComplexItem*>(model->get(i))) surveys.append(settings(survey));
    }
    auto items = mission["items"].toArray();
    int index = 0;
    for (int i = 0; i < items.size(); ++i) {
        auto item = items[i].toObject();
        if (item["complexItemType"] == "survey" && index < surveys.size()) {
            item[settingsKey] = QJsonObject::fromVariantMap(surveys[index++]);
            items[i] = item;
        }
    }
    mission["items"] = items;
}

void ChimeraPlanner::loadSettings(PlanMasterController* controller, const QJsonObject& plan)
{
    QList<QVariantMap> surveys;
    for (const auto& value : plan["mission"].toObject()["items"].toArray()) {
        auto item = value.toObject();
        if (item["complexItemType"] == "survey") surveys.append(item[settingsKey].toObject().toVariantMap());
    }
    int index = 0;
    auto* model = controller->missionController()->visualItems();
    for (int i = 0; i < model->count() && index < surveys.size(); ++i) {
        if (auto* survey = qobject_cast<SurveyComplexItem*>(model->get(i))) {
            survey->setProperty(settingsKey, surveys[index++]);
            emit settingsChanged(survey);
        }
    }
}
