#include "ChimeraPlanner.h"
#include "CameraCalc.h"
#include "MissionController.h"
#include "MissionItem.h"
#include "MissionSettingsItem.h"
#include "PlanMasterController.h"
#include "QGCMapPolygon.h"
#include "QmlObjectListModel.h"
#include "SimpleMissionItem.h"
#include "SurveyComplexItem.h"
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
    QVariantMap values{{"enabled", false}, {"yaw", 0.0}, {"pitch", -45.0}, {"overlap", 70.0},
        {"width", 1920.0}, {"height", 1080.0}, {"fx", 1200.0}, {"fy", 1200.0},
        {"cx", 960.0}, {"cy", 540.0}};
    if (item) {
        const auto saved = item->property(settingsKey).toMap();
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
                                QVariantList& path, QString& error)
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
    QJsonArray saved;
    source->save(saved);
    auto json = saved.first().toObject();
    auto transect = json["TransectStyleComplexItem"].toObject();
    auto calc = transect["CameraCalc"].toObject();
    calc["CameraName"] = CameraCalc::canonicalManualCameraName();
    calc["AdjustedFootprintSide"] = shape.spacing;
    calc["AdjustedFootprintFrontal"] = shape.spacing;
    transect["CameraCalc"] = calc;
    transect["Items"] = QJsonArray();
    json["TransectStyleComplexItem"] = transect;
    SurveyComplexItem generated(source->masterController(), false, QString());
    if (!generated.load(json, 1, error)) return false;
    generated.setWizardMode(false);
    generated.hoverAndCapture()->setRawValue(false);
    generated.refly90Degrees()->setRawValue(false);
    QList<QGeoCoordinate> polygon;
    for (const auto& point : source->surveyAreaPolygon()->coordinateList()) {
        polygon.append(translate(point, -shape.center));
    }
    generated.surveyAreaPolygon()->setPath(polygon);
    generated.appendMissionItems(items, owner);
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
    auto* survey = qobject_cast<SurveyComplexItem*>(item);
    if (!survey) return tr("Select a survey first.");
    auto* controller = survey->masterController();
    QString path = QUrl(filename).isLocalFile() ? QUrl(filename).toLocalFile() : filename;
    if (!path.endsWith(".plan", Qt::CaseInsensitive)) path += ".plan";
    if (QFileInfo(path).absoluteFilePath() == QFileInfo(controller->currentPlanFile()).absoluteFilePath()) {
        return tr("Choose a new filename to keep the editable survey source.");
    }
    QObject owner;
    QList<MissionItem*> compiled;
    auto* visualItems = controller->missionController()->visualItems();
    for (int i = 0; i < visualItems->count(); ++i) {
        auto* visual = qobject_cast<VisualMissionItem*>(visualItems->get(i));
        auto* candidate = qobject_cast<SurveyComplexItem*>(visual);
        if (candidate && settings(candidate)["enabled"].toBool()) {
            QList<MissionItem*> generated;
            QVariantList points;
            QString error;
            if (!buildSurvey(candidate, generated, &owner, points, error)) return error;
            compiled.append(command(&owner, MAV_CMD_DO_SET_ROI_NONE));
            compiled.append(command(&owner, MAV_CMD_DO_MOUNT_CONTROL, settings(candidate)["pitch"].toDouble()));
            compiled.append(command(&owner, MAV_CMD_NAV_DELAY, 0));
            compiled.append(generated);
        } else {
            if (visual->readyForSaveState() != VisualMissionItem::ReadyForSave) {
                return tr("Complete all mission items before export.");
            }
            visual->appendMissionItems(compiled, &owner);
        }
    }
    if (auto* home = qobject_cast<MissionSettingsItem*>(visualItems->get(0))) {
        home->addMissionEndAction(compiled, compiled.size(), &owner);
    }
    QJsonArray items;
    for (int i = 1; i < compiled.size(); ++i) {
        auto* missionItem = compiled[i];
        if (missionItem->command() == MAV_CMD_DO_JUMP || missionItem->command() == MAV_CMD_DO_JUMP_TAG) {
            return tr("Remove mission jumps before export. Expansion changes their target sequences.");
        }
        items.append(serialize(missionItem, i));
    }
    auto root = controller->saveToJson().object();
    auto mission = root["mission"].toObject();
    mission["items"] = items;
    root["mission"] = mission;
    QSaveFile file(path);
    if (!file.open(QIODevice::WriteOnly)) return file.errorString();
    const auto bytes = QJsonDocument(root).toJson();
    if (file.write(bytes) != bytes.size() || !file.commit()) return file.errorString();
    return {};
}

QString ChimeraPlanner::insertBlock(QObject* object, const QString& kind)
{
    auto* controller = qobject_cast<MissionController*>(object);
    if (!controller) return tr("Open a mission first.");
    const int selected = controller->currentPlanViewVIIndex();
    auto* model = controller->visualItems();
    if (selected < 1 || selected >= model->count()) return tr("Select a waypoint first.");
    auto* anchor = qobject_cast<SimpleMissionItem*>(model->get(selected));
    if (!anchor || anchor->command() != MAV_CMD_NAV_WAYPOINT) return tr("Select an ordinary waypoint first.");
    QObject owner;
    QList<MissionItem*> block;
    const auto checkpoint = anchor->missionItem();
    double pitch = qQNaN();
    double triggerDistance = 0;
    int region = -1;
    for (int i = 0; i <= selected; ++i) {
        QList<MissionItem*> preceding;
        qobject_cast<VisualMissionItem*>(model->get(i))->appendMissionItems(preceding, &owner);
        for (const auto* item : preceding) {
            if (item->command() == MAV_CMD_DO_MOUNT_CONTROL) pitch = item->param1();
            if (item->command() == MAV_CMD_DO_SET_CAM_TRIGG_DIST) triggerDistance = item->param1();
            if (item->command() == MAV_CMD_NAV_DELAY && (item->param1() == 0 || item->param1() == 1)) {
                region = int(item->param1());
            }
        }
    }
    if (kind == "inspection") {
        if (region != 0) return tr("Select a waypoint inside a search region for an ROI inspection.");
        if (!std::isfinite(checkpoint.param4()) || !std::isfinite(pitch)) {
            return tr("Set an explicit waypoint yaw and a preceding gimbal pitch before inserting an inspection.");
        }
        block.append(command(&owner, MAV_CMD_DO_SET_CAM_TRIGG_DIST, 0));
        auto* roi = command(&owner, MAV_CMD_DO_SET_ROI_LOCATION);
        roi->setFrame(MAV_FRAME_GLOBAL_RELATIVE_ALT);
        roi->setParam5(checkpoint.param5());
        roi->setParam6(checkpoint.param6());
        roi->setParam7(0);
        block.append(roi);
        block.append(new MissionItem(checkpoint, &owner));
        block.append(command(&owner, MAV_CMD_DO_SET_ROI_NONE));
        auto* returnPoint = new MissionItem(checkpoint, &owner);
        returnPoint->setParam1(2);
        block.append(returnPoint);
        block.append(command(&owner, MAV_CMD_DO_MOUNT_CONTROL, pitch));
        block.append(command(&owner, MAV_CMD_NAV_DELAY, 0));
        auto* settlePoint = new MissionItem(checkpoint, &owner);
        settlePoint->setParam1(2);
        block.append(settlePoint);
        block.append(command(&owner, MAV_CMD_DO_SET_CAM_TRIGG_DIST, triggerDistance));
    } else if (kind == "fiducial") {
        for (int i = 0; i <= selected; ++i) {
            auto* simple = qobject_cast<SimpleMissionItem*>(model->get(i));
            if (simple && simple->command() == MAV_CMD_NAV_DELAY &&
                (simple->missionItem().param1() == 0 || simple->missionItem().param1() == 1)) {
                return tr("Place fiducial holds before the first search or assessment marker.");
            }
        }
        block.append(command(&owner, MAV_CMD_DO_MOUNT_CONTROL, -90));
        block.append(command(&owner, MAV_CMD_NAV_DELAY, 60));
    } else if (kind == "assessment") {
        block.append(command(&owner, MAV_CMD_DO_SET_CAM_TRIGG_DIST, 0));
        block.append(command(&owner, MAV_CMD_DO_SET_ROI_NONE));
        block.append(command(&owner, MAV_CMD_NAV_DELAY, 1));
        block.append(new MissionItem(checkpoint, &owner));
        block.append(command(&owner, MAV_CMD_NAV_DELAY, 60));
    } else {
        return tr("Unknown workflow block.");
    }
    int index = selected + 1;
    for (const auto* item : block) {
        auto* inserted = qobject_cast<SimpleMissionItem*>(controller->insertSimpleMissionItem(anchor->coordinate(), index++));
        inserted->setCommand(item->command());
        auto& target = inserted->missionItem();
        target.setFrame(item->frame());
        target.setParam1(item->param1()); target.setParam2(item->param2());
        target.setParam3(item->param3()); target.setParam4(item->param4());
        target.setParam5(item->param5()); target.setParam6(item->param6()); target.setParam7(item->param7());
        if (item->frame() == MAV_FRAME_GLOBAL_RELATIVE_ALT || item->frame() == MAV_FRAME_GLOBAL) {
            inserted->setAltitudeMode(item->frame() == MAV_FRAME_GLOBAL ?
                QGroundControlQmlGlobal::AltitudeModeAbsolute : QGroundControlQmlGlobal::AltitudeModeRelative);
            inserted->altitude()->setRawValue(item->param7());
        }
    }
    controller->setCurrentPlanViewSeqNum(anchor->sequenceNumber(), true);
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
