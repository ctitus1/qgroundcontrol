#include "ChimeraMissionItem.h"
#include "ChimeraPlanner.h"
#include "SurveyComplexItem.h"
#include "CameraCalc.h"
#include "QGCMapPolygon.h"
#include "MissionItem.h"
#include "PlanMasterController.h"
#include <QJsonArray>
#include <QJsonObject>
#include <QScopedValueRollback>

namespace {
QVariantMap point(const QGeoCoordinate& c, bool roi, double yaw)
{
    return {{"latitude", c.latitude()}, {"longitude", c.longitude()},
            {"altitude", roi ? 0 : c.altitude()}, {"yaw", yaw}, {"roi", roi}};
}
QGeoCoordinate coordinateOf(const QVariantMap& p)
{
    return {p.value("latitude").toDouble(), p.value("longitude").toDouble(), p.value("altitude").toDouble()};
}
}

ChimeraMissionItem::ChimeraMissionItem(PlanMasterController* master, bool flyView,
                                     const QString& kind, ChimeraPlanner* planner)
    : ComplexMissionItem(master, flyView), _kind(kind), _planner(planner),
      _survey(new SurveyComplexItem(master, flyView, QString()))
{
    _survey->setParent(this);
    _editorQml = "qrc:/Planner/BlockEditor.qml";
    _values = planner->settings(nullptr);
    _values["enabled"] = true;
    _values["altitude"] = 40.;
    _values["speed"] = 5.;
    _values["delay"] = 60.;
    _values["angle"] = 0.;
    _values["pitch"] = kind == "fiducial" ? -90. : -45.;
    _survey->cameraCalc()->setDistanceMode(QGroundControlQmlGlobal::AltitudeModeRelative);
    _survey->turnAroundDistance()->setRawValue(0);
    _survey->setWizardMode(false);
    connect(surveyAreaPolygon(), &QGCMapPolygon::pathChanged, this, &ChimeraMissionItem::rebuild);
    connect(surveyAreaPolygon(), &QGCMapPolygon::centerDragChanged, this, [this](bool dragging) {
        if (!dragging && !_updating) {
            moveInspectionPoints(_lastCenter, centerCoordinate());
            _lastCenter = centerCoordinate();
            rebuild();
        }
    });
    connect(this, &VisualMissionItem::wizardModeChanged, this, [this](bool value) { if (value) setWizardMode(false); });
    connect(master->missionController(), &MissionController::plannedHomePositionChanged, this, [this] { rebuild(); });
}

QGCMapPolygon* ChimeraMissionItem::surveyAreaPolygon() const { return _survey->surveyAreaPolygon(); }
QGeoCoordinate ChimeraMissionItem::centerCoordinate() const { return isSurveyItem() ? surveyAreaPolygon()->center() : _position; }
QGeoCoordinate ChimeraMissionItem::coordinate() const { return _route.isEmpty() ? _position : _route.first().value<QGeoCoordinate>(); }
QGeoCoordinate ChimeraMissionItem::exitCoordinate() const { return _route.isEmpty() ? _position : _route.last().value<QGeoCoordinate>(); }
double ChimeraMissionItem::homeAltitude() const { return _masterController->missionController()->plannedHomePosition().altitude(); }
double ChimeraMissionItem::amslEntryAlt() const { return homeAltitude() + _values.value("altitude").toDouble(); }
double ChimeraMissionItem::minAMSLAltitude() const
{
    double alt = amslEntryAlt();
    for (const auto& p : _route) alt = qMin(alt, homeAltitude() + p.value<QGeoCoordinate>().altitude());
    return alt;
}
double ChimeraMissionItem::maxAMSLAltitude() const
{
    double alt = amslEntryAlt();
    for (const auto& p : _route) alt = qMax(alt, homeAltitude() + p.value<QGeoCoordinate>().altitude());
    return alt;
}
double ChimeraMissionItem::complexDistance() const
{
    double distance = 0;
    for (int i = 1; i < _route.size(); ++i) distance += _route[i-1].value<QGeoCoordinate>().distanceTo(_route[i].value<QGeoCoordinate>());
    return distance;
}
double ChimeraMissionItem::greatestDistanceTo(const QGeoCoordinate& other) const
{
    double distance = other.distanceTo(coordinate());
    for (const auto& p : _route) distance = qMax(distance, other.distanceTo(p.value<QGeoCoordinate>()));
    return distance;
}
void ChimeraMissionItem::setDirty(bool dirty) { if (_dirty != dirty) { _dirty = dirty; emit dirtyChanged(dirty); } }
void ChimeraMissionItem::setSequenceNumber(int sequence)
{
    if (_sequence == sequence) return;
    _sequence = sequence;
    emit sequenceNumberChanged(sequence);
    emit lastSequenceNumberChanged(lastSequenceNumber());
}
void ChimeraMissionItem::moveInspectionPoints(const QGeoCoordinate& from, const QGeoCoordinate& to)
{
    if (!from.isValid() || !to.isValid()) return;
    const double distance = from.distanceTo(to), azimuth = from.azimuthTo(to);
    for (auto& entry : _inspections) {
        auto block = entry.toMap(); auto points = block["points"].toList();
        for (auto& value : points) {
            auto p = value.toMap();
            const auto moved = coordinateOf(p).atDistanceAndAzimuth(distance, azimuth);
            p["latitude"] = moved.latitude(); p["longitude"] = moved.longitude(); value = p;
        }
        block["points"] = points; entry = block;
    }
}
void ChimeraMissionItem::setCoordinate(const QGeoCoordinate& c)
{
    if (!c.isValid() || (centerCoordinate().isValid() && centerCoordinate().latitude() == c.latitude() && centerCoordinate().longitude() == c.longitude())) return;
    {
        QScopedValueRollback<bool> guard(_updating, true);
        const auto old = centerCoordinate();
        moveInspectionPoints(old, c);
        _lastCenter = c;
        _position = c;
        _position.setAltitude(_values["altitude"].toDouble());
        if (isSurveyItem()) {
            if (surveyAreaPolygon()->isValid()) surveyAreaPolygon()->setCenter(c);
            else surveyAreaPolygon()->setPath(QList<QGeoCoordinate>{c.atDistanceAndAzimuth(70.71,225),
                c.atDistanceAndAzimuth(70.71,135),c.atDistanceAndAzimuth(70.71,45),c.atDistanceAndAzimuth(70.71,315)});
        }
    }
    rebuild();
}
void ChimeraMissionItem::setValue(const QString& key, double value)
{
    if (!std::isfinite(value)) return;
    if (key == "latitude" || key == "longitude") {
        auto c = centerCoordinate();
        if (key == "latitude") c.setLatitude(value); else c.setLongitude(value);
        setCoordinate(c);
        return;
    }
    if (!_values.contains(key) || _values[key] == value) return;
    _values[key] = value;
    rebuild();
}
void ChimeraMissionItem::addInspection(int checkpoint)
{
    const auto path = _preview.value("path").toList();
    if (!isSurveyItem() || checkpoint < 0 || checkpoint >= path.size()) return;
    const auto c = path[checkpoint].value<QGeoCoordinate>();
    QVariantList points{point(c, true, 0), point(c, false, _values["yaw"].toDouble())};
    _inspections.append(QVariantMap{{"checkpoint", checkpoint}, {"points", points}});
    rebuild();
}
void ChimeraMissionItem::removeInspection(int index)
{
    if (index < 0 || index >= _inspections.size()) return;
    _inspections.removeAt(index); rebuild();
}
void ChimeraMissionItem::addInspectionPoint(int index, bool roi)
{
    if (index < 0 || index >= _inspections.size()) return;
    auto block = _inspections[index].toMap(); auto points = block["points"].toList();
    auto c = points.isEmpty() ? coordinate() : coordinateOf(points.last().toMap());
    if (!roi && c.altitude() <= 0) c.setAltitude(_values["altitude"].toDouble());
    points.append(point(c, roi, _values["yaw"].toDouble()));
    block["points"] = points; _inspections[index] = block; rebuild();
}
void ChimeraMissionItem::removeInspectionPoint(int index, int n)
{
    if (index < 0 || index >= _inspections.size()) return;
    auto block = _inspections[index].toMap(); auto points = block["points"].toList();
    if (n < 0 || n >= points.size()) return;
    points.removeAt(n); block["points"] = points; _inspections[index] = block; rebuild();
}
void ChimeraMissionItem::setInspectionPoint(int index, int n, const QString& key, double value)
{
    if (index < 0 || index >= _inspections.size() || !std::isfinite(value)) return;
    auto block = _inspections[index].toMap(); auto points = block["points"].toList();
    if (n < 0 || n >= points.size()) return;
    auto p = points[n].toMap();
    if (!p.contains(key) || key == "roi") return;
    p[key] = value; points[n] = p; block["points"] = points; _inspections[index] = block; rebuild();
}

void ChimeraMissionItem::rebuild()
{
    if (_updating) return;
    QScopedValueRollback<bool> guard(_updating, true);
    const auto center = centerCoordinate();
    if (isSurveyItem() && surveyAreaPolygon()->centerDrag()) moveInspectionPoints(_lastCenter, center);
    _lastCenter = center;
    qDeleteAll(_commands); _commands.clear(); _route.clear(); _error.clear(); _preview.clear();
    const double altitude = _values["altitude"].toDouble(), yaw = _values["yaw"].toDouble();
    const double speed = _values["speed"].toDouble(), pitch = _values["pitch"].toDouble();
    if (altitude <= 0 || speed <= 0 || yaw < 0 || yaw >= 360 || pitch < -90 || pitch >= 0)
        _error = tr("Use positive altitude and speed, heading 0 to 359.99, and pitch -90 to below 0.");
    auto cmd = [this](MAV_CMD command, double p1 = 0, double p2 = 0, double p3 = 0) {
        auto* item = new MissionItem(0, command, MAV_FRAME_MISSION, p1,p2,p3,0,0,0,
            command == MAV_CMD_DO_MOUNT_CONTROL ? MAV_MOUNT_MODE_MAVLINK_TARGETING : 0, true,false,this);
        _commands.append(item); return item;
    };
    auto nav = [&](const QGeoCoordinate& c, double heading, bool roi = false) {
        auto* item = cmd(roi ? MAV_CMD_DO_SET_ROI_LOCATION : MAV_CMD_NAV_WAYPOINT);
        item->setFrame(MAV_FRAME_GLOBAL_RELATIVE_ALT);
        item->setParam4(heading); item->setParam5(c.latitude()); item->setParam6(c.longitude()); item->setParam7(c.altitude());
    };
    auto restore = [&] {
        cmd(MAV_CMD_DO_CHANGE_SPEED, 1, speed, -1);
        cmd(MAV_CMD_DO_MOUNT_CONTROL, pitch);
        cmd(MAV_CMD_NAV_DELAY, 0);
    };
    if (_error.isEmpty() && isSurveyItem()) {
        _survey->cameraCalc()->distanceToSurface()->setRawValue(altitude);
        _survey->gridAngle()->setRawValue(_values["angle"]);
        _survey->setProperty("chimeraSurvey", _values);
        QObject owner;
        QList<MissionItem*> generated; QVariantList path;
        if (_planner->buildSurvey(_survey, generated, &owner, path, _error)) {
            _preview = _planner->preview(_survey);
            for (const auto& value : _inspections) {
                const auto b = value.toMap();
                if (b["checkpoint"].toInt() < 0 || b["checkpoint"].toInt() >= path.size())
                    _error = tr("An inspection checkpoint is outside the route. Remove it and select a new checkpoint.");
                for (const auto& p : b["points"].toList()) {
                    const auto v = p.toMap();
                    if (!coordinateOf(v).isValid() || !std::isfinite(coordinateOf(v).altitude()) ||
                        (!v["roi"].toBool() && (v["altitude"].toDouble() <= 0 || v["yaw"].toDouble() < 0 || v["yaw"].toDouble() >= 360)))
                        _error = tr("Check the inspection coordinates, altitude, and heading.");
                }
            }
            cmd(MAV_CMD_DO_SET_ROI_NONE); restore();
            int checkpoint = -1;
            double trigger = 0;
            for (auto* item : generated) {
                _commands.append(new MissionItem(*item, this));
                if (item->command() == MAV_CMD_DO_SET_CAM_TRIGG_DIST) trigger = item->param1();
                if (item->command() != MAV_CMD_NAV_WAYPOINT) continue;
                ++checkpoint;
                for (const auto& value : _inspections) {
                    const auto b = value.toMap();
                    if (b["checkpoint"].toInt() != checkpoint) continue;
                    cmd(MAV_CMD_DO_SET_CAM_TRIGG_DIST, 0);
                    for (const auto& value : b["points"].toList()) {
                        const auto p = value.toMap(); nav(coordinateOf(p), p["yaw"].toDouble(), p["roi"].toBool());
                    }
                    cmd(MAV_CMD_DO_SET_ROI_NONE);
                    const QGeoCoordinate resume(item->param5(), item->param6(), item->param7());
                    nav(resume, yaw);
                    restore();
                    nav(resume, yaw);
                    cmd(MAV_CMD_DO_SET_CAM_TRIGG_DIST, trigger);
                }
            }
        }
    } else if (_error.isEmpty()) {
        _position.setAltitude(altitude);
        if (!_position.isValid() || _values["delay"].toDouble() <= 1) _error = tr("Set a valid position and a delay until greater than 1 second.");
        cmd(MAV_CMD_DO_SET_CAM_TRIGG_DIST, 0);
        cmd(MAV_CMD_DO_SET_ROI_NONE);
        cmd(MAV_CMD_DO_CHANGE_SPEED, 1, speed, -1);
        if (_kind == "vantage") { cmd(MAV_CMD_DO_MOUNT_CONTROL, pitch); cmd(MAV_CMD_NAV_DELAY, 1); }
        nav(_position, yaw);
        cmd(MAV_CMD_DO_MOUNT_CONTROL, pitch);
        cmd(MAV_CMD_NAV_DELAY, _values["delay"].toDouble());
    }
    if (!_error.isEmpty()) { qDeleteAll(_commands); _commands.clear(); }
    for (auto* item : std::as_const(_commands)) {
        if (item->command() == MAV_CMD_NAV_WAYPOINT)
            _route.append(QVariant::fromValue(QGeoCoordinate(item->param5(), item->param6(), item->param7())));
    }
    _cTerrainCollisionSegments = 0;
    emit terrainCollisionChanged(false);
    _flightPathSegments.beginResetModel();
    _flightPathSegments.clearAndDeleteContents();
    for (int i = 1; i < _route.size(); ++i) {
        const auto from = _route[i-1].value<QGeoCoordinate>(), to = _route[i].value<QGeoCoordinate>();
        _appendFlightPathSegment(FlightPathSegment::SegmentTypeGeneric, from, homeAltitude() + from.altitude(), to, homeAltitude() + to.altitude());
    }
    _flightPathSegments.endResetModel();
    if (!_route.isEmpty()) {
        double north = -90, south = 90, east = -180, west = 180;
        for (const auto& value : _route) {
            const auto c = value.value<QGeoCoordinate>();
            north = qMax(north, c.latitude()); south = qMin(south, c.latitude());
            east = qMax(east, c.longitude()); west = qMin(west, c.longitude());
        }
        _setBoundingCube(QGCGeoBoundingCube(QGeoCoordinate(north, west, minAMSLAltitude()), QGeoCoordinate(south, east, maxAMSLAltitude())));
    } else _setBoundingCube(QGCGeoBoundingCube());
    emit ComplexMissionItem::boundingCubeChanged();
    _isIncomplete = !_error.isEmpty();
    setDirty(true);
    emit changed(); emit isIncompleteChanged(); emit readyForSaveStateChanged();
    emit coordinateChanged(coordinate()); emit exitCoordinateChanged(exitCoordinate());
    emit amslEntryAltChanged(amslEntryAlt()); emit amslExitAltChanged(amslExitAlt());
    emit minAMSLAltitudeChanged(); emit maxAMSLAltitudeChanged(); emit complexDistanceChanged();
    emit lastSequenceNumberChanged(lastSequenceNumber()); emit greatestDistanceToChanged();
    emit specifiedFlightSpeedChanged(); emit specifiedGimbalPitchChanged(); emit specifiedVehicleYawChanged();
    emit additionalTimeDelayChanged();
}
void ChimeraMissionItem::appendMissionItems(QList<MissionItem*>& items, QObject* parent)
{
    int seq = _sequence;
    for (auto* command : std::as_const(_commands)) {
        auto* copy = new MissionItem(*command, parent); copy->setSequenceNumber(seq++); items.append(copy);
    }
}
void ChimeraMissionItem::save(QJsonArray& items)
{
    QJsonArray polygon;
    for (const auto& c : surveyAreaPolygon()->coordinateList()) polygon.append(QJsonArray{c.latitude(), c.longitude()});
    items.append(QJsonObject{{"type", "ComplexItem"}, {"complexItemType", "chimera-" + _kind}, {"version", 1},
        {"values", QJsonObject::fromVariantMap(_values)}, {"position", QJsonArray{_position.latitude(), _position.longitude()}},
        {"polygon", polygon}, {"inspections", QJsonArray::fromVariantList(_inspections)}});
}
bool ChimeraMissionItem::load(const QJsonObject& json, int sequence, QString& error)
{
    if (json["version"].toInt() != 1 || !json["values"].isObject() || !json["inspections"].isArray()) {
        error = tr("Invalid Chimera block format or version."); return false;
    }
    {
        QScopedValueRollback<bool> guard(_updating, true);
        const auto values = json["values"].toObject().toVariantMap();
        for (auto it = _values.begin(); it != _values.end(); ++it) {
            if (!values.contains(it.key()) || !std::isfinite(values[it.key()].toDouble())) {
                error = tr("Missing or invalid Chimera setting: %1").arg(it.key()); return false;
            }
            it.value() = values[it.key()];
        }
        const auto position = json["position"].toArray();
        _position = QGeoCoordinate(position[0].toDouble(), position[1].toDouble(), _values["altitude"].toDouble());
        QList<QGeoCoordinate> polygon;
        for (const auto& p : json["polygon"].toArray()) { const auto a = p.toArray(); polygon.append(QGeoCoordinate(a[0].toDouble(), a[1].toDouble())); }
        surveyAreaPolygon()->setPath(polygon);
        _inspections = json["inspections"].toArray().toVariantList();
        _sequence = sequence;
    }
    rebuild();
    error = _error;
    return error.isEmpty();
}
