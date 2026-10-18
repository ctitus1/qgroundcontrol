#include "ChimeraMissionItem.h"
#include "ChimeraPlanner.h"
#include "SurveyComplexItem.h"
#include "PlannerSurvey.h"
#include "CameraCalc.h"
#include "QGCMapPolygon.h"
#include "MissionItem.h"
#include "PlanMasterController.h"
#include <QJsonArray>
#include <QJsonObject>
#include <QScopedValueRollback>
#include <QUuid>
#include "SettingsManager.h"
#include "AppSettings.h"
#include <QFile>
#include <QJsonDocument>
#include "Fact.h"
#include "FactMetaData.h"

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
QList<QGeoCoordinate> routePoints(const QJsonArray& commands)
{
    QList<QGeoCoordinate> points;
    for (const auto& value : commands) {
        const auto item = value.toObject();
        if (item["command"].toInt() != MAV_CMD_NAV_WAYPOINT) continue;
        const auto p = item["params"].toArray();
        points.append(QGeoCoordinate(p[4].toDouble(), p[5].toDouble(), p[6].toDouble()));
    }
    return points;
}
double routeLength(const QList<QGeoCoordinate>& path)
{
    double length = 0;
    for (int i = 1; i < path.size(); ++i) length += path[i-1].distanceTo(path[i]);
    return length;
}
double routeFraction(const QList<QGeoCoordinate>& path, const QGeoCoordinate& coordinate, double* nearestDistance = nullptr)
{
    const double total = routeLength(path);
    if (total <= 0 || !coordinate.isValid()) return -1;
    double travelled = 0, nearest = INFINITY, fraction = 0;
    for (int i = 1; i < path.size(); ++i) {
        const auto from = path[i-1], to = path[i];
        const double length = from.distanceTo(to);
        if (length < 0.001) continue;
        const double bearing = (from.azimuthTo(coordinate) - from.azimuthTo(to)) * M_PI / 180;
        const double offset = qBound(0., from.distanceTo(coordinate) * std::cos(bearing), length);
        const auto projected = from.atDistanceAndAzimuth(offset, from.azimuthTo(to));
        const double distance = projected.distanceTo(coordinate);
        if (distance < nearest) { nearest = distance; fraction = (travelled + offset) / total; }
        travelled += length;
    }
    if (nearestDistance) *nearestDistance = nearest;
    return fraction;
}
QPair<QJsonArray, QJsonArray> cutRoute(const QJsonArray& commands, double fraction)
{
    const auto points = routePoints(commands);
    const double target = fraction * routeLength(points);
    double travelled = 0, trigger = 0;
    int waypoint = -1, cut = -1;
    QGeoCoordinate endpoint;
    for (int i = 0; i < commands.size(); ++i) {
        const auto item = commands[i].toObject();
        if (item["command"].toInt() == MAV_CMD_DO_SET_CAM_TRIGG_DIST) trigger = item["params"].toArray()[0].toDouble();
        if (item["command"].toInt() != MAV_CMD_NAV_WAYPOINT) continue;
        ++waypoint;
        if (waypoint + 1 >= points.size()) break;
        const double length = points[waypoint].distanceTo(points[waypoint+1]);
        if (length > 0 && travelled + length >= target) {
            endpoint = points[waypoint].atDistanceAndAzimuth(qBound(0., target-travelled, length), points[waypoint].azimuthTo(points[waypoint+1]));
            cut = i; break;
        }
        travelled += length;
    }
    if (cut < 0) return {};
    auto endCommand = commands[cut].toObject(); auto params = endCommand["params"].toArray();
    params[4] = endpoint.latitude(); params[5] = endpoint.longitude(); endCommand["params"] = params;
    MissionItem triggerItem(0, MAV_CMD_DO_SET_CAM_TRIGG_DIST, MAV_FRAME_MISSION, 0,0,0,0,0,0,0,true,false);
    QJsonObject stop, restart; triggerItem.save(stop); triggerItem.setParam1(trigger); triggerItem.save(restart);
    QJsonArray first, second;
    for (int i = 0; i <= cut; ++i) first.append(commands[i]);
    first.append(endCommand); first.append(stop);
    second.append(endCommand); second.append(restart);
    for (int i = cut + 1; i < commands.size(); ++i) second.append(commands[i]);
    return {first, second};
}

}

ChimeraMissionItem::ChimeraMissionItem(PlanMasterController* master, bool flyView,
                                     const QString& kind, ChimeraPlanner* planner)
    : ComplexMissionItem(master, flyView), _kind(kind), _planner(planner),
      _survey(new SurveyComplexItem(master, flyView, QString())),
      _flightSurvey(new PlannerSurvey(master, flyView, QString()))
{
    _survey->setParent(this);
    _flightSurvey->setParent(this);
    _editorQml = "qrc:/Planner/BlockEditor.qml";
    _values = planner->settings(nullptr);
    _values["enabled"] = true;
    _values["altitude"] = SettingsManager::instance()->appSettings()->defaultMissionItemAltitude()->rawValue();
    _values["speed"] = 1.;
    _values["delay"] = 60.;
    _values["angle"] = 0.;
    _values["pitch"] = -70.;
    auto keys = _values.keys(); keys.append("latitude"); keys.append("longitude");
    for (const auto& key : keys) {
        if (key == "enabled") continue;
        auto* field = new Fact(0, key, FactMetaData::valueTypeDouble, this);
        auto* metadata = field->metaData();
        metadata->setDecimalPlaces(key == "latitude" || key == "longitude" ? 7 :
            QStringList{"fx", "fy", "cx", "cy"}.contains(key) ? 6 :
            key == "width" || key == "height" ? 0 : 1);
        if (key == "altitude") { metadata->setRawUnits("m"); metadata->setRawMin(0.1); }
        if (key == "speed") { metadata->setRawUnits("m/s"); metadata->setRawMin(0.1); }
        if (key == "delay") { metadata->setRawUnits("s"); metadata->setRawMin(1.01); }
        if (key == "yaw" || key == "angle") { metadata->setRawUnits("deg"); metadata->setRawMin(0); metadata->setRawMax(359.99); }
        if (key == "pitch") { metadata->setRawUnits("deg"); metadata->setRawMin(-90); metadata->setRawMax(-0.01); }
        if (key == "overlap") { metadata->setRawUnits("%"); metadata->setRawMin(0); metadata->setRawMax(94.99); }
        if (key == "latitude") { metadata->setRawMin(-90); metadata->setRawMax(90); }
        if (key == "longitude") { metadata->setRawMin(-180); metadata->setRawMax(180); }
        if (QStringList{"width", "height", "fx", "fy", "cx", "cy"}.contains(key)) { metadata->setRawUnits("px"); metadata->setRawMin(key == "cx" || key == "cy" ? 0 : 0.1); }
        field->setRawValue(_values.value(key, 0));
        _facts.insert(key, field);
        connect(field, &Fact::rawValueChanged, this, [this, key](const QVariant& value) { if (!_updating) setValue(key, value.toDouble()); });
    }
    _survey->cameraCalc()->setDistanceMode(QGroundControlQmlGlobal::AltitudeModeRelative);
    _survey->turnAroundDistance()->setRawValue(0);
    _survey->setWizardMode(false);
    connect(surveyAreaPolygon(), &QGCMapPolygon::pathChanged, this, [this] {
        if (_updating) return;
        if (surveyAreaPolygon()->centerDrag()) {
            const auto center = centerCoordinate();
            const double distance = _lastCenter.distanceTo(center), azimuth = _lastCenter.azimuthTo(center);
            moveInspectionPoints(_lastCenter, center);
            _lastCenter = center;
            emit surveyAreaPolygon()->centerChanged(center);
            emit translated(distance, azimuth);
            for (auto* part : linkedParts()) if (part != this) emit part->translated(distance, azimuth);
            return;
        }
        rebuild(); syncSurvey();
    });
    connect(surveyAreaPolygon(), &QGCMapPolygon::centerDragChanged, this, [this](bool dragging) {
        if (!dragging && !_updating) {
            moveInspectionPoints(_lastCenter, centerCoordinate());
            _lastCenter = centerCoordinate();
            rebuild(); syncSurvey();
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
    for (int i = 0; i < _frozen.size(); ++i) {
        auto item = _frozen[i].toObject();
        if (item["command"].toInt() != MAV_CMD_NAV_WAYPOINT) continue;
        auto params = item["params"].toArray();
        const auto moved = QGeoCoordinate(params[4].toDouble(), params[5].toDouble()).atDistanceAndAzimuth(from.distanceTo(to), from.azimuthTo(to));
        params[4] = moved.latitude(); params[5] = moved.longitude(); item["params"] = params; _frozen[i] = item;
    }
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
    rebuild(); syncSurvey();
}
QVariantList ChimeraMissionItem::cameraPresets() const
{
    static const QVariantList presets = [] {
        QFile file(":/Planner/CameraPresets.json");
        if (!file.open(QIODevice::ReadOnly)) return QVariantList{};
        return QJsonDocument::fromJson(file.readAll()).array().toVariantList();
    }();
    return presets;
}

int ChimeraMissionItem::cameraPreset() const
{
    const auto presets = cameraPresets();
    for (int i = 1; i < presets.size(); ++i) if (presets[i].toMap()["id"].toString() == _cameraPreset) return i;
    return 0;
}

void ChimeraMissionItem::setCameraPreset(int index)
{
    const auto presets = cameraPresets();
    if (!isSurveyItem() || splitPart() || index < 0 || index >= presets.size()) return;
    const auto preset = presets[index].toMap();
    _cameraPreset = preset["id"].toString();
    if (index > 0) {
        const auto matrix = preset["camera_matrix"].toMap()["data"].toList();
        _values["width"] = preset["image_width"];
        _values["height"] = preset["image_height"];
        _values["fx"] = matrix[0]; _values["fy"] = matrix[4];
        _values["cx"] = matrix[2]; _values["cy"] = matrix[5];
    }
    rebuild(); syncSurvey();
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
    if (splitPart() && key != "speed") return;
    if (!_values.contains(key)) return;
    const bool angle = isSurveyItem() && (key == "yaw" || key == "angle");
    if (_values[key] == value && (!angle || (_values["yaw"] == value && _values["angle"] == value))) return;
    if (angle) {
        if (value < 0 || value >= 360) return;
        _values["yaw"] = value; _values["angle"] = value;
    }
    if (QStringList{"width", "height", "fx", "fy", "cx", "cy"}.contains(key)) _cameraPreset.clear();
    _values[key] = value;
    rebuild(); syncSurvey();
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
    if (altitude <= 0 || speed <= 0 || yaw < 0 || yaw >= 360 || (isSurveyItem() && (pitch < -90 || pitch >= 0)))
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
        _survey->setProperty("chimeraCenterOut", _centerOut);
        QObject owner;
        QList<MissionItem*> generated; QVariantList path;
        bool built = true;
        if (splitPart()) {
            for (const auto& value : _frozen) {
                auto* item = new MissionItem(&owner);
                if (!item->load(value.toObject(), 0, _error)) { built = false; break; }
                if (item->command() != MAV_CMD_NAV_WAYPOINT && item->command() != MAV_CMD_DO_SET_CAM_TRIGG_DIST) {
                    _error = tr("Invalid command in a split survey."); built = false; break;
                }
                generated.append(item);
                if (item->command() == MAV_CMD_NAV_WAYPOINT) path.append(QVariant::fromValue(QGeoCoordinate(item->param5(), item->param6(), item->param7())));
            }
            if (path.size() < 2) { _error = tr("A split survey needs at least two route points."); built = false; }
            for (const auto& value : path) {
                const auto c = value.value<QGeoCoordinate>();
                if (!c.isValid() || !std::isfinite(c.altitude()) || c.altitude() <= 0) {
                    _error = tr("Invalid coordinate in a split survey."); built = false;
                }
            }
            _preview = {{"path", path}};
        } else {
            auto parameters = _values;
            parameters.remove("angle");
            parameters.remove("yaw");
            parameters.remove("speed");
            const bool configure = parameters != _generationValues;
            if (configure) _generationValues.clear();
            built = _planner->buildSurvey(_survey, generated, &owner, path, _error, _flightSurvey, configure);
            if (built) {
                _generationValues = parameters;
                _preview = _planner->preview(_survey, path);
            }
        }
        _generated = {};
        if (built) {
            for (auto* item : generated) { QJsonObject json; item->save(json); _generated.append(json); }
            if (linkedSurvey()) {
                auto section = _end < 1 ? cutRoute(_generated, _end).first : _generated;
                if (_start > 0) section = cutRoute(section, _start / _end).second;
                generated.clear(); path.clear();
                for (const auto& value : section) {
                    auto* item = new MissionItem(&owner);
                    if (!item->load(value.toObject(), 0, _error)) break;
                    generated.append(item);
                    if (item->command() == MAV_CMD_NAV_WAYPOINT) path.append(QVariant::fromValue(QGeoCoordinate(item->param5(), item->param6(), item->param7())));
                }
                _preview["path"] = path;
                if (path.size() < 2) _error = tr("The survey section has no flight path.");
                _resume = _start > 0;
            }
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
            cmd(MAV_CMD_DO_SET_CAM_TRIGG_DIST, 0);
            cmd(MAV_CMD_DO_SET_ROI_NONE);
            if (_resume && !path.isEmpty()) {
                cmd(MAV_CMD_DO_CHANGE_SPEED, 1, speed, -1);
                nav(path.first().value<QGeoCoordinate>(), yaw);
            }
            restore();
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
        if (_kind == "vantage") cmd(MAV_CMD_NAV_DELAY, 1);
        cmd(MAV_CMD_DO_SET_CAM_TRIGG_DIST, 0);
        cmd(MAV_CMD_DO_CHANGE_SPEED, 1, speed, -1);
        nav(_position, yaw);
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
    for (auto it = _facts.begin(); it != _facts.end(); ++it) {
        const double value = it.key() == "latitude" ? centerCoordinate().latitude() : it.key() == "longitude" ? centerCoordinate().longitude() : _values[it.key()].toDouble();
        if (std::isfinite(value)) it.value()->setRawValue(value);
    }
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
    items.append(QJsonObject{{"type", "ComplexItem"}, {"complexItemType", "chimera-" + _kind}, {"version", linkedSurvey() ? 3 : splitPart() ? 2 : 1},
        {"fleetRegion", _fleetRegion}, {"centerOut", _centerOut}, {"cameraPreset", _cameraPreset}, {"values", QJsonObject::fromVariantMap(_values)}, {"position", QJsonArray{_position.latitude(), _position.longitude()}},
        {"polygon", polygon}, {"surveyGroup", _groupId}, {"rangeStart", _start}, {"rangeEnd", _end}, {"frozenRoute", _frozen}, {"resume", _resume}, {"inspections", QJsonArray::fromVariantList(_inspections)}});
}
bool ChimeraMissionItem::load(const QJsonObject& json, int sequence, QString& error)
{
    const int version = json["version"].toInt();
    if ((version != 1 && version != 2 && version != 3) || (version == 2 && json["frozenRoute"].toArray().isEmpty()) || !json["values"].isObject() || !json["inspections"].isArray()) {
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
        _fleetRegion = json["fleetRegion"].toInt(-1);
        _centerOut = json["centerOut"].toBool();
        _cameraPreset = json["cameraPreset"].toString();
        _syncedCameraPreset = _cameraPreset;
        _groupId = json["surveyGroup"].toString();
        _start = json["rangeStart"].toDouble(0); _end = json["rangeEnd"].toDouble(1);
        if (version == 3 && (_groupId.isEmpty() || !std::isfinite(_start) || !std::isfinite(_end) || _start < 0 || _end > 1 || _start >= _end)) {
            error = tr("Invalid linked survey range."); return false;
        }
        _frozen = json["frozenRoute"].toArray();
        _resume = json["resume"].toBool();
        _syncedValues = _values; _syncedPolygon = surveyAreaPolygon()->coordinateList();
        _sequence = sequence;
    }
    rebuild();
    error = _error;
    return error.isEmpty();
}

QList<ChimeraMissionItem*> ChimeraMissionItem::linkedParts() const
{
    QList<ChimeraMissionItem*> parts;
    if (_groupId.isEmpty()) return parts;
    auto* model = _masterController->missionController()->visualItems();
    for (int i = 1; i < model->count(); ++i) {
        auto* item = qobject_cast<ChimeraMissionItem*>(model->get(i));
        if (item && item->_groupId == _groupId) parts.append(item);
    }
    return parts;
}

void ChimeraMissionItem::syncSurvey()
{
    if (_updating || !linkedSurvey()) return;
    const auto polygon = surveyAreaPolygon()->coordinateList();
    if (_syncedValues == _values && _syncedPolygon == polygon && _syncedCameraPreset == _cameraPreset) return;
    _syncedValues = _values; _syncedPolygon = polygon; _syncedCameraPreset = _cameraPreset;
    for (auto* item : linkedParts()) {
        if (item == this || (item->_cameraPreset == _cameraPreset && item->_values == _values && item->surveyAreaPolygon()->coordinateList() == surveyAreaPolygon()->coordinateList() && item->_position == _position)) continue;
        {
            QScopedValueRollback<bool> guard(item->_updating, true);
            item->_cameraPreset = _cameraPreset; item->_syncedCameraPreset = _cameraPreset;
            item->_values = _values;
            item->_syncedValues = _values; item->_syncedPolygon = polygon;
            item->surveyAreaPolygon()->setPath(surveyAreaPolygon()->coordinateList());
            item->_position = _position;
        }
        item->rebuild();
    }
}

ChimeraMissionItem* ChimeraMissionItem::nextPart() const
{
    for (auto* item : linkedParts()) if (item != this && qAbs(item->_start - _end) < 1e-8) return item;
    return nullptr;
}

QString ChimeraMissionItem::linkedSurveyError() const
{
    if (!linkedSurvey()) return {};
    double end = 0;
    for (auto* item : linkedParts()) {
        if (qAbs(item->_start - end) > 1e-8 || item->_values != _values || item->surveyAreaPolygon()->coordinateList() != surveyAreaPolygon()->coordinateList())
            return tr("The linked survey has missing, reordered, or inconsistent parts. Restore the parts or remove the whole survey.");
        end = item->_end;
    }
    return qAbs(end - 1) < 1e-8 ? QString() : tr("The linked survey is missing its final part.");
}

QVariantList ChimeraMissionItem::splitHandles() const
{
    QVariantList handles;
    const auto path = _preview.value("path").toList();
    if (splitPart() || !_inspections.isEmpty()) return handles;
    for (int i = 1; i < path.size(); ++i) {
        const auto a = path[i-1].value<QGeoCoordinate>(), b = path[i].value<QGeoCoordinate>();
        if (a.distanceTo(b) > 1) handles.append(QVariant::fromValue(a.atDistanceAndAzimuth(a.distanceTo(b)/2, a.azimuthTo(b))));
    }
    return handles;
}

QString ChimeraMissionItem::addSplitPoint(const QGeoCoordinate& coordinate)
{
    const auto path = _preview.value("path").toList();
    QList<QGeoCoordinate> points;
    for (const auto& value : path) points.append(value.value<QGeoCoordinate>());
    const double fraction = routeFraction(points, coordinate);
    if (fraction <= 0 || fraction >= 1) return tr("Choose a point inside the survey path.");
    return splitAt(_start + fraction * (_end - _start));
}

QString ChimeraMissionItem::splitAt(double fraction)
{
    if (!isSurveyItem() || splitPart() || !_error.isEmpty() || !_inspections.isEmpty())
        return tr("Use an editable survey without embedded inspections to add split points.");
    if (fraction - _start < 1e-6 || _end - fraction < 1e-6) return tr("The split point is too close to an existing split.");
    auto* controller = _masterController->missionController();
    int index = -1;
    for (int i = 1; i < controller->visualItems()->count(); ++i) if (controller->visualItems()->get(i) == this) index = i;
    if (index < 0) return tr("The survey is not in this mission.");
    const auto group = _groupId.isEmpty() ? QUuid::createUuid().toString(QUuid::WithoutBraces) : _groupId;
    QJsonArray saved; save(saved);
    auto json = saved.first().toObject();
    json["version"] = 3; json["surveyGroup"] = group; json["rangeStart"] = fraction; json["rangeEnd"] = _end;
    auto* continuation = qobject_cast<ChimeraMissionItem*>(controller->insertComplexMissionItem("Chimera survey", centerCoordinate(), index + 1));
    if (!continuation) return tr("Could not create the survey continuation.");
    QString error;
    if (!continuation->load(json, continuation->sequenceNumber(), error)) { controller->removeVisualItem(index + 1); return error; }
    _groupId = group; _end = fraction;
    _syncedValues = _values; _syncedPolygon = surveyAreaPolygon()->coordinateList();
    rebuild();
    controller->setCurrentPlanViewSeqNum(sequenceNumber(), true);
    return {};
}

void ChimeraMissionItem::moveEndSplit(const QGeoCoordinate& coordinate)
{
    auto* next = nextPart();
    if (!next) return;
    const double fraction = routeFraction(routePoints(_generated), coordinate);
    if (fraction <= _start + 1e-6 || fraction >= next->_end - 1e-6) return;
    _end = fraction; next->_start = fraction;
    rebuild(); next->rebuild();
}

QString ChimeraMissionItem::removeEndSplit()
{
    auto* next = nextPart();
    if (!next) return tr("No following survey part was found.");
    auto* controller = _masterController->missionController();
    int index = -1;
    for (int i = 1; i + 1 < controller->visualItems()->count(); ++i)
        if (controller->visualItems()->get(i) == this && controller->visualItems()->get(i+1) == next) index = i;
    if (index < 0) return tr("Move or remove the manual mission items between these parts before removing this split.");
    _end = next->_end;
    controller->removeVisualItem(index + 1);
    if (_start == 0 && _end == 1) _groupId.clear();
    rebuild();
    controller->setCurrentPlanViewSeqNum(sequenceNumber(), true);
    return {};
}

QString ChimeraMissionItem::upgradeFrozenSurvey()
{
    if (!splitPart()) return {};
    QObject owner; QList<MissionItem*> commands; QVariantList path; QString error;
    if (!_planner->buildSurvey(_survey, commands, &owner, path, error)) return error;
    QList<QGeoCoordinate> full;
    for (const auto& value : path) full.append(value.value<QGeoCoordinate>());
    QList<ChimeraMissionItem*> parts;
    QList<QPair<double, double>> ranges;
    auto* model = _masterController->missionController()->visualItems();
    double end = 0;
    for (int i = 1; i < model->count(); ++i) {
        auto* item = qobject_cast<ChimeraMissionItem*>(model->get(i));
        if (!item || !item->splitPart() || item->_values != _values || item->surveyAreaPolygon()->coordinateList() != surveyAreaPolygon()->coordinateList()) continue;
        const auto points = routePoints(item->_frozen);
        if (points.size() < 2) continue;
        double startDistance = INFINITY, endDistance = INFINITY;
        const double start = routeFraction(full, points.first(), &startDistance), finish = routeFraction(full, points.last(), &endDistance);
        if (startDistance > 1 || endDistance > 1) return tr("The legacy route differs from its source. Open the original unsplit plan.");
        if (qAbs(start - end) > 1e-6 || finish <= start) return tr("These legacy parts cannot be linked automatically. Open the original unsplit plan.");
        parts.append(item); ranges.append({start, finish}); end = finish;
    }
    if (parts.isEmpty() || qAbs(end - 1) > 1e-6) return tr("All original survey parts are needed to restore the editable source.");
    const auto group = QUuid::createUuid().toString(QUuid::WithoutBraces);
    for (int i = 0; i < parts.size(); ++i) {
        auto* item = parts[i]; item->_groupId = group; item->_start = ranges[i].first; item->_end = ranges[i].second;
        item->_syncedValues = item->_values; item->_syncedPolygon = item->surveyAreaPolygon()->coordinateList();
        item->_frozen = {}; item->rebuild();
    }
    return {};
}
