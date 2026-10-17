#include "FleetPlanner.h"
#include "ChimeraMissionItem.h"
#include "QGCMapPolygon.h"
#include "QmlObjectListModel.h"
#include "SimpleMissionItem.h"
#include "Fact.h"
#include "Vehicle.h"
#include "MissionSettingsItem.h"
#include "Splitters/ControlPointSplitter.h"
#include <QJsonArray>
#include <QJsonDocument>
#include <QSaveFile>
#include <QScopedValueRollback>
#include <QUrl>
#include <QCoreApplication>
#include <cmath>
#include <limits>

namespace {
QString color(int i) { return QStringList{"#00d8ef", "#ffb040", "#eb79c6", "#80d56b", "#a68cff", "#ff7865", "#d9d35b", "#73b5ff"}[i % 8]; }
QList<ChimeraMissionItem*> blocks(PlanMasterController* plan) {
    QList<ChimeraMissionItem*> result;
    if (!plan) return result;
    auto* items = plan->missionController()->visualItems();
    for (int i=1; i<items->count(); ++i) if (auto* b = qobject_cast<ChimeraMissionItem*>(items->get(i))) result.append(b);
    return result;
}
}
FleetPlanner::FleetPlanner(QObject* parent) : QObject(parent)
{
    connect(QCoreApplication::instance(), &QCoreApplication::aboutToQuit, this, [this] { clear(true); qDeleteAll(findChildren<PlanMasterController*>(QString(),Qt::FindDirectChildrenOnly)); });
    _syncTimer.setSingleShot(true); _syncTimer.setInterval(100);
    _previewTimer.setSingleShot(true); _previewTimer.setInterval(100);
    connect(&_syncTimer, &QTimer::timeout, this, &FleetPlanner::synchronize);
    connect(&_previewTimer, &QTimer::timeout, this, [this] { watch(); emit changed(); });
}
void FleetPlanner::attach(PlanMasterController* master)
{
    if (_master == master) return;
    clear(); _master = master;
    if (master) connect(master->missionController(), &MissionController::visualItemsChanged, this, [this] { watch(); });
    watch(); emit changed();
}
void FleetPlanner::clear(bool immediate)
{
    _selected = 0; _syncTimer.stop(); _previewTimer.stop();
    auto children = _children; _children.clear(); _profiles.clear(); _error.clear();
    emit changed();
    for (auto* child : children) { if (immediate) delete child; else child->deleteLater(); }
}
PlanMasterController* FleetPlanner::makeChild()
{
    auto* child = new PlanMasterController(this);
    child->setFlyView(false);
    child->start(true);
    _children.append(child);
    connect(child->missionController(), &MissionController::visualItemsChanged, this, [this] { watch(); });
    return child;
}
bool FleetPlanner::owns(PlanMasterController* controller) const { return isMaster(controller) || _children.contains(controller); }
PlanMasterController* FleetPlanner::activeController() const { return _selected > 0 && _selected <= _children.size() ? _children[_selected-1] : _master.data(); }
void FleetPlanner::setSelected(int selected) { if (selected >= 0 && selected <= _children.size()) { _selected=selected; emit changed(); } }
ChimeraMissionItem* FleetPlanner::domain() const {
    ChimeraMissionItem* result = nullptr;
    for (auto* b : blocks(_master)) if (b->kind() == "survey") { if (result) return nullptr; result=b; }
    return result;
}
ChimeraMissionItem* FleetPlanner::fiducial() const {
    ChimeraMissionItem* result = nullptr;
    for (auto* b : blocks(_master)) if (b->kind() == "fiducial") { if (result) return nullptr; result=b; }
    return result;
}
double FleetPlanner::separation() const {
    if (_profiles.size() < 2) return 0;
    const double gap = (_maximum-_minimum)/(_profiles.size()-1);
    return _maximumGap > 0 ? std::min(gap,_maximumGap) : gap;
}
QList<QList<QGeoCoordinate>> FleetPlanner::regions(QString& error) const
{
    auto* source = domain();
    if (!source || !fiducial()) { error=tr("The master needs one Chimera survey and one shared fiducial."); return {}; }
    if (source->linkedSurvey() || source->splitPart() || !source->inspections().isEmpty()) { error=tr("Add survey interruptions in the drone views, not in the master area."); return {}; }
    const auto polygon = source->surveyAreaPolygon()->coordinateList();
    if (polygon.size()<3) { error=tr("Draw the shared search area."); return {}; }
    const auto center = source->centerCoordinate();
    QList<QPointF> points;
    for (const auto& c : polygon) { const double a=center.azimuthTo(c)*M_PI/180; points.append(QPointF(std::sin(a),std::cos(a))*center.distanceTo(c)); }
    double sign=0;
    for (int i=0;i<points.size();++i) {
        const auto a=points[(i+1)%points.size()]-points[i], b=points[(i+2)%points.size()]-points[(i+1)%points.size()];
        const double cross=a.x()*b.y()-a.y()*b.x();
        if (std::abs(cross)<0.001) continue;
        if (sign*cross<0) { error=tr("Fleet regions currently need a convex search area."); return {}; }
        sign=cross;
    }
    SplitInput input; input.masterPolygon=polygon; input.center=center;
    auto cross = [](const QPointF& a, const QPointF& b) { return a.x()*b.y()-a.y()*b.x(); };
    for (int i=0;i<_profiles.size();++i) {
        const double angle = 2*M_PI*i/_profiles.size();
        const QPointF ray(std::sin(angle),std::cos(angle));
        double distance = std::numeric_limits<double>::infinity();
        for (int j=0;j<points.size();++j) {
            const QPointF edge=points[(j+1)%points.size()]-points[j];
            const double denominator=cross(ray,edge);
            if (std::abs(denominator)<1e-9) continue;
            const double along=cross(points[j],edge)/denominator, fraction=cross(points[j],ray)/denominator;
            if (along>0 && fraction>=0 && fraction<=1) distance=std::min(distance,along);
        }
        if (!std::isfinite(distance)) { error=tr("The region divider does not meet the area boundary."); return {}; }
        input.edgePoints.append(center.atDistanceAndAzimuth(distance,360.*i/_profiles.size()));
    }
    const auto split=ControlPointSplitter().split(input,error);
    if (split.size()!=_profiles.size()) { error=tr("The area could not be divided into one region per drone."); return {}; }
    QList<QList<QGeoCoordinate>> result;
    for (const auto& region : split) result.append(region.polygon);
    return result;
}
QString FleetPlanner::create(QObject* object,int count,double minimum,double maximum,double maximumGap)
{
    auto* source=qobject_cast<ChimeraMissionItem*>(object);
    if (!source || source->kind()!="survey" || count<2 || count>8) return tr("Select a survey and use 2 to 8 drones.");
    if (enabled()) return tr("This master already has drone missions. Edit them from the fleet views.");
    attach(source->masterController());
    if (!_master->controllerVehicle()->multiRotor()) return tr("Fleet planning currently supports multicopters.");
    if (domain()!=source || !fiducial()) return tr("The master needs one survey and one shared fiducial.");
    if (!source->error().isEmpty()) return source->error();
    if (!fiducial()->error().isEmpty()) return fiducial()->error();
    if (!std::isfinite(minimum) || !std::isfinite(maximum) || !std::isfinite(maximumGap) || minimum<=0 || maximum<=minimum || maximumGap<0)
        return tr("Use positive altitude limits, maximum above minimum, and a nonnegative maximum gap.");
    _minimum=minimum; _maximum=maximum; _maximumGap=maximumGap;
    auto home=_master->missionController()->plannedHomePosition();
    if (!home.isValid() || !std::isfinite(home.altitude())) return tr("Set the master launch location and elevation before creating drone missions.");
    for (int i=0;i<count;++i) _profiles.append(QVariantMap{{"name",QString("Drone %1").arg(i+1)}, {"color",color(i)},
        {"heading",std::fmod(source->values()["yaw"].toDouble()+360.*i/count,360.)}, {"latitude",home.latitude()}, {"longitude",home.longitude()}});
    QString error; const auto polygons=regions(error);
    if (!error.isEmpty()) { _profiles.clear(); return error; }
    QScopedValueRollback<bool> guard(_updating,true);
    for (int i=0;i<count;++i) {
        auto* child=makeChild(); auto* mission=child->missionController();
        auto* takeoff=mission->insertTakeoffItem(home,1);
        takeoff->setCoordinate(home); takeoff->setWizardMode(false);
        QJsonArray fid; fiducial()->save(fid);
        auto* marker=qobject_cast<ChimeraMissionItem*>(mission->insertComplexMissionItem("Chimera fiducial",fiducial()->coordinate(),2));
        marker->load(fid.first().toObject(),1,error); marker->setFleetRegion(-2);
        QJsonArray saved; source->save(saved);
        for (int phase=0;phase<count;++phase) {
            const int region=(i+phase)%count;
            auto* survey=qobject_cast<ChimeraMissionItem*>(mission->insertComplexMissionItem("Chimera survey",source->centerCoordinate(),3+phase));
            survey->load(saved.first().toObject(),1,error); survey->setFleetRegion(region);
            survey->surveyAreaPolygon()->setPath(polygons[region]);
        }
        auto* rtl=qobject_cast<SimpleMissionItem*>(mission->insertSimpleMissionItem(home,3+count));
        rtl->missionItem().setCommand(MAV_CMD_NAV_RETURN_TO_LAUNCH);
    }
    _syncTimer.start(); watch(); _master->setDirty(true); emit changed(); return {};
}
void FleetPlanner::setLayers(double minimum,double maximum,double maximumGap)
{
    if (!std::isfinite(minimum) || !std::isfinite(maximum) || !std::isfinite(maximumGap) || minimum<=0 || maximum<=minimum || maximumGap<0) {
        _error=tr("Use positive altitude limits, maximum above minimum, and a nonnegative maximum gap."); emit changed(); return;
    }
    _minimum=minimum; _maximum=maximum; _maximumGap=maximumGap; synchronize();
}
void FleetPlanner::setProfile(int drone,const QString& key,double value)
{
    if (drone<0 || drone>=_profiles.size() || !std::isfinite(value)) return;
    if ((key=="heading" && (value<0 || value>=360)) || (key=="latitude" && std::abs(value)>90) || (key=="longitude" && std::abs(value)>180)) return;
    if (!QStringList{"heading","latitude","longitude"}.contains(key)) return;
    auto profile=_profiles[drone].toMap(); profile[key]=value; _profiles[drone]=profile; synchronize();
}
void FleetPlanner::synchronize()
{
    if (_updating || !enabled()) return;
    _syncTimer.stop(); QScopedValueRollback<bool> guard(_updating,true);
    _error.clear(); const auto polygons=regions(_error);
    if (!_error.isEmpty()) { emit changed(); return; }
    const double homeAltitude=_master->missionController()->plannedHomePosition().altitude();
    if (!std::isfinite(homeAltitude)) { _error=tr("Set a finite master launch elevation."); emit changed(); return; }
    for (int i=0;i<_children.size();++i) {
        auto profile=_profiles[i].toMap(); const double altitude=_minimum+i*separation(); profile["altitude"]=altitude; _profiles[i]=profile;
        auto* mission=_children[i]->missionController(); auto* items=mission->visualItems();
        auto* settings=qobject_cast<MissionSettingsItem*>(items->get(0));
        QGeoCoordinate launch(profile["latitude"].toDouble(),profile["longitude"].toDouble(),homeAltitude);
        settings->setCoordinate(launch);
        settings->plannedHomePositionAltitude()->setRawValue(homeAltitude);
        for (auto* block : blocks(_children[i])) {
            if (block->fleetRegion()==-2) {
                block->setCoordinate(fiducial()->coordinate()); block->setValue("delay",fiducial()->values()["delay"].toDouble());
            } else if (block->fleetRegion()>=0 && block->fleetRegion()<polygons.size()) {
                if (block->surveyAreaPolygon()->coordinateList()!=polygons[block->fleetRegion()]) block->surveyAreaPolygon()->setPath(polygons[block->fleetRegion()]);
            }
            if (block->fleetRegion()!=-1 || block->kind()=="survey" || block->kind()=="vantage") { block->setValue("altitude",altitude); block->setValue("yaw",profile["heading"].toDouble()); }
        }
        for (int j=1;j<items->count();++j) if (auto* simple=qobject_cast<SimpleMissionItem*>(items->get(j))) {
            const auto command=simple->missionItem().command();
            if (command==MAV_CMD_NAV_TAKEOFF) simple->setCoordinate(launch);
            if (command==MAV_CMD_NAV_TAKEOFF || command==MAV_CMD_NAV_WAYPOINT) {
                simple->setAltitudeMode(QGroundControlQmlGlobal::AltitudeModeRelative);
                simple->applyNewAltitude(altitude);
            }
        }
    }
    _master->setDirty(true); watch(); emit changed();
}
void FleetPlanner::watch()
{
    QList<PlanMasterController*> plans=_children; if (_master) plans.append(_master);
    for (auto* plan : plans) {
        auto* items=plan->missionController()->visualItems();
        if (!items->property("fleetWatched").toBool()) {
            items->setProperty("fleetWatched",true);
            connect(items,&QmlObjectListModel::countChanged,this,[this,plan,items] {
                if (_updating) return;
                if (plan==_master && items->count()==1 && enabled()) { clear(); return; }
                _previewTimer.start(); _syncTimer.start();
            });
        }
        for (int i=1;i<items->count();++i) {
            auto* item=qobject_cast<VisualMissionItem*>(items->get(i));
            if (item->property("fleetWatched").toBool()) continue;
            item->setProperty("fleetWatched",true);
            auto update=[this,plan] { if (_updating) return; if (_master && enabled()) _master->setDirty(true); if (!_syncTimer.isActive()) _syncTimer.start(); if (!_previewTimer.isActive()) _previewTimer.start(); };
            if (auto* simple=qobject_cast<SimpleMissionItem*>(item); simple && plan!=_master) {
                connect(simple,&VisualMissionItem::coordinateChanged,this,[this,plan,simple] {
                    if (_updating || simple->missionItem().command()!=MAV_CMD_NAV_TAKEOFF) return;
                    const int index=_children.indexOf(plan); if (index<0) return;
                    auto profile=_profiles[index].toMap();
                    profile["latitude"]=simple->coordinate().latitude(); profile["longitude"]=simple->coordinate().longitude();
                    _profiles[index]=profile;
                });
            }
            connect(item,&VisualMissionItem::coordinateChanged,this,update);
            connect(item,&VisualMissionItem::lastSequenceNumberChanged,this,update);
            connect(item,&VisualMissionItem::dirtyChanged,this,update);
            if (auto* block=qobject_cast<ChimeraMissionItem*>(item)) {
                connect(block,&ChimeraMissionItem::changed,this,update);
                if (plan==_master) connect(block,&ChimeraMissionItem::translated,this,&FleetPlanner::translated);
            }
        }
    }
}
QVariantList FleetPlanner::overlays() const
{
    QVariantList result;
    for (int i=0;i<_children.size();++i) {
        QVariantList path, moving; auto* items=_children[i]->missionController()->visualItems();
        for (int j=1;j<items->count();++j) {
            auto* visual=qobject_cast<VisualMissionItem*>(items->get(j));
            auto* block=qobject_cast<ChimeraMissionItem*>(visual);
            const int previousCount=path.size();
            if (block) path.append(block->route());
            else if (auto* simple=qobject_cast<SimpleMissionItem*>(visual); simple && simple->missionItem().command()==MAV_CMD_NAV_RETURN_TO_LAUNCH)
                path.append(QVariant::fromValue(_children[i]->missionController()->plannedHomePosition()));
            else if (visual->specifiesCoordinate() && !visual->isStandaloneCoordinate()) path.append(QVariant::fromValue(visual->coordinate()));
            for (int k=previousCount;k<path.size();++k) moving.append(block && block->fleetRegion()>=0);
        }
        result.append(QVariantMap{{"path",path},{"moving",moving},{"color",color(i)},{"drone",i+1},{"name",_profiles[i].toMap()["name"]}});
    }
    QString error;
    const auto polygons = regions(error);
    for (int i=0;i<polygons.size();++i) {
        QVariantList path; for (const auto& c : polygons[i]) path.append(QVariant::fromValue(c));
        path.append(path.first());
        double lat=0,lon=0; for (const auto& c : polygons[i]) { lat+=c.latitude(); lon+=c.longitude(); }
        result.append(QVariantMap{{"path",path},{"color","#ffffff"},{"drone",0},{"name",QString("R%1").arg(i+1)},
            {"labelCoordinate",QVariant::fromValue(QGeoCoordinate(lat/polygons[i].size(),lon/polygons[i].size()))}});
    }
    return result;
}
QString FleetPlanner::validate() const
{
    if (!_error.isEmpty()) return _error;
    if (!domain() || !fiducial()) return tr("Restore the shared master survey and fiducial.");
    if (_children.size()!=_profiles.size()) return tr("The fleet is missing a drone mission.");
    if (!std::isfinite(_minimum) || !std::isfinite(_maximum) || !std::isfinite(_maximumGap) || _minimum<=0 || _maximum<=_minimum || _maximumGap<0)
        return tr("Invalid fleet altitude limits.");
    for (int i=0;i<_children.size();++i) {
        auto* items=_children[i]->missionController()->visualItems();
        for (int j=0;j<items->count();++j) {
            auto* visual=qobject_cast<VisualMissionItem*>(items->get(j));
            if (visual->readyForSaveState()!=VisualMissionItem::ReadyForSave)
                return tr("Drone %1: complete %2 (item %3) before export.").arg(i+1).arg(visual->commandName()).arg(j);
        }
        QSet<int> covered; int markers=0;
        for (auto* b : blocks(_children[i])) {
            if (!b->error().isEmpty()) return b->error();
            if (!b->linkedSurveyError().isEmpty()) return b->linkedSurveyError();
            if (b->fleetRegion()>=0) covered.insert(b->fleetRegion());
            if (b->fleetRegion()==-2) ++markers;
        }
        if (covered.size()!=_children.size() || markers!=1) return tr("Each drone needs its shared fiducial and every assigned region.");
    }
    return {};
}
void FleetPlanner::save(PlanMasterController* controller,QJsonObject& mission)
{
    if (!isMaster(controller)) return;
    synchronize(); QJsonArray plans;
    for (auto* child : _children) plans.append(child->saveToJson().object());
    mission["chimeraFleet"]=QJsonObject{{"version",1},{"minimum",_minimum},{"maximum",_maximum},{"maximumGap",_maximumGap},
        {"profiles",QJsonArray::fromVariantList(_profiles)},{"plans",plans}};
}
void FleetPlanner::load(PlanMasterController* controller,const QJsonObject& plan)
{
    if (_children.contains(controller)) return;
    if (_master && controller!=_master) return;
    attach(controller); clear();
    const auto fleet=plan["mission"].toObject()["chimeraFleet"].toObject();
    if (fleet.isEmpty()) { watch(); return; }
    const auto profiles=fleet["profiles"].toArray().toVariantList();
    const auto plans=fleet["plans"].toArray();
    if (fleet["version"].toInt()!=1 || profiles.size()<2 || profiles.size()>8 || profiles.size()!=plans.size()) { _error=tr("Invalid fleet plan."); emit changed(); return; }
    for (const auto& value : profiles) {
        const auto p=value.toMap();
        if (!p.contains("heading") || !p.contains("latitude") || !p.contains("longitude") ||
            !std::isfinite(p["heading"].toDouble()) || p["heading"].toDouble()<0 || p["heading"].toDouble()>=360 ||
            !QGeoCoordinate(p["latitude"].toDouble(),p["longitude"].toDouble()).isValid()) {
            _error=tr("Invalid drone heading or launch coordinate."); emit changed(); return;
        }
    }
    _profiles=profiles; _minimum=fleet["minimum"].toDouble(); _maximum=fleet["maximum"].toDouble(); _maximumGap=fleet["maximumGap"].toDouble();
    if (!std::isfinite(_minimum) || !std::isfinite(_maximum) || !std::isfinite(_maximumGap) || _minimum<=0 || _maximum<=_minimum || _maximumGap<0) {
        _profiles.clear(); _error=tr("Invalid fleet altitude limits."); emit changed(); return;
    }
    QScopedValueRollback<bool> guard(_updating,true);
    for (const auto& value : plans) {
        auto* child=makeChild();
        if (!child->missionController()->load(value.toObject()["mission"].toObject(),_error)) break;
    }
    watch(); emit changed();
}
QString FleetPlanner::exportDrone(int drone,const QString& filename)
{
    synchronize(); const auto error=validate(); if (!error.isEmpty()) return error;
    if (drone<0 || drone>=_children.size()) return tr("Select a drone to export.");
    auto root=_children[drone]->saveToJson().object(); auto mission=root["mission"].toObject(); auto items=mission["items"].toArray();
    for (int i=0;i<items.size();++i) { auto item=items[i].toObject(); item.remove("fleetRegion"); items[i]=item; }
    mission["items"]=items; root["mission"]=mission;
    QString path=QUrl(filename).isLocalFile()?QUrl(filename).toLocalFile():filename;
    if (!path.endsWith(".plan",Qt::CaseInsensitive)) path+=".plan";
    QSaveFile file(path); if (!file.open(QIODevice::WriteOnly)) return file.errorString();
    const auto data=QJsonDocument(root).toJson(); if (file.write(data)!=data.size() || !file.commit()) return file.errorString();
    return {};
}
