#include "FleetPlanner.h"
#include "AppMessages.h"
#include "ChimeraMissionItem.h"
#include "ChimeraPlanner.h"
#include "CenterOut.h"
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
#include <QFileInfo>
#include <QGCGeo.h>
#include <QScopedValueRollback>
#include <QUrl>
#include <QCoreApplication>
#include <algorithm>
#include <cmath>
#include <limits>

namespace {
QGeoCoordinate rayBoundaryIntersection(const QList<QGeoCoordinate>& polygon,
                                                             const QGeoCoordinate& center,
                                                             double azimuthDeg)
{
    const int n = polygon.size();
    if (n < 3 || !center.isValid()) {
        return QGeoCoordinate();
    }

    const QGeoCoordinate origin = polygon.first();
    auto toLocal = [&origin](const QGeoCoordinate& c) {
        double north, east, down;
        QGCGeo::convertGeoToNed(c, origin, north, east, down);
        return QPointF(east, north);
    };

    QList<QPointF> V;
    V.reserve(n);
    for (const QGeoCoordinate& c : polygon) {
        V.append(toLocal(c));
    }
    const QPointF C = toLocal(center);

    // Bearing: 0 = north, clockwise. In (east, north): (sin, cos).
    const double rad = azimuthDeg * M_PI / 180.0;
    const QPointF dir(std::sin(rad), std::cos(rad));

    double bestT = std::numeric_limits<double>::max();
    QPointF bestPoint;
    bool found = false;
    for (int j = 0; j < n; ++j) {
        const QPointF a = V[j];
        const QPointF ab = V[(j + 1) % n] - a;
        const double denom = (dir.x() * ab.y()) - (dir.y() * ab.x());   // cross(dir, ab)
        if (std::abs(denom) < 1e-12) {
            continue;   // parallel
        }
        const QPointF ac = a - C;
        const double t = ((ac.x() * ab.y()) - (ac.y() * ab.x())) / denom;   // distance along ray
        const double u = ((ac.x() * dir.y()) - (ac.y() * dir.x())) / denom; // position along edge
        if (t > 1e-6 && u >= -1e-6 && u <= 1.0 + 1e-6 && t < bestT) {
            bestT = t;
            bestPoint = C + (t * dir);
            found = true;
        }
    }
    if (!found) {
        return QGeoCoordinate();
    }

    QGeoCoordinate cut;
    QGCGeo::convertNedToGeo(bestPoint.y() /*north*/, bestPoint.x() /*east*/, 0.0, origin, cut);
    return cut;
}

bool loadPlan(PlanMasterController* child, const QJsonObject& plan, QString& error) {
    return child->missionController()->load(plan["mission"].toObject(),error)
        && child->geoFenceController()->load(plan["geoFence"].toObject(),error)
        && child->rallyPointController()->load(plan["rallyPoints"].toObject(),error);
}
void upgradeTrackAngle(QVariantMap& profile, const QJsonObject& plan)
{
    if (profile.contains("trackAngle")) return;
    const bool automatic=profile.value("automaticHeading",false).toBool();
    profile["automaticTrackAngle"]=automatic;
    profile["trackAngle"]=CenterOut::normalizeAngle(profile["heading"].toDouble()+(automatic ? 90. : 0.));
    if (automatic) return;
    for (const auto& value : plan["mission"].toObject()["items"].toArray()) {
        const auto item=value.toObject();
        if (item["complexItemType"]!="chimera-survey" || item["fleetRegion"].toInt(-1)<0) continue;
        profile["trackAngle"]=item["values"].toObject()["angle"].toDouble();
        break;
    }
}
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
    connect(&_previewTimer, &QTimer::timeout, this, [this] { watch(); notify(); });
}
void FleetPlanner::setPathDisplay(int display)
{
    if (display<WholePath || display>NoPaths || display==_pathDisplay) return;
    _pathDisplay=static_cast<PathDisplay>(display);
    if (_master && enabled()) _master->missionController()->setDirty(true);
    emit changed();
}
void FleetPlanner::notify()
{
    if (_publishedProfiles!=_profiles) { _publishedProfiles=_profiles; emit profilesChanged(); }
    if (_publishedController!=activeController()) {
        _publishedController=activeController();
        emit activeControllerChanged();
    }
    if (_publishedSelection!=_selected || _publishedEnabled!=enabled()) {
        _publishedSelection=_selected; _publishedEnabled=enabled();
        emit controllersChanged();
    }
    emit changed();
}
void FleetPlanner::attach(PlanMasterController* master)
{
    if (_master == master) return;
    clear(); _master = master;
    if (master) connect(master->missionController(), &MissionController::visualItemsReset, this, [this] { _previewTimer.start(); });
    watch(); notify();
}
void FleetPlanner::clear(bool immediate)
{
    _selected = 0; _syncTimer.stop(); _previewTimer.stop();
    auto children = _children; _children.clear(); _profiles.clear(); _masterValues.clear(); _masterCamera=-1; _archive = {}; _error.clear();
    _partitionCenter={}; _partitionOrigin={}; _partitionVertices.clear(); _partitionPolygon.clear(); _cachedRegions.clear(); _night=false; _pathDisplay=WholePath;
    notify();
    for (auto* child : children) { if (immediate) delete child; else child->deleteLater(); }
}
PlanMasterController* FleetPlanner::makeChild()
{
    auto* child = new PlanMasterController(this);
    child->setFlyView(false);
    child->start(true);
    _children.append(child);
    connect(child->missionController(), &MissionController::visualItemsReset, this, [this] { _previewTimer.start(); });
    return child;
}
bool FleetPlanner::owns(PlanMasterController* controller) const { return isMaster(controller) || _children.contains(controller); }
PlanMasterController* FleetPlanner::activeController() const { return _selected > 0 && _selected <= _children.size() ? _children[_selected-1] : _master.data(); }
void FleetPlanner::setSelected(int selected)
{
    if (selected == _selected || selected < 0 || selected > _children.size()) return;
    _selected=selected;
    for (auto* block : blocks(activeController())) if (block->kind()=="survey") {
        activeController()->missionController()->setCurrentPlanViewSeqNum(block->sequenceNumber(),true);
        break;
    }
    notify();
}
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
double FleetPlanner::assignedAltitude(int drone) const
{
    const auto profile=_profiles[drone].toMap();
    if (profile.value("manualAltitude",false).toBool()) return std::clamp(profile["altitude"].toDouble(),_minimum,_maximum);
    const double step=(_maximum-_minimum)/std::max(1, int(_profiles.size())-1);
    return _minimum+drone*(_maximumGap>0 ? std::min(step,_maximumGap) : step);
}
double FleetPlanner::separation() const
{
    if (_profiles.size()<2) return 0;
    QList<double> layers;
    for (int i=0;i<_profiles.size();++i) layers.append(assignedAltitude(i));
    std::sort(layers.begin(),layers.end());
    double gap=INFINITY;
    for (int i=1;i<layers.size();++i) gap=std::min(gap,layers[i]-layers[i-1]);
    return gap;
}
QList<QList<QGeoCoordinate>> FleetPlanner::regions(QString& error, int count) const
{
    if (count == 0) count = _profiles.size();
    if (count < 2) return {};
    auto* source = domain();
    if (!source || !fiducial()) { error=tr("The master needs one Chimera survey and one shared fiducial."); return {}; }
    if (source->linkedSurvey() || source->splitPart() || !source->inspections().isEmpty()) { error=tr("Add survey interruptions in the drone views, not in the master area."); return {}; }
    const auto polygon = source->surveyAreaPolygon()->coordinateList();
    if (polygon.size()<3) { error=tr("Draw the shared search area."); return {}; }
    const auto input=partitionInput(count);
    const auto center=input.center;
    if (!_cachedRegions.isEmpty() && input.masterPolygon==_cachedInput.masterPolygon && input.center==_cachedInput.center && input.edgePoints==_cachedInput.edgePoints)
        return _cachedRegions;
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
    for (const auto& point : input.edgePoints) if (!point.isValid()) { error=tr("Keep the partition center inside the survey area."); return {}; }
    const auto split=ControlPointSplitter().split(input,error);
    if (split.size()!=count) { error=tr("The area could not be divided into one region per drone."); return {}; }
    QList<QList<QGeoCoordinate>> result;
    for (const auto& region : split) result.append(region.polygon);
    _cachedInput=input; _cachedRegions=result;
    return result;
}
void FleetPlanner::requestCreate(QObject* source,int count,double minimum,double maximum,double maximumGap)
{
    QPointer<QObject> guardedSource(source);
    // Fleet creation can unload the QML editor that requested it.
    QTimer::singleShot(0,this,[this,guardedSource,count,minimum,maximum,maximumGap] {
        if (!guardedSource) return;
        const auto error=create(guardedSource.data(),count,minimum,maximum,maximumGap);
        if (!error.isEmpty()) QGC::showAppMessage(error);
    });
}
double FleetPlanner::fiducialRadius() const
{
    return fiducial() ? fiducial()->values()["radius"].toDouble() : 2.;
}
void FleetPlanner::setFiducialRadius(double radius)
{
    if (!fiducial() || !std::isfinite(radius) || radius < 0) return;
    fiducial()->setValue("radius",radius);
    synchronize();
}
double FleetPlanner::rapidTransitSpeed() const
{
    return domain() ? domain()->values()["transitSpeed"].toDouble() : 5.;
}
void FleetPlanner::setRapidTransitSpeed(double speed)
{
    if (!domain() || !std::isfinite(speed) || speed <= 0) return;
    domain()->setValue("transitSpeed",speed);
    synchronize();
}
QString FleetPlanner::create(QObject* object,int count,double minimum,double maximum,double maximumGap)
{
    auto* source=qobject_cast<ChimeraMissionItem*>(object);
    if (!source || source->kind()!="survey" || count<2 || count>8) return tr("Select a survey and use 2 to 8 drones.");
    if (enabled()) return setCount(count);
    attach(source->masterController());
    if (!_master->controllerVehicle()->multiRotor()) return tr("Fleet planning currently supports multicopters.");
    if (domain()!=source || !fiducial()) return tr("The master needs one survey and one shared fiducial.");
    if (!source->error().isEmpty()) return source->error();
    if (!fiducial()->error().isEmpty()) return fiducial()->error();
    if (!std::isfinite(minimum) || !std::isfinite(maximum) || !std::isfinite(maximumGap) || minimum<=0 || maximum<=minimum || maximumGap<0)
        return tr("Use positive altitude limits, maximum above minimum, and a nonnegative maximum gap.");
    _minimum=minimum; _maximum=maximum; _maximumGap=maximumGap;
    _masterValues=source->values(); _masterCamera=source->cameraPreset();
    auto home=_master->missionController()->plannedHomePosition();
    if (!home.isValid() || !std::isfinite(home.altitude())) return tr("Set the master launch location and elevation before creating drone missions.");
    for (int i=0;i<count;++i) _profiles.append(defaultProfile(i,count));
    QString error; const auto polygons=regions(error);
    if (!error.isEmpty()) { _profiles.clear(); return error; }
    QScopedValueRollback<bool> guard(_updating,true);
    const auto searchCenter=partitionInput(count).center;
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
            survey->load(saved.first().toObject(),1,error); survey->setSearchCenter(searchCenter); survey->setFleetRegion(region);
            survey->surveyAreaPolygon()->setPath(polygons[region]);
        }
        auto* rtl=qobject_cast<SimpleMissionItem*>(mission->insertSimpleMissionItem(home,3+count));
        rtl->missionItem().setCommand(MAV_CMD_NAV_RETURN_TO_LAUNCH);
    }
    for (int i=0;i<count;++i) applyDroneSettings(i);
    _syncTimer.start(); watch(); _master->missionController()->setDirty(true); notify(); return {};
}
void FleetPlanner::archive()
{
    for (int i=0; i<_children.size(); ++i) {
        auto entry=_archive[QString::number(i)].toObject();
        const auto plan=_children[i]->saveToJson().object();
        auto regions=entry["regions"].toObject();
        QJsonObject current;
        QJsonArray prefix, suffix;
        int region=-1;
        bool tail=false;
        for (const auto& value : plan["mission"].toObject()["items"].toArray()) {
            const auto item=value.toObject();
            if (item["fleetRegion"].toInt(-1)>=0) region=item["fleetRegion"].toInt();
            if (item["command"].toInt()==MAV_CMD_NAV_RETURN_TO_LAUNCH) tail=true;
            if (tail) suffix.append(value);
            else if (region<0) prefix.append(value);
            else {
                const auto key=QString::number(region);
                auto bundle=current[key].toArray(); bundle.append(value); current[key]=bundle;
            }
        }
        // Empty active regions record deliberate deletions, rather than restoring older edits.
        for (int r=0; r<_children.size(); ++r) regions[QString::number(r)]=current[QString::number(r)].toArray();
        entry["regions"]=regions; entry["prefix"]=prefix; entry["suffix"]=suffix;
        entry["profile"]=QJsonObject::fromVariantMap(_profiles[i].toMap()); entry["plan"]=plan;
        _archive[QString::number(i)]=entry;
    }
}
QString FleetPlanner::setCount(int count)
{
    if (!enabled() || count<2 || count>8) return tr("Use 2 to 8 drones.");
    if (count==_children.size()) return {};
    return rebuildFleet(count);
}
QString FleetPlanner::rebuildFleet(int count)
{
    synchronize();
    if (!_error.isEmpty()) return _error;
    archive();
    QScopedValueRollback<bool> guard(_updating,true);
    const auto home=_master->missionController()->plannedHomePosition();
    QVariantList profiles;
    for (int i=0; i<count; ++i) {
        auto profile=_archive[QString::number(i)].toObject()["profile"].toObject().toVariantMap();
        if (profile.isEmpty()) profile=defaultProfile(i,count);
        upgradeTrackAngle(profile,_archive[QString::number(i)].toObject()["plan"].toObject());
        profiles.append(profile);
    }
    for (const auto& key : {QString("startRegion"),QString("uas")}) {
        QSet<int> used;
        const int first=key=="uas"?1:0, limit=key=="uas"?8:count-1;
        for (int i=0;i<profiles.size();++i) {
            auto profile=profiles[i].toMap(); int value=profile.value(key,i+first).toInt();
            if (value<first || value>limit || used.contains(value)) value=-1; else used.insert(value);
            profile[key]=value; profiles[i]=profile;
        }
        for (int i=0;i<profiles.size();++i) {
            auto profile=profiles[i].toMap();
            if (profile[key].toInt()>=0) continue;
            int value=first; while (used.contains(value)) ++value;
            used.insert(value); profile[key]=value;
            if (key=="uas") { profile["name"]=QString("UAS%1").arg(value); updateCamera(profile); }
            profiles[i]=profile;
        }
    }
    QString error; const auto polygons=regions(error,count);
    if (!error.isEmpty()) return error;
    QJsonArray source; domain()->save(source);
    QList<PlanMasterController*> replacement;
    for (int i=0; i<count; ++i) {
        auto entry=_archive[QString::number(i)].toObject();
        if (entry.isEmpty()) {
            entry=_archive["0"].toObject();
            entry["regions"]=QJsonObject();
            QJsonArray prefix;
            for (const auto& value : entry["prefix"].toArray()) {
                const auto item=value.toObject();
                if (item["command"].toInt()==MAV_CMD_NAV_TAKEOFF || item["fleetRegion"].toInt(-1)==-2) prefix.append(value);
            }
            entry["prefix"]=prefix;
            entry["suffix"]=QJsonArray{QJsonObject{{"type","SimpleItem"},{"command",MAV_CMD_NAV_RETURN_TO_LAUNCH},
                {"frame",MAV_FRAME_MISSION},{"autoContinue",true},{"params",QJsonArray{0,0,0,0,0,0,0}},{"doJumpId",0}}};
        }
        auto plan=entry["plan"].toObject(); auto mission=plan["mission"].toObject();
        auto items=entry["prefix"].toArray(); const auto stored=entry["regions"].toObject();
        for (int phase=0; phase<count; ++phase) {
            const int region=(profiles[i].toMap().value("startRegion",i).toInt()+phase)%count; const auto key=QString::number(region);
            if (stored.contains(key)) {
                for (const auto& item : stored[key].toArray()) items.append(item);
            } else {
                auto item=source.first().toObject(); item["fleetRegion"]=region; item["centerOut"]=true;
                auto values=item["values"].toObject(); const auto profile=profiles[i].toMap();
                for (const auto& field : {"speed","pitch","overlap","width","height","fx","fy","cx","cy"}) if (profile.contains(field)) values[field]=profile[field].toDouble();
                item["values"]=values;
                const int preset=profile.value("cameraPreset",0).toInt();
                item["cameraPreset"]=cameraPresets().value(preset).toMap()["id"].toString();
                items.append(item);
            }
        }
        for (const auto& item : entry["suffix"].toArray()) items.append(item);
        mission["items"]=items; plan["mission"]=mission;
        auto* child=new PlanMasterController(this); child->setFlyView(false); child->start(true);
        replacement.append(child);
        if (!loadPlan(child,plan,error)) break;
    }
    if (!error.isEmpty()) {
        qDeleteAll(replacement); return error;
    }
    const auto previous=_children;
    _children=replacement; _profiles=profiles;
    if (_partitionVertices.size()!=count) _partitionVertices.clear();
    _selected=std::min(_selected,count);
    for (auto* child : _children) connect(child->missionController(),&MissionController::visualItemsReset,this,[this] { _previewTimer.start(); });
    _updating=false;
    synchronize();
    for (auto* child : previous) child->deleteLater();
    return {};
}
void FleetPlanner::setLayers(double minimum,double maximum,double maximumGap)
{
    if (!std::isfinite(minimum) || !std::isfinite(maximum) || !std::isfinite(maximumGap) || minimum<=0 || maximum<=minimum || maximumGap<0) {
        _error=tr("Use positive altitude limits, maximum above minimum, and a nonnegative maximum gap."); notify(); return;
    }
    if (_minimum==minimum && _maximum==maximum && _maximumGap==maximumGap && _error.isEmpty()) return;
    _minimum=minimum; _maximum=maximum; _maximumGap=maximumGap;
    for (int i=0;i<_profiles.size();++i) { auto profile=_profiles[i].toMap(); profile["manualAltitude"]=false; _profiles[i]=profile; }
    synchronize();
}
void FleetPlanner::setProfile(int drone,const QString& key,double value,bool deferred)
{
    if (drone<0 || drone>=_profiles.size() || !std::isfinite(value)) return;
    const QMap<QString,QPair<double,double>> limits{{"heading",{0,359.99}},{"trackAngle",{0,359.99}},{"latitude",{-90,90}},{"longitude",{-180,180}},
        {"uas",{1,8}},{"startRegion",{0,double(_profiles.size()-1)}},{"cameraPreset",{-1,8}},
        {"altitude",{_minimum,_maximum}},{"speed",{0.01,100}},{"pitch",{-90,-0.01}},{"overlap",{0,94.99}},
        {"width",{1,100000}},{"height",{1,100000}},{"fx",{0.1,100000}},{"fy",{0.1,100000}},{"cx",{0,100000}},{"cy",{0,100000}}};
    if (!limits.contains(key) || value<limits[key].first || value>limits[key].second) return;
    auto profile=_profiles[drone].toMap();
    if (profile.value(key).toDouble()==value && key!="cameraPreset") return;
    const auto previous=_profiles;
    if (key=="uas" || key=="startRegion") {
        if (value!=std::floor(value)) return;
        for (int i=0;i<_profiles.size();++i) if (i!=drone && _profiles[i].toMap().value(key,i+(key=="uas"?1:0)).toInt()==int(value)) {
            auto other=_profiles[i].toMap(); other[key]=profile.value(key,drone+(key=="uas"?1:0));
            if (key=="uas") { other["name"]=QString("UAS%1").arg(other[key].toInt()); updateCamera(other); }
            _profiles[i]=other;
            if (key=="uas") applyDroneSettings(i,"uas");
        }
    }
    profile[key]=value;
    if (key=="heading") profile["automaticHeading"]=false;
    if (key=="trackAngle") profile["automaticTrackAngle"]=false;
    if (key=="altitude") profile["manualAltitude"]=true;
    if (key=="uas") profile["name"]=QString("UAS%1").arg(int(value));
    if (key=="cameraPreset") profile["cameraAutomatic"]=value<0;
    if (QStringList{"width","height","fx","fy","cx","cy"}.contains(key)) { profile["cameraAutomatic"]=false; profile["cameraPreset"]=0; }
    if (key=="cameraPreset" || key=="uas") updateCamera(profile);
    _profiles[drone]=profile;
    if (key=="startRegion") {
        const auto error=rebuildFleet(_children.size());
        if (!error.isEmpty()) { _profiles=previous; _error=error; notify(); }
        return;
    }
    if (QStringList{"uas","cameraPreset","speed","pitch","overlap","width","height","fx","fy","cx","cy"}.contains(key)) applyDroneSettings(drone,key);
    if (deferred) { if (!_syncTimer.isActive()) _syncTimer.start(); notify(); }
    else synchronize();
}
void FleetPlanner::synchronize()
{
    if (_updating || !enabled()) return;
    _syncTimer.stop(); QScopedValueRollback<bool> guard(_updating,true);
    if (auto* source=domain()) {
        const auto values=source->values();
        const double rotation=values["yaw"].toDouble()-_masterValues.value("yaw",values["yaw"]).toDouble();
        const double trackRotation=values["angle"].toDouble()-_masterValues.value("angle",values["angle"]).toDouble();
        bool cameraChanged=_masterCamera!=source->cameraPreset();
        for (const auto& key : {"width","height","fx","fy","cx","cy"})
            cameraChanged |= values[key]!=_masterValues.value(key,values[key]);
        for (int i=0;i<_profiles.size();++i) {
            auto profile=_profiles[i].toMap();
            profile["heading"]=CenterOut::normalizeAngle(profile["heading"].toDouble()+rotation);
            profile["trackAngle"]=CenterOut::normalizeAngle(profile["trackAngle"].toDouble()+trackRotation);
            for (const auto& key : {"speed","pitch","overlap","width","height","fx","fy","cx","cy"})
                if (values[key]!=_masterValues.value(key,values[key])) profile[key]=values[key];
            if (cameraChanged) {
                profile["cameraPreset"]=source->cameraPreset(); profile["cameraAutomatic"]=false;
                for (const auto& key : {"width","height","fx","fy","cx","cy"}) profile[key]=values[key];
            }
            const auto previous=_profiles[i].toMap();
            _profiles[i]=profile;
            if (cameraChanged) applyDroneSettings(i,"cameraPreset");
            for (const auto& key : {"speed","pitch","overlap","width","height","fx","fy","cx","cy"})
                if (profile[key]!=previous[key]) applyDroneSettings(i,key);
        }
        _masterValues=values; _masterCamera=source->cameraPreset();
    }

    if (auto* source=domain()) {
        const auto center=source->centerCoordinate(); const auto polygon=source->surveyAreaPolygon()->coordinateList();
        if (_partitionOrigin.isValid() && center.distanceTo(_partitionOrigin)>0.001 && _partitionPolygon.size()==polygon.size()) {
            const double distance=_partitionOrigin.distanceTo(center), azimuth=_partitionOrigin.azimuthTo(center);
            bool rigid=true;
            for (int i=0;i<polygon.size();++i) if (_partitionPolygon[i].atDistanceAndAzimuth(distance,azimuth).distanceTo(polygon[i])>0.05) rigid=false;
            if (rigid) movePartition(distance,azimuth);
        }
        _partitionOrigin=center; _partitionPolygon=polygon;
    }
    _error.clear(); const auto polygons=regions(_error);
    if (!_error.isEmpty()) { notify(); return; }
    const double homeAltitude=_master->missionController()->plannedHomePosition().altitude();
    if (!std::isfinite(homeAltitude)) { _error=tr("Set a finite master launch elevation."); notify(); return; }
    const auto searchCenter=partitionInput(_children.size()).center;
    for (int i=0;i<_children.size();++i) {
        auto profile=_profiles[i].toMap();
        const int startRegion=profile.value("startRegion",i).toInt();
        if (startRegion<0 || startRegion>=polygons.size()) {
            _error=tr("Assign each drone a valid starting region."); notify(); return;
        }
        const double inward=CenterOut::inwardHeading(polygons[startRegion],searchCenter);
        if (profile.value("automaticHeading",false).toBool())
            profile["heading"]=CenterOut::normalizeAngle(inward+domain()->values()["yaw"].toDouble());
        if (profile.value("automaticTrackAngle",false).toBool())
            profile["trackAngle"]=CenterOut::normalizeAngle(inward+domain()->values()["angle"].toDouble());
        const double altitude=assignedAltitude(i); profile["altitude"]=altitude; _profiles[i]=profile;
        auto* mission=_children[i]->missionController(); auto* items=mission->visualItems();
        auto* settings=qobject_cast<MissionSettingsItem*>(items->get(0));
        QGeoCoordinate launch(profile["latitude"].toDouble(),profile["longitude"].toDouble(),homeAltitude);
        settings->setCoordinate(launch);
        settings->plannedHomePositionAltitude()->setRawValue(homeAltitude);
        for (auto* block : blocks(_children[i])) {
            if (block->fleetRegion()==-2) {
                block->setCoordinate(fiducial()->coordinate().atDistanceAndAzimuth(fiducialRadius(),profile["heading"].toDouble()+180));
                block->setValue("delay",fiducial()->values()["delay"].toDouble());
                block->setValue("speed",profile["speed"].toDouble());
            } else if (block->fleetRegion()>=0 && block->fleetRegion()<polygons.size()) {
                block->setSearchCenter(searchCenter);
                if (block->surveyAreaPolygon()->coordinateList()!=polygons[block->fleetRegion()]) block->surveyAreaPolygon()->setPath(polygons[block->fleetRegion()]);
            }
            if (block->kind()=="survey") block->setValue("angle",profile["trackAngle"].toDouble());
            if (block->kind()=="survey" || block->kind()=="vantage") block->setValue("transitSpeed",rapidTransitSpeed());
            if (block->fleetRegion()!=-1 || block->kind()=="survey" || block->kind()=="vantage") { block->setValue("altitude",altitude); block->setValue("yaw",profile["heading"].toDouble()); }
        }
        for (int j=1;j<items->count();++j) if (auto* simple=qobject_cast<SimpleMissionItem*>(items->get(j))) {
            const auto command=simple->missionItem().command();
            if (command==MAV_CMD_NAV_TAKEOFF) simple->setCoordinate(launch);
            if (command==MAV_CMD_NAV_TAKEOFF || command==MAV_CMD_NAV_WAYPOINT) {
                simple->setAltitudeFrame(QGroundControlQmlGlobal::AltitudeFrameRelative);
                simple->applyNewAltitude(altitude);
            }
        }
    }
    _master->missionController()->setDirty(true); watch(); notify();
}
void FleetPlanner::watch()
{
    QList<PlanMasterController*> plans=_children; if (_master) plans.append(_master);
    for (auto* plan : plans) {
        auto* items=plan->missionController()->visualItems();
        if (!items->property("fleetWatched").toBool()) {
            items->setProperty("fleetWatched",true);
            connect(items,&QmlObjectListModel::countChanged,this,[this,plan,items] {
                if (_updating || (plan!=_master && !_children.contains(plan))) return;
                if (plan==_master && items->count()==1 && enabled()) { clear(); return; }
                _previewTimer.start(); _syncTimer.start();
            });
        }
        for (int i=1;i<items->count();++i) {
            auto* item=qobject_cast<VisualMissionItem*>(items->get(i));
            if (item->property("fleetWatched").toBool()) continue;
            item->setProperty("fleetWatched",true);
            auto update=[this,plan] { if (_updating || (plan!=_master && !_children.contains(plan))) return; if (_master && enabled()) _master->missionController()->setDirty(true); if (!_syncTimer.isActive()) _syncTimer.start(); if (!_previewTimer.isActive()) _previewTimer.start(); };
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
                connect(block,&ChimeraMissionItem::changed,this,[this,plan,block] {
                    if (_updating || plan==_master || block->fleetRegion()<0) return;
                    const int index=_children.indexOf(plan); if (index<0) return;
                    auto profile=_profiles[index].toMap();
                    const double heading=block->values()["yaw"].toDouble(), altitude=block->values()["altitude"].toDouble();
                    if (profile["heading"].toDouble()!=heading) { profile["heading"]=heading; profile["automaticHeading"]=false; }
                    const double trackAngle=block->values()["angle"].toDouble();
                    if (profile["trackAngle"].toDouble()!=trackAngle) { profile["trackAngle"]=trackAngle; profile["automaticTrackAngle"]=false; }
                    if (profile["altitude"].toDouble()!=altitude) { profile["altitude"]=altitude; profile["manualAltitude"]=true; }
                    _profiles[index]=profile;
                });
                connect(block,&ChimeraMissionItem::changed,this,update);
                if (plan==_master) connect(block,&ChimeraMissionItem::translated,this,[this](double distance,double azimuth) {
                    movePartition(distance,azimuth); emit translated(distance,azimuth);
                });
            }
        }
    }
}
QVariantList FleetPlanner::overlays() const
{
    QVariantList result;
    for (int i=0;i<_children.size();++i) {
        QVariantList path, moving; auto* items=_children[i]->missionController()->visualItems();
        int firstRegion=-1;
        for (int j=1;_pathDisplay!=NoPaths && j<items->count();++j) {
            auto* visual=qobject_cast<VisualMissionItem*>(items->get(j));
            auto* block=qobject_cast<ChimeraMissionItem*>(visual);
            if (block && block->fleetRegion()>=0) {
                if (firstRegion<0) firstRegion=block->fleetRegion();
                else if (_pathDisplay==FirstSectorPath && block->fleetRegion()!=firstRegion) break;
            }
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
        int owner=0;
        for (int d=0;d<_profiles.size();++d) if (_profiles[d].toMap().value("startRegion",d).toInt()==i) owner=d;
        result.append(QVariantMap{{"path",path},{"color",color(owner)},{"drone",0},{"name",QString("R%1 / %2").arg(i+1).arg(_profiles[owner].toMap()["name"].toString())},
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
    if (separation()<0.1) return tr("Separate drone altitude layers by at least 0.1 m.");
    if (_maximumGap>0) {
        QList<double> layers;
        for (int i=0;i<_profiles.size();++i) layers.append(assignedAltitude(i));
        std::sort(layers.begin(),layers.end());
        for (int i=1;i<layers.size();++i) if (layers[i]-layers[i-1]>_maximumGap+1e-6)
            return tr("Reduce the altitude gap or increase the maximum gap.");
    }
    QSet<int> aircraft, starts;
    for (int i=0;i<_profiles.size();++i) {
        const auto profile=_profiles[i].toMap(); const int uas=profile.value("uas",i+1).toInt(), region=profile.value("startRegion",i).toInt();
        if (uas<1 || uas>8 || aircraft.contains(uas) || region<0 || region>=_profiles.size() || starts.contains(region))
            return tr("Assign each aircraft and starting region once.");
        aircraft.insert(uas); starts.insert(region);
    }
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
    synchronize(); QJsonArray plans, partition;
    for (const auto& point : controlPoints()) {
        const auto c=point.value<QGeoCoordinate>(); partition.append(QJsonArray{c.latitude(),c.longitude()});
    }
    archive();
    for (auto* child : _children) plans.append(child->saveToJson().object());
    mission["chimeraFleet"]=QJsonObject{{"version",1},{"minimum",_minimum},{"maximum",_maximum},{"maximumGap",_maximumGap},
        {"profiles",QJsonArray::fromVariantList(_profiles)},{"plans",plans},{"archive",_archive},{"night",_night},
        {"partition",partition},{"pathDisplay",_pathDisplay}};
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
    if (fleet["version"].toInt()!=1 || profiles.size()<2 || profiles.size()>8 || profiles.size()!=plans.size()) { _error=tr("Invalid fleet plan."); notify(); return; }
    for (const auto& value : profiles) {
        const auto p=value.toMap();
        if (!p.contains("heading") || !p.contains("latitude") || !p.contains("longitude") ||
            !std::isfinite(p["heading"].toDouble()) || p["heading"].toDouble()<0 || p["heading"].toDouble()>=360 ||
            !QGeoCoordinate(p["latitude"].toDouble(),p["longitude"].toDouble()).isValid()) {
            _error=tr("Invalid drone heading or launch coordinate."); notify(); return;
        }
    }
    if (!domain() || !fiducial()) { _error=tr("Restore the shared master survey and fiducial."); notify(); return; }
    _night=fleet["night"].toBool();
    _pathDisplay=static_cast<PathDisplay>(std::clamp(fleet["pathDisplay"].toInt(),int(WholePath),int(NoPaths)));
    const auto controls=fleet["partition"].toArray();
    for (int i=0;i<controls.size();++i) {
        const auto p=controls[i].toArray();
        if (p.size()!=2) { _error=tr("Invalid partition point."); notify(); return; }
        const QGeoCoordinate c(p[0].toDouble(),p[1].toDouble());
        if (!c.isValid()) { _error=tr("Invalid partition coordinate."); notify(); return; }
        if (i==0) _partitionCenter=c; else _partitionVertices.append(c);
    }
    if (domain()) { _partitionOrigin=domain()->centerCoordinate(); _partitionPolygon=domain()->surveyAreaPolygon()->coordinateList(); }
    _archive=fleet["archive"].toObject();
    _profiles=profiles;
    const bool legacyAngles=std::all_of(_profiles.cbegin(),_profiles.cend(),[](const QVariant& profile) {
        return !profile.toMap().contains("trackAngle");
    });
    if (legacyAngles && domain()->values()["angle"]==domain()->values()["yaw"]) {
        QScopedValueRollback<bool> guard(_updating,true);
        domain()->setValue("angle",CenterOut::normalizeAngle(domain()->values()["yaw"].toDouble()+90.));
    }
    _masterValues=domain()->values(); _masterCamera=domain()->cameraPreset();
    if (auto* source=domain()) for (int i=0; i<_profiles.size(); ++i) {
        auto profile=_profiles[i].toMap();
        if (!profile.contains("automaticHeading")) {
            const double expected=std::fmod(source->values()["yaw"].toDouble()+360.*i/_profiles.size(),360.);
            profile["automaticHeading"]=std::abs(profile["heading"].toDouble()-expected)<1e-6;
            _profiles[i]=profile;
        }
    }
    _minimum=fleet["minimum"].toDouble(); _maximum=fleet["maximum"].toDouble(); _maximumGap=fleet["maximumGap"].toDouble();
    if (!std::isfinite(_minimum) || !std::isfinite(_maximum) || !std::isfinite(_maximumGap) || _minimum<=0 || _maximum<=_minimum || _maximumGap<0) {
        _profiles.clear(); _error=tr("Invalid fleet altitude limits."); notify(); return;
    }
    QScopedValueRollback<bool> guard(_updating,true);
    for (const auto& value : plans) {
        auto* child=makeChild();
        if (!loadPlan(child,value.toObject(),_error)) break;
    }
    for (int i=0;i<_children.size();++i) {
        auto profile=_profiles[i].toMap(); auto defaults=defaultProfile(i,_children.size());
        upgradeTrackAngle(profile,plans[i].toObject());
        if (!profile.contains("cameraPreset")) for (auto* block : blocks(_children[i])) if (block->fleetRegion()>=0) {
            defaults["startRegion"]=block->fleetRegion(); defaults["cameraPreset"]=block->cameraPreset(); defaults["cameraAutomatic"]=false;
            for (const auto& key : {"speed","pitch","overlap","width","height","fx","fy","cx","cy"}) defaults[key]=block->values()[key];
            break;
        }
        for (auto it=defaults.begin();it!=defaults.end();++it) if (!profile.contains(it.key())) profile[it.key()]=it.value();
        profile["name"]=QString("UAS%1").arg(profile["uas"].toInt());
        _profiles[i]=profile;
    }
    const auto searchCenter=partitionInput(_children.size()).center;
    for (auto* child : _children) for (auto* block : blocks(child))
        if (block->fleetRegion()>=0) block->setSearchCenter(searchCenter);
    _syncTimer.start(); watch(); notify();
}
QString FleetPlanner::exportDrone(int drone,const QString& filename,bool standardItems)
{
    synchronize(); const auto error=validate(); if (!error.isEmpty()) return error;
    if (drone<0 || drone>=_children.size()) return tr("Select a drone to export.");
    if (standardItems) return ChimeraPlanner().exportPlan(_children[drone],filename);
    auto root=_children[drone]->saveToJson().object(); auto mission=root["mission"].toObject(); auto items=mission["items"].toArray();
    for (int i=0;i<items.size();++i) { auto item=items[i].toObject(); item.remove("fleetRegion"); items[i]=item; }
    mission["items"]=items; root["mission"]=mission;
    QString path=QUrl(filename).isLocalFile()?QUrl(filename).toLocalFile():filename;
    if (!path.endsWith(".plan",Qt::CaseInsensitive)) path+=".plan";
    QSaveFile file(path); if (!file.open(QIODevice::WriteOnly)) return file.errorString();
    const auto data=QJsonDocument(root).toJson(); if (file.write(data)!=data.size() || !file.commit()) return file.errorString();
    return {};
}

QVariantList FleetPlanner::cameraPresets() const
{
    return domain() ? domain()->cameraPresets() : QVariantList{};
}
void FleetPlanner::updateCamera(QVariantMap& profile) const
{
    if (profile.value("cameraAutomatic",true).toBool()) {
        const int uas=profile["uas"].toInt();
        profile["cameraPreset"]=uas<=4 ? 2*uas-(_night?0:1) : 0;
    }
    const auto preset=cameraPresets().value(profile["cameraPreset"].toInt()).toMap();
    if (preset.value("id").toString().isEmpty()) return;
    const auto matrix=preset["camera_matrix"].toMap()["data"].toList();
    profile["width"]=preset["image_width"]; profile["height"]=preset["image_height"];
    profile["fx"]=matrix[0]; profile["fy"]=matrix[4]; profile["cx"]=matrix[2]; profile["cy"]=matrix[5];
}
QVariantMap FleetPlanner::defaultProfile(int drone, int count) const
{
    auto* source=domain();
    const auto home=_master->missionController()->plannedHomePosition();
    QString error;
    const auto polygons=regions(error,count);
    const double inward=drone<polygons.size() ? CenterOut::inwardHeading(polygons[drone],partitionInput(count).center) : 0.;
    QVariantMap profile{{"name",QString("UAS%1").arg(drone+1)},{"uas",drone+1},{"startRegion",drone},
        {"color",color(drone)},{"latitude",home.latitude()},{"longitude",home.longitude()},
        {"automaticHeading",true},{"heading",CenterOut::normalizeAngle(inward+source->values()["yaw"].toDouble())},
        {"automaticTrackAngle",true},{"trackAngle",CenterOut::normalizeAngle(inward+source->values()["angle"].toDouble())},
        {"cameraAutomatic",true}};
    for (const auto& key : {"speed","pitch","overlap","width","height","fx","fy","cx","cy"}) profile[key]=source->values()[key];
    updateCamera(profile);
    return profile;
}
void FleetPlanner::applyDroneSettings(int drone, const QString& field)
{
    if (drone<0 || drone>=_children.size()) return;
    QScopedValueRollback<bool> guard(_updating,true);
    const auto profile=_profiles[drone].toMap(); QSet<int> seen;
    for (auto* block : blocks(_children[drone])) {
        if (block->fleetRegion()<0 || seen.contains(block->fleetRegion())) continue;
        seen.insert(block->fleetRegion());
        const bool camera=field.isEmpty() || QStringList{"uas","cameraPreset","width","height","fx","fy","cx","cy"}.contains(field);
        if (camera) block->setCameraPreset(profile["cameraPreset"].toInt());
        for (const auto& key : {"speed","pitch","overlap","width","height","fx","fy","cx","cy"})
            if (profile.contains(key) && (field.isEmpty() || field==key || (camera && QStringList{"width","height","fx","fy","cx","cy"}.contains(key)))) block->setValue(key,profile[key].toDouble());
    }
    if (_master) _master->missionController()->setDirty(true);
    _previewTimer.start();
}
void FleetPlanner::setNight(bool night)
{
    if (_night==night) return;
    _night=night;
    for (int i=0;i<_profiles.size();++i) {
        auto p=_profiles[i].toMap();
        if (!p.value("cameraAutomatic",true).toBool()) continue;
        updateCamera(p); _profiles[i]=p; applyDroneSettings(i,"cameraPreset");
    }
    emit profilesChanged(); notify();
}
SplitInput FleetPlanner::partitionInput(int count) const
{
    SplitInput input;
    if (!domain()) return input;
    input.masterPolygon=domain()->surveyAreaPolygon()->coordinateList();
    input.center=_partitionCenter.isValid() ? _partitionCenter : domain()->centerCoordinate();
    for (int i=0;i<count;++i) {
        const double angle=_partitionVertices.size()==count ? input.center.azimuthTo(_partitionVertices[i]) : 360.*i/count;
        input.edgePoints.append(rayBoundaryIntersection(input.masterPolygon,input.center,angle));
    }
    return input;
}
QVariantList FleetPlanner::controlPoints() const
{
    if (!enabled()) return {};
    const auto input=partitionInput(_children.size());
    QVariantList points{QVariant::fromValue(input.center)};
    for (const auto& edge : input.edgePoints) points.append(QVariant::fromValue(input.center.atDistanceAndAzimuth(input.center.distanceTo(edge)/2,input.center.azimuthTo(edge))));
    return points;
}
void FleetPlanner::setPartitionPoint(int index,const QGeoCoordinate& coordinate)
{
    if (!enabled() || !coordinate.isValid() || index<0 || index>_children.size()) return;
    const auto points=controlPoints();
    const auto oldCenter=_partitionCenter; const auto oldVertices=_partitionVertices;
    if (!_partitionCenter.isValid()) _partitionCenter=points[0].value<QGeoCoordinate>();
    if (_partitionVertices.size()!=_children.size()) {
        _partitionVertices.clear();
        for (int i=1;i<points.size();++i) _partitionVertices.append(points[i].value<QGeoCoordinate>());
    }
    if (index==0 && !domain()->surveyAreaPolygon()->containsCoordinate(coordinate)) return;
    if (index==0) _partitionCenter=coordinate; else _partitionVertices[index-1]=coordinate;
    QString error; regions(error);
    if (!error.isEmpty()) { _partitionCenter=oldCenter; _partitionVertices=oldVertices; return; }
    _partitionOrigin=domain()->centerCoordinate();
    _master->missionController()->setDirty(true); _syncTimer.start(150); notify();
}
void FleetPlanner::resetPartition()
{
    _partitionCenter={}; _partitionVertices.clear(); _partitionOrigin={}; synchronize();
}
void FleetPlanner::movePartition(double distance,double azimuth)
{
    if (_partitionCenter.isValid()) _partitionCenter=_partitionCenter.atDistanceAndAzimuth(distance,azimuth);
    for (auto& point : _partitionVertices) point=point.atDistanceAndAzimuth(distance,azimuth);
    if (domain()) { _partitionOrigin=domain()->centerCoordinate(); _partitionPolygon=domain()->surveyAreaPolygon()->coordinateList(); }
}
void FleetPlanner::editRegion(int drone,int region)
{
    if (drone<0 || drone>=_children.size()) return;
    setSelected(drone+1);
    for (auto* block : blocks(_children[drone])) if (block->fleetRegion()==region) {
        _children[drone]->missionController()->setCurrentPlanViewSeqNum(block->sequenceNumber(),true); return;
    }
}
QString FleetPlanner::exportAll(const QString& filename)
{
    synchronize(); const auto error=validate(); if (!error.isEmpty()) return error;
    QString base=QUrl(filename).isLocalFile()?QUrl(filename).toLocalFile():filename;
    if (base.endsWith(".plan",Qt::CaseInsensitive)) base.chop(5);
    if (QFileInfo(base).fileName().isEmpty()) return tr("Enter a mission name.");
    for (int i=0;i<_children.size();++i) for (bool standardItems : {false,true}) {
        const auto path=QString("%1-uas%2%3.plan").arg(base).arg(_profiles[i].toMap().value("uas",i+1).toInt()).arg(standardItems ? "-plain" : "");
        const auto result=exportDrone(i,path,standardItems); if (!result.isEmpty()) return tr("Could not export %1: %2").arg(path,result);
    }
    auto master=_master->saveToJson().object(); auto mission=master["mission"].toObject();
    save(_master,mission); master["mission"]=mission;
    QSaveFile file(base+".plan"); if (!file.open(QIODevice::WriteOnly)) return file.errorString();
    const auto data=QJsonDocument(master).toJson(); if (file.write(data)!=data.size() || !file.commit()) return file.errorString();
    return {};
}
