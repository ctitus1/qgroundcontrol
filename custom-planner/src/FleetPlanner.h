#pragma once
#include <QObject>
#include <QPointer>
#include <QVariantList>
#include <QJsonObject>
#include <QTimer>
#include "PlanMasterController.h"
#include "Splitters/RegionSplitter.h"
class ChimeraMissionItem;
class ChimeraPlanner;

class FleetPlanner : public QObject
{
    Q_OBJECT
    Q_PROPERTY(bool enabled READ enabled NOTIFY controllersChanged)
    Q_PROPERTY(int selected READ selected WRITE setSelected NOTIFY controllersChanged)
    Q_PROPERTY(PlanMasterController* activeController READ activeController NOTIFY activeControllerChanged)
    Q_PROPERTY(QVariantList drones READ drones NOTIFY profilesChanged)
    Q_PROPERTY(QVariantList overlays READ overlays NOTIFY changed)
    Q_PROPERTY(int pathDisplay READ pathDisplay WRITE setPathDisplay NOTIFY changed)
    Q_PROPERTY(bool night READ night WRITE setNight NOTIFY profilesChanged)
    Q_PROPERTY(QVariantList cameraPresets READ cameraPresets NOTIFY controllersChanged)
    Q_PROPERTY(QVariantList controlPoints READ controlPoints NOTIFY changed)
    Q_PROPERTY(QString error READ error NOTIFY changed)
    Q_PROPERTY(double rapidTransitSpeed READ rapidTransitSpeed WRITE setRapidTransitSpeed NOTIFY changed)
    Q_PROPERTY(double fiducialRadius READ fiducialRadius WRITE setFiducialRadius NOTIFY changed)
    Q_PROPERTY(double minimum READ minimum NOTIFY changed)
    Q_PROPERTY(double maximum READ maximum NOTIFY changed)
    Q_PROPERTY(double maximumGap READ maximumGap NOTIFY changed)
    Q_PROPERTY(double separation READ separation NOTIFY changed)
public:
    enum PathDisplay { WholePath, FirstSectorPath, NoPaths };
    Q_ENUM(PathDisplay)
    explicit FleetPlanner(QObject* parent = nullptr);
    int pathDisplay() const { return _pathDisplay; }
    void setPathDisplay(int display);
    Q_INVOKABLE void attach(PlanMasterController* master);
    Q_INVOKABLE void requestCreate(QObject* source, int count, double minimum, double maximum, double maximumGap);
    Q_INVOKABLE QString create(QObject* source, int count, double minimum, double maximum, double maximumGap);
    Q_INVOKABLE QString setCount(int count);
    Q_INVOKABLE void setProfile(int drone, const QString& key, double value, bool deferred = false);
    Q_INVOKABLE void setLayers(double minimum, double maximum, double maximumGap);
    Q_INVOKABLE void setNight(bool night);
    bool night() const { return _night; }
    QVariantList cameraPresets() const;
    QVariantList controlPoints() const;
    Q_INVOKABLE void setPartitionPoint(int index, const QGeoCoordinate& coordinate);
    Q_INVOKABLE void resetPartition();
    Q_INVOKABLE void finishPartitionDrag() { synchronize(); }
    Q_INVOKABLE void applyDroneSettings(int drone, const QString& field = QString());
    Q_INVOKABLE void editRegion(int drone, int region);
    Q_INVOKABLE QString exportAll(const QString& filename);
    Q_INVOKABLE QString exportDrone(int drone, const QString& filename, bool standardItems = false);
    bool enabled() const { return !_children.isEmpty(); }
    bool owns(PlanMasterController* controller) const;
    bool isMaster(PlanMasterController* controller) const { return enabled() && controller == _master; }
    int selected() const { return _selected; }
    void setSelected(int selected);
    PlanMasterController* activeController() const;
    QVariantList drones() const { return _profiles; }
    QVariantList overlays() const;
    QString error() const { return _error.isEmpty() && enabled() ? validate() : _error; }
    double rapidTransitSpeed() const;
    void setRapidTransitSpeed(double speed);
    double fiducialRadius() const;
    void setFiducialRadius(double radius);
    double minimum() const { return _minimum; }
    double maximum() const { return _maximum; }
    double maximumGap() const { return _maximumGap; }
    double separation() const;
    void save(PlanMasterController* controller, QJsonObject& mission);
    void load(PlanMasterController* controller, const QJsonObject& plan);
    QString validate() const;
    void synchronize();
signals:
    void changed();
    void controllersChanged();
    void activeControllerChanged();
    void profilesChanged();
    void translated(double distance, double azimuth);
private:
    QList<QList<QGeoCoordinate>> regions(QString& error, int count = 0) const;
    ChimeraMissionItem* domain() const;
    ChimeraMissionItem* fiducial() const;
    void watch();
    void archive();
    QString rebuildFleet(int count);
    QVariantMap defaultProfile(int drone, int count) const;
    void updateCamera(QVariantMap& profile) const;
    SplitInput partitionInput(int count) const;
    void movePartition(double distance, double azimuth);
    void notify();
    double assignedAltitude(int drone) const;
    void clear(bool immediate = false);
    PlanMasterController* makeChild();
    QPointer<PlanMasterController> _master;
    QList<PlanMasterController*> _children;
    QVariantList _profiles;
    QVariantMap _masterValues;
    int _masterCamera = -1;
    QJsonObject _archive;
    QVariantList _publishedProfiles;
    QPointer<PlanMasterController> _publishedController;
    int _publishedSelection = -1;
    bool _publishedEnabled = false;
    bool _night = false;
    PathDisplay _pathDisplay = WholePath;
    QGeoCoordinate _partitionCenter, _partitionOrigin;
    QList<QGeoCoordinate> _partitionVertices, _partitionPolygon;
    mutable SplitInput _cachedInput;
    mutable QList<QList<QGeoCoordinate>> _cachedRegions;
    double _minimum = 20, _maximum = 30, _maximumGap = 0;
    int _selected = 0;
    bool _updating = false;
    QString _error;
    QTimer _syncTimer, _previewTimer;
};
