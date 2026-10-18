#pragma once
#include <QObject>
#include <QPointer>
#include <QVariantList>
#include <QJsonObject>
#include <QTimer>
#include "PlanMasterController.h"
class ChimeraMissionItem;
class ChimeraPlanner;

class FleetPlanner : public QObject
{
    Q_OBJECT
    Q_PROPERTY(bool enabled READ enabled NOTIFY controllersChanged)
    Q_PROPERTY(int selected READ selected WRITE setSelected NOTIFY controllersChanged)
    Q_PROPERTY(PlanMasterController* activeController READ activeController NOTIFY controllersChanged)
    Q_PROPERTY(QVariantList drones READ drones NOTIFY profilesChanged)
    Q_PROPERTY(QVariantList overlays READ overlays NOTIFY changed)
    Q_PROPERTY(QString error READ error NOTIFY changed)
    Q_PROPERTY(double minimum READ minimum NOTIFY changed)
    Q_PROPERTY(double maximum READ maximum NOTIFY changed)
    Q_PROPERTY(double maximumGap READ maximumGap NOTIFY changed)
    Q_PROPERTY(double separation READ separation NOTIFY changed)
public:
    explicit FleetPlanner(QObject* parent = nullptr);
    Q_INVOKABLE void attach(PlanMasterController* master);
    Q_INVOKABLE QString create(QObject* source, int count, double minimum, double maximum, double maximumGap);
    Q_INVOKABLE QString setCount(int count);
    Q_INVOKABLE void setProfile(int drone, const QString& key, double value);
    Q_INVOKABLE void setLayers(double minimum, double maximum, double maximumGap);
    Q_INVOKABLE QString exportDrone(int drone, const QString& filename);
    bool enabled() const { return !_children.isEmpty(); }
    bool owns(PlanMasterController* controller) const;
    bool isMaster(PlanMasterController* controller) const { return enabled() && controller == _master; }
    int selected() const { return _selected; }
    void setSelected(int selected);
    PlanMasterController* activeController() const;
    QVariantList drones() const { return _profiles; }
    QVariantList overlays() const;
    QString error() const { return _error; }
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
    void profilesChanged();
    void translated(double distance, double azimuth);
private:
    QList<QList<QGeoCoordinate>> regions(QString& error, int count = 0) const;
    ChimeraMissionItem* domain() const;
    ChimeraMissionItem* fiducial() const;
    void watch();
    void archive();
    void notify();
    void clear(bool immediate = false);
    PlanMasterController* makeChild();
    QPointer<PlanMasterController> _master;
    QList<PlanMasterController*> _children;
    QVariantList _profiles;
    QJsonObject _archive;
    QVariantList _publishedProfiles;
    QPointer<PlanMasterController> _publishedController;
    int _publishedSelection = -1;
    bool _publishedEnabled = false;
    double _minimum = 30, _maximum = 60, _maximumGap = 0;
    int _selected = 0;
    bool _updating = false;
    QString _error;
    QTimer _syncTimer, _previewTimer;
};
