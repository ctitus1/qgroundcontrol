#pragma once

#include "ComplexMissionItem.h"
#include <QVariantMap>
#include <QJsonArray>

class ChimeraPlanner;
class SurveyComplexItem;
class QGCMapPolygon;
class Fact;

class ChimeraMissionItem : public ComplexMissionItem
{
    Q_OBJECT
    Q_MOC_INCLUDE("QGCMapPolygon.h")
    Q_MOC_INCLUDE("Fact.h")
    Q_PROPERTY(QVariantList cameraPresets READ cameraPresets CONSTANT)
    Q_PROPERTY(int cameraPreset READ cameraPreset NOTIFY changed)
    Q_PROPERTY(int fleetRegion READ fleetRegion NOTIFY changed)
    Q_PROPERTY(bool centerOut READ centerOut NOTIFY changed)
    Q_PROPERTY(bool hasEndSplit READ hasEndSplit NOTIFY changed)
    Q_PROPERTY(bool linkedSurvey READ linkedSurvey NOTIFY changed)
    Q_PROPERTY(QVariantList splitHandles READ splitHandles NOTIFY changed)
    Q_PROPERTY(bool splitPart READ splitPart NOTIFY changed)
    Q_PROPERTY(QString kind READ kind CONSTANT)
    Q_PROPERTY(QVariantMap values READ values NOTIFY changed)
    Q_PROPERTY(QVariantList route READ route NOTIFY changed)
    Q_PROPERTY(QVariantList inspections READ inspections NOTIFY changed)
    Q_PROPERTY(QVariantMap preview READ preview NOTIFY changed)
    Q_PROPERTY(QString error READ error NOTIFY changed)
    Q_PROPERTY(QGCMapPolygon* surveyAreaPolygon READ surveyAreaPolygon CONSTANT)
    Q_PROPERTY(QGeoCoordinate centerCoordinate READ centerCoordinate WRITE setCoordinate NOTIFY changed)
public:
    ChimeraMissionItem(PlanMasterController* master, bool flyView, const QString& kind, ChimeraPlanner* planner);
    int fleetRegion() const { return _fleetRegion; }
    QGeoCoordinate searchCenter() const { return _searchCenter; }
    void setSearchCenter(const QGeoCoordinate& center);
    bool centerOut() const { return _centerOut; }
    void setFleetRegion(int region) { _fleetRegion = region; _centerOut = region >= 0; rebuild(); }
    QVariantList cameraPresets() const;
    int cameraPreset() const;
    Q_INVOKABLE void setCameraPreset(int index);
    Q_INVOKABLE Fact* fact(const QString& key) const { return _facts.value(key); }
    bool hasEndSplit() const { return !_groupId.isEmpty() && _end < 1; }
    bool linkedSurvey() const { return !_groupId.isEmpty(); }
    QVariantList splitHandles() const;
    Q_INVOKABLE void rotateEntryPoint();
    Q_INVOKABLE QString addSplitPoint(const QGeoCoordinate& coordinate);
    Q_INVOKABLE void moveEndSplit(const QGeoCoordinate& coordinate);
    Q_INVOKABLE QString removeEndSplit();
    Q_INVOKABLE QString upgradeFrozenSurvey();
    QString linkedSurveyError() const;
    bool splitPart() const { return !_frozen.isEmpty(); }
    Q_INVOKABLE void clearInspections() { _inspections.clear(); rebuild(); }
    QString kind() const { return _kind; }
    QVariantMap values() const { return _values; }
    QVariantList route() const { return _route; }
    QVariantList inspections() const { return _inspections; }
    QVariantMap preview() const { return _preview; }
    QString error() const { return _error; }
    QGCMapPolygon* surveyAreaPolygon() const;
    QGeoCoordinate centerCoordinate() const;
    Q_INVOKABLE void setValue(const QString& key, double value);
    Q_INVOKABLE void addInspection(int checkpoint);
    Q_INVOKABLE void removeInspection(int index);
    Q_INVOKABLE void addInspectionPoint(int index, bool roi);
    Q_INVOKABLE void removeInspectionPoint(int index, int point);
    Q_INVOKABLE void setInspectionPoint(int index, int point, const QString& key, double value);
    bool dirty() const override { return _dirty; }
    bool isSimpleItem() const override { return false; }
    bool isSurveyItem() const override { return _kind == "survey"; }
    bool isStandaloneCoordinate() const override { return false; }
    bool specifiesCoordinate() const override { return true; }
    bool specifiesAltitudeOnly() const override { return false; }
    QString patternName() const override { return "Chimera " + _kind; }
    QString commandName() const override { return patternName(); }
    QString commandDescription() const override { return patternName(); }
    QString abbreviation() const override { return _kind == "survey" ? "S" : _kind == "fiducial" ? "F" : "V"; }
    QGeoCoordinate coordinate() const override;
    QGeoCoordinate exitCoordinate() const override;
    double amslEntryAlt() const override;
    double amslExitAlt() const override { return amslEntryAlt(); }
    double minAMSLAltitude() const override;
    double maxAMSLAltitude() const override;
    double complexDistance() const override;
    double greatestDistanceTo(const QGeoCoordinate& other) const override;
    int sequenceNumber() const override { return _sequence; }
    int lastSequenceNumber() const override { return _sequence + qMax(1, _commands.size()) - 1; }
    double specifiedFlightSpeed() override { return _values.value("speed").toDouble(); }
    double specifiedGimbalYaw() override { return isSurveyItem() ? 0 : qQNaN(); }
    double specifiedGimbalPitch() override { return isSurveyItem() ? _values.value("pitch").toDouble() : qQNaN(); }
    double specifiedVehicleYaw() override { return _values.value("yaw").toDouble(); }
    bool exitCoordinateSameAsEntry() const override { return _kind != "survey"; }
    void setDirty(bool dirty) override;
    void setCoordinate(const QGeoCoordinate& coordinate) override;
    void setSequenceNumber(int sequence) override;
    ReadyForSaveState readyForSaveState() const override { return _error.isEmpty() ? ReadyForSave : NotReadyForSaveData; }
    void save(QJsonArray& items) override;
    bool load(const QJsonObject& json, int sequence, QString& error) override;
    QString mapVisualQML() const override { return "qrc:/Planner/BlockMap.qml"; }
    void appendMissionItems(QList<MissionItem*>& items, QObject* parent) override;
    void applyNewAltitude(double altitude) override { setValue("altitude", altitude); }
    double additionalTimeDelay() const override { return _kind == "survey" ? 0 : _values.value("delay").toDouble(); }
signals:
    void changed();
    void translated(double distance, double azimuth);
private:
    void rebuild();
    void syncSurvey();
    QString splitAt(double fraction);
    QList<ChimeraMissionItem*> linkedParts() const;
    ChimeraMissionItem* nextPart() const;
    void moveInspectionPoints(const QGeoCoordinate& from, const QGeoCoordinate& to);
    double homeAltitude() const;
    int _fleetRegion = -1;
    bool _centerOut = false;
    QGeoCoordinate _searchCenter;
    QString _kind;
    ChimeraPlanner* _planner;
    SurveyComplexItem* _survey;
    SurveyComplexItem* _flightSurvey;
    QVariantMap _generationValues;
    QVariantMap _values;
    QVariantMap _preview;
    QVariantList _route;
    QVariantList _inspections;
    QList<MissionItem*> _commands;
    QMap<QString, Fact*> _facts;
    QString _groupId;
    QString _cameraPreset;
    QString _syncedCameraPreset;
    QVariantMap _syncedValues;
    QList<QGeoCoordinate> _syncedPolygon;
    double _start = 0, _end = 1;
    QJsonArray _frozen;
    QJsonArray _generated;
    bool _resume = false;
    QString _error;
    QGeoCoordinate _position;
    QGeoCoordinate _lastCenter;
    int _sequence = 0;
    bool _updating = false;
};
