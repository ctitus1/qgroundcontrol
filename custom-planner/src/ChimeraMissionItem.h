#pragma once

#include "ComplexMissionItem.h"
#include <QVariantMap>

class ChimeraPlanner;
class SurveyComplexItem;
class QGCMapPolygon;

class ChimeraMissionItem : public ComplexMissionItem
{
    Q_OBJECT
    Q_MOC_INCLUDE("QGCMapPolygon.h")
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
    double specifiedGimbalYaw() override { return 0; }
    double specifiedGimbalPitch() override { return _values.value("pitch").toDouble(); }
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
private:
    void rebuild();
    void moveInspectionPoints(const QGeoCoordinate& from, const QGeoCoordinate& to);
    double homeAltitude() const;
    QString _kind;
    ChimeraPlanner* _planner;
    SurveyComplexItem* _survey;
    QVariantMap _values;
    QVariantMap _preview;
    QVariantList _route;
    QVariantList _inspections;
    QList<MissionItem*> _commands;
    QString _error;
    QGeoCoordinate _position;
    QGeoCoordinate _lastCenter;
    int _sequence = 0;
    bool _updating = false;
};
