import QtQuick
import QtQuick.Controls
import QtLocation
import QtPositioning

import QGroundControl
import QGroundControl.Controls
import QGroundControl.FlightMap

/// Survey Complex Mission Item visuals
TransectStyleMapVisuals {
    id: _root
    polygonInteractive: true
    Loader {
        active: typeof chimeraPlanner !== "undefined"
        source: "qrc:/Planner/SurveyOverlay.qml"
        onLoaded: { item.map = _root.map; item.survey = _root._missionItem }
    }
}
