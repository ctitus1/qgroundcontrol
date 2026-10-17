/****************************************************************************
 *
 * (c) 2009-2020 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

import QtQuick
import QtQuick.Controls
import QtLocation
import QtPositioning

import QGroundControl
import QGroundControl.ScreenTools
import QGroundControl.Palette
import QGroundControl.Controls
import QGroundControl.FlightMap

/// Survey Complex Mission Item visuals
TransectStyleMapVisuals {
    id: surveyVisual
    polygonInteractive: true
    property bool plannerEnabled: false
    _transectPoints: plannerEnabled ? [] : _missionItem.visualTransectPoints
    Connections {
        target: typeof chimeraPlanner !== "undefined" ? chimeraPlanner : null
        function onSettingsChanged(item) {
            if (item === object) surveyVisual.plannerEnabled = chimeraPlanner.settings(object).enabled
        }
    }
    Loader {
        active: typeof chimeraPlanner !== "undefined"
        source: active ? "qrc:/Planner/SurveyOverlay.qml" : ""
        onLoaded: {
            item.map = surveyVisual.map
            item.survey = object
            surveyVisual.plannerEnabled = chimeraPlanner.settings(object).enabled
        }
    }
}
