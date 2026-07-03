/****************************************************************************
 *
 * (c) 2009-2024 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
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
Item {
    id: _root

    property var    map
    property bool   polygonInteractive: true
    property bool   interactive: true
    property var    vehicle

    property var    _missionItem: object
    property var    _quadrants: []

    signal clicked(int sequenceNumber)

    function _refreshQuadrants() {
        _quadrants = customSurveyManager.isCustomSurvey(_missionItem) ? customSurveyManager.quadrantPolygons(_missionItem) : []
        _rebuildQuadrantVisuals()
    }

    function _rebuildQuadrantVisuals() {
        quadrantObjMgr.destroyObjects()
        if (!_missionItem || !_missionItem.isCurrentItem || _quadrants.length === 0) {
            return
        }

        for (var i = 0; i < _quadrants.length; i++) {
            var quadrant = _quadrants[i]
            var obj = quadrantVisualComponent.createObject(map, {
                "quadrantPath": quadrant.polygon,
                "quadrantIndex": i
            })
            if (obj) {
                quadrantObjMgr.addObject(obj, map)
            }
        }
    }

    Component.onCompleted: _refreshQuadrants()
    Component.onDestruction: quadrantObjMgr.destroyObjects()

    Connections {
        target: _missionItem.surveyAreaPolygon

        function onPathChanged() {
            _refreshQuadrants()
        }
    }

    Connections {
        target: _missionItem

        function onIsCurrentItemChanged() {
            _rebuildQuadrantVisuals()
        }
    }

    Connections {
        target: customSurveyManager

        function onCustomSurveyChanged(item) {
            if (item === _missionItem) {
                _refreshQuadrants()
            }
        }
    }

    TransectStyleMapVisuals {
        map:                _root.map
        polygonInteractive: true
        interactive:        _root.interactive
        vehicle:            _root.vehicle
        opacity:            _root.opacity

        onClicked: (sequenceNumber) => _root.clicked(sequenceNumber)
    }

    QGCDynamicObjectManager {
        id: quadrantObjMgr
    }

    Component {
        id: quadrantVisualComponent

        MapPolygon {
            property var quadrantPath: []
            property int quadrantIndex: 0

            path:           quadrantPath
            color:          ["#332F80ED", "#3327AE60", "#33F2994A", "#33EB5757"][quadrantIndex % 4]
            border.color:   ["#2F80ED", "#27AE60", "#F2994A", "#EB5757"][quadrantIndex % 4]
            border.width:   2
            opacity:        _root.opacity
            visible:        _missionItem.isCurrentItem && customSurveyManager.isCustomSurvey(_missionItem)
            z:              QGroundControl.zOrderMapItems + 1
        }
    }
}
