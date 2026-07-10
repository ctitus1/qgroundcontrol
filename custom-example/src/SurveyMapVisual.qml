/****************************************************************************
 *
 * (c) 2009-2024 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

/*
 * Map visualization for custom surveys.
 *
 * Draws generated survey regions and keeps the map
 * synchronized with the current survey.
 */


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
    property var    _regions: []

    signal clicked(int sequenceNumber)

    function _refreshRegions() {
        customSurveyManager.regionCountForSurvey(_missionItem)

        _regions = customSurveyManager.isCustomSurvey(_missionItem) ? customSurveyManager.regionPolygons(_missionItem) : []
        _rebuildRegionVisuals()
    }

    function _rebuildRegionVisuals() {
        regionObjMgr.destroyObjects()
        if (!_missionItem || !_missionItem.isCurrentItem || _regions.length === 0) {
            return
        }

        for (var i = 0; i < _regions.length; i++) {
            var region = _regions[i]
            var obj = regionVisualComponent.createObject(map, {
                "regionPath": region.polygon,
                "regionIndex": i
            })
            if (obj) {
                regionObjMgr.addObject(obj, map)
            }
        }
    }

    Component.onCompleted: _refreshRegions()
    Component.onDestruction: regionObjMgr.destroyObjects()

    Connections {
        target: _missionItem.surveyAreaPolygon

        function onPathChanged() {
            _refreshRegions()
        }
    }

    Connections {
        target: _missionItem

        function onIsCurrentItemChanged() {
            _rebuildRegionVisuals()
        }
    }

    Connections {
        target: customSurveyManager

        function onCustomSurveyChanged(item) {
            if (item === _missionItem) {
                _refreshRegions()
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
        id: regionObjMgr
    }

    Component {
        id: regionVisualComponent

        MapPolygon {
            property var regionPath: []
            property int regionIndex: 0

            path:           regionPath
            color:          ["#332F80ED", "#3327AE60", "#33F2994A", "#33EB5757"][regionIndex % 4]
            border.color:   ["#2F80ED", "#27AE60", "#F2994A", "#EB5757"][regionIndex % 4]
            border.width:   2
            opacity:        _root.opacity
            visible:        _missionItem.isCurrentItem && customSurveyManager.isCustomSurvey(_missionItem)
            z:              QGroundControl.zOrderMapItems + 1
        }
    }
}


// ============================================================
// PATCH TODO
//
// Verify all region rendering comes from
//
//      regionPolygons(missionItem)
//
// and never from compatibility state.
//
// ============================================================

