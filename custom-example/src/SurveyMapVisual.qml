/****************************************************************************
 *
 * (c) 2009-2024 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

/*
 * CUSTOM PLUGIN OVERRIDE of src/QmlControls/SurveyMapVisual.qml.
 *
 * Core is simply `TransectStyleMapVisuals { polygonInteractive: true }`. This
 * wrapper keeps those stock visuals (polygon tracing + vertex drag handles +
 * transects) and, for surveys flagged as custom, additionally draws:
 *   - colored sub-region overlays computed live from the control points
 *   - draggable control-point handles (one center + one per region)
 *
 * A plain (non-custom) survey renders exactly like core. `object` (the mission
 * item) is inherited from the QML context set up by MissionItemMapVisual, so
 * the inner TransectStyleMapVisuals must NOT have `object` set explicitly.
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

Item {
    id: _root

    property var    map
    property bool   polygonInteractive: true
    property bool   interactive:        true
    property var    vehicle

    property var    _missionItem:   object
    property var    _regions:       []
    property int    _controlCount:  0
    property var    _handles:       []      ///< live control-point indicator objects
    property bool   _dragging:      false   ///< true while a control point is being dragged

    signal clicked(int sequenceNumber)

    function _shouldShow() {
        return _missionItem && customSurveyManager.isCustomSurvey(_missionItem) && _missionItem.isCurrentItem
    }

    // Recompute regions + (re)build handles when the count changes. Region
    // overlays refresh every call (cheap, keeps the preview live); handles are
    // only rebuilt when their count changes so an in-progress drag is never
    // destroyed out from under the user.
    function _sync() {
        var show = _shouldShow()
        _regions = show ? customSurveyManager.regionPolygons(_missionItem) : []
        _rebuildRegionVisuals(show)

        var count = show ? customSurveyManager.edgeControlPoints(_missionItem).length : 0
        if (count !== _controlCount) {
            _controlCount = count
            _rebuildControlVisuals(show)
        } else if (!_dragging) {
            // Count is unchanged but positions may have moved (e.g. the whole
            // survey was dragged). Nudge the existing handles to follow — but
            // never while the user is actively dragging one.
            _updateHandlePositions()
        }
    }

    function _rebuildRegionVisuals(show) {
        regionObjMgr.destroyObjects()
        if (!show || _regions.length === 0) {
            return
        }
        for (var i = 0; i < _regions.length; i++) {
            var obj = regionComponent.createObject(map, { "regionPath": _regions[i].polygon, "regionIndex": i })
            if (obj) {
                regionObjMgr.addObject(obj, map)
            }
        }
    }

    function _rebuildControlVisuals(show) {
        controlObjMgr.destroyObjects()
        _handles = []
        if (!show || _controlCount < 2) {
            return
        }
        _createHandle(-1, customSurveyManager.centerControlPoint(_missionItem))
        var edges = customSurveyManager.edgeControlPoints(_missionItem)
        for (var i = 0; i < edges.length; i++) {
            _createHandle(i, edges[i])
        }
    }

    function _createHandle(pointIndex, coordinate) {
        var indicator = handleIndicatorComponent.createObject(map, { "coordinate": coordinate, "pointIndex": pointIndex })
        if (!indicator) {
            return
        }
        map.addMapItem(indicator)
        controlObjMgr.addObject(indicator, map)
        _handles.push(indicator)

        var dragger = handleDragComponent.createObject(map, {
            "mapControl":       map,
            "itemIndicator":    indicator,
            "itemCoordinate":   coordinate,
            "pointIndex":       pointIndex
        })
        if (dragger) {
            controlObjMgr.addObject(dragger, map)
        }
    }

    // Move existing handle markers to the manager's current control-point
    // positions without recreating them (keeps a move smooth, no flicker).
    function _updateHandlePositions() {
        if (_handles.length === 0) {
            return
        }
        var center = customSurveyManager.centerControlPoint(_missionItem)
        var edges  = customSurveyManager.edgeControlPoints(_missionItem)
        for (var i = 0; i < _handles.length; i++) {
            var handle = _handles[i]
            if (!handle) {
                continue
            }
            if (handle.pointIndex < 0) {
                handle.coordinate = center
            } else if (handle.pointIndex < edges.length) {
                handle.coordinate = edges[handle.pointIndex]
            }
        }
    }

    Component.onCompleted:   _sync()
    Component.onDestruction: { regionObjMgr.destroyObjects(); controlObjMgr.destroyObjects(); _handles = [] }

    Connections {
        target: _missionItem ? _missionItem.surveyAreaPolygon : null
        function onPathChanged() { _sync() }
    }

    Connections {
        target: _missionItem
        function onIsCurrentItemChanged() { _sync() }
    }

    Connections {
        target: customSurveyManager
        function onCustomSurveyChanged(item) {
            if (item === _missionItem) {
                _sync()
            }
        }
    }

    // Stock survey visuals (polygon + transects). `object` comes from context.
    TransectStyleMapVisuals {
        map:                _root.map
        polygonInteractive: _root.polygonInteractive
        interactive:        _root.interactive
        vehicle:            _root.vehicle
        opacity:            _root.opacity

        onClicked: (sequenceNumber) => _root.clicked(sequenceNumber)
    }

    QGCDynamicObjectManager { id: regionObjMgr }
    QGCDynamicObjectManager { id: controlObjMgr }

    // Colored, semi-transparent fill for each sub-region.
    Component {
        id: regionComponent

        MapPolygon {
            property var regionPath:  []
            property int regionIndex: 0

            path:           regionPath
            color:          ["#332F80ED", "#3327AE60", "#33F2994A", "#33EB5757"][regionIndex % 4]
            border.color:   ["#2F80ED", "#27AE60", "#F2994A", "#EB5757"][regionIndex % 4]
            border.width:   2
            opacity:        _root.opacity
            z:              QGroundControl.zOrderMapItems + 1
        }
    }

    // Draggable marker for a control point. pointIndex == -1 is the center.
    Component {
        id: handleIndicatorComponent

        MapQuickItem {
            id:             handleIndicator
            anchorPoint.x:  sourceItem.width  / 2
            anchorPoint.y:  sourceItem.height / 2
            z:              QGroundControl.zOrderMapItems + 2

            property int pointIndex: -1

            sourceItem: Rectangle {
                width:          ScreenTools.defaultFontPixelHeight * (handleIndicator.pointIndex < 0 ? 1.5 : 1.1)
                height:         width
                radius:         width / 2
                color:          handleIndicator.pointIndex < 0 ? "#F2C94C" : "#2F80ED"
                border.color:   "white"
                border.width:   2
            }
        }
    }

    Component {
        id: handleDragComponent

        MissionItemIndicatorDrag {
            property int pointIndex: -1

            onDragStart: _root._dragging = true
            onDragStop:  _root._dragging = false

            onItemCoordinateChanged: {
                if (pointIndex < 0) {
                    customSurveyManager.setCenterControlPoint(_root._missionItem, itemCoordinate)
                } else {
                    customSurveyManager.setEdgeControlPoint(_root._missionItem, pointIndex, itemCoordinate)
                }
                if (itemIndicator) {
                    itemIndicator.coordinate = itemCoordinate
                }
            }
        }
    }
}
