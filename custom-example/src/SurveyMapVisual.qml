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
    property bool   _divided:       false   ///< true when showing region visuals instead of the master grid

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
        _divided = show && count >= 2
        if (count !== _controlCount) {
            _controlCount = count
            _rebuildControlVisuals(show)
        } else if (!_dragging) {
            // Count is unchanged but positions may have moved (e.g. the whole
            // survey was dragged). Nudge the existing handles to follow — but
            // never while the user is actively dragging one.
            _updateHandlePositions()
        }

        // Rebuild the per-region flight paths live so they track control-point
        // drags and parameter changes as they happen (not only after settling).
        _rebuildFlightPaths()
    }

    function _rebuildFlightPaths() {
        flightPathObjMgr.destroyObjects()
        if (!_shouldShow()) {
            return
        }
        var paths = customSurveyManager.regionFlightPaths(_missionItem)
        for (var i = 0; i < paths.length; i++) {
            if (!paths[i] || paths[i].length < 2) {
                continue
            }
            var obj = flightPathComponent.createObject(map, { "pathPoints": paths[i], "regionIndex": i })
            if (obj) {
                flightPathObjMgr.addObject(obj, map)
            }
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
        // The indicator is a MapQuickItem -> add it to the map (addObject with a
        // mapControl arg calls map.addMapItem for us).
        var indicator = handleIndicatorComponent.createObject(map, { "coordinate": coordinate, "pointIndex": pointIndex })
        if (!indicator) {
            return
        }
        controlObjMgr.addObject(indicator, map)
        _handles.push(indicator)

        // The dragger is a plain Item overlay (NOT a map item), parented to the
        // map and positioned in screen space. Track it for cleanup, but do NOT
        // pass it to addMapItem (that throws "incompatible arguments").
        var dragger = handleDragComponent.createObject(map, {
            "mapControl":       map,
            "itemIndicator":    indicator,
            "itemCoordinate":   coordinate,
            "pointIndex":       pointIndex,
            // Keep the center dragger BELOW the survey's own center-drag handle
            // (zOrderMapItems + 1) so grabbing the middle moves the whole survey
            // (control points follow); the larger center ring around it stays
            // grabbable to move just the division center.
            "z":                (pointIndex < 0 ? QGroundControl.zOrderMapItems : (QGroundControl.zOrderMapItems + 1))
        })
        if (dragger) {
            controlObjMgr.addObject(dragger)
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
    Component.onDestruction: { regionObjMgr.destroyObjects(); controlObjMgr.destroyObjects(); flightPathObjMgr.destroyObjects(); _handles = [] }

    Connections {
        target: _missionItem ? _missionItem.surveyAreaPolygon : null
        function onPathChanged() { _sync() }
    }

    Connections {
        target: _missionItem
        function onIsCurrentItemChanged() { _sync() }
        // Panel edits (grid angle, altitude, spacing, turnaround, ...) rebuild
        // the master's transects and fire this; refresh the per-region grids too,
        // live (e.g. while the angle slider is being dragged).
        function onVisualTransectPointsChanged() { _rebuildFlightPaths() }
    }

    Connections {
        target: customSurveyManager
        function onCustomSurveyChanged(item) {
            if (item === _missionItem) {
                _sync()
            }
        }
    }

    // Not divided: stock survey visuals (polygon + white transect grid + click
    // to select). `object` comes from context. Also used whenever the item is
    // not current, so click-to-select still works.
    Loader {
        active: !_root._divided
        sourceComponent: Component {
            TransectStyleMapVisuals {
                map:                _root.map
                polygonInteractive: _root.polygonInteractive
                interactive:        _root.interactive
                vehicle:            _root.vehicle
                opacity:            _root.opacity
                onClicked:          (sequenceNumber) => _root.clicked(sequenceNumber)
            }
        }
    }

    // Divided: only the editable survey-area polygon (no white transect grid) —
    // the per-region colored paths replace it.
    Loader {
        active: _root._divided
        sourceComponent: Component {
            QGCMapPolygonVisuals {
                mapControl:         _root.map
                mapPolygon:         _root._missionItem.surveyAreaPolygon
                interactive:        _root.polygonInteractive && _root._missionItem.isCurrentItem && _root.interactive
                borderWidth:        1
                borderColor:        "black"
                interiorColor:      QGroundControl.globalPalette.surveyPolygonInterior
                altColor:           QGroundControl.globalPalette.surveyPolygonTerrainCollision
                interiorOpacity:    0.5 * _root.opacity
            }
        }
    }

    QGCDynamicObjectManager { id: regionObjMgr }
    QGCDynamicObjectManager { id: controlObjMgr }
    QGCDynamicObjectManager { id: flightPathObjMgr }

    // One region's transect flight path.
    Component {
        id: flightPathComponent

        MapPolyline {
            property var pathPoints:  []
            property int regionIndex: 0

            path:       pathPoints
            line.color: ["#2F80ED", "#27AE60", "#F2994A", "#EB5757"][regionIndex % 4]
            line.width: 3
            opacity:    _root.opacity
            z:          QGroundControl.zOrderMapItems + 3
        }
    }

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
            z:              QGroundControl.zOrderMapItems + 2

            property int  pointIndex: -1
            // Stable size drives BOTH the anchor and the marker. Binding
            // anchorPoint to sourceItem.width/height instead creates a
            // MapQuickItem polish() loop (freeze/crash). The center is larger so
            // its ring stays grabbable behind the survey centroid marker
            // (z + 2, below the survey marker's z + 3).
            property real markerSize: ScreenTools.defaultFontPixelHeight * (pointIndex < 0 ? 2.0 : 1.1)

            anchorPoint.x:  markerSize / 2
            anchorPoint.y:  markerSize / 2

            sourceItem: Rectangle {
                width:          handleIndicator.markerSize
                height:         handleIndicator.markerSize
                radius:         handleIndicator.markerSize / 2
                color:          handleIndicator.pointIndex < 0 ? "#F2C94C" : "#2F80ED"
                border.color:   "white"
                border.width:   2
            }
        }
    }

    Component {
        id: handleDragComponent

        MissionItemIndicatorDrag {
            property int  pointIndex: -1
            // Only mutate manager state while a drag is genuinely in progress.
            // itemCoordinate also changes at creation (initial property) and on
            // map pan; acting on those wrongly ran the center branch (pointIndex
            // still -1 at creation) and overwrote the center with an edge point.
            property bool _dragActive: false

            onDragStart: { _dragActive = true; _root._dragging = true }
            // On release, snap the (edge) handles back onto their ray midpoints.
            onDragStop: {
                _dragActive = false
                _root._dragging = false
                _root._updateHandlePositions()
            }

            onItemCoordinateChanged: {
                if (!_dragActive) {
                    return
                }
                // Store the drag. The center moves freely; an edge handle sets
                // its ray's bearing from the center (the manager re-derives the
                // boundary cut), so regions update live. The marker follows the
                // cursor during the drag and snaps back onto its ray midpoint on
                // release (see onDragStop) — re-deriving the snapped point every
                // frame here caused a geo<->screen feedback / polish loop.
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
