import QtQuick
import QtLocation
import QtPositioning
import QGroundControl
import QGroundControl.FlightMap
import QGroundControl.Controls
import QGroundControl.ScreenTools

Item {
    id: root
    property var map: null
    property var lines: []
    property var markers: []
    property var fills: []
    property var handles: []
    property var draggers: []
    property bool dragging: false
    function refreshControls() {
        const points = chimeraFleet.selected === 0 ? chimeraFleet.controlPoints : []
        while (handles.length > points.length) {
            const h = handles.pop(); map.removeMapItem(h); h.destroy(); draggers.pop().destroy()
        }
        for (var i = 0; i < points.length; ++i) {
            if (i >= handles.length) {
                const h = handle.createObject(map, {pointIndex: i, coordinate: points[i]})
                map.addMapItem(h); handles.push(h)
                draggers.push(dragger.createObject(map, {pointIndex: i, mapControl: map, itemIndicator: h, itemCoordinate: points[i], z: QGroundControl.zOrderMapItems + (i === 0 ? 0 : 1)}))
            } else if (!dragging) {
                handles[i].coordinate = points[i]; draggers[i].itemCoordinate = points[i]
            }
        }
    }
    function refresh() {
        if (!map) return
        const paths = chimeraFleet.overlays
        while (lines.length > paths.length) { const item = lines.pop(); map.removeMapItem(item); item.destroy() }
        while (fills.length > paths.length) { const item = fills.pop(); map.removeMapItem(item); item.destroy() }
        while (markers.length > paths.length) { const item = markers.pop(); map.removeMapItem(item); item.destroy() }
        for (var i = 0; i < paths.length; ++i) {
            if (i >= lines.length) {
                const item = line.createObject(map); map.addMapItem(item); lines.push(item)
                const fill = sector.createObject(map); map.addMapItem(fill); fills.push(fill)
                const label = marker.createObject(map); map.addMapItem(label); markers.push(label)
            }
            markers[i].coordinate = paths[i].labelCoordinate || (paths[i].path.length ? paths[i].path[0] : QtPositioning.coordinate())
            markers[i].title = paths[i].name
            markers[i].tint = paths[i].color
            markers[i].drone = paths[i].drone
            markers[i].visible = (chimeraFleet.selected === 0 || chimeraPlanner.showContext) && (paths[i].drone === 0 || chimeraFleet.showRoutes)
            fills[i].visible = paths[i].drone === 0 && (chimeraFleet.selected === 0 || chimeraPlanner.showContext)
            fills[i].path = paths[i].drone === 0 ? paths[i].path : []
            fills[i].color = paths[i].color
            lines[i].path = paths[i].path
            lines[i].moving = paths[i].moving || []
            lines[i].region = paths[i].drone === 0
            lines[i].line.color = paths[i].color
            lines[i].line.width = chimeraFleet.selected === 0 ? 3 : 2
            lines[i].opacity = chimeraFleet.selected === 0 ? 0.9 : 0.3
            lines[i].visible = (chimeraFleet.selected === 0 || chimeraPlanner.showContext) && (paths[i].drone === 0 || (chimeraFleet.showRoutes && chimeraFleet.selected !== paths[i].drone))
        }
        refreshControls()
    }
    Connections { target: chimeraPlanner; function onDisplayChanged() { refreshTimer.restart() } }
    onMapChanged: refresh()
    Connections {
        target: chimeraFleet
        function onChanged() { if (!refreshTimer.running) refreshTimer.start() }
        function onTranslated(distance, azimuth) {
            for (const item of lines) {
                var shifted = []
                for (var i = 0; i < item.path.length; ++i) shifted.push(item.region || item.moving[i] ? item.path[i].atDistanceAndAzimuth(distance, azimuth) : item.path[i])
                item.path = shifted
            }
            for (const item of fills) if (item.visible) item.path = item.path.map(p => p.atDistanceAndAzimuth(distance, azimuth))
            for (var n = 0; n < handles.length; ++n) {
                handles[n].coordinate = handles[n].coordinate.atDistanceAndAzimuth(distance, azimuth)
                draggers[n].itemCoordinate = handles[n].coordinate
            }
            for (const item of markers) if (item.drone === 0) item.coordinate = item.coordinate.atDistanceAndAzimuth(distance, azimuth)
        }
    }
    Timer { id: refreshTimer; interval: 50; onTriggered: root.refresh() }
    Component.onDestruction: {
        if (map) for (const item of lines.concat(markers, fills, handles)) { map.removeMapItem(item); item.destroy() }
        for (const item of draggers) item.destroy()
    }
    Component {
        id: sector
        MapPolygon { z: 2; border.width: 0; opacity: 0.16 }
    }
    Component {
        id: handle
        MapQuickItem {
            id: control
            property int pointIndex
            property real markerSize: ScreenTools.defaultFontPixelHeight * (pointIndex === 0 ? 2.5 : 1.1)
            z: QGroundControl.zOrderMapItems + 2
            anchorPoint.x: markerSize / 2
            anchorPoint.y: markerSize / 2
            sourceItem: Rectangle {
                width: control.markerSize; height: control.markerSize; radius: width / 2
                color: control.pointIndex === 0 ? "#F2C94C" : "#2F80ED"
                border.color: "white"; border.width: 2
            }
        }
    }
    Component {
        id: dragger
        MissionItemIndicatorDrag {
            property int pointIndex
            property bool dragging: false
            onDragStart: { dragging = true; root.dragging = true }
            onDragStop: { dragging = false; root.dragging = false; chimeraFleet.finishPartitionDrag(); root.refreshControls() }
            onItemCoordinateChanged: {
                if (!dragging) return
                chimeraFleet.setPartitionPoint(pointIndex, itemCoordinate)
                if (itemIndicator) itemIndicator.coordinate = itemCoordinate
            }
        }
    }
    Component {
        id: marker
        MapQuickItem {
            id: indicator
            property string title
            property int drone
            property color tint
            anchorPoint.x: sourceItem.anchorPointX
            anchorPoint.y: sourceItem.anchorPointY
            z: 6
            sourceItem: MissionItemIndexLabel { label: indicator.drone === 0 ? "" : indicator.title; index: indicator.drone === 0 ? parseInt(indicator.title.substring(1)) : -1; small: false; color: indicator.tint; onClicked: chimeraFleet.selected = indicator.drone }
        }
    }
    Component { id: line; MapPolyline { property var moving: []; property bool region: false; z: 5 } }
}
