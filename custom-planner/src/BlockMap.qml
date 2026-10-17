import QtQuick
import QtLocation
import QtPositioning
import QGroundControl
import QGroundControl.Controls
import QGroundControl.FlightMap

Item {
    id: root
    property var map
    property var vehicle
    property bool interactive: true
    property var block: object
    property var objects: []
    signal clicked(int sequenceNumber)
    function clear() {
        for (var item of objects) { map.removeMapItem(item); item.destroy() }
        objects = []
    }
    function add(component, properties) {
        var item = component.createObject(map, properties)
        map.addMapItem(item)
        objects.push(item)
    }
    function refresh() {
        clear()
        add(line, { path: block.route })
        if (block.kind === "survey") add(marker, { coordinate: block.coordinate, label: block.abbreviation })
        if (block.isCurrentItem && block.kind === "survey") {
            for (var corners of (block.preview.footprints || [])) add(footprint, {path: corners})
            var path = block.preview.path || []
            for (var i = 0; i < path.length; ++i) add(marker, {coordinate: path[i], label: "", routePoint: i})
            if (block.splitCoordinate.isValid) add(marker, {coordinate: block.splitCoordinate, label: "Split", selected: true})
            for (var inspection of block.inspections) {
                for (var point of inspection.points) {
                    add(marker, {coordinate: QtPositioning.coordinate(point.latitude, point.longitude), label: point.roi ? "ROI" : "I"})
                }
            }
        }
    }
    Component.onCompleted: refresh()
    Component.onDestruction: clear()
    Connections {
        target: root.block
        function onChanged() { timer.restart() }
        function onSplitPositionChanged() { timer.restart() }
        function onIsCurrentItemChanged() { timer.restart() }
    }
    Timer { id: timer; interval: 50; onTriggered: root.refresh() }
    Loader {
        id: pointLoader
        active: root.block.kind !== "survey"
        sourceComponent: Component {
            MissionItemIndicator {
                coordinate: root.block.coordinate
                missionItem: root.block
                sequenceNumber: root.block.sequenceNumber
                onClicked: if (root.interactive) root.clicked(root.block.sequenceNumber)
                Component.onDestruction: root.map.removeMapItem(this)
            }
        }
        onLoaded: { item.parent = root.map; root.map.addMapItem(item) }
    }
    Loader {
        active: root.block.kind !== "survey" && root.block.isCurrentItem && root.interactive && pointLoader.item !== null
        sourceComponent: Component {
            MissionItemIndicatorDrag {
                mapControl: root.map
                itemIndicator: pointLoader.item
                itemCoordinate: root.block.coordinate
                onItemCoordinateChanged: root.block.coordinate = itemCoordinate
            }
        }
        onLoaded: item.parent = root.map
    }
    QGCMapPolygonVisuals {
        mapControl: root.map
        mapPolygon: root.block.surveyAreaPolygon
        interactive: root.interactive && root.block.isCurrentItem && root.block.kind === "survey" && !root.block.splitPart
        visible: root.block.kind === "survey"
        borderWidth: 2; borderColor: "white"; interiorColor: "#2080ff"; interiorOpacity: 0.15
    }
    Component { id: line; MapPolyline { z: QGroundControl.zOrderMapItems + 1; line.color: "white"; line.width: 3 } }
    Component { id: footprint; MapPolygon { z: QGroundControl.zOrderMapItems; color: "#10ffb040"; border.color: "#ffb040"; border.width: 1 } }
    Component {
        id: marker
        MapQuickItem {
            id: routeMarker
            z: QGroundControl.zOrderMapItems + 2
            property string label
            property int routePoint: -1
            property bool selected: false
            anchorPoint.x: indicator.anchorPointX
            anchorPoint.y: indicator.anchorPointY
            sourceItem: MissionItemIndexLabel {
                id: indicator
                label: routeMarker.label
                index: routeMarker.routePoint >= 0 ? routeMarker.routePoint + 1 : -1
                checked: routeMarker.selected
                highlightSelected: true
                onClicked: {
                    if (!root.interactive) return
                    root.clicked(root.block.sequenceNumber)
                    if (routeMarker.routePoint >= 0) root.block.splitPosition = routeMarker.routePoint
                }
            }
        }
    }
}
