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
    property bool masterDomain: chimeraFleet.enabled && chimeraFleet.selected === 0 && block.kind === "survey" && block.fleetRegion === -1
    property bool routeDetails: !chimeraFleet.enabled || chimeraFleet.pathDisplay === 0 ||
        (chimeraFleet.pathDisplay === 1 && chimeraFleet.selected > 0 &&
         block.fleetRegion === (chimeraFleet.drones[chimeraFleet.selected - 1] || {}).startRegion)
    property var pools: ({})
    property var used: ({})
    signal clicked(int sequenceNumber)
    function report(error) { if (error) mainWindow.showMessageDialog(qsTr("Survey split"), error) }
    function clear() {
        for (var key in pools) for (var item of pools[key]) { map.removeMapItem(item); item.destroy() }
        pools = ({})
    }
    function add(component, properties) {
        var key = component.toString()
        var pool = pools[key] || []
        var index = used[key] || 0
        var item = pool[index]
        if (!item) {
            item = component.createObject(map, properties)
            map.addMapItem(item)
            pool.push(item)
            pools[key] = pool
        } else {
            for (var name in properties) item[name] = properties[name]
        }
        used[key] = index + 1
    }
    function trim() {
        for (var key in pools) {
            var pool = pools[key]
            while (pool.length > (used[key] || 0)) {
                var item = pool.pop()
                map.removeMapItem(item)
                item.destroy()
            }
        }
    }
    function translate(distance, azimuth) {
        for (var key in pools) for (var item of pools[key]) {
            if (item.path !== undefined) {
                var path = []
                for (var point of item.path) path.push(point.atDistanceAndAzimuth(distance, azimuth))
                item.path = path
            } else item.coordinate = item.coordinate.atDistanceAndAzimuth(distance, azimuth)
        }
    }
    function refresh() {
        used = ({})
        if (!masterDomain && routeDetails) add(line, { path: block.route })
        if (block.kind === "survey" && (masterDomain || routeDetails)) add(marker, { coordinate: block.coordinate, label: block.abbreviation, routePoint: -1 })
        if (block.isCurrentItem && block.kind === "survey" && !masterDomain && routeDetails) {
            for (var corners of (block.preview.footprints || [])) add(footprint, {path: corners})
            var path = block.preview.path || []
            for (var i = 0; i < path.length; ++i) add(marker, {coordinate: path[i], label: "", routePoint: i})
            if (root.interactive) for (var point of block.splitHandles) add(splitHandle, {coordinate: point})
            for (var inspection of block.inspections) {
                for (var point of inspection.points) {
                    add(marker, {coordinate: QtPositioning.coordinate(point.latitude, point.longitude), label: point.roi ? "ROI" : "I", routePoint: -1})
                }
            }
        }
        trim()
    }
    Component.onCompleted: refresh()
    Component.onDestruction: clear()
    Connections {
        target: root.block
        function onChanged() { if (!timer.running) timer.start() }
        function onTranslated(distance, azimuth) { timer.stop(); root.translate(distance, azimuth) }
        function onIsCurrentItemChanged() { timer.restart() }
    }
    Connections { target: chimeraFleet; function onChanged() { if (!timer.running) timer.start() } }
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
        active: root.block.kind !== "survey" && root.block.fleetRegion === -1 && root.block.isCurrentItem && root.interactive && pointLoader.item !== null
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
    Loader {
        id: endHandle
        active: root.block.hasEndSplit && root.block.isCurrentItem && root.routeDetails
        sourceComponent: Component {
            MapQuickItem {
                coordinate: root.block.exitCoordinate
                anchorPoint.x: sourceItem.anchorPointX
                anchorPoint.y: sourceItem.anchorPointY
                z: QGroundControl.zOrderMapItems + 4
                sourceItem: MissionItemIndexLabel {
                    width: 32; height: 32
                    label: "S"; index: 0; checked: true
                    onClicked: if (root.interactive) splitMenu.popup()
                }
                Component.onDestruction: root.map.removeMapItem(this)
            }
        }
        onLoaded: { item.parent = root.map; root.map.addMapItem(item) }
    }
    Loader {
        active: endHandle.item !== null && root.interactive
        sourceComponent: Component {
            MissionItemIndicatorDrag {
                objectName: "chimeraSplitDragger"
                property var missionBlock: root.block
                property bool dragging: false
                mapControl: root.map
                itemIndicator: endHandle.item
                itemCoordinate: root.block.exitCoordinate
                z: QGroundControl.zOrderMapItems + 5
                onDragStart: dragging = true
                onItemCoordinateChanged: if (dragging && itemIndicator) itemIndicator.coordinate = itemCoordinate
                onDragStop: {
                    dragging = false
                    root.block.moveEndSplit(itemCoordinate)
                    itemCoordinate = Qt.binding(function() { return root.block.exitCoordinate })
                    if (itemIndicator) itemIndicator.coordinate = Qt.binding(function() { return root.block.exitCoordinate })
                }
                onClicked: splitMenu.popup()
            }
        }
        onLoaded: item.parent = root.map
    }
    QGCMenu {
        id: splitMenu
        QGCMenuItem { text: qsTr("Remove split point"); onTriggered: root.report(root.block.removeEndSplit()) }
    }
    Component {
        id: splitHandle
        MapQuickItem {
            id: handle
            anchorPoint.x: sourceItem.width / 2
            anchorPoint.y: sourceItem.height / 2
            z: QGroundControl.zOrderMapItems + 3
            sourceItem: SplitIndicator { objectName: "chimeraSplitButton"; property var missionBlock: root.block; onClicked: root.report(root.block.addSplitPoint(handle.coordinate)) }
        }
    }
    QGCMapPolygonVisuals {
        mapControl: root.map
        mapPolygon: root.block.surveyAreaPolygon
        interactive: root.interactive && root.block.isCurrentItem && root.block.kind === "survey" && !root.block.splitPart && root.block.fleetRegion === -1
        visible: root.block.kind === "survey"
        borderWidth: 2; borderColor: "white"; interiorColor: "#2080ff"; interiorOpacity: 0.15
    }
    Component { id: line; MapPolyline { z: QGroundControl.zOrderMapItems + 1; line.color: root.block.isCurrentItem ? "#00d8ef" : "white"; line.width: root.block.isCurrentItem ? 4 : 2 } }
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

                }
            }
        }
    }
}
