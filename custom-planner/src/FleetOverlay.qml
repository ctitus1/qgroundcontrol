import QtQuick
import QtLocation
import QtPositioning
import QGroundControl.Controls

Item {
    id: root
    property var map: null
    property var lines: []
    property var markers: []
    function refresh() {
        if (!map) return
        const paths = chimeraFleet.overlays
        while (lines.length > paths.length) { const item = lines.pop(); map.removeMapItem(item); item.destroy() }
        while (markers.length > paths.length) { const item = markers.pop(); map.removeMapItem(item); item.destroy() }
        for (var i = 0; i < paths.length; ++i) {
            if (i >= lines.length) {
                const item = line.createObject(map); map.addMapItem(item); lines.push(item)
                const label = marker.createObject(map); map.addMapItem(label); markers.push(label)
            }
            markers[i].coordinate = paths[i].labelCoordinate || (paths[i].path.length ? paths[i].path[0] : QtPositioning.coordinate())
            markers[i].title = paths[i].name
            markers[i].tint = paths[i].color
            markers[i].drone = paths[i].drone
            lines[i].path = paths[i].path
            lines[i].moving = paths[i].moving || []
            lines[i].region = paths[i].drone === 0
            lines[i].line.color = paths[i].color
            lines[i].line.width = chimeraFleet.selected === 0 ? 3 : 2
            lines[i].opacity = chimeraFleet.selected === 0 ? 0.9 : 0.3
            lines[i].visible = paths[i].drone === 0 || chimeraFleet.selected !== paths[i].drone
        }
    }
    onMapChanged: refresh()
    Connections {
        target: chimeraFleet
        function onChanged() { root.refresh() }
        function onTranslated(distance, azimuth) {
            for (const item of lines) {
                var shifted = []
                for (var i = 0; i < item.path.length; ++i) shifted.push(item.region || item.moving[i] ? item.path[i].atDistanceAndAzimuth(distance, azimuth) : item.path[i])
                item.path = shifted
            }
            for (const item of markers) if (item.drone === 0) item.coordinate = item.coordinate.atDistanceAndAzimuth(distance, azimuth)
        }
    }
    Component.onDestruction: { if (map) for (const item of lines.concat(markers)) { map.removeMapItem(item); item.destroy() } }
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
            sourceItem: MissionItemIndexLabel { label: indicator.title; index: -1; small: false; color: indicator.tint; onClicked: chimeraFleet.selected = indicator.drone }
        }
    }
    Component { id: line; MapPolyline { property var moving: []; property bool region: false; z: 5 } }
}
