pragma ComponentBehavior: Bound

import QtQuick
import QtQml.Models
import QtLocation
import QGroundControl

Item {
    id: root
    property var planner: null
    property var map: null
    property var controllers: []
    property var lines: []

    function refresh() {
        if (!map || !planner) return
        let segments = []
        for (const controller of controllers) segments = segments.concat(root.planner.speedSegments(controller))
        while (lines.length > segments.length) {
            const item = lines.pop()
            map.removeMapItem(item)
            item.destroy()
        }
        for (let i = 0; i < segments.length; ++i) {
            if (i >= lines.length) {
                const item = line.createObject(map)
                map.addMapItem(item)
                lines.push(item)
            }
            lines[i].path = segments[i].path
            lines[i].line.color = segments[i].color
        }
    }
    onPlannerChanged: refreshTimer.restart()
    onMapChanged: refreshTimer.restart()
    onControllersChanged: refreshTimer.restart()
    Component.onDestruction: {
        if (map) for (const item of lines) { map.removeMapItem(item); item.destroy() }
    }
    Instantiator {
        model: root.controllers
        delegate: Connections {
            required property var modelData
            target: modelData ? modelData.missionController : null
            function onMissionTimeChanged() { refreshTimer.restart() }
            function onVisualItemsReset() { refreshTimer.restart() }
            function onPlannedHomePositionChanged() { refreshTimer.restart() }
        }
    }
    Timer { id: refreshTimer; interval: 100; onTriggered: root.refresh() }
    Component { id: line; MapPolyline { z: QGroundControl.zOrderMapItems + 0.5; line.width: 8; opacity: 0.85 } }
}
