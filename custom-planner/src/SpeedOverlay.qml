pragma ComponentBehavior: Bound

import QtQuick
import QtQml.Models
import QtLocation
import QGroundControl

Item {
    id: root
    property var planner: null
    property var map: null
    property var missions: []
    property var lines: []

    function scheduleRefresh() {
        if (!refreshTimer.running) refreshTimer.start()
    }
    function refresh() {
        if (!map || !planner) return
        let segments = []
        for (const mission of missions) segments = segments.concat(root.planner.speedSegments(mission.controller, mission.itemCount))
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
    onPlannerChanged: root.scheduleRefresh()
    onMapChanged: root.scheduleRefresh()
    onMissionsChanged: root.scheduleRefresh()
    Component.onDestruction: {
        if (map) for (const item of lines) { map.removeMapItem(item); item.destroy() }
    }
    Instantiator {
        model: root.missions
        delegate: Connections {
            required property var modelData
            target: modelData && modelData.controller ? modelData.controller.missionController : null
            function onMissionTimeChanged() { root.scheduleRefresh() }
            function onVisualItemsReset() { root.scheduleRefresh() }
            function onPlannedHomePositionChanged() { root.scheduleRefresh() }
        }
    }
    Timer { id: refreshTimer; interval: 100; onTriggered: root.refresh() }
    Component { id: line; MapPolyline { z: 4; line.width: 8; opacity: 0.85 } }
}
