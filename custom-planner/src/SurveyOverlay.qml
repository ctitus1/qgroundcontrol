import QtQuick
import QtLocation
import QGroundControl

Item {
    id: root
    property var map: null
    property var survey: null
    property var objects: []
    function clear() {
        for (var obj of objects) {
            if (map) map.removeMapItem(obj)
            obj.destroy()
        }
        objects = []
    }
    function refresh() {
        clear()
        if (!map || !survey) return
        var result = chimeraPlanner.preview(survey)
        if (!result.path) return
        var line = lineComponent.createObject(map, {path: result.path})
        map.addMapItem(line)
        objects.push(line)
        if (survey.isCurrentItem) {
            for (var corners of result.footprints) {
                var polygon = footprintComponent.createObject(map, {path: corners})
                map.addMapItem(polygon)
                objects.push(polygon)
            }
        }
    }
    onSurveyChanged: timer.restart()
    onMapChanged: timer.restart()
    Component.onDestruction: clear()
    Timer { id: timer; interval: 150; onTriggered: root.refresh() }
    Connections {
        target: chimeraPlanner
        function onSettingsChanged(item) { if (item === root.survey) timer.restart() }
    }
    Connections {
        target: root.survey
        function onVisualTransectPointsChanged() { timer.restart() }
        function onIsCurrentItemChanged() { timer.restart() }
    }
    Component {
        id: lineComponent
        MapPolyline { line.color: "#00d8ef"; line.width: 3; z: QGroundControl.zOrderMapItems + 1 }
    }
    Component {
        id: footprintComponent
        MapPolygon { color: "#10ffb040"; border.color: "#ffb040"; border.width: 1; z: QGroundControl.zOrderMapItems }
    }
}
