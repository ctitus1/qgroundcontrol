import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QGroundControl.Controls

ColumnLayout {
    id: root
    width: availableWidth
    property var block: missionItem
    property var values: block.values
    property var center: block.centerCoordinate
    property string splitError: ""
    QGCLabel {
        Layout.fillWidth: true; Layout.minimumWidth: 0
        wrapMode: Text.WordWrap; text: root.block.error || root.splitError
        visible: text.length > 0; color: "orange"
    }
    QGCLabel {
        Layout.fillWidth: true; Layout.minimumWidth: 0
        wrapMode: Text.WordWrap
        text: root.block.kind === "survey" ? (root.block.splitPart ? qsTr("Split route: geometry is fixed. Move this part as a unit or change its speed.") : qsTr("Edit the survey area on the map. The white line is the aircraft route; orange outlines show camera footprints.")) :
              root.block.kind === "vantage" ? qsTr("Starts with Delay until 1 s to arm vantage autonomy. 5g_drone controls the gimbal.") : qsTr("Delay until uses NAV_DELAY seconds, separate from waypoint hold time. 5g_drone controls the gimbal.")
    }
    Repeater {
        model: root.block.kind === "survey" ? [
            ["latitude", "Area centre latitude"], ["longitude", "Area centre longitude"],
            ["altitude", "Altitude above home (m)"], ["yaw", "Aircraft heading (deg)"],
            ["pitch", "Gimbal pitch (deg)"], ["speed", "Speed (m/s)"],
            ["angle", "Track angle (deg)"], ["overlap", "Lateral overlap (%)"],
            ["width", "Image width (px)"], ["height", "Image height (px)"],
            ["fx", "fx (px)"], ["fy", "fy (px)"], ["cx", "cx (px)"], ["cy", "cy (px)"]
        ] : [["latitude", "Latitude"], ["longitude", "Longitude"], ["altitude", "Altitude above home (m)"],
             ["yaw", "Aircraft heading (deg)"], ["delay", "Delay until (s)"], ["speed", "Speed (m/s)"]]
        delegate: ColumnLayout {
            required property var modelData
            Layout.fillWidth: true; Layout.minimumWidth: 0
            spacing: 2
            QGCLabel { text: modelData[1]; Layout.fillWidth: true; Layout.minimumWidth: 0; wrapMode: Text.WordWrap }
            QGCTextField {
                objectName: "field_" + modelData[0]
                Layout.fillWidth: true; Layout.minimumWidth: 0
                enabled: !root.block.splitPart || ["latitude", "longitude", "speed"].indexOf(modelData[0]) >= 0
                text: modelData[0] === "latitude" ? root.center.latitude.toFixed(7) : modelData[0] === "longitude" ? root.center.longitude.toFixed(7) : root.values[modelData[0]]
                onEditingFinished: root.block.setValue(modelData[0], Number(text))
            }
        }
    }
    QGCLabel {
        visible: root.block.kind === "survey" && !root.block.splitPart
        Layout.fillWidth: true; Layout.minimumWidth: 0; wrapMode: Text.WordWrap
        text: qsTr("Flat ground at home height; rectified camera intrinsics required. Track spacing: %1 m. Mission arm (Delay until 0 s), heading, pitch and speed are included.").arg(Number(root.block.preview.spacing || 0).toFixed(1))
    }
    ColumnLayout {
        visible: root.block.kind === "survey"
        Layout.fillWidth: true; Layout.minimumWidth: 0
        QGCLabel { text: qsTr("Split along flight path"); Layout.fillWidth: true; wrapMode: Text.WordWrap }
        Slider {
            objectName: "splitSlider"
            Layout.fillWidth: true; Layout.minimumWidth: 0
            from: 0; to: Math.max(0, (root.block.preview.path || []).length - 1)
            value: root.block.splitPosition
            onMoved: root.block.splitPosition = value
        }
        QGCLabel {
            Layout.fillWidth: true; Layout.minimumWidth: 0; wrapMode: Text.WordWrap
            text: qsTr("Route position %1. Click a route marker or move the slider to choose the split.").arg((root.block.splitPosition + 1).toFixed(2))
        }
        QGCButton {
            Layout.fillWidth: true; Layout.minimumWidth: 0
            text: qsTr("Split survey here")
            enabled: root.block.splitPosition > 0 && root.block.splitPosition < (root.block.preview.path || []).length - 1 && root.block.inspections.length === 0 && !root.block.error
            onClicked: root.splitError = root.block.splitSurvey()
        }
        QGCLabel {
            Layout.fillWidth: true; Layout.minimumWidth: 0; wrapMode: Text.WordWrap
            text: qsTr("The first part stays selected. Insert normal QGC ROI commands and waypoints after it. The second part returns to the split point, clears ROI, and restores survey heading, pitch, speed and camera triggering. Save a copy before splitting to keep the editable area.")
        }
        QGCButton {
            visible: root.block.inspections.length > 0
            Layout.fillWidth: true; Layout.minimumWidth: 0
            text: qsTr("Remove old embedded inspections")
            onClicked: root.block.clearInspections()
        }
    }
}
