import QtQuick
import QtQuick.Layouts
import QtPositioning
import QGroundControl.Controls

ColumnLayout {
    id: root
    width: availableWidth
    property var block: missionItem
    property var values: block.values
    property var center: block.centerCoordinate
    QGCLabel { Layout.fillWidth: true; wrapMode: Text.WordWrap; text: root.block.error; visible: text.length > 0; color: "orange" }
    QGCLabel {
        Layout.fillWidth: true
        wrapMode: Text.WordWrap
        text: root.block.kind === "survey" ? qsTr("Edit the survey area on the map. Cyan shows the aircraft route; orange shows sample camera footprints.") : qsTr("Delay until uses NAV_DELAY seconds, separate from waypoint hold time.")
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
             ["yaw", "Aircraft heading (deg)"], ["delay", "Delay until (s)"], ["pitch", "Gimbal pitch (deg)"], ["speed", "Speed (m/s)"]]
        delegate: RowLayout {
            required property var modelData
            Layout.fillWidth: true
            QGCLabel { text: modelData[1]; Layout.fillWidth: true }
            QGCTextField {
                Layout.preferredWidth: 100
                text: modelData[0] === "latitude" ? root.center.latitude.toFixed(7) : modelData[0] === "longitude" ? root.center.longitude.toFixed(7) : root.values[modelData[0]]
                onEditingFinished: root.block.setValue(modelData[0], Number(text))
            }
        }
    }
    QGCLabel {
        visible: root.block.kind === "survey"
        Layout.fillWidth: true; wrapMode: Text.WordWrap
        text: qsTr("Flat ground at home height; rectified camera intrinsics required. Track spacing: %1 m. Mission arm (delay until 0 s), heading, pitch and speed are included.").arg(Number(root.block.preview.spacing || 0).toFixed(1))
    }
    RowLayout {
        visible: root.block.kind === "survey"
        QGCLabel { text: qsTr("Interrupt after route point") }
        QGCTextField { id: checkpoint; text: "1"; Layout.preferredWidth: 60 }
        QGCButton {
            text: qsTr("Add ROI")
            enabled: Number(checkpoint.text) >= 1 && Number(checkpoint.text) <= (root.block.preview.path || []).length
            onClicked: root.block.addInspection(Number(checkpoint.text) - 1)
        }
    }
    QGCLabel {
        visible: root.block.kind === "survey"
        Layout.fillWidth: true; wrapMode: Text.WordWrap
        text: qsTr("Route point numbers appear when selected. Review checkpoint positions after changing the survey. Each inspection returns to its checkpoint and restores survey settings.")
    }
    Repeater {
        model: root.block.inspections
        delegate: ColumnLayout {
            id: inspection
            required property var modelData
            required property int index
            Layout.fillWidth: true
            RowLayout {
                QGCLabel { text: qsTr("Inspection at point %1").arg(inspection.modelData.checkpoint + 1); Layout.fillWidth: true }
                QGCButton { text: qsTr("Remove"); onClicked: root.block.removeInspection(inspection.index) }
            }
            Repeater {
                model: inspection.modelData.points
                delegate: ColumnLayout {
                    id: pathPoint
                    required property var modelData
                    required property int index
                    Layout.fillWidth: true
                    RowLayout {
                        QGCLabel { text: (pathPoint.index + 1) + (pathPoint.modelData.roi ? " ROI target" : " Waypoint"); Layout.fillWidth: true }
                        QGCButton { text: "−"; onClicked: root.block.removeInspectionPoint(inspection.index, pathPoint.index) }
                    }
                    Repeater {
                        model: pathPoint.modelData.roi ? ["latitude", "longitude", "altitude"] : ["latitude", "longitude", "altitude", "yaw"]
                        delegate: RowLayout {
                            required property string modelData
                            QGCLabel { text: modelData; Layout.fillWidth: true }
                            QGCTextField {
                                Layout.preferredWidth: 120
                                text: pathPoint.modelData[modelData]
                                onEditingFinished: root.block.setInspectionPoint(inspection.index, pathPoint.index, modelData, Number(text))
                            }
                        }
                    }
                }
            }
            RowLayout {
                QGCButton { text: qsTr("Add waypoint"); onClicked: root.block.addInspectionPoint(inspection.index, false) }
                QGCButton { text: qsTr("Add ROI target"); onClicked: root.block.addInspectionPoint(inspection.index, true) }
            }
        }
    }
}
