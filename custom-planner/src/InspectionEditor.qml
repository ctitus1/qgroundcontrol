import QtQuick
import QtQuick.Layouts
import QGroundControl.Controls
import QGroundControl.FactControls
import QGroundControl.Palette
import QGroundControl.ScreenTools

Rectangle {
    id: root
    property var block: missionItem
    property real margin: ScreenTools.defaultFontPixelHeight / 2
    property real fieldWidth: Math.min(135, (width - 2 * margin) * 0.58)
    property string notice: ""
    width: availableWidth
    height: fields.implicitHeight + 2 * margin
    color: qgcPal.windowShadeDark
    radius: ScreenTools.defaultFontPixelWidth / 2
    QGCPalette { id: qgcPal }
    ColumnLayout {
        id: fields
        anchors.left: parent.left; anchors.right: parent.right; anchors.top: parent.top
        anchors.margins: root.margin
        spacing: root.margin
        QGCLabel { text: root.block.error || root.notice; visible: text.length > 0; color: "orange"; Layout.fillWidth: true; wrapMode: Text.WordWrap }
        QGCTextField { text: root.block.inspectionName; Layout.fillWidth: true; onEditingFinished: root.block.inspectionName = text; Accessible.name: qsTr("Building and ROI name") }
        QGCLabel {
            Layout.fillWidth: true; wrapMode: Text.WordWrap
            text: qsTr("ROI → orbit → survey resumes. Drag the ROI marker to move the whole inspection. The orbit stays one editable mission item.")
        }
        QGCLabel { text: chimeraPlanner.placementPrompt; visible: chimeraPlanner.placementSource === root.block; Layout.fillWidth: true; wrapMode: Text.WordWrap }
        QGCButton { text: qsTr("Cancel placement"); visible: chimeraPlanner.placementSource === root.block; onClicked: chimeraPlanner.cancelPlacement(); Layout.fillWidth: true }
        Repeater {
            model: [["radius","Orbit radius"],["altitude","Flight altitude"],["targetHeight","ROI height above home"],["speed","Inspection speed"],["bearing","Entry bearing"],["arc","Orbit arc"]]
            delegate: RowLayout {
                required property var modelData
                Layout.fillWidth: true
                QGCLabel { text: modelData[1]; Layout.fillWidth: true; Layout.minimumWidth: 0; wrapMode: Text.WordWrap }
                FactTextField { fact: root.block.fact(modelData[0]); Layout.preferredWidth: root.fieldWidth; Layout.minimumWidth: root.fieldWidth; Layout.maximumWidth: root.fieldWidth; enabled: modelData[0] !== "altitude" || !chimeraFleet.enabled }
            }
        }
        QGCComboBox {
            model: [qsTr("Clockwise"), qsTr("Counterclockwise")]
            currentIndex: root.block.values.clockwise ? 0 : 1
            Layout.fillWidth: true
            onActivated: index => root.block.setValue("clockwise", index === 0 ? 1 : 0)
        }
        QGCCheckBox { text: qsTr("Aircraft faces ROI"); checked: root.block.values.faceTarget !== 0; onClicked: root.block.setValue("faceTarget",checked ? 1 : 0) }
        QGCLabel { visible: chimeraFleet.enabled; Layout.fillWidth: true; wrapMode: Text.WordWrap; text: qsTr("Flight altitude follows this drone's master layer. ROI height is separate.") }
        QGCButton { text: qsTr("Add another ROI / orbit"); Layout.fillWidth: true; onClicked: chimeraPlanner.beginInspection(root.block) }
        QGCCheckBox { text: qsTr("Show other routes"); checked: chimeraPlanner.showContext; onClicked: chimeraPlanner.showContext = checked }
        SectionHeader { id: position; text: qsTr("ROI coordinates / heading"); checked: false; Layout.fillWidth: true }
        ColumnLayout {
            visible: position.checked
            Layout.fillWidth: true
            Repeater {
                model: [["latitude","Latitude"],["longitude","Longitude"],["yaw","Fixed heading"]]
                delegate: RowLayout {
                    required property var modelData
                    QGCLabel { text: modelData[1]; Layout.fillWidth: true; wrapMode: Text.WordWrap }
                    FactTextField { fact: root.block.fact(modelData[0]); Layout.preferredWidth: root.fieldWidth; enabled: modelData[0] !== "yaw" || !root.block.values.faceTarget }
                }
            }
        }
        SectionHeader { id: advanced; text: qsTr("Custom waypoint path"); checked: false; Layout.fillWidth: true }
        ColumnLayout {
            visible: advanced.checked
            Layout.fillWidth: true
            QGCLabel { Layout.fillWidth: true; wrapMode: Text.WordWrap; text: qsTr("Convert this orbit to normal QGC ROI and waypoint items to change each point. This removes the orbit controls. Save a copy first if you need them later.") }
            QGCButton { text: qsTr("Convert to QGC waypoints"); Layout.fillWidth: true; onClicked: root.notice = root.block.expandInspection() }
        }
    }
}
