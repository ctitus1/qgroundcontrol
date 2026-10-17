import QtQuick
import QtQuick.Layouts
import QGroundControl.Controls
import QGroundControl.FactControls
import QGroundControl.Palette
import QGroundControl.ScreenTools

Rectangle {
    id: root
    width: availableWidth
    height: editorColumn.height + margin * 2
    implicitHeight: height
    color: palette.windowShadeDark
    radius: ScreenTools.defaultFontPixelWidth / 2
    property var block: missionItem
    property real margin: ScreenTools.defaultFontPixelHeight / 2
    property real fieldWidth: Math.min(Math.max(120, ScreenTools.defaultFontPixelWidth * 18), (width - margin * 2) * 0.62)
    property string notice: ""
    QGCPalette { id: palette; colorGroupEnabled: root.enabled }

    ColumnLayout {
        id: editorColumn
        anchors.top: parent.top
        anchors.left: parent.left
        anchors.right: parent.right
        anchors.margins: root.margin
        spacing: root.margin
        QGCLabel {
            Layout.fillWidth: true; Layout.minimumWidth: 0
            wrapMode: Text.WordWrap
            text: root.block.error || root.notice
            visible: text.length > 0
            color: "orange"
        }
        QGCLabel {
            Layout.fillWidth: true; Layout.minimumWidth: 0
            font.pointSize: ScreenTools.smallFontPointSize
            wrapMode: Text.WordWrap
            text: root.block.kind === "survey" ? (root.block.linkedSurvey ? qsTr("Edits apply to the whole linked survey.") : qsTr("The polygon defines the ground area to cover.")) :
                  root.block.kind === "vantage" ? qsTr("Delay until 1 s arms vantage autonomy. 5g_drone controls the gimbal.") : qsTr("5g_drone controls the gimbal. Delay until is separate from waypoint hold time.")
        }
        SectionHeader { id: flightHeader; text: qsTr("Flight"); Layout.fillWidth: true }
        ColumnLayout {
            visible: flightHeader.checked
            Layout.fillWidth: true
            spacing: root.margin / 2
            Repeater {
                model: root.block.kind === "survey" ? [
                    ["latitude", "Latitude"], ["longitude", "Longitude"], ["altitude", "Altitude"],
                    ["yaw", "Heading"], ["pitch", "Gimbal pitch"], ["speed", "Speed"],
                    ["angle", "Track angle"], ["overlap", "Side overlap"]
                ] : [["latitude", "Latitude"], ["longitude", "Longitude"], ["altitude", "Altitude"],
                     ["yaw", "Heading"], ["delay", "Delay until"], ["speed", "Speed"]]
                delegate: RowLayout {
                    required property var modelData
                    Layout.fillWidth: true
                    spacing: root.margin / 2
                    QGCLabel { text: modelData[1]; Layout.fillWidth: true; Layout.minimumWidth: 0; wrapMode: Text.WordWrap }
                    FactTextField {
                        objectName: "field_" + modelData[0]
                        Layout.preferredWidth: root.fieldWidth
                        Layout.minimumWidth: root.fieldWidth
                        Layout.maximumWidth: root.fieldWidth
                        fact: root.block.fact(modelData[0])
                        enabled: !root.block.splitPart || ["latitude", "longitude", "speed"].indexOf(modelData[0]) >= 0
                    }
                }
            }
        }
        SectionHeader {
            id: cameraHeader; objectName: "cameraSection"
            text: qsTr("Camera intrinsics"); checked: false
            visible: root.block.kind === "survey"; Layout.fillWidth: true
        }
        ColumnLayout {
            visible: cameraHeader.visible && cameraHeader.checked
            Layout.fillWidth: true
            spacing: root.margin / 2
            Repeater {
                model: root.block.kind === "survey" ? [["width", "Image width"], ["height", "Image height"], ["fx", "fx"], ["fy", "fy"], ["cx", "cx"], ["cy", "cy"]] : []
                delegate: RowLayout {
                    required property var modelData
                    Layout.fillWidth: true
                    spacing: root.margin / 2
                    QGCLabel { text: modelData[1]; Layout.fillWidth: true; Layout.minimumWidth: 0; wrapMode: Text.WordWrap }
                    FactTextField {
                        objectName: "field_" + modelData[0]
                        Layout.preferredWidth: root.fieldWidth
                        Layout.minimumWidth: root.fieldWidth
                        Layout.maximumWidth: root.fieldWidth
                        fact: root.block.fact(modelData[0])
                        enabled: !root.block.splitPart
                    }
                }
            }
        }
        SectionHeader { id: splitHeader; text: qsTr("Split points"); visible: root.block.kind === "survey"; Layout.fillWidth: true }
        ColumnLayout {
            visible: splitHeader.visible && splitHeader.checked
            Layout.fillWidth: true
            QGCLabel {
                Layout.fillWidth: true; Layout.minimumWidth: 0; wrapMode: Text.WordWrap
                font.pointSize: ScreenTools.smallFontPointSize
                text: qsTr("Click + on a flight leg to add a split. Drag the split marker along the route to move it. Insert normal mission items between the survey parts.")
            }
            QGCButton {
                visible: root.block.hasEndSplit
                text: qsTr("Remove end split"); Layout.fillWidth: true
                onClicked: root.notice = root.block.removeEndSplit()
            }
            QGCButton {
                visible: root.block.splitPart
                text: qsTr("Restore editable survey"); Layout.fillWidth: true
                onClicked: root.notice = root.block.upgradeFrozenSurvey()
            }
            QGCButton {
                visible: root.block.inspections.length > 0
                text: qsTr("Remove embedded inspections"); Layout.fillWidth: true
                onClicked: root.block.clearInspections()
            }
        }
    }
}
