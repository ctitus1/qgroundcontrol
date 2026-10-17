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
    color: qgcPal.windowShadeDark
    radius: ScreenTools.defaultFontPixelWidth / 2
    property var block: missionItem
    property real margin: ScreenTools.defaultFontPixelHeight / 2
    property real fieldWidth: Math.min(Math.max(120, ScreenTools.defaultFontPixelWidth * 18), (width - margin * 2) * 0.62)
    property string notice: ""
    QGCPalette { id: qgcPal; colorGroupEnabled: root.enabled }

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
                    ["speed", "Speed"], ["pitch", "Gimbal pitch"], ["overlap", "Side overlap"],
                    ["yaw", "Heading"], ["angle", "Track angle"]
                ] : [["latitude", "Latitude"], ["longitude", "Longitude"], ["altitude", "Altitude"],
                     ["yaw", "Heading"], ["delay", "Delay until"], ["speed", "Speed"]]
                delegate: RowLayout {
                    required property var modelData
                    Layout.fillWidth: true
                    spacing: root.margin / 2
                    QGCLabel { text: modelData[1]; Layout.fillWidth: true; Layout.minimumWidth: 0; wrapMode: Text.Wrap }
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
        QGCSlider {
            objectName: "headingSlider"
            visible: root.block.kind === "survey" && flightHeader.checked
            enabled: !root.block.splitPart
            Layout.fillWidth: true
            from: 0; to: 359; stepSize: 1
            value: root.block.values.yaw
            onMoved: root.block.setValue("yaw", value)
            Accessible.name: qsTr("Heading and track angle")
        }
        QGCLabel {
            visible: root.block.kind === "survey" && flightHeader.checked
            Layout.fillWidth: true; wrapMode: Text.WordWrap
            font.pointSize: ScreenTools.smallFontPointSize
            text: qsTr("Heading and track angle change together.")
        }
        QGCLabel { text: qsTr("Camera"); visible: root.block.kind === "survey" }
        QGCComboBox {
            objectName: "cameraPresetSelector"
            visible: root.block.kind === "survey"
            enabled: !root.block.splitPart
            Layout.fillWidth: true; Layout.minimumWidth: 0
            model: root.block.cameraPresets
            textRole: "label"
            currentIndex: root.block.cameraPreset
            sizeToContents: true
            onActivated: (index) => root.block.setCameraPreset(index)
            Accessible.name: qsTr("Aircraft and camera preset")
        }
        QGCLabel {
            Layout.fillWidth: true; Layout.minimumWidth: 0; wrapMode: Text.WordWrap
            visible: root.block.kind === "survey" && text.length > 0
            text: root.block.cameraPresets[root.block.cameraPreset].note || ""
            color: "orange"
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
            QGCLabel {
                Layout.fillWidth: true; Layout.minimumWidth: 0; wrapMode: Text.WordWrap
                font.pointSize: ScreenTools.smallFontPointSize
                text: qsTr("camera_matrix (pixels):\n[fx, 0, cx; 0, fy, cy; 0, 0, 1]\nEditing these values selects Custom.")
            }
            Repeater {
                model: root.block.kind === "survey" ? [["width", "image_width"], ["height", "image_height"], ["fx", "fx"], ["fy", "fy"], ["cx", "cx"], ["cy", "cy"]] : []
                delegate: RowLayout {
                    required property var modelData
                    Layout.fillWidth: true
                    spacing: root.margin / 2
                    QGCLabel { text: modelData[1]; Layout.fillWidth: true; Layout.minimumWidth: 0; wrapMode: Text.Wrap }
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
