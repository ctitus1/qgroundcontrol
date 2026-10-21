import QtQuick
import QtQuick.Layouts
import QGroundControl
import QGroundControl.Controls
import QGroundControl.FactControls

Rectangle {
    id: root
    width: availableWidth
    height: editorColumn.height + margin * 2
    implicitHeight: height
    color: qgcPal.windowShadeDark
    radius: ScreenTools.defaultFontPixelWidth / 2
    required property var missionItem
    required property real availableWidth
    property var block: missionItem
    property bool fleetMasterSource: chimeraFleet.enabled && chimeraFleet.selected === 0 && block.kind === "survey" && block.fleetRegion === -1
    property real margin: ScreenTools.defaultFontPixelHeight / 2
    property real fieldWidth: Math.min(Math.max(120, ScreenTools.defaultFontPixelWidth * 18), (width - margin * 2) * 0.62)
    property string notice: ""

    function angleLabel(key) {
        if (fleetMasterSource)
            return key === "yaw" ? qsTr("Aircraft heading offset") : qsTr("Track angle offset")
        return key === "yaw" ? qsTr("Aircraft heading") : qsTr("Track angle")
    }
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
            text: root.fleetMasterSource ? qsTr("Edit the shared area, fleet setup, and flight defaults here. Master heading and track angle adjust the drones independently.") : root.block.kind === "survey" ? (root.block.linkedSurvey ? qsTr("Edits apply to the whole linked survey.") : qsTr("The polygon defines the ground area to cover.")) :
                  root.block.kind === "vantage" ? qsTr("Delay until 1 s arms vantage autonomy. 5g_drone controls the gimbal.") : qsTr("5g_drone controls the gimbal during the timed loiter.")
        }
        RowLayout {
            visible: root.block.kind === "fiducial" && root.block.fleetRegion === -1
            Layout.fillWidth: true
            QGCLabel { text: qsTr("Drone offset radius"); Layout.fillWidth: true; Layout.minimumWidth: 0; wrapMode: Text.WordWrap }
            FactTextField { fact: root.block.fact("radius"); Layout.preferredWidth: root.fieldWidth; Layout.minimumWidth: root.fieldWidth; Layout.maximumWidth: root.fieldWidth }
        }
        ColumnLayout {
            visible: root.block.kind === "fiducial" && root.block.fleetRegion === -1
            Layout.fillWidth: true
            QGCLabel { text: qsTr("Drone position rotation"); Layout.fillWidth: true }
            FactTextField { fact: root.block.fact("fiducialRotation"); Layout.preferredWidth: root.fieldWidth }
            QGCSlider {
                Layout.fillWidth: true
                from: 0; to: 359; stepSize: 1
                value: root.block.values.fiducialRotation
                onMoved: root.block.setValue("fiducialRotation", value)
            }
        }
        SectionHeader { id: flightHeader; text: root.fleetMasterSource ? qsTr("Search area") : qsTr("Flight"); Layout.fillWidth: true }
        ColumnLayout {
            visible: flightHeader.checked
            Layout.fillWidth: true
            spacing: root.margin / 2
            Repeater {
                model: root.block.kind === "survey" ? [
                    ["latitude", "Latitude"], ["longitude", "Longitude"], ["altitude", "Altitude"],
                    ["speed", "Speed"], ["transitSpeed", "Rapid transit speed"], ["pitch", "Gimbal pitch"], ["overlap", "Side overlap"],
                    ["yaw", "Heading"], ["angle", "Track angle"]
                ] : [["latitude", "Latitude"], ["longitude", "Longitude"], ["altitude", "Altitude"],
                     ["yaw", "Heading"], ["delay", "Loiter time"],
                     ["transitSpeed", "Rapid transit speed"]]
                delegate: RowLayout {
                    required property var modelData
                    visible: !root.fleetMasterSource || modelData[0] !== "altitude"
                    Layout.fillWidth: true
                    spacing: root.margin / 2
                    QGCLabel { text: ["yaw", "angle"].indexOf(modelData[0]) >= 0 ? root.angleLabel(modelData[0]) : modelData[1]; Layout.fillWidth: true; Layout.minimumWidth: 0; wrapMode: Text.Wrap }
                    FactTextField {
                        objectName: "field_" + modelData[0]
                        Layout.preferredWidth: root.fieldWidth
                        Layout.minimumWidth: root.fieldWidth
                        Layout.maximumWidth: root.fieldWidth
                        fact: root.block.fact(modelData[0])
                        enabled: (!root.block.splitPart || ["latitude", "longitude", "speed"].indexOf(modelData[0]) >= 0) &&
                            (root.block.fleetRegion === -1 || (root.block.fleetRegion >= 0 && ["latitude", "longitude", "transitSpeed"].indexOf(modelData[0]) < 0))
                    }
                }
            }
        }
        Repeater {
            model: ["yaw", "angle"]
            delegate: ColumnLayout {
                id: angleControl
                required property string modelData
                property real pendingValue: 0
                function commit() { root.block.setValue(modelData, pendingValue) }
                Layout.fillWidth: true
                visible: root.block.kind === "survey" && flightHeader.checked
                QGCLabel { text: root.angleLabel(angleControl.modelData) }
                QGCSlider {
                    id: angleSlider
                    objectName: angleControl.modelData === "yaw" ? "headingSlider" : "trackSlider"
                    enabled: !root.block.splitPart && root.block.fleetRegion >= -1
                    Layout.fillWidth: true
                    from: 0; to: 359; stepSize: 1
                    Binding {
                        target: angleSlider
                        property: "value"
                        value: root.block.values[angleControl.modelData]
                        when: !angleSlider.pressed
                        restoreMode: Binding.RestoreNone
                    }
                    onMoved: { angleControl.pendingValue = value; if (!angleUpdate.running) angleUpdate.start() }
                    onPressedChanged: if (!pressed && angleUpdate.running) { angleUpdate.stop(); angleControl.commit() }
                    Accessible.name: root.angleLabel(angleControl.modelData)
                }
                Timer {
                    id: angleUpdate
                    interval: 50
                    onTriggered: angleControl.commit()
                }
            }
        }
        QGCLabel {
            visible: root.block.kind === "survey" && flightHeader.checked
            Layout.fillWidth: true; wrapMode: Text.WordWrap
            font.pointSize: ScreenTools.smallFontPointSize
            text: qsTr("Aircraft heading and track angle are independent. Fleet defaults face the shared sector junction and fly across that heading.")
        }
        QGCButton {
            text: qsTr("Rotate entry point")
            visible: root.block.kind === "survey" && !root.fleetMasterSource
            enabled: !root.block.splitPart
            Layout.fillWidth: true
            onClicked: root.block.rotateEntryPoint()
        }
        QGCLabel {
            visible: root.block.kind === "survey" && root.block.centerOut && !root.fleetMasterSource
            Layout.fillWidth: true; wrapMode: Text.WordWrap
            text: qsTr("Rotate entry reverses the first pass. Center-out order and aircraft heading stay fixed.")
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
        Loader {
            Layout.fillWidth: true
            active: chimeraFleet.enabled && root.block.kind === "survey"
            source: "qrc:/qml/Chimera/Planner/FleetSettings.qml"
        }
        Loader {
            Layout.fillWidth: true
            active: root.block.kind === "survey" && root.block.fleetRegion === -1 && !chimeraFleet.enabled
            source: "qrc:/qml/Chimera/Planner/FleetSetup.qml"
            onLoaded: item.sourceSurvey = root.block
        }
        QGCLabel {
            visible: root.block.fleetRegion !== -1
            Layout.fillWidth: true; wrapMode: Text.WordWrap
            text: root.block.fleetRegion === -2 ? qsTr("Shared fiducial: edit its position and loiter time in Master view.") :
                qsTr("Region %1. Heading and altitude edits apply to this drone. Camera and flight settings can be adjusted for this region.").arg(root.block.fleetRegion + 1)
        }
        SectionHeader { id: splitHeader; text: qsTr("Split points"); visible: root.block.kind === "survey" && !(chimeraFleet.enabled && chimeraFleet.selected === 0 && root.block.fleetRegion === -1); Layout.fillWidth: true }
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
        QGCButton {
            text: qsTr("Export mission files…")
            visible: !chimeraFleet.enabled || chimeraFleet.selected > 0
            Layout.fillWidth: true
            onClicked: plainExport.openForSave()
        }
        QGCFileDialog {
            id: plainExport
            title: qsTr("Export editable and standard mission files")
            nameFilters: [qsTr("QGC plan (*.plan)")]
            onAcceptedForSave: file => {
                const error = chimeraFleet.enabled
                    ? chimeraFleet.exportAll(file)
                    : chimeraPlanner.exportMission(root.block, file)
                if (error) QGroundControl.showMessageDialog(root, qsTr("Export mission"), error)
                close()
            }
        }

    }
}
