import QtQuick
import QtQuick.Layouts
import QGroundControl
import QGroundControl.Controls

ColumnLayout {
    id: root
    objectName: "fleetSettings"
    readonly property real fieldWidth: Math.min(ScreenTools.defaultFontPixelWidth * 16, width * 0.55)
    property bool master: chimeraFleet.selected === 0
    spacing: ScreenTools.defaultFontPixelHeight / 2
    SectionHeader {
        id: setupHeader
        text: root.master ? qsTr("Master setup") : qsTr("Drone setup")
        Layout.fillWidth: true
        checked: root.master
    }
    ColumnLayout {
        id: settingsFields
        visible: setupHeader.checked
        Layout.fillWidth: true
        ColumnLayout {
            visible: root.master
            Layout.fillWidth: true
            RowLayout {
                QGCLabel {
                    text: qsTr("Drones")
                    Layout.fillWidth: true
                }
                QGCComboBox {
                    sizeToContents: true
                    model: [2, 3, 4, 5, 6, 7, 8]
                    currentIndex: chimeraFleet.drones.length - 2
                    Layout.preferredWidth: root.fieldWidth
                    Layout.minimumWidth: 0
                    Layout.maximumWidth: root.fieldWidth
                    onActivated: index => {
                        const error = chimeraFleet.setCount(index + 2);
                        if (error)
                            QGroundControl.showMessageDialog(root, qsTr("Fleet plan"), error);
                    }
                }
            }
            QGCLabel {
                Layout.fillWidth: true
                Layout.minimumWidth: 0
                wrapMode: Text.WordWrap
                text: qsTr("Change the count at any time. Removed drones and regions stay saved and return when the count increases. Existing split points follow their region. Manual inspection coordinates stay fixed.")
            }
            RowLayout {
                QGCLabel {
                    text: qsTr("Camera mode")
                    Layout.fillWidth: true
                }
                QGCComboBox {
                    sizeToContents: true
                    model: [qsTr("Day / visible"), qsTr("Night / thermal")]
                    currentIndex: chimeraFleet.night ? 1 : 0
                    Layout.preferredWidth: root.fieldWidth
                    Layout.minimumWidth: 0
                    Layout.maximumWidth: root.fieldWidth
                    onActivated: index => chimeraFleet.night = index === 1
                }
            }
            QGCLabel {
                Layout.fillWidth: true
                Layout.minimumWidth: 0
                wrapMode: Text.WordWrap
                text: qsTr("In Master view, drag the yellow center and blue ray handles to divide the area. Colors show each drone's starting region.")
            }
            QGCButton {
                text: qsTr("Reset equal sectors")
                onClicked: chimeraFleet.resetPartition()
            }
            RowLayout {
                QGCLabel {
                    text: qsTr("Fiducial radius (m)")
                    Layout.fillWidth: true
                    Layout.minimumWidth: 0
                    wrapMode: Text.WordWrap
                }
                QGCTextField {
                    Layout.preferredWidth: root.fieldWidth
                    text: String(chimeraFleet.fiducialRadius)
                    validator: DoubleValidator {
                        bottom: 0
                        locale: "C"
                    }
                    onEditingFinished: chimeraFleet.fiducialRadius = Number(text)
                }
            }
            QGCLabel { text: qsTr("Fiducial position rotation"); Layout.fillWidth: true }
            QGCSlider {
                Layout.fillWidth: true
                from: 0; to: 359; stepSize: 1
                value: chimeraFleet.fiducialRotation
                onMoved: chimeraFleet.fiducialRotation = value
            }
            QGCLabel {
                text: qsTr("%1° — aircraft headings stay with their surveys").arg(chimeraFleet.fiducialRotation.toFixed(0))
                Layout.fillWidth: true
                wrapMode: Text.WordWrap
            }
            QGCLabel { text: qsTr("Fiducial heading relative to center"); Layout.fillWidth: true }
            QGCSlider {
                Layout.fillWidth: true
                from: 0; to: 359; stepSize: 1
                value: chimeraFleet.fiducialHeading
                onMoved: chimeraFleet.fiducialHeading = value
            }
            QGCLabel {
                text: qsTr("%1° — 0° faces the center; 90° faces sideways").arg(chimeraFleet.fiducialHeading.toFixed(0))
                Layout.fillWidth: true
                wrapMode: Text.WordWrap
            }
            RowLayout {
                QGCLabel {
                    text: qsTr("Rapid transit speed (m/s)")
                    Layout.fillWidth: true
                    Layout.minimumWidth: 0
                    wrapMode: Text.WordWrap
                }
                QGCTextField {
                    Layout.preferredWidth: root.fieldWidth
                    text: String(chimeraFleet.rapidTransitSpeed)
                    validator: DoubleValidator {
                        bottom: 0.1
                        locale: "C"
                    }
                    onEditingFinished: chimeraFleet.rapidTransitSpeed = Number(text)
                }
            }
            QGCLabel {
                text: qsTr("Shared altitude layers")
                font.bold: true
            }
            Repeater {
                model: [["Minimum altitude (m)", "minimum"], ["Maximum altitude (m)", "maximum"], ["Maximum gap (m, 0 = no limit)", "maximumGap"]]
                delegate: RowLayout {
                    required property var modelData
                    QGCLabel {
                        text: modelData[0]
                        Layout.fillWidth: true
                        Layout.minimumWidth: 0
                        wrapMode: Text.WordWrap
                    }
                    QGCTextField {
                        Layout.preferredWidth: root.fieldWidth
                        text: String(chimeraFleet[modelData[1]])
                        validator: DoubleValidator {
                            bottom: 0
                        }
                        onEditingFinished: chimeraFleet.setLayers(modelData[1] === "minimum" ? Number(text) : chimeraFleet.minimum, modelData[1] === "maximum" ? Number(text) : chimeraFleet.maximum, modelData[1] === "maximumGap" ? Number(text) : chimeraFleet.maximumGap)
                    }
                }
            }
            QGCLabel {
                Layout.fillWidth: true
                Layout.minimumWidth: 0
                wrapMode: Text.WordWrap
                text: qsTr("Gap: %1 m. Layers use a shared home elevation; they do not separate takeoff or landing. Drones advance independently, without a phase barrier.").arg(chimeraFleet.separation.toFixed(1))
            }
        }
        Repeater {
            model: chimeraFleet.drones.length
            delegate: ColumnLayout {
                required property int index
                property int drone: index
                visible: root.master || index === chimeraFleet.selected - 1
                property var profile: chimeraFleet.drones[index] || ({})
                Layout.fillWidth: true
                SectionHeader {
                    id: droneHeader
                    checked: !root.master
                    text: profile.name + " / " + Number(profile.altitude).toFixed(1) + " m"
                    Layout.fillWidth: true
                }
                ColumnLayout {
                    visible: droneHeader.checked
                    Layout.fillWidth: true
                    RowLayout {
                        QGCLabel {
                            text: qsTr("Aircraft")
                            Layout.fillWidth: true
                        }
                        QGCComboBox {
                            sizeToContents: true
                            model: ["UAS1", "UAS2", "UAS3", "UAS4", "UAS5", "UAS6", "UAS7", "UAS8"]
                            currentIndex: (profile.uas || drone + 1) - 1
                            Layout.preferredWidth: root.fieldWidth
                            Layout.minimumWidth: 0
                            Layout.maximumWidth: root.fieldWidth
                            onActivated: index => chimeraFleet.setProfile(drone, "uas", index + 1)
                        }
                    }
                    RowLayout {
                        QGCLabel {
                            text: qsTr("Start region")
                            Layout.fillWidth: true
                        }
                        QGCComboBox {
                            sizeToContents: true
                            model: Array.from({
                                length: chimeraFleet.drones.length
                            }, (_, n) => "R" + (n + 1))
                            currentIndex: profile.startRegion || 0
                            Layout.preferredWidth: root.fieldWidth
                            Layout.minimumWidth: 0
                            Layout.maximumWidth: root.fieldWidth
                            onActivated: index => chimeraFleet.setProfile(drone, "startRegion", index)
                        }
                    }
                    QGCLabel {
                        text: qsTr("Region order: %1").arg(Array.from({
                            length: chimeraFleet.drones.length
                        }, (_, n) => ((Number(profile.startRegion || 0) + n) % chimeraFleet.drones.length) + 1).join(" → "))
                        Layout.fillWidth: true
                        Layout.minimumWidth: 0
                        wrapMode: Text.WordWrap
                    }
                    Repeater {
                        model: [["Aircraft heading (deg)", "heading", 0, 359.99], ["Track angle (deg)", "trackAngle", 0, 359.99], ["Altitude (m)", "altitude", chimeraFleet.minimum, chimeraFleet.maximum], ["Speed (m/s)", "speed", 0.01, 100], ["Gimbal pitch (deg)", "pitch", -90, -0.01], ["Side overlap (%)", "overlap", 0, 94.99], ["Takeoff latitude", "latitude", -90, 90], ["Takeoff longitude", "longitude", -180, 180]]
                        delegate: RowLayout {
                            required property var modelData
                            QGCLabel {
                                text: modelData[0]
                                Layout.fillWidth: true
                                Layout.minimumWidth: 0
                                wrapMode: Text.WordWrap
                            }
                            QGCTextField {
                                Layout.preferredWidth: root.fieldWidth
                                Layout.minimumWidth: 0
                                Layout.maximumWidth: root.fieldWidth
                                text: Number(profile[modelData[1]]).toFixed(["latitude", "longitude"].indexOf(modelData[1]) >= 0 ? 7 : 2)
                                validator: DoubleValidator {
                                    bottom: modelData[2]
                                    top: modelData[3]
                                    locale: "C"
                                }
                                onEditingFinished: chimeraFleet.setProfile(drone, modelData[1], Number(text))
                            }
                        }
                    }
                    Repeater {
                        model: [["heading", qsTr("Aircraft heading")], ["trackAngle", qsTr("Track angle")]]
                        delegate: ColumnLayout {
                            id: profileAngleControl
                            required property var modelData
                            Layout.fillWidth: true
                            QGCLabel { text: profileAngleControl.modelData[1] }
                            QGCSlider {
                                id: profileAngleSlider
                                Layout.fillWidth: true
                                from: 0
                                to: 359
                                stepSize: 1
                                Binding {
                                    target: profileAngleSlider
                                    property: "value"
                                    value: profile[profileAngleControl.modelData[0]] || 0
                                    when: !profileAngleSlider.pressed
                                    restoreMode: Binding.RestoreNone
                                }
                                onMoved: chimeraFleet.setProfile(drone, profileAngleControl.modelData[0], value, true)
                                onPressedChanged: if (!pressed) chimeraFleet.synchronize()
                                Accessible.name: profileAngleControl.modelData[1]
                            }
                        }
                    }
                    QGCComboBox {
                        sizeToContents: true
                        Layout.fillWidth: true
                        Layout.minimumWidth: 0
                        model: [qsTr("Automatic day/night camera")].concat(chimeraFleet.cameraPresets.map(p => p.label))
                        currentIndex: profile.cameraAutomatic ? 0 : Number(profile.cameraPreset || 0) + 1
                        onActivated: index => chimeraFleet.setProfile(drone, "cameraPreset", index - 1)
                    }
                    QGCLabel {
                        Layout.fillWidth: true
                        Layout.minimumWidth: 0
                        wrapMode: Text.WordWrap
                        text: (chimeraFleet.cameraPresets[profile.cameraPreset || 0] || {}).label || ""
                    }
                    QGCLabel {
                        Layout.fillWidth: true
                        Layout.minimumWidth: 0
                        wrapMode: Text.WordWrap
                        color: "orange"
                        text: (chimeraFleet.cameraPresets[profile.cameraPreset || 0] || {}).note || ""
                        visible: text.length > 0
                    }
                    SectionHeader {
                        id: intrinsics
                        text: qsTr("Camera intrinsics")
                        checked: false
                        Layout.fillWidth: true
                    }
                    ColumnLayout {
                        visible: intrinsics.checked
                        Layout.fillWidth: true
                        Repeater {
                            model: ["width", "height", "fx", "fy", "cx", "cy"]
                            delegate: RowLayout {
                                required property string modelData
                                QGCLabel {
                                    text: modelData + " (px)"
                                    Layout.fillWidth: true
                                }
                                QGCTextField {
                                    Layout.preferredWidth: root.fieldWidth
                                    Layout.minimumWidth: 0
                                    Layout.maximumWidth: root.fieldWidth
                                    text: String(profile[modelData])
                                    validator: DoubleValidator {
                                        bottom: 0
                                        locale: "C"
                                    }
                                    onEditingFinished: chimeraFleet.setProfile(drone, modelData, Number(text))
                                }
                            }
                        }
                    }
                    QGCLabel {
                        Layout.fillWidth: true
                        Layout.minimumWidth: 0
                        wrapMode: Text.WordWrap
                        text: qsTr("Drone settings apply to every region. Edit region fields below for local camera and speed overrides.")
                    }
                    QGCButton {
                        text: qsTr("Apply defaults to every region")
                        onClicked: chimeraFleet.applyDroneSettings(drone)
                    }
                    RowLayout {
                        QGCComboBox {
                            sizeToContents: true
                            id: inspectRegion
                            Layout.minimumWidth: 0
                            Layout.preferredWidth: ScreenTools.defaultFontPixelWidth * 5
                            Accessible.name: qsTr("Region to edit")
                            model: Array.from({
                                length: chimeraFleet.drones.length
                            }, (_, n) => "R" + (n + 1))
                            Layout.fillWidth: true
                        }
                        QGCButton {
                            text: qsTr("Edit region / ROI")
                            onClicked: {
                                chimeraFleet.editRegion(drone, inspectRegion.currentIndex);
                            }
                        }
                    }
                }
            }
        }
        QGCLabel {
            Layout.fillWidth: true
            Layout.minimumWidth: 0
            wrapMode: Text.WordWrap
            text: qsTr("Export all saves the master, editable drone plans, and standard QGC drone plans together.")
        }
        QGCLabel {
            text: chimeraFleet.error
            visible: text.length > 0
            Layout.fillWidth: true
            Layout.minimumWidth: 0
            wrapMode: Text.WordWrap
            color: "orange"
        }

        QGCButton {
            text: qsTr("Export mission files…")
            Layout.fillWidth: true
            onClicked: outputDialog.openForSave()
        }
        QGCFileDialog {
            id: outputDialog
            title: qsTr("Export all mission files")
            nameFilters: [qsTr("QGC plan (*.plan)")]
            onAcceptedForSave: file => {
                const error = chimeraFleet.exportAll(file);
                if (error)
                    QGroundControl.showMessageDialog(root, qsTr("Export mission"), error);
                close();
            }
        }
    }
}
