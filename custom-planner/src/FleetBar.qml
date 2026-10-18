import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QGroundControl.Controls
import QGroundControl.Palette

Rectangle {
    id: root
    property var fit
    property bool standardExport: false
    width: bar.implicitWidth + 16
    height: bar.implicitHeight + 12
    color: qgcPal.windowShadeDark
    QGCPalette { id: qgcPal }
    RowLayout {
        id: bar
        anchors.centerIn: parent
        QGCComboBox {
            model: [qsTr("Master")].concat(chimeraFleet.drones.map(d => d.name))
            currentIndex: chimeraFleet.selected
            onActivated: index => { chimeraFleet.selected = index; if (root.fit) root.fit() }
            Layout.preferredWidth: 130
        }
        QGCLabel { text: qsTr("Layer gap: %1 m").arg(chimeraFleet.separation.toFixed(1)) }
        QGCButton { text: qsTr("Master setup"); onClicked: settings.open() }
        QGCButton { text: qsTr("Export all"); onClicked: exportMenu.popup() }
    }
    QGCMenu {
        id: exportMenu
        QGCMenuItem { text: qsTr("Editable plans"); onTriggered: { root.standardExport = false; exportDialog.openForSave() } }
        QGCMenuItem { text: qsTr("Standard QGC plans (no plugin)"); onTriggered: { root.standardExport = true; exportDialog.openForSave() } }
    }
    QGCFileDialog {
        id: exportDialog
        title: qsTr("Mission name for all drone plans")
        nameFilters: [qsTr("QGC plan (*.plan)")]
        onAcceptedForSave: file => {
            const error = chimeraFleet.exportAll(file, root.standardExport)
            if (error) mainWindow.showMessageDialog(qsTr("Fleet plan"), error)
            close()
        }
    }
    Popup {
        id: settings
        objectName: "fleetSettings"
        width: 400
        height: Math.min(650, (typeof mainWindow !== "undefined" ? mainWindow.height : 800) - 80)
        modal: false
        closePolicy: Popup.CloseOnEscape
        background: Rectangle { color: qgcPal.windowShadeDark }
        contentItem: Flickable {
            contentHeight: fields.implicitHeight
            clip: true
            ScrollBar.vertical: ScrollBar {}
            ColumnLayout {
                id: fields
                width: parent.width
                RowLayout {
                    QGCLabel { text: qsTr("Master setup"); font.bold: true; Layout.fillWidth: true }
                    QGCButton { text: qsTr("Close"); onClicked: settings.close() }
                }
                RowLayout {
                    QGCLabel { text: qsTr("Drones"); Layout.fillWidth: true }
                    QGCComboBox {
                        model: [2,3,4,5,6,7,8]
                        currentIndex: chimeraFleet.drones.length - 2
                        Layout.preferredWidth: 100
                        onActivated: index => {
                            const error = chimeraFleet.setCount(index + 2)
                            if (error) mainWindow.showMessageDialog(qsTr("Fleet plan"), error)
                        }
                    }
                }
                QGCLabel {
                    Layout.fillWidth: true; wrapMode: Text.WordWrap
                    text: qsTr("Change the count at any time. Removed drones and regions stay saved and return when the count increases. Existing split points follow their region. Manual inspection coordinates stay fixed.")
                }
                RowLayout {
                    QGCLabel { text: qsTr("Camera mode"); Layout.fillWidth: true }
                    QGCComboBox {
                        model: [qsTr("Day / visible"), qsTr("Night / thermal")]
                        currentIndex: chimeraFleet.night ? 1 : 0
                        Layout.preferredWidth: 180
                        onActivated: index => chimeraFleet.night = index === 1
                    }
                }
                QGCLabel { Layout.fillWidth: true; wrapMode: Text.WordWrap; text: qsTr("In Master view, drag the yellow center and blue ray handles to divide the area. Colors show each drone's starting region.") }
                QGCCheckBox { text: qsTr("Show flight paths"); checked: chimeraFleet.showRoutes; onClicked: chimeraFleet.showRoutes = checked }
                QGCButton { text: qsTr("Reset equal sectors"); onClicked: chimeraFleet.resetPartition() }
                QGCLabel { text: qsTr("Shared altitude layers"); font.bold: true }
                Repeater {
                    model: [["Minimum altitude (m)", "minimum"], ["Maximum altitude (m)", "maximum"], ["Maximum gap (m, 0 = no limit)", "maximumGap"]]
                    delegate: RowLayout {
                        required property var modelData
                        QGCLabel { text: modelData[0]; Layout.fillWidth: true; wrapMode: Text.WordWrap }
                        QGCTextField {
                            Layout.preferredWidth: 100
                            text: String(chimeraFleet[modelData[1]])
                            validator: DoubleValidator { bottom: 0 }
                            onEditingFinished: chimeraFleet.setLayers(modelData[1] === "minimum" ? Number(text) : chimeraFleet.minimum,
                                modelData[1] === "maximum" ? Number(text) : chimeraFleet.maximum,
                                modelData[1] === "maximumGap" ? Number(text) : chimeraFleet.maximumGap)
                        }
                    }
                }
                QGCLabel { Layout.fillWidth: true; wrapMode: Text.WordWrap; text: qsTr("Gap: %1 m. Layers use a shared home elevation; they do not separate takeoff or landing. Drones advance independently, without a phase barrier.").arg(chimeraFleet.separation.toFixed(1)) }
                Repeater {
                    model: chimeraFleet.drones.length
                    delegate: ColumnLayout {
                        required property int index
                        property int drone: index
                        property var profile: chimeraFleet.drones[index] || ({})
                        Layout.fillWidth: true
                        SectionHeader { id: droneHeader; text: profile.name + " / " + Number(profile.altitude).toFixed(1) + " m"; Layout.fillWidth: true }
                        ColumnLayout {
                            visible: droneHeader.checked
                            Layout.fillWidth: true
                            RowLayout {
                                QGCLabel { text: qsTr("Aircraft"); Layout.fillWidth: true }
                                QGCComboBox {
                                    model: ["UAS1","UAS2","UAS3","UAS4","UAS5","UAS6","UAS7","UAS8"]
                                    currentIndex: (profile.uas || drone + 1) - 1
                                    Layout.preferredWidth: 170
                                    onActivated: index => chimeraFleet.setProfile(drone,"uas",index+1)
                                }
                            }
                            RowLayout {
                                QGCLabel { text: qsTr("Start region"); Layout.fillWidth: true }
                                QGCComboBox {
                                    model: Array.from({length: chimeraFleet.drones.length}, (_, n) => "R" + (n+1))
                                    currentIndex: profile.startRegion || 0
                                    Layout.preferredWidth: 170
                                    onActivated: index => chimeraFleet.setProfile(drone,"startRegion",index)
                                }
                            }
                            QGCLabel { text: qsTr("Region order: %1").arg(Array.from({length: chimeraFleet.drones.length}, (_, n) => ((Number(profile.startRegion || 0)+n)%chimeraFleet.drones.length)+1).join(" → ")); Layout.fillWidth: true; wrapMode: Text.WordWrap }
                            Repeater {
                                model: [["Heading / track angle (deg)","heading",0,359.99],["Speed (m/s)","speed",0.01,100],["Gimbal pitch (deg)","pitch",-90,-0.01],["Side overlap (%)","overlap",0,94.99],["Takeoff latitude","latitude",-90,90],["Takeoff longitude","longitude",-180,180]]
                                delegate: RowLayout {
                                    required property var modelData
                                    QGCLabel { text: modelData[0]; Layout.fillWidth: true; wrapMode: Text.WordWrap }
                                    QGCTextField {
                                        Layout.preferredWidth: 145
                                        text: Number(profile[modelData[1]]).toFixed(["latitude","longitude"].indexOf(modelData[1]) >= 0 ? 7 : 2)
                                        validator: DoubleValidator { bottom: modelData[2]; top: modelData[3]; locale: "C" }
                                        onEditingFinished: chimeraFleet.setProfile(drone, modelData[1], Number(text))
                                    }
                                }
                            }
                            QGCSlider {
                                Layout.fillWidth: true
                                from: 0; to: 359; stepSize: 1
                                value: profile.heading || 0
                                onPressedChanged: if (!pressed) chimeraFleet.setProfile(drone,"heading",value)
                                Accessible.name: qsTr("Fixed aircraft heading and track angle")
                            }
                            QGCComboBox {
                                Layout.fillWidth: true
                                model: [qsTr("Automatic day/night camera")].concat(chimeraFleet.cameraPresets.map(p => p.label))
                                currentIndex: profile.cameraAutomatic ? 0 : Number(profile.cameraPreset || 0)+1
                                onActivated: index => chimeraFleet.setProfile(drone,"cameraPreset",index-1)
                            }
                            QGCLabel {
                                Layout.fillWidth: true; wrapMode: Text.WordWrap
                                text: (chimeraFleet.cameraPresets[profile.cameraPreset || 0] || {}).label || ""
                            }
                            QGCLabel {
                                Layout.fillWidth: true; wrapMode: Text.WordWrap; color: "orange"
                                text: (chimeraFleet.cameraPresets[profile.cameraPreset || 0] || {}).note || ""
                                visible: text.length > 0
                            }
                            SectionHeader { id: intrinsics; text: qsTr("Camera intrinsics"); checked: false; Layout.fillWidth: true }
                            ColumnLayout {
                                visible: intrinsics.checked
                                Layout.fillWidth: true
                                Repeater {
                                    model: ["width","height","fx","fy","cx","cy"]
                                    delegate: RowLayout {
                                        required property string modelData
                                        QGCLabel { text: modelData + " (px)"; Layout.fillWidth: true }
                                        QGCTextField {
                                            Layout.preferredWidth: 145
                                            text: String(profile[modelData])
                                            validator: DoubleValidator { bottom: 0; locale: "C" }
                                            onEditingFinished: chimeraFleet.setProfile(drone,modelData,Number(text))
                                        }
                                    }
                                }
                            }
                            QGCLabel { Layout.fillWidth: true; wrapMode: Text.WordWrap; text: qsTr("These settings apply to every region for this drone. Use the drone editor for region overrides and ROI waypoints.") }
                            QGCButton { text: qsTr("Apply defaults to every region"); onClicked: chimeraFleet.applyDroneSettings(drone) }
                            RowLayout {
                                QGCComboBox {
                                    id: inspectRegion
                                    model: Array.from({length: chimeraFleet.drones.length}, (_, n) => "R" + (n+1))
                                    Layout.fillWidth: true
                                }
                                QGCButton { text: qsTr("Edit region / ROI"); onClicked: { chimeraFleet.editRegion(drone,inspectRegion.currentIndex); settings.close() } }
                            }
                        }
                    }
                }
                QGCLabel { Layout.fillWidth: true; wrapMode: Text.WordWrap; text: qsTr("Export all saves <name>.plan as the master and <name>-uasN.plan for each aircraft.") }
                QGCLabel { text: chimeraFleet.error; visible: text.length > 0; Layout.fillWidth: true; wrapMode: Text.WordWrap; color: "orange" }
                QGCButton { text: qsTr("Close"); onClicked: settings.close() }
            }
        }
    }
}
