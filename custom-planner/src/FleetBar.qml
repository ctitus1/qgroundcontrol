import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QGroundControl.Controls
import QGroundControl.Palette

Rectangle {
    id: root
    property var fit
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
        QGCButton { text: qsTr("Fleet setup"); onClicked: settings.open() }
        QGCButton { text: qsTr("Export drone"); enabled: chimeraFleet.selected > 0; onClicked: exportDialog.openForSave() }
    }
    QGCFileDialog {
        id: exportDialog
        title: qsTr("Export drone plan")
        nameFilters: [qsTr("QGC plan (*.plan)")]
        onAcceptedForSave: file => {
            const error = chimeraFleet.exportDrone(chimeraFleet.selected - 1, file)
            if (error) mainWindow.showMessageDialog(qsTr("Fleet plan"), error)
            close()
        }
    }
    Popup {
        id: settings
        width: 360
        height: Math.min(650, (typeof mainWindow !== "undefined" ? mainWindow.height : 800) - 80)
        modal: true
        closePolicy: Popup.CloseOnEscape | Popup.CloseOnPressOutside
        background: Rectangle { color: qgcPal.windowShadeDark }
        contentItem: Flickable {
            contentHeight: fields.implicitHeight
            clip: true
            ColumnLayout {
                id: fields
                width: parent.width
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
                    model: chimeraFleet.drones
                    delegate: ColumnLayout {
                        required property var modelData
                        required property int index
                        property int drone: index
                        property var profile: modelData
                        Layout.fillWidth: true
                        QGCLabel { text: modelData.name + " / " + Number(modelData.altitude).toFixed(1) + " m"; color: modelData.color; font.bold: true }
                        QGCLabel { text: qsTr("Region order: %1").arg(Array.from({length: chimeraFleet.drones.length}, (_, n) => ((drone+n)%chimeraFleet.drones.length)+1).join(" → ")); Layout.fillWidth: true; wrapMode: Text.WordWrap }
                        Repeater {
                            model: [["Heading (deg)","heading"],["Takeoff latitude","latitude"],["Takeoff longitude","longitude"]]
                            delegate: RowLayout {
                                required property var modelData
                                QGCLabel { text: modelData[0]; Layout.fillWidth: true; wrapMode: Text.WordWrap }
                                QGCTextField {
                                    Layout.preferredWidth: 145
                                    text: Number(profile[modelData[1]]).toFixed(modelData[1] === "heading" ? 1 : 7)
                                    validator: DoubleValidator { locale: "C" }
                                    onEditingFinished: chimeraFleet.setProfile(drone, modelData[1], Number(text))
                                }
                            }
                        }
                    }
                }
                QGCLabel { text: chimeraFleet.error; visible: text.length > 0; Layout.fillWidth: true; wrapMode: Text.WordWrap; color: "orange" }
                QGCButton { text: qsTr("Close"); onClicked: settings.close() }
            }
        }
    }
}
