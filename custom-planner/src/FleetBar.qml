import QtQuick
import QtQuick.Layouts
import QGroundControl
import QGroundControl.Controls

Rectangle {
    id: root
    objectName: "fleetToolbar"
    signal fitRequested()
    property real margin: ScreenTools.defaultFontPixelHeight / 3
    width: Math.min(bar.implicitWidth + margin * 2, typeof mainWindow !== "undefined" ? mainWindow.width - margin * 2 : bar.implicitWidth + margin * 2)
    height: bar.implicitHeight + margin * 2
    color: qgcPal.windowShadeDark
    radius: ScreenTools.defaultFontPixelWidth / 2
    QGCPalette { id: qgcPal }
    RowLayout {
        id: bar
        anchors.fill: parent
        anchors.margins: root.margin
        spacing: root.margin
        QGCComboBox {
            objectName: "fleetSelector"
            model: [qsTr("Master")].concat(chimeraFleet.drones.map(d => d.name))
            currentIndex: chimeraFleet.selected
            onActivated: index => { chimeraFleet.selected = index; root.fitRequested() }
            Layout.preferredWidth: ScreenTools.defaultFontPixelWidth * 16
            Layout.minimumWidth: 0
            Layout.fillWidth: true
            Layout.fillHeight: true
            Accessible.name: qsTr("Active mission")
        }
        QGCComboBox {
            objectName: "fleetPathDisplay"
            model: [qsTr("Whole path"), qsTr("First sector path"), qsTr("No paths")]
            currentIndex: chimeraFleet.pathDisplay
            onActivated: index => chimeraFleet.pathDisplay = index
            Layout.preferredWidth: ScreenTools.defaultFontPixelWidth * 20
            Layout.minimumWidth: 0
            Layout.fillWidth: true
            Layout.fillHeight: true
            sizeToContents: true
            Accessible.name: qsTr("Flight path display")
        }
        QGCButton {
            objectName: "fleetExport"
            text: qsTr("Export missions…")
            Layout.fillHeight: true
            onClicked: outputDialog.openForSave()
        }
    }
    QGCFileDialog {
        id: outputDialog
        title: qsTr("Export all mission files")
        nameFilters: [qsTr("QGC plan (*.plan)")]
        onAcceptedForSave: file => {
            const error = chimeraFleet.exportAll(file)
            if (error) QGroundControl.showMessageDialog(root, qsTr("Export mission"), error)
            close()
        }
    }
}
