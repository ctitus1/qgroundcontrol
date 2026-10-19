import QtQuick
import QtQuick.Layouts
import QGroundControl
import QGroundControl.Controls

ColumnLayout {
    id: root
    readonly property real fieldWidth: Math.min(ScreenTools.defaultFontPixelWidth * 16, width * 0.55)
    property var sourceSurvey
    spacing: ScreenTools.defaultFontPixelHeight / 2
    SectionHeader { id: header; text: qsTr("Fleet setup"); checked: false; Layout.fillWidth: true }
    ColumnLayout {
        visible: header.checked
        Layout.fillWidth: true
        QGCLabel { Layout.fillWidth: true; wrapMode: Text.WordWrap; text: qsTr("Use one master survey and one shared fiducial. Each drone visits every region, starting in its own region. Drone missions keep independent camera settings and inspection paths.") }
        RowLayout {
            QGCLabel { text: qsTr("Drones"); Layout.fillWidth: true }
            QGCComboBox { id: count; sizeToContents: true; Accessible.name: qsTr("Number of drones"); model: [2,3,4,5,6,7,8]; Layout.preferredWidth: root.fieldWidth; Layout.minimumWidth: 0; Layout.maximumWidth: root.fieldWidth }
        }
        RowLayout {
            QGCLabel { text: qsTr("Min altitude (m)"); Layout.fillWidth: true; wrapMode: Text.WordWrap }
            QGCTextField { id: minimum; objectName: "fleetMinimum"; text: "20"; Layout.preferredWidth: root.fieldWidth; Layout.minimumWidth: 0; Layout.maximumWidth: root.fieldWidth; validator: DoubleValidator { bottom: 0.1 } }
        }
        RowLayout {
            QGCLabel { text: qsTr("Max altitude (m)"); Layout.fillWidth: true; wrapMode: Text.WordWrap }
            QGCTextField { id: maximum; objectName: "fleetMaximum"; text: "30"; Layout.preferredWidth: root.fieldWidth; Layout.minimumWidth: 0; Layout.maximumWidth: root.fieldWidth; validator: DoubleValidator { bottom: 0.1 } }
        }
        QGCLabel { Layout.fillWidth: true; wrapMode: Text.WordWrap; text: qsTr("Altitude layers assume all launch sites share the master home elevation. Takeoff and landing are not sequenced.") }
        QGCButton {
            objectName: "fleetCreate"; text: qsTr("Create drone missions"); Layout.fillWidth: true
            onClicked: chimeraFleet.requestCreate(sourceSurvey, Number(count.currentText), Number(minimum.text), Number(maximum.text), 0)
        }
    }
}
