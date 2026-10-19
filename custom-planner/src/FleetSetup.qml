import QtQuick
import QtQuick.Layouts
import QGroundControl.Controls

ColumnLayout {
    property var sourceSurvey
    spacing: 6
    SectionHeader { id: header; text: qsTr("Fleet setup"); checked: false; Layout.fillWidth: true }
    ColumnLayout {
        visible: header.checked
        Layout.fillWidth: true
        QGCLabel { Layout.fillWidth: true; wrapMode: Text.WordWrap; text: qsTr("Use one master survey and one shared fiducial. Each drone visits every region, starting in its own region. Drone missions keep independent camera settings and inspection paths.") }
        RowLayout {
            QGCLabel { text: qsTr("Drones"); Layout.fillWidth: true }
            QGCComboBox { id: count; model: [2,3,4,5,6,7,8]; Layout.preferredWidth: 90 }
        }
        RowLayout {
            QGCLabel { text: qsTr("Min altitude (m)"); Layout.fillWidth: true; wrapMode: Text.WordWrap }
            QGCTextField { id: minimum; objectName: "fleetMinimum"; text: "20"; Layout.preferredWidth: 90; validator: DoubleValidator { bottom: 0.1 } }
        }
        RowLayout {
            QGCLabel { text: qsTr("Max altitude (m)"); Layout.fillWidth: true; wrapMode: Text.WordWrap }
            QGCTextField { id: maximum; objectName: "fleetMaximum"; text: "30"; Layout.preferredWidth: 90; validator: DoubleValidator { bottom: 0.1 } }
        }
        QGCLabel { Layout.fillWidth: true; wrapMode: Text.WordWrap; text: qsTr("Altitude layers assume all launch sites share the master home elevation. Takeoff and landing are not sequenced.") }
        QGCButton {
            objectName: "fleetCreate"; text: qsTr("Create drone missions"); Layout.fillWidth: true
            onClicked: chimeraFleet.requestCreate(sourceSurvey, Number(count.currentText), Number(minimum.text), Number(maximum.text), 0)
        }
    }
}
