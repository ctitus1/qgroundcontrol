import QtQuick
import QtQuick.Layouts
import QGroundControl.Controls

ColumnLayout {
    property var sourceSurvey
    property string error: ""
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
            QGCTextField { id: minimum; text: sourceSurvey ? sourceSurvey.values.altitude : "30"; Layout.preferredWidth: 90; validator: DoubleValidator { bottom: 0.1 } }
        }
        RowLayout {
            QGCLabel { text: qsTr("Max altitude (m)"); Layout.fillWidth: true; wrapMode: Text.WordWrap }
            QGCTextField { id: maximum; text: String(Number(minimum.text) + 10 * (count.currentIndex+1)); Layout.preferredWidth: 90; validator: DoubleValidator { bottom: 0.1 } }
        }
        QGCLabel { Layout.fillWidth: true; wrapMode: Text.WordWrap; text: qsTr("Altitude layers assume all launch sites share the master home elevation. Takeoff and landing are not sequenced.") }
        QGCButton {
            text: qsTr("Create drone missions"); Layout.fillWidth: true
            onClicked: error = chimeraFleet.create(sourceSurvey, Number(count.currentText), Number(minimum.text), Number(maximum.text), 0)
        }
        QGCLabel { text: error; visible: text.length > 0; Layout.fillWidth: true; wrapMode: Text.WordWrap; color: "orange" }
    }
}
