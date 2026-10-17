import QtQuick
import QtQuick.Layouts
import QGroundControl.Controls

ColumnLayout {
    property var controller: null
    QGCLabel { text: qsTr("Chimera: insert after selected waypoint") }
    Repeater {
        model: [
            {kind: "fiducial", label: qsTr("Fiducial hold (60 s)")},
            {kind: "inspection", label: qsTr("ROI inspection and return")},
            {kind: "assessment", label: qsTr("Vantage assessment (60 s)")}
        ]
        delegate: QGCButton {
            required property var modelData
            Layout.fillWidth: true
            text: modelData.label
            onClicked: {
                var error = chimeraPlanner.insertBlock(controller, modelData.kind)
                mainWindow.showMessageDialog(qsTr("Chimera planner"), error ||
                    qsTr("Block inserted. Edit its target, waypoints, and hold duration with the normal mission controls. Keep the ROI clear and return settings after your inspection waypoints."))
            }
        }
    }
}
