import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QGroundControl
import QGroundControl.Controls
import QGroundControl.ScreenTools

ColumnLayout {
    id: root
    property var survey: null
    property var values: ({})
    property var result: ({})
    spacing: ScreenTools.defaultFontPixelHeight / 2

    function refresh() {
        if (!survey) return
        values = chimeraPlanner.settings(survey)
        result = chimeraPlanner.preview(survey)
    }
    onSurveyChanged: refresh()
    Connections {
        target: chimeraPlanner
        function onSettingsChanged(item) { if (item === root.survey) root.refresh() }
    }
    Connections {
        target: root.survey
        function onVisualTransectPointsChanged() { refreshTimer.restart() }
    }
    Timer { id: refreshTimer; interval: 150; onTriggered: root.refresh() }

    QGCCheckBox {
        text: qsTr("Chimera offset survey")
        checked: root.values.enabled === true
        onClicked: chimeraPlanner.setSetting(root.survey, "enabled", checked)
    }
    QGCLabel {
        Layout.fillWidth: true
        visible: root.values.enabled === true
        wrapMode: Text.WordWrap
        text: qsTr("The polygon is the ground area to cover. Cyan shows the export path. Orange shows sample camera footprints. Use calibrated, rectified intrinsics. Flat ground at home height only.")
    }
    GridLayout {
        Layout.fillWidth: true
        visible: root.values.enabled === true
        columns: 2
        Repeater {
            model: [
                {key: "yaw", label: qsTr("Aircraft yaw (degrees)")},
                {key: "pitch", label: qsTr("Gimbal pitch (-90 = down)")},
                {key: "overlap", label: qsTr("Overlap (%)")},
                {key: "width", label: qsTr("Image width (px)")},
                {key: "height", label: qsTr("Image height (px)")},
                {key: "fx", label: qsTr("Focal fx (px)")},
                {key: "fy", label: qsTr("Focal fy (px)")},
                {key: "cx", label: qsTr("Center cx (px)")},
                {key: "cy", label: qsTr("Center cy (px)")}
            ]
            delegate: RowLayout {
                required property var modelData
                Layout.columnSpan: 2
                Layout.fillWidth: true
                QGCLabel { text: modelData.label; Layout.fillWidth: true }
                QGCTextField {
                    Layout.preferredWidth: ScreenTools.defaultFontPixelWidth * 10
                    text: root.values[modelData.key] === undefined ? "" : String(root.values[modelData.key])
                    validator: DoubleValidator { locale: "C" }
                    onEditingFinished: {
                        var number = Number(text)
                        if (isFinite(number) && text.trim() !== "")
                            chimeraPlanner.setSetting(root.survey, modelData.key, number)
                        else root.refresh()
                    }
                }
            }
        }
    }
    QGCLabel {
        Layout.fillWidth: true
        visible: root.values.enabled === true
        wrapMode: Text.WordWrap
        text: root.result.error || (root.result.spacing === undefined ? "" :
            qsTr("Track/capture spacing: %1 m. Footprint offset: %2 m.")
                .arg(root.result.spacing.toFixed(1)).arg(root.result.offset.toFixed(1)))
    }
    QGCButton {
        Layout.fillWidth: true
        visible: root.values.enabled === true
        enabled: root.result.path !== undefined && !root.result.error
        text: qsTr("Export editable mission…")
        onClicked: outputDialog.openForSave()
    }
    QGCLabel {
        Layout.fillWidth: true
        visible: root.values.enabled === true
        wrapMode: Text.WordWrap
        text: qsTr("Save this survey source, then export and open the waypoint copy for flight. Add inspections from the Pattern menu. Export includes all enabled surveys in this mission.")
    }
    QGCFileDialog {
        id: outputDialog
        title: qsTr("Export editable mission")
        folder: QGroundControl.settingsManager.appSettings.missionSavePath
        onAcceptedForSave: (file) => {
            var error = chimeraPlanner.exportPlan(root.survey, file)
            mainWindow.showMessageDialog(qsTr("Chimera planner"), error || qsTr("Mission exported. Open the new .plan file to edit inspection waypoints and upload."))
            close()
        }
    }
}
