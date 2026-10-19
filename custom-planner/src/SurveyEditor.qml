import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QGroundControl
import QGroundControl.Controls

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
        text: qsTr("Export mission files…")
        onClicked: outputDialog.openForSave()
    }
    QGCLabel {
        Layout.fillWidth: true
        visible: root.values.enabled === true
        wrapMode: Text.WordWrap
        text: qsTr("Export saves the editable mission and a -plain.plan copy for flight. Export includes the whole mission.")
    }
    QGCFileDialog {
        id: outputDialog
        title: qsTr("Export mission files")
        folder: QGroundControl.settingsManager.appSettings.missionSavePath
        onAcceptedForSave: (file) => {
            var error = chimeraPlanner.exportMission(root.survey, file)
            QGroundControl.showMessageDialog(root, qsTr("Chimera planner"), error || qsTr("Mission files exported. Open the -plain.plan file for flight."))
            close()
        }
    }
}
