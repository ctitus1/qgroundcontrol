
/*
 * Mission editor UI for custom surveys.
 *
 * Displays and edits per-survey region settings.
 */

import QtQuick
import QtQuick.Controls
import QtQuick.Dialogs
import QtQuick.Layouts

import QGroundControl
import QGroundControl.ScreenTools
import QGroundControl.Vehicle
import QGroundControl.Controls
import QGroundControl.FactSystem
import QGroundControl.FactControls
import QGroundControl.Palette
import QGroundControl.FlightMap

TransectStyleComplexItemEditor {
    transectAreaDefinitionComplete: missionItem.surveyAreaPolygon.isValid
    transectAreaDefinitionHelp:     qsTr("Use the Polygon Tools to create the polygon which outlines your survey area.")
    transectValuesHeaderName:       qsTr("Transects")
    transectValuesComponent:        _transectValuesComponent
    presetsTransectValuesComponent: _transectValuesComponent

    // The following properties must be available up the hierarchy chain
    //  property real   availableWidth    ///< Width for control
    //  property var    missionItem       ///< Mission Item for editor

    property real   _margin:        ScreenTools.defaultFontPixelWidth / 2
    property var    _missionItem:   missionItem
    property bool   _customSurvey:  customSurveyManager.isCustomSurvey(missionItem)
    property var    _customRegions: []

    function _refreshCustomRegions() {
        _customSurvey = customSurveyManager.isCustomSurvey(missionItem)
        _customRegions = _customSurvey ? customSurveyManager.regionPolygons(missionItem) : []
    }

    Component.onCompleted: _refreshCustomRegions()


        


    Connections {
        target: missionItem.surveyAreaPolygon

        function onPathChanged() {
            _refreshCustomRegions()
        }
    }

    Connections {
        target: customSurveyManager

        function onCustomSurveyChanged(item) {
            if (item === missionItem) {
                _refreshCustomRegions()
            }
        }
    }


    Connections {
        target: customSurveyManager

        function onRegionCountChanged() {
            _refreshCustomRegions()

            regionCountSpinBox.value =
                customSurveyManager.regionCountForSurvey(
                    missionItem)
        }
    }

    Component {
        id: _transectValuesComponent

        GridLayout {
            Layout.fillWidth:   true
            columnSpacing:      _margin
            rowSpacing:         _margin
            columns:            2

            QGCLabel { text: qsTr("Angle") }
            FactTextField {
                fact:                   missionItem.gridAngle
                Layout.fillWidth:       true
                onUpdated:              angleSlider.value = missionItem.gridAngle.value
            }

            QGCSlider {
                id:                     angleSlider
                from: 1
                to:           359
                stepSize:               1
                tickmarksEnabled:       false
                Layout.fillWidth:       true
                Layout.columnSpan:      2
                Layout.preferredHeight: ScreenTools.defaultFontPixelHeight * 1.5
                onValueChanged:         missionItem.gridAngle.value = value
                Component.onCompleted:  value = missionItem.gridAngle.value
                live: true
            }

            QGCLabel {
                text:       qsTr("Turnaround dist")
                visible:    !forPresets
            }
            FactTextField {
                Layout.fillWidth:   true
                fact:               missionItem.turnAroundDistance
                visible:            !forPresets
            }

            QGCOptionsComboBox {
                Layout.columnSpan:  2
                Layout.fillWidth:   true
                visible:            !forPresets

                model: [
                    {
                        text:       qsTr("Hover and capture image"),
                        fact:       missionItem.hoverAndCapture,
                        enabled:    missionItem.cameraCalc.distanceMode === QGroundControl.AltitudeModeRelative || missionItem.cameraCalc.distanceMode === QGroundControl.AltitudeModeAbsolute,
                        visible:    missionItem.hoverAndCaptureAllowed
                    },
                    {
                        text:       qsTr("Refly at 90 deg offset"),
                        fact:       missionItem.refly90Degrees,
                        enabled:    missionItem.cameraCalc.distanceMode !== QGroundControl.AltitudeModeCalcAboveTerrain,
                        visible:    true
                    },
                    {
                        text:       qsTr("Images in turnarounds"),
                        fact:       missionItem.cameraTriggerInTurnAround,
                        enabled:    missionItem.hoverAndCaptureAllowed ? !missionItem.hoverAndCapture.rawValue : true,
                        visible:    true
                    },
                    {
                        text:       qsTr("Fly alternate transects"),
                        fact:       missionItem.flyAlternateTransects,
                        enabled:    true,
                        visible:    _vehicle ? (_vehicle.fixedWing || _vehicle.vtol) : false
                    }
                ]
            }

            SectionHeader {
                id:                 customRegionsHeader
                Layout.columnSpan:  2
                Layout.fillWidth:   true
                text:               qsTr("Custom Regions")
                visible:            _customSurvey && !forPresets
            }

            ColumnLayout {

            RowLayout {
                spacing: ScreenTools.defaultFontPixelWidth

                QGCLabel {
                    text: "Regions"
                }

                SpinBox {
                    id: regionCountSpinBox
                    from: 1
                    to: 20
                    value: customSurveyManager.regionCountForSurvey(missionItem)
                    onValueChanged: customSurveyManager.setRegionCountForSurvey(missionItem, value)
                }
            }

                Layout.columnSpan:  2
                Layout.fillWidth:   true
                spacing:            _margin
                visible:            customRegionsHeader.visible && customRegionsHeader.checked

                Repeater {
                    model: _customRegions

                    RowLayout {
                        Layout.fillWidth: true

                        QGCLabel {
                            Layout.fillWidth: true
                            text: modelData.name
                        }

                        QGCLabel {
                            text: QGroundControl.unitsConversion.squareMetersToAppSettingsAreaUnits(modelData.area).toFixed(2) + " " + QGroundControl.unitsConversion.appSettingsAreaUnitsString
                        }

                        QGCLabel {
                            text: qsTr("%1 pts").arg(modelData.vertexCount)
                        }
                    }
                }

                QGCLabel {
                    Layout.fillWidth:   true
                    wrapMode:           Text.WordWrap
                    color:              QGroundControl.globalPalette.warningText
                    text:               customSurveyManager.lastError
                    visible:            _customRegions.length === 0 && customSurveyManager.lastError !== ""
                }

                QGCButton {
                    Layout.alignment:   Qt.AlignHCenter
                    text:               qsTr("Save Region Plans")
                    enabled:            missionItem.surveyAreaPolygon.isValid && _customRegions.length > 0

                    onClicked:          regionFolderDialog.openForLoad()
                }
            }
        }
    }

    QGCFileDialog {
        id:             regionFolderDialog
        folder:         QGroundControl.settingsManager.appSettings.missionSavePath
        title:          qsTr("Select Output Folder")
        selectFolder:   true

        onAcceptedForLoad: (folder) => {
            customSurveyManager.saveRegionPlans(missionItem.masterController, missionItem, folder)
            mainWindow.showMessageDialog(qsTr("Custom Regions"), customSurveyManager.lastError)
            _refreshCustomRegions()
            close()
        }
    }

    KMLOrSHPFileDialog {
        id:             kmlOrSHPLoadDialog
        title:          qsTr("Select Polygon File")

        onAcceptedForLoad: (file) => {
            missionItem.surveyAreaPolygon.loadKMLOrSHPFile(file)
            missionItem.resetState = false
            //editorMap.mapFitFunctions.fitMapViewportTomissionItems()
            close()
        }
    }
}

// ============================================================
// PATCH TODO
//
// Migrate UI to QObject*-based runtime identity.
//
// regionCountForSurvey(missionItem)
//
// should become the single source of truth.
//
// After PATCH D:
//
// remove any compatibility logic.
//
// ============================================================

