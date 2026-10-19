import QtQuick
import QtQuick.Layouts
import QGroundControl.ScreenTools
import QGroundControl.Controls

RowLayout {
    id: root
    property var fit
    QGCComboBox {
        model: [qsTr("Master")].concat(chimeraFleet.drones.map(d => d.name))
        currentIndex: chimeraFleet.selected
        onActivated: index => { chimeraFleet.selected = index; if (root.fit) root.fit() }
        Layout.preferredWidth: ScreenTools.defaultFontPixelWidth * 18
    }
}
