# Chimera mission planner

The `custom` link selects `custom-planner`. The plugin keeps the standard QGC interface and adds native mission blocks. The old `custom-example` remains as reference code.

## Use

1. Add a takeoff. In **Pattern**, select **Chimera fiducial**. Set latitude, longitude, altitude above home, aircraft heading, and **Delay until (s)** in its panel. Drag its map marker to move it. The delay uses `MAV_CMD_NAV_DELAY`, separate from waypoint hold time. Use a delay greater than 1 second; 0 and 1 are reserved markers.
2. Add **Chimera survey**. Edit the area polygon on the map. Set aircraft heading, gimbal pitch, speed, track angle, lateral overlap, and calibrated camera intrinsics in the panel. The block includes mission arm (`NAV_DELAY(0)`), pitch, speed, and fixed waypoint headings.
3. Set the survey geometry, then save a copy. Use **Split along flight path** or click a route marker to choose a split point. Press **Split survey here**. The first part stays selected. Add normal QGC ROI commands, inspection waypoints, or other commands after it, before the continuation. The continuation returns to the split point before it restores pitch, mission arm, and camera triggering. Both parts retain the survey heading and speed.
4. Add **Chimera vantage** after the survey for assessment. Set its position, heading, and delay until. Its first command is the assessment marker (`NAV_DELAY(1)`). Fiducial and vantage blocks emit no gimbal mount commands; `5g_drone` controls their gimbal pointing.
5. Save and upload the native plan through QGC. Each block stays one mission-list item. Its generated commands expand only for upload. Use the panel centre fields to move a whole survey part. Unsplit surveys also have a map centre handle.

Use separate plans for each drone with QGC's normal vehicle selection. Downloading a mission from an aircraft returns MAVLink commands; it cannot recover the source blocks. Keep the saved native plan.

## Survey and inspection rules

The polygon defines the required ground area. White lines and standard QGC index markers show the aircraft route. Orange shows sample camera footprints. The aircraft path is offset from the area to account for aircraft yaw and gimbal pitch. Pitch -90 degrees points down. The split slider can select a point within a flight leg. Each split part stores its route, so later edits cannot move the shared endpoint by regenerating the grid. Geometry and camera fields are read-only after a split; speed and whole-part position remain editable. Use the saved unsplit plan to change coverage geometry. A part can be split again. Moving one part does not move the other part or the manual inspection items.

The camera model takes rectified image dimensions and `fx`, `fy`, `cx`, and `cy` in pixels. Initial values are examples. Use the calibration for the selected camera, zoom, and crop. Corner rays intersect flat ground at home height. A rectangle aligned with the tracks fits inside the projected footprint. Its width and lateral overlap set track spacing. Capture spacing uses its length with fixed 70% forward overlap. QGC generates the survey legs and trigger commands.

The first part stops camera triggering at the split. Insert inspection commands with the normal QGC editor. The continuation stops triggering and clears ROI before returning to the split point, then restores survey settings. ROI altitude means height above target terrain in the Chimera runtime. Inspection waypoint altitude follows the normal QGC altitude setting.

Existing embedded inspections from older plans still load and execute. The split panel provides a button to remove them before splitting. Save a copy before removal.

Fiducials must precede the first search or assessment marker. Upload rejects invalid block settings and misplaced fiducials. Sample footprints do not prove full boundary coverage. Review turns, area edges, and inspection transitions.

## Scope

- Relative altitude and flat ground only. No terrain coverage or obstacle clearance checks.
- Aircraft yaw controls optical bearing. No independent gimbal yaw or lens distortion model.
- Inspection commands are normal QGC items between the split parts. They do not move with either part. The shared source area remains visible for reference; split parts do not recalculate coverage.
- Native flight-path segments support QGC's existing altitude display. No PX4 scene importer or embedded Gazebo view is added.
- No fleet scheduler, collision avoidance between drones, or new undo system.
- Earlier offset-survey source files retain their export controls. New missions should use the native Chimera patterns.

## Build and focused checks

```sh
cmake -S . -B build -DCMAKE_PREFIX_PATH=/home/user/Qt/6.8.3/gcc_64
cmake --build build -j 12
```

`tests/coverage_test.cc` checks camera geometry, lateral overlap, rotation, rectangle containment, and invalid rays. Enable `QGC_PLANNER_SMOKE_TEST` at build time and set `QGC_PLANNER_SMOKE=1` at launch for the offline function check. It checks native blocks, command order, no-gimbal fiducial and vantage behavior, offset routes, split boundaries, manual item insertion, save/load, and QML loading. It renders all three panels at 240 pixels wide and checks that each input fits. Use an isolated configuration with vehicle connections disabled. These checks do not test aircraft execution.
