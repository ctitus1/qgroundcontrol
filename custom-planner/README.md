# Chimera mission planner

The `custom` link selects `custom-planner`. The plugin keeps the standard QGC interface and adds native mission blocks. The old `custom-example` remains as reference code.

## Use

1. Add a takeoff. In **Pattern**, select **Chimera fiducial**. Set latitude, longitude, altitude above home, aircraft heading, and **Delay until (s)** in its panel. Drag its map marker to move it. The delay uses `MAV_CMD_NAV_DELAY`, separate from waypoint hold time. Use a delay greater than 1 second; 0 and 1 are reserved markers.
2. Add **Chimera survey**. Edit the area polygon on the map. Set aircraft heading, gimbal pitch, speed, track angle, lateral overlap, and calibrated camera intrinsics in the panel. The block includes mission arm (`NAV_DELAY(0)`), pitch, speed, and fixed waypoint headings.
3. Select a numbered route point in the panel and press **Add ROI**. Edit the ROI target and inspection waypoint. Add further waypoints or ROI targets in execution order. The block inserts the return to that checkpoint and restores survey heading, altitude, pitch, speed, and camera triggering.
4. Add **Chimera vantage** after the survey for assessment. Set its position, heading, and delay until. It includes the assessment marker (`NAV_DELAY(1)`).
5. Save and upload the native plan through QGC. Each block stays one mission-list item. Its generated commands expand only for upload. Use the panel centre fields or map centre handle to move a survey and its inspection paths together.

Use separate plans for each drone with QGC's normal vehicle selection. Downloading a mission from an aircraft returns MAVLink commands; it cannot recover the source blocks. Keep the saved native plan.

## Survey and inspection rules

The polygon defines the required ground area. Cyan shows the aircraft route, including inspection paths. Orange shows sample camera footprints. The aircraft path is offset from the area to account for aircraft yaw and gimbal pitch. Pitch -90 degrees points down. Review numbered checkpoint positions after any change to survey geometry. Checkpoints are route-point indices; changing the route can change their locations.

The camera model takes rectified image dimensions and `fx`, `fy`, `cx`, and `cy` in pixels. Initial values are examples. Use the calibration for the selected camera, zoom, and crop. Corner rays intersect flat ground at home height. A rectangle aligned with the tracks fits inside the projected footprint. Its width and lateral overlap set track spacing. Capture spacing uses its length with fixed 70% forward overlap. QGC generates the survey legs and trigger commands.

An inspection stops camera triggering, applies the user-defined ROI targets and waypoints, clears ROI, and returns to its checkpoint. It then restores survey settings before continuing. ROI altitude means height above target terrain in the Chimera runtime. Inspection waypoint altitude is relative to home. Inspection points remain fixed when individual polygon vertices change; moving the whole survey moves them with it.

Fiducials must precede the first search or assessment marker. Upload rejects invalid block settings and misplaced fiducials. Sample footprints do not prove full boundary coverage. Review turns, area edges, and inspection transitions.

## Scope

- Relative altitude and flat ground only. No terrain coverage or obstacle clearance checks.
- Aircraft yaw controls optical bearing. No independent gimbal yaw or lens distortion model.
- Inspection paths support editable waypoints and multiple ROI targets. Arbitrary nested QGC commands and inspection-point drag handles are not included.
- Native flight-path segments support QGC's existing altitude display. No PX4 scene importer or embedded Gazebo view is added.
- No fleet scheduler, collision avoidance between drones, or new undo system.
- Earlier offset-survey source files retain their export controls. New missions should use the native Chimera patterns.

## Build and focused checks

```sh
cmake -S . -B build -DCMAKE_PREFIX_PATH=/home/user/Qt/6.8.3/gcc_64
cmake --build build -j 12
```

`tests/coverage_test.cc` checks camera geometry, lateral overlap, rotation, rectangle containment, and invalid rays. Enable `QGC_PLANNER_SMOKE_TEST` at build time and set `QGC_PLANNER_SMOKE=1` at launch for the offline function check. It checks native blocks, command order, delay semantics, offset routes, ROI return, grouped movement, save/load, and QML loading. Use an isolated configuration with vehicle connections disabled. These checks do not test aircraft execution.
