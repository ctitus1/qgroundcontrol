# Chimera mission planner

The `custom` link selects `custom-planner`. The plugin keeps the standard QGC interface and adds native mission blocks. The old `custom-example` remains as reference code.

## Use

1. Add a takeoff and **Chimera fiducial** from the Pattern menu. Set position, altitude, heading, and **Delay until**. The delay uses `NAV_DELAY`, separate from waypoint hold time. Values 0 and 1 are reserved autonomy markers.
2. Add **Chimera survey**. Edit its area polygon with the normal QGC polygon controls. Set altitude, heading, gimbal pitch, speed, track angle, and side overlap. Expand **Camera intrinsics** to enter the camera calibration. The panels use QGC Fact fields, units, validation, and collapsible sections.
3. Select a survey part. Click a **+** handle on a flight leg to add a split point. Drag the split marker to move it along the route. Use its menu or **Remove end split** to remove it. Insert normal QGC ROI commands and inspection waypoints between the resulting mission-list items.
4. Edit the full survey from any linked part. Changes to its polygon, position, altitude, heading, speed, overlap, or camera settings update all parts. Splits divide the regenerated route by relative distance along that route. Review their positions after changing the geometry.
5. Add **Chimera vantage** after the survey. Its first command is `NAV_DELAY(1)` to arm vantage autonomy. Fiducial and vantage blocks emit no gimbal or ROI commands; `5g_drone` controls their gimbal pointing.
6. Save and upload the native plan through QGC. Each part stays one mission-list item. Commands expand for upload. Use separate plans with QGC's normal vehicle selection for multiple drones.

Removing a split joins adjacent survey parts. If manual mission items lie between them, move or remove those items first. They are never deleted or reordered by split removal. Removing the last split restores the full editable survey. Deleting a linked part through QGC's normal delete action leaves a gap; upload rejects an incomplete group.

## Shared survey source

Linked parts keep the complete source polygon and camera settings, a group ID, and their start/end fractions. They do not store fixed flight routes. Editing any part updates the source settings on the other parts. Saving preserves both the source and the split positions. Manual inspection items remain independent and do not move when the survey moves.

The first part stops camera triggering at the split. The continuation clears ROI, returns to the shared split point with the survey heading and altitude, and restores speed, gimbal pitch, mission arm (`NAV_DELAY(0)`), and camera triggering.

Version 2 plans from the earlier draft stored fixed routes. When all matching parts are present and still agree with their original source, loading the plan restores a linked editable survey. **Restore editable survey** can also retry this conversion. Parts with missing or changed source data stay in the old fixed-route form; use the original unsplit plan to rebuild those surveys. Existing embedded inspections still load; remove them before adding split points.

Keep the saved native plan. Downloading from an aircraft returns MAVLink commands and cannot recover the source groups.

## Geometry and limits

The polygon defines the required ground area. The selected survey part is cyan with a thicker line; other parts are white. Orange outlines show sample camera footprints. The camera model uses rectified image dimensions and `fx`, `fy`, `cx`, and `cy` in pixels. Initial values are examples, not calibration data. Optical bearing follows aircraft yaw; pitch -90 degrees points down.

Corner rays intersect flat ground at home height. A rectangle aligned with the tracks fits inside the footprint. Its width and side overlap set track spacing. Capture spacing uses its length with fixed 70% forward overlap. QGC generates the flight legs and trigger commands. Sample footprints do not prove full boundary coverage.

- Relative altitude and flat ground only. No terrain coverage or obstacle clearance checks.
- No independent gimbal yaw, lens distortion model, fleet scheduler, or new undo system.
- ROI altitude means height above target terrain in the Chimera runtime. Manual waypoint altitude uses the normal QGC setting.
- Native flight-path segments support QGC's altitude display. No PX4 scene importer or embedded Gazebo view is added.
- Earlier stock offset-survey source files retain their export controls. Use native Chimera patterns for new missions.

## Build and focused checks

```sh
cmake -S . -B build -DCMAKE_PREFIX_PATH=/home/user/Qt/6.8.3/gcc_64
cmake --build build -j 12
```

Enable `QGC_PLANNER_SMOKE_TEST` at build time and set `QGC_PLANNER_SMOKE=1` at launch for the offline function check. Use an isolated configuration with vehicle connections disabled. The check covers command order, source edits across linked parts, split movement and removal, manual item preservation, persistence, Fact field widths, and map controls. It renders each panel at 240 pixels wide. These checks do not test aircraft execution.

Center drags move the existing map paths and rebuild flight commands on release. Resizing reuses the survey generator and map objects; previews use the generated route.
