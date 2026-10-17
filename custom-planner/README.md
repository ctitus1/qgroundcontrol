# Chimera mission planner: first draft

This plugin uses the standard QGC interface. The tracked `custom` symlink selects `custom-planner`. The old `custom-example` plugin remains as reference code.

## Use

1. Create a normal mission with a takeoff and a Survey polygon. The polygon defines the required ground area.
2. In the Survey transect panel, enable **Chimera offset survey**. Enter calibrated intrinsics, aircraft yaw, gimbal pitch, and overlap. Set the altitude and track angle with the standard survey controls.
3. Save the source plan. Select **Export editable mission**, then open the exported `.plan` file. The export expands all enabled surveys and preserves the other mission items.
4. Select an ordinary survey waypoint. In the Pattern menu, select **ROI inspection and return**. Edit the ROI target and inspection waypoint. Insert additional maneuver waypoints before ROI clear. Keep the return waypoint and restored camera settings after the inspection.
5. Use the Pattern menu to add a fiducial hold before search or a vantage assessment after search. Edit the default 60-second holds as needed. Review and upload the exported mission to the selected vehicle.

Use separate plans for each drone with QGC's normal vehicle selection. The standard 3D viewer can show exported missions. This draft does not add a fleet scheduler or a shared fleet project.

## Geometry

The camera model uses rectified image dimensions and `fx`, `fy`, `cx`, and `cy` in pixels. The initial values are examples. Replace them with the calibration for the selected camera, zoom, and image crop. The optical bearing follows aircraft yaw. Gimbal pitch uses the runtime convention: -90 degrees points straight down.

Corner rays intersect a horizontal ground plane. The planner fits a track-aligned square inside the resulting trapezoid. That square sets conservative track and capture spacing. Its center sets the offset between the required ground area and the aircraft path. QGC generates the flight legs and camera trigger commands.

The orange polygons are sampled footprints, not a coverage certificate. Cyan shows the path in the exported mission. The source polygon remains visible for editing. The cyan path replaces the native grid preview. An enabled source plan cannot upload through the normal Plan upload action. Export it first.

## Inspection behavior

An inspection inserts camera-trigger stop, ROI location, an editable waypoint, ROI clear, return waypoint, gimbal pitch, search marker, settling waypoint, and camera-trigger restoration. Return waypoints retain the selected aircraft yaw and altitude. The initial maneuver waypoint duplicates the checkpoint so the operator can place it.

The return checkpoint is an ordinary waypoint. It does not move automatically after later edits to the survey or inspection. Regenerate the export when survey geometry changes, then add the inspection again.

The generated commands use the existing Chimera protocol: positive pre-region delays for fiducials, `NAV_DELAY(0)` for search, and `NAV_DELAY(1)` for assessment. ROI altitude means height above target terrain in the Chimera runtime. Review that value even though QGC uses its normal altitude editor.

## Limits

- Flat ground at home height and relative altitude only. No terrain-aware coverage or obstacle clearance checks.
- Rectified pinhole camera with body-follow bearing. No lens distortion model or independent gimbal yaw.
- Constant yaw requires suitable multirotor firmware behavior. No flight or simulator execution test was part of this draft.
- No automatic coverage-gap proof, fleet deconfliction, linked survey undo, or simulation-scene importer.
- Mission jumps block export because expansion changes sequence references.

## Build and focused checks

Build from the repository root with the existing Qt installation:

```sh
cmake -S . -B build -DCMAKE_PREFIX_PATH=/home/user/Qt/6.8.3/gcc_64
cmake --build build -j 12
```

`tests/coverage_test.cc` checks nadir geometry, overlap, yaw rotation, the conservative square, and invalid rays. The optional `QGC_PLANNER_SMOKE_TEST` build option enables the offline function check. Set `QGC_PLANNER_SMOKE=1` when starting that build to check preview, export, reload, ROI insertion, settings persistence, and QML components. Use a temporary configuration directory for the check.
