# Chimera mission planner

The `custom` link selects `custom-planner`. The plugin keeps the standard QGC interface and adds native mission blocks. The old `custom-example` remains as reference code.

## Use

1. Add a takeoff and **Chimera fiducial** from the Pattern menu. Set position, altitude, heading, and **Delay until**. The delay uses `NAV_DELAY`, separate from waypoint hold time. Values 0 and 1 are reserved autonomy markers.
2. Add **Chimera survey**. Edit its area polygon with the normal QGC polygon controls. Set altitude, heading, gimbal pitch, speed, track angle, and side overlap. Expand **Camera intrinsics** to enter the camera calibration. The panels use QGC Fact fields, units, validation, and collapsible sections.
3. Select a survey part. Click a **+** handle on a flight leg to add a split point. Drag the split marker to move it along the route. Use its menu or **Remove end split** to remove it. Insert normal QGC ROI commands and inspection waypoints between the resulting mission-list items.
4. Edit the full survey from any linked part. Changes to its polygon, position, altitude, heading, speed, overlap, or camera settings update all parts. Splits divide the regenerated route by relative distance along that route. Review their positions after changing the geometry.
5. Add **Chimera vantage** after the survey. Its first command is `NAV_DELAY(1)` to arm vantage autonomy. Fiducial and vantage blocks emit no gimbal or ROI commands; `5g_drone` controls their gimbal pointing.
6. Save and upload the native plan through QGC. Each part stays one mission-list item. Commands expand for upload. For coordinated planning, use the fleet workflow below.

Removing a split joins adjacent survey parts. If manual mission items lie between them, move or remove those items first. They are never deleted or reordered by split removal. Removing the last split restores the full editable survey. Deleting a linked part through QGC's normal delete action leaves a gap; upload rejects an incomplete group.

## Shared survey source

Linked parts keep the complete source polygon and camera settings, a group ID, and their start/end fractions. They do not store fixed flight routes. Editing any part updates the source settings on the other parts. Saving preserves both the source and the split positions. Manual inspection items remain independent and do not move when the survey moves.

The first part stops camera triggering at the split. The continuation clears ROI, returns to the shared split point with the survey heading and altitude, and restores speed, gimbal pitch, mission arm (`NAV_DELAY(0)`), and camera triggering.

Version 2 plans from the earlier draft stored fixed routes. When all matching parts are present and still agree with their original source, loading the plan restores a linked editable survey. **Restore editable survey** can also retry this conversion. Parts with missing or changed source data stay in the old fixed-route form; use the original unsplit plan to rebuild those surveys. Existing embedded inspections still load; remove them before adding split points.

Keep the saved native plan. Downloading from an aircraft returns MAVLink commands and cannot recover the source groups.

## Geometry and limits

The polygon defines the required ground area. The selected survey part is cyan with a thicker line; other parts are white. Orange outlines show sample camera footprints. The pinhole camera model uses image dimensions and `fx`, `fy`, `cx`, and `cy` in pixels. Initial values are examples, not calibration data. Optical bearing follows aircraft yaw; pitch -90 degrees points down.

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

New blocks use QGC's default flight altitude and a speed of 1 m/s. New surveys use a gimbal pitch of -70 degrees and 25% side overlap. The slider and numeric angle fields change heading and track angle together. Loading an older plan preserves its saved angles until either is edited.

The camera list contains Custom and eight named aircraft/camera presets: rgb_mid and thermal for UAS1 and UAS2, and pilot and thermal for UAS3 and UAS4. Selecting a preset applies all six intrinsic values at once. Editing an intrinsic selects Custom. The preset and numeric values are saved with the plan and shared by linked survey parts.

`CameraPresets.json` contains the ROS calibration fields from `5g_drone/config/camera_calibration_files/uas*-*.yaml` (local source snapshot, 2026-10-17). Planning uses `image_width`, `image_height`, and `camera_matrix.data` (K), not the projection matrix. The source distortion and rectification fields are retained for reference; coverage still uses the pinhole model. Source files marked as borrowed placeholders have a visible note in the selector panel. Presets are bundled and do not change when the external source files change.


## Fleet planning

1. In the master, draw one convex Chimera survey and add one Chimera fiducial. The master survey supplies the shared area and initial flight defaults. Each aircraft gets its day camera preset automatically. In its **Fleet setup** section, choose 2 to 8 multicopters and the altitude limits, then select **Create drone missions**. The **Drones** selector in the top **Master setup** panel remains editable after creation.
2. The top selector switches between **Master** and each drone's normal QGC mission editor. Master shows all routes in separate colors and numbered region boundaries. Each drone view keeps the other routes faintly visible. Select **Master setup** in the top bar to assign aircraft, starting regions, launch coordinates, fixed headings, cameras, speed, pitch, and overlap. Default headings are spaced by 360 degrees divided by drone count. Count changes adjust default headings and preserve headings you edited. Launch points initially use the master launch location; set each drone's location before export.
3. Drone 1 visits regions 1, 2, ... . Drone 2 visits 2, 3, ... , 1, and so on. Each keeps its own heading for every region. Each region has its own camera, speed, pitch, and overlap. Add split points and ordinary QGC ROI/inspection waypoints in the drone views. Shared-area edits preserve those splits and manual items. Manual inspection coordinates do not move with the area.
4. Altitudes are assigned from the minimum upward at equal spacing. **Maximum gap** can reduce the spacing; 0 leaves it uncapped. With a cap, the highest layer can be below the maximum altitude. The bar reports the assigned vertical gap. Survey, fiducial, takeoff, and manual waypoint altitudes use the assigned layer. Their home elevations use the master home elevation, so this requires launch sites on the same ground plane. ROI target altitude is not changed.
5. Normal **Save** and **Save As** save the whole master, including every drone's edited mission. Use **Export all** and enter a mission name. This saves `<name>.plan` as the master and `<name>-uasN.plan` for each active aircraft. Open that file in QGC, select the intended vehicle, and upload it normally. Direct upload from a fleet view is blocked because the master contains multiple missions.

Count changes reuse the old control-point splitter and keep the master area editable. Each active drone visits every active region. Each drone starts in its assigned region, then visits the remaining regions in cyclic order. Changing the start region swaps assignments and keeps survey groups intact. Region numbers identify saved groups of survey parts and the manual items that follow them. Split positions remain fractions of the region route. Camera settings and inspection items stay with their region. Manual inspection coordinates stay fixed when the region changes shape, so review those paths after a count or area change.

Removed drones and regions remain in the master file as inactive data. Increasing the count restores their edits, including after save and reload. Each aircraft export includes only its active mission. New regions use that drone’s defaults from the master setup. Shared geometry, launch points, altitude limits, and per-region settings remain editable. Counts range from 2 to 8.

Center-first routes retain complete QGC survey tracks. Each region starts at the shared partition center, where all sectors meet, and accounts for the camera offset. Tracks then run from nearest to farthest from that junction, with alternating flight directions. The aircraft heading stays fixed. Moving the yellow center handle changes the start reference for every region. The master file saves this reference. Existing master plans also use their partition center. Full tracks reach the boundary before the survey covers all inner ground. In each drone view, insert normal QGC ROI commands and inspection waypoints after the first survey item, before the next region's survey.

This is an offline planner, not a fleet flight scheduler. Drones advance to the next region independently. There is no runtime phase barrier, launch/landing sequence, collision check, or obstacle check. Altitude layers describe planned survey and inspection flight, not guaranteed separation during takeoff, RTL, or landing. Each generated mission ends with ordinary RTL, which can be edited in its drone view. Region boundaries use the old planner’s control-point splitter and convex-area checks.


Master setup selects day or night cameras for UAS1–4. Day uses rgb_mid for UAS1–2 and pilot for UAS3–4. Night uses thermal. A named camera override stays fixed when day/night changes. Custom intrinsics remain editable. UAS5–8 need custom calibration or an explicit preset choice. Changing a flight default applies that field to all regions for that drone. Camera changes apply the camera values without changing region speed or pitch overrides. **Apply defaults to every region** resets camera and flight overrides for that drone.

In Master view, the yellow handle moves the partition center. Blue handles change the boundary rays and snap to their midpoints on release. Colored sectors show starting-region assignments. **Show flight paths** hides routes when you need a clear sector view. Whole-area moves carry the partition controls with them. Boundary resizing leaves the controls fixed. Sector outlines update during a drag. Flight paths rebuild after a short pause or on release. The map reuses its objects and caches unchanged partitions. Changing drone count resets the rays to equal angles. **Reset equal sectors** also restores the partition center.

Use **Edit region / ROI** to open the assigned drone’s ordinary QGC editor at that region. Split points and manual inspection waypoints remain in that editor, with the other drones’ routes visible for context. The master file saves assignments, day/night mode, flight defaults, partition controls, and all drone missions.

**Rotate entry point** changes the entry direction without changing aircraft heading. Center-out surveys alternate between the two directions of the first track. Other surveys cycle through QGC's four entry locations. The setting applies to all linked parts and stays in saved plans. Split markers move during dragging, and the route updates when you release the marker.

Choose **Export all → Standard QGC plans (no plugin)** to save each drone as `<name>-uasN-plain.plan`. These files contain only standard mission commands and can open in QGC without the planner plugin. The export preserves the delay markers, headings, camera commands, ROI commands, and inspection waypoints. It also saves `<name>.plan` as the editable master. Choose **Editable plans** to keep custom blocks in each drone file. For one mission, use **Export standard QGC plan…** in its item panel. This exports the whole mission, not only the selected item.

Set **Fiducial radius** in Master setup or **Drone offset radius** in the master fiducial panel. The default is 2 m. Each drone waits behind the real fiducial along its survey heading, at its assigned survey altitude. Changes to the fiducial position, radius, heading, or altitude layers update these points.

**Rapid transit speed** defaults to 5 m/s. Set it in Master setup for all drones, or in a standalone survey panel. Plain exports add a transit to each survey entry, restore that survey's flight speed, and then send the 0-second mission arm marker. Split survey returns use the same transit speed. Every waypoint in the plain export has an explicit speed command. Existing speed controls and manual speed overrides remain available in the editable mission. The flight software controls the autonomy response to speed.
