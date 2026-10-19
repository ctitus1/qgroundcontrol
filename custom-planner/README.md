# Chimera mission planner

The `custom-planner` overlay adds native Chimera mission blocks to QGC 5.1.4. The `custom` link selects this overlay.

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

The continuation clears ROI, returns to the shared split point with the survey heading and altitude, and restores speed, gimbal pitch, and mission arm (`NAV_DELAY(0)`).

Version 2 plans from the earlier draft stored fixed routes. When all matching parts are present and still agree with their original source, loading the plan restores a linked editable survey. **Restore editable survey** can also retry this conversion. Parts with missing or changed source data stay in the old fixed-route form; use the original unsplit plan to rebuild those surveys. Existing embedded inspections still load; remove them before adding split points.

Keep the saved native plan. Downloading from an aircraft returns MAVLink commands and cannot recover the source groups.

## Geometry and limits

The polygon defines the required ground area. The selected survey part is cyan with a thicker line; other parts are white. Orange outlines show sample camera footprints. The pinhole camera model uses image dimensions and `fx`, `fy`, `cx`, and `cy` in pixels. Initial values are examples, not calibration data. Optical bearing follows aircraft yaw; pitch -90 degrees points down.

Corner rays intersect flat ground at home height. A rectangle aligned with the tracks fits inside the footprint. Its width and side overlap set track spacing. QGC generates the flight legs. Sample footprints do not prove full boundary coverage.

- Relative altitude and flat ground only. No terrain coverage or obstacle clearance checks.
- No independent gimbal yaw, lens distortion model, fleet scheduler, or new undo system.
- ROI altitude means height above target terrain in the Chimera runtime. Manual waypoint altitude uses the normal QGC setting.
- Native flight-path segments support QGC's altitude display. No PX4 scene importer or embedded Gazebo view is added.
- Earlier stock offset-survey source files retain their export controls. Use native Chimera patterns for new missions.

## QGC 5.1.4

This overlay targets the upstream `v5.1.4` tag. It uses QGC's custom mission factory and native controls. The port keeps the existing Chimera plan format. Fleet editors remain independent of connected vehicles. Save and upload use QGC's separate change flags.

## Build and focused checks

Use Qt 6.11.1 and the QGC 5.1.4 build dependencies. The local build uses Ubuntu 24.04 in Docker because this host runs Ubuntu 22.04. Install `libopengl-dev` with the Linux dependencies so the video module can link EGL.

```sh
cmake -S . -B build -G Ninja \
    -DCMAKE_PREFIX_PATH="$HOME/Qt/6.11.1/gcc_64" \
    -DQGC_CUSTOM_DIR=custom-planner \
    -DQGC_STABLE_BUILD=ON \
    -DQGC_BUILD_INSTALLER=OFF
cmake --build build -j 12
```

The executable is `build/Release/Custom-QGroundControl`. The stock taskbar application remains separate.

The stock and custom builds pass with compiler warnings treated as errors. Camera-geometry checks pass. Static QML checks report context-property and scope warnings, with no missing types or properties. Planner function checks remain for the user.

The optional `QGC_PLANNER_SMOKE_TEST` build compiles the existing offline function check. Run it with `QGC_PLANNER_SMOKE=1` and `--allow-multiple`, with separate configuration directories. The production build keeps this option off. These checks do not test aircraft execution.

## Fleet planning

1. In the master, draw one convex Chimera survey and add one Chimera fiducial. The master survey supplies the shared area and initial flight defaults. Each aircraft gets its day camera preset automatically. In its **Fleet setup** section, choose 2 to 8 multicopters and the altitude limits, then select **Create drone missions**. The **Drones** field stays in the right **Master setup** section after creation.
2. The top selector switches between **Master** and each drone's normal QGC mission editor. Master shows all routes in separate colors and numbered region boundaries. Each drone view keeps the other routes faintly visible. Use **Master setup** in the right survey panel to assign aircraft, starting regions, launch coordinates, fixed headings, cameras, speed, pitch, and overlap. Default headings are spaced by 360 degrees divided by drone count. Count changes adjust default headings and preserve headings you edited. Launch points initially use the master launch location; set each drone's location before export.
3. Drone 1 visits regions 1, 2, ... . Drone 2 visits 2, 3, ... , 1, and so on. Each keeps its own heading for every region. Each region has its own camera, speed, pitch, and overlap. Add split points and ordinary QGC ROI/inspection waypoints in the drone views. Shared-area edits preserve those splits and manual items. Manual inspection coordinates do not move with the area.
4. Altitudes are assigned from the minimum upward at equal spacing. **Maximum gap** can reduce the spacing; 0 leaves it uncapped. With a cap, the highest layer can be below the maximum altitude. The panel reports the smallest assigned vertical gap. Survey, fiducial, takeoff, and manual waypoint altitudes use the assigned layer. Their home elevations use the master home elevation, so this requires launch sites on the same ground plane. ROI target altitude is not changed.
5. Normal **Save** and **Save As** save the whole master, including every drone's edited mission. Use **Export missions…** and enter a mission name. This saves the master, editable drone plans, and plain drone plans. Open a plain drone plan in QGC, select the intended vehicle, and upload it normally. Direct upload from a fleet view is blocked because the master contains multiple missions.

Count changes reuse the old control-point splitter and keep the master area editable. Each active drone visits every active region. Each drone starts in its assigned region, then visits the remaining regions in cyclic order. Changing the start region swaps assignments and keeps survey groups intact. Region numbers identify saved groups of survey parts and the manual items that follow them. Split positions remain fractions of the region route. Camera settings and inspection items stay with their region. Manual inspection coordinates stay fixed when the region changes shape, so review those paths after a count or area change.

Removed drones and regions remain in the master file as inactive data. Increasing the count restores their edits, including after save and reload. Each aircraft export includes only its active mission. New regions use that drone’s defaults from the master setup. Shared geometry, launch points, altitude limits, and per-region settings remain editable. Counts range from 2 to 8.

Center-first routes retain complete QGC survey tracks. Each region starts at the shared partition center, where all sectors meet, and accounts for the camera offset. Tracks then run from nearest to farthest from that junction, with alternating flight directions. The aircraft heading stays fixed. Moving the yellow center handle changes the start reference for every region. The master file saves this reference. Existing master plans also use their partition center. Full tracks reach the boundary before the survey covers all inner ground. In each drone view, insert normal QGC ROI commands and inspection waypoints after the first survey item, before the next region's survey.

This is an offline planner, not a fleet flight scheduler. Drones advance to the next region independently. There is no runtime phase barrier, launch/landing sequence, collision check, or obstacle check. Altitude layers describe planned survey and inspection flight, not guaranteed separation during takeoff, RTL, or landing. Each generated mission ends with ordinary RTL, which can be edited in its drone view. Region boundaries use the old planner’s control-point splitter and convex-area checks.


Master setup selects day or night cameras for UAS1–4. Day uses rgb_mid for UAS1–2 and pilot for UAS3–4. Night uses thermal. A named camera override stays fixed when day/night changes. Custom intrinsics remain editable. UAS5–8 need custom calibration or an explicit preset choice. Changing a flight default applies that field to all regions for that drone. Camera changes apply the camera values without changing region speed or pitch overrides. **Apply defaults to every region** resets camera and flight overrides for that drone.

In Master view, the yellow handle moves the partition center. Blue handles change the boundary rays and snap to their midpoints on release. Colored sectors show starting-region assignments. The top path display selector hides later sectors or all routes when you need a clear sector view. Whole-area moves carry the partition controls with them. Boundary resizing leaves the controls fixed. Sector outlines update during a drag. Flight paths rebuild after a short pause or on release. The map reuses its objects and caches unchanged partitions. Changing drone count resets the rays to equal angles. **Reset equal sectors** also restores the partition center.

Use **Edit region / ROI** to open the assigned drone’s ordinary QGC editor at that region. Split points and manual inspection waypoints remain in that editor, with the other drones’ routes visible for context. The master file saves assignments, day/night mode, flight defaults, partition controls, and all drone missions.

**Rotate entry point** changes the entry direction without changing aircraft heading. Center-out surveys alternate between the two directions of the first track. Other surveys cycle through QGC's four entry locations. The setting applies to all linked parts and stays in saved plans. Split markers move during dragging, and the route updates when you release the marker.

Export saves all formats together. **Export missions…** writes `<name>.plan` as the full master, `<name>-uasN.plan` as each editable drone plan, and `<name>-uasN-plain.plan` as each standard QGC drone plan. The plain files open without the planner plugin. **Export mission files…** uses the same complete export in a fleet view. For a standalone mission, it saves `<name>.plan` and `<name>-plain.plan`.

Generated surveys and plain exports omit camera trigger distance commands. Survey starts retain ROI cancellation, speed, pitch, and the 0-second autonomy marker. The 5g_drone ROI handoff restores pitch after clearing a held ROI, and the next mission marker reasserts survey pointing. Fiducial and vantage blocks send no gimbal commands.

Set **Fiducial radius** in Master setup or **Drone offset radius** in the master fiducial panel. The default is 2 m. Each drone waits behind the real fiducial along its survey heading, at its assigned survey altitude. Changes to the fiducial position, radius, heading, or altitude layers update these points.

**Rapid transit speed** defaults to 5 m/s. Set it in Master setup for all drones, or in a standalone survey panel. Plain exports add a transit to each survey entry, restore that survey's flight speed, and then send the 0-second mission arm marker. Split survey returns use the same transit speed. Every waypoint in the plain export has an explicit speed command. Existing speed controls and manual speed overrides remain available in the editable mission. The flight software controls the autonomy response to speed.

Plain exports use rapid transit speed for transfers between surveys and vantage points. Each transfer reaches the destination before the mission speed returns and the 0-second survey or 1-second vantage marker runs.

Fleet setup stays in the right mission editor after drone missions are created. The top selector switches between Master and individual drones. Select the master survey to edit shared settings, rotate all headings, or change the drone count. Master rotation preserves each drone's heading offset. Master flight and camera edits apply to all drones. Drone setup changes all regions for that drone. Region fields retain local speed and camera overrides.

Heading sliders update paths during a drag. New fleets use altitude limits of 20–30 m. Each drone can adjust its altitude within those limits. The panel reports the smallest vertical gap, and export rejects overlapping layers. Changing the altitude limits restores automatic layers.

The top bar contains the mission selector, path display, and **Export missions…**. **Whole path** shows every survey region. **First sector path** shows each drone's first assigned sector, including its linked parts and inspection waypoints. **No paths** keeps the sector outlines and partition handles. The master saves this display choice. Display changes do not change exported missions.

Plain exports place compatible settings in QGC's native order: waypoint, gimbal/camera, then flight speed. QGC loads these into the waypoint's checked settings. Aircraft heading stays in waypoint parameter 4. Required speed changes still precede transit waypoints and autonomy markers. The export omits redundant standalone speed commands.
