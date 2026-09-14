# Payload Housing: Mechanical Design

The enclosure that carries the LiDAR, camera, IMU, Pi 5, and power electronics, and attaches the assembly to an F450 airframe. Designed, printed, and integrated by me as part of the [search and rescue drone project](../README.md).

Finished payload: **0.76 kg**, PETG, cylindrical body with a detachable front face.

<!-- Replace these filenames with your actual uploads in hardware/images/ -->
![Assembled payload mounted to the F450](images/payload_assembly.jpg)
![Internal layout: Pi 5, buck converter, IMU](images/internals.jpg)

## What the housing had to do

The mechanical requirements came from the sensors and the airframe, not from styling:

| Constraint | Where it came from |
|---|---|
| 360° unobstructed LiDAR horizon | The DFR0315 scans a full circle, so any structure in that plane becomes a permanent artifact in every map |
| Fixed, known camera angle | `person_mapper` composes a camera-to-body rotation as a constant. If the camera can shift, that constant is wrong and every projected detection is wrong |
| IMU near the payload center of mass | Off-axis mounting turns airframe rotation into spurious linear acceleration |
| Minimum mass | The F450 was already at its practical limit with battery and electronics |
| Attach to an existing airframe | The payload mounts to an F450 I did not design and could not modify |
| Serviceable between flights | Battery swaps and payload removal had to be fast |

## Design decisions

**Cylindrical body.** Chosen so the LiDAR scan plane clears the structure in every direction. A rectangular box is easier to print and pack, but its corners intrude into the scan horizon at four bearings and show up as fixed returns the SLAM front end has to reject.

**LiDAR mounted upside down at the bottom of the payload.** This was the best available position on two counts. It keeps the scan plane well clear of the propeller disc, and it tucks the unit under the body where it is less exposed to damage on hard landings. The drone's legs still sit inside the scan plane, which is handled in software with a minimum range filter.

**Detachable angled front face.** The 25° downward camera tilt is printed into the geometry rather than set with an adjustable bracket. A fixed angle is a known angle, and `person_mapper` depends on that. Making the face detachable kept the camera serviceable without making the angle adjustable.

**Screwed to the underside of the drone's bottom plate, with a cork damping layer in the joint.** Mounting underneath rather than on top left the battery bay clear, so batteries could be swapped between flights without touching the payload. The whole unit also comes off in a few screws, which mattered for a system that was constantly being reopened. The cork takes some of the airframe vibration out of the joint before it reaches the sensors.

**PETG.** More flexible and less brittle than PLA, so it deforms under crash loads instead of shattering. That toughness is also what allowed thinner walls than PLA would have survived, which mattered given how tight the mass budget was.

**Access and vent holes.** The body has multiple openings for cable routing, port access, and airflow over the Pi and buck converter. Internal airflow is still limited and the IMU likely ran warm, though I have no measurements to say how much that mattered.

## What I would change

**Weight was the defining constraint and we never got out from under it.** The finished payload put the F450 over its rated maximum. The right answer was a larger airframe with more thrust margin, and that was outside the project budget, so the only lever left was making the payload lighter. I redesigned the housing several times to cut mass, thinning walls and removing material wherever the structure allowed. It was not enough. The LiDAR, camera, and Pi are what they are, and the battery had to be large enough for a useful flight time, so the mass of the sensing and power hardware sat above the margin no matter what the enclosure weighed. The result was persistent difficulty keeping the drone stable, shorter test flights than planned, and less data collected than the project needed.

Next time I would size the mass budget against the airframe's measured thrust margin at the start and let it drive component selection, instead of choosing sensors first and trying to recover the difference in the enclosure.

**Foam tape was not enough isolation for the IMU.** It damped some vibration and was fine on the bench. Under spinning props it was clearly inadequate. Motor vibration coupled into the accelerometer, which cannot separate it from real acceleration. A designed isolator with a corner frequency below the prop passing frequency is the standard fix and is what I would build next time.

**Better separation between the IMU and the heat sources.** The vent holes helped, but I would move the IMU out of the same enclosed volume as the Pi and the buck converter. Even without knowing how much heat actually contributed to the drift, keeping a temperature sensitive sensor away from the two components dissipating the most power is cheap insurance.

## Files

```
cad/
├── *.step      Neutral format, opens in any CAD package
├── *.stl       Print ready, GitHub renders these in the browser
└── *.sldprt    Native SolidWorks
images/         Build and assembly photos
```

Click any STL to rotate it in the browser without downloading.

## Notes

3D printed in PETG. The hardware was returned to the school when the course ended, so this is a design record rather than a live build.

Most of the compromises above come back to the same two limits. The budget fixed the airframe, which fixed the mass ceiling, which drove several rounds of housing redesign that still could not close the gap. The schedule meant the system had to be built, wired, printed, and integrated before any of it could be tested, so problems like the vibration coupling only surfaced late, with no time left to redesign around them. A lot of what looks like a sensing or software problem in this project traces back to those two constraints.
