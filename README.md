# rudn-ordjo-building

## Origin of the building model

The floor meshes in `models/meshes/` were extracted from the APK of the mobile application
"РУДН Орджоникидзе" (Android package `com.MaratInc.RudnApp`, version 1.7) and arranged into
the per-floor files here. Later commits in this repository moved them to zero height and
removed the floor surfaces. The occupancy maps in `maps/` are baked
from these meshes by `tools/bake_floor_map.py`. The meshes and the maps derived from them are
not covered by this repository's license.
