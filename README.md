# Litchi_DJI_FlightPlanner
Generates GSD-optimized Litchi flight plans from a KML AOI. Outputs CSV missions and KML previews with waypoints and photo footprints. 
Includes flight line spacing, altitude calc, mission chunking, and summary stats for distance, time, and batteries. Ideal for mapping with DJI drones.

### Overview

This Python script generates GSD-optimized flight plans for DJI drones using the Litchi mission format. It ingests a polygonal AOI from a KML file, computes flight line spacing and altitude based on target GSD and camera specs, and outputs:

- A full Litchi-compatible CSV mission file
- KML previews showing waypoints, flight paths, and photo footprints
- Chunked CSV and KML outputs to comply with Litchi's 99-waypoint limit
- Summary stats including total distance, estimated flight time, and battery count

Supports stop-and-shoot or continuous flight modes, adjustable overlap, flight angle, gimbal pitch, and curve size. Designed for aerial surveys in precision agriculture, environmental monitoring, and other photogrammetric workflows.
