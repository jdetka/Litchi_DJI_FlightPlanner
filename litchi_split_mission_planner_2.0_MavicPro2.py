import numpy as np
import pandas as pd
from shapely.geometry import Polygon, LineString
from xml.dom import minidom
from geopy.distance import geodesic
from simplekml import Kml, AltitudeMode
import os

# === SETTINGS ===
USE_GSD_MODE = True
KML_PATH = r"C:\Users\jdetk\Desktop\Litchi_DJI_FlightPlanner\AOI\Spence_Mavic2Pro.kml"
OUTPUT_FOLDER = r"C:\Users\jdetk\Desktop\Litchi_DJI_FlightPlanner\mission_output\Spence_Rd"
OUTPUT_CSV = os.path.join(OUTPUT_FOLDER, "Spence_Rd_MavicPro_litchi_mission.csv")
OUTPUT_KML = os.path.join(OUTPUT_FOLDER, "Spence_Rd_MavicPro_photo_preview.kml")
ALTITUDE_INPUT = 20
TARGET_GSD = 0.005
SPEED = 3.5
FRONTLAP = 0.75
SIDELAP = 0.75
GIMBAL_PITCH = -90
CURVE_SIZE = 50
FLIGHT_ANGLE_DEG = 90
STOP_AND_SHOOT = False
MAX_WAYPOINTS = 98

# === Sensor Parameters ===
PIXEL_SIZE_MM = 0.00241 # (13.2 mm / 5472 px ≈ 0.00241 mm/px)
FOCAL_LENGTH_MM = 10.26 
IMAGE_WIDTH_PX = 5472
IMAGE_HEIGHT_PX = 3648

# === CALCULATIONS ===
if USE_GSD_MODE:
    ALTITUDE = (TARGET_GSD * FOCAL_LENGTH_MM) / PIXEL_SIZE_MM
else:
    ALTITUDE = ALTITUDE_INPUT
GSD = (ALTITUDE * PIXEL_SIZE_MM) / FOCAL_LENGTH_MM
image_width_m = GSD * IMAGE_WIDTH_PX
image_height_m = GSD * IMAGE_HEIGHT_PX
FRONTLAP_SPACING = image_height_m * (1 - FRONTLAP)
SIDELAP_SPACING = image_width_m * (1 - SIDELAP)

# === PARSE KML ===
xmldoc = minidom.parse(KML_PATH)
coords = xmldoc.getElementsByTagName('coordinates')[0].firstChild.data.strip()
coord_list = coords.split()
polygon_coords = [(float(c.split(',')[0]), float(c.split(',')[1])) for c in coord_list]
polygon = Polygon(polygon_coords)

# === GENERATE FLIGHT LINES ===
centroid = polygon.centroid
angle_rad = np.radians(FLIGHT_ANGLE_DEG)
buffer = 250
line_length = 2 * buffer
line_count = 100

lines = []
for i in range(-line_count // 2, line_count // 2 + 1):
    offset_x = i * SIDELAP_SPACING / 111320 * np.cos(angle_rad + np.pi/2)
    offset_y = i * SIDELAP_SPACING / 111320 * np.sin(angle_rad + np.pi/2)
    dx = np.cos(angle_rad) * line_length / 111320
    dy = np.sin(angle_rad) * line_length / 111320
    x0 = centroid.x + offset_x - dx / 2
    y0 = centroid.y + offset_y - dy / 2
    x1 = centroid.x + offset_x + dx / 2
    y1 = centroid.y + offset_y + dy / 2
    line = LineString([(x0, y0), (x1, y1)])
    clipped = line.intersection(polygon)
    if not clipped.is_empty and clipped.length > 0:
        lines.append(clipped)

# === Heading Calculation ===
def calculate_heading(lat1, lon1, lat2, lon2):
    d_lon = np.radians(lon2 - lon1)
    y = np.sin(d_lon) * np.cos(np.radians(lat2))
    x = np.cos(np.radians(lat1)) * np.sin(np.radians(lat2)) - \
        np.sin(np.radians(lat1)) * np.cos(np.radians(lat2)) * np.cos(d_lon)
    return (np.degrees(np.arctan2(y, x)) + 360) % 360

# === GENERATE WAYPOINTS WITH DYNAMIC HEADINGS ===
waypoints = []
last_lat = None
last_lon = None

for i, line in enumerate(lines):
    num_points = int(line.length * 111320 // FRONTLAP_SPACING)
    if num_points < 1:
        continue
    fractions = [j / num_points for j in range(num_points + 1)]
    if i % 2 == 1:
        fractions = fractions[::-1]
    for f in fractions:
        pt = line.interpolate(f, normalized=True)
        lat = float(pt.y)
        lon = float(pt.x)

        if last_lat is not None and last_lon is not None:
            heading = calculate_heading(last_lat, last_lon, lat, lon)
        else:
            heading = FLIGHT_ANGLE_DEG

        wp = {
            "latitude": lat,
            "longitude": lon,
            "altitude(m)": float(ALTITUDE),
            "heading(deg)": float(heading),
            "curvesize(m)": 0.0 if STOP_AND_SHOOT else float(CURVE_SIZE),
            "rotationdir": 1,
            "gimbalmode": 1,
            "gimbalpitchangle": float(GIMBAL_PITCH),
            "altitudemode": 1,
            "speed(m/s)": float(SPEED),
            "poi_latitude": 0.0,
            "poi_longitude": 0.0,
            "poi_altitude(m)": 0.0,
            "poi_altitudemode": 1,
            "photo_timeinterval": 0,
            "photo_distinterval": 0
        }
        for j in range(1, 17):
            wp[f"actiontype{j}"] = 0
            wp[f"actionparam{j}"] = 0

        waypoints.append(wp)
        last_lat = lat
        last_lon = lon

# === COLUMNS IN LITCHI ORDER ===
columns = [
    "latitude", "longitude", "altitude(m)", "heading(deg)", "curvesize(m)", "rotationdir",
    "gimbalmode", "gimbalpitchangle"
] + [f"actiontype{j}" for j in range(1, 17)] + [f"actionparam{j}" for j in range(1, 17)] + [
    "altitudemode", "speed(m/s)", "poi_latitude", "poi_longitude", "poi_altitude(m)",
    "poi_altitudemode", "photo_timeinterval", "photo_distinterval"
]

# === EXPORT FULL CSV ===
df = pd.DataFrame(waypoints, columns=columns)
df.to_csv(OUTPUT_CSV, sep=";", index=False, encoding="utf-8")

# === KML EXPORT FUNCTION ===
def create_kml(waypoints, path, base_index=1):
    kml = Kml()
    f_pts = kml.newfolder(name="Photo Points")
    f_boxes = kml.newfolder(name="Photo Footprints")
    f_path = kml.newfolder(name="Flight Path")

    flight_line = f_path.newlinestring(name="Flight Path")
    flight_line.coords = [(wp["longitude"], wp["latitude"], wp["altitude(m)"]) for wp in waypoints]
    flight_line.altitudemode = AltitudeMode.relativetoground
    flight_line.extrude = 1
    flight_line.style.linestyle.color = "ff0000ff"
    flight_line.style.linestyle.width = 2

    half_w = image_width_m / 2
    half_h = image_height_m / 2
    heading_rad = np.radians(FLIGHT_ANGLE_DEG)

    for i, wp in enumerate(waypoints, start=base_index):
        lat, lon = wp["latitude"], wp["longitude"]
        deg_lat = 1 / 111320
        deg_lon = 1 / (40075000 * np.cos(np.radians(lat)) / 360)

        corners = []
        for dx, dy in [(-half_w, -half_h), (-half_w, half_h), (half_w, half_h), (half_w, -half_h)]:
            x_rot = dx * np.cos(heading_rad) - dy * np.sin(heading_rad)
            y_rot = dx * np.sin(heading_rad) + dy * np.cos(heading_rad)
            corners.append((lon + x_rot * deg_lon, lat + y_rot * deg_lat))

        pol = f_boxes.newpolygon(name=f"Footprint {i}")
        pol.outerboundaryis = corners + [corners[0]]
        pol.altitudemode = AltitudeMode.clamptoground
        pol.style.polystyle.color = "7d00ffff"
        pol.style.linestyle.color = "ff00ffff"
        pol.style.linestyle.width = 1

        pnt = f_pts.newpoint(name=f"WP {i}", coords=[(lon, lat, wp["altitude(m)"])])
        pnt.altitudemode = AltitudeMode.relativetoground
        pnt.style.iconstyle.icon.href = "http://maps.google.com/mapfiles/kml/shapes/camera.png"
        pnt.style.labelstyle.scale = 0.5

    kml.save(path)

# === EXPORT KML FOR FULL MISSION ===
create_kml(waypoints, OUTPUT_KML)

# === EXPORT CHUNKED CSV + KML ===
for i in range(0, len(df), MAX_WAYPOINTS):
    chunk = df.iloc[i:i+MAX_WAYPOINTS]
    chunk_index = i // MAX_WAYPOINTS + 1
    csv_path = OUTPUT_CSV.replace(".csv", f"_chunk{chunk_index}.csv")
    kml_path = OUTPUT_KML.replace(".kml", f"_chunk{chunk_index}.kml")
    chunk.to_csv(csv_path, sep=";", index=False, encoding="utf-8")
    create_kml(chunk.to_dict("records"), kml_path, base_index=i+1)

# === SUMMARY ===
total_distance = sum(
    geodesic((waypoints[i-1]["latitude"], waypoints[i-1]["longitude"]),
             (waypoints[i]["latitude"], waypoints[i]["longitude"])).meters
    for i in range(1, len(waypoints))
)
flight_time = total_distance / SPEED / 60
batteries_needed = int(np.ceil(flight_time / 25))

print(f"🔍 Target GSD: {TARGET_GSD*100:.1f} cm/pixel")
print(f"⛰️ Altitude AGL: {ALTITUDE:.1f} meters")
print(f"📏 Flight Distance: {total_distance:.1f} m")
print(f"⏱️ Flight Time: {flight_time:.1f} min @ {SPEED:.1f} m/s")
print(f"📸 Photos: {len(waypoints)}")
print(f"🔋 Batteries (20 min): {batteries_needed}")
