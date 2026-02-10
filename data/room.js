/* ==========================================================================
   room.js — 3D Scanner Viewer (Three.js)
   ==========================================================================

   Browser-based 3D point cloud viewer for the VL53L5CX ToF sensor + BNO085 IMU.
   This is a browser-native port of the Python Viser-based viewer found in
   VL53L5CX-BNO08X-viewer-main/. All processing runs client-side — the ESP32
   only streams raw sensor data via SSE.

   Architecture:
   - Three.js scene with board meshes, live point cloud, zone rays, and map points
   - SSE event stream from /api/events (tof @4Hz, imu @10Hz, device @1Hz)
   - Two coordinate conversion methods: Uniform Grid and ST Lookup Table
   - IMU quaternion with Z-up→Y-up remap and ToF frame correction
   - Temporal filtering (EMA), plane fitting (LS + RANSAC), mapping mode
   - Full control panel matching the Python viewer's Viser GUI

   Scene hierarchy:
     Scene
       ├── GridHelper (4m x 4m floor grid)
       ├── AxesHelper (0.5m RGB axes at origin)
       ├── AmbientLight + DirectionalLight
       ├── boardGroup (Group — IMU quaternion applied here)
       │     ├── imuBoard (purple box 15x26x1mm)
       │     ├── imuAxes (10mm axes at sensor offset)
       │     ├── tofBoard (green box 10x16x1mm, -25.4mm offset)
       │     └── sensorGroup (Group — 90° CCW yaw for ToF sensor frame)
       │           ├── sensorAxes (10mm axes)
       │           ├── livePoints (Points — 64 sensor-local positions)
       │           ├── rayLines (LineSegments — 64 zone rays)
       │           └── planeMesh (Mesh — yellow transparent fitted plane)
       └── mapPoints (Points — world-coordinate accumulated map)
   ========================================================================== */

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

/* ==========================================================================
   VL53L5CX Sensor Constants
   ========================================================================== */

const RESOLUTION = 8;           // 8x8 zone grid
const NUM_ZONES = 64;           // Total zones per sensor frame
const FOV_DIAGONAL_DEG = 65.0;  // Diagonal field of view in degrees
const MAX_RANGE_MM = 4000;      // Maximum measurable distance
const MIN_RANGE_MM = 20;        // Minimum measurable distance
const MAX_RANGE_M = MAX_RANGE_MM / 1000;
const MIN_RANGE_M = MIN_RANGE_MM / 1000;
const RAY_COLOR = 0x6496ff;     // Light blue for zone ray visualization

// Mapping mode downsampling thresholds (matches Python viewer's config.py)
const DOWNSAMPLE_POINT_THRESHOLD = 500;   // Trigger after this many new points
const DOWNSAMPLE_BUFFER_THRESHOLD = 15;   // Or after this many frame buffers

/* ==========================================================================
   ST-Calibrated Lookup Tables (from VL53L5CX Datasheet)
   ==========================================================================

   These tables contain factory-calibrated pitch and yaw angles for each of
   the 64 zones. They account for actual lens optical characteristics which
   are non-uniform (zones near edges have different angular spacing than center).
   Source: https://community.st.com/t5/imaging-sensors/vl53l5cx-multi-zone-sensor-get-x-y-z-of-points-relative-to/td-p/172929

   Pitch angles: elevation from the optical axis (90° = straight ahead).
   Yaw angles: azimuth around the optical axis.
   Layout: row-major order, zone 0 = top-left when viewed from sensor front. */

const ST_PITCH_ANGLES_DEG = [
    59.00, 64.00, 67.50, 70.00, 70.00, 67.50, 64.00, 59.00,
    64.00, 70.00, 72.90, 74.90, 74.90, 72.90, 70.00, 64.00,
    67.50, 72.90, 77.40, 80.50, 80.50, 77.40, 72.90, 67.50,
    70.00, 74.90, 80.50, 85.75, 85.75, 80.50, 74.90, 70.00,
    70.00, 74.90, 80.50, 85.75, 85.75, 80.50, 74.90, 70.00,
    67.50, 72.90, 77.40, 80.50, 80.50, 77.40, 72.90, 67.50,
    64.00, 70.00, 72.90, 74.90, 74.90, 72.90, 70.00, 64.00,
    59.00, 64.00, 67.50, 70.00, 70.00, 67.50, 64.00, 59.00,
];

const ST_YAW_ANGLES_DEG = [
    135.00, 125.40, 113.20,  98.13,  81.87,  66.80,  54.60,  45.00,
    144.60, 135.00, 120.96, 101.31,  78.69,  59.04,  45.00,  35.40,
    156.80, 149.04, 135.00, 108.45,  71.55,  45.00,  30.96,  23.20,
    171.87, 168.69, 161.55, 135.00,  45.00,  18.45,  11.31,   8.13,
    188.13, 191.31, 198.45, 225.00, 315.00, 341.55, 348.69, 351.87,
    203.20, 210.96, 225.00, 251.55, 288.45, 315.00, 329.04, 336.80,
    215.40, 225.00, 239.04, 258.69, 281.31, 300.96, 315.00, 324.60,
    225.00, 234.60, 246.80, 261.87, 278.13, 293.20, 305.40, 315.00,
];

/* ==========================================================================
   Precomputed Zone Angles
   ==========================================================================

   Called once at module load. Precomputes trigonometric values for both
   coordinate conversion methods so per-frame processing only needs
   multiplications (no trig calls in the hot path).

   For Uniform Grid: precomputes tan(angle) for X/Y per zone, plus
   normalized ray direction vectors.

   For ST Lookup: precomputes sin/cos of pitch and yaw angles, plus
   normalized ray direction vectors using the spherical-to-cartesian
   conversion from the factory-calibrated tables. */

const zoneAngles = computeZoneAngles();

function computeZoneAngles() {
    // --- Uniform Grid Method ---
    // Convert diagonal FOV to per-axis FOV assuming square sensor:
    // For a square, diagonal = side * sqrt(2), so side = diagonal / sqrt(2)
    const fovPerAxisRad = (FOV_DIAGONAL_DEG / Math.sqrt(2)) * Math.PI / 180;
    // Angular spacing between zone centers
    const angleStep = fovPerAxisRad / RESOLUTION;

    const tanX = new Float32Array(NUM_ZONES);
    const tanY = new Float32Array(NUM_ZONES);
    const rayDirX = new Float32Array(NUM_ZONES);
    const rayDirY = new Float32Array(NUM_ZONES);
    const rayDirZ = new Float32Array(NUM_ZONES);

    for (let i = 0; i < NUM_ZONES; i++) {
        const row = Math.floor(i / RESOLUTION);
        const col = i % RESOLUTION;
        // Zone center offset from optical axis. The sensor lens inverts
        // the image, so we flip both X and Y (3.5 - col, 3.5 - row)
        const colOffset = 3.5 - col;
        const rowOffset = 3.5 - row;
        // Precompute tangent for per-frame XY calculation:
        // x = z * tan(angle_x), y = z * tan(angle_y)
        tanX[i] = Math.tan(colOffset * angleStep);
        tanY[i] = Math.tan(rowOffset * angleStep);
        // Normalized ray direction for visualization (unit vector along zone ray)
        const norm = Math.sqrt(tanX[i] ** 2 + tanY[i] ** 2 + 1);
        rayDirX[i] = tanX[i] / norm;
        rayDirY[i] = tanY[i] / norm;
        rayDirZ[i] = 1.0 / norm;
    }

    // --- ST Lookup Table Method ---
    // Precompute sin/cos of factory-calibrated pitch and yaw angles
    const stSinPitch = new Float32Array(NUM_ZONES);
    const stCosPitch = new Float32Array(NUM_ZONES);
    const stSinYaw = new Float32Array(NUM_ZONES);
    const stCosYaw = new Float32Array(NUM_ZONES);
    const stRayDirX = new Float32Array(NUM_ZONES);
    const stRayDirY = new Float32Array(NUM_ZONES);
    const stRayDirZ = new Float32Array(NUM_ZONES);

    for (let i = 0; i < NUM_ZONES; i++) {
        const pRad = ST_PITCH_ANGLES_DEG[i] * Math.PI / 180;
        const yRad = ST_YAW_ANGLES_DEG[i] * Math.PI / 180;
        stSinPitch[i] = Math.sin(pRad);
        stCosPitch[i] = Math.cos(pRad);
        stSinYaw[i] = Math.sin(yRad);
        stCosYaw[i] = Math.cos(yRad);
        // Spherical-to-cartesian ray direction:
        // Negate X to match lens-flip convention (same as Python viewer)
        const rx = -stCosYaw[i] * stCosPitch[i];
        const ry = stSinYaw[i] * stCosPitch[i];
        const rz = stSinPitch[i];
        // Normalize (should already be unit length, but ensure numerical stability)
        const rn = Math.sqrt(rx * rx + ry * ry + rz * rz);
        stRayDirX[i] = rx / rn;
        stRayDirY[i] = ry / rn;
        stRayDirZ[i] = rz / rn;
    }

    return {
        tanX, tanY, rayDirX, rayDirY, rayDirZ,
        stSinPitch, stCosPitch, stSinYaw, stCosYaw,
        stRayDirX, stRayDirY, stRayDirZ
    };
}

/* ==========================================================================
   Three.js Scene Setup
   ========================================================================== */

// Renderer: uses the existing <canvas id="viewport"> element from room.html.
// Antialiasing enabled for smooth edges on board meshes and rays.
const canvas = document.getElementById('viewport');
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
renderer.setPixelRatio(window.devicePixelRatio);  // Retina display support
renderer.setSize(window.innerWidth, window.innerHeight);

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x1a1a2e);  // Match dark theme background

// Camera: narrow 20° FOV for a telephoto-like view that matches the Python viewer's
// initial perspective. Near plane at 1mm to see board meshes up close.
const camera = new THREE.PerspectiveCamera(20, window.innerWidth / window.innerHeight, 0.001, 100);
camera.position.set(0, 16.0, -16.0);  // Above and behind, looking down at origin
camera.lookAt(0, 0, 0);

// OrbitControls: click-drag to rotate, scroll to zoom, right-drag to pan.
// Damping gives smooth deceleration when releasing the mouse.
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.dampingFactor = 0.08;
controls.target.set(0, 0, 0);

// Reference grid: 4m x 4m on the XZ plane with 0.2m spacing (20 divisions).
// Provides spatial reference for distance estimation in the 3D view.
const grid = new THREE.GridHelper(4, 20, 0xa0a0a0, 0x404040);
scene.add(grid);

// Axes helper: RGB arrows (R=X, G=Y, B=Z) at origin, 0.5m long
const axes = new THREE.AxesHelper(0.5);
scene.add(axes);

// Lighting: ambient fills shadows, directional provides depth via shading
scene.add(new THREE.AmbientLight(0xffffff, 0.4));
const dirLight = new THREE.DirectionalLight(0xffffff, 0.8);
dirLight.position.set(1, 2, -1);
scene.add(dirLight);

/* ==========================================================================
   Board Group — Physical Board Meshes + Sensor Frame
   ==========================================================================

   boardGroup is the root transform for all sensor-related objects.
   The IMU quaternion is applied to boardGroup.quaternion, which rotates
   everything together (both boards, sensor frame, live points, rays, plane).

   Board dimensions and positions match the Python viewer's config.py:
   - IMU board (BNO08X): 15x26x1mm purple box at origin
   - ToF board (VL53L5CX): 10x16x1mm green box, 25.4mm below IMU
   - Sensor group: positioned at ToF sensor aperture with 90° CCW yaw
     to align the VL53L5CX internal coordinate system with world frame */

const boardGroup = new THREE.Group();
scene.add(boardGroup);

// IMU board: BNO08X on 15mm x 26mm breakout (purple).
// Board center is offset from the IMU sensor position (origin) by -sensor_offset,
// so the sensor chip sits at the IMU world_position. Matches Python's scene.py
// where board_pos = world_position - sensor_offset = (0, -0.004, -0.0005)_zup
// → remapped to Y-up: (0, -0.0005, -0.004).
const imuGeo = new THREE.BoxGeometry(0.015, 0.001, 0.026);
const imuMat = new THREE.MeshStandardMaterial({ color: 0x800080 });
const imuBoard = new THREE.Mesh(imuGeo, imuMat);
imuBoard.position.set(0, -0.0005, -0.004);
boardGroup.add(imuBoard);

// IMU sensor axes: at IMU world_position = origin (0, 0, 0).
const imuAxes = new THREE.AxesHelper(0.01);
boardGroup.add(imuAxes);

// ToF board: VL53L5CX on 10mm x 16mm breakout (green).
// Board center = ToF world_position - sensor_offset = (0, -0.0294, -0.0005)_zup
// → remapped to Y-up: (0, -0.0005, -0.0294).
const tofGeo = new THREE.BoxGeometry(0.010, 0.001, 0.016);
const tofMat = new THREE.MeshStandardMaterial({ color: 0x006400 });
const tofBoard = new THREE.Mesh(tofGeo, tofMat);
tofBoard.position.set(0, -0.0005, -0.0294);
boardGroup.add(tofBoard);

// Sensor group: represents the ToF sensor's local coordinate frame.
// Positioned at the ToF sensor world_position = (0, -0.0254, 0)_zup
// → remapped to Y-up: (0, 0, -0.0254).
// 90° CCW yaw (around Y in Y-up space) corrects the VL53L5CX internal orientation.
// All sensor-local objects (points, rays, plane) are children of this group.
const sensorGroup = new THREE.Group();
sensorGroup.position.set(0, 0, -0.0254);
sensorGroup.rotation.y = -Math.PI / 2;  // 90° CCW around Y axis
boardGroup.add(sensorGroup);

// Sensor axes: shows the ToF sensor's local XYZ directions
const sensorAxes = new THREE.AxesHelper(0.01);
sensorGroup.add(sensorAxes);

/* ==========================================================================
   Live Points — Real-time 64-point cloud from current sensor frame
   ==========================================================================

   Pre-allocated Float32Arrays for position and color attributes.
   Updated every ToF frame (~10Hz). Each of the 64 zones produces one point
   in sensor-local coordinates. Colors are computed from distance (blue=close,
   red=far) with gray for invalid zones (status != 5). */

const livePositions = new Float32Array(NUM_ZONES * 3);
const liveColors = new Float32Array(NUM_ZONES * 3);
const liveGeometry = new THREE.BufferGeometry();
liveGeometry.setAttribute('position', new THREE.BufferAttribute(livePositions, 3));
liveGeometry.setAttribute('color', new THREE.BufferAttribute(liveColors, 3));
const liveMaterial = new THREE.PointsMaterial({
    size: 5,                // 5px default (matches point-size slider)
    vertexColors: true,     // Per-point color from color attribute
    sizeAttenuation: false, // Fixed pixel size regardless of zoom level
});
const livePoints = new THREE.Points(liveGeometry, liveMaterial);
livePoints.visible = false;  // Mapping mode is on by default — live points hidden
sensorGroup.add(livePoints);

/* ==========================================================================
   Zone Rays — 64 line segments showing sensor field of view
   ==========================================================================

   Each ray extends from MIN_RANGE to MAX_RANGE along its zone's direction.
   Rendered as LineSegments (pairs of vertices). When "Clip to Measurement"
   is enabled, each ray's end point is shortened to the actual measured
   distance for that zone. Ray directions depend on the selected coordinate
   method (Uniform vs ST Lookup). */

const rayPositions = new Float32Array(NUM_ZONES * 2 * 3);  // 64 lines * 2 endpoints * xyz
const rayGeometry = new THREE.BufferGeometry();
rayGeometry.setAttribute('position', new THREE.BufferAttribute(rayPositions, 3));
const rayMaterial = new THREE.LineBasicMaterial({ color: RAY_COLOR });
const rayLines = new THREE.LineSegments(rayGeometry, rayMaterial);
rayLines.visible = false;  // Show Zone Rays is unchecked by default
sensorGroup.add(rayLines);

/**
 * Update zone ray geometry based on current distances and coordinate method.
 * @param {number[]|null} distances - 64 distance values in mm (null = full length)
 * @param {number[]|null} status - 64 status values (5 = valid)
 * @param {string} method - 'uniform' or 'st' coordinate conversion method
 * @param {boolean} clip - If true, clip ray endpoints to measured distances
 */
function updateRays(distances, status, method, clip) {
    const pos = rayPositions;
    for (let i = 0; i < NUM_ZONES; i++) {
        // Select ray direction using unnormalized z=1 convention (matches Python viewer).
        // This ensures clipped ray endpoints land exactly on the measured point position,
        // because the perpendicular distance IS the z-component of the endpoint.
        let dx, dy, dz;
        if (method === 'uniform') {
            // Tangent-based: (tanX, tanY, 1) — unnormalized, z=1
            dx = zoneAngles.tanX[i];
            dy = zoneAngles.tanY[i];
            dz = 1.0;
        } else {
            // ST lookup: rescale normalized ray direction to z=1 convention
            const rdz = zoneAngles.stRayDirZ[i];
            if (rdz > 0) {
                dx = zoneAngles.stRayDirX[i] / rdz;
                dy = zoneAngles.stRayDirY[i] / rdz;
                dz = 1.0;
            } else {
                dx = zoneAngles.stRayDirX[i];
                dy = zoneAngles.stRayDirY[i];
                dz = zoneAngles.stRayDirZ[i];
            }
        }

        // Start point: MIN_RANGE along ray direction (z = MIN_RANGE_M)
        const startDist = MIN_RANGE_M;
        // End point: MAX_RANGE by default, or measured distance if clipping
        let endDist = MAX_RANGE_M;
        if (clip && distances && status) {
            const d = distances[i];
            const s = status[i];
            if (s === 5 && d >= MIN_RANGE_MM) {
                endDist = d / 1000;
            }
        }

        // Write two endpoints (start, end) into the position buffer.
        // With z=1 convention: z-component = perpendicular distance (startDist or endDist)
        const idx = i * 6;
        pos[idx]     = dx * startDist;
        pos[idx + 1] = dy * startDist;
        pos[idx + 2] = dz * startDist;
        pos[idx + 3] = dx * endDist;
        pos[idx + 4] = dy * endDist;
        pos[idx + 5] = dz * endDist;
    }
    rayGeometry.attributes.position.needsUpdate = true;
}

// Initialize rays with full length using uniform grid directions
updateRays(null, null, 'uniform', false);

/* ==========================================================================
   Plane Fitting Mesh — Yellow transparent plane overlay
   ==========================================================================

   A unit box (1x1x0.0001) that gets scaled and oriented to match the fitted
   plane. Hidden by default, shown when "Fit Plane" checkbox is enabled.
   DoubleSide rendering ensures visibility from both sides. */

const planeGeo = new THREE.BoxGeometry(1, 1, 0.0001);
const planeMat = new THREE.MeshStandardMaterial({
    color: 0xffff00,            // Yellow
    transparent: true,
    opacity: 0.5,               // Semi-transparent to see points through it
    side: THREE.DoubleSide,     // Visible from both sides
});
const planeMesh = new THREE.Mesh(planeGeo, planeMat);
planeMesh.visible = false;
sensorGroup.add(planeMesh);

/* ==========================================================================
   Map Points — World-coordinate accumulated point cloud
   ==========================================================================

   Used in mapping mode to accumulate points over time as the device moves.
   Pre-allocated to MAX_MAP_ALLOC (500k) points for performance — avoids
   re-creating GPU buffers on every frame. drawRange.count controls how many
   points are actually rendered. Sits at the scene root (not in boardGroup)
   so accumulated points stay in world space while the device rotates. */

const MAX_MAP_ALLOC = 500000;
const mapPositions = new Float32Array(MAX_MAP_ALLOC * 3);
const mapColors = new Float32Array(MAX_MAP_ALLOC * 3);
const mapGeometry = new THREE.BufferGeometry();
mapGeometry.setAttribute('position', new THREE.BufferAttribute(mapPositions, 3));
mapGeometry.setAttribute('color', new THREE.BufferAttribute(mapColors, 3));
mapGeometry.setDrawRange(0, 0);  // Nothing visible until points are added
const mapMaterial = new THREE.PointsMaterial({
    size: 5,
    vertexColors: true,
    sizeAttenuation: false,
});
const mapPoints = new THREE.Points(mapGeometry, mapMaterial);
scene.add(mapPoints);

/* ==========================================================================
   Application State
   ========================================================================== */

// Latest raw sensor data from SSE events
let latestDistances = null;   // 64-element distance array (mm)
let latestStatus = null;      // 64-element status array (5 = valid)
let latestIMU = null;         // {w, x, y, z, a} quaternion data
let imuConnected = false;     // Whether BNO085 is sending quaternion data

// ToF frame rate measurement
let tofHz = 0;                // Computed ToF data frequency
let lastTofTime = 0;          // Timestamp of last frame
let tofFrameCount = 0;        // Frames received since last Hz calculation
let tofHzTimer = 0;           // Timer for Hz calculation window

// Temporal filter state (exponential moving average)
const filteredDistances = new Float32Array(NUM_ZONES);
let filterInitialized = false;  // First frame initializes the filter buffer

// Mapping mode state — accumulates points in world coordinates
const mappingBufferPoints = [];   // Array of Float32Array position buffers
const mappingBufferColors = [];   // Array of Float32Array color buffers
let mappingTotalPoints = 0;       // Total accumulated point count
let mappingBufferCount = 0;       // Number of frame buffers in accumulator
let clearMapRequested = false;    // Deferred clear flag (processed in main loop)

/* ==========================================================================
   Distance-to-Color Mapping (matches Python viewer's geometry.get_colors)
   ==========================================================================

   Converts distance to RGB color:
   - Blue (close, 20mm) → Purple (mid) → Red (far, 4000mm)
   - Green channel peaks in the middle for warm mid-tones
   - Invalid zones (status != 5 or below minimum range) are gray

   Returns [r, g, b] as floats in [0, 1] for Three.js vertex colors. */

function distanceColor(dist, status) {
    if (status !== 5 || dist < MIN_RANGE_MM) {
        return [0.5, 0.5, 0.5];  // Gray for invalid measurements
    }
    // Normalize to [0, 1] range across the sensor's measurement range
    const norm = Math.max(0, Math.min(1, (dist - MIN_RANGE_MM) / (MAX_RANGE_MM - MIN_RANGE_MM)));
    const r = norm;                                        // Red increases with distance
    const g = (1 - Math.abs(2 * norm - 1)) * (200 / 255); // Green peaks at mid-range
    const b = 1 - norm;                                    // Blue decreases with distance
    return [r, g, b];
}

/* ==========================================================================
   Zone-to-3D Coordinate Conversion
   ==========================================================================

   Converts 64 distance measurements into 3D points in sensor-local coords.
   Two methods available (selected via dropdown):

   Uniform Grid: assumes ideal pinhole model with uniform angular spacing.
     x = z * tan(angle_x), y = z * tan(angle_y), z = distance/1000

   ST Lookup: uses factory-calibrated pitch/yaw angles per zone.
     hypotenuse = distance / sin(pitch)
     x = -cos(yaw) * cos(pitch) * hyp  (negated X for lens flip)
     y =  sin(yaw) * cos(pitch) * hyp
     z = distance/1000

   Returns array of 64 THREE.Vector3 (or null for invalid zones). */

function distancesToPoints(distances, status, method) {
    const points = [];
    for (let i = 0; i < NUM_ZONES; i++) {
        const d = distances[i];
        const s = status[i];
        // Skip invalid zones: status must be 5 (valid) and distance above minimum
        if (s !== 5 || d < MIN_RANGE_MM) {
            points.push(null);
            continue;
        }
        const zm = d / 1000;  // Convert mm to meters
        let x, y, z;
        if (method === 'uniform') {
            // Uniform grid: perpendicular distance * precomputed tangent
            x = zm * zoneAngles.tanX[i];
            y = zm * zoneAngles.tanY[i];
            z = zm;
        } else {
            // ST lookup: hypotenuse from perpendicular distance, then project
            const hyp = (d / zoneAngles.stSinPitch[i]) / 1000;
            x = -zoneAngles.stCosYaw[i] * zoneAngles.stCosPitch[i] * hyp;
            y = zoneAngles.stSinYaw[i] * zoneAngles.stCosPitch[i] * hyp;
            z = zm;
        }
        points.push(new THREE.Vector3(x, y, z));
    }
    return points;
}

/* ==========================================================================
   Temporal Filter — Exponential Moving Average (EMA)
   ==========================================================================

   Smooths noisy distance measurements over time. The filter strength
   controls the blend ratio between new and previous values:
     filtered = alpha * new + (1-alpha) * previous
     alpha = 1.0 - strength  (strength=0: no filtering, strength=1: max)

   First frame initializes the filter buffer (no smoothing applied).
   Filter state is reset when the user unchecks "Enable Filtering". */

function applyFilter(distances, strength) {
    const alpha = 1.0 - strength;
    if (!filterInitialized) {
        // First frame: copy raw values as initial state
        filteredDistances.set(distances);
        filterInitialized = true;
        return filteredDistances.slice();
    }
    for (let i = 0; i < NUM_ZONES; i++) {
        filteredDistances[i] = alpha * distances[i] + (1 - alpha) * filteredDistances[i];
    }
    return filteredDistances.slice();
}

/* ==========================================================================
   Plane Fitting — Least Squares and RANSAC
   ==========================================================================

   Fits a plane z = ax + by + c to valid 3D points using either:

   1. Least Squares: solves the 3x3 normal equations via Cramer's rule.
      Fast but sensitive to outliers.

   2. RANSAC: runs 100 iterations of random 3-point plane fitting,
      selects the model with most inliers within the threshold distance,
      then refits using only the inliers. Robust to outliers.

   Both return: { position, quaternion, size, rmse } for rendering
   the yellow transparent plane mesh, or null if fitting fails. */

/**
 * Fit a plane to 3D points using least squares (z = ax + by + c).
 * Solves the 3x3 normal equation system via Cramer's rule (no matrix library needed).
 * @param {THREE.Vector3[]} validPoints - Array of valid 3D points
 * @returns {Object|null} { position, quaternion, size, rmse } or null if < 3 points
 */
function fitPlaneLS(validPoints) {
    if (validPoints.length < 3) return null;

    const n = validPoints.length;
    // Accumulate sums for the normal equation matrix
    let sx = 0, sy = 0, sz = 0, sxx = 0, syy = 0, sxy = 0, sxz = 0, syz = 0;
    for (const p of validPoints) {
        sx += p.x; sy += p.y; sz += p.z;
        sxx += p.x * p.x; syy += p.y * p.y; sxy += p.x * p.y;
        sxz += p.x * p.z; syz += p.y * p.z;
    }

    // Normal equations: A * [a,b,c]^T = B
    // A = [sxx sxy sx; sxy syy sy; sx sy n], B = [sxz, syz, sz]
    const A = [
        [sxx, sxy, sx],
        [sxy, syy, sy],
        [sx,  sy,  n ]
    ];
    const B = [sxz, syz, sz];

    // Determinant of A (Cramer's rule)
    const det = A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1])
              - A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0])
              + A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);

    if (Math.abs(det) < 1e-12) return null;  // Degenerate (collinear points)

    // Solve for a, b, c using Cramer's rule
    const invDet = 1 / det;
    const a = ((A[1][1] * A[2][2] - A[1][2] * A[2][1]) * B[0]
             + (A[0][2] * A[2][1] - A[0][1] * A[2][2]) * B[1]
             + (A[0][1] * A[1][2] - A[0][2] * A[1][1]) * B[2]) * invDet;
    const b = ((A[1][2] * A[2][0] - A[1][0] * A[2][2]) * B[0]
             + (A[0][0] * A[2][2] - A[0][2] * A[2][0]) * B[1]
             + (A[0][2] * A[1][0] - A[0][0] * A[1][2]) * B[2]) * invDet;
    const c = ((A[1][0] * A[2][1] - A[1][1] * A[2][0]) * B[0]
             + (A[0][1] * A[2][0] - A[0][0] * A[2][1]) * B[1]
             + (A[0][0] * A[1][1] - A[0][1] * A[1][0]) * B[2]) * invDet;

    // Plane normal: for z = ax + by + c, the normal is (-a, -b, 1) normalized
    const normal = new THREE.Vector3(-a, -b, 1).normalize();

    // RMSE: root mean square error of point-to-plane distances (in mm)
    const denom = Math.sqrt(a * a + b * b + 1);
    let sumSqErr = 0;
    for (const p of validPoints) {
        const res = Math.abs(a * p.x + b * p.y - p.z + c) / denom;
        sumSqErr += res * res;
    }
    const rmse = Math.sqrt(sumSqErr / n) * 1000;  // Convert meters to mm

    // Position: centroid of points projected onto the fitted plane
    const cx = sx / n, cy = sy / n;
    const cz = a * cx + b * cy + c;
    const position = new THREE.Vector3(cx, cy, cz);

    // Size: largest XY span of points, padded by 1.2x, minimum 5cm
    let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
    for (const p of validPoints) {
        if (p.x < minX) minX = p.x;
        if (p.x > maxX) maxX = p.x;
        if (p.y < minY) minY = p.y;
        if (p.y > maxY) maxY = p.y;
    }
    let size = Math.max(maxX - minX, maxY - minY) * 1.2;
    if (size < 0.05) size = 0.05;  // Minimum 5cm for visibility

    // Quaternion: aligns the mesh's Z-axis with the plane normal
    const quat = new THREE.Quaternion().setFromUnitVectors(
        new THREE.Vector3(0, 0, 1), normal
    );

    return { position, quaternion: quat, size, rmse };
}

/**
 * Fit a plane using RANSAC for robust outlier rejection.
 * Runs 100 iterations of random 3-point plane fitting, keeps the model
 * with the most inliers, then refits with least squares on the inlier set.
 * @param {THREE.Vector3[]} validPoints - Array of valid 3D points
 * @param {number} thresholdMM - Inlier distance threshold in mm
 * @returns {Object|null} { position, quaternion, size, rmse } or null
 */
function fitPlaneRANSAC(validPoints, thresholdMM) {
    if (validPoints.length < 3) return null;

    const threshold = thresholdMM / 1000;  // Convert mm to meters
    const iterations = 100;
    let bestInliers = null;
    let bestCount = 0;

    for (let iter = 0; iter < iterations; iter++) {
        // Randomly sample 3 non-duplicate points to define a candidate plane
        const idx = [];
        while (idx.length < 3) {
            const r = Math.floor(Math.random() * validPoints.length);
            if (!idx.includes(r)) idx.push(r);
        }
        const p0 = validPoints[idx[0]];
        const p1 = validPoints[idx[1]];
        const p2 = validPoints[idx[2]];

        // Compute plane normal from cross product of two edge vectors
        const v1 = new THREE.Vector3().subVectors(p1, p0);
        const v2 = new THREE.Vector3().subVectors(p2, p0);
        const normal = new THREE.Vector3().crossVectors(v1, v2);
        if (normal.length() < 1e-10) continue;  // Skip degenerate (collinear) samples
        normal.normalize();

        // Count inliers: points within threshold distance from candidate plane
        const inliers = [];
        for (let i = 0; i < validPoints.length; i++) {
            const diff = new THREE.Vector3().subVectors(validPoints[i], p0);
            const dist = Math.abs(diff.dot(normal));
            if (dist < threshold) inliers.push(i);
        }

        if (inliers.length > bestCount) {
            bestCount = inliers.length;
            bestInliers = inliers;
        }
    }

    if (!bestInliers || bestCount < 3) return null;

    // Refit plane using only the best inlier set (more accurate than 3-point fit)
    const inlierPoints = bestInliers.map(i => validPoints[i]);
    return fitPlaneLS(inlierPoints);
}

/* ==========================================================================
   IMU Quaternion Processing
   ==========================================================================

   Converts BNO085 quaternion data to Three.js orientation:

   Step 1 — IMU-to-ToF frame correction:
     The BNO085 is mounted 90° CCW relative to the VL53L5CX. We apply a
     90° CW correction around Z: corrected = imu_quat * Rotation_z(-90°).
     This matches Python viewer's correct_imu_to_tof_frame().

   Step 2 — Z-up (ENU) to Y-up (Three.js) coordinate remap:
     BNO085 uses Z-up (East-North-Up): X=East, Y=North, Z=Up
     Three.js uses Y-up: X=right, Y=up, Z=backward
     Remap quaternion components: (x, y, z, w) → (x, z, -y, w) */

function correctIMUQuaternion(w, x, y, z) {
    // Step 1: Apply 90° CW correction around Z axis
    // THREE.Quaternion constructor is (x, y, z, w)
    const imuQuat = new THREE.Quaternion(x, y, z, w);
    const correction = new THREE.Quaternion().setFromAxisAngle(
        new THREE.Vector3(0, 0, 1), -Math.PI / 2  // -90° around Z
    );
    imuQuat.multiply(correction);  // corrected = imu * correction (local frame)

    // Step 2: Remap Z-up to Y-up by swapping and negating components
    return new THREE.Quaternion(imuQuat.x, imuQuat.z, -imuQuat.y, imuQuat.w);
}

/* ==========================================================================
   Mapping Mode — World-Space Point Accumulation
   ==========================================================================

   When mapping mode is enabled, each frame's valid points are transformed
   from sensor-local coordinates to world coordinates (using the full
   matrix chain: sensorGroup → boardGroup with IMU rotation → scene root)
   and accumulated in a growing buffer.

   Voxel downsampling prevents unbounded memory growth: points are grouped
   into a 3D grid (voxel size configurable), keeping only the first point
   per voxel. When the total exceeds maxPoints, oldest points are dropped.

   The accumulated map uses a separate THREE.Points at the scene root
   (not inside boardGroup) so points stay fixed in world space. */

/**
 * Transform current frame's points to world coordinates and add to map.
 * Triggers downsampling if point/buffer thresholds are exceeded.
 */
function addToMap(localPoints, colors) {
    const pts = [];
    const cols = [];
    const tempVec = new THREE.Vector3();

    // Ensure world matrices are current before transforming
    sensorGroup.updateMatrixWorld(true);

    for (let i = 0; i < localPoints.length; i++) {
        if (!localPoints[i]) continue;  // Skip invalid zones
        // Transform from sensor-local to world coordinates
        tempVec.copy(localPoints[i]);
        tempVec.applyMatrix4(sensorGroup.matrixWorld);
        pts.push(tempVec.x, tempVec.y, tempVec.z);
        cols.push(colors[i][0], colors[i][1], colors[i][2]);
    }

    if (pts.length === 0) return;

    // Append new frame's points to the accumulation buffers
    mappingBufferPoints.push(new Float32Array(pts));
    mappingBufferColors.push(new Float32Array(cols));
    mappingTotalPoints += pts.length / 3;
    mappingBufferCount++;

    // Check if downsampling is needed (matches Python viewer thresholds)
    const maxPts = getMaxPoints() * 1000;
    if (mappingTotalPoints > DOWNSAMPLE_POINT_THRESHOLD || mappingBufferCount > DOWNSAMPLE_BUFFER_THRESHOLD) {
        downsampleMap(maxPts);
    }

    updateMapGeometry();
}

/**
 * Merge all accumulation buffers and apply voxel downsampling.
 * Reduces point density by keeping only one point per voxel grid cell.
 * Enforces maxPoints limit by dropping oldest points (lowest index = oldest).
 */
function downsampleMap(maxPoints) {
    // Merge all separate frame buffers into single flat arrays
    const totalFloats = mappingBufferPoints.reduce((s, a) => s + a.length, 0);
    const allPts = new Float32Array(totalFloats);
    const allCols = new Float32Array(totalFloats);
    let offset = 0;
    for (let i = 0; i < mappingBufferPoints.length; i++) {
        allPts.set(mappingBufferPoints[i], offset);
        allCols.set(mappingBufferColors[i], offset);
        offset += mappingBufferPoints[i].length;
    }

    const numPts = totalFloats / 3;
    const voxelSize = getVoxelSize() / 1000;  // Convert mm slider value to meters

    // Voxel downsampling: group points by floor(position/voxelSize),
    // keep only the first point encountered per voxel cell
    const voxelMap = new Map();
    for (let i = 0; i < numPts; i++) {
        const vx = Math.floor(allPts[i * 3] / voxelSize);
        const vy = Math.floor(allPts[i * 3 + 1] / voxelSize);
        const vz = Math.floor(allPts[i * 3 + 2] / voxelSize);
        const key = `${vx},${vy},${vz}`;
        if (!voxelMap.has(key)) {
            voxelMap.set(key, i);
        }
    }

    // Collect unique point indices from the voxel map
    let indices = Array.from(voxelMap.values());

    // Enforce max points: sort by index (oldest first) and keep newest
    if (indices.length > maxPoints) {
        indices.sort((a, b) => a - b);
        indices = indices.slice(indices.length - maxPoints);
    }

    // Rebuild compact buffers from the surviving point indices
    const newPts = new Float32Array(indices.length * 3);
    const newCols = new Float32Array(indices.length * 3);
    for (let j = 0; j < indices.length; j++) {
        const i = indices[j];
        newPts[j * 3] = allPts[i * 3];
        newPts[j * 3 + 1] = allPts[i * 3 + 1];
        newPts[j * 3 + 2] = allPts[i * 3 + 2];
        newCols[j * 3] = allCols[i * 3];
        newCols[j * 3 + 1] = allCols[i * 3 + 1];
        newCols[j * 3 + 2] = allCols[i * 3 + 2];
    }

    // Replace all frame buffers with single merged buffer
    mappingBufferPoints.length = 0;
    mappingBufferColors.length = 0;
    mappingBufferPoints.push(newPts);
    mappingBufferColors.push(newCols);
    mappingTotalPoints = indices.length;
    mappingBufferCount = 1;
}

/**
 * Copy accumulated point/color data into the GPU buffer and update draw range.
 * Only copies data that fits within the pre-allocated MAX_MAP_ALLOC buffer.
 */
function updateMapGeometry() {
    let offset = 0;
    for (const buf of mappingBufferPoints) {
        if (offset + buf.length <= mapPositions.length) {
            mapPositions.set(buf, offset);
        }
        offset += buf.length;
    }
    offset = 0;
    for (const buf of mappingBufferColors) {
        if (offset + buf.length <= mapColors.length) {
            mapColors.set(buf, offset);
        }
        offset += buf.length;
    }
    mapGeometry.attributes.position.needsUpdate = true;
    mapGeometry.attributes.color.needsUpdate = true;
    mapGeometry.setDrawRange(0, mappingTotalPoints);
}

/** Clear all accumulated mapping data and reset the display. */
function clearMap() {
    mappingBufferPoints.length = 0;
    mappingBufferColors.length = 0;
    mappingTotalPoints = 0;
    mappingBufferCount = 0;
    mapGeometry.setDrawRange(0, 0);
    document.getElementById('map-point-count').textContent = '0';
}

/* ==========================================================================
   UI Control Wiring
   ==========================================================================

   Connects HTML control panel elements to Three.js state and behavior.
   Each control mirrors the Python Viser viewer's GUI with matching
   defaults, ranges, and dependency logic (e.g., clip rays disabled
   when show rays is off, filter strength disabled when filtering is off). */

/** Read the current coordinate method from the dropdown ('uniform' or 'st') */
function getCoordMethod() {
    return document.getElementById('coord-method').value;
}

/** Read the voxel size slider value in mm */
function getVoxelSize() {
    return parseInt(document.getElementById('voxel-size').value);
}

/** Read the max points slider value in thousands */
function getMaxPoints() {
    return parseInt(document.getElementById('max-points').value);
}

// --- Panel collapse/expand ---
document.getElementById('panel-toggle').addEventListener('click', () => {
    document.getElementById('panel').classList.toggle('collapsed');
});

// --- Section collapse/expand ---
// Each section header has a data-section attribute matching the section body's ID.
// Clicking toggles the 'collapsed' class and rotates the arrow indicator.
document.querySelectorAll('.section-header[data-section]').forEach(header => {
    header.addEventListener('click', () => {
        const body = document.getElementById(header.dataset.section);
        if (body) {
            body.classList.toggle('collapsed');
            header.querySelector('.arrow').style.transform =
                body.classList.contains('collapsed') ? 'rotate(-90deg)' : '';
        }
    });
});

// --- Point Size slider (1-20mm, default 5mm) ---
// Updates both live and map point materials simultaneously
const pointSizeSlider = document.getElementById('point-size');
pointSizeSlider.addEventListener('input', () => {
    const val = parseInt(pointSizeSlider.value);
    document.getElementById('point-size-val').textContent = val + 'px';
    liveMaterial.size = val;
    mapMaterial.size = val;
});

// --- Show Zone Rays checkbox ---
// When unchecked: hides rays and disables + unchecks clip checkbox
const showRaysCB = document.getElementById('show-rays');
const clipRaysCB = document.getElementById('clip-rays');
showRaysCB.addEventListener('change', () => {
    rayLines.visible = showRaysCB.checked;
    clipRaysCB.disabled = !showRaysCB.checked;
    if (!showRaysCB.checked) {
        clipRaysCB.checked = false;
    }
});

// --- Clip to Measurement checkbox ---
// When unchecked: restore full-length rays immediately
clipRaysCB.addEventListener('change', () => {
    if (!clipRaysCB.checked) {
        updateRays(null, null, getCoordMethod(), false);
    }
});

// --- Coordinate Method dropdown ---
// Switching method updates ray directions and recalculates point positions
document.getElementById('coord-method').addEventListener('change', () => {
    const clip = clipRaysCB.checked;
    updateRays(latestDistances, latestStatus, getCoordMethod(), clip);
});

// --- Apply IMU Rotation checkbox ---
// When unchecked: reset board group to identity quaternion (no rotation)
const applyIMUCB = document.getElementById('apply-imu');
applyIMUCB.addEventListener('change', () => {
    if (!applyIMUCB.checked) {
        boardGroup.quaternion.identity();
    }
});

// --- Enable Filtering checkbox + Filter Strength slider ---
// Disabling filter: disables strength slider and resets filter state
const enableFilterCB = document.getElementById('enable-filter');
const filterStrengthSlider = document.getElementById('filter-strength');
enableFilterCB.addEventListener('change', () => {
    filterStrengthSlider.disabled = !enableFilterCB.checked;
    if (!enableFilterCB.checked) {
        filterInitialized = false;  // Reset EMA buffer on next enable
    }
});
filterStrengthSlider.addEventListener('input', () => {
    document.getElementById('filter-strength-val').textContent =
        parseFloat(filterStrengthSlider.value).toFixed(2);
});

// --- Plane Fitting controls ---
// Fit Plane checkbox enables/disables the method dropdown.
// RANSAC threshold row is only visible when method=RANSAC and plane fitting is on.
const fitPlaneCB = document.getElementById('fit-plane');
const planeMethodSel = document.getElementById('plane-method');
const ransacRow = document.getElementById('ransac-row');
const ransacThreshSlider = document.getElementById('ransac-thresh');

fitPlaneCB.addEventListener('change', () => {
    planeMethodSel.disabled = !fitPlaneCB.checked;
    if (!fitPlaneCB.checked) {
        planeMesh.visible = false;
        ransacRow.style.display = 'none';
        document.getElementById('plane-rmse').textContent = '--';
    } else {
        updateRansacVisibility();
    }
});

planeMethodSel.addEventListener('change', updateRansacVisibility);

/** Show/hide the RANSAC threshold slider based on current control state */
function updateRansacVisibility() {
    const show = fitPlaneCB.checked && planeMethodSel.value === 'ransac';
    ransacRow.style.display = show ? 'flex' : 'none';
}

ransacThreshSlider.addEventListener('input', () => {
    document.getElementById('ransac-thresh-val').textContent = ransacThreshSlider.value;
});

// --- Mapping Mode controls ---
// When mapping mode is unchecked: clear accumulated map and restore live points.
// Matches Python viewer behavior where exiting mapping mode discards the map.
const mappingCB = document.getElementById('mapping-mode');
mappingCB.addEventListener('change', () => {
    if (!mappingCB.checked) {
        clearMap();
        livePoints.visible = true;
    }
});

// Voxel size and max points sliders: update display values on change
document.getElementById('voxel-size').addEventListener('input', () => {
    document.getElementById('voxel-size-val').textContent =
        document.getElementById('voxel-size').value;
});
document.getElementById('max-points').addEventListener('input', () => {
    document.getElementById('max-points-val').textContent =
        document.getElementById('max-points').value;
});

// Clear Map button: sets a deferred flag processed in the main frame loop
document.getElementById('btn-clear-map').addEventListener('click', () => {
    clearMapRequested = true;
});

/* ==========================================================================
   SSE Event Handling
   ==========================================================================

   Connects to the ESP32's /api/events SSE endpoint to receive real-time
   sensor data. Three event types are handled:

   - "tof" (~4Hz): ToF sensor frame with 64 distances and status values.
     Triggers the full processing pipeline (filter → convert → render).

   - "imu" (~10Hz): BNO085 quaternion (w, x, y, z) with accuracy.
     Applies frame correction and Z-up→Y-up remap to boardGroup.

   - "device" (1Hz): Device status including IMU ready flag.
     Updates the IMU connection status indicator.

   Auto-reconnects with 2-second delay on connection errors. */

let evtSource = null;

function connectSSE() {
    evtSource = new EventSource('/api/events');

    // ToF sensor data: 64 distance + status values per sensor
    evtSource.addEventListener('tof', (e) => {
        const data = JSON.parse(e.data);
        if (!data.sensors || data.sensors.length === 0) return;

        // Use first sensor (sensor ring may have multiple, but typically one)
        const sensor = data.sensors[0];
        latestDistances = sensor.d;  // 64 distances in mm
        latestStatus = sensor.s;     // 64 status values (5 = valid)

        // Compute data frequency (frames per second) over 1-second windows
        tofFrameCount++;
        const now = performance.now();
        if (now - tofHzTimer > 1000) {
            tofHz = tofFrameCount / ((now - tofHzTimer) / 1000);
            tofFrameCount = 0;
            tofHzTimer = now;
        }

        processFrame();
    });

    // IMU quaternion data: applied to boardGroup for real-time orientation
    evtSource.addEventListener('imu', (e) => {
        const data = JSON.parse(e.data);
        latestIMU = data;
        imuConnected = true;

        if (applyIMUCB.checked) {
            // Apply frame correction and coordinate remap
            const q = correctIMUQuaternion(data.w, data.x, data.y, data.z);
            boardGroup.quaternion.copy(q);
        }
    });

    // Device status: periodic health check including IMU ready flag
    evtSource.addEventListener('device', (e) => {
        const data = JSON.parse(e.data);
        if (data.imuReady !== undefined) {
            imuConnected = data.imuReady;
        }
    });

    // Auto-reconnect on connection loss (ESP32 may reboot, WiFi may drop)
    evtSource.onerror = () => {
        evtSource.close();
        setTimeout(connectSSE, 2000);
    };
}

/* ==========================================================================
   Per-Frame Processing Pipeline
   ==========================================================================

   Called on every ToF SSE event (~10Hz). Executes the full pipeline:
   1. Apply temporal filter (if enabled)
   2. Convert distances to 3D points using selected coordinate method
   3. Compute per-point colors from distance
   4. Handle mapping mode (accumulate) or live mode (update sensor-local points)
   5. Update zone rays (if visible, with optional clipping)
   6. Fit plane (if enabled, LS or RANSAC)
   7. Update sensor info display (range, Hz, IMU status) */

function processFrame() {
    if (!latestDistances || !latestStatus) return;

    let distances = latestDistances;
    const status = latestStatus;
    const method = getCoordMethod();

    // Step 1: Apply temporal filter if enabled
    if (enableFilterCB.checked) {
        const strength = parseFloat(filterStrengthSlider.value);
        distances = applyFilter(new Float32Array(distances), strength);
    }

    // Step 2-3: Convert to 3D points and compute colors
    const points = distancesToPoints(distances, status, method);
    const colors = [];
    for (let i = 0; i < NUM_ZONES; i++) {
        colors.push(distanceColor(distances[i], status[i]));
    }

    // Process any pending clear request before adding new data
    if (clearMapRequested) {
        clearMap();
        clearMapRequested = false;
    }

    // Step 4: Mapping mode vs. live mode
    if (mappingCB.checked) {
        // Mapping: transform points to world coords and accumulate
        addToMap(points, colors);
        livePoints.visible = false;   // Hide sensor-local points
        mapPoints.visible = true;     // Show accumulated map
        document.getElementById('map-point-count').textContent =
            mappingTotalPoints.toLocaleString();
    } else {
        // Live: pack only valid points into the buffer front, skip invalid zones.
        // Uses drawRange to render only the valid subset (matches Python viewer
        // which completely omits invalid zones from the point cloud).
        livePoints.visible = true;
        mapPoints.visible = mappingTotalPoints > 0;  // Keep showing map if it exists

        let validCount = 0;
        for (let i = 0; i < NUM_ZONES; i++) {
            if (points[i]) {
                const idx = validCount * 3;
                livePositions[idx] = points[i].x;
                livePositions[idx + 1] = points[i].y;
                livePositions[idx + 2] = points[i].z;
                liveColors[idx] = colors[i][0];
                liveColors[idx + 1] = colors[i][1];
                liveColors[idx + 2] = colors[i][2];
                validCount++;
            }
        }
        liveGeometry.setDrawRange(0, validCount);
        liveGeometry.attributes.position.needsUpdate = true;
        liveGeometry.attributes.color.needsUpdate = true;
    }

    // Step 5: Update zone rays if visible
    if (showRaysCB.checked) {
        updateRays(distances, status, method, clipRaysCB.checked);
    }

    // Step 6: Plane fitting if enabled
    if (fitPlaneCB.checked) {
        const validPoints = points.filter(p => p !== null);
        let result = null;
        if (planeMethodSel.value === 'ransac') {
            const thresh = parseInt(ransacThreshSlider.value);
            result = fitPlaneRANSAC(validPoints, thresh);
        } else {
            result = fitPlaneLS(validPoints);
        }

        if (result) {
            // Position, orient, and scale the plane mesh to match the fit
            planeMesh.position.copy(result.position);
            planeMesh.quaternion.copy(result.quaternion);
            planeMesh.scale.set(result.size, result.size, 1);
            planeMesh.visible = true;
            document.getElementById('plane-rmse').textContent = result.rmse.toFixed(2);
        } else {
            planeMesh.visible = false;
            document.getElementById('plane-rmse').textContent = '--';
        }
    } else {
        planeMesh.visible = false;
    }

    // Step 7: Update sensor info display
    const validDists = [];
    for (let i = 0; i < NUM_ZONES; i++) {
        if (status[i] === 5 && distances[i] >= MIN_RANGE_MM) {
            validDists.push(distances[i]);
        }
    }
    if (validDists.length > 0) {
        const minD = Math.round(Math.min(...validDists));
        const maxD = Math.round(Math.max(...validDists));
        document.getElementById('info-range').textContent = `Range: ${minD}-${maxD}mm`;
    } else {
        document.getElementById('info-range').textContent = 'No valid data';
    }

    document.getElementById('info-freq').textContent = tofHz.toFixed(1);
    document.getElementById('info-imu').textContent = imuConnected ? 'Connected' : 'Not detected';
}

/* ==========================================================================
   Animation Loop
   ==========================================================================

   Runs at display refresh rate (typically 60fps). Updates OrbitControls
   damping and renders the scene. The actual data processing happens in
   processFrame() triggered by SSE events, not in the animation loop. */

function animate() {
    requestAnimationFrame(animate);
    controls.update();  // Apply damping to orbit controls
    renderer.render(scene, camera);
}

/* ==========================================================================
   Window Resize Handler
   ==========================================================================

   Updates camera aspect ratio and renderer size when the browser window
   is resized. Required to prevent distortion of the 3D view. */

window.addEventListener('resize', () => {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
});

/* ==========================================================================
   Initialization
   ========================================================================== */

connectSSE();   // Start receiving sensor data from ESP32
animate();      // Start the render loop
