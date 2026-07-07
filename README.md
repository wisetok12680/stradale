# Stradale: Research-Grade Modular Vehicle Dynamics Simulator

Stradale is a high-performance, modular vehicle dynamics physics simulator implemented in modern C++ (UCRT64/MSYS2 toolchain). The project is engineered to act as a high-fidelity synthetic data generator for training **Machine Learning Setup Predictors** and executing **Bayesian Optimization** algorithms. The ultimate objective is to automatically solve for the fastest possible lap setup (ride height, aerodynamic wing angles, spring rates, gear ratios) on any real-world racing circuit.

---

## 📂 Project Structure & Module Layout

The codebase strictly adheres to modular segregation. Subsystems are decoupled (e.g., the `Track` has no knowledge of vehicle dimensions, and the `Engine` has no knowledge of GPS tracking or GPX formats).

```
stradale/
├── .gitignore                   # Excludes binaries, build directories, and JSON simulation logs
├── README.md                    # Core mathematical documentation, build commands, and physics formulations
└── simulator/
    ├── data/
    │   ├── lemans.gpx           # Raw GPS coordinate data for Circuit de la Sarthe
    │   └── monza.gpx            # Raw GPS coordinate data for Autodromo Nazionale Monza
    ├── include/
    │   ├── TrackPoint.hpp       # [DATA] Encapsulates 3D spherical and Cartesian coordinates
    │   ├── TrackSegment.hpp     # [DATA] Stores segment horizontal length, heading, grade, and curvature
    │   ├── Track.hpp            # [DATA] Unified container for track points, segments, and metadata
    │   ├── Engine.hpp           # [DATA] Drivetrain crankshaft torque and power constraints
    │   ├── Vehicle.hpp          # [DATA] Vehicle mass, aero area, tyres, and final drive ratio
    │   ├── TrackLoader.hpp      # [INFRASTRUCTURE] GPX XML parser interface
    │   ├── CoordinateConverter.hpp # [PHYSICS] Geodetic to Cartesian projection interface
    │   ├── TrackAnalyzer.hpp    # [ANALYSIS] Track geometry validation and curvature calculations
    │   └── VehicleDynamics.hpp  # [SIMULATION] Longitudinal integration & lap simulator interface
    ├── src/
    │   ├── GPX_parser.cpp       # [INFRASTRUCTURE] XML node navigation (tinyxml2)
    │   ├── TrackLoader.cpp      # [INFRASTRUCTURE] Filters Monza GP circuit loop (isolating indices)
    │   ├── CoordinateConverter.cpp # [PHYSICS] Projects latitude/longitude to flat Cartesian space
    │   ├── TrackAnalyzer.cpp    # [ANALYSIS] signed circumcircle curvatures & validation
    │   ├── VehicleDynamics.cpp  # [SIMULATION] Drag, rolling resistance, grade forces, & braking limits
    │   └── main.cpp             # [INFRASTRUCTURE] Entry point running Monza & Le Mans lap logs
    └── index.html               # [INFRASTRUCTURE] Motec-style HUD telemetry visualizer dashboard
```

---

## 🧮 Subsystem Class Classifications

*   **Data Classes (`TrackPoint`, `TrackSegment`, `Track`, `Engine`, `Vehicle`):** Structs and containers representing the state of physical entities. They contain no physics calculations or active logic.
*   **Physics Classes (`CoordinateConverter`):** Perform spatial translations and transform geodetic coordinates to local metric scales.
*   **Analysis Classes (`TrackAnalyzer`):** Compute track-specific geometric values (headings, curvature, slopes) and assert physical validity thresholds.
*   **Simulation Classes (`VehicleDynamics`):** Integrate state equations forward in time using numerical methods (Euler integration) subject to tractive and resistive forces.
*   **Infrastructure Classes (`TrackLoader`, `main`, `index.html`):** Handle disk I/O, configuration parsing, console outputs, and visual rendering dashboards.

---

## 📐 Mathematical Formulations & Physics Derivations

### 1. Geodetic to Cartesian Translation (Equirectangular Projection)
To convert raw GPS coordinates (latitude $\phi$, longitude $\lambda$, elevation $z$) to a localized Cartesian framework $(x, y, z)$ in meters, we establish the starting point $P_0(\phi_0, \lambda_0, z_0)$ as the origin $(0, 0, 0)$. 

Latitude spacing remains constant globally, whereas longitude spacing compresses towards the poles as a cosine function of the latitude.
$$x_i = (\lambda_i - \lambda_0) \cdot R_{\text{lat}} \cdot \cos(\phi_0)$$
$$y_i = (\phi_i - \phi_0) \cdot R_{\text{lat}}$$
$$z_i = \text{elevation}_i - z_0$$

Where:
*   $R_{\text{lat}} = 111,320.0 \text{ meters/degree}$ is the physical meridional distance of $1^{\circ}$ of latitude.
*   $\cos(\phi_0)$ scales the parallel radius to adjust the longitude distance relative to the polar coordinate projection.

---

### 2. Segment Geometry Equations
For each segment $i$ connecting consecutive Cartesian nodes $P_i(x_i, y_i, z_i)$ and $P_{i+1}(x_{i+1}, y_{i+1}, z_{i+1})$:

#### Horizontal Segment Length ($L_i$):
$$L_i = \sqrt{(x_{i+1} - x_i)^2 + (y_{i+1} - y_i)^2}$$

#### Heading Angle ($\psi_i$):
$$\psi_i = \text{atan2}(y_{i+1} - y_i, x_{i+1} - x_i)$$
*Result is bounded between $[-\pi, \pi]\text{ rad}$.*

#### Vertical Grade ($g_i$):
$$g_i = \frac{z_{i+1} - z_i}{L_i} \quad (\text{slope value, for } L_i > 0.1\text{ m})$$

#### Signed Curvature ($\kappa_i$):
Curvature ($\kappa = 1/R$) at point $P_i$ is computed using the circumcircle circumscribed by three consecutive points $P_{i-1}$, $P_i$, and $P_{i+1}$.

$$\kappa_i = \frac{2 \cdot \text{double\_area\_signed}}{a \cdot b \cdot c}$$

Where:
*   $a = \|P_i - P_{i-1}\|$ (length of incoming segment)
*   $b = \|P_{i+1} - P_i\|$ (length of outgoing segment)
*   $c = \|P_{i+1} - P_{i-1}\|$ (chord length)
*   $\text{double\_area\_signed} = (x_i - x_{i-1})(y_{i+1} - y_i) - (y_i - y_{i-1})(x_{i+1} - x_i)$ (Z-component of the vector cross product, determining the turn direction: positive for left-hand bends, negative for right-hand bends).

---

### 3. Longitudinal Vehicle Dynamics Model
The simulator advances the vehicle along the track layout using a 1D coordinate system $s$ mapping distance traveled. The net force acting on the vehicle parallel to the track heading is:

$$F_{\text{net}} = F_{\text{engine}} - F_{\text{drag}} - F_{\text{rr}} - F_{\text{grade}}$$

#### Aerodynamic Drag Force ($F_{\text{drag}}$):
$$F_{\text{drag}} = 0.5 \cdot \rho \cdot C_d \cdot A \cdot v^2$$
*   $\rho = 1.225 \text{ kg/m}^3$ (Standard sea-level air density).
*   $C_d$ (Vehicle drag coefficient, e.g., $0.33$).
*   $A$ (Frontal cross-sectional area in $\text{m}^2$, e.g., $2.0$).
*   $v$ (Velocity in $\text{m/s}$).

#### Rolling Resistance Force ($F_{\text{rr}}$):
$$F_{\text{rr}} = C_{\text{rr}} \cdot m \cdot g \cdot \cos(\theta)$$
*   $C_{\text{rr}}$ (Tire rolling resistance coefficient, e.g., $0.015$).
*   $m$ (Vehicle mass, e.g., $1300\text{ kg}$).
*   $g = 9.80665 \text{ m/s}^2$ (Acceleration due to gravity).
*   $\theta = \text{atan}(g_{\text{seg}})$ (Local vertical incline slope angle).

#### Gravitational Grade Force ($F_{\text{grade}}$):
$$F_{\text{grade}} = m \cdot g \cdot \sin(\theta) = m \cdot g \cdot \frac{g_{\text{seg}}}{\sqrt{1 + g_{\text{seg}}^2}}$$
*Positive under climb (resists motion); negative under descent (assists motion).*

#### Engine Tractive Force ($F_{\text{engine}}$):
The powertrain operates under two distinct physical regimes: torque-limited at low speeds and power-limited at high speeds.
$$F_{\text{engine}} = \min\left(F_{\text{max\_tractive}}, \frac{P_{\text{max}}}{v}\right)$$
*   $P_{\text{max}}$ (Maximum engine power, e.g., $370\text{ kW}$).
*   $F_{\text{max\_tractive}}$ (Maximum tractive force at tyre patch derived from engine crankshaft peak torque $T_{\text{max}}$ and final drive ratio):
    $$F_{\text{max\_tractive}} = \frac{T_{\text{max}} \cdot \text{final\_drive\_ratio}}{r_{\text{tyre}}}$$
    *(For $T_{\text{max}} = 500\text{ Nm}$, final drive $= 4.0$, tyre radius $= 0.33\text{ m}$, $F_{\text{max\_tractive}} = 6060.61\text{ N}$).*

---

### 4. Corner Speed Constraints & Progressive Braking

#### Friction Circle Lateral Grip Limit:
Cornering speed is limited by available lateral grip at the tire contact patch.
$$F_{\text{lateral}} = m \cdot a_y = m \cdot \frac{v^2}{R} = m \cdot v^2 \cdot |\kappa|$$
$$F_{\text{lateral, max}} = \mu_{\text{lateral}} \cdot m \cdot g$$

Equating lateral force to the maximum available grip yields the corner speed limit:
$$v_{\text{limit}} = \sqrt{\frac{\mu_{\text{lateral}} \cdot g}{|\kappa|}}$$
*   $\mu_{\text{lateral}} = 1.2$ (Typical GT3 racing tyre friction coefficient).
*   To ignore micro GPS measurement noise on straightaways, this limit is only evaluated when $|\kappa| > 0.002\text{ m}^{-1}$ (radii $< 500\text{ m}$).

#### Progressive Braking Model:
Rather than forcing instantaneous speed drops at corners, the simulator limits deceleration rates:
$$a_{\text{brake, max}} = -1.8 \cdot g \approx -17.65\text{ m/s}^2 \quad (\text{Max GT3 braking limit})$$
$$v_{t+\Delta t} = \max\left(v_{\text{limit}}, v_t + a_{\text{brake, max}} \cdot \Delta t\right)$$
This ensures the vehicle brakes gradually and realistically over several meters before negotiating turns.

---

### 5. Numerical Integration (Euler Step)
The simulator integrates the vehicle state forward in time using a fixed interval $\Delta t = 0.05\text{ s}$:
$$a_t = \frac{F_{\text{net}, t}}{m}$$
$$v_{t+\Delta t} = \max(0.0, v_t + a_t \cdot \Delta t) \quad \text{[subject to braking cap above]}$$
$$s_{t+\Delta t} = s_t + v_t \cdot \Delta t$$

During lap simulations, the vehicle coordinates $(x_t, y_t)$ are interpolated along the active segment $i$ using the fractional displacement factor $\tau$:
$$\tau = \frac{s_t - D_i}{L_i}$$
$$x_t = (1-\tau)x_i + \tau x_{i+1}$$
$$y_t = (1-\tau)y_i + \tau y_{i+1}$$
*(Where $D_i$ is the cumulative distance along the track up to node $i$, and $L_i$ is the segment length).*

---

## 📊 Verification & Lap Simulation Results

Running the vehicle simulation (1300 kg GT3 Spec sports car, 370 kW / 500 Nm Engine) around the parsed track layouts yields the following results:

| Circuit Layout | Length (m) | Loop Closed | Simulated Lap Time | Real-World GT3 Pace |
| :--- | :--- | :--- | :--- | :--- |
| **Monza GP Circuit** | 5,791.39 m | Yes | **1:44.15** (104.15s) | $\approx$ 1:47.00 |
| **Le Mans (Sarthe)** | 13,623.00 m | Yes | **3:51.55** (231.55s) | $\approx$ 3:52.00 |

*Monza GP loop is isolated and parsed by selecting indices `0-39` (Start $\to$ Ascari chicane) and `211-229` (Ascari exit $\to$ Start-Finish) to bypass the historic banking detour.*

---

## 🖥️ Motec-Style Telemetry HUD Dashboard

The visualizer ([index.html](file:///c:/Users/sriva/Desktop/stradale/simulator/index.html)) provides an interactive telemetry terminal using standard web technologies:

1.  **Monospace Race Engineering HUD:** Features digital gauges rendering velocity (km/h), acceleration (G), distance (m), and local track grade.
2.  **Sequential Shift Lights:** Uses 10 colored LED dots (4 Green $\to$ 3 Yellow $\to$ 3 Red) that illuminate sequentially as vehicle speed increases.
3.  **Fast-Forward Playback Loop:** Animates the vehicle at a variable increment to complete a full lap within 15–20 seconds (rather than waiting 4 minutes for a real-time lap).
4.  **Oscilloscope 2D Tracker:** Draws the circuit outline by mapping Cartesian coordinates. The vehicle chassis rotates dynamically to align with the tangent path heading:
    $$\theta_{\text{heading}} = \text{atan2}(-(y_{t+\Delta t} - y_t), x_{t+\Delta t} - x_t)$$
5.  **Force Vector Projections:** Projects real-time force vectors from the center of the car:
    *   **Green Vector:** Engine tractive force $F_{\text{engine}}$ directed forward.
    *   **Red Vector:** Combined resisting forces ($F_{\text{drag}} + F_{\text{rr}} + F_{\text{grade}}$) directed backward.
6.  **Chart Downsampling:** To maintain a smooth 25 FPS rendering loop without chart-redraw lag, logs are downsampled to $\approx 300$ plot points and updated using Chart.js animation-free modes (`update('none')`).

---

## 🛠️ Build and Compile Instructions

### Prerequisites
*   `g++` compiler from MSYS2 UCRT64 toolchain.
*   `tinyxml2` library.

### Compile Command
Navigate to the `simulator/` directory and execute:
```powershell
C:/msys64/ucrt64/bin/g++.exe -O3 src/main.cpp src/TrackLoader.cpp src/CoordinateConverter.cpp src/TrackAnalyzer.cpp src/VehicleDynamics.cpp -ltinyxml2 -o simulator.exe
```

### Run Command
Set the environment path to include MSYS2 runtime binaries before launching:
```powershell
$env:PATH += ';C:\msys64\ucrt64\bin'; .\simulator.exe
```

---

## 🗺️ Future Roadmap

### Stage 1: Spline Smoothing & Resampling
*   Apply cubic splines to smooth GPS coordinates.
*   Resample tracks at uniform intervals ($1\text{ m}$) to eliminate noise in circumcircle curvature calculations.

### Stage 2: Powertrain & Gearbox Model
*   Introduce dynamic engine RPM calculation based on wheel speed and transmission gear ratios.
*   Model clutch heat loss and mechanical drivetrain efficiency losses.

### Stage 3: tyres & Weight Transfer
*   Integrate the **Pacejka 89/94 tyre model** for longitudinal force calculation:
    $$F_x = D \sin(C \arctan(B \kappa - E(B \kappa - \arctan(B \kappa))))$$
*   Compute dynamic tyre vertical loads ($F_z$) under acceleration and braking (longitudinal weight transfer).

### Stage 4: Bayesian Setup Predictor
*   Link C++ simulator to a Python script using Gaussian Processes to automatically optimize suspension spring rates, ride heights, damper coefficients, and wing angles for minimum lap times.
