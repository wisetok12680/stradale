# Stradale: Research-Grade Modular Vehicle Dynamics Simulator

Stradale is a high-performance, modular physics simulator designed in modern C++ to predict lap times and optimize race car setup parameters using Bayesian Optimization and Machine Learning. 

It is designed to model accurate track geometries, vehicle dynamics, and tyre load transfer profiles rather than simple arcade physics.

---

## 🏎️ Simulator Pipeline Architecture

The geometry pipeline follows SOLID principles, separating responsibilities into dedicated modular steps:

```
    [GPX Track File]
           │
           ▼
     [TrackLoader]  ◄─── Performs index-based GP loop filtering & closure
           │ (Returns Track object)
           ▼
 [CoordinateConverter] ◄─── Equirectangular projection to local Cartesian
           │ (Modifies in-place)
           ▼
   [TrackAnalyzer]  ◄─── Computes segment lengths, heading angles, and grades
```

### Module Breakdown:
1. **`Track` Class ([Track.hpp](file:///c:/Users/sriva/Desktop/stradale/simulator/include/Track.hpp)):** The central data model that encapsulates all track-related data, passing a unified state through the pipeline rather than loose vectors.
2. **`TrackLoader` ([TrackLoader.cpp](file:///c:/Users/sriva/Desktop/stradale/simulator/src/TrackLoader.cpp)):** Uses `tinyxml2` to parse GPS tracks. Includes custom heuristics to filter overlapping multi-lap sessions or detour data (e.g. Monza's southern park detour) to isolate a single clean GP loop.
3. **`CoordinateConverter` ([CoordinateConverter.cpp](file:///c:/Users/sriva/Desktop/stradale/simulator/src/CoordinateConverter.cpp)):** Projects spherical lat/lon coordinates into Cartesian coordinates centered on the starting line.
4. **`TrackAnalyzer` ([TrackAnalyzer.cpp](file:///c:/Users/sriva/Desktop/stradale/simulator/src/TrackAnalyzer.cpp)):** Computes segment metrics, performs geometric validations (steep slopes, degenerate segments), and tracks loop closure.

---

## 🧮 Mathematical Formulations

### 1. Coordinate Conversion (Equirectangular Projection)
For each track point $i$ with latitude $\phi_i$, longitude $\lambda_i$, and elevation $z_i$:
$$x_i = (\lambda_i - \lambda_0) \cdot R_{\text{lat}} \cdot \cos(\phi_0)$$
$$y_i = (\phi_i - \phi_0) \cdot R_{\text{lat}}$$
$$z_i = \text{elevation}_i$$

Where:
- $\phi_0, \lambda_0$ are the coordinates of the starting point (index 0).
- $R_{\text{lat}} = 111,320.0 \text{ m/degree}$ is the physical length of one degree of latitude.
- $\cos(\phi_0)$ scales the longitude degree width relative to the origin latitude.

### 2. Segment Properties (Phase 1 Geometry)
For a segment $i$ connecting $P_i(x_i, y_i, z_i)$ to $P_{i+1}(x_{i+1}, y_{i+1}, z_{i+1})$:
*   **Horizontal Length ($L_i$):**
    $$L_i = \sqrt{(x_{i+1} - x_i)^2 + (y_{i+1} - y_i)^2}$$
*   **Heading Angle ($\psi_i$):**
    $$\psi_i = \text{atan2}(y_{i+1} - y_i, x_{i+1} - x_i)$$
*   **Grade Slope ($g_i$):**
    $$g_i = \frac{z_{i+1} - z_i}{L_i} \quad (\text{if } L_i > 10^{-6} \text{ m})$$

---

## 🛠️ Build and Execution Instructions

The project is compiled using the `g++` compiler on Windows (MSYS2 toolchain, linking against `tinyxml2`).

### 1. Build Compilation Command
Run the following command in the `simulator/` directory:
```powershell
C:/msys64/ucrt64/bin/g++.exe -O3 src/main.cpp src/TrackLoader.cpp src/CoordinateConverter.cpp src/TrackAnalyzer.cpp -ltinyxml2 -o simulator.exe
```

### 2. Execution Prefix (DLL path binding)
To ensure system DLLs load properly, append MSYS2 runtime bin directory to your environment path when running:
```powershell
$env:PATH += ';C:\msys64\ucrt64\bin'; .\simulator.exe
```

---

## 📊 Verification & Diagnostics

### 1. Le Mans (Circuit de la Sarthe)
*   **Points / Segments:** 233 / 232
*   **Calculated Length:** **13,623.00 m** (99.97% match with actual 13.626 km layout)
*   **Closed Loop:** Yes (detected successfully)
*   **Grade Bounds:** $[-7.44\%, 9.57\%]$ slope

### 2. Monza (Autodromo Nazionale Monza GP Loop)
Monza's raw GPX contains overlapping loops and a southern detour. The isolated loop is extracted by keeping indexes `0-39` (Start $\to$ Ascari entrance) and `211-229` (Ascari exit $\to$ Start-Finish), and then looping back to `0`.
*   **Points / Segments:** 60 / 59
*   **Calculated Length:** **5,791.39 m** (99.97% match with actual 5.793 km GP layout)
*   **Closed Loop:** Yes (detected successfully)
*   **Grade Bounds:** $[-3.00\%, 11.44\%]$ slope
*   **Validation:** `PASSED. Geometry is physically sound.`

---

## 🗺️ Roadmap
- **Stage 2:** Implement signed track curvature ($1/R$) using three-point circumcircle calculations and cross-slope banking profile projection.
- **Stage 3 (Vehicle Dynamics):** Load transfer calculations, multi-body suspensions, tyre slip angles, and Bayesian engine optimization.
