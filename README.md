# Can Any Model Be Fabricated? Inverse Operation Based Planning for Hybrid Additive–Subtractive Manufacturing

Yongxue Chen, Tao Liu, Yuming Huang, Weiming Wang, Tianyu Zhang, Kun Qian, Zikang Shi, Charlie C.L. Wang "[Can Any Model Be Fabricated? Inverse Operation Based Planning for Hybrid Additive–Subtractive Manufacturing]()", ACM Transactions on Graphics, 2025.
[[Arxiv Paper]()] [[Video]()]

## Overview
hybManuAccEro implements inverse-operation-based planning for hybrid additive–subtractive manufacturing (AM+SM). It takes a voxelized model as input and outputs a time-annotated per-voxel schedule for AM and SM, suitable for downstream toolpath generation.

## Installation

### Option A: Prebuilt Binary (Release)
1. Download the `.zip` from Releases.
2. Extract the archive.
3. Double-click `ShapeLab.exe` in the `hybManuAccEro` folder to start.

### Option B: Build from Source
- Open the QMake project `ShapeLab.pro` with the Qt Visual Studio Tools extension in Visual Studio.

#### Tested Environment
- Windows
- Visual Studio 2022
- Qt 5.14.2 (toolchain: `msvc2017_64`)

#### Build Steps
1. Install Visual Studio (tested: VS2022).
2. Install Qt (tested: Qt 5.14.2 with `msvc2017_64`).
3. Install and configure the “Qt Visual Studio Tools” extension.
4. Use Qt VS Tools to open `ShapeLab.pro` and generate the solution.
5. Set `ShapeLab` as the Startup Project.
6. Ensure the Platform Toolset matches your environment for all three projects: `GLKLib`, `QMeshLib`, and `ShapeLab`  
   - Property Pages -> Configuration Properties -> General -> Platform Toolset.
7. Set the C++ language standard to C++17 for all three projects.
   - Property Pages -> Configuration Properties -> General -> C++ Language Standard.
8. Enable OpenMP for best performance:  
   - `ShapeLab` -> C/C++ -> Language -> Open MP Support -> Yes (/openmp).
9. Enable console subsystem (for logs):  
   - `ShapeLab` -> Linker -> System -> SubSystem -> Console (/SUBSYSTEM:CONSOLE).
10. Increase stack reserve size (recommended):  
    - `ShapeLab` -> Linker -> System -> Stack Reserve Size -> `8388608`.

## Usage

1. Place your voxel model file in `DataSet/inputVoxModel/`.
2. In the UI (right panel, Main Page, `model`), enter or select the model filename.
3. Keep `inputHMPTpye` set to `AccEro`.
4. Configure parameters:
   - `checkWindow`: local stability check window size.
   - `toolLength`: tool length.
   - `preProcess` checkbox: enable (recommended).
5. Click `accEro`.
6. The generated schedule will be saved to `DataSet/outputTime3D/` with filename:  
   `<modelName>_afterPre<toolLength>_tool<toolLength>_local<checkWindow>_AccEroHMP.txt`.
7. Users can also use button `accEroBatch` to batch generate results. Simply place all target voxel model files in `DataSet/inputVoxModel/` and configure parameters for each model in the `DataSet/outputTime3D/testList.txt` file.
8. Whether using the `accEro` button or `accEroBatch` button, the processing time required for each model and various result information will be saved in `DataSet/log/log.txt`.

### Voxel Model File Format
- File type: `<modelName>.txt`
- Line 1: `nx,ny,nz` — voxel counts along x, y, z.
- Line 2: `0,0,0`.
- Line 3: A flattened 0/1 vector of length `nx * ny * nz` representing the 3D occupancy:
  - `1` = solid (occupied), `0` = empty.
  - Ordering rule: increasing z first; within the same z, increasing y; within the same y and z, increasing x.

### Output Schedule File Format
- Line 1: `tEnd` — operations at times greater than `tEnd` are invalid and should be ignored.
- Lines 2..end: for each voxel (in the same ordering as the input voxel file), provide two timestamps per line:
  - First = AM (additive) time.
  - Second = SM (subtractive) time.
- Note: Times greater than `tEnd` are considered invalid. Use this schedule with your specific machine to generate toolpaths or perform subsequent operations.

### testList.txt File Format (for Batch Processing)
- File location: `DataSet/outputTime3D/testList.txt`
- Each line represents one model with 4 space-separated parameters:
  - `a b c d`
  - `a`: model name (string, without .txt extension)
  - `b`: tool length (int)
  - `c`: checkWindow value (int)
  - `d`: preProcess option (int, 1 = enabled, 0 = disabled)
- Example:
  ```
  bunny 10 5 1
  sphere 15 3 0
  cube 8 4 1
  ```

## Contact
- Yongxue Chen: yongxue.chen@postgrad.manchester.ac.uk  
- Charlie C.L. Wang: charlie.wang@manchester.ac.uk

## Notes
- Platform: Windows + Visual Studio + Qt (tested: VS2022 + Qt 5.14.2 + `msvc2017_64`).
- Build settings: C++17, OpenMP enabled, console subsystem enabled, increased stack reserve size.

## Citation
If you use ShapeLab in academic work, please cite the corresponding paper.
