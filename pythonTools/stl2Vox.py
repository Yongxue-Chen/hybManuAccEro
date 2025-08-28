import pyvista as pv
import numpy as np


def stl2VoxNew(stl_file, resolution=100):
    # 1. 读取 STL 文件
    mesh = pv.read(stl_file)
    mesh = mesh.extract_geometry()  # 确保提取几何数据

    # 2. 获取 AABB (Axis-Aligned Bounding Box)
    bounds = mesh.bounds
    print("AABB Bounds:", bounds)

    maxBounds = bounds[1] - bounds[0]
    maxBounds = max(maxBounds, bounds[3] - bounds[2])
    maxBounds = max(maxBounds, bounds[5] - bounds[4])
    spaceIdx = maxBounds / resolution
    print("spaceIdx:", spaceIdx)

    # 3. 设置体素分辨率
    # spaceIdx=10.0
    spacing = [spaceIdx, spaceIdx, spaceIdx]  # 可以调整为更精细的分辨率

    # 4. 计算体素网格的维度
    dims = [
        max(int((bounds[1] - bounds[0]) / spacing[0]) + 1, 1),
        max(int((bounds[3] - bounds[2]) / spacing[1]) + 1, 1),
        max(int((bounds[5] - bounds[4]) / spacing[2]) + 1, 1),
    ]
    # dims = [
    #     max(int((bounds[1] - bounds[0]) / spacing[0]), 1),
    #     max(int((bounds[3] - bounds[2]) / spacing[1]), 1),
    #     max(int((bounds[5] - bounds[4]) / spacing[2]), 1),
    # ]
    print("Voxel Grid Dimensions:", dims)

    # 5. 生成体素化 ImageData
    grid = pv.ImageData(dimensions=dims, spacing=spacing)
    grid.origin = (bounds[0], bounds[2], bounds[4])

    # 6. 计算隐式距离场
    grid.compute_implicit_distance(mesh, inplace=True)
    dist = grid.point_data["implicit_distance"]

    # 7. 标记体素：dist < 0 代表内部
    inner = dist <= 0  # 生成 0/1 矩阵

    voxel_matrix = inner.reshape(dims[2], dims[1], dims[0])  # 转换为 3D 矩阵

    cropped_voxel = voxel_matrix.copy()
    coords = np.argwhere(voxel_matrix)  # 获取所有 1-体素的索引
    min_coords = coords.min(axis=0)  # 计算 X, Y, Z 的最小坐标
    # min_coords[0] = 1
    print("min_coords:", min_coords)
    max_coords = coords.max(axis=0)  # 计算 X, Y, Z 的最大坐标
    print("max_coords:", max_coords)
    cropped_voxel = voxel_matrix[
        min_coords[0] : max_coords[0] + 1,
        min_coords[1] : max_coords[1] + 1,
        min_coords[2] : max_coords[2] + 1,
    ].astype(int)

    print("Voxel Matrix Shape:", cropped_voxel.shape)

    # 8. 可视化
    # p = pv.Plotter()
    # p.add_mesh(mesh, color="lightblue", opacity=0.5)  # STL 网格
    # p.add_volume(grid, cmap="coolwarm", opacity="sigmoid")  # 体素化结果
    # p.show()

    return cropped_voxel

def writeBoard(board, filename):
    boardSize = board.shape
    vecBoard = board.flatten()
    with open(filename, "w") as f:
        f.write(
            str(boardSize[2])
            + ","
            + str(boardSize[1])
            + ","
            + str(boardSize[0])
            + "\n"
        )
        f.write("0,0,0\n")
        for i in range(len(vecBoard) - 1):
            f.write(str(int(vecBoard[i])) + ",")
        f.write(str(int(vecBoard[-1])) + "\n")

if __name__ == "__main__":
    board=stl2VoxNew("../stlFile/bracket.stl", resolution=450)
    writeBoard(board, "TPMS450.txt")
