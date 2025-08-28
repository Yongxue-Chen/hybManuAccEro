import numpy as np
from stl import mesh
from hmp2VTK import readModelFile

modelFile = "C:/Users/j46868yc/Desktop/hybManuAccEro/DataSet/inputVoxModel/TO_noNeedSupport.txt"
nx, ny, nz, model = readModelFile(modelFile)
voxelSize = 1.2  # 体素大小


def voxelState(x, y, z, model):
    idx = z * nx * ny + y * nx + x
    if idx >= len(model):
        raise IndexError("Index out of bounds")
    return model[idx]


# 立方体顶点（局部坐标）
cube_verts = (
    np.array(
        [
            [0, 0, 0],
            [1, 0, 0],
            [1, 1, 0],
            [0, 1, 0],
            [0, 0, 1],
            [1, 0, 1],
            [1, 1, 1],
            [0, 1, 1],
        ]
    )
    * voxelSize
)

# 每个面由两个三角形组成，定义面（三角形）的顶点索引
faces = [
    [0, 3, 1],
    [1, 3, 2],  # bottom
    [4, 5, 7],
    [5, 6, 7],  # top
    [0, 1, 4],
    [1, 5, 4],  # front
    [1, 2, 5],
    [2, 6, 5],  # right
    [2, 3, 6],
    [3, 7, 6],  # back
    [3, 0, 7],
    [0, 4, 7],  # left
]

# 相邻偏移量（用于去除内部面）
neighbors = {
    "bottom": (0, 0, -1),
    "top": (0, 0, 1),
    "front": (0, -1, 0),
    "back": (0, 1, 0),
    "left": (-1, 0, 0),
    "right": (1, 0, 0),
}
face_indices = {
    "bottom": [0, 1],
    "top": [2, 3],
    "front": [4, 5],
    "right": [6, 7],
    "back": [8, 9],
    "left": [10, 11],
}

triangles = []

# 遍历所有体素
for x in range(nx):
    for y in range(ny):
        for z in range(nz):
            if voxelState(x, y, z, model) == 1:
                base = np.array([x, y, z]) * voxelSize
                cube = cube_verts + base
                # 只生成暴露的面
                for dir, (dx, dy, dz) in neighbors.items():
                    nx_, ny_, nz_ = x + dx, y + dy, z + dz
                    if 0 <= nx_ < nx and 0 <= ny_ < ny and 0 <= nz_ < nz:
                        if voxelState(nx_, ny_, nz_, model) == 1:
                            continue  # 相邻 voxel 也占用，不画这面
                    for fi in face_indices[dir]:
                        triangle = [cube[i] for i in faces[fi]]
                        triangles.append(triangle)

# 转 numpy 格式写入 STL
triangles_np = np.array(triangles)
model = mesh.Mesh(np.zeros(triangles_np.shape[0], dtype=mesh.Mesh.dtype))
for i, t in enumerate(triangles_np):
    model.vectors[i] = t

model.save("voxel_model.stl")
print("STL 文件已保存为 voxel_model.stl")
