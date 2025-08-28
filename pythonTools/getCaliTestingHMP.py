import numpy as np


def coordinateToIdx(coordinate, modelSize):
    nx, ny, nz = modelSize
    x, y, z = coordinate
    return x + y * nx + z * nx * ny


def idxToCoordinate(idx, modelSize):
    nx, ny, nz = modelSize
    x = idx % nx
    y = idx // nx % ny
    z = idx // (nx * ny)
    coordinate = (x, y, z)
    return coordinate


# Define the model size
nx = 100
ny = 20
nz = 3
modelSize = (nx, ny, nz)

model = np.zeros((1, nx * ny * nz))

# Define the tP for AM
tEnd = 9
tP = np.ones((nx * ny * nz, 2))
tP = tP * (tEnd + 1)
coordList = []
for z in range(0, 2):
    for x in range(0, nx):
        coordList.append((x, 0, z))
        coordList.append((x, 1, z))
        coordList.append((x, ny - 2, z))
        coordList.append((x, ny - 1, z))
    for y in range(2, ny - 2):
        coordList.append((0, y, z))
        coordList.append((1, y, z))
        coordList.append((nx - 2, y, z))
        coordList.append((nx - 1, y, z))

for x in range(2, nx - 2):
    coordList.append((x, 5, 0))
for y in range(2, 5):
    coordList.append((5, y, 0))
for y in range(6, ny - 2):
    coordList.append((5, y, 0))

coordList.append((5, 4, 1))
coordList.append((5, 5, 1))
coordList.append((5, 6, 1))
coordList.append((4, 5, 1))
coordList.append((6, 5, 1))

coordList.append((5, 0, 2))
coordList.append((5, ny - 1, 2))
coordList.append((0, 5, 2))
coordList.append((nx - 1, 5, 2))

for coord in coordList:
    idx = coordinateToIdx(coord, modelSize)
    tP[idx, 0] = 1
    model[0, idx] = 1

coordList = []
coordList.append((5, 5, 1))
coordList.append((5, 0, 1))
coordList.append((5, ny - 1, 1))
coordList.append((0, 5, 1))
coordList.append((nx - 1, 5, 1))

coordList.append((0, 10, 1))
coordList.append((10, 0, 1))
coordList.append((nx - 1, 10, 1))
coordList.append((10, ny - 1, 1))

for coord in coordList:
    idx = coordinateToIdx(coord, modelSize)
    tP[idx, 1] = 3
    model[0, idx] = 0

# save tP to file
with open("../../c++/DataSet/outputTime3D/testExample_tmpHMP.txt", "w") as f:
    f.write(f"{tEnd}\n")  # 写入tEnd
    np.savetxt(f, tP, fmt="%.2f", delimiter=",")  # 写入tP，从第二行开始

with open("../../c++/DataSet/outputTime3D/testExample.txt", "w") as f:
    f.write(f"{nx},{ny},{nz}\n")
    f.write("0,0,0\n")
    np.savetxt(f, model, fmt="%d", delimiter=",")
