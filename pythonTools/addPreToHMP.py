from hmp2VTK import readHMPFile
import numpy as np

if __name__ == "__main__":
    hmpFile = "C:/Users/j46868yc/Desktop/saves/TPMS50_afterPre10_AccEroHMP_AMSize4.txt"
    tEnd, HMP = readHMPFile(hmpFile)
    growListFile = "C:/Users/j46868yc/Desktop/saves/TPMS50_tool10_coreGrowList.txt"
    growList = np.genfromtxt(growListFile, delimiter=",", dtype=int)
    diffChange = 0.5 / len(growList)
    for idx in range(len(growList)):
        i = growList[idx]
        if HMP[i, 1] < tEnd:
            print("Error: HMP[i,1] < tEnd")
        HMP[i, 1] = tEnd - diffChange * (idx + 1)
    newHmpFile = (
        "C:/Users/j46868yc/Desktop/saves/TPMS50_tool10_AMSize4_addPre_AccEroHMP.txt"
    )
    with open(newHmpFile, "w") as f:
        f.write(f"{tEnd}\n")
        np.savetxt(f, HMP, delimiter=",", fmt="%.6f")
    print("HMP file saved as", newHmpFile)
