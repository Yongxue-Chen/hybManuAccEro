#pragma once
#include "../QMeshLib/PolygenMesh.h"
#include <deque>

class model3D {
public:
	model3D();
	~model3D() {};
	void init(Eigen::Vector3i size, Eigen::VectorXi matrixValue, Eigen::VectorXi outerStatus, bool pureCoreModel = false);
	void resetVoxels();

public:
	int nx, ny, nz;
	int nSpace;
	
	int xBrim, yBrim, zBrim;
	int outerBrim;
	Eigen::VectorXi limitActionSpace;// xl, xh, yl, yh, zl, zh
	int nActionSpace;

	std::vector<Eigen::Vector2i> solidBrimX1, solidBrimX2, solidBrimY1, solidBrimY2;
	std::vector<Eigen::Vector2i> solidOuterX1, solidOuterX2, solidOuterY1, solidOuterY2;

	Eigen::Vector4i solidLimit;

public:
	double preProcessTime = 0;

public:
	int supportSize = 3;
	std::vector<Eigen::Vector3i> neighborDir;

public:
	int coordinateToIndex(Eigen::Vector3i pos);
	Eigen::Vector3i indexToCoordinate(int index);

public:
	Eigen::VectorXi targetModelValue;
	Eigen::VectorXi currentModelValue;
	Eigen::VectorXi onlyBrimValue;
	Eigen::VectorXi startModelValue;
	
public:
	bool isEmpty();
	Eigen::VectorXi getSolidLimit(Eigen::MatrixXi& height);
	bool hasSolidNeighbor(Eigen::Vector3i pos);
	void getSolidBrimPos();
	double distToSolidBrim(Eigen::Vector2i pos, int brimType);
	bool isSupportingFree(Eigen::Vector3i pos);
	int getSolidFaceNeighborSameHeight(Eigen::Vector3i pos, std::vector<Eigen::Vector3i>& neighbor);
	bool stillStable(Eigen::Vector3i pos, Eigen::VectorXi model = Eigen::VectorXi(0));
	bool stillStable(std::vector<Eigen::Vector3i> posList, Eigen::VectorXi model = Eigen::VectorXi(0));

public:
	void DFS(Eigen::Vector3i pos, int componentIdx, const Eigen::VectorXi& modelNew, Eigen::VectorXi& visited,
		std::vector<Eigen::Vector3i>& queue, int xl, int xh, int yl, int yh, int zl, int zh);

public: 
	Eigen::VectorXi expandModel(Eigen::VectorXi model, Eigen::Vector3i& size, int expansionSize);
	Eigen::VectorXi wrapModel(Eigen::VectorXi model, Eigen::Vector3i& size, std::vector<int>& wrapIdx);
	Eigen::VectorXi augmentModel(Eigen::VectorXi model, Eigen::Vector3i size, Eigen::Vector3i startPos, Eigen::Vector3i coreSize, Eigen::Vector3i shellSize);

public:
	Eigen::MatrixXd HMP;
	double tEnd;

};
