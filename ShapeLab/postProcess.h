#pragma once
#include "../QMeshLib/PolygenMesh.h"
#include <Eigen/Sparse>
#include <Eigen/Dense>
#include "fileIO.h"
#include "AccEro3DNew.h"
#include "model3D.h"
#include <vector>
#include <unordered_set>
#include <queue>
#include <algorithm>
#include <iostream>

struct matrixElement {
	int row;
	int col;
	double value;
};

struct opeGroup {
	std::vector<Eigen::Vector3i> voxelPos;
	int opeType = -1;
	Eigen::Vector3i rangeLimitLow = -1 * Eigen::Vector3i::Ones();
	Eigen::Vector3i rangeLimitHigh = -1 * Eigen::Vector3i::Ones();
};

struct opeGroupRough {
	std::vector<std::pair<Eigen::Vector3i, int>> voxelPos;
};

struct twoDimGroup {
	std::vector<std::pair<Eigen::Vector2i, bool>> voxels;
	int opeType = -1;
	Eigen::Vector2i rangeLimitLow = -1 * Eigen::Vector2i::Ones();
	Eigen::Vector2i rangeLimitHigh = -1 * Eigen::Vector2i::Ones();
};

struct pathGroup {
	std::vector<Eigen::MatrixXd> path;
	int opeType = -1;
};

struct pathGroup2D {
	std::vector<Eigen::MatrixXd> path1;
	std::vector<Eigen::MatrixXd> path2;
	int opeType = -1;
};

struct Vector3iHash {
	std::size_t operator()(const Eigen::Vector3i& v) const {
		return std::hash<int>()(v.x()) ^ std::hash<int>()(v.y()) ^ std::hash<int>()(v.z());
	}
};

struct Vector3iEqual {
	bool operator()(const Eigen::Vector3i& a, const Eigen::Vector3i& b) const {
		return a.x() == b.x() && a.y() == b.y() && a.z() == b.z();
	}
};

struct subVoxel {
	Eigen::Vector2d position;
	double distToBoundary;
	bool hasSupport;
};

struct node2D {
	Eigen::Vector2i coord;
	int idx;
	int distToBoundary = 1e5;
	std::vector<int> voxelIdx;
};


class postProcess {

public:
	postProcess(model3D* _model3D, commandParas _cmParas, Eigen::Vector2d _zeroInCenterFrame) : modelObj(_model3D), cmParas(_cmParas), zeroInCenterFrame(_zeroInCenterFrame) {};
	~postProcess() {};

	bool justForShowing = false;

	void tP2PathFile(const Eigen::MatrixXd& tP, double tEnd, const std::string& modelName);

private:
	model3D* modelObj;

private:
	Eigen::MatrixXi tP2Sequnce(const Eigen::MatrixXd& tP, double tEnd);
	int checkSMAccess(const Eigen::MatrixXi& model, int idx);
	std::vector<opeGroup> sequence2Group(const Eigen::MatrixXi& sequence);
	int getIntersect(int idx1, int idx2);
	bool isConnect(opeGroup group, Eigen::Vector3i pos);
	void updateGroupRange(opeGroup& group, Eigen::Vector3i pos);
	std::vector<opeGroup> splitSMGroup(opeGroup group);
	std::vector<opeGroup> splitGroupIntoConnected(opeGroup group);
	std::vector<opeGroup> splitGroupIntoSameHeight(opeGroupRough group);
	std::vector<opeGroup> splitGroupIntoSameOpe(opeGroupRough group);
	bool inSameLayer(opeGroup& group, Eigen::Vector3i pos, int opeType);
	std::vector<pathGroup> voxelGroup2PathGroup(const std::vector<opeGroup>& groupSet);
	twoDimGroup three2twoDimGroup(const opeGroup& groupSet, const Eigen::VectorXi& model);
	pathGroup2D voxelGroup2D2PathGroup2D(twoDimGroup group2D);
	pathGroup twoDimPath2ThreeDimPath(const pathGroup2D& pGroup2D, const opeGroup& vGroup, int maxHeight);
	std::vector<Eigen::Vector3i> getNeighbors(const Eigen::Vector3i& voxel);
	bool canMerge(const Eigen::RowVectorXd& row1, const Eigen::RowVectorXd& row2, double threshold, int idx3);
	void pathGroup2CommandFile(std::vector<pathGroup>& pGroup, const std::string& modelName, bool splitDiffOpe = true);
	int getDistToBoundary(std::vector<Eigen::Vector4i>& distToBoundary, const twoDimGroup& group2D, int idx, int direction);
	bool neighborSubVoxel(const subVoxel& subVoxel1, const subVoxel& subVoxel2, double threshold);
	std::vector<subVoxel> getSubVoxelAndDist(const twoDimGroup& group2D, double pathWidth);
	std::vector<opeGroup> adjustGroupSet(std::vector<opeGroup> groupSet);

private:
	int getCommandLengthAndMoveToCenter(pathGroup& pGroup);

private:
	double voxelSize = 1.2;
	double nozzelD = 0.6;
	//double cutterD = 1.0;
	//double cutterD = 0.8;
	//double cutterD = 0.75;
	double cutterD = 1.18;

	double layerHeightAM = 0.6;
	double layerHeightSM = 0.2;

private:
	int commandLengthLimit = 100000;
	commandParas cmParas;
	Eigen::Vector2d zeroInCenterFrame;
	

};