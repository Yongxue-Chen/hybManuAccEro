#pragma once
#include "../QMeshLib/PolygenMesh.h"
#include "model3D.h"
#include "fileIO.h"
#include <queue>
#include <set>
#include "AccEro3D.h"
#include <unordered_set>

class AccEro3DNew {
public:
	AccEro3DNew(model3D* modelObj3D, int checkWindow, bool globalSearch, int toolLength);
	~AccEro3DNew() {};

	HMP_struct3D solveByAccEro3d(std::string modelName, Eigen::VectorXi& resultModel, Eigen::VectorXd& timeList, bool opt = true, bool outLog = false, bool outInfo = true);
	HMP_struct3D redundancyCheck(std::string modelName, int range);
	void preProcess(Eigen::VectorXi& resultModel, std::vector<int>& growedList, double& time);


	Eigen::MatrixXi hmp2Matrix(const HMP_struct3D& T, int& threshold) const;

private:
	model3D* modelObj3D = nullptr;

private:
	std::vector<int> checkOverHang();
	void sortOverHang(std::vector<int>& overhangList);
	void sortOverHang(std::vector<Eigen::Vector3i>& overhangList);
	std::vector<int> searchGrowPath(std::vector<Eigen::Vector3i>growList);
	bool isConnect(Eigen::Vector3i pos1, Eigen::Vector3i pos2);

private:

	void AccAndEro(Eigen::VectorXi& Limits, Eigen::MatrixXi& height, std::vector<std::pair<int, int>>& accElements, std::vector<int>& eroElements, Eigen::Vector2i& checkTimeConnect, Eigen::VectorXd& timeList);

	bool addToStable(std::vector<Eigen::Vector3i> posList, Eigen::VectorXi model, Eigen::MatrixXi topStatus, std::vector<std::pair<std::pair<int, int>, int>>& tryAdd, int z, Eigen::Vector2i& checkTimeConnect,
		int seqTime, Eigen::VectorXd& timeList,int boxSize = 2, bool searchMore = true);
	bool grow(std::vector<Eigen::Vector3i> posList, std::vector<std::pair<std::pair<int, int>, int>>& tryAdd, int z, Eigen::VectorXi box);
	int smAccessible(Eigen::Vector3i pos, int height, Eigen::VectorXi model, std::vector<std::pair<std::pair<int, int>, int>> tryAdd);


	std::vector<std::pair<int, int>> Acc(Eigen::VectorXi& Limits, Eigen::MatrixXi& height, int accRange);
	std::vector<int> Ero(Eigen::VectorXi& Limits, Eigen::MatrixXi& height, int eroRange);
	void deAcc(HMP_struct3D& T, std::vector<std::pair<int, int>> AccElement, Eigen::VectorXi& Limits, Eigen::MatrixXi& height, Eigen::VectorXi& checkTime, bool opt);
	void deEro(HMP_struct3D& T, std::vector<int> EroElement, Eigen::VectorXi& Limits, Eigen::MatrixXi& height, Eigen::VectorXi& checkTime, bool opt);

	void addNewTime(HMP_struct3D& T, int tNew, std::pair<int, int> idxAndDir, Eigen::VectorXi& checkTime, int shreshold);
	void addNewTime(HMP_struct3D& T, int tNew, std::vector<int> AMidxList, Eigen::VectorXi& checkTime, int shreshold);
	int checkAMRedundancy(HMP_struct3D& T, int amTime, Eigen::VectorXi& checkTime, int threshold, std::vector<int>& checkedTime, std::pair<int,int> currentPosition);
	int checkConnected(std::vector<int>& idxList, int currentIdx);

	bool changeLastTime(HMP_struct3D& T, int tChange, int idx, Eigen::VectorXi& checkTime, int shreshold);
	bool emptyToSolid(const HMP_struct3D& T, int startCheckTime, int endCheckTime, int idx) const;
	bool solidToEmpty(const HMP_struct3D& T, int startCheckTime, int endCheckTime, int idx) const;


public:
	//int lengthSM = 3;
	//int radiusSM = 100;

	//double tanHalfAngleAM = 10.0 / 11.0;
	//int lengthConeAM = 55;
	//int radiusAM = 50;

	int checkWindow = 2;
	bool globalSearch = false;
	int redunWindow = 1e8;
	//int redunWindow = 4000;


	int lengthSM = 2;

	int radiusSM = 100;
	double tanHalfAngleAM = 10.0;
	int radiusAM = 50;

	//int lengthSM = 50;
	//int radiusSM = 100;
	//double tanHalfAngleAM = 10.0;
	//int radiusAM = 50;


	//double tanHalfAngleAM = 10.0;
	////int lengthConeAM = 55;
	//int radiusAM = 50;

	//int AMMiniSize = 8;
	//int AMMiniGap = 4;

	//int AMMiniSize = 4;
	//int AMMiniGap = 3;
	int AMMiniSize = 1;
	int AMMiniGap = 1;

	int numSolids;

public:
	int toolBoundaryAM(Eigen::Vector3i tipPoint, Eigen::Vector2i xy) const;
	bool isColliFreeAM(Eigen::Vector3i tipPoint, const Eigen::MatrixXi& height) const;


private:
	bool isSupportFree(const HMP_struct3D& T, int idx, int time, int emptyIdx = -1) const;
	bool isColliFreeAM(const HMP_struct3D& T, Eigen::Vector3i tipPoint, int time) const;
	bool hasSolidNeighbor(const HMP_struct3D& T, int time, Eigen::Vector3i pos) const;
	bool isColliFreeSM(const HMP_struct3D& T, int posIdx, int smIdx, Eigen::Vector3i checkPos) const;
	int getSolidFaceNeighborSameHeight(const HMP_struct3D& T, int time, Eigen::VectorXi& statusT, Eigen::Vector3i pos, std::vector<Eigen::Vector3i>& neighbor) const;
	int getStatusFromTime(const HMP_struct3D& T, int time, Eigen::VectorXi& statusT, int idx) const;
	Eigen::VectorXi getModelFromTime(const HMP_struct3D& T, int time, int emptyIdx) const;

	//int toolBoundarySM(Eigen::VectorXi tipPoint, int direction, Eigen::VectorXi checkPos);
	//int isColliFreeSM(Eigen::VectorXi tipPoint, int direction);

public:
	std::vector<Eigen::Vector3i> _getToolPosInRadius(Eigen::Vector3i xyzPos, int toolDirection, double radius);
};