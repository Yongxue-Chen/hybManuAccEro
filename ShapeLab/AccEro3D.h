#pragma once
#include "../QMeshLib/PolygenMesh.h"
#include "model3D.h"
#include "fileIO.h"

#define PI		3.141592654
#define DEGREE_TO_ROTATE(x)		0.0174532922222*x
#define ROTATE_TO_DEGREE(x)		57.295780490443*x

struct HMP_struct3D {
	std::vector<std::vector<std::pair<int, int>>> Time;
	int tLast;

	HMP_struct3D(int size) {
		Time.resize(size);
		tLast = 0;
	}
};

class AccEro3D {
public:
	AccEro3D(model3D* modelObj3D) : modelObj3D(modelObj3D) {};
	~AccEro3D() {};

	HMP_struct3D solveByAccEro3d(Eigen::VectorXi& resultModel, bool opt = true, bool outLog = false, bool outInfo = true);
	Eigen::MatrixXi hmp2Matrix(const HMP_struct3D& T, int& threshold) const;

private:
	model3D* modelObj3D = nullptr;

private:
	std::vector<std::pair<int, int>> Acc(Eigen::VectorXi& Limits, Eigen::MatrixXi& height, int accRange);
	std::vector<int> Ero(Eigen::VectorXi& Limits, Eigen::MatrixXi& height, int eroRange);
	void deAcc(HMP_struct3D& T, std::vector<std::pair<int, int>> AccElement, Eigen::VectorXi& Limits, Eigen::MatrixXi& height, Eigen::VectorXi& checkTime, bool opt);
	void deEro(HMP_struct3D& T, std::vector<int> EroElement, Eigen::VectorXi& Limits, Eigen::MatrixXi& height, Eigen::VectorXi& checkTime, bool opt);

	bool addNewTime(HMP_struct3D& T, int tNew, std::pair<int, int> idxAndDir, Eigen::VectorXi& checkTime, int shreshold);
	bool changeLastTime(HMP_struct3D& T, int tChange, int idx, Eigen::VectorXi& checkTime, int shreshold);
	bool emptyToSolid(const HMP_struct3D& T, int startCheckTime, int endCheckTime, int idx) const;
	bool solidToEmpty(const HMP_struct3D& T, int startCheckTime, int endCheckTime, int idx) const;


public:
	//int lengthSM = 3;
	//int radiusSM = 100;

	//double tanHalfAngleAM = 10.0 / 11.0;
	//int lengthConeAM = 55;
	//int radiusAM = 50;

	int lengthSM = 5;

	int radiusSM = 100;
	double tanHalfAngleAM = 10.0;
	int radiusAM = 50;

	//int lengthSM = 50;
	//int radiusSM = 100;
	//double tanHalfAngleAM = 10.0;
	//int radiusAM = 50;

	//int lengthSM = 2;
	//int radiusSM = 100;

	//double tanHalfAngleAM = 10.0;
	////int lengthConeAM = 55;
	//int radiusAM = 50;

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