#pragma once
#include "../QMeshLib/PolygenMesh.h"

#include <Eigen/Sparse>
#include <unordered_set>

class hmPara {
public:
	hmPara() {};
	~hmPara() {};

	void setTargetModel(const Eigen::SparseMatrix<int>& MTarget);
	Eigen::SparseMatrix<int> getTargetModel() const;
	Eigen::SparseMatrix<int> getInitialModel() const;

	void readAccEroSolution(const Eigen::MatrixXd& accEroSolution, int tEnd = -1);
	Eigen::MatrixXd getAccEroSolution() const;
	Eigen::VectorXi getInfo() const;
	Eigen::SparseMatrix<int> getMTarget() const { return MTarget; }

	void getParas(int& nx, int& ny, int& tEnd, Eigen::SparseMatrix<int>& MTarget, Eigen::SparseMatrix<int>& MInitial) const;

private:
	int nx;
	int ny;
	int tEnd;
	Eigen::SparseMatrix<int> MTarget;//target model
	Eigen::SparseMatrix<int> MInitial;//initial model
	Eigen::MatrixXd accEroSolution;//acceleration error solution

	void adjustAccEroSolution();

};