#include "hmPara.h"

namespace std {
	template<>
	struct hash<Eigen::Vector2i> {
		size_t operator()(const Eigen::Vector2i& vec) const {
			return hash<int>()(vec.x()) ^ (hash<int>()(vec.y()) << 1);
		}
	};
}


void hmPara::setTargetModel(const Eigen::SparseMatrix<int>& MTarget) {
	this->MTarget = MTarget;
	this->nx = MTarget.rows();
	this->ny = MTarget.cols();

	this->MInitial = Eigen::SparseMatrix<int>(this->nx, this->ny);
	this->MInitial.setZero();

	this->tEnd = 0;
	std::cout << "Target model set successfully" << std::endl;
}

Eigen::SparseMatrix<int> hmPara::getTargetModel() const {
	return this->MTarget;
}

Eigen::SparseMatrix<int> hmPara::getInitialModel() const {
	return this->MInitial;
}

void hmPara::readAccEroSolution(const Eigen::MatrixXd& accEroSolution, int tEnd) {
	this->accEroSolution = accEroSolution;
	if (tEnd > 0) {
		this->tEnd = tEnd;
	}
	this->adjustAccEroSolution();
}

Eigen::MatrixXd hmPara::getAccEroSolution() const {
	return this->accEroSolution;
}

Eigen::VectorXi hmPara::getInfo() const {
	Eigen::VectorXi info(3);
	info << this->nx, this->ny, this->tEnd;
	return info;
}

void hmPara::adjustAccEroSolution() {
	int nSpace=this->accEroSolution.rows();
	int nAS = this->accEroSolution.cols();
	
	for (int i = 0; i < nSpace; i++) {
		for (int j = 0; j < nAS; j++) {
			if (this->accEroSolution(i, j) > this->tEnd) {
				for (int k = j; k < nAS; k++) {
					//this->accEroSolution(i, k) = this->tEnd + k - j + 1;
					this->accEroSolution(i, k) = this->tEnd + 1;
				}
				break;
			}
		}
	}

}

void hmPara::getParas(int& nx, int& ny, int& tEnd, Eigen::SparseMatrix<int>& MTarget, Eigen::SparseMatrix<int>& MInitial) const {
	nx = this->nx;
	ny = this->ny;
	tEnd = this->tEnd;
	MTarget = this->MTarget;
	MInitial = this->MInitial;
}