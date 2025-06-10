#include "AccEro3D.h"

HMP_struct3D AccEro3D::solveByAccEro3d(Eigen::VectorXi& resultModel, bool opt, bool outLog, bool outInfo){
	
	// set Timer
	//auto start = std::chrono::system_clock::now();
	//auto end = std::chrono::system_clock::now();
	//std::chrono::duration<double> elapsed_seconds;

	Eigen::VectorXi checkTime(this->modelObj3D->nSpace);
	checkTime.setZero();

	Eigen::MatrixXi height;
	Eigen::VectorXi Limits = this->modelObj3D->getSolidLimit(height);// solid limit in action space

	bool modelEmpty = false;
	std::vector<std::vector<std::pair<int, int>>> AccAll;
	std::vector<std::vector<int>> EroAll;

	resultModel.resize(this->modelObj3D->nSpace);
	resultModel.setZero();

	if (outInfo) {
		std::cout << "\ninitial solid number: " << this->modelObj3D->currentModelValue.sum() << std::endl;
	}
	
	std::vector<int> solidNums;
	solidNums.push_back(this->modelObj3D->currentModelValue.sum());
	int lastTopNumSolid = this->modelObj3D->currentModelValue.tail(this->modelObj3D->nx * this->modelObj3D->ny).sum();

	fileIO* IO_operator = new fileIO();

	while (!modelEmpty) {
		Eigen::VectorXi modelValueBefore = this->modelObj3D->currentModelValue;

		if (outLog) {
			IO_operator->writeVector(modelValueBefore, "../DataSet/model1.txt");
		}
			
		std::vector<std::pair<int, int>> AccElement = this->Acc(Limits, height, 10);

		if (outLog) {
			IO_operator->writeVector(this->modelObj3D->currentModelValue, "../DataSet/model2.txt");
		}

		std::vector<int> EroElement = this->Ero(Limits, height, 10);

		if (outLog) {
			IO_operator->writeVector(this->modelObj3D->currentModelValue, "../DataSet/model3.txt");
		}

		int numSolid = this->modelObj3D->currentModelValue.sum();
		int topNumSolid = this->modelObj3D->currentModelValue.tail(this->modelObj3D->nx * this->modelObj3D->ny).sum();

		bool modelNotChanged = modelValueBefore.cwiseEqual(this->modelObj3D->currentModelValue).all();
		if (!modelNotChanged && topNumSolid >= lastTopNumSolid) {
			if (numSolid > solidNums.back()) {
				if (std::find(solidNums.begin(), solidNums.end(), numSolid) != solidNums.end()) {
					modelNotChanged = true;
				}
			}
		}

		if (modelNotChanged) {
			resultModel = modelValueBefore;
			this->modelObj3D->startModelValue = modelValueBefore;
			if (outInfo) {
				std::cout << "Model Cannot Be Converted." << std::endl;
			}
			break;
		}
		else if (outInfo) {
			std::cout << "Solid Number: " << numSolid << std::endl;
		}

		resultModel = this->modelObj3D->currentModelValue;

		lastTopNumSolid = topNumSolid;
		solidNums.push_back(numSolid);
		AccAll.push_back(AccElement);
		EroAll.push_back(EroElement);

		modelEmpty = (height.maxCoeff() <= 0);
	}

	if (outLog) {
		IO_operator->writeVector(this->modelObj3D->currentModelValue, "../DataSet/model4.txt");
		IO_operator->writeVector(resultModel, "../DataSet/model5.txt");
	}

	delete IO_operator;

	if (modelEmpty) {
		Limits << this->modelObj3D->limitActionSpace(1), this->modelObj3D->limitActionSpace(0),
			this->modelObj3D->limitActionSpace(3), this->modelObj3D->limitActionSpace(2), this->modelObj3D->limitActionSpace(4);
		if (outInfo) {
			std::cout << "Model is empty." << std::endl;
		}
		this->modelObj3D->startModelValue = this->modelObj3D->currentModelValue;
	}

	HMP_struct3D T(this->modelObj3D->nSpace);

	int iteTime = AccAll.size();
	for (int i = 0; i < iteTime; i++) {
		std::vector<std::pair<int, int>> AccElement = AccAll[iteTime - i - 1];
		std::vector<int> EroElement = EroAll[iteTime - i - 1];

		this->deEro(T, EroElement, Limits, height, checkTime, opt);
		this->deAcc(T, AccElement, Limits, height, checkTime, opt);

		if (outInfo) {
			std::cout << "Solid Number: " << this->modelObj3D->currentModelValue.sum() << std::endl;
		}
	}

	return T;
}

std::vector<std::pair<int, int>> AccEro3D::Acc(Eigen::VectorXi& Limits, Eigen::MatrixXi& height, int accRange) {

	Eigen::VectorXi LimitInitial = Limits;
	std::vector<std::pair<int, int>> AccElement;

	int xAccl1 = std::max(this->modelObj3D->limitActionSpace(0), Limits(0) - this->modelObj3D->supportSize + 1);
	int xAccr1 = std::min(this->modelObj3D->limitActionSpace(1), Limits(1) + this->modelObj3D->supportSize - 1);
	int xAccl2 = std::min(xAccr1, Limits(0) + this->lengthSM - 1);
	int xAccr2 = std::max(xAccl1, Limits(1) - this->lengthSM + 1);

	int yAccl1 = std::max(this->modelObj3D->limitActionSpace(2), Limits(2) - this->modelObj3D->supportSize + 1);
	int yAccr1 = std::min(this->modelObj3D->limitActionSpace(3), Limits(3) + this->modelObj3D->supportSize - 1);
	int yAccl2 = std::min(yAccr1, Limits(2) + this->lengthSM - 1);
	int yAccr2 = std::max(yAccl1, Limits(3) - this->lengthSM + 1);


	int lastIndex = -1;
	for (int i = this->modelObj3D->currentModelValue.size() - 1; i >= 0; i--) {
		if (this->modelObj3D->currentModelValue[i] == 1) {
			lastIndex = i;
			break;
		}
	}
	if (lastIndex < 0) {
		std::cout << "error: currentModel is empty" << std::endl;
	}
	int zHeight = lastIndex / (this->modelObj3D->nx * this->modelObj3D->ny);

	if (zHeight < Limits(4)) {
		std::cout << "Error: zHeight < Limits(4)" << std::endl;
	}

	int zAccl1 = std::max(this->modelObj3D->limitActionSpace(4), zHeight - this->lengthSM + 1);

	// Limits: current Solid Limit
	// xAccl1, xAccl2: the boundary of x for ACC from the small x direction
	// zAccl1: the lowest z for ACC
	

	int accStartZ = std::max(LimitInitial(4) - accRange, this->modelObj3D->limitActionSpace(4));

	Eigen::Vector3i checkPos(3);
	for (int z = accStartZ; z < LimitInitial(4); z++) {
		
		// from small x direction
		for (int y = LimitInitial(2); y <= LimitInitial(3); y++) {
			Eigen::Vector2i yz(y, z);
			double distToSolidBrim = this->modelObj3D->distToSolidBrim(yz, 0);
			if (distToSolidBrim <= this->radiusSM) {
				continue;
			}
			distToSolidBrim = this->modelObj3D->distToSolidBrim(yz, 4);
			if (distToSolidBrim == 0) {
				continue;
			}

			checkPos << LimitInitial(0), y, z;
			int xNow = -1;

			// find the last empty voxel in x direction
			for (int x = xAccl1; x <= xAccl2; x++) {
				if (x > this->lengthSM) {
					if (distToSolidBrim <= this->radiusSM) {
						break;
					}
				}
				checkPos(0) = x;
				int index = this->modelObj3D->coordinateToIndex(checkPos);
				if (this->modelObj3D->currentModelValue(index) == 1) {
					break;
				}
				else {
					xNow = x;
				}
			}

			// find the last empty voxel in x direction with solid neighbor
			int xNow2 = xNow;
			xNow = -1;
			for (int x = xNow2; x >= xAccl1; x--) {
				checkPos(0) = x;
				if (this->modelObj3D->hasSolidNeighbor(checkPos)) {
					xNow = x;
					break;
				}
			}

			// Acc from xNow to xAccStart
			for (int x = xNow; x >= xAccl1; x--) {
				checkPos(0) = x;
				int index = this->modelObj3D->coordinateToIndex(checkPos);
				this->modelObj3D->currentModelValue(index) = 1;
				AccElement.push_back(std::make_pair(index, 1));
				if (z > height(x, y)) {
					height(x, y) = z;
				}
			}

			if (xNow >= xAccl1) {
				Limits(0) = xAccl1;
			}

		}

		// from small y direction
		for (int x = LimitInitial(0); x <= LimitInitial(1); x++) {
			Eigen::Vector2i xz(x, z);
			double distToSolidBrim = this->modelObj3D->distToSolidBrim(xz, 2);
			if (distToSolidBrim <= this->radiusSM) {
				continue;
			}
			distToSolidBrim = this->modelObj3D->distToSolidBrim(xz, 6);
			if (distToSolidBrim == 0) {
				continue;
			}

			checkPos << x, LimitInitial(2), z;
			int yNow = -1;

			// find the last empty voxel in y direction
			for (int y = yAccl1; y <= yAccl2; y++) {
				if (y > this->lengthSM) {
					if (distToSolidBrim <= this->radiusSM) {
						break;
					}
				}
				checkPos(1) = y;
				int index = this->modelObj3D->coordinateToIndex(checkPos);
				if (this->modelObj3D->currentModelValue(index) == 1) {
					break;
				}
				else {
					yNow = y;
				}
			}

			// find the last empty voxel with solid neighbor
			int yNow2 = yNow;
			yNow = -1;
			for (int y = yNow2; y >= yAccl1; y--) {
				checkPos(1) = y;
				if (this->modelObj3D->hasSolidNeighbor(checkPos)) {
					yNow = y;
					break;
				}
			}

			// Acc from yNow to yAccl1
			for (int y = yNow; y >= yAccl1; y--) {
				checkPos(1) = y;
				int index = this->modelObj3D->coordinateToIndex(checkPos);
				this->modelObj3D->currentModelValue(index) = 1;
				AccElement.push_back(std::make_pair(index, 3));
				if (z > height(x, y)) {
					height(x, y) = z;
				}
			}

			if (yNow >= yAccl1) {
				Limits(2) = yAccl1;
			}
		}

		// from large x direction
		for (int y = LimitInitial(2); y <= LimitInitial(3); y++) {
			Eigen::Vector2i yz(y, z);
			double distToSolidBrim = this->modelObj3D->distToSolidBrim(yz, 1);
			if (distToSolidBrim <= this->radiusSM) {
				continue;
			}
			distToSolidBrim = this->modelObj3D->distToSolidBrim(yz, 5);
			if (distToSolidBrim == 0) {
				continue;
			}

			checkPos << LimitInitial(1), y, z;
			int xNow = 1e10;

			// find the last empty voxel in x direction
			for (int x = xAccr1; x >= xAccr2; x--) {
				if (x < this->modelObj3D->nx - 1 - this->lengthSM) {
					if (distToSolidBrim <= this->radiusSM) {
						break;
					}
				}
				checkPos(0) = x;
				int index = this->modelObj3D->coordinateToIndex(checkPos);
				if (this->modelObj3D->currentModelValue(index) == 1) {
					break;
				}
				else {
					xNow = x;
				}
			}

			// find the last empty voxel with solid neighbor
			int xNow2 = xNow;
			xNow = 1e10;
			for (int x = xNow2; x <= xAccr1; x++) {
				checkPos(0) = x;
				if (this->modelObj3D->hasSolidNeighbor(checkPos)) {
					xNow = x;
					break;
				}
			}

			// Acc from xNow to xAccStart
			for (int x = xNow; x <= xAccr1; x++) {
				checkPos(0) = x;
				int index = this->modelObj3D->coordinateToIndex(checkPos);
				this->modelObj3D->currentModelValue(index) = 1;
				AccElement.push_back(std::make_pair(index, 2));
				if (z > height(x, y)) {
					height(x, y) = z;
				}
			}

			if (xNow <= xAccr1) {
				Limits(1) = xAccr1;
			}
		}

		// from large y direction
		for (int x = LimitInitial(0); x <= LimitInitial(1); x++) {
			Eigen::Vector2i xz(x, z);
			double distToSolidBrim = this->modelObj3D->distToSolidBrim(xz, 3);
			if (distToSolidBrim <= this->radiusSM) {
				continue;
			}
			distToSolidBrim = this->modelObj3D->distToSolidBrim(xz, 7);
			if (distToSolidBrim == 0) {
				continue;
			}

			checkPos << x, LimitInitial(3), z;
			int yNow = 1e10;

			// find the last empty voxel in y direction
			for (int y = yAccr1; y >= yAccr2; y--) {
				if (y < this->modelObj3D->ny - 1 - this->lengthSM) {
					if (distToSolidBrim <= this->radiusSM) {
						break;
					}
				}
				checkPos(1) = y;
				int index = this->modelObj3D->coordinateToIndex(checkPos);
				if (this->modelObj3D->currentModelValue(index) == 1) {
					break;
				}
				else {
					yNow = y;
				}
			}

			// find the last empty voxel with solid neighbor
			int yNow2 = yNow;
			yNow = 1e10;
			for (int y = yNow2; y <= yAccr1; y++) {
				checkPos(1) = y;
				if (this->modelObj3D->hasSolidNeighbor(checkPos)) {
					yNow = y;
					break;
				}
			}

			// Acc from yNow to yAccr1
			for (int y = yNow; y <= yAccr1; y++) {
				checkPos(1) = y;
				int index = this->modelObj3D->coordinateToIndex(checkPos);
				this->modelObj3D->currentModelValue(index) = 1;
				AccElement.push_back(std::make_pair(index, 4));
				if (z > height(x, y)) {
					height(x, y) = z;
				}
			}

			if (yNow <= yAccr1) {
				Limits(3) = yAccr1;
			}
		}

		// from up to down
		if (z < zAccl1) {
			continue;
		}

		for (int x = LimitInitial(0); x <= LimitInitial(1); x++) {
			for (int y = LimitInitial(2); y <= LimitInitial(3); y++) {
				int heightTmp = height(x, y);
				if (z <= heightTmp) {
					continue;
				}
				checkPos << x, y, z;
				if (this->modelObj3D->hasSolidNeighbor(checkPos)) {
					int index = this->modelObj3D->coordinateToIndex(checkPos);
					this->modelObj3D->currentModelValue(index) = 1;
					AccElement.push_back(std::make_pair(index, 0));
					height(x, y) = z;
				}
			}
		}
	}

	return AccElement;
}

std::vector<int> AccEro3D::Ero(Eigen::VectorXi& Limits, Eigen::MatrixXi& height, int eroRange) {
	Eigen::VectorXi LimitInitial = Limits;
	Eigen::MatrixXi heightInitial = height;

	std::vector<int> EroElement;

	int eroEndZ = std::max(LimitInitial(4) - eroRange + 1, this->modelObj3D->limitActionSpace(4));

	for (int z = LimitInitial(4); z >= eroEndZ; z--) {
		for (int x = LimitInitial(0); x <= LimitInitial(1); x++) {
			for (int y = LimitInitial(2); y <= LimitInitial(3); y++) {
				// check whether solid
				Eigen::Vector3i checkPos(3);
				checkPos << x, y, z;

				if (z != height(x, y)) {
					continue;
				}

				//check AM-accessibility
				if (!this->isColliFreeAM(checkPos, height)) {
					continue;
				}

				//check Supporting
				if (!this->modelObj3D->isSupportingFree(checkPos)) {
					continue;
				}

				//check Stability
				if (!this->modelObj3D->stillStable(checkPos)) {
					continue;
				}

				int index = this->modelObj3D->coordinateToIndex(checkPos);
				this->modelObj3D->currentModelValue(index) = 0;
				EroElement.push_back(index);

				//update the height
				height(x, y) = -1;
				for (int zTmp = z - 1; zTmp >= this->modelObj3D->limitActionSpace(4); zTmp--) {
					Eigen::Vector3i checkPosTmp(3);
					checkPosTmp << x, y, zTmp;
					int indexTmp = this->modelObj3D->coordinateToIndex(checkPosTmp);
					if (this->modelObj3D->currentModelValue(indexTmp) == 1) {
						height(x, y) = zTmp;
						break;
					}
				}

				//update the limits
				if (x == Limits(0)) {
					Limits(0) = Limits(1);
					for (int xTmp = x; xTmp <= Limits(1); xTmp++) {
						if (height.row(xTmp).maxCoeff() > 0) {
							Limits(0) = xTmp;
							break;
						}
					}
				}
				if (x == Limits(1)) {
					Limits(1) = Limits(0);
					for (int xTmp = x; xTmp >= Limits(0); xTmp--) {
						if (height.row(xTmp).maxCoeff() > 0) {
							Limits(1) = xTmp;
							break;
						}
					}
				}
				if (y == Limits(2)) {
					Limits(2) = Limits(3);
					for (int yTmp = y; yTmp <= Limits(3); yTmp++) {
						if (height.col(yTmp).maxCoeff() > 0) {
							Limits(2) = yTmp;
							break;
						}
					}
				}
				if (y == Limits(3)) {
					Limits(3) = Limits(2);
					for (int yTmp = y; yTmp >= Limits(2); yTmp--) {
						if (height.col(yTmp).maxCoeff() > 0) {
							Limits(3) = yTmp;
							break;
						}
					}
				}

				if (height.maxCoeff() <= 0) {
					break;
				}
			}
			if (height.maxCoeff() <= 0) {
				break;
			}
		}
		if (height.maxCoeff() <= 0) {
			break;
		}
	}

	Limits(4) = height.maxCoeff();
	return EroElement;
}



int AccEro3D::toolBoundaryAM(Eigen::Vector3i tipPoint, Eigen::Vector2i xy) const {
	// AM tool boundary
	Eigen::Vector2i tipXY(tipPoint(0), tipPoint(1));
	Eigen::Vector2d diff = (tipXY - xy).cast<double>();
	double dist = diff.norm();
	if (dist > this->radiusAM) {
		return 1e5;
	}
	else {
		double distZ = dist / tanHalfAngleAM;
		int distZInt = std::ceil(distZ);
		return tipPoint(2) + distZInt;
	}
}

bool AccEro3D::isColliFreeAM(Eigen::Vector3i tipPoint, const Eigen::MatrixXi& height) const{
	bool colliFree = true;

	for (int x = 0; x < this->modelObj3D->nx; x++) {
		for (int y = 0; y < this->modelObj3D->ny; y++) {
			if (x == tipPoint(0) && y == tipPoint(1)) {
				continue;
			}
			Eigen::Vector2i xy(x, y);
			int boundary = this->toolBoundaryAM(tipPoint, xy);
			if (height(x, y) >= boundary) {
				colliFree = false;
				break;
			}
		}
		if (!colliFree) {
			break;
		}
	}

	return colliFree;
}


//int AccEro3D::toolBoundarySM(Eigen::VectorXi tipPoint, int direction, Eigen::VectorXi checkPos) {
//	//output the tool boundary for SM
//	//tipPoint: the tip point of the tool
//	//direction: 0: from up to down; 1: from small x; 2: from large x; 3: from small y; 4: from large y
//	//checkPos: give the information of the line position to check
//
//	int boundary = -100;
//
//	switch (direction) {
//		case 0:
//			// from up to down
//			if (checkPos(0) == tipPoint(0) && checkPos(1) == tipPoint(1)) {
//				boundary = tipPoint(2) + 1;
//			}
//			else {
//				boundary = 1e5;
//			}
//			break;
//		case 1:
//			// from small x
//			if (checkPos(1) == tipPoint(1) && checkPos(2) == tipPoint(2)) {
//				boundary = tipPoint(0) - 1;
//			}
//			else {
//				boundary = -1e5;
//			}
//			break;
//		case 2:
//			// from large x
//			if (checkPos(1) == tipPoint(1) && checkPos(2) == tipPoint(2)) {
//				boundary = tipPoint(0) + 1;
//			}
//			else {
//				boundary = 1e5;
//			}
//			break;
//		case 3:
//			// from small y
//			if (checkPos(0) == tipPoint(0) && checkPos(2) == tipPoint(2)) {
//				boundary = tipPoint(1) - 1;
//			}
//			else {
//				boundary = -1e5;
//			}
//			break;
//		case 4:
//			// from large y
//			if (checkPos(0) == tipPoint(0) && checkPos(2) == tipPoint(2)) {
//				boundary = tipPoint(1) + 1;
//			}
//			else {
//				boundary = 1e5;
//			}
//			break;
//		default:
//			std::cout<<"Error: direction is not correct."<<std::endl;
//			break;
//	}
//
//	return boundary;
//}
//
//int AccEro3D::isColliFreeSM(Eigen::VectorXi tipPoint, int direction) {
//	//check whether the tool is collision free
//	//tipPoint: the tip point of the tool
//	//direction: 0: from up to down; 1: from small x; 2: from large x; 3: from small y; 4: from large y
//
//	int colliFree = 1; //collision free
//
//	switch (direction) {
//		case 0:
//			// from up to down
//			for (int x = 0; x < this->modelObj3D->nx; x++) {
//				for (int y = 0; y < this->modelObj3D->ny; y++) {
//					Eigen::VectorXi checkPos(3);
//					checkPos << x, y, tipPoint(2);
//
//					int maxSolidZ = -1;
//					for (int z = this->modelObj3D->nz - 1; z > tipPoint(2); z--) {
//						checkPos(2) = z;
//						int index = this->modelObj3D->coordinateToIndex(checkPos);
//						if (this->modelObj3D->currentModelValue(index) == 1) {
//							maxSolidZ = z;
//							break;
//						}
//					}
//
//					if (maxSolidZ == -1) {
//						continue;
//					}
//					else {
//						int boundary = toolBoundarySM(tipPoint, direction, checkPos);
//						if (boundary <= maxSolidZ) {
//							colliFree = boundary - maxSolidZ - 1;
//							break;
//						}
//					}
//				}
//				if (colliFree < 0) {
//					break;
//				}
//			}
//			break;
//		case 1:
//			// from small x
//			for (int y = 0; y < this->modelObj3D->ny; y++) {
//				for (int z = 0; z < this->modelObj3D->nz; z++) {
//					Eigen::VectorXi checkPos(3);
//					checkPos << tipPoint(0), y, z;
//
//					int minSolidX = -1;
//					for (int x = 0; x < tipPoint(0); x++) {
//						checkPos(0) = x;
//						int index = this->modelObj3D->coordinateToIndex(checkPos);
//						if (this->modelObj3D->currentModelValue(index) == 1) {
//							minSolidX = x;
//							break;
//						}
//					}
//
//					if (minSolidX == -1) {
//						continue;
//					}
//					else {
//						int boundary = toolBoundarySM(tipPoint, direction, checkPos);
//						if (boundary >= minSolidX) {
//							colliFree = minSolidX - boundary - 1;
//							break;
//						}
//					}
//				}
//				if (colliFree < 0) {
//					break;
//				}
//			}
//			break;
//		case 2:
//			// from large x
//			for (int y = 0; y < this->modelObj3D->ny; y++) {
//				for (int z = 0; z < this->modelObj3D->nz; z++) {
//					Eigen::VectorXi checkPos(3);
//					checkPos << tipPoint(0), y, z;
//
//					int maxSolidX = -1;
//					for (int x = this->modelObj3D->nx - 1; x > tipPoint(0); x--) {
//						checkPos(0) = x;
//						int index = this->modelObj3D->coordinateToIndex(checkPos);
//						if (this->modelObj3D->currentModelValue(index) == 1) {
//							maxSolidX = x;
//							break;
//						}
//					}
//
//					if (maxSolidX == -1) {
//						continue;
//					}
//					else {
//						int boundary = toolBoundarySM(tipPoint, direction, checkPos);
//						if (boundary <= maxSolidX) {
//							colliFree = boundary - maxSolidX - 1;
//							break;
//						}
//					}
//				}
//				if (colliFree < 0) {
//					break;
//				}
//			}
//			break;
//		case 3:
//			// from small y
//			for (int x = 0; x < this->modelObj3D->nx; x++) {
//				for (int z = 0; z < this->modelObj3D->nz; z++) {
//					Eigen::VectorXi checkPos(3);
//					checkPos << x, tipPoint(1), z;
//
//					int minSolidY = -1;
//					for (int y = 0; y < tipPoint(1); y++) {
//						checkPos(1) = y;
//						int index = this->modelObj3D->coordinateToIndex(checkPos);
//						if (this->modelObj3D->currentModelValue(index) == 1) {
//							minSolidY = y;
//							break;
//						}
//					}
//
//					if (minSolidY == -1) {
//						continue;
//					}
//					else {
//						int boundary = toolBoundarySM(tipPoint, direction, checkPos);
//						if (boundary >= minSolidY) {
//							colliFree = minSolidY - boundary - 1;
//							break;
//						}
//					}
//				}
//				if (colliFree < 0) {
//					break;
//				}
//			}
//			break;
//		case 4:
//			// from large y
//			for (int x = 0; x < this->modelObj3D->nx; x++) {
//				for (int z = 0; z < this->modelObj3D->nz; z++) {
//					Eigen::VectorXi checkPos(3);
//					checkPos << x, tipPoint(1), z;
//
//					int maxSolidY = -1;
//					for (int y = this->modelObj3D->ny - 1; y > tipPoint(1); y--) {
//						checkPos(1) = y;
//						int index = this->modelObj3D->coordinateToIndex(checkPos);
//						if (this->modelObj3D->currentModelValue(index) == 1) {
//							maxSolidY = y;
//							break;
//						}
//					}
//
//					if (maxSolidY == -1) {
//						continue;
//					}
//					else {
//						int boundary = toolBoundarySM(tipPoint, direction, checkPos);
//						if (boundary <= maxSolidY) {
//							colliFree = boundary - maxSolidY - 1;
//							break;
//						}
//					}
//				}
//				if (colliFree < 0) {
//					break;
//				}
//			}
//			break;
//		default:
//			std::cout<<"Error: direction is not correct."<<std::endl;
//			break;
//	}
//
//	return colliFree;
//}

void AccEro3D::deEro(HMP_struct3D& T, std::vector<int> EroElement, Eigen::VectorXi& Limits, Eigen::MatrixXi& height, Eigen::VectorXi& checkTime, bool opt) {

	for (int i = 0; i < EroElement.size(); i++) {
		//std::cout << i << " ";

		int idx = EroElement[EroElement.size() - i - 1];
		Eigen::Vector3i pos = this->modelObj3D->indexToCoordinate(idx);

		if (!opt) {
			T.Time[idx].push_back(std::make_pair(T.tLast + 1, -1));
			T.tLast = T.tLast + 1;
			this->modelObj3D->currentModelValue(idx) = 1;
			continue;
		}

		int tNew = T.tLast + 1;
		bool updateTlast = this->addNewTime(T, tNew, std::make_pair(idx, -1), checkTime, 10000);

		// update the model
		this->modelObj3D->currentModelValue(idx) = 1;
		// upddate Limits
		if (pos(0) < Limits(0)) {
			Limits(0) = pos(0);
		}	
		if (pos(0) > Limits(1)) {
			Limits(1) = pos(0);
		}
		if (pos(1) < Limits(2)) {
			Limits(2) = pos(1);
		}
		if (pos(1) > Limits(3)) {
			Limits(3) = pos(1);
		}
		if (pos(2) > Limits(4)) {
			Limits(4) = pos(2);
		}
		if (pos(2) > height(pos(0), pos(1))) {
			height(pos(0), pos(1)) = pos(2);
		}


		if (!updateTlast) {
			continue;
		}

		std::vector<Eigen::Vector3i> neighbors;
		int solidNeighbors = this->modelObj3D->getSolidFaceNeighborSameHeight(pos, neighbors);

		if (solidNeighbors == 0) {
			continue;
		}

		int tNeighbor = 0;
		int idxNeighbor = -1;
		for (int j = 0; j < solidNeighbors; j++) {
			int idxTmp = this->modelObj3D->coordinateToIndex(neighbors[j]);
			if (T.Time[idxTmp].size() == 0) {
				continue;
			}
			if (T.Time[idxTmp].back().first > tNeighbor) {
				tNeighbor = T.Time[idxTmp].back().first;
				idxNeighbor = idxTmp;
			}
		}

		if (tNeighbor > 0) {
			changeLastTime(T, tNeighbor, idx, checkTime, 10000);
		}
	}
}

bool AccEro3D::addNewTime(HMP_struct3D& T, int tNew, std::pair<int, int> idxAndDir, Eigen::VectorXi& checkTime, int threshold) {

	int idx = idxAndDir.first;
	std::vector<std::pair<int, int>> tIdx = T.Time[idx];
	int tLast = T.tLast;

	if (tNew < tLast) {
		std::cout << "Error: tNew is smaller than tLast." << std::endl;
	}

	if (tIdx.size() == 0) {
		tIdx.push_back(std::make_pair(tNew, idxAndDir.second));
		T.Time[idx] = tIdx;
		T.tLast = tNew;
		checkTime = checkTime.cwiseMin(tNew);
		checkTime(idx) = tNew;
		return true;
	}

	int iTime = tIdx.size() - 1;
	tIdx.push_back(std::make_pair(tNew, idxAndDir.second));

	int checkTimeTmp = checkTime(idx);
	checkTime = checkTime.cwiseMin(tNew);

	while (iTime >= 0) {
		if (tIdx[iTime + 1].first <= checkTimeTmp) {
			break;
		}

		int timeTmp = tIdx[iTime].first;
		bool flagTmp;

		if (tIdx[iTime + 1].first - timeTmp > threshold) {
			flagTmp = false;
		}
		else if (iTime % 2 == 1) {
			flagTmp = this->emptyToSolid(T, timeTmp, tIdx[iTime + 1].first, idx);
		}
		else {
			flagTmp = this->solidToEmpty(T, timeTmp, tIdx[iTime + 1].first, idx);
		}

		if (flagTmp) {
			checkTime = checkTime.cwiseMin(timeTmp);
			tIdx.erase(tIdx.begin() + iTime);
			tIdx.erase(tIdx.begin() + iTime);
			iTime -= 2;
		}
		else {
			iTime--;
		}
	}

	std::vector<int> tIdxTest;
	T.Time[idx] = tIdx;

	if (tIdx.size() == 0) {
		checkTime(idx) = 0;
		return false;
	}
	checkTime(idx) = tIdx[tIdx.size() - 1].first;
	if (tIdx[tIdx.size() - 1].first == tNew) {
		T.tLast = tNew;
		return true;
	}
	else {
		return false;
	}
}

bool AccEro3D::changeLastTime(HMP_struct3D& T, int tChange, int idx, Eigen::VectorXi& checkTime, int shreshold) {
	int tIdxLast = T.Time[idx].back().first;
	Eigen::Vector3i pos = this->modelObj3D->indexToCoordinate(idx);

	int tIdxBefore;
	if (T.Time[idx].size() == 1) {
		tIdxBefore = 0;
	}
	else {
		tIdxBefore = T.Time[idx][T.Time[idx].size() - 2].first;
	}

	if (tChange >= tIdxLast || tChange <= tIdxBefore) {
		return false;
	}

	if (tIdxLast - tChange > shreshold) {
		return false;
	}

	// check AM support
	if (!this->isSupportFree(T, idx, tChange)) {
		return false;
	}

	// check AM accessibility
	if (!this->isColliFreeAM(T, pos, tChange)) {
		return false;
	}

	bool flag = emptyToSolid(T, tChange, tIdxLast, idx);
	if (flag) {
		T.Time[idx].back().first = tChange;
		checkTime(idx) = tChange;
		return true;
	}
	else {
		return false;
	}
}

bool AccEro3D::isSupportFree(const HMP_struct3D& T, int idx, int time, int emptyIdx) const{
	Eigen::Vector3i pos = this->modelObj3D->indexToCoordinate(idx);
	if (pos(2) <= this->modelObj3D->outerBrim) {
		return true;
	}

	Eigen::VectorXi statusT = this->modelObj3D->onlyBrimValue;
	if (emptyIdx >= 0) {
		statusT(emptyIdx) = 0;
	}

	bool isSupportFree = false;

	//check the status of the voxel under the current voxel
	Eigen::Vector3i checkPos = pos;
	checkPos(2) -= 1;
	std::vector<Eigen::Vector3i> solidPos;
	int count = this->getSolidFaceNeighborSameHeight(T, time, statusT, checkPos, solidPos);
	if (count == 0) {
		return isSupportFree = false;
	}

	int checkIdx = this->modelObj3D->coordinateToIndex(checkPos);
	int status = getStatusFromTime(T, time, statusT, checkIdx);
	if (status == -1) {
		status = 0;
		statusT(checkIdx) = 0;
	}

	if (status == 1) {
		if (count >= 2) {
			return isSupportFree = true;
		}
		else {
			Eigen::Vector3i checkPos2 = solidPos[0];
			int count2 = this->getSolidFaceNeighborSameHeight(T, time, statusT, checkPos2, solidPos);
			if (count2 >= 2) {
				return isSupportFree = true;
			}
			else {
				return isSupportFree = false;
			}
		}
	}
	else {
		for (int i = 0; i < count; i++) {
			Eigen::Vector3i checkPos2 = solidPos[i];
			std::vector<Eigen::Vector3i> solidPos2;
			int count2 = this->getSolidFaceNeighborSameHeight(T, time, statusT, checkPos2, solidPos2);
			if (count2 >= 2) {
				return isSupportFree = true;
			}
			else if (count2 == 0) {
				continue;
			}
			else {
				checkPos2 = solidPos2[0];
				count2 = this->getSolidFaceNeighborSameHeight(T, time, statusT, checkPos2, solidPos2);
				if (count2 >= 2) {
					return isSupportFree = true;
				}
				else {
					continue;
				}
			}
		}
	}

	return isSupportFree;
}

int AccEro3D::getSolidFaceNeighborSameHeight(const HMP_struct3D& T, int time, Eigen::VectorXi& statusT, Eigen::Vector3i pos, std::vector<Eigen::Vector3i>& neighbor) const{
	neighbor.clear();
	int count = 0;

	if (pos(0) > this->modelObj3D->outerBrim) {
		Eigen::Vector3i checkPos = pos;
		checkPos(0) -= 1;
		int idx = this->modelObj3D->coordinateToIndex(checkPos);
		int status = getStatusFromTime(T, time, statusT, idx);
		if (status == -1) {
			status = 0;
			statusT(idx) = 0;
		}

		if (status == 1) {
			neighbor.push_back(checkPos);
			count++;
		}
	}

	if (pos(0) < this->modelObj3D->nx - this->modelObj3D->outerBrim - 1) {
		Eigen::Vector3i checkPos = pos;
		checkPos(0) += 1;
		int idx = this->modelObj3D->coordinateToIndex(checkPos);
		int status = getStatusFromTime(T, time, statusT, idx);
		if (status == -1) {
			status = 0;
			statusT(idx) = 0;
		}
		if (status == 1) {
			neighbor.push_back(checkPos);
			count++;
		}
	}

	if (pos(1) > this->modelObj3D->outerBrim) {
		Eigen::Vector3i checkPos = pos;
		checkPos(1) -= 1;
		int idx = this->modelObj3D->coordinateToIndex(checkPos);
		int status = getStatusFromTime(T, time, statusT, idx);
		if (status == -1) {
			status = 0;
			statusT(idx) = 0;
		}
		if (status == 1) {
			neighbor.push_back(checkPos);
			count++;
		}
	}

	if (pos(1) < this->modelObj3D->ny - this->modelObj3D->outerBrim - 1) {
		Eigen::Vector3i checkPos = pos;
		checkPos(1) += 1;
		int idx = this->modelObj3D->coordinateToIndex(checkPos);
		int status = getStatusFromTime(T, time, statusT, idx);
		if (status == -1) {
			status = 0;
			statusT(idx) = 0;
		}
		if (status == 1) {
			neighbor.push_back(checkPos);
			count++;
		}
	}

	return count;
}

int AccEro3D::getStatusFromTime(const HMP_struct3D& T, int time, Eigen::VectorXi& statusT, int idx) const{
	int status = statusT(idx);
	if (status >= 0) {
		return status;
	}

	std::vector<std::pair<int, int>> tIdx = T.Time[idx];
	if (tIdx.size() == 0) {
		statusT(idx) = this->modelObj3D->startModelValue(idx);
		status = statusT(idx);
		return status;
	}
	
	auto it = std::lower_bound(tIdx.begin(), tIdx.end(), time, [](const std::pair<int, int>& a, int b) {return a.first < b; });

	if (it != tIdx.end() && it->first == time) {
		return -1;
	}

	if (it == tIdx.end()) {
		int type = tIdx[tIdx.size() - 1].second;
		if (type < 0) {
			statusT(idx) = 1;
			return 1;
		}
		else {
			statusT(idx) = 0;
			return 0;
		}
	}

	int type = it->second;
	if (type < 0) {
		statusT(idx) = 0;
		return 0;
	}
	else {
		statusT(idx) = 1;
		return 1;
	}

}

bool AccEro3D::isColliFreeAM(const HMP_struct3D& T, Eigen::Vector3i tipPoint, int time) const{
	bool colliFree = true;

	Eigen::VectorXi statusT = this->modelObj3D->onlyBrimValue;

	for (int x = 0; x < this->modelObj3D->nx; x++) {
		for (int y = 0; y < this->modelObj3D->ny; y++) {
			Eigen::Vector2i xy(x, y);
			int boundary = this->toolBoundaryAM(tipPoint, xy);
			for (int z = boundary; z < this->modelObj3D->nz; z++) {
				int idx = this->modelObj3D->coordinateToIndex(Eigen::Vector3i(x, y, z));
				int status = getStatusFromTime(T, time, statusT, idx);
				if (status == -1) {
					status = 1;
					statusT(idx) = 1;
				}
				if (status == 1) {
					colliFree = false;
					break;
				}
			}
			if (!colliFree) {
				break;
			}
		}
		if (!colliFree) {
			break;
		}
	}

	return colliFree;
}

void AccEro3D::deAcc(HMP_struct3D& T, std::vector<std::pair<int, int>> AccElement, Eigen::VectorXi& Limits, Eigen::MatrixXi& height, Eigen::VectorXi& checkTime, bool opt) {

	for (int i = 0; i < AccElement.size(); i++) {
		std::pair<int, int> idxAndDir = AccElement[AccElement.size() - i - 1];

		if (!opt) {
			T.Time[idxAndDir.first].push_back(std::make_pair(T.tLast + 1, idxAndDir.second));
			T.tLast = T.tLast + 1;
			this->modelObj3D->currentModelValue(idxAndDir.first) = 0;
		}
		else {
			bool updateTLast = this->addNewTime(T, T.tLast + 1, idxAndDir, checkTime, 10000);
			this->modelObj3D->currentModelValue(idxAndDir.first) = 0;
			
			// update height and Limits
			Eigen::Vector3i pos = this->modelObj3D->indexToCoordinate(idxAndDir.first);
			if (pos(2) == height(pos(0), pos(1))) {
				height(pos(0), pos(1)) = -1;
				for (int zTmp = pos(2) - 1; zTmp >= this->modelObj3D->limitActionSpace(4); zTmp--) {
					Eigen::Vector3i checkPos(3);
					checkPos << pos(0), pos(1), zTmp;
					int index = this->modelObj3D->coordinateToIndex(checkPos);
					if (this->modelObj3D->currentModelValue(index) == 1) {
						height(pos(0), pos(1)) = zTmp;
						break;
					}
				}
			}

			if (pos(0) == Limits(0)) {
				Limits(0) = Limits(1);
				for (int xTmp = pos(0); xTmp <= Limits(1); xTmp++) {
					if (height.row(xTmp).maxCoeff() > 0) {
						Limits(0) = xTmp;
						break;
					}
				}
			}
			if (pos(0) == Limits(1)) {
				Limits(1) = Limits(0);
				for (int xTmp = pos(0); xTmp >= Limits(0); xTmp--) {
					if (height.row(xTmp).maxCoeff() > 0) {
						Limits(1) = xTmp;
						break;
					}
				}
			}
			if (pos(1) == Limits(2)) {
				Limits(2) = Limits(3);
				for (int yTmp = pos(1); yTmp <= Limits(3); yTmp++) {
					if (height.col(yTmp).maxCoeff() > 0) {
						Limits(2) = yTmp;
						break;
					}
				}
			}
			if (pos(1) == Limits(3)) {
				Limits(3) = Limits(2);
				for (int yTmp = pos(1); yTmp >= Limits(2); yTmp--) {
					if (height.col(yTmp).maxCoeff() > 0) {
						Limits(3) = yTmp;
						break;
					}
				}
			}
		}

	}

	Limits(4) = height.maxCoeff();	
}

bool AccEro3D::emptyToSolid(const HMP_struct3D& T, int startCheckTime, int endCheckTime, int idx) const {
	// check AM and SM accessibility, and stability of voxel idx
	bool flag = true;
	Eigen::Vector3i pos = this->modelObj3D->indexToCoordinate(idx);

	int sizeToCheck = this->modelObj3D->nx * this->modelObj3D->ny * (pos(2) + 1);

	for (int i = 0; i < sizeToCheck; i++) {
		if (!flag) {
			break;
		}

		std::vector<std::pair<int, int>> tIdx = T.Time[i];
		if (tIdx.size() == 0 || i == idx) {
			continue;
		}
		if (tIdx.back().first < startCheckTime) {
			continue;
		}
		if (tIdx[0].first >= endCheckTime) {
			continue;
		}

		Eigen::Vector3i posTmp = this->modelObj3D->indexToCoordinate(i);

		bool goodAM = false;
		if (posTmp(2) == pos(2)) {
			goodAM = true;
		}

		for (int j = 0; j < tIdx.size(); j++) {
			int AMSM; // 1: AM; -1: SM
			int tNow = tIdx[j].first;
			int type = tIdx[j].second;
			if (type < 0) {
				AMSM = 1;
			}
			else {
				AMSM = -1;
			}

			if (tNow >= endCheckTime) {
				break;
			}
			if (tNow < startCheckTime) {
				continue;
			}

			if (AMSM == 1) {
				// check AM Accessibility
				if (goodAM) {
					continue;
				}

				int boundary = toolBoundaryAM(posTmp, Eigen::Vector2i(pos(0), pos(1)));
				if (pos(2) >= boundary) {
					flag = false;
					break;
				}
				else {
					goodAM = true;
				}
			}
			else {
				//check idx stability
				if ((pos - posTmp).cwiseAbs().maxCoeff() == 1 && (pos - posTmp).cwiseAbs().minCoeff() == 0) {
					bool hasSolidNeighbor = this->hasSolidNeighbor(T, tNow, pos);
					if (!hasSolidNeighbor) {
						flag = false;
						break;
					}
				}

				//check SM Accessibility
				if (!isColliFreeSM(T, i, j, pos)) {
					flag = false;
					break;
				}

			}
		}
	}

	return flag;
}

bool AccEro3D::hasSolidNeighbor(const HMP_struct3D& T, int time, Eigen::Vector3i pos) const {
	bool hasSolidNeighbor = false;

	Eigen::VectorXi statusT = this->modelObj3D->onlyBrimValue;
	for (int i = 0; i < this->modelObj3D->neighborDir.size(); i++) {
		Eigen::Vector3i checkPos(3);
		checkPos = pos + this->modelObj3D->neighborDir[i];
		if (checkPos(0) <= 0 || checkPos(0) >= this->modelObj3D->nx - 1 || checkPos(1) <= 0 || checkPos(1) >= this->modelObj3D->ny - 1 || checkPos(2) >= this->modelObj3D->nz) {
			continue;
		}
		if (checkPos(2) < 0) {
			hasSolidNeighbor = true;
			break;
		}
		int index = this->modelObj3D->coordinateToIndex(checkPos);

		int status = getStatusFromTime(T, time, statusT, index);

		if (status == -1) {
			status = 0;
			statusT(index) = 0;
		}
		if (status == 1) {
			hasSolidNeighbor = true;
			break;
		}
	}

	return hasSolidNeighbor;
}

bool AccEro3D::isColliFreeSM(const HMP_struct3D& T, int posIdx, int smIdx, Eigen::Vector3i checkPos) const {
	bool colliFree = true;
	Eigen::Vector3i tipPos = this->modelObj3D->indexToCoordinate(posIdx);
	int direction = T.Time[posIdx][smIdx].second;
	int time = T.Time[posIdx][smIdx].first;

	if ((tipPos - checkPos).cwiseAbs().maxCoeff() == 0) {
		std::cout << "Error: tipPos and checkPos are the same." << std::endl;
		return false;
	}


	switch (direction)
	{
	case 0:
		if (tipPos(2) >= checkPos(2)) {
			return true;
		}
		if (tipPos(0) == checkPos(0) && tipPos(1) == checkPos(1)) {
			return false;
		}
		if (checkPos(2) - tipPos(2) < this->lengthSM) {
			return true;
		}
		if ((checkPos.segment(0, 2) - tipPos.segment(0, 2)).norm() > this->radiusSM) {
			return true;
		}
		return false;

	case 1:
		if (tipPos(0) <= checkPos(0)) {
			return true;
		}
		if (tipPos(1) == checkPos(1) && tipPos(2) == checkPos(2)) {
			return false;
		}
		if (tipPos(0) - checkPos(0) < this->lengthSM) {
			return true;
		}
		if ((tipPos.segment(1, 2) - checkPos.segment(1, 2)).norm() > this->radiusSM) {
			return true;
		}
		return false;

	case 2:
		if (tipPos(0) >= checkPos(0)) {
			return true;
		}
		if (tipPos(1) == checkPos(1) && tipPos(2) == checkPos(2)) {
			return false;
		}
		if (checkPos(0) - tipPos(0) < this->lengthSM) {
			return true;
		}
		if ((checkPos.segment(1, 2) - tipPos.segment(1, 2)).norm() > this->radiusSM) {
			return true;
		}
		return false;

	case 3:
		if (tipPos(1) <= checkPos(1)) {
			return true;
		}
		if (tipPos(0) == checkPos(0) && tipPos(2) == checkPos(2)) {
			return false;
		}
		if (tipPos(1) - checkPos(1) < this->lengthSM) {
			return true;
		}
		if (sqrt(pow(tipPos(0) - checkPos(0), 2) + pow(tipPos(2) - checkPos(2), 2)) > this->radiusSM) {
			return true;
		}
		return false;

	case 4:
		if (tipPos(1) >= checkPos(1)) {
			return true;
		}
		if (tipPos(0) == checkPos(0) && tipPos(2) == checkPos(2)) {
			return false;
		}
		if (checkPos(1) - tipPos(1) < this->lengthSM) {
			return true;
		}
		if (sqrt(pow(checkPos(0) - tipPos(0), 2) + pow(checkPos(2) - tipPos(2), 2)) > this->radiusSM) {
			return true;
		}
		return false;


	default:
		std::cout << "Error: direction is not correct." << std::endl;
		break;
	}


	return colliFree;
}

bool AccEro3D::solidToEmpty(const HMP_struct3D& T, int startCheckTime, int endCheckTime, int idx) const {
	// check AM support and SM connectivity

	Eigen::Vector3i pos = this->modelObj3D->indexToCoordinate(idx);
	bool flag = true;

	std::vector<int> checkSMList;
	for (int i = 0; i < this->modelObj3D->nSpace; i++) {
		std::vector<std::pair<int, int>> tIdx = T.Time[i];
		if (tIdx.size() == 0 || i == idx) {
			continue;
		}
		if (tIdx.back().first < startCheckTime) {
			continue;
		}
		if (tIdx[0].first >= endCheckTime) {
			continue;
		}

		Eigen::Vector3i posTmp = this->modelObj3D->indexToCoordinate(i);

		bool goodAM = false;
		if (posTmp(2) != pos(2) + 1) {
			goodAM = true;
		}
		else if ((posTmp - pos).segment(0, 2).cwiseAbs().minCoeff() > this->modelObj3D->supportSize) {
			goodAM = true;
		}

		if (!goodAM) {
			// check AM support
			int type = tIdx[0].second;
			int startTmp = 0;
			if (type >= 0) {
				startTmp = 1;
			}

			for (int j = startTmp; j < tIdx.size(); j = j + 2) {
				int tNow = tIdx[j].first;

				if (tNow >= endCheckTime) {
					break;
				}
				if (tNow < startCheckTime) {
					continue;
				}

				if (!isSupportFree(T, i, tNow, idx)) {
					return false;
				}
			}
		}
		checkSMList.push_back(i);
	}

	int iiStart = 0;
	int iiEnd = std::min(50, static_cast<int> (checkSMList.size()));

	while (iiStart < checkSMList.size()) {
		if (!flag) {
			break;
		}

#pragma omp parallel for shared(flag)
		for (int ii = iiStart; ii < iiEnd; ii++) {
			if (!flag) {
				continue;
			}

			int i = checkSMList[ii];
			std::vector<std::pair<int, int>> tIdx = T.Time[i];
			Eigen::VectorXi posTmp = this->modelObj3D->indexToCoordinate(i);

			int type = tIdx[0].second;
			int startTmp = 1;
			if (type >= 0) {
				startTmp = 0;
			}

			for (int j = startTmp; j < tIdx.size(); j = j + 2) {
				if (!flag) {
					break;
				}

				int tNow = tIdx[j].first;
				if (tNow >= endCheckTime) {
					break;
				}
				if (tNow < startCheckTime) {
					continue;
				}

				//check SM connectivity
				Eigen::VectorXi modelFromTime = getModelFromTime(T, tNow, idx);
				bool flagTmp = this->modelObj3D->stillStable(posTmp, modelFromTime);
				if (!flagTmp) {
#pragma omp critical
					{
						flag = false;
					}
					break;
				}
			}
		}

		iiStart = iiEnd;
		iiEnd = std::min(iiEnd + 50, static_cast<int> (checkSMList.size()));
	}

	return flag;
}

Eigen::VectorXi AccEro3D::getModelFromTime(const HMP_struct3D& T, int time, int emptyIdx) const {
	Eigen::VectorXi modelFromTime = this->modelObj3D->onlyBrimValue;

	for (int i = 0; i < this->modelObj3D->nSpace; i++) {
		if (i == emptyIdx) {
			modelFromTime(i) = 0;
			continue;
		}
		if (T.Time[i].size() == 0) {
			continue;
		}

		int status = getStatusFromTime(T, time, modelFromTime, i);

		if (status == -1) {
			//std::cout << "Error: status is -1." << std::endl;
			modelFromTime(i) = 0;
		}

	}

	return modelFromTime;
}

Eigen::MatrixXi AccEro3D::hmp2Matrix(const HMP_struct3D& T, int& threshold) const {
	int nAS = 0;
	for (int i = 0; i < T.Time.size(); i++) {
		if (T.Time[i].size() > nAS) {
			nAS = T.Time[i].size();
		}
	}

	int tEnd = T.tLast;
	threshold = tEnd + 1;


	Eigen::MatrixXi M(T.Time.size(), nAS);
	M.setOnes();
	M = M * threshold;

	for (int i = 0; i < T.Time.size(); i++) {
		for (int j = 0; j < T.Time[i].size(); j++) {
			M(i, j) = T.Time[i][j].first;
		}
	}

	return M;
}

std::vector<Eigen::Vector3i> AccEro3D::_getToolPosInRadius(Eigen::Vector3i xyzPos, int toolDirection, double radius) {
	std::vector<Eigen::Vector3i> toolPosInRadius;

	Eigen::Vector2i planePos;
	Eigen::Vector2i range1, range2;
	int tmp, tmp1, tmp2;
	switch (toolDirection) {
	case 0:
		planePos = Eigen::Vector2i(xyzPos(0), xyzPos(1));
		tmp = std::floor(xyzPos(0) - radius);
		tmp1 = std::max(0, tmp);
		tmp = std::ceil(xyzPos(0) + radius);
		tmp2 = std::min(this->modelObj3D->nx - 1, tmp);
		range1 = Eigen::Vector2i(tmp1, tmp2);
		tmp = std::floor(xyzPos(1) - radius);
		tmp1 = std::max(0, tmp);
		tmp = std::ceil(xyzPos(1) + radius);
		tmp2 = std::min(this->modelObj3D->ny - 1, tmp);
		range2 = Eigen::Vector2i(tmp1, tmp2);
		break;
	case 1:
		planePos = Eigen::Vector2i(xyzPos(1), xyzPos(2));
		tmp = std::floor(xyzPos(1) - radius);
		tmp1 = std::max(0, tmp);
		tmp = std::ceil(xyzPos(1) + radius);
		tmp2 = std::min(this->modelObj3D->ny - 1, tmp);
		range1 = Eigen::Vector2i(tmp1, tmp2);
		tmp = std::floor(xyzPos(2) - radius);
		tmp1 = std::max(0, tmp);
		tmp = std::ceil(xyzPos(2) + radius);
		tmp2 = std::min(this->modelObj3D->nz - 1, tmp);
		range2 = Eigen::Vector2i(tmp1, tmp2);
		break;
	case 2:
		planePos = Eigen::Vector2i(xyzPos(1), xyzPos(2));
		tmp = std::floor(xyzPos(1) - radius);
		tmp1 = std::max(0, tmp);
		tmp = std::ceil(xyzPos(1) + radius);
		tmp2 = std::min(this->modelObj3D->ny - 1, tmp);
		range1 = Eigen::Vector2i(tmp1, tmp2);
		tmp = std::floor(xyzPos(2) - radius);
		tmp1 = std::max(0, tmp);
		tmp = std::ceil(xyzPos(2) + radius);
		tmp2 = std::min(this->modelObj3D->nz - 1, tmp);
		range2 = Eigen::Vector2i(tmp1, tmp2);
		break;
	case 3:
		planePos = Eigen::Vector2i(xyzPos(0), xyzPos(2));
		tmp = std::floor(xyzPos(0) - radius);
		tmp1 = std::max(0, tmp);
		tmp = std::ceil(xyzPos(0) + radius);
		tmp2 = std::min(this->modelObj3D->nx - 1, tmp);
		range1 = Eigen::Vector2i(tmp1, tmp2);
		tmp = std::floor(xyzPos(2) - radius);
		tmp1 = std::max(0, tmp);
		tmp = std::ceil(xyzPos(2) + radius);
		tmp2 = std::min(this->modelObj3D->nz - 1, tmp);
		range2 = Eigen::Vector2i(tmp1, tmp2);
		break;
	case 4:
		planePos = Eigen::Vector2i(xyzPos(0), xyzPos(2));
		tmp = std::floor(xyzPos(0) - radius);
		tmp1 = std::max(0, tmp);
		tmp = std::ceil(xyzPos(0) + radius);
		tmp2 = std::min(this->modelObj3D->nx - 1, tmp);
		range1 = Eigen::Vector2i(tmp1, tmp2);
		tmp = std::floor(xyzPos(2) - radius);
		tmp1 = std::max(0, tmp);
		tmp = std::ceil(xyzPos(2) + radius);
		tmp2 = std::min(this->modelObj3D->nz - 1, tmp);
		range2 = Eigen::Vector2i(tmp1, tmp2);
		break;
	default:
		std::cout << "Error: invalid toolDirection" << std::endl;
		return toolPosInRadius;
	}


	for (int i = range1(0); i <= range1(1); i++) {
		for (int j = range2(0); j <= range2(1); j++) {
			Eigen::Vector2i posTmp(i, j);
			double dis = (posTmp - planePos).norm();
			if (dis <= radius) {
				switch (toolDirection) {
				case 0:
					toolPosInRadius.push_back(Eigen::Vector3i(posTmp(0), posTmp(1), xyzPos(2)));
					break;
				case 1:
					toolPosInRadius.push_back(Eigen::Vector3i(xyzPos(0), posTmp(0), posTmp(1)));
					break;
				case 2:
					toolPosInRadius.push_back(Eigen::Vector3i(xyzPos(0), posTmp(0), posTmp(1)));
					break;
				case 3:
					toolPosInRadius.push_back(Eigen::Vector3i(posTmp(0), xyzPos(1), posTmp(1)));
					break;
				case 4:
					toolPosInRadius.push_back(Eigen::Vector3i(posTmp(0), xyzPos(1), posTmp(1)));
					break;
				default:
					break;
				}
			}
		}
	}

	return toolPosInRadius;
}