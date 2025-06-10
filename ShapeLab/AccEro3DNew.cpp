#include "AccEro3DNew.h"

AccEro3DNew::AccEro3DNew(model3D* modelObj3D, int checkWindow, bool globalSearch, int toolLength) {
	this->modelObj3D = modelObj3D;
	this->checkWindow = checkWindow;
	this->globalSearch = globalSearch;
	this->lengthSM = toolLength;
}


HMP_struct3D AccEro3DNew::solveByAccEro3d(std::string modelName, Eigen::VectorXi& resultModel, Eigen::VectorXd& timeList, bool opt, bool outLog, bool outInfo){

	Eigen::Vector2i checkTimeConnect = Eigen::Vector2i::Zero();

	auto start = std::chrono::high_resolution_clock::now();

	Eigen::VectorXi checkTime(this->modelObj3D->nSpace);
	checkTime.setZero();

	Eigen::MatrixXi height;
	Eigen::VectorXi Limits = this->modelObj3D->getSolidLimit(height);// solid limit in action space

	if (outInfo) {
		std::cout << "Limits: " << Limits.transpose() << std::endl;
		std::cout << "Height: " << height.maxCoeff() << std::endl;
	}
	
	bool modelEmpty = false;
	std::vector<std::vector<std::pair<int, int>>> AccAll;
	std::vector<std::vector<int>> EroAll;

	resultModel.resize(this->modelObj3D->nSpace);
	resultModel.setZero();

	this->numSolids = this->modelObj3D->currentModelValue.sum();

	if (outInfo) {
		std::cout << "\ninitial solid number: " << this->modelObj3D->currentModelValue.sum() << std::endl;
	}
	
	std::vector<int> solidNums;
	solidNums.push_back(this->modelObj3D->currentModelValue.sum());
	int lastTopNumSolid = this->modelObj3D->currentModelValue.tail(this->modelObj3D->nx * this->modelObj3D->ny).sum();

	fileIO* IO_operator = new fileIO();

	timeList.resize(1);
	timeList(0) = 0.0;

	while (!modelEmpty) {
		Eigen::VectorXi modelValueBefore = this->modelObj3D->currentModelValue;

		std::vector<std::pair<int, int>> AccElement;
		std::vector<int> EroElement;
		AccAndEro(Limits, height, AccElement, EroElement, checkTimeConnect, timeList);

		//std::cout << checkTimeConnect.transpose() << std::endl;

		int numSolid = this->modelObj3D->currentModelValue.sum();
			
		std::cout << "Solid Number: " << numSolid << std::endl;

		int heightMax = height.maxCoeff();
		int topNumSolid;
		if (heightMax >= 0) {
			topNumSolid = this->modelObj3D->currentModelValue.segment(heightMax * this->modelObj3D->nx * this->modelObj3D->ny, this->modelObj3D->nx * this->modelObj3D->ny).sum();
			
			bool modelNotChanged = modelValueBefore.cwiseEqual(this->modelObj3D->currentModelValue).all();
			if (EroElement.size()==0){
				modelNotChanged = true;
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
				//std::cout << "Solid Number: " << numSolid << std::endl;
			}
		
		}
		else {
			topNumSolid = 0;
		}
		
		//std::cout << "topNumSolid: " << topNumSolid << std::endl;

		

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

	

	if (modelEmpty) {
		Limits << this->modelObj3D->limitActionSpace(1), this->modelObj3D->limitActionSpace(0),
			this->modelObj3D->limitActionSpace(3), this->modelObj3D->limitActionSpace(2), this->modelObj3D->limitActionSpace(4);
		if (outInfo) {
			std::cout << "Model is empty." << std::endl;
		}
		this->modelObj3D->startModelValue = this->modelObj3D->currentModelValue;
		std::cout << "Solid Number: " << this->modelObj3D->currentModelValue.sum() << std::endl;
	}

	IO_operator->writeAccAll("../DataSet/" + modelName + "_AccAll.bin", AccAll);
	IO_operator->writeEroAll("../DataSet/" + modelName + "_EroAll.bin", EroAll);

	std::cout << "AccAll size: " << AccAll.size() << std::endl;
	std::cout << "EroAll size: " << EroAll.size() << std::endl;

	HMP_struct3D T(this->modelObj3D->nSpace);

	int iteTime = AccAll.size();
	for (int i = 0; i < iteTime; i++) {
		std::vector<std::pair<int, int>> AccElement = AccAll[iteTime - i - 1];
		std::vector<int> EroElement = EroAll[iteTime - i - 1];

		this->deEro(T, EroElement, Limits, height, checkTime, opt);
		if (outInfo) {
			std::cout << "deEro: Solid Number: " << this->modelObj3D->currentModelValue.sum() << std::endl;
		}

		this->deAcc(T, AccElement, Limits, height, checkTime, opt);
		if (outInfo) {
			std::cout << "deAcc: Solid Number: " << this->modelObj3D->currentModelValue.sum() << std::endl;
		}
		int tmp = this->modelObj3D->currentModelValue.sum();

	}

	auto end = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> elapsed = end - start;
	if (outInfo) {
		std::cout << "Time: " << elapsed.count() << "s" << std::endl;
	}

	//std::cout << "check connectivity time: " << checkTimeConnect.transpose() << std::endl;

	delete IO_operator;
	return T;
}


HMP_struct3D AccEro3DNew::redundancyCheck(std::string modelName, int range) {
	this->redunWindow = range;

	auto start = std::chrono::high_resolution_clock::now();

	Eigen::VectorXi checkTime(this->modelObj3D->nSpace);
	checkTime.setZero();

	this->modelObj3D->currentModelValue = this->modelObj3D->startModelValue;
	std::cout << "start solid number: " << this->modelObj3D->currentModelValue.sum() << std::endl;

	Eigen::MatrixXi height = Eigen::MatrixXi::Ones(this->modelObj3D->nx, this->modelObj3D->ny);
	height = height * -1;
	Eigen::VectorXi Limits = Eigen::VectorXi::Zero(5);
	Limits << this->modelObj3D->limitActionSpace(1), this->modelObj3D->limitActionSpace(0),
		this->modelObj3D->limitActionSpace(3), this->modelObj3D->limitActionSpace(2), this->modelObj3D->limitActionSpace(4);

	this->numSolids = this->modelObj3D->currentModelValue.sum();

	fileIO* IO_operator = new fileIO();

	HMP_struct3D T(this->modelObj3D->nSpace);

	std::vector<std::vector<std::pair<int, int>>> AccAll = IO_operator->readAccAll("../DataSet/" + modelName + "_AccAll.bin");
	std::vector<std::vector<int>> EroAll = IO_operator->readEroAll("../DataSet/" + modelName + "_EroAll.bin");

	std::cout << "AccAll size: " << AccAll.size() << std::endl;
	std::cout << "EroAll size: " << EroAll.size() << std::endl;

	int iteTime = AccAll.size();
	for (int i = 0; i < iteTime; i++) {
		std::vector<std::pair<int, int>> AccElement = AccAll[iteTime - i - 1];
		std::vector<int> EroElement = EroAll[iteTime - i - 1];

		this->deEro(T, EroElement, Limits, height, checkTime, true);
		std::cout << "deEro: Solid Number: " << this->modelObj3D->currentModelValue.sum() << std::endl;

		this->deAcc(T, AccElement, Limits, height, checkTime, true);
		std::cout << "deAcc: Solid Number: " << this->modelObj3D->currentModelValue.sum() << std::endl;
	}

	auto end = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> elapsed = end - start;

	std::cout << "Time: " << elapsed.count() << "s" << std::endl;

	delete IO_operator;
	return T;
}








void AccEro3DNew::AccAndEro(Eigen::VectorXi& Limits, Eigen::MatrixXi& height, std::vector<std::pair<int, int>>& accElements, std::vector<int>& eroElements, Eigen::Vector2i& checkTimeConnect, Eigen::VectorXd& timeList) {

	accElements.clear();
	eroElements.clear();

	if (Limits(4) == this->modelObj3D->outerBrim) {
		for (int x = Limits(0); x <= Limits(1); x++) {
			for (int y = Limits(2); y <= Limits(3); y++) {
				if (height(x, y) == Limits(4)) {
					int idx = this->modelObj3D->coordinateToIndex(Eigen::Vector3i(x, y, Limits(4)));
					this->modelObj3D->currentModelValue(idx) = 0;
					eroElements.push_back(idx);
					height(x, y) = -1;
				}
			}
		}
		return;
	}

	std::vector<Eigen::Vector2i> directions;
	directions.push_back(Eigen::Vector2i(0, 0));
	directions.push_back(Eigen::Vector2i(1, 0));
	directions.push_back(Eigen::Vector2i(-1, 0));
	directions.push_back(Eigen::Vector2i(0, 1));
	directions.push_back(Eigen::Vector2i(0, -1));

	Eigen::MatrixXi topStatus = Eigen::MatrixXi::Zero(this->modelObj3D->nx, this->modelObj3D->ny);
	//0: empty, 1: solid, 2: can ero
	for (int x = Limits(0); x <= Limits(1); x++) {
		for (int y = Limits(2); y <= Limits(3); y++) {
			if (height(x, y) == Limits(4)) {
				topStatus(x, y) = 1;

				std::vector<std::pair<std::pair<int, int>, int>> tryAdd;
				Eigen::Vector3i checkPos = Eigen::Vector3i(x, y, Limits(4));
				bool supportFree = this->modelObj3D->isSupportingFree(checkPos);
				if (!supportFree) {
					for (int i = 0; i < directions.size(); i++) {
						int newX = x + directions[i](0);
						int newY = y + directions[i](1);
						if (newX <= this->modelObj3D->limitActionSpace(0) || newX >= this->modelObj3D->limitActionSpace(1) ||
							newY <= this->modelObj3D->limitActionSpace(2) || newY >= this->modelObj3D->limitActionSpace(3)) {
							continue;
						}
						int opeDirection = smAccessible(Eigen::Vector3i(newX, newY, Limits(4) - 1), Limits(4), this->modelObj3D->currentModelValue, tryAdd);
						if (opeDirection < 0) {
							continue;
						}
						else {
							int idxTmp = this->modelObj3D->coordinateToIndex(Eigen::Vector3i(newX, newY, Limits(4) - 1));
							std::pair<int, int> tmpPair = std::pair<int, int>(idxTmp, opeDirection);
							std::pair<std::pair<int, int>, int> tmpPair2 = std::pair<std::pair<int, int>, int>(tmpPair, 0);
							tryAdd.push_back(tmpPair2);
							supportFree = true;
							break;
						}
					}
				}
				if (!supportFree) {
					continue;
				}

				std::vector<Eigen::Vector3i> posList;
				posList.push_back(checkPos);
				if (addToStable(posList, this->modelObj3D->currentModelValue, topStatus, tryAdd, Limits(4), checkTimeConnect, 0, timeList, this->checkWindow, this->globalSearch)) {
					topStatus(x, y) = 2;
					for (int i = 0; i < tryAdd.size(); i++) {
						accElements.push_back(tryAdd[i].first);
						int idx = tryAdd[i].first.first;
						this->modelObj3D->currentModelValue(idx) = 1;
						Eigen::Vector3i posTmp = this->modelObj3D->indexToCoordinate(idx);
						if (posTmp(2) >= height(posTmp(0), posTmp(1))) {
							height(posTmp(0), posTmp(1)) = posTmp(2);
						}
						if (posTmp(0) < Limits(0)) {
							Limits(0) = posTmp(0);
						}
						if (posTmp(0) > Limits(1)) {
							Limits(1) = posTmp(0);
						}
						if (posTmp(1) < Limits(2)) {
							Limits(2) = posTmp(1);
						}
						if (posTmp(1) > Limits(3)) {
							Limits(3) = posTmp(1);
						}
					}
				}
			}
		}
	}


	for (int x = Limits(0); x <= Limits(1); x++) {
		for (int y = Limits(2); y <= Limits(3); y++) {
			if (topStatus(x, y) == 2) {
				int idx = this->modelObj3D->coordinateToIndex(Eigen::Vector3i(x, y, Limits(4)));
				this->modelObj3D->currentModelValue(idx) = 0;
				eroElements.push_back(idx);
				
				// update the height
				height(x, y) = -1;
				for (int zTmp = Limits(4)-1; zTmp >= this->modelObj3D->limitActionSpace(4); zTmp--) {
					Eigen::Vector3i checkPosTmp(3);
					checkPosTmp << x, y, zTmp;
					int indexTmp = this->modelObj3D->coordinateToIndex(checkPosTmp);
					if (this->modelObj3D->currentModelValue(indexTmp) == 1) {
						height(x, y) = zTmp;
						break;
					}
				}

				// update the limits
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
			}
		}
	}
	Limits(4) = height.maxCoeff();
}

int AccEro3DNew::smAccessible(Eigen::Vector3i pos, int height, Eigen::VectorXi model, std::vector<std::pair<std::pair<int, int>, int>> accElements) {
	for (int i = 0; i < accElements.size(); i++) {
		model(accElements[i].first.first) = 1;
	}
	
	int idx = this->modelObj3D->coordinateToIndex(pos);
	if (model(idx) == 1) {
		return -1;
	}
	if (pos(2) >= height - this->lengthSM + 1) {
		bool tmp = true;
		for (int zTmp = pos(2) + 1; zTmp <=height; zTmp++) {
			int idxTmp = this->modelObj3D->coordinateToIndex(Eigen::Vector3i(pos(0), pos(1), zTmp));
			if (model(idxTmp) == 1) {
				tmp = false;
				break;
			}
		}
		if (tmp){
			return 0;
		}
	}
	if (pos(0) <= this->modelObj3D->limitActionSpace(0) + this->lengthSM - 1) {
		bool tmp = true;
		for (int xTmp = 0; xTmp < pos(0); xTmp++) {
			int idxTmp = this->modelObj3D->coordinateToIndex(Eigen::Vector3i(xTmp, pos(1), pos(2)));
			if (model(idxTmp) == 1) {
				tmp = false;
				break;
			}
		}
		if (tmp) {
			return 1;
		}
	}
	if (pos(0) >= this->modelObj3D->limitActionSpace(1) - this->lengthSM + 1) {
		bool tmp = true;
		for (int xTmp = pos(0) + 1; xTmp < this->modelObj3D->nx; xTmp++) {
			int idxTmp = this->modelObj3D->coordinateToIndex(Eigen::Vector3i(xTmp, pos(1), pos(2)));
			if (model(idxTmp) == 1) {
				tmp = false;
				break;
			}
		}
		if (tmp) {
			return 2;
		}
	}
	if (pos(1) <= this->modelObj3D->limitActionSpace(2) + this->lengthSM - 1) {
		bool tmp = true;
		for (int yTmp = 0; yTmp < pos(1); yTmp++) {
			int idxTmp = this->modelObj3D->coordinateToIndex(Eigen::Vector3i(pos(0), yTmp, pos(2)));
			if (model(idxTmp) == 1) {
				tmp = false;
				break;
			}
		}
		if (tmp) {
			return 3;
		}
	}
	if (pos(1) >= this->modelObj3D->limitActionSpace(3) - this->lengthSM + 1) {
		bool tmp = true;
		for (int yTmp = pos(1) + 1; yTmp < this->modelObj3D->ny; yTmp++) {
			int idxTmp = this->modelObj3D->coordinateToIndex(Eigen::Vector3i(pos(0), yTmp, pos(2)));
			if (model(idxTmp) == 1) {
				tmp = false;
				break;
			}
		}
		if (tmp) {
			return 4;
		}
	}
	return -1;
}

bool AccEro3DNew::addToStable(std::vector<Eigen::Vector3i> posList, Eigen::VectorXi model, Eigen::MatrixXi topStatus, std::vector<std::pair<std::pair<int, int>,int>>& tryAdd, int z, 
	Eigen::Vector2i& checkTimeConnect, int seqTime, Eigen::VectorXd& timeList, int boxSize, bool searchMore) {

	auto start = std::chrono::high_resolution_clock::now();

	if (seqTime > 10) {
		auto end = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double> elapsed = end - start;
		timeList(0) = timeList(0) + elapsed.count();
		return false;
	}

	bool isStable = false;

	Eigen::VectorXi modelNew = model;//new model after removing the voxel

	std::vector<int> checkIdxList;
	checkIdxList.reserve(posList.size());
	for (int i = 0; i < posList.size(); i++) {
		int checkIdx = this->modelObj3D->coordinateToIndex(posList[i]);
		checkIdxList.push_back(checkIdx);
		modelNew(checkIdx) = 0;
	}
	for (int x = 0; x < this->modelObj3D->nx; x++) {
		for (int y = 0; y < this->modelObj3D->ny; y++) {
			if (topStatus(x, y) == 2) {
				int idxTmp = this->modelObj3D->coordinateToIndex(Eigen::Vector3i(x, y, z));
				modelNew(idxTmp) = 0;
			}
		}
	}
	for (int i = 0; i < tryAdd.size(); i++) {
		modelNew(tryAdd[i].first.first) = 1;
	}

	Eigen::VectorXi visited(this->modelObj3D->nSpace);//visited elements
	visited.setZero();
	int numConnectedComponent = 0;//number of connected components

	std::vector<Eigen::Vector3i> queue;//queue for all non-zero elements

	//search in the neighborhood of pos
	int tmp1 = 1e10, tmp2 = -1, tmp3 = 1e10, tmp4 = -1, tmp5 = 1e10, tmp6 = -1;
	for (int i = 0; i < posList.size(); i++) {
		Eigen::Vector3i pos = posList[i];
		tmp1 = std::min(tmp1, pos(0));
		tmp2 = std::max(tmp2, pos(0));
		tmp3 = std::min(tmp3, pos(1));
		tmp4 = std::max(tmp4, pos(1));
		tmp5 = std::min(tmp5, pos(2));
		tmp6 = std::max(tmp6, pos(2));
	}
	tmp1 = std::max(this->modelObj3D->outerBrim, tmp1 - boxSize);
	tmp2 = std::min(this->modelObj3D->nx - this->modelObj3D->outerBrim - 1, tmp2 + boxSize);
	tmp3 = std::max(this->modelObj3D->outerBrim, tmp3 - boxSize);
	tmp4 = std::min(this->modelObj3D->ny - this->modelObj3D->outerBrim - 1, tmp4 + boxSize);
	tmp5 = std::max(this->modelObj3D->outerBrim, tmp5 - boxSize);
	tmp6 = std::min(this->modelObj3D->nz - 1, tmp6 + boxSize);

	for (int i = 0; i < posList.size(); i++) {
		Eigen::Vector3i pos = posList[i];
		for (int j = 0; j < this->modelObj3D->neighborDir.size(); j++) {
			Eigen::Vector3i newPos = pos + this->modelObj3D->neighborDir[j];
			if (newPos(0) < tmp1 || newPos(0) > tmp2 || newPos(1) < tmp3 || newPos(1) > tmp4 || newPos(2) < tmp5 || newPos(2) > tmp6) {
				continue;
			}
			int idx = this->modelObj3D->coordinateToIndex(newPos);
			if (modelNew(idx) % 2 == 1 && visited(idx) == 0) {
				numConnectedComponent++;
				this->modelObj3D->DFS(newPos, numConnectedComponent, modelNew, visited, queue, tmp1, tmp2, tmp3, tmp4, tmp5, tmp6);
			}
		}
	}

	if (numConnectedComponent == 0) {
		auto end = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double> elapsed = end - start;
		timeList(0) = timeList(0) + elapsed.count();

		checkTimeConnect(0) = checkTimeConnect(0) + 1;
		return isStable = true;
	}
	else if (numConnectedComponent == 1 && posList[0](2) > this->modelObj3D->outerBrim) {
		auto end = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double> elapsed = end - start;
		timeList(0) = timeList(0) + elapsed.count();

		checkTimeConnect(0) = checkTimeConnect(0) + 1;
		return isStable = true;
	}

	Eigen::VectorXi connectedBottom(numConnectedComponent);
	connectedBottom.setZero();
	std::vector<std::deque<Eigen::Vector3i>> queueList(numConnectedComponent);
	Eigen::VectorXi componentSize(numConnectedComponent);
	componentSize.setZero();

	for (int i = 0; i < queue.size(); i++) {
		int idx = this->modelObj3D->coordinateToIndex(queue[i]);
		int componentIdx = visited(idx);
		queueList[componentIdx - 1].push_back(queue[i]);
		componentSize(componentIdx - 1)++;
		if (queue[i](2) <= this->modelObj3D->outerBrim) {
			connectedBottom(componentIdx - 1) = 1;
		}
	}
	queue.clear();
	queue.shrink_to_fit();

	if (connectedBottom.minCoeff() == 1) {
		auto end = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double> elapsed = end - start;
		timeList(0) = timeList(0) + elapsed.count();

		checkTimeConnect(0) = checkTimeConnect(0) + 1;
		return isStable = true;
	}
	Eigen::VectorXi box = this->modelObj3D->limitActionSpace;


	if (!searchMore) {
		auto end = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double> elapsed = end - start;
		timeList(0) = timeList(0) + elapsed.count();

		box << tmp1, tmp2, tmp3, tmp4, tmp5, tmp6;
		checkTimeConnect(1) = checkTimeConnect(1) + 1;
		if (grow(posList, tryAdd, z, box)) {
			return addToStable(posList, model, topStatus, tryAdd, z, checkTimeConnect, seqTime + 1, timeList, boxSize, searchMore);
		}
		else {
			return false;
		}
	}

	int newNumConnected = numConnectedComponent;
	while (componentSize.sum() > 0 && connectedBottom.minCoeff() == 0) {
		for (int i = 0; i < numConnectedComponent; i++) {
			//search from each connected component that has no connection to the bottom
			if (componentSize(i) <= 0 || connectedBottom(i) != 0) {
				continue;
			}

			Eigen::Vector3i posTmp = queueList[i].front();
			queueList[i].pop_front();
			componentSize(i)--;

			//search in the neighborhood of posTmp
			for (int j = 0; j < this->modelObj3D->neighborDir.size(); j++) {
				Eigen::Vector3i newPos = posTmp + this->modelObj3D->neighborDir[j];
				if (newPos(0) < this->modelObj3D->outerBrim || newPos(0) > this->modelObj3D->nx - 1 - this->modelObj3D->outerBrim
					|| newPos(1) < this->modelObj3D->outerBrim || newPos(1) > this->modelObj3D->ny - 1 - this->modelObj3D->outerBrim
					|| newPos(2) < this->modelObj3D->outerBrim || newPos(2) > this->modelObj3D->nz - 1) {
					continue;
				}
				int newIdx = this->modelObj3D->coordinateToIndex(newPos);
				if (modelNew(newIdx) % 2 == 0) {
					continue;
				}

				int visitedIdx = visited(newIdx);
				if (visitedIdx > 0) {
					while (connectedBottom(visitedIdx - 1) > 1) {
						visited(newIdx) = connectedBottom(visitedIdx - 1) - 1;
						visitedIdx = visited(newIdx);
					}
				}

				if (visitedIdx == 0) {
					visited(newIdx) = i + 1;
					if (newPos(2) <= this->modelObj3D->outerBrim) {
						connectedBottom(i) = 1;
						break;
					}
					queueList[i].push_back(newPos);
					componentSize(i)++;
				}
				else if (visitedIdx != i + 1) {
					for (int k = 0; k < componentSize(visitedIdx - 1); k++) {
						Eigen::Vector3i posTmp2 = queueList[visitedIdx - 1][k];
						queueList[i].push_back(posTmp2);
						componentSize(i)++;
						visited(this->modelObj3D->coordinateToIndex(posTmp2)) = i + 1;
					}
					queueList[visitedIdx - 1].clear();
					queueList[visitedIdx - 1].shrink_to_fit();
					componentSize(visitedIdx - 1) = 0;

					int tmpConnect = connectedBottom(visitedIdx - 1);
					connectedBottom(visitedIdx - 1) = 1 + i + 1;

					newNumConnected--;
					if (newNumConnected == 1 && posList[0](2) > this->modelObj3D->outerBrim) {
						auto end = std::chrono::high_resolution_clock::now();
						std::chrono::duration<double> elapsed = end - start;
						timeList(0) = timeList(0) + elapsed.count();

						checkTimeConnect(0) = checkTimeConnect(0) + 1;
						return true;
					}

					if (tmpConnect == 1) {
						connectedBottom(i) = 1;
						break;
					}
				}
			}

			for (int j = 0; j < numConnectedComponent; j++) {
				if (componentSize(j) == 0 && connectedBottom(j) == 0) {
					auto end = std::chrono::high_resolution_clock::now();
					std::chrono::duration<double> elapsed = end - start;
					timeList(0) = timeList(0) + elapsed.count();

					checkTimeConnect(1) = checkTimeConnect(1) + 1;
					if (grow(posList, tryAdd, z, box)) {
						return addToStable(posList, model, topStatus, tryAdd, z, checkTimeConnect, seqTime+1, timeList, boxSize, searchMore);
					}
					else {
						//checkTimeConnect(1) = checkTimeConnect(1) + 1;
						return false;
					}
				}
			}

		}
	}

	auto end = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> elapsed = end - start;
	timeList(0) = timeList(0) + elapsed.count();

	if (connectedBottom.minCoeff() == 0) {
		checkTimeConnect(1) = checkTimeConnect(1) + 1;
		if (grow(posList, tryAdd, z, box)) {
			return addToStable(posList, model, topStatus, tryAdd, z, checkTimeConnect, seqTime + 1, timeList, boxSize, searchMore);
		}
		else {
			//checkTimeConnect(1) = checkTimeConnect(1) + 1;
			return false;
		}
	}
	else {
		checkTimeConnect(0) = checkTimeConnect(0) + 1;
		return true;
	}

}


bool AccEro3DNew::grow(std::vector<Eigen::Vector3i> posList, std::vector<std::pair<std::pair<int, int>, int>>& tryAdd, int z, Eigen::VectorXi box) {
	bool added = false;

	if (tryAdd.size() > (this->numSolids / 10.0)) {
		return false;
	}

	Eigen::Vector3i posTmp = posList[0];
	std::vector<Eigen::Vector3i> posListAnother;
	for (int j = 0; j < this->modelObj3D->neighborDir.size(); j++) {
		Eigen::Vector3i neighbor = posTmp + this->modelObj3D->neighborDir[j];
		if (neighbor(2) >= box(5) || neighbor(2) < box(4) || neighbor(0) < box(0) || neighbor(0) > box(1) || neighbor(1) < box(2) || neighbor(1) > box(3)) {
			continue;
		}
		int idxTmp = this->modelObj3D->coordinateToIndex(neighbor);
		if (this->modelObj3D->currentModelValue(idxTmp) == 1) {
			posListAnother.push_back(neighbor);
		}
	}

	for (int i = 0; i < tryAdd.size(); i++) {
		if (tryAdd[i].second != 0) {
			continue;
		}
		Eigen::Vector3i pos = this->modelObj3D->indexToCoordinate(tryAdd[i].first.first);
		posList.push_back(pos);
		tryAdd[i].second = 1;
	}

	box(5) = std::min(z, box(5));

	for (int i = 0; i < posList.size(); i++) {
		Eigen::Vector3i pos = posList[i];
		for (int j = 0; j < this->modelObj3D->neighborDir.size(); j++) {
			Eigen::Vector3i neighbor = pos + this->modelObj3D->neighborDir[j];
			
			if (neighbor(2) >= box(5) || neighbor(2) < box(4) || neighbor(0) < box(0) || neighbor(0) > box(1) || neighbor(1) < box(2) || neighbor(1) > box(3)) {
				continue;
			}

			//if (neighbor(2) >= z || neighbor(2) < this->modelObj3D->limitActionSpace(4) || neighbor(0) < this->modelObj3D->limitActionSpace(0) || neighbor(0) > this->modelObj3D->limitActionSpace(1) || neighbor(1) < this->modelObj3D->limitActionSpace(2) || neighbor(1) > this->modelObj3D->limitActionSpace(3)) {
			//	continue;
			//}

			int opeDirection = smAccessible(neighbor, z, this->modelObj3D->currentModelValue, tryAdd);
			if (opeDirection < 0) {
				continue;
			}

			std::pair<int, int> tmp = std::pair<int, int>(this->modelObj3D->coordinateToIndex(neighbor), opeDirection);
			std::pair<std::pair<int, int>, int> tmp2 = std::pair<std::pair<int, int>, int>(tmp, 0);

			tryAdd.push_back(tmp2);
			added = true;
		}
	}

	if (added) {
		return true;
	}

	//posList = posListAnother;
	//for (int i = 0; i < posList.size(); i++) {
	//	Eigen::Vector3i pos = posList[i];
	//	for (int j = 0; j < this->modelObj3D->neighborDir.size(); j++) {
	//		Eigen::Vector3i neighbor = pos + this->modelObj3D->neighborDir[j];

	//		if (neighbor(2) >= box(5) || neighbor(2) < box(4) || neighbor(0) < box(0) || neighbor(0) > box(1) || neighbor(1) < box(2) || neighbor(1) > box(3)) {
	//			continue;
	//		}

	//		int opeDirection = smAccessible(neighbor, z, this->modelObj3D->currentModelValue, tryAdd);
	//		if (opeDirection < 0) {
	//			continue;
	//		}

	//		std::pair<int, int> tmp = std::pair<int, int>(this->modelObj3D->coordinateToIndex(neighbor), opeDirection);
	//		std::pair<std::pair<int, int>, int> tmp2 = std::pair<std::pair<int, int>, int>(tmp, 0);

	//		tryAdd.push_back(tmp2);
	//		added = true;
	//	}
	//}

	return added;

}








std::vector<std::pair<int, int>> AccEro3DNew::Acc(Eigen::VectorXi& Limits, Eigen::MatrixXi& height, int accRange) {

	Eigen::VectorXi LimitInitial = Limits; // solid limit in action space
	std::vector<std::pair<int, int>> AccElement;

	// xAccl1, xAccl2: the boundary of x for ACC from the small x direction
	// the boundary can ensure the accessibility of the spindle, but still need to check the cutting part
	int xAccl1 = std::max(this->modelObj3D->limitActionSpace(0), Limits(0) - 1);
	int xAccr1 = std::min(this->modelObj3D->limitActionSpace(1), Limits(1) + 1);
	if (Limits(1)-Limits(0)+1 < this->modelObj3D->supportSize) {
		xAccl1 = std::max(this->modelObj3D->limitActionSpace(0), Limits(0) - this->modelObj3D->supportSize+1);
		xAccr1 = std::min(this->modelObj3D->limitActionSpace(1), Limits(1) + this->modelObj3D->supportSize-1);
	}
	int xAccl2 = std::min(xAccr1, this->modelObj3D->solidLimit(0) + this->lengthSM - 1);
	int xAccr2 = std::max(xAccl1, this->modelObj3D->solidLimit(1) - this->lengthSM + 1);

	int yAccl1 = std::max(this->modelObj3D->limitActionSpace(2), Limits(2) - 1);
	int yAccr1 = std::min(this->modelObj3D->limitActionSpace(3), Limits(3) + 1);
	int yAccl2 = std::min(yAccr1, this->modelObj3D->solidLimit(2) + this->lengthSM - 1);
	int yAccr2 = std::max(yAccl1, this->modelObj3D->solidLimit(3) - this->lengthSM + 1);

	// find the last solid voxel in the model and the height of the solid voxel
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

	// zAccl1: the lowest z for vertical ACC
	int zAccl1 = std::max(this->modelObj3D->limitActionSpace(4), zHeight - this->lengthSM + 1);
	
	// accStartZ: the lowest z for horizontal ACC
	int accStartZ = std::max(LimitInitial(4) - accRange, this->modelObj3D->limitActionSpace(4));

	Eigen::Vector3i checkPos(3);
	for (int z = accStartZ; z < LimitInitial(4); z++) {
		
		// from small x direction
		for (int y = LimitInitial(2); y <= LimitInitial(3); y++) {
			checkPos << LimitInitial(0), y, z;
			int xNow = -1;
			// find the last empty voxel in x direction
			for (int x = xAccl1; x <= xAccl2; x++) {
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
			// Acc from xNow to xAccl1
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
				// the condition means that xAccl1 has solid voxel
				// since xAccl1 <= Limits(0)
				Limits(0) = xAccl1;
			}
		}

		// from small y direction
		for (int x = LimitInitial(0); x <= LimitInitial(1); x++) {
			checkPos << x, LimitInitial(2), z;
			int yNow = -1;
			// find the last empty voxel in y direction
			for (int y = yAccl1; y <= yAccl2; y++) {
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
			checkPos << LimitInitial(1), y, z;
			int xNow = 1e10;
			// find the last empty voxel in x direction
			for (int x = xAccr1; x >= xAccr2; x--) {
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
			// Acc from xNow to xAccr1
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
			checkPos << x, LimitInitial(3), z;
			int yNow = 1e10;
			// find the last empty voxel in y direction
			for (int y = yAccr1; y >= yAccr2; y--) {
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

		for (int x = xAccl1; x <= xAccr1; x++) {
			for (int y = yAccl1; y <= yAccr1; y++) {
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
					if (x<Limits(0)) {
						Limits(0) = x;
					}
					if (x>Limits(1)) {
						Limits(1) = x;
					}
					if (y<Limits(2)) {
						Limits(2) = y;
					}
					if (y>Limits(3)) {
						Limits(3) = y;
					}
				}
			}
		}
	}

	if (this->AMMiniSize == 1) {
		return AccElement;
	}

	// find the solid voxels in the top layer
	std::vector<std::vector<Eigen::Vector3i>> posTopSolid;
	for (int y=LimitInitial(2);y<=LimitInitial(3);y++){
		std::vector<Eigen::Vector3i> posTmp;
		for (int x=LimitInitial(0);x<=LimitInitial(1);x++){
			if (height(x,y)==Limits(4)){
				posTmp.push_back(Eigen::Vector3i(x,y,Limits(4)));
			}
		}
		posTopSolid.push_back(posTmp);
	}

	for (int j=0;j<posTopSolid.size();j++){
		if (posTopSolid[j].size()==0){
			continue;
		}

		int lastSolidX = -1e10;
		for (int i=0;i<posTopSolid[j].size();i++){
			Eigen::Vector3i pos = posTopSolid[j][i];

			// connect with the last group of solid voxels
			if (pos(0)-lastSolidX-1 <this->AMMiniGap ){
				for (int x=lastSolidX+1;x<=pos(0)-1;x++){
					int index = this->modelObj3D->coordinateToIndex(Eigen::Vector3i(x,pos(1),pos(2)));
					this->modelObj3D->currentModelValue(index) = 1;
					AccElement.push_back(std::make_pair(index, 0));
					height(x,pos(1)) = pos(2);
				}
				lastSolidX = std::max(lastSolidX, pos(0));
				continue;
			}

			if (pos(0)>=this->modelObj3D->limitActionSpace(1)-this->AMMiniSize){
				// there is no enough space at the right side
				for (int x=pos(0)+1;x<=this->modelObj3D->limitActionSpace(1)-1;x++){
					int index = this->modelObj3D->coordinateToIndex(Eigen::Vector3i(x,pos(1),pos(2)));
					if (this->modelObj3D->currentModelValue(index)==1){
						continue;
					}
					else{
						this->modelObj3D->currentModelValue(index) = 1;
						AccElement.push_back(std::make_pair(index, 0));
						height(x,pos(1)) = pos(2);
						if (x>Limits(1)){
							Limits(1) = x;
						}
					}
				}
				for (int x=pos(0)-1;x>=this->modelObj3D->limitActionSpace(1)-this->AMMiniSize;x--){
					int index = this->modelObj3D->coordinateToIndex(Eigen::Vector3i(x,pos(1),pos(2)));
					if (this->modelObj3D->currentModelValue(index)==1){
						continue;
					}
					else{
						this->modelObj3D->currentModelValue(index) = 1;
						AccElement.push_back(std::make_pair(index, 0));
						height(x,pos(1)) = pos(2);
						if (x<Limits(0)){
							Limits(0) = x;
						}
					}
				}
				break;
			}
			else{
				for (int x = pos(0)+1;x<=pos(0)+this->AMMiniSize-1;x++){
					int index = this->modelObj3D->coordinateToIndex(Eigen::Vector3i(x,pos(1),pos(2)));
					if (this->modelObj3D->currentModelValue(index)==1){
						continue;
					}
					else{
						this->modelObj3D->currentModelValue(index) = 1;
						AccElement.push_back(std::make_pair(index, 0));
						height(x,pos(1)) = pos(2);
						if (x>Limits(1)){
							Limits(1) = x;
						}
					}
				}
				lastSolidX = pos(0)+this->AMMiniSize-1;
			}
		}
	}

	return AccElement;
}

std::vector<int> AccEro3DNew::Ero(Eigen::VectorXi& Limits, Eigen::MatrixXi& height, int eroRange) {
	Eigen::VectorXi LimitInitial = Limits;
	Eigen::MatrixXi heightInitial = height;

	std::vector<int> EroElement;

	int eroEndZ = std::max(LimitInitial(4) - eroRange + 1, this->modelObj3D->limitActionSpace(4));

	for (int z = LimitInitial(4); z >= eroEndZ; z--) {
		for (int y = LimitInitial(2); y <= LimitInitial(3); y++) {
			int x = LimitInitial(0);
			int xPartLeft = -1, xPartRight = -1;
			bool lastSatisfied = false;
			while (x <= this->modelObj3D->limitActionSpace(1)+1) {
				if (height.maxCoeff() <= 0) {
					break;
				}
				// process the last voxel case
				if (x==this->modelObj3D->limitActionSpace(1)+1){
					lastSatisfied = false;
				}

				if (!lastSatisfied){
					if (xPartRight >0){
						for (int xTmp = xPartLeft; xTmp <= xPartRight; xTmp++){
							Eigen::Vector3i posTmp(xTmp, y, z);
							int index = this->modelObj3D->coordinateToIndex(posTmp);
							if (this->modelObj3D->currentModelValue(index) == 0) {
								std::cout << height(xTmp, y) << " " << z << std::endl;
								std::cout << "Error: Ero Element is not empty" << std::endl;
							}

							this->modelObj3D->currentModelValue(index) = 0;
							EroElement.push_back(index);

							// update the height
							height(xTmp, y) = -1;
							for (int zTmp = z - 1; zTmp >= this->modelObj3D->limitActionSpace(4); zTmp--) {
								Eigen::Vector3i checkPosTmp(3);
								checkPosTmp << xTmp, y, zTmp;
								int indexTmp = this->modelObj3D->coordinateToIndex(checkPosTmp);
								if (this->modelObj3D->currentModelValue(indexTmp) == 1) {
									height(xTmp, y) = zTmp;
									break;
								}
							}

							// update the limits
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
						}
					}
					xPartLeft = -1;
					xPartRight = -1;
				}

				if (x==this->modelObj3D->limitActionSpace(1)+1){
					break;
				}

				// check whether solid
				Eigen::Vector3i checkPos(3);
				checkPos << x, y, z;
				if (z != height(x, y)) {
					x++;
					lastSatisfied = false;
					continue;
				}
				//check AM-accessibility
				if (!this->isColliFreeAM(checkPos, height)) {
					x++;
					lastSatisfied = false;
					continue;
				}
				//check Supporting
				if (!this->modelObj3D->isSupportingFree(checkPos)) {
					x++;
					lastSatisfied = false;
					continue;
				}
				//check Stability
				std::vector<Eigen::Vector3i> checkPosList;
				if (xPartLeft >= 0) {
					for (int xTmp = xPartLeft; xTmp <= x; xTmp++) {
						checkPosList.push_back(Eigen::Vector3i(xTmp, y, z));
					}
				}
				else {
					checkPosList.push_back(checkPos);
				}
				if (!this->modelObj3D->stillStable(checkPosList)) {
					if (checkPosList.size() == 1) {
						x++;
					}
					//x++;
					lastSatisfied = false;
					continue;
				}

				lastSatisfied = true;

				// set the left voxel
				if (xPartLeft == -1){
					xPartLeft = x;
					if (this->AMMiniSize == 1) {
						xPartRight = x;
						x++;
						continue;
					}
					x++;
					continue;
				}
				
				// set the right voxel
				if (x - xPartLeft + 1 >= this->AMMiniSize) {
					xPartRight = x;
					x++;
					continue;
				}

				x++;
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



int AccEro3DNew::toolBoundaryAM(Eigen::Vector3i tipPoint, Eigen::Vector2i xy) const {
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

bool AccEro3DNew::isColliFreeAM(Eigen::Vector3i tipPoint, const Eigen::MatrixXi& height) const{
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


void AccEro3DNew::deEro(HMP_struct3D& T, std::vector<int> EroElement, Eigen::VectorXi& Limits, Eigen::MatrixXi& height, Eigen::VectorXi& checkTime, bool opt) {

	// Check elements in EroElement from the last to the first and group them
	std::vector<std::vector<int>> groups;
	int lastHeight = -1;
	// Traverse EroElement from the end to the beginning
	for (int i = EroElement.size() - 1; i >= 0; i--) {
		int idx = EroElement[i];
		Eigen::Vector3i currentPos = this->modelObj3D->indexToCoordinate(idx);
		
		// Check if the current element can be added to an existing group
		bool addedToExistingGroup = false;

		// Check the previous element in EroElement
		if (currentPos(2) == lastHeight) {			
			// Check if currentPos is connected to any element in the last group
			bool connected = false;
			for (const auto& element : groups.back()) {
				Eigen::Vector3i neighborPos = this->modelObj3D->indexToCoordinate(element);
				if ((currentPos - neighborPos).cwiseAbs().sum() == 1) {
					connected = true;
					break;
				}
			}
			if (connected) {
				groups.back().push_back(idx); 
				addedToExistingGroup = true;
			}
		}
		
		// If the current element is not connected to any existing group, create a new group
		if (!addedToExistingGroup) {
			if (groups.size()>1){
				// check if there is other group with the same height and connected to the last group
				for (int g = 0; g < groups.size() - 1; g++) {
					bool sameHeight = true;
					bool connected = false;

					// check if the height is the same
					Eigen::Vector3i pos = this->modelObj3D->indexToCoordinate(groups[g].back());
					if (pos(2) != lastHeight) {
						sameHeight = false;
						break;
					}
					
					// check if the group is connected
					if (sameHeight) {
						for (const auto& element1 : groups[g]) {
							Eigen::Vector3i pos1 = this->modelObj3D->indexToCoordinate(element1);
							for (const auto& element2 : groups.back()) {
								Eigen::Vector3i pos2 = this->modelObj3D->indexToCoordinate(element2);
								if ((pos1 - pos2).cwiseAbs().sum() == 1) {
									connected = true;
									break;
								}
							}
							if (connected) break;
						}
					}
					
					// if the group is connected, merge the group
					if (sameHeight && connected) {
						groups[g].insert(groups[g].end(), groups.back().begin(), groups.back().end());
						groups.pop_back();
						break;
					}
				}
			}
			
			std::vector<int> newGroup;
			newGroup.push_back(idx);
			groups.push_back(newGroup);
			lastHeight = currentPos(2);
		}
	}

	for (int i = 0; i < groups.size(); i++) {
		std::vector<int> groupTmp = groups[i];
		//if (checkConnected(groupTmp, 0) < 3) {
		//	std::cout << "Error: Ero Element is not connected." << std::endl;
		//}

		if (!opt) {
			for (int j = 0; j < groups[i].size(); j++) {
				int idx = groups[i][j];
				T.Time[idx].push_back(std::make_pair(T.tLast + 1, -1));
			}
			T.tLast = T.tLast + 1;
		}
		else{
			this->addNewTime(T, T.tLast + 1, groups[i], checkTime, this->redunWindow);
		}
		
		// update the model
		for (int j = 0; j < groups[i].size(); j++) {
			int idx = groups[i][j];
			this->modelObj3D->currentModelValue(idx) = 1;

			// update the limits
			Eigen::Vector3i pos = this->modelObj3D->indexToCoordinate(idx);
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
		}
	}
}

void AccEro3DNew::addNewTime(HMP_struct3D& T, int tNew, std::pair<int, int> idxAndDir, Eigen::VectorXi& checkTime, int threshold) {

	int idx = idxAndDir.first;
	int tLast = T.tLast;

	if (tNew < tLast) {
		std::cout << "Error: tNew is smaller than tLast." << std::endl;
	}

	T.Time[idx].push_back(std::make_pair(tNew, idxAndDir.second));
	T.tLast = tNew;
	checkTime = checkTime.cwiseMin(tNew);

	if (T.Time[idx].size() == 0) {
		checkTime(idx) = tNew;
		return;
	}

	std::vector<int> checedTimeList;
	int j = T.Time[idx].size() - 2;
	if (j % 2 == 1 - this->modelObj3D->startModelValue(idx)) {
		std::cout << "Error: j is odd." << std::endl;
	}

	while (j >= 0) {
		int amTime = T.Time[idx][j].first;
		std::pair<int, int> currentPosition = std::make_pair(idx, j);
		this->checkAMRedundancy(T, amTime, checkTime, threshold, checedTimeList, currentPosition);
		j -= 2;
	}

	if (T.Time[idx].size()==0){
		checkTime(idx) = 0;
	}
	else{
		checkTime(idx) = T.Time[idx].back().first;
	}
}

int AccEro3DNew::checkConnected(std::vector<int>& idxList, int currentIdx){
	std::vector<Eigen::Vector3i> positions;
	int spectialIdx = -1;
	for (int i = 0; i < idxList.size(); i++) {
		positions.push_back(this->modelObj3D->indexToCoordinate(idxList[i]));
		if (idxList[i] == currentIdx) {
			spectialIdx = i;
		}
	}

	// Check if all positions are connected as a single block
	if (positions.empty()) {
		return true;
	}
	
	// If there is only one position, consider it connected
	if (positions.size() == 1) {
		return true;
	}
	
	// Create an adjacency list to represent the graph
	std::unordered_map<int, std::vector<int>> graph;
	
	// For each position, find its neighbors
	for (int i = 0; i < positions.size(); i++) {
		Eigen::Vector3i pos_i = positions[i];
		for (int j = i + 1; j < positions.size(); j++) {
			Eigen::Vector3i pos_j = positions[j];
			Eigen::Vector3i diff = (pos_i - pos_j).cwiseAbs();
			
			// Check if the two positions are neighbors (18-connectivity: face or edge neighbors)
			if (diff.sum() == 1) {
				if (graph.find(i) == graph.end()) {
					graph[i] = std::vector<int>();
				}
				if (graph.find(j) == graph.end()) {
					graph[j] = std::vector<int>();
				}
				graph[i].push_back(j);
				graph[j].push_back(i);
			}
		}
	}
	
	// Use BFS to check connectivity
	std::vector<bool> visited(positions.size(), false);
	std::vector<std::vector<int>> connectedComponents;
	for (int i = 0; i < positions.size(); i++) {
		if (!visited[i]) {
			std::vector<int> connectedComponent;
			std::queue<int> q;
			q.push(i);
			visited[i] = true;
			while (!q.empty()) {
				int current = q.front();
				q.pop();
				connectedComponent.push_back(current);

				for (int neighbor : graph[current]) {
					if (!visited[neighbor]) {
						visited[neighbor] = true;
						q.push(neighbor);
					}
				}
			}
			connectedComponents.push_back(connectedComponent);
		}
	}

	if (connectedComponents.size() == 1) {
		if (spectialIdx >= 0) {
			return 3;
		}
		else {
			return 4;
		}
	}

	for (int i = 0; i < connectedComponents.size(); i++) {
		if (connectedComponents[i].size() < this->AMMiniSize) {
			return 0;
		}
		if (spectialIdx >= 0) {
			if (std::find(connectedComponents[i].begin(), connectedComponents[i].end(), spectialIdx) != connectedComponents[i].end()) {
				idxList.clear();
				for (int j = 0; j < connectedComponents[i].size(); j++) {
					Eigen::Vector3i posTmp = positions[connectedComponents[i][j]];
					int idxTmp = this->modelObj3D->coordinateToIndex(posTmp);
					idxList.push_back(idxTmp);
				}
			}
		}
	}

	if (spectialIdx >= 0) {
		return 1;
	}
	else {
		return 2;
	}

}

int AccEro3DNew::checkAMRedundancy(HMP_struct3D& T, int amTime, Eigen::VectorXi& checkTime, int threshold, std::vector<int>& checkedTime, std::pair<int, int> currentPosition) {
	bool flagR = false;
	int redundantFlag = 0;
	int idx = currentPosition.first;
	int j = currentPosition.second;
	if (j < T.Time[idx].size() - 1 && T.Time[idx][j + 1].first - T.Time[idx][j].first <= threshold) {
		if (T.Time[idx][j + 1].first > checkTime(idx)) {
			flagR = this->solidToEmpty(T, T.Time[idx][j].first, T.Time[idx][j + 1].first, idx);
		}
	}
	if (flagR) {
		redundantFlag = 1;
	}
	if (!flagR) {
		if (j > 0 && T.Time[idx][j].first - T.Time[idx][j - 1].first <= threshold) {
			if (T.Time[idx][j].first > checkTime(idx)) {
				flagR = this->emptyToSolid(T, T.Time[idx][j - 1].first, T.Time[idx][j].first, idx);
			}
		}
		if (flagR) {
			redundantFlag = 2;
		}
	}
	if (!flagR) {
		return 0;
	}
	int checkTimeTmp;
	if (redundantFlag == 1) {
		checkTimeTmp = T.Time[idx][j].first;
		T.Time[idx].erase(T.Time[idx].begin() + j);
		T.Time[idx].erase(T.Time[idx].begin() + j);
	}
	else if (redundantFlag == 2) {
		checkTimeTmp = T.Time[idx][j - 1].first;
		T.Time[idx].erase(T.Time[idx].begin() + j - 1);
		T.Time[idx].erase(T.Time[idx].begin() + j - 1);
	}
	checkTime = checkTime.cwiseMin(checkTimeTmp);
	return 1;
}

void AccEro3DNew::addNewTime(HMP_struct3D& T, int tNew, std::vector<int> AMidxList, Eigen::VectorXi& checkTime, int threshold) {

	int tLast = T.tLast;
	if (tNew < tLast) {
		std::cout << "Error: tNew is smaller than tLast." << std::endl;
	}

	bool flag = false;

	if (AMidxList.size() < this->AMMiniSize) {
		std::cout << "Error: the number of positions is less than AMMiniSize." << std::endl;
	}


	// set the last time
	T.tLast = tNew;
	checkTime = checkTime.cwiseMin(tNew);
	for (int i = 0; i < AMidxList.size(); i++) {
		int idx = AMidxList[i];
		std::vector<std::pair<int, int>> tIdx = T.Time[idx];
		tIdx.push_back(std::make_pair(tNew, -1));
		T.Time[idx] = tIdx;
		if (tIdx.size() == 0) {
			checkTime(idx) = tNew;
			flag=true;
		}
	}

	if (flag && AMidxList.size()==this->AMMiniSize){
		return;
	}

	std::vector<int> checkedTimeList;
	for (int i=0;i<AMidxList.size();i++){
		int idx = AMidxList[i];
		int j=T.Time[idx].size()-1;
		if (j % 2 == 1 - this->modelObj3D->startModelValue(idx)) {
			std::cout << "Error: j is odd." << std::endl;
		}
		while (j >= 0) {
			int amTime = T.Time[idx][j].first;
			std::pair<int, int> currentPosition = std::make_pair(idx, j);
			this->checkAMRedundancy(T, amTime, checkTime, threshold, checkedTimeList, currentPosition);
			j -= 2;
		}

		if (T.Time[idx].size() == 0) {
			checkTime(idx) = 0;
		}
		else {
			checkTime(idx) = T.Time[idx].back().first;
		}
	}

}


bool AccEro3DNew::changeLastTime(HMP_struct3D& T, int tChange, int idx, Eigen::VectorXi& checkTime, int shreshold) {
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

bool AccEro3DNew::isSupportFree(const HMP_struct3D& T, int idx, int time, int emptyIdx) const{
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

	int checkIdx = this->modelObj3D->coordinateToIndex(checkPos);
	int status = getStatusFromTime(T, time, statusT, checkIdx);

	if (status == 1) {
		return true;
	}

	std::vector<Eigen::Vector3i> solidPos;
	int count = this->getSolidFaceNeighborSameHeight(T, time, statusT, checkPos, solidPos);
	if (count == 0) {
		return isSupportFree = false;
	}
	else {
		return true;
	}
	

	checkIdx = this->modelObj3D->coordinateToIndex(checkPos);
	status = getStatusFromTime(T, time, statusT, checkIdx);
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

int AccEro3DNew::getSolidFaceNeighborSameHeight(const HMP_struct3D& T, int time, Eigen::VectorXi& statusT, Eigen::Vector3i pos, std::vector<Eigen::Vector3i>& neighbor) const{
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

int AccEro3DNew::getStatusFromTime(const HMP_struct3D& T, int time, Eigen::VectorXi& statusT, int idx) const{
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

bool AccEro3DNew::isColliFreeAM(const HMP_struct3D& T, Eigen::Vector3i tipPoint, int time) const{
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

void AccEro3DNew::deAcc(HMP_struct3D& T, std::vector<std::pair<int, int>> AccElement, Eigen::VectorXi& Limits, Eigen::MatrixXi& height, Eigen::VectorXi& checkTime, bool opt) {

	for (int i = 0; i < AccElement.size(); i++) {
		std::pair<int, int> idxAndDir = AccElement[AccElement.size() - i - 1];

		if (!opt) {
			T.Time[idxAndDir.first].push_back(std::make_pair(T.tLast + 1, idxAndDir.second));
			T.tLast = T.tLast + 1;
		}
		else {
			this->addNewTime(T, T.tLast + 1, idxAndDir, checkTime, this->redunWindow);
		}
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
	Limits(4) = height.maxCoeff();	
}

bool AccEro3DNew::emptyToSolid(const HMP_struct3D& T, int startCheckTime, int endCheckTime, int idx) const {
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

bool AccEro3DNew::hasSolidNeighbor(const HMP_struct3D& T, int time, Eigen::Vector3i pos) const {
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

bool AccEro3DNew::isColliFreeSM(const HMP_struct3D& T, int posIdx, int smIdx, Eigen::Vector3i checkPos) const {
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

bool AccEro3DNew::solidToEmpty(const HMP_struct3D& T, int startCheckTime, int endCheckTime, int idx) const {
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

				//if (tNow == 676 && startCheckTime==118 && endCheckTime==727) {
				//	modelFromTime = getModelFromTime(T, tNow, idx);
				//	fileIO* IO_operator = new fileIO();
				//	IO_operator->writeVector(modelFromTime, "M3.txt");
				//	delete IO_operator;
				//}

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

Eigen::VectorXi AccEro3DNew::getModelFromTime(const HMP_struct3D& T, int time, int emptyIdx) const {
	Eigen::VectorXi modelFromTime = this->modelObj3D->onlyBrimValue;

	for (int i = 0; i < this->modelObj3D->nSpace; i++) {
		if (i == emptyIdx) {
			modelFromTime(i) = 0;
			continue;
		}
		if (T.Time[i].size() == 0) {
			modelFromTime(i) = this->modelObj3D->startModelValue(i);
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

Eigen::MatrixXi AccEro3DNew::hmp2Matrix(const HMP_struct3D& T, int& threshold) const {
	int nAS = 0;
	for (int i = 0; i < T.Time.size(); i++) {
		if (T.Time[i].size() > nAS) {
			nAS = T.Time[i].size();
		}
	}

	int tEnd = T.tLast;
	threshold = tEnd + 2;

	int nxCore = this->modelObj3D->nx - 2;
	int nyCore = this->modelObj3D->ny - 2;
	int nzCore = this->modelObj3D->nz - 1;

	//std::cout << nxCore << " " << nyCore << " " << nzCore << std::endl;

	Eigen::MatrixXi M(nxCore * nyCore * nzCore, nAS);
	M.setOnes();
	M = M * (threshold + 1);

	for (int i = 0; i < nxCore * nyCore * nzCore; i++) {
		int zCore = i / (nxCore * nyCore);
		int yCore = (i / nxCore) % nyCore;
		int xCore = i % nxCore;
		Eigen::Vector3i posGlobal(xCore + 1, yCore + 1, zCore + 1);
		int idx = this->modelObj3D->coordinateToIndex(posGlobal);
		for (int j = 0; j < T.Time[idx].size(); j++) {
			M(i, j) = T.Time[idx][j].first;
		}
	}

	//for (int i = 0; i < T.Time.size(); i++) {
	//	for (int j = 0; j < T.Time[i].size(); j++) {
	//		M(i, j) = T.Time[i][j].first;
	//	}
	//}

	return M;
}

std::vector<Eigen::Vector3i> AccEro3DNew::_getToolPosInRadius(Eigen::Vector3i xyzPos, int toolDirection, double radius) {
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


void AccEro3DNew::preProcess(Eigen::VectorXi& resultModel, std::vector<int>& growedList, double& time) {
	auto start = std::chrono::high_resolution_clock::now();

	std::vector<int> overhangList = this->checkOverHang();
	this->sortOverHang(overhangList);

	std::cout << "Overhang size: " << overhangList.size() << std::endl;

	growedList.clear();

	while (overhangList.size() > 0) {
		if (overhangList.size() % 10 == 0) {
			std::cout << overhangList.size() << " ";
		}

		int idx = overhangList.back();
		overhangList.pop_back();
		Eigen::Vector3i pos = this->modelObj3D->indexToCoordinate(idx);

		std::vector<std::pair<std::pair<int, int>, int>> growTmpTmp;
		std::vector<Eigen::Vector3i> growTmp;
		growTmp.push_back(pos);
		std::queue<Eigen::Vector3i> queue;
		queue.push(pos);

		bool success = false;
		while (!queue.empty()) {
			Eigen::Vector3i curPos = queue.front();
			queue.pop();

			std::vector<Eigen::Vector3i> baseNeighbor;
			Eigen::Vector3i neighborTmp = curPos;
			neighborTmp(2) -= 1;
			baseNeighbor.push_back(neighborTmp);
			if (neighborTmp(0) > this->modelObj3D->outerBrim) {
				Eigen::Vector3i neighborTmpTmp = neighborTmp;
				neighborTmpTmp(0) -= 1;
				baseNeighbor.push_back(neighborTmpTmp);
			}
			if (neighborTmp(0) < this->modelObj3D->nx - this->modelObj3D->outerBrim - 1) {
				Eigen::Vector3i neighborTmpTmp = neighborTmp;
				neighborTmpTmp(0) += 1;
				baseNeighbor.push_back(neighborTmpTmp);
			}
			if (neighborTmp(1) > this->modelObj3D->outerBrim) {
				Eigen::Vector3i neighborTmpTmp = neighborTmp;
				neighborTmpTmp(1) -= 1;
				baseNeighbor.push_back(neighborTmpTmp);
			}
			if (neighborTmp(1) < this->modelObj3D->ny - this->modelObj3D->outerBrim - 1) {
				Eigen::Vector3i neighborTmpTmp = neighborTmp;
				neighborTmpTmp(1) += 1;
				baseNeighbor.push_back(neighborTmpTmp);
			}
			this->sortOverHang(baseNeighbor);

			while (!baseNeighbor.empty()) {
				Eigen::Vector3i checkPos = baseNeighbor.back();
				baseNeighbor.pop_back();
				int idxNeighbor = this->modelObj3D->coordinateToIndex(checkPos);
				if (this->modelObj3D->currentModelValue(idxNeighbor) == 1 || checkPos(2) == 0) {
					growTmp.push_back(checkPos);
					success = true;
					break;
				}

				int smAccess = this->smAccessible(checkPos, this->modelObj3D->nz - 1, this->modelObj3D->currentModelValue, growTmpTmp);
				if (smAccess >= 0) {
					std::pair<int, int> idxAndDir = std::make_pair(idxNeighbor, smAccess);
					growTmpTmp.push_back(std::make_pair(idxAndDir, 0));
					growTmp.push_back(checkPos);
					queue.push(checkPos);
				}
			}

			if (success) {
				break;
			}
		}

		if (success) {
			std::vector<int> growPath = this->searchGrowPath(growTmp);
			if (growPath.size() <= 2) {
				continue;
			}
			growedList.insert(growedList.end(), growPath.begin() + 1, growPath.end() - 1);
			for (int i = 1; i < growPath.size() - 1; i++) {
				this->modelObj3D->currentModelValue(growPath[i]) = 1;
			}
		}

	}
	std::cout << std::endl;

	resultModel = this->modelObj3D->currentModelValue;

	auto end = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> elapsed = end - start;
	std::cout << "Preprocess time: " << elapsed.count() << " seconds" << std::endl;
	time = elapsed.count();
}





std::vector<int> AccEro3DNew::checkOverHang() {
	std::vector<int> overhangList;
	for (int i = 0; i < this->modelObj3D->nSpace; i++) {
		if (this->modelObj3D->currentModelValue(i) == 0) {
			continue;
		}

		Eigen::Vector3i pos = this->modelObj3D->indexToCoordinate(i);
		if (pos(2) == 0) {
			continue;
		}

		bool supportFree = this->modelObj3D->isSupportingFree(pos);
		if (!supportFree) {
			overhangList.push_back(i);
		}
	}

	return overhangList;
}

void AccEro3DNew::sortOverHang(std::vector<int>& overhangList) {
	int nx = this->modelObj3D->nx;
	int ny = this->modelObj3D->ny;

	std::sort(overhangList.begin(), overhangList.end(), [this, nx, ny](int a, int b) {
		Eigen::Vector3i va = this->modelObj3D->indexToCoordinate(a);
		Eigen::Vector3i vb = this->modelObj3D->indexToCoordinate(b);
		int minA = std::min({ va.x(),va.y(),nx - 1 - va.x(),ny - 1 - va.y() });
		int minB = std::min({ vb.x(),vb.y(),nx - 1 - vb.x(),ny - 1 - vb.y() });
		if (minA != minB) {
			return minA < minB;
		}
		else {
			return va.z() > vb.z();
		}
		});
}

void AccEro3DNew::sortOverHang(std::vector<Eigen::Vector3i>& overhangList) {
	int nx = this->modelObj3D->nx;
	int ny = this->modelObj3D->ny;

	std::sort(overhangList.begin(), overhangList.end(), [this, nx, ny](Eigen::Vector3i va, Eigen::Vector3i vb) {
		int minA = std::min({ va.x(),va.y(),nx - 1 - va.x(),ny - 1 - va.y() });
		int minB = std::min({ vb.x(),vb.y(),nx - 1 - vb.x(),ny - 1 - vb.y() });
		if (minA != minB) {
			return minA < minB;
		}
		else {
			return va.z() > vb.z();
		}
		});
}

struct Vector3iHash {
	std::size_t operator()(const Eigen::Vector3i& v) const {
		return std::hash<int>()(v.x()) ^ std::hash<int>()(v.y() << 1) ^ std::hash<int>()(v.z() << 2);
	}
};

std::vector<int> AccEro3DNew::searchGrowPath(std::vector<Eigen::Vector3i>growList) {

	int n = growList.size();
	std::unordered_map<Eigen::Vector3i, int, Vector3iHash> posMap;
	for (int i = 0; i < n; ++i)
		posMap[growList[i]] = i;

	std::vector<int> prev(n, -1);
	std::deque<int> queue;

	queue.push_back(n - 1);  // from the end to the start

	while (!queue.empty()) {
		int cur = queue.front();
		queue.pop_front();

		for (int i = cur - 1; i >= 0; --i) {
			if (growList[i](2) - growList[cur](2) > 1) {
				continue;
			}

			if (prev[i] == -1 && isConnect(growList[i], growList[cur])) {
				prev[i] = cur;
				if (i == 0) {  // found the start point
					std::vector<int> path;
					int p = 0;
					while (p != -1) {
						int idxTmp = this->modelObj3D->coordinateToIndex(growList[p]);
						path.push_back(idxTmp);
						p = prev[p];
					}
					return path;
				}
				queue.push_back(i);
			}
		}
	}

	std::cout << "Error: no path found." << std::endl;
	return {}; 
}

bool AccEro3DNew::isConnect(Eigen::Vector3i pos1, Eigen::Vector3i pos2) {
	Eigen::Vector3i diff = pos1 - pos2;
	if (std::abs(diff(2)) != 1) {
		return false;
	}
	if (diff.cwiseAbs().maxCoeff() > 1) {
		return false;
	}
	if (diff.cwiseAbs().sum() == 3) {
		return false;
	}
	return true;
}