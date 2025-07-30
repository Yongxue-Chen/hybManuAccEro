#include "postProcess.h"

void postProcess::tP2PathFile(const Eigen::MatrixXd& tP, double tEnd, const std::string& modelName) {
	Eigen::MatrixXi sequence = tP2Sequnce(tP, tEnd);
	std::vector<opeGroup> groupSet = sequence2Group(sequence);
	groupSet = adjustGroupSet(groupSet);

	if (this->justForShowing) {
		fileIO* IO_operator = new fileIO();
		Eigen::VectorXi groupSize;
		groupSize.resize(4);
		groupSize(0) = groupSet.size();
		groupSize(1) = this->modelObj->nx;
		groupSize(2) = this->modelObj->ny;
		groupSize(3) = this->modelObj->nz;
		IO_operator->writeVector(groupSize, "../DataSet/3DGroupResult/groupSize.txt");
		for (int i = 0; i < groupSet.size(); i++) {
			IO_operator->writeVoxelGroup(groupSet[i].voxelPos, groupSet[i].opeType, "../DataSet/3DGroupResult/group" + std::to_string(i) + ".txt");
		}
		delete IO_operator;
		//return;
	}

	std::vector<pathGroup> pathGroupSet = voxelGroup2PathGroup(groupSet);
	std::cout << "get path group" << std::endl;
	pathGroup2CommandFile(pathGroupSet, modelName, false);
}

void postProcess::tP2Field(const Eigen::MatrixXd& tP, double tEnd, const std::string& modelName, const std::string& paraName) {
	Eigen::MatrixXi sequence = tP2Sequnce(tP, tEnd);
	std::vector<opeGroup> groupSet = sequence2Group(sequence);
	groupSet = adjustGroupSet(groupSet);

	Eigen::MatrixXi fieldValue = Eigen::MatrixXi::Ones(this->modelObj->nx * this->modelObj->ny * this->modelObj->nz, 2);
	fieldValue = fieldValue * (int(tEnd) + 1);
	for (int i= 0; i < groupSet.size(); i++) {
		opeGroup voxelGroup = groupSet[i];
		int opeType = voxelGroup.opeType;
		for (int j = 0; j < voxelGroup.voxelPos.size(); j++) {
			Eigen::Vector3i pos = voxelGroup.voxelPos[j];
			int idx = this->modelObj->coordinateToIndex(pos);
			if (opeType == 100000) {
				fieldValue(idx, 0) = i;
			}
			else {
				fieldValue(idx, 1) = i;
			}
		}
	}

	Eigen::MatrixXi field = Eigen::MatrixXi::Zero(fieldValue.rows(), 5);

	for (int i = 0; i < fieldValue.rows(); i++) {
		Eigen::Vector3i pos = this->modelObj->indexToCoordinate(i);
		field(i, 0) = pos(0);
		field(i, 1) = pos(1);
		field(i, 2) = pos(2);
		field(i, 3) = fieldValue(i, 0);
		field(i, 4) = fieldValue(i, 1);
	}

	fileIO* IO_operator = new fileIO();
	IO_operator->writeMatrix(field, "../DataSet/outputTime3D/" + modelName + "_" + paraName + "_field.txt");
	delete IO_operator;
}

Eigen::MatrixXi postProcess::tP2Sequnce(const Eigen::MatrixXd& tP, double tEnd) {
	std::vector<matrixElement> elements;
	elements.reserve(tP.size());
	for (int i = 0; i < tP.rows(); i++) {
		for (int j = 0; j < tP.cols(); j++) {
			matrixElement element;
			element.row = i;
			element.col = j;
			element.value = tP(i, j);
			if (element.value < tEnd) {
				if (j == 0 && i < this->modelObj->nx * this->modelObj->ny) {
					element.value = 0;
				}
				elements.push_back(element);
			}
		}
	}
	std::sort(elements.begin(), elements.end(), [](const matrixElement& a, const matrixElement& b) {
		return a.value < b.value;
		});
	Eigen::MatrixXi sequence(elements.size(), 2);
//	 operation type: 
//	 AM SMup SM-x SM+x SM-y SM+y
//	 e.g., 100000: AM; 010000: SMup; 011000: SMup and SM-x
	Eigen::VectorXi model = Eigen::VectorXi::Zero(tP.rows());
	for (int i = 0; i < elements.size(); i++) {
		sequence(i, 0) = elements[i].row;
		if (elements[i].col % 2 == 0) {
			if (model(elements[i].row) == 1) {
				std::cout << "error" << std::endl;
			}
			//std::cout << elements[i].row << " " << elements[i].col << " " << elements[i].value << std::endl;
			sequence(i, 1) = 100000;
			model(elements[i].row) = 1;
		}
		else {
			if (model(elements[i].row) == 0) {
				std::cout << "error" << std::endl;
			}

			//std::cout << elements[i].row << " " << elements[i].col << " " << elements[i].value << std::endl;
			sequence(i, 1) = checkSMAccess(model, elements[i].row);
			model(elements[i].row) = 0;
		}

		if (sequence(i, 1) == 0) {
			std::cout << "Error: sequence type is 0!" << std::endl;
		}

	}
	
	std::cout << "get sequence" << std::endl;
	return sequence;
}

int postProcess::checkSMAccess(const Eigen::MatrixXi& model, int idx) {
	AccEro3DNew* accEroObj = new AccEro3DNew(this->modelObj, 4, true, 10);
	Eigen::Vector3i xyzPos = this->modelObj->indexToCoordinate(idx);

	std::vector<int> status(5, 1);

#pragma omp parallel for
	for (int toolDirection = 0; toolDirection < 5; toolDirection++) {
		int increEnd;
		int increStart = 1;
		Eigen::Vector3i direction;
		switch (toolDirection) {
		case 0:
			direction = Eigen::Vector3i(0, 0, 1);
			increEnd = this->modelObj->nz - 1 - xyzPos(2);
			break;
		case 1:
			direction = Eigen::Vector3i(-1, 0, 0);
			increEnd = xyzPos(0);
			break;
		case 2:
			direction = Eigen::Vector3i(1, 0, 0);
			increEnd = this->modelObj->nx - 1 - xyzPos(0);
			break;
		case 3:
			direction = Eigen::Vector3i(0, -1, 0);
			increEnd = xyzPos(1);
			break;
		case 4:
			direction = Eigen::Vector3i(0, 1, 0);
			increEnd = this->modelObj->ny - 1 - xyzPos(1);
			break;
		default:
			std::cout << toolDirection << std::endl;
			std::cout << "Error: toolDirection is out of range!" << std::endl;
			break;
		}

		//if (toolDirection > 0) {
		//	if (xyzPos(2) < accEroObj->radiusSM) {
		//		status[toolDirection] = 0;
		//		continue;
		//	}
		//}

		std::vector<Eigen::Vector3i> toolPosInRadius;
		for (int incre = increStart; incre <= increEnd; incre++) {
			if (incre < accEroObj->lengthSM) {
				Eigen::Vector3i posTmp = xyzPos + incre * direction;
				int idxTmp = this->modelObj->coordinateToIndex(posTmp);
				if (model(idxTmp) == 1) {
					status[toolDirection] = 0;
					break;
				}
			}
			else {
				if (toolDirection > 0) {
					if (xyzPos(2) < accEroObj->radiusSM) {
						status[toolDirection] = 0;
						break;
					}
				}
				if (incre == accEroObj->lengthSM) {
					toolPosInRadius = accEroObj->_getToolPosInRadius(xyzPos, toolDirection, accEroObj->radiusSM);
				}
				for (int i = 0; i < toolPosInRadius.size(); i++) {
					Eigen::Vector3i posTmp = toolPosInRadius[i] + incre * direction;
					int idxTmp = this->modelObj->coordinateToIndex(posTmp);
					if (model(idxTmp) == 1) {
						status[toolDirection] = 0;
						break;
					}
				}
				if (status[toolDirection] == 0) break;
			}
		}
	}

	//std::vector<int> statusTmp(5, 0);
	//for (int i = 4; i >= 0; i--) {
	//	if (status[i] == 1) {
	//		statusTmp[i] = 1;
	//		break;
	//	}
	//}
	//status = statusTmp;

	std::vector<int> statusTmp(5, 0);
	for (int i = 0; i < 5; i++) {
		if (status[i] == 1) {
			statusTmp[i] = 1;
			break;
		}
	}
	//for (int i = 4; i >= 0; i--) {
	//	if (status[i] == 1) {
	//		statusTmp[i] = 1;
	//		break;
	//	}
	//}
	status = statusTmp;

	int result = status[4] + 10 * status[3] + 100 * status[2] + 1000 * status[1] + 10000 * status[0];
	delete accEroObj;

	if (result == 0) {
		std::cout << "Error: all directions are blocked!" << std::endl;
		std::cout << accEroObj->lengthSM << std::endl;
	}

	return result;
}

std::vector<opeGroup> postProcess::sequence2Group(const Eigen::MatrixXi& sequence) {
	std::vector<opeGroup> groupSet;

	//std::cout << sequence.col(1).minCoeff() << std::endl;
	
	int idx = 0;
	opeGroupRough group;
	while (idx < sequence.rows()) {
		Eigen::Vector3i pos = this->modelObj->indexToCoordinate(sequence(idx, 0));
		int opeType = sequence(idx, 1);
		std::pair<Eigen::Vector3i, int> voxelRough(pos, opeType);

		if (idx == 0) {
			group.voxelPos.push_back(voxelRough);
			idx++;
			continue;
		}

		bool newGroup = false;
		if (opeType == 100000) {
			if (group.voxelPos[0].second < 100000) {
				newGroup = true;
			}
		}
		else if (group.voxelPos[0].second == 100000 || group.voxelPos[0].second < 0) {
			newGroup = true;
		}

		if (newGroup) {
			if (group.voxelPos[0].second == 100000) {
				std::vector<opeGroup> groupSameHeight = splitGroupIntoSameHeight(group);
				groupSet.insert(groupSet.end(), groupSameHeight.begin(), groupSameHeight.end());
				// for (int i = 0; i < groupSameHeight.size(); i++) {
				// 	opeGroup subGroup = groupSameHeight[i];
				// 	std::vector<opeGroup> splitedGroups = splitGroupIntoConnected(subGroup);
				// 	groupSet.insert(groupSet.end(), splitedGroups.begin(), splitedGroups.end());
				// }
			}
			else {
				std::vector<opeGroup> sameOpeGroups = splitGroupIntoSameOpe(group);
				groupSet.insert(groupSet.end(), sameOpeGroups.begin(), sameOpeGroups.end());
				// for (int i = 0; i < sameOpeGroups.size(); i++) {
				// 	std::vector<opeGroup> connectedGroups = splitGroupIntoConnected(sameOpeGroups[i]);
				// 	for (int j = 0; j < connectedGroups.size(); j++) {
				// 		opeGroup subGroup = connectedGroups[j];
				// 		std::vector<opeGroup> splitedGroups = splitSMGroup(subGroup);
				// 		groupSet.insert(groupSet.end(), splitedGroups.begin(), splitedGroups.end());
				// 	}
				// }
			}
			group.voxelPos.clear();
		}

		group.voxelPos.push_back(voxelRough);

		idx++;
	}

	if (group.voxelPos[0].second == 100000) {
		std::vector<opeGroup> groupSameHeight = splitGroupIntoSameHeight(group);
		groupSet.insert(groupSet.end(), groupSameHeight.begin(), groupSameHeight.end());
		// for (int i = 0; i < groupSameHeight.size(); i++) {
		// 	opeGroup subGroup = groupSameHeight[i];
		// 	std::vector<opeGroup> splitedGroups = splitGroupIntoConnected(subGroup);
		// 	groupSet.insert(groupSet.end(), splitedGroups.begin(), splitedGroups.end());
		// }
	}
	else {
		std::vector<opeGroup> sameOpeGroups = splitGroupIntoSameOpe(group);
		groupSet.insert(groupSet.end(), sameOpeGroups.begin(), sameOpeGroups.end());
		// for (int i = 0; i < sameOpeGroups.size(); i++) {
		// 	std::vector<opeGroup> connectedGroups = splitGroupIntoConnected(sameOpeGroups[i]);
		// 	for (int j = 0; j < connectedGroups.size(); j++) {
		// 		opeGroup subGroup = connectedGroups[j];
		// 		std::vector<opeGroup> splitedGroups = splitSMGroup(subGroup);
		// 		groupSet.insert(groupSet.end(), splitedGroups.begin(), splitedGroups.end());
		// 	}
		// }
	}

	std::cout << "get group" << std::endl;
	return groupSet;
}

std::vector<opeGroup> postProcess::splitGroupIntoConnected(opeGroup group) {
	std::unordered_set<Eigen::Vector3i, Vector3iHash, Vector3iEqual> voxelSet(group.voxelPos.begin(), group.voxelPos.end());
	std::vector<std::vector<Eigen::Vector3i>> connectedComponents;

	while (!voxelSet.empty()) {
		Eigen::Vector3i start = *voxelSet.begin();
		std::vector<Eigen::Vector3i> component;
		std::queue<Eigen::Vector3i> q;

		q.push(start);
		voxelSet.erase(start);

		while (!q.empty()) {
			Eigen::Vector3i voxel = q.front();
			q.pop();
			component.push_back(voxel);

			for (const auto& neighbor : getNeighbors(voxel)) {
				if (voxelSet.find(neighbor) != voxelSet.end()) {
					q.push(neighbor);
					voxelSet.erase(neighbor);
				}
			}
		}
		connectedComponents.push_back(component);
	}

	std::vector<opeGroup> splitedGroups;
	for (const auto& component : connectedComponents) {
		opeGroup newGroup;
		newGroup.voxelPos = component;
		newGroup.opeType = group.opeType;
		newGroup.rangeLimitLow = component[0];
		newGroup.rangeLimitHigh = component[0];
		for (const auto& voxel : component) {
			updateGroupRange(newGroup, voxel);
		}
		splitedGroups.push_back(newGroup);
	}

	return splitedGroups;
}

std::vector<opeGroup> postProcess::splitGroupIntoSameHeight(opeGroupRough group) {
	std::vector<std::pair<Eigen::Vector3i, int>> voxels = group.voxelPos;
	//sort voxels according to z value
	std::sort(voxels.begin(), voxels.end(), [](const std::pair<Eigen::Vector3i, int>& a, const std::pair<Eigen::Vector3i, int>& b) {
		return a.first(2) < b.first(2);
		});

	std::vector<opeGroup> splitedGroups;
	opeGroup subGroup;
	subGroup.opeType = group.voxelPos[0].second;

	for (int i = 0; i < voxels.size(); i++) {
		if (i == 0) {
			subGroup.voxelPos.push_back(voxels[i].first);
			subGroup.rangeLimitLow = voxels[i].first;
			subGroup.rangeLimitHigh = voxels[i].first;
			if (i == voxels.size() - 1) {
				splitedGroups.push_back(subGroup);
			}
			continue;
		}

		if (voxels[i].first(2) != voxels[i - 1].first(2)) {
			splitedGroups.push_back(subGroup);
			subGroup.voxelPos.clear();
			subGroup.voxelPos.push_back(voxels[i].first);
			subGroup.rangeLimitLow = voxels[i].first;
			subGroup.rangeLimitHigh = voxels[i].first;
		}
		else {
			subGroup.voxelPos.push_back(voxels[i].first);
			updateGroupRange(subGroup, voxels[i].first);
		}

		if (i == voxels.size() - 1) {
			splitedGroups.push_back(subGroup);
		}
	}

	return splitedGroups;
}

std::vector<opeGroup> postProcess::splitGroupIntoSameOpe(opeGroupRough group) {
	std::vector<std::pair<Eigen::Vector3i, int>> voxels = group.voxelPos;
	//sort voxels according to z value
	std::sort(voxels.begin(), voxels.end(), [](const std::pair<Eigen::Vector3i, int>& a, const std::pair<Eigen::Vector3i, int>& b) {
		return a.second < b.second;
		});

	std::vector<opeGroup> splitedGroups;
	opeGroup subGroup;

	for (int i = 0; i < voxels.size(); i++) {
		if (i == 0) {
			subGroup.voxelPos.push_back(voxels[i].first);
			subGroup.opeType = voxels[i].second;
			subGroup.rangeLimitLow = voxels[i].first;
			subGroup.rangeLimitHigh = voxels[i].first;
			if (i == voxels.size() - 1) {
				splitedGroups.push_back(subGroup);
			}
			continue;
		}

		if (voxels[i].second != voxels[i - 1].second) {
			splitedGroups.push_back(subGroup);
			subGroup.voxelPos.clear();
			subGroup.voxelPos.push_back(voxels[i].first);
			subGroup.opeType = voxels[i].second;
			subGroup.rangeLimitLow = voxels[i].first;
			subGroup.rangeLimitHigh = voxels[i].first;
		}
		else {
			subGroup.voxelPos.push_back(voxels[i].first);
			updateGroupRange(subGroup, voxels[i].first);
		}

		if (i == voxels.size() - 1) {
			splitedGroups.push_back(subGroup);
		}
	}

	return splitedGroups;
}


std::vector<Eigen::Vector3i> postProcess::getNeighbors(const Eigen::Vector3i& voxel) {
	return {
		voxel + Eigen::Vector3i(1, 0, 0),
		voxel + Eigen::Vector3i(-1, 0, 0),
		voxel + Eigen::Vector3i(0, 1, 0),
		voxel + Eigen::Vector3i(0, -1, 0),
		voxel + Eigen::Vector3i(0, 0, 1),
		voxel + Eigen::Vector3i(0, 0, -1),
		//voxel + Eigen::Vector3i(1, 1, 0),
		//voxel + Eigen::Vector3i(1, -1, 0),
		//voxel + Eigen::Vector3i(-1, 1, 0),
		//voxel + Eigen::Vector3i(-1, -1, 0),
		//voxel + Eigen::Vector3i(1, 0, 1),
		//voxel + Eigen::Vector3i(1, 0, -1),
		//voxel + Eigen::Vector3i(-1, 0, 1),
		//voxel + Eigen::Vector3i(-1, 0, -1),
		//voxel + Eigen::Vector3i(0, 1, 1),
		//voxel + Eigen::Vector3i(0, 1, -1),
		//voxel + Eigen::Vector3i(0, -1, 1),
		//voxel + Eigen::Vector3i(0, -1, -1)
	};
}



int postProcess::getIntersect(int idx1, int idx2) {
	if (idx1 < 0 || idx2 < 0) return 0;

	int intersect = 0;

	for (int i = 0; i < 5; i++) {
		if (idx1 % 10 == idx2 % 10) {
			intersect += (idx1 % 10) * pow(10, i);
		}
		idx1 /= 10;
		idx2 /= 10;
	}

	return intersect;
}

bool postProcess::isConnect(opeGroup group, Eigen::Vector3i pos) {
	if (group.voxelPos.size() == 0) return true;

	if (pos(0) < group.rangeLimitLow(0) - 1 || pos(0) > group.rangeLimitHigh(0) + 1 ||
		pos(1) < group.rangeLimitLow(1) - 1 || pos(1) > group.rangeLimitHigh(1) + 1 ||
		pos(2) < group.rangeLimitLow(2) - 1 || pos(2) > group.rangeLimitHigh(2) + 1) {
		return false;
	}

	bool flag = false;
	for (int i = 0; i < group.voxelPos.size(); i++) {
		Eigen::Vector3i diff = (group.voxelPos[i] - pos).cwiseAbs();
		if (diff.maxCoeff() == 0) {
			std::cout << "Error: two voxel positions are the same!" << std::endl;
		}
		else if (diff.maxCoeff() > 1) {
			continue;
		}
		else if (diff.sum() == 1 || diff.sum() == 2) {
			flag = true;
			break;
		}
	}

	return flag;
}

void postProcess::updateGroupRange(opeGroup& group, Eigen::Vector3i pos) {
	if (pos(0) < group.rangeLimitLow(0)) group.rangeLimitLow(0) = pos(0);
	if (pos(0) > group.rangeLimitHigh(0)) group.rangeLimitHigh(0) = pos(0);
	if (pos(1) < group.rangeLimitLow(1)) group.rangeLimitLow(1) = pos(1);
	if (pos(1) > group.rangeLimitHigh(1)) group.rangeLimitHigh(1) = pos(1);
	if (pos(2) < group.rangeLimitLow(2)) group.rangeLimitLow(2) = pos(2);
	if (pos(2) > group.rangeLimitHigh(2)) group.rangeLimitHigh(2) = pos(2);
}

std::vector<opeGroup> postProcess::splitSMGroup(opeGroup group) {

	std::string opeTypeStr = std::to_string(group.opeType);
	for (size_t i = 1; i < opeTypeStr.size(); i++) {
		opeTypeStr[i] = '0';
	}
	//bool findOne = false;
	//for (int i = opeTypeStr.size() - 1; i >= 0; i--) {
	//	if (findOne) {
	//		opeTypeStr[i] = '0';
	//	}

	//	else if (opeTypeStr[i] == '1') {
	//		findOne = true;
	//	}
	//}
	//if (!findOne) {
	//	std::cout << "Error: opeType is out of range!" << std::endl;
	//}
	group.opeType = std::stoi(opeTypeStr);

	std::vector<opeGroup> splitedGroups;
	opeGroup subGroup;
	subGroup.opeType = group.opeType;

	if (group.voxelPos.size() == 1) {
		subGroup.voxelPos.push_back(group.voxelPos[0]);
		subGroup.rangeLimitLow = group.voxelPos[0];
		subGroup.rangeLimitHigh = group.voxelPos[0];
		splitedGroups.push_back(subGroup);
		return splitedGroups;
	}

	std::vector<Eigen::Vector3i> filteredVoxelPos;
	switch (group.opeType) {
	case 10000:
		std::sort(group.voxelPos.begin(), group.voxelPos.end(), [](const Eigen::Vector3i& a, const Eigen::Vector3i& b) {
			return a(2) > b(2);
			});

		if (!this->justForShowing) {
			filteredVoxelPos.clear();
			for (size_t i = 0; i < group.voxelPos.size(); ++i) {
				const auto& a = group.voxelPos[i];
				if (a(2) < group.voxelPos[0](2)) {
					filteredVoxelPos.push_back(a);
					continue;
				}
				bool shouldDelete = false;
				for (size_t j = i + 1; j < group.voxelPos.size(); ++j) {
					const auto& b = group.voxelPos[j];
					if (a(0) == b(0) && a(1) == b(1) && a(2) > b(2)) {
						shouldDelete = true;
						break;
					}
				}
				if (!shouldDelete) {
					filteredVoxelPos.push_back(a);
				}
			}
			group.voxelPos = filteredVoxelPos;
		}

		break;
	case 1000:
		std::sort(group.voxelPos.begin(), group.voxelPos.end(), [](const Eigen::Vector3i& a, const Eigen::Vector3i& b) {
			return a(0) < b(0);
			});
		
		if (!this->justForShowing) {
			filteredVoxelPos.clear();
			for (size_t i = 0; i < group.voxelPos.size(); ++i) {
				const auto& a = group.voxelPos[i];
				if (a(0) > group.voxelPos[0](0)) {
					filteredVoxelPos.push_back(a);
					continue;
				}
				bool shouldDelete = false;
				for (size_t j = i + 1; j < group.voxelPos.size(); ++j) {
					const auto& b = group.voxelPos[j];
					if (a(0) < b(0) && a(1) == b(1) && a(2) == b(2)) {
						shouldDelete = true;
						break;
					}
				}
				if (!shouldDelete) {
					filteredVoxelPos.push_back(a);
				}
			}
			group.voxelPos = filteredVoxelPos;
		}

		break;
	case 100:
		std::sort(group.voxelPos.begin(), group.voxelPos.end(), [](const Eigen::Vector3i& a, const Eigen::Vector3i& b) {
			return a(0) > b(0);
			});

		if (!this->justForShowing) {
			filteredVoxelPos.clear();
			for (size_t i = 0; i < group.voxelPos.size(); ++i) {
				const auto& a = group.voxelPos[i];
				if (a(0) < group.voxelPos[0](0)) {
					filteredVoxelPos.push_back(a);
					continue;
				}
				bool shouldDelete = false;
				for (size_t j = i + 1; j < group.voxelPos.size(); ++j) {
					const auto& b = group.voxelPos[j];
					if (a(0) > b(0) && a(1) == b(1) && a(2) == b(2)) {
						shouldDelete = true;
						break;
					}
				}
				if (!shouldDelete) {
					filteredVoxelPos.push_back(a);
				}
			}
			group.voxelPos = filteredVoxelPos;
		}

		break;
	case 10:
		std::sort(group.voxelPos.begin(), group.voxelPos.end(), [](const Eigen::Vector3i& a, const Eigen::Vector3i& b) {
			return a(1) < b(1);
			});

		if (!this->justForShowing) {
			filteredVoxelPos.clear();
			for (size_t i = 0; i < group.voxelPos.size(); ++i) {
				const auto& a = group.voxelPos[i];
				if (a(1) > group.voxelPos[0](1)) {
					filteredVoxelPos.push_back(a);
					continue;
				}
				bool shouldDelete = false;
				for (size_t j = i + 1; j < group.voxelPos.size(); ++j) {
					const auto& b = group.voxelPos[j];
					if (a(1) < b(1) && a(0) == b(0) && a(2) == b(2)) {
						shouldDelete = true;
						break;
					}
				}
				if (!shouldDelete) {
					filteredVoxelPos.push_back(a);
				}
			}
			group.voxelPos = filteredVoxelPos;
		}

		break;
	case 1:
		std::sort(group.voxelPos.begin(), group.voxelPos.end(), [](const Eigen::Vector3i& a, const Eigen::Vector3i& b) {
			return a(1) > b(1);
			});

		if (!this->justForShowing) {
			filteredVoxelPos.clear();
			for (size_t i = 0; i < group.voxelPos.size(); ++i) {
				const auto& a = group.voxelPos[i];
				if (a(1) < group.voxelPos[0](1)) {
					filteredVoxelPos.push_back(a);
					continue;
				}
				bool shouldDelete = false;
				for (size_t j = i + 1; j < group.voxelPos.size(); ++j) {
					const auto& b = group.voxelPos[j];
					if (a(1) > b(1) && a(0) == b(0) && a(2) == b(2)) {
						shouldDelete = true;
						break;
					}
				}
				if (!shouldDelete) {
					filteredVoxelPos.push_back(a);
				}
			}
			group.voxelPos = filteredVoxelPos;
		}

		break;
	default:
		std::cout << "Error: opeType is out of range!" << std::endl;
		break;
	}


	for (int i = 0; i < group.voxelPos.size(); i++) {
		if (i == 0) {
			subGroup.voxelPos.push_back(group.voxelPos[i]);
			subGroup.rangeLimitLow = group.voxelPos[i];
			subGroup.rangeLimitHigh = group.voxelPos[i];
			continue;
		}
		if (!inSameLayer(subGroup, group.voxelPos[i], group.opeType)) {	
			std::vector<opeGroup> connectedGroups = splitGroupIntoConnected(subGroup);
			splitedGroups.insert(splitedGroups.end(), connectedGroups.begin(), connectedGroups.end());
			subGroup.voxelPos.clear();
			subGroup.voxelPos.push_back(group.voxelPos[i]);
			subGroup.rangeLimitLow = group.voxelPos[i];
			subGroup.rangeLimitHigh = group.voxelPos[i];
		}
		else {
			subGroup.voxelPos.push_back(group.voxelPos[i]);
			updateGroupRange(subGroup, group.voxelPos[i]);
		}
		if (i == group.voxelPos.size() - 1) {
			std::vector<opeGroup> connectedGroups = splitGroupIntoConnected(subGroup);
			splitedGroups.insert(splitedGroups.end(), connectedGroups.begin(), connectedGroups.end());
		}
	}

	return splitedGroups;
}

bool postProcess::inSameLayer(opeGroup& group, Eigen::Vector3i pos, int opeType) {
	int groupCase, posCase;
	switch (opeType) {
	case 100000:
		groupCase = group.rangeLimitLow(2);
		posCase = pos(2);
		break;
	case 10000:
		groupCase = group.rangeLimitHigh(2);
		posCase = pos(2);
		break;
	case 1000:
		groupCase = group.rangeLimitLow(0);
		posCase = pos(0);
		break;
	case 100:
		groupCase = group.rangeLimitHigh(0);
		posCase = pos(0);
		break;
	case 10:
		groupCase = group.rangeLimitLow(1);
		posCase = pos(1);
		break;
	case 1:
		groupCase = group.rangeLimitHigh(1);
		posCase = pos(1);
		break;
	default:
		std::cout << opeType << std::endl;
		std::cout << "Error: opeType is out of range!" << std::endl;
		break;
	}

	if (groupCase == posCase) return true;
	else return false;

}

std::vector<pathGroup> postProcess::voxelGroup2PathGroup(const std::vector<opeGroup>& groupSet) {
	std::vector<pathGroup> pathGroupSet;
	fileIO* IO_operator = new fileIO();

	Eigen::VectorXi modelOld = Eigen::VectorXi::Zero(this->modelObj->nx * this->modelObj->ny * this->modelObj->nz);
	Eigen::VectorXi modelNew = modelOld;

	std::cout << "Total number of groups: " << groupSet.size() << std::endl;

	int maxHeight = 0;
	for (int i = 0; i < groupSet.size(); i++) {
		modelOld = modelNew;
		opeGroup voxelGroup = groupSet[i];

		if (voxelGroup.opeType == 0) {
			std::cout << "Error: opeType out of range:0" << std::endl;
		}

		for (int j = 0; j < voxelGroup.voxelPos.size(); j++) {
			Eigen::Vector3i pos = voxelGroup.voxelPos[j];
			int idx = this->modelObj->coordinateToIndex(pos);
			modelNew(idx) = 1 - modelNew(idx);
			if (modelNew(idx) == 1) {
				maxHeight = std::max(maxHeight, pos(2));
			}
		}

		twoDimGroup voxelGroup2D = three2twoDimGroup(voxelGroup, modelOld);
		pathGroup2D pathGroup2D = voxelGroup2D2PathGroup2D(voxelGroup2D);
		pathGroup pathGroup = twoDimPath2ThreeDimPath(pathGroup2D, voxelGroup, maxHeight);
		pathGroupSet.push_back(pathGroup);

		IO_operator->writeVector(modelOld, "../DataSet/3DPathResult/" + std::to_string(i) + "_model.txt");
		IO_operator->writePathGroup(pathGroup.path, voxelGroup.opeType, "../DataSet/3DPathResult/" + std::to_string(i) + "_path.txt");

		std::cout << ".";
	}

	std::cout << std::endl;

	delete IO_operator;

	std::cout << "get path" << std::endl;
	return pathGroupSet;
}

twoDimGroup postProcess::three2twoDimGroup(const opeGroup& groupSet, const Eigen::VectorXi& model) {
	twoDimGroup voxelGroup2D;
	int idx1, idx2;
	switch (groupSet.opeType) {
	case 100000:
		idx1 = 0;
		idx2 = 1;
		break;
	case 10000:
		idx1 = 0;
		idx2 = 1;
		break;
	case 1000:
		idx1 = 1;
		idx2 = 2;
		break;
	case 100:
		idx1 = 1;
		idx2 = 2;
		break;
	case 10:
		idx1 = 0;
		idx2 = 2;
		break;
	case 1:
		idx1 = 0;
		idx2 = 2;
		break;
	default:
		std::cout << "Error: opeType is out of range!" << std::endl;
		break;
	}

	voxelGroup2D.rangeLimitLow(0) = groupSet.rangeLimitLow(idx1);
	voxelGroup2D.rangeLimitLow(1) = groupSet.rangeLimitLow(idx2);
	voxelGroup2D.rangeLimitHigh(0) = groupSet.rangeLimitHigh(idx1);
	voxelGroup2D.rangeLimitHigh(1) = groupSet.rangeLimitHigh(idx2);
	voxelGroup2D.opeType = groupSet.opeType;

	for (int i = 0; i < groupSet.voxelPos.size(); i++) {
		Eigen::Vector2i pos2D;
		pos2D(0) = groupSet.voxelPos[i](idx1);
		pos2D(1) = groupSet.voxelPos[i](idx2);
		std::pair<Eigen::Vector2i, int> voxel(pos2D, false);
		if (groupSet.opeType == 100000) {
			if (groupSet.voxelPos[i](2) == 0) {
				voxel.second = true;
			}
			else {
				Eigen::Vector3i underPos = groupSet.voxelPos[i];
				underPos(2) -= 1;
				int idx = this->modelObj->coordinateToIndex(underPos);
				if (model(idx) == 1) {
					voxel.second = true;
				}
			}
		}
		else {
			voxel.second = true;
		}
		voxelGroup2D.voxels.push_back(voxel);
	}

	return voxelGroup2D;
}


pathGroup postProcess::twoDimPath2ThreeDimPath(const pathGroup2D& pGroup2D, const opeGroup& vGroup, int maxHeight) {
	int idx1, idx2, idx3;
	bool lowToHigh;
	double mergeThreshold = cutterD + 0.01;
	double layerHeight = layerHeightSM;

	int fixVoxelIdx;
	double beginHeight;
	double endHeight;
	switch (pGroup2D.opeType) {
	case 100000:
		idx1 = 0;
		idx2 = 1;
		idx3 = 2;
		fixVoxelIdx = vGroup.rangeLimitLow(idx3);
		lowToHigh = true;
		mergeThreshold = nozzelD + 0.01;
		layerHeight = layerHeightAM;
		beginHeight = fixVoxelIdx * voxelSize + layerHeight;
		endHeight = (fixVoxelIdx + 1) * voxelSize;
		break;
	case 10000:
		idx1 = 0;
		idx2 = 1;
		idx3 = 2;
		fixVoxelIdx = vGroup.rangeLimitLow(idx3);

		if (vGroup.rangeLimitHigh(idx3) != fixVoxelIdx) {
			std::cout << "Error: rangeLimitHigh is not equal to rangeLimitLow!" << std::endl;
		}

		lowToHigh = false;
		//beginHeight = (maxHeight + 1) * voxelSize - layerHeight;
		beginHeight = (fixVoxelIdx + 1) * voxelSize;
		endHeight = fixVoxelIdx * voxelSize;
		break;
	case 1000:
		idx1 = 1;
		idx2 = 2;
		idx3 = 0;
		fixVoxelIdx = vGroup.rangeLimitLow(idx3);

		if (vGroup.rangeLimitHigh(idx3) != fixVoxelIdx) {
			std::cout << "Error: rangeLimitHigh is not equal to rangeLimitLow!" << std::endl;
		}

		lowToHigh = true;
		//beginHeight = 0;
		beginHeight = fixVoxelIdx * voxelSize;
		endHeight = (fixVoxelIdx + 1) * voxelSize;
		break;
	case 100:
		idx1 = 1;
		idx2 = 2;
		idx3 = 0;
		fixVoxelIdx = vGroup.rangeLimitLow(idx3);

		if (vGroup.rangeLimitHigh(idx3) != fixVoxelIdx) {
			std::cout << "Error: rangeLimitHigh is not equal to rangeLimitLow!" << std::endl;
		}

		lowToHigh = false;
		//beginHeight = this->modelObj->nx * voxelSize;
		beginHeight = (fixVoxelIdx + 1) * voxelSize;
		endHeight = fixVoxelIdx * voxelSize;
		break;
	case 10:
		idx1 = 0;
		idx2 = 2;
		idx3 = 1;
		fixVoxelIdx = vGroup.rangeLimitLow(idx3);

		if (vGroup.rangeLimitHigh(idx3) != fixVoxelIdx) {
			std::cout << "Error: rangeLimitHigh is not equal to rangeLimitLow!" << std::endl;
		}

		lowToHigh = true;
		//beginHeight = 0;
		beginHeight = fixVoxelIdx * voxelSize;
		endHeight = (fixVoxelIdx + 1) * voxelSize;
		break;
	case 1:
		idx1 = 0;
		idx2 = 2;
		idx3 = 1;
		fixVoxelIdx = vGroup.rangeLimitLow(idx3);

		if (vGroup.rangeLimitHigh(idx3) != fixVoxelIdx) {
			std::cout << "Error: rangeLimitHigh is not equal to rangeLimitLow!" << std::endl;
		}

		lowToHigh = false;
		//beginHeight = this->modelObj->ny * voxelSize;
		beginHeight = (fixVoxelIdx + 1) * voxelSize;
		endHeight = fixVoxelIdx * voxelSize;
		break;
	default:
		std::cout << pGroup2D.opeType << std::endl;
		std::cout << "Error: opeType is out of range!" << std::endl;
		break;
	}

	pathGroup pGroup;
	pGroup.opeType = pGroup2D.opeType;

	std::vector<double> heightList;
	if (beginHeight < endHeight) {
		for (double height = beginHeight; height < endHeight+0.01; height += layerHeight) {
			heightList.push_back(height);
		}
	}
	else {
		for (double height = beginHeight; height > endHeight-0.01; height -= layerHeight) {
			heightList.push_back(height);
		}
	}

	int usePath1 = 1;
	for (int i = 0; i < heightList.size(); i++) {
		double height = heightList[i];
		std::vector < Eigen::MatrixXd> path2D = pGroup2D.path2;
		if (usePath1 == 1) {
			path2D = pGroup2D.path1;
		}

		for (int i = 0; i < path2D.size(); i++) {
			int sizePath2D = path2D[i].rows();
			Eigen::MatrixXd path = Eigen::MatrixXd::Zero(sizePath2D, 3);
			path.col(idx1) = path2D[i].col(0);
			path.col(idx2) = path2D[i].col(1);
			path.col(idx3) = Eigen::VectorXd::Constant(sizePath2D, height);

			pGroup.path.push_back(path);
		}

		usePath1 = 1 - usePath1;
	}

	std::vector<Eigen::MatrixXd> mergedPath;
	mergedPath.push_back(pGroup.path[0]);
	for (int i = 1; i < pGroup.path.size(); i++) {
		Eigen::MatrixXd& last = mergedPath.back();
		Eigen::RowVectorXd lastRow = last.row(last.rows() - 1);
		Eigen::RowVectorXd firstRow = pGroup.path[i].row(0);
		if (canMerge(lastRow, firstRow, mergeThreshold,idx3)) {
			Eigen::MatrixXd merged(last.rows() + pGroup.path[i].rows(), last.cols());
			merged << last, pGroup.path[i];
			last = merged;
			//std::cout << mergedPath.back().size() << std::endl;
		}
		else {
			mergedPath.push_back(pGroup.path[i]);
		}
	}

	pGroup.path = mergedPath;

	return pGroup;
}


bool postProcess::canMerge(const Eigen::RowVectorXd& row1, const Eigen::RowVectorXd& row2, double threshold, int idx3) {
	//int diffCount = 0;

	for (int i = 0; i < row1.size(); ++i) {
		double diff = std::abs(row1[i] - row2[i]);
		//if (diff > 1e-5) {
		//	if (diff > threshold) {
		//		return false; 
		//	}
		//	diffCount++;
		//}

		if (diff>threshold){
			//double diffTmp = std::abs(row1[idx3] - row2[idx3]);
			//if (diffTmp > 1e-5) {
			//	std::cout << "error: " << row1 << " " << row2 << std::endl;
			//}
			return false;
		}
	}

	return true;
	//return diffCount == 1; 
}


pathGroup2D postProcess::voxelGroup2D2PathGroup2D(twoDimGroup group2D) {
	
	int nVoxel = group2D.voxels.size();
	std::vector<Eigen::Vector4i> distToBoundary(nVoxel, -1 * Eigen::Vector4i::Ones());
	// to coordinate 1 +, to coordinate 1 -, to coordinate 2 +, to coordinate 2 -

	double pathWidth;
	if (group2D.opeType == 100000) {
		pathWidth = nozzelD;
	}
	else {
		pathWidth = cutterD;
	}

	//std::vector<subVoxel> subVoxelSet;
	//subVoxelSet.reserve(4 * nVoxel);
	/*for (int i = 0; i < nVoxel; i++) {
		for (int j = 0; j < 4; j++) {
			getDistToBoundary(distToBoundary, group2D, i, j);
		}

		Eigen::Vector2d position = group2D.voxels[i].first.cast<double>();
		position = position * voxelSize;
		position(0) += pathWidth / 2;
		position(1) += pathWidth / 2;
		double dist = distToBoundary[i](0) * voxelSize + voxelSize - pathWidth / 2;
		dist = std::min(dist, distToBoundary[i](1) * voxelSize + pathWidth / 2);
		dist = std::min(dist, distToBoundary[i](2) * voxelSize + voxelSize - pathWidth / 2);
		dist = std::min(dist, distToBoundary[i](3) * voxelSize + pathWidth / 2);
		subVoxel subVoxel1;
		subVoxel1.hasSupport = group2D.voxels[i].second;
		subVoxel1.position = position;
		subVoxel1.distToBoundary = dist;
		subVoxelSet.push_back(subVoxel1);

		position = group2D.voxels[i].first.cast<double>();
		position(0) += 1;
		position = position * voxelSize;
		position(0) -= pathWidth / 2;
		position(1) += pathWidth / 2;
		dist = distToBoundary[i](0) * voxelSize + pathWidth / 2;
		dist = std::min(dist, distToBoundary[i](1) * voxelSize + voxelSize - pathWidth / 2);
		dist = std::min(dist, distToBoundary[i](2) * voxelSize + voxelSize - pathWidth / 2);
		dist = std::min(dist, distToBoundary[i](3) * voxelSize + pathWidth / 2);
		subVoxel subVoxel2;
		subVoxel2.hasSupport = group2D.voxels[i].second;
		subVoxel2.position = position;
		subVoxel2.distToBoundary = dist;
		subVoxelSet.push_back(subVoxel2);

		position = group2D.voxels[i].first.cast<double>();
		position(1) += 1;
		position = position * voxelSize;
		position(0) += pathWidth / 2;
		position(1) -= pathWidth / 2;
		dist = distToBoundary[i](0) * voxelSize + voxelSize - pathWidth / 2;
		dist = std::min(dist, distToBoundary[i](1) * voxelSize + pathWidth / 2);
		dist = std::min(dist, distToBoundary[i](2) * voxelSize + pathWidth / 2);
		dist = std::min(dist, distToBoundary[i](3) * voxelSize + voxelSize - pathWidth / 2);
		subVoxel subVoxel3;
		subVoxel3.hasSupport = group2D.voxels[i].second;
		subVoxel3.position = position;
		subVoxel3.distToBoundary = dist;
		subVoxelSet.push_back(subVoxel3);

		position = group2D.voxels[i].first.cast<double>();
		position(0) += 1;
		position(1) += 1;
		position = position * voxelSize;
		position(0) -= pathWidth / 2;
		position(1) -= pathWidth / 2;
		dist = distToBoundary[i](0) * voxelSize + pathWidth / 2;
		dist = std::min(dist, distToBoundary[i](1) * voxelSize + voxelSize - pathWidth / 2);
		dist = std::min(dist, distToBoundary[i](2) * voxelSize + pathWidth / 2);
		dist = std::min(dist, distToBoundary[i](3) * voxelSize + voxelSize - pathWidth / 2);
		subVoxel subVoxel4;
		subVoxel4.hasSupport = group2D.voxels[i].second;
		subVoxel4.position = position;
		subVoxel4.distToBoundary = dist;
		subVoxelSet.push_back(subVoxel4);
	}*/

	std::vector<subVoxel> subVoxelSet = getSubVoxelAndDist(group2D, pathWidth);

	//sort subVoxelSet according to distToBoundary
	std::sort(subVoxelSet.begin(), subVoxelSet.end(), [](const subVoxel& a, const subVoxel& b) {
		if (a.distToBoundary == b.distToBoundary) {
			if (a.position(0) == b.position(0)) {
				return a.position(1) < b.position(1);
			}
			else {
				return a.position(0) < b.position(0);
			}
		}
		else {
			return a.distToBoundary < b.distToBoundary;
		}
		});

	std::vector<std::vector<subVoxel>> pathSet;
	std::vector<subVoxel> contour;
	int nextIdx = 0;
	subVoxel startSubVoxel = subVoxelSet[nextIdx];
	//std::cout << startSubVoxel.position << std::endl;
	Eigen::Vector2d refDirection;
	refDirection << 0, 1;
	contour.push_back(subVoxelSet[nextIdx]);
	subVoxelSet.erase(subVoxelSet.begin() + nextIdx);
	while (subVoxelSet.size() > 0) {
		
		nextIdx = -1;
		double cross = -1000;

		// check neighbor
		for (int i = 0; i < subVoxelSet.size(); i++) {
			if (subVoxelSet[i].distToBoundary != contour.back().distToBoundary) {
				break;
			}
			if (neighborSubVoxel(contour.back(), subVoxelSet[i], pathWidth+0.01)) {
				Eigen::Vector2d direction = subVoxelSet[i].position - contour.back().position;
				double crossTmp = refDirection(0) * direction(1) - refDirection(1) * direction(0);
				if (crossTmp > cross) {
					cross = crossTmp;
					nextIdx = i;
				}
				if (cross > 0.0001) {
					break;
				}
			}
		}

		if (nextIdx >= 0) {
			// find neighbor
			//std::cout << subVoxelSet[nextIdx].position << std::endl;
			refDirection = subVoxelSet[nextIdx].position - contour.back().position;
			contour.push_back(subVoxelSet[nextIdx]);
			subVoxelSet.erase(subVoxelSet.begin() + nextIdx);
			if (subVoxelSet.size() == 0) {
				if (neighborSubVoxel(contour.back(), startSubVoxel, pathWidth + 0.01)) {
					//contour.push_back(startSubVoxel);
					//std::cout << "closed" << std::endl;
				}
				else {
					//std::cout << contour.back().position << std::endl;
					//std::cout << startSubVoxel.position << std::endl;
					std::cout << "not closed" << std::endl;
				}
				pathSet.push_back(contour);
				contour.clear();
			}
		}
		else {
			// no neighbor with same distToBoundary

			// check if the countour is closed
			if (neighborSubVoxel(contour.back(), startSubVoxel, pathWidth + 0.01)) {
				/*refDirection = startSubVoxel.position - contour.back().position;
				contour.push_back(startSubVoxel);*/
				//std::cout << "closed" << std::endl;
			}
			else {
				//std::cout << contour.back().position << std::endl;
				//std::cout << startSubVoxel.position << std::endl;
				std::cout << "not closed" << std::endl;
			}

			// check if the last subVoxel has a neighbor in subVoxelSet
			for (int i = 0; i < subVoxelSet.size(); i++) {
				if (subVoxelSet[i].distToBoundary != subVoxelSet[0].distToBoundary) {
					break;
				}
				if (neighborSubVoxel(contour.back(), subVoxelSet[i], pathWidth + 0.01)) {
					Eigen::Vector2d direction = subVoxelSet[i].position - contour.back().position;
					double crossTmp = refDirection(0) * direction(1) - refDirection(1) * direction(0);
					if (crossTmp > cross) {
						cross = crossTmp;
						nextIdx = i;
					}
					if (cross > 0.0001) {
						break;
					}
				}
			}

			if (nextIdx >= 0) {
				// find neighbor
				startSubVoxel = subVoxelSet[nextIdx];
				//std::cout << startSubVoxel.position << std::endl;
				refDirection = subVoxelSet[nextIdx].position - contour.back().position;
				contour.push_back(subVoxelSet[nextIdx]);
				subVoxelSet.erase(subVoxelSet.begin() + nextIdx);
				if (subVoxelSet.size() == 0) {
					pathSet.push_back(contour);
					contour.clear();
				}
			}
			else {
				// no neighbor
				pathSet.push_back(contour);
				
				// find the start of the next contour
				nextIdx = 0;
				//bool hasSupportTmp = subVoxelSet[0].hasSupport;
				//double dist = (subVoxelSet[0].position - contour.back().position).norm();
				//for (int i = 1; i < subVoxelSet.size(); i++) {
				//	if (subVoxelSet[i].distToBoundary != subVoxelSet[0].distToBoundary) {
				//		break;
				//	}

				//	if (!subVoxelSet[i].hasSupport) {
				//		if (hasSupportTmp) {
				//			continue;
				//		}
				//	}
				//	else if (!hasSupportTmp) {
				//		hasSupportTmp = true;
				//		nextIdx = i;
				//		dist = (subVoxelSet[i].position - contour.back().position).norm();
				//		continue;
				//	}

				//	double distTmp = (subVoxelSet[i].position - contour.back().position).norm();
				//	if (distTmp < dist) {
				//		nextIdx = i;
				//		dist = distTmp;
				//	}
				//}
				startSubVoxel = subVoxelSet[nextIdx];
				refDirection = Eigen::Vector2d(0, 1);
				contour.clear();
				//std::cout << startSubVoxel.position << std::endl;
				contour.push_back(subVoxelSet[nextIdx]);
				subVoxelSet.erase(subVoxelSet.begin() + nextIdx);
				if (subVoxelSet.size() == 0) {
					pathSet.push_back(contour);
					contour.clear();
				}
			}
		}
	}

	// change startPosition of each contour
	for (int i = 0; i < pathSet.size(); i++) {
		if (pathSet[i][0].hasSupport) {
			continue;
		}
		for (int j = 1; j < pathSet[i].size(); j++) {
			if (pathSet[i][j].hasSupport) {
				std::rotate(pathSet[i].begin(), pathSet[i].begin() + j, pathSet[i].end());
				break;
			}
		}
	}


	pathGroup2D pathGroup2DObj;
	pathGroup2DObj.opeType = group2D.opeType;
	pathGroup2DObj.path1.resize(pathSet.size());
	for (int i = 0; i < pathSet.size(); i++) {
		Eigen::MatrixXd path = Eigen::MatrixXd::Zero(pathSet[i].size(), 2);
		for (int j = 0; j < pathSet[i].size(); j++) {
			path.row(j) = pathSet[i][j].position;
		}
		pathGroup2DObj.path1[i] = path;
	}
	pathGroup2DObj.path2 = pathGroup2DObj.path1;

	return pathGroup2DObj;
}

std::vector<subVoxel> postProcess::getSubVoxelAndDist(const twoDimGroup& group2D, double pathWidth) {
	int nVoxel = group2D.voxels.size();

	std::vector<Eigen::Vector4i> nodeIdx(nVoxel);
	std::vector<node2D> nodes;

	// create nodes
	for (int i = 0; i < nVoxel; i++) {
		std::vector<Eigen::Vector2i> nodeCoords;
		nodeCoords.push_back(group2D.voxels[i].first);
		nodeCoords.push_back(group2D.voxels[i].first + Eigen::Vector2i(1, 0));
		nodeCoords.push_back(group2D.voxels[i].first + Eigen::Vector2i(0, 1));
		nodeCoords.push_back(group2D.voxels[i].first + Eigen::Vector2i(1, 1));
		Eigen::Vector4i nodeIdxTmp;
		for (int j = 0; j < 4; j++) {
			Eigen::Vector2i nodeCoord = nodeCoords[j];
			auto it = std::find_if(nodes.begin(), nodes.end(), [nodeCoord](const node2D& node) {
				return node.coord == nodeCoord;
				});
			if (it == nodes.end()) {
				node2D node;
				node.coord = nodeCoord;
				node.idx = nodes.size();
				node.distToBoundary = 1e5;
				node.voxelIdx.push_back(i);
				nodes.push_back(node);
				nodeIdxTmp(j) = node.idx;
			}
			else {
				it->voxelIdx.push_back(i);
				nodeIdxTmp(j) = std::distance(nodes.begin(), it);
			}

		}
		nodeIdx[i] = nodeIdxTmp;
	}

	// update distToBoundary
	std::vector<Eigen::Vector2i> neighborDire = {
		Eigen::Vector2i(1, 0),
		Eigen::Vector2i(-1, 0),
		Eigen::Vector2i(0, 1),
		Eigen::Vector2i(0, -1),
		Eigen::Vector2i(1, 1),
		Eigen::Vector2i(-1, 1),
		Eigen::Vector2i(1, -1),
		Eigen::Vector2i(-1, -1)
	};
	std::queue<int> nodeToExplore;
	for (int i = 0; i < nodes.size(); i++) {
		if (nodes[i].voxelIdx.size() < 4) {
			nodes[i].distToBoundary = 0;
			nodeToExplore.push(i);
		}
	}
	 while (!nodeToExplore.empty()) {
		int idx = nodeToExplore.front();
		nodeToExplore.pop();
		Eigen::Vector2i coord = nodes[idx].coord;
		for (int i = 0; i < 8; i++) {
			Eigen::Vector2i neighborCoord = coord + neighborDire[i];
			auto it = std::find_if(nodes.begin(), nodes.end(), [neighborCoord](const node2D& node) {
				return node.coord == neighborCoord;
				});
			if (it == nodes.end()) {
				continue;
			}
			int neighborIdx = std::distance(nodes.begin(), it);
			if (nodes[neighborIdx].distToBoundary < 1e5) {
				continue;
			}
			nodes[neighborIdx].distToBoundary = nodes[idx].distToBoundary + 1;
			nodeToExplore.push(neighborIdx);
		}
	}


	std::vector<subVoxel> subVoxelSet;
	subVoxelSet.reserve(4 * nVoxel);
	for (int i = 0; i < nVoxel; i++) {
		std::vector<subVoxel> subVoxels;

		Eigen::Vector2d position = group2D.voxels[i].first.cast<double>();
		position = position * voxelSize;
		position(0) += pathWidth / 2;
		position(1) += pathWidth / 2;
		subVoxel subVoxel1;
		subVoxel1.hasSupport = group2D.voxels[i].second;
		subVoxel1.position = position;
		subVoxels.push_back(subVoxel1);

		position = group2D.voxels[i].first.cast<double>();
		position(0) += 1;
		position = position * voxelSize;
		position(0) -= pathWidth / 2;
		position(1) += pathWidth / 2;
		subVoxel subVoxel2;
		subVoxel2.hasSupport = group2D.voxels[i].second;
		subVoxel2.position = position;
		subVoxels.push_back(subVoxel2);

		position = group2D.voxels[i].first.cast<double>();
		position(1) += 1;
		position = position * voxelSize;
		position(0) += pathWidth / 2;
		position(1) -= pathWidth / 2;
		subVoxel subVoxel3;
		subVoxel3.hasSupport = group2D.voxels[i].second;
		subVoxel3.position = position;
		subVoxels.push_back(subVoxel3);

		position = group2D.voxels[i].first.cast<double>();
		position(0) += 1;
		position(1) += 1;
		position = position * voxelSize;
		position(0) -= pathWidth / 2;
		position(1) -= pathWidth / 2;
		subVoxel subVoxel4;
		subVoxel4.hasSupport = group2D.voxels[i].second;
		subVoxel4.position = position;
		subVoxels.push_back(subVoxel4);

		for (int j = 0; j < 4; j++) {
			subVoxel subVoxelNow = subVoxels[j];
			subVoxelNow.distToBoundary = 1e5;

			for (int k = 0; k < 4; k++) {
				int nodeIdxTmp = nodeIdx[i](k);
				node2D node = nodes[nodeIdxTmp];
				double distNow = node.distToBoundary * voxelSize;
				if (k == j) {
					distNow = distNow + pathWidth / 2;
				}
				else {
					distNow = distNow + voxelSize - pathWidth / 2;
				}
				if (distNow < subVoxelNow.distToBoundary) {
					subVoxelNow.distToBoundary = distNow;
				}
			}

			subVoxelSet.push_back(subVoxelNow);
		}
	}

	return subVoxelSet;
}

bool postProcess::neighborSubVoxel(const subVoxel& subVoxel1, const subVoxel& subVoxel2, double threshold) {

	double dx = std::abs(subVoxel1.position(0) - subVoxel2.position(0));
	double dy = std::abs(subVoxel1.position(1) - subVoxel2.position(1));

	return (dx < 0.01 && dy < threshold) || (dx < threshold && dy < 0.01);
}


int postProcess::getDistToBoundary(std::vector<Eigen::Vector4i>& distToBoundary, const twoDimGroup& group2D, int idx, int direction) {
	if (distToBoundary[idx](direction) >= 0) {
		return distToBoundary[idx](direction);
	}
	
	Eigen::Vector2i nextVoxel = group2D.voxels[idx].first;
	Eigen::Vector2i nextV1, nextV2;

	switch (direction) {
	case 0:
		if (nextVoxel(0) == group2D.rangeLimitHigh(0)) {
			distToBoundary[idx](direction) = 0;
			return 0;
		}
		nextVoxel(0) += 1;
		nextV1 = nextVoxel;
		nextV1(1) -= 1;
		nextV2 = nextVoxel;
		nextV2(1) += 1;
		break;
	case 1:
		if (nextVoxel(0) == group2D.rangeLimitLow(0)) {
			distToBoundary[idx](direction) = 0;
			return 0;
		}
		nextVoxel(0) -= 1;
		nextV1 = nextVoxel;
		nextV1(1) -= 1;
		nextV2 = nextVoxel;
		nextV2(1) += 1;
		break;
	case 2:
		if (nextVoxel(1) == group2D.rangeLimitHigh(1)) {
			distToBoundary[idx](direction) = 0;
			return 0;
		}
		nextVoxel(1) += 1;
		nextV1 = nextVoxel;
		nextV1(0) -= 1;
		nextV2 = nextVoxel;
		nextV2(0) += 1;
		break;
	case 3:
		if (nextVoxel(1) == group2D.rangeLimitLow(1)) {
			distToBoundary[idx](direction) = 0;
			return 0;
		}
		nextVoxel(1) -= 1;
		nextV1 = nextVoxel;
		nextV1(0) -= 1;
		nextV2 = nextVoxel;
		nextV2(0) += 1;
		break;
	default:
		std::cout << direction << std::endl;
		std::cout << "Error: direction is out of range!" << std::endl;
		return -1;
	}

	//check whether nextVoxel is in group2D.path
	auto it = std::find_if(group2D.voxels.begin(), group2D.voxels.end(), [nextVoxel](const std::pair<Eigen::Vector2i, bool>& voxel) {
		return voxel.first == nextVoxel;
		});

	if (it == group2D.voxels.end()) {
		distToBoundary[idx](direction) = 0;
		return 0;
	}
	else {
		int nextIdx = std::distance(group2D.voxels.begin(), it);

		//it = std::find_if(group2D.voxels.begin(), group2D.voxels.end(), [nextV1](const std::pair<Eigen::Vector2i, bool>& voxel) {
		//	return voxel.first == nextV1;
		//	});
		//if (it == group2D.voxels.end()) {
		//	distToBoundary[idx](direction) = 0;
		//	return 0;
		//}
		//it = std::find_if(group2D.voxels.begin(), group2D.voxels.end(), [nextV2](const std::pair<Eigen::Vector2i, bool>& voxel) {
		//	return voxel.first == nextV2;
		//	});
		//if (it == group2D.voxels.end()) {
		//	distToBoundary[idx](direction) = 0;
		//	return 0;
		//}

		int distTmp = getDistToBoundary(distToBoundary, group2D, nextIdx, direction);
		distToBoundary[idx](direction) = distTmp + 1;
		return distToBoundary[idx](direction);
	}
}

int postProcess::getCommandLengthAndMoveToCenter(pathGroup& pGroup) {
	int segNum = pGroup.path.size();
	int length = 0;
	for (int i = 0; i < segNum; i++) {
		length += pGroup.path[i].rows();
		pGroup.path[i].col(0).array() += zeroInCenterFrame(0);
		pGroup.path[i].col(1).array() += zeroInCenterFrame(1);
	}
	return length;
}


void postProcess::pathGroup2CommandFile(std::vector<pathGroup>& pGroup, const std::string& modelName, bool splitDiffOpe) {
	fileIO* IO_operator = new fileIO();

	std::string folderPath = "C:/Users/j46868yc/OneDrive/5AxisMaker/" + modelName;
	IO_operator->createDirectory(folderPath);

	int lengthFile = 0;
	int fileIdx = 0;
	int lastPos = 0;
	std::string filePath = folderPath + "/" + std::to_string(fileIdx) + ".txt";
	IO_operator->startCommandFile(filePath);
	for (int i = 0; i < pGroup.size(); i++) {
		
		int lengthPGroup = getCommandLengthAndMoveToCenter(pGroup[i]);

		if (lengthPGroup > commandLengthLimit) {
			std::cout << "lengthPGroup: " << lengthPGroup << std::endl;
			std::cout << "Error: length of path group is out of range!" << std::endl;
			return;
		}

		bool newFile = (lengthFile + lengthPGroup > commandLengthLimit);
		if (splitDiffOpe) {
			newFile = newFile || (pGroup[i].opeType != lastPos && lastPos > 0);
		}

		if (newFile) {
			IO_operator->changeToolCommand(filePath, lastPos, -1, cmParas);
			IO_operator->endCommandFile(filePath, lastPos);
			//std::cout << "lengthFile: " << lengthFile << std::endl;
			fileIdx++;
			filePath = folderPath + "/" + std::to_string(fileIdx) + ".txt";
			IO_operator->startCommandFile(filePath);
			lengthFile = 0;
			lastPos = 0;
		}
		
		int currentPos = pGroup[i].opeType;
		if (lastPos != currentPos) {
			IO_operator->changeToolCommand(filePath, lastPos, currentPos, cmParas);
		}

		IO_operator->moveCommand(filePath, pGroup[i].path, currentPos, cmParas, nozzelD * layerHeightAM);


		lengthFile += lengthPGroup;
		lastPos = currentPos;

		if (i == pGroup.size() - 1) {
			IO_operator->changeToolCommand(filePath, lastPos, -1, cmParas);
			IO_operator->endCommandFile(filePath, lastPos);
			//std::cout << "lengthFile: " << lengthFile << std::endl;
		}
		std::cout << ".";
	}
	std::cout << std::endl;

	delete IO_operator;
	
	std::cout << "get command" << std::endl;
}

std::vector<opeGroup> postProcess::adjustGroupSet(std::vector<opeGroup> groupSet) {

	AccEro3D* accEroObj = new AccEro3D(this->modelObj);

	// adjust AM groups
	for (int i = 0; i < groupSet.size() - 1; i++) {
		std::cout << ".";

		if (groupSet[i].opeType < 100000) {
			continue;
		}
		if (groupSet[i].voxelPos.size() == 0) {
			continue;
		}
		int heightNow = groupSet[i].rangeLimitLow(2);

		int nextIdx = i + 1;
		bool hasSM = false;
		std::vector<std::pair<int,Eigen::Vector3i>>influenceVoxels;
		while (nextIdx < groupSet.size()) {
			if (groupSet[nextIdx].voxelPos.size() == 0) {
				nextIdx++;
				continue;
			}


			if (groupSet[nextIdx].opeType < 100000) {
				// SM group

				if (groupSet[nextIdx].opeType == 10000 && groupSet[nextIdx].rangeLimitLow(2) < heightNow - 1) {
					break; // influence the SM accessiblity
				}

				for (const auto& voxel : groupSet[nextIdx].voxelPos) {
					influenceVoxels.emplace_back(groupSet[nextIdx].opeType, voxel);
				}
				
				nextIdx++;
				hasSM = true;
				continue;
			}

			int heightTmp = groupSet[nextIdx].rangeLimitLow(2);
			if (!hasSM || heightTmp != heightNow) {
				for (const auto& voxel : groupSet[nextIdx].voxelPos) {
					influenceVoxels.emplace_back(groupSet[nextIdx].opeType, voxel);
				}
				nextIdx++;
				continue;
			}

			bool canChange = true;
			for (int j = 0; j < groupSet[nextIdx].voxelPos.size(); j++) {
				Eigen::Vector3i voxel = groupSet[nextIdx].voxelPos[j];
				for (int k = 0; k < influenceVoxels.size(); k++) {
					Eigen::Vector3i diff = voxel - influenceVoxels[k].second;
					if (diff.head<2>().array().abs().maxCoeff() == 0) {
						// same voxel or in the same xy vertical line
						canChange = false;
						break;
					}

					// check AM support of the changed voxel
					// and check the stability of the changed voxel
					if (influenceVoxels[k].second(2) == heightNow -1){
						if (diff.head<2>().array().abs().maxCoeff() <=1){
							canChange = false;
							break;
						}
					}

					if (influenceVoxels[k].first == 100000){
						if (influenceVoxels[k].second(2) < heightNow){
							// check AM accessiblity of influence voxel
							int boundaryTmp = accEroObj -> toolBoundaryAM(influenceVoxels[k].second, voxel.head<2>());
							if (boundaryTmp <= heightNow){
								canChange = false;
								break;
							}	
						}
					}
					else{
						// check AM accessibility of the changed voxel
						if (influenceVoxels[k].second(2)>voxel(2)){
							int boundaryTmp = accEroObj -> toolBoundaryAM(voxel, influenceVoxels[k].second.head<2>());
							if (boundaryTmp <= influenceVoxels[k].second(2)){
								canChange = false;
								break;
							}
						}

						// check SM accessibility of the influence voxel
						if (influenceVoxels[k].second(2) == voxel(2) &&
							influenceVoxels[k].first < 10000){

							switch (influenceVoxels[k].first){
								case 1000:
									if (voxel(0) == influenceVoxels[k].second(0)-1 &&
										voxel(1) == influenceVoxels[k].second(1)){
										canChange = false;
									}
									break;
								case 100:
									if (voxel(0) == influenceVoxels[k].second(0)+1 &&
										voxel(1) == influenceVoxels[k].second(1)){
										canChange = false;
									}
									break;
								case 10:
									if (voxel(1) == influenceVoxels[k].second(1)-1 &&
										voxel(0) == influenceVoxels[k].second(0)){
										canChange = false;
									}
									break;
								case 1:
									if (voxel(1) == influenceVoxels[k].second(1)+1 &&
										voxel(0) == influenceVoxels[k].second(0)){
										canChange = false;
									}
									break;
								default:
									break;
							}

							if (!canChange){
								break;
							}
						}
					}	
				}
				if (!canChange) {
					break;
				}
			}

			if (canChange) {
				for (int j = 0; j < groupSet[nextIdx].voxelPos.size(); j++) {
					groupSet[i].voxelPos.push_back(groupSet[nextIdx].voxelPos[j]);
					updateGroupRange(groupSet[i], groupSet[nextIdx].voxelPos[j]);
				}
				groupSet[nextIdx].voxelPos.clear();
			}
			else {
				for (int j = 0; j < groupSet[nextIdx].voxelPos.size(); j++) {
					influenceVoxels.emplace_back(groupSet[nextIdx].opeType, groupSet[nextIdx].voxelPos[j]);
				}
			}
			nextIdx++;
		}
	}

	// adjust SM groups
	for (int i = groupSet.size()-1; i >= 1; i--) {
		std::cout << ".";

		if (groupSet[i].opeType == 100000) {
			continue;
		}
		if (groupSet[i].voxelPos.size() == 0) {
			continue;
		}

		int typeNow = groupSet[i].opeType;
		int nextIdx = i - 1;
		bool hasAM = false;
		std::vector<std::pair<int,Eigen::Vector3i>>influenceVoxels;
		while (nextIdx >= 0) {
			//if (nextIdx == 1) {
			//	std::cout << std::endl;
			//	std::cout << "i: " << i << std::endl;
			//	for (int tmp = 0; tmp < groupSet[nextIdx].voxelPos.size(); tmp++) {
			//		std::cout << groupSet[nextIdx].voxelPos[tmp].transpose() << std::endl;
			//	}
			//	std::cout << std::endl;
			//	for (int tmp = 0; tmp < groupSet[nextIdx + 1].voxelPos.size(); tmp++) {
			//		std::cout << groupSet[nextIdx + 1].voxelPos[tmp].transpose() << std::endl;
			//	}
			//}

			if (groupSet[nextIdx].voxelPos.size() == 0) {
				nextIdx--;
				continue;
			}


			if (groupSet[nextIdx].opeType == 100000) {
				// AM group
				for (const auto& voxel : groupSet[nextIdx].voxelPos) {
					influenceVoxels.emplace_back(groupSet[nextIdx].opeType, voxel);
				}
				nextIdx--;
				hasAM = true;
				continue;
			}

			int typeTmp = groupSet[nextIdx].opeType;
			if (typeTmp != typeNow){
				for (const auto& voxel : groupSet[nextIdx].voxelPos) {
					influenceVoxels.emplace_back(groupSet[nextIdx].opeType, voxel);
				}
				nextIdx--;
				continue;
			}

			bool canChange = false;
			if (!hasAM){
				canChange = true;
			}

			if (!canChange){
				canChange = true;
				for (int j = 0; j < groupSet[nextIdx].voxelPos.size(); j++) {
					Eigen::Vector3i voxel = groupSet[nextIdx].voxelPos[j];
					
					for (int k = 0; k < influenceVoxels.size(); k++) {
						Eigen::Vector3i diff = voxel - influenceVoxels[k].second;
						if (diff.array().abs().maxCoeff() == 0) {
							// same voxel
							canChange = false;
							break;
						}

						if (influenceVoxels[k].first == 100000){
							if (influenceVoxels[k].second(2) < voxel(2)){
								// check AM accessiblity of influence voxel
								int boundaryTmp = accEroObj -> toolBoundaryAM(influenceVoxels[k].second, voxel.head<2>());
								if (boundaryTmp <= voxel(2)){
									canChange = false;
									break;
								}	
							}
							else{
								// check SM accessibility of the changed voxel
								switch (typeNow){
									case 10000:
										if (influenceVoxels[k].second(2)>voxel(2)+1 || 
											diff.head<2>().array().abs().maxCoeff() == 0){
											canChange = false;
										}
										break;
									case 1000:
										if (influenceVoxels[k].second(0)==voxel(0)-1 &&
											influenceVoxels[k].second(1)==voxel(1) &&
											influenceVoxels[k].second(2)==voxel(2)){
											canChange = false;
										}
										break;
									case 100:
										if (influenceVoxels[k].second(0)==voxel(0)+1 &&
											influenceVoxels[k].second(1)==voxel(1) &&
											influenceVoxels[k].second(2)==voxel(2)){
											canChange = false;
										}
										break;
									case 10:
										if (influenceVoxels[k].second(1)==voxel(1)-1 &&
											influenceVoxels[k].second(0)==voxel(0) &&
											influenceVoxels[k].second(2)==voxel(2)){
											canChange = false;
										}
										break;
									case 1:
										if (influenceVoxels[k].second(1)==voxel(1)+1 &&
											influenceVoxels[k].second(0)==voxel(0) &&
											influenceVoxels[k].second(2)==voxel(2)){
											canChange = false;
										}
										break;
									default:
										break;
								}
								if (!canChange){
									break;
								}
							}
						}
						else{
							// check AM stability of the changed voxel
							if (diff.array().abs().maxCoeff() <= 1){
								canChange = false;
								break;
							}

							// check SM accessibility of the influence voxel
							if (influenceVoxels[k].first == 10000){
								if (voxel(2) > influenceVoxels[k].second(2)+1){
									canChange = false;
									break;
								}
								if (diff.head<2>().array().abs().maxCoeff() == 0 && 
									voxel(2)>=influenceVoxels[k].second(2)){
									canChange = false;
									break;
								}
							}
							else if (influenceVoxels[k].second(2) == voxel(2) &&
								influenceVoxels[k].first < 10000){

								switch (influenceVoxels[k].first){
									case 1000:
										if (voxel(0) == influenceVoxels[k].second(0)-1 &&
											voxel(1) == influenceVoxels[k].second(1)){
											canChange = false;
										}
										break;
									case 100:
										if (voxel(0) == influenceVoxels[k].second(0)+1 &&
											voxel(1) == influenceVoxels[k].second(1)){
											canChange = false;
										}
										break;
									case 10:
										if (voxel(1) == influenceVoxels[k].second(1)-1 &&
											voxel(0) == influenceVoxels[k].second(0)){
											canChange = false;
										}
										break;
									case 1:
										if (voxel(1) == influenceVoxels[k].second(1)+1 &&
											voxel(0) == influenceVoxels[k].second(0)){
											canChange = false;
										}
										break;
									default:
										break;
								}

								if (!canChange){
									break;
								}
							}
						}	
					}
					if (!canChange) {
						break;
					}
				}


			}

			if (canChange) {
				for (int j = 0; j < groupSet[nextIdx].voxelPos.size(); j++) {
					groupSet[i].voxelPos.push_back(groupSet[nextIdx].voxelPos[j]);
					updateGroupRange(groupSet[i], groupSet[nextIdx].voxelPos[j]);
				}
				groupSet[nextIdx].voxelPos.clear();
			}
			else {
				for (int j = 0; j < groupSet[nextIdx].voxelPos.size(); j++) {
					influenceVoxels.emplace_back(groupSet[nextIdx].opeType, groupSet[nextIdx].voxelPos[j]);
				}
			}
			nextIdx--;
		}
	}

	std::cout << std::endl;

	std::vector<opeGroup> groupSetNew;
	for (int i = 0; i < groupSet.size(); i++) {
		if (groupSet[i].voxelPos.size() == 0) {
			continue;
		}

		if (i < groupSet.size() - 1 && groupSet[i].voxelPos.size() == groupSet[i + 1].voxelPos.size()) {
			// check if the two groups include the same voxels
			std::unordered_set<Eigen::Vector3i, Vector3iHash, Vector3iEqual> set1(groupSet[i].voxelPos.begin(), groupSet[i].voxelPos.end());
			std::unordered_set<Eigen::Vector3i, Vector3iHash, Vector3iEqual> set2(groupSet[i + 1].voxelPos.begin(), groupSet[i + 1].voxelPos.end());
			if (set1 == set2) {
				groupSet[i].voxelPos.clear();
				groupSet[i + 1].voxelPos.clear();
				continue;
			}
		}

		//std::cout << i << " " << groupSet[i].opeType << " " << groupSet[i].voxelPos.size() << std::endl;

		if (groupSet[i].opeType < 100000) {
			std::vector<opeGroup> connectedGroups = splitGroupIntoConnected(groupSet[i]);
			for (int j = 0; j < connectedGroups.size(); j++) {
				std::vector<opeGroup> splitedGroups = splitSMGroup(connectedGroups[j]);
				groupSetNew.insert(groupSetNew.end(), splitedGroups.begin(), splitedGroups.end());
			}
		}
		else{
			std::vector<opeGroup> connectedGroups = splitGroupIntoConnected(groupSet[i]);
			groupSetNew.insert(groupSetNew.end(), connectedGroups.begin(), connectedGroups.end());
		}
	}

	std::cout << "adjust group set" << std::endl;

	delete accEroObj;
	return groupSetNew;
}