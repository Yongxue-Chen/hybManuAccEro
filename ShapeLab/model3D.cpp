#include "model3D.h"

model3D::model3D() {
	this->neighborDir.clear();

	for (int x = -1; x <= 1; x++) {
		for (int y = -1; y <= 1; y++) {
			for (int z = -1; z <= 1; z++) {
				if (x == 0 && y == 0 && z == 0) {
					continue;
				}
				if (abs(x) + abs(y) + abs(z) == 3) {
					continue;
				}
				Eigen::Vector3i dir(3);
				dir << x, y, z;
				this->neighborDir.push_back(dir);
			}
		}
	}
}


void model3D::init(Eigen::Vector3i size, Eigen::VectorXi matrixValue, Eigen::VectorXi outerStatus, bool pureCoreModel) {
	this->nx = size(0);
	this->ny = size(1);
	this->nz = size(2);
	this->nSpace = nx * ny * nz;

	//std::cout<< "nx: " << nx << " ny: " << ny << " nz: " << nz << " nSpace: " << nSpace << std::endl;

	this->xBrim = outerStatus(0);
	this->yBrim = outerStatus(1);
	this->zBrim = outerStatus(2);
	if (pureCoreModel) {
		this->outerBrim = 0;
		if (this->xBrim > 0 || this->yBrim > 0 || this->zBrim > 0) {
			std::cout << "Error: outer brim should be 0 for pure core model" << std::endl;
			exit(0);
		}
	}
	else {
		this->outerBrim = 1;
	}

	this->limitActionSpace.resize(6);
	this->limitActionSpace << xBrim + outerBrim, nx - xBrim - outerBrim - 1,
		yBrim + outerBrim, ny - yBrim - outerBrim - 1,
		zBrim + outerBrim, nz - 1;

	this->nActionSpace = (nx - xBrim * 2 - outerBrim * 2) * (ny - yBrim * 2 - outerBrim * 2) * (nz - zBrim - outerBrim);

	//std::cout<< "xBrim: " << xBrim << " yBrim: " << yBrim << " zBrim: " << zBrim << " outerBrim: " << outerBrim << std::endl;

	//std::cout << "matrixLength: " << matrixValue.size() << std::endl;

	this->targetModelValue = matrixValue;
	this->currentModelValue = matrixValue;

	getSolidBrimPos();


	this->onlyBrimValue = this->targetModelValue;
	this->startModelValue = this->targetModelValue;
	for (int x = limitActionSpace(0); x <= limitActionSpace(1); x++) {
		for (int y = limitActionSpace(2); y <= limitActionSpace(3); y++) {
			for (int z = limitActionSpace(4); z <= limitActionSpace(5); z++) {
				Eigen::Vector3i pos(3);
				pos << x, y, z;
				int index = coordinateToIndex(pos);
				this->onlyBrimValue(index) = -1;
				this->startModelValue(index) = 0;
			}
		}
	}

}

void model3D::resetVoxels() {
	currentModelValue = targetModelValue;
}

bool model3D::isEmpty() {
	int count = this->currentModelValue.sum();
	if (count == 0) {
		return true;
	}
	else {
		return false;
	}
}

Eigen::VectorXi model3D::getSolidLimit(Eigen::MatrixXi& height) {
	Eigen::VectorXi limit(5);
	limit << limitActionSpace(1), limitActionSpace(0), limitActionSpace(3), limitActionSpace(2), limitActionSpace(4);

	height.resize(this->nx, this->ny);
	height.setOnes();
	height *= -1;

	for (int x = limitActionSpace(0); x <= limitActionSpace(1); x++) {
		for (int y = limitActionSpace(2); y <= limitActionSpace(3); y++) {
			for (int z = limitActionSpace(4); z <= limitActionSpace(5); z++) {
				Eigen::Vector3i pos(3);
				pos << x, y, z;
				int index = coordinateToIndex(pos);
				if (currentModelValue(index) == 1) {
					if (x < limit(0)) {
						limit(0) = x;
					}
					if (x > limit(1)) {
						limit(1) = x;
					}
					if (y < limit(2)) {
						limit(2) = y;
					}
					if (y > limit(3)) {
						limit(3) = y;
					}
					if (z > limit(4)) {
						limit(4) = z;
					}
					height(x, y) = z;
				}

			}
		}
	}

	return limit;
}

int model3D::coordinateToIndex(Eigen::Vector3i pos) {
	int x = pos(0), y = pos(1), z = pos(2);
	int index = z * nx * ny + y * nx + x;
	return index;
}

Eigen::Vector3i model3D::indexToCoordinate(int index) {
	Eigen::Vector3i pos(3);
	pos(0) = index % nx;
	pos(1) = (index / nx) % ny;
	pos(2) = index / (nx * ny);
	return pos;
}

bool model3D::hasSolidNeighbor(Eigen::Vector3i pos) {
	bool hasNeighbor = false;

	for (int i = 0; i < this->neighborDir.size(); i++) {
		Eigen::Vector3i checkPos(3);
		checkPos = pos + this->neighborDir[i];
		if (checkPos(0) <= 0 || checkPos(0) >= nx - 1 || checkPos(1) <= 0 || checkPos(1) >= ny - 1 || checkPos(2) >= nz) {
			continue;
		}

		if (checkPos(2) < 0) {
			hasNeighbor = true;
			break;
		}

		int index = coordinateToIndex(checkPos);
		if (currentModelValue(index) == 1) {
			hasNeighbor = true;
			break;
		}

	}

	return hasNeighbor;
}

void model3D::getSolidBrimPos() {

	this->solidLimit<<nx-1,0,ny-1,0;

	bool getSolidLimit = false;
	for (int x=0;x<nx;x++){
		for (int y=0;y<ny;y++){
			for (int z=0;z<nz;z++){
				Eigen::Vector3i pos(3);
				pos<<x,y,z;
				int index = coordinateToIndex(pos);
				if (currentModelValue(index) == 1){
					this->solidLimit(0)=x;
					getSolidLimit = true;
					break;
				}
			}
			if (getSolidLimit){
				break;
			}
		}
		if (getSolidLimit){
			break;
		}
	}

	getSolidLimit = false;
	for (int x=nx-1;x>=0;x--){
		for (int y=0;y<ny;y++){
			for (int z=0;z<nz;z++){
				Eigen::Vector3i pos(3);
				pos<<x,y,z;
				int index = coordinateToIndex(pos);
				if (currentModelValue(index) == 1){
					this->solidLimit(1)=x;
					getSolidLimit = true;
					break;
				}
			}
			if (getSolidLimit){
				break;
			}
		}
		if (getSolidLimit){
			break;
		}
	}

	getSolidLimit = false;
	for (int y=0;y<ny;y++){
		for (int x=0;x<nx;x++){
			for (int z=0;z<nz;z++){
				Eigen::Vector3i pos(3);
				pos<<x,y,z;
				int index = coordinateToIndex(pos);
				if (currentModelValue(index) == 1){
					this->solidLimit(2)=y;
					getSolidLimit = true;
					break;
				}
			}
			if (getSolidLimit){
				break;
			}
		}
		if (getSolidLimit){
			break;
		}
	}

	getSolidLimit = false;
	for (int y=ny-1;y>=0;y--){
		for (int x=0;x<nx;x++){
			for (int z=0;z<nz;z++){
				Eigen::Vector3i pos(3);
				pos<<x,y,z;
				int index = coordinateToIndex(pos);
				if (currentModelValue(index) == 1){
					this->solidLimit(3)=y;
					getSolidLimit = true;
					break;
				}
			}
			if (getSolidLimit){
				break;
			}
		}
		if (getSolidLimit){
			break;
		}
	}

	this->solidBrimX1.clear();
	this->solidOuterX1.clear();
	for (int y = 0; y < ny; y++) {
		for (int z = 0; z < nz; z++) {
			for (int x = 0; x < xBrim + outerBrim; x++) {
				Eigen::Vector3i pos(3);
				pos << x, y, z;
				int index = coordinateToIndex(pos);
				if (currentModelValue(index) == 1) {
					if (x == 0) {
						this->solidOuterX1.push_back(Eigen::Vector2i(y, z));
					}
					else {
						this->solidBrimX1.push_back(Eigen::Vector2i(y, z));
						break;
					}	
				}
			}
		}
	}

	this->solidBrimX2.clear();
	this->solidOuterX2.clear();
	for (int y = 0; y < ny; y++) {
		for (int z = 0; z < nz; z++) {
			for (int x = nx - 1; x >= nx - xBrim - outerBrim; x--) {
				Eigen::Vector3i pos(3);
				pos << x, y, z;
				int index = coordinateToIndex(pos);
				if (currentModelValue(index) == 1) {
					if (x == nx - 1) {
						this->solidOuterX2.push_back(Eigen::Vector2i(y, z));
					}
					else {
						this->solidBrimX2.push_back(Eigen::Vector2i(y, z));
						break;
					}
				}
			}
		}
	}

	this->solidBrimY1.clear();
	this->solidOuterY1.clear();
	for (int x = 0; x < nx; x++) {
		for (int z = 0; z < nz; z++) {
			for (int y = 0; y < yBrim + outerBrim; y++) {
				Eigen::Vector3i pos(3);
				pos << x, y, z;
				int index = coordinateToIndex(pos);
				if (currentModelValue(index) == 1) {
					if (y == 0) {
						this->solidOuterY1.push_back(Eigen::Vector2i(x, z));
					}
					else {
						this->solidBrimY1.push_back(Eigen::Vector2i(x, z));
						break;
					}	
				}
			}
		}
	}

	this->solidBrimY2.clear();
	this->solidOuterY2.clear();
	for (int x = 0; x < nx; x++) {
		for (int z = 0; z < nz; z++) {
			for (int y = ny - 1; y >= ny - yBrim - outerBrim; y--) {
				Eigen::Vector3i pos(3);
				pos << x, y, z;
				int index = coordinateToIndex(pos);
				if (currentModelValue(index) == 1) {
					if (y == ny - 1) {
						this->solidOuterY2.push_back(Eigen::Vector2i(x, z));
					}
					else {
						this->solidBrimY2.push_back(Eigen::Vector2i(x, z));
						break;
					}
				}
			}
		}
	}

}

double model3D::distToSolidBrim(Eigen::Vector2i pos, int brimType) {
	//brimType: 0 - solidBrimX1, 1 - solidBrimX2, 2 - solidBrimY1, 3 - solidBrimY2

	std::vector<Eigen::Vector2i> brim;
	switch (brimType) {
	case 0:
		brim = this->solidOuterX1;
		break;
	case 1:
		brim = this->solidOuterX2;
		break;
	case 2:
		brim = this->solidOuterY1;
		break;
	case 3:
		brim = this->solidOuterY2;
		break;
	case 4:
		brim = this->solidBrimX1;
		break;
	case 5:
		brim = this->solidBrimX2;
		break;
	case 6:
		brim = this->solidBrimY1;
		break;
	case 7:
		brim = this->solidBrimY2;
		break;
	default:
		return -1;
	}

	double minDist = 1e10;
	for (int i = 0; i < brim.size(); i++) {
		double dist = sqrt(pow(pos(0) - brim[i](0), 2) + pow(pos(1) - brim[i](1), 2));
		if (dist < minDist) {
			minDist = dist;
		}
	}

	return minDist;
}

bool model3D::isSupportingFree(Eigen::Vector3i pos) {
	Eigen::Vector3i checkPos;
	bool isSupporting = false;

	// check the status of the voxel underneath
	if (pos(2) <= 0) {
		std::cout << "Error: z coordinate should be larger than 0" << std::endl;
		return false;
	}
	checkPos = pos;
	checkPos(2) -= 1;
	int idx = coordinateToIndex(checkPos);
	if (currentModelValue(idx) == 1) {
		return isSupporting = true;
	}

	std::vector<Eigen::Vector3i> solidPos;
	int count = getSolidFaceNeighborSameHeight(checkPos, solidPos);
	if (count == 0) {
		return isSupporting = false;
	}
	else {
		return true;
	}

	idx = coordinateToIndex(checkPos);
	if (currentModelValue(idx) == 1) {
		if (count >= 2) {
			return isSupporting = true;
		}
		else {
			Eigen::Vector3i checkPos2 = solidPos[0];
			int count2 = getSolidFaceNeighborSameHeight(checkPos2, solidPos);
			if (count2 >= 2) {
				return isSupporting = true;
			}
			else {
				return isSupporting = false;
			}
		}
	}
	else {
		for (int i = 0; i < count; i++) {
			Eigen::Vector3i checkPos2 = solidPos[i];
			std::vector<Eigen::Vector3i> solidBasePos;
			solidBasePos.push_back(checkPos2);
			std::vector<Eigen::Vector3i> solidBasePos2;
			int count2 = getSolidFaceNeighborSameHeight(checkPos2, solidBasePos2);
			if (count2 >= 2) {
				return isSupporting = true;
			}
			else if (count2 == 0) {
				continue;
			}
			else {
				checkPos2 = solidBasePos2[0];
				count2 = getSolidFaceNeighborSameHeight(checkPos2, solidBasePos2);
				if (count2 >= 2) {
					return isSupporting = true;
				}
				else {
					continue;
				}
			}
		}
	}

	return isSupporting;	
}

int model3D::getSolidFaceNeighborSameHeight(Eigen::Vector3i pos, std::vector<Eigen::Vector3i>& neighbor) {
	neighbor.clear();
	int count = 0;

	if (pos(0) > outerBrim) {
		Eigen::Vector3i checkPos = pos;
		checkPos(0) -= 1;
		int idx = coordinateToIndex(checkPos);
		if (currentModelValue(idx) == 1) {
			neighbor.push_back(checkPos);
			count++;
		}
	}

	if (pos(0) < nx - outerBrim - 1) {
		Eigen::Vector3i checkPos = pos;
		checkPos(0) += 1;
		int idx = coordinateToIndex(checkPos);
		if (currentModelValue(idx) == 1) {
			neighbor.push_back(checkPos);
			count++;
		}
	}

	if (pos(1) > outerBrim) {
		Eigen::Vector3i checkPos = pos;
		checkPos(1) -= 1;
		int idx = coordinateToIndex(checkPos);
		if (currentModelValue(idx) == 1) {
			neighbor.push_back(checkPos);
			count++;
		}
	}

	if (pos(1) < ny - outerBrim - 1) {
		Eigen::Vector3i checkPos = pos;
		checkPos(1) += 1;
		int idx = coordinateToIndex(checkPos);
		if (currentModelValue(idx) == 1) {
			neighbor.push_back(checkPos);
			count++;
		}
	}

	return count;
}

bool model3D::stillStable(Eigen::Vector3i pos, Eigen::VectorXi model) {
	bool isStable = false;
	int checkIdx = coordinateToIndex(pos);

	Eigen::VectorXi visited(nSpace);//visited elements
	visited.setZero();
	int numConnectedComponent = 0;//number of connected components
	Eigen::VectorXi modelNew;//new model after removing the voxel
	if (model.size() == 0) {
		modelNew = currentModelValue;
	}
	else {
		modelNew = model;
	}
	modelNew(checkIdx) = 0;
	std::vector<Eigen::Vector3i> queue;//queue for all non-zero elements

	//search in the neighborhood of pos
	int tmp1 = std::max(outerBrim, pos(0) - 1);
	int tmp2 = std::min(nx - outerBrim - 1, pos(0) + 1);
	int tmp3 = std::max(outerBrim, pos(1) - 1);
	int tmp4 = std::min(ny - outerBrim - 1, pos(1) + 1);
	int tmp5 = std::max(0, pos(2) - 1);
	int tmp6 = std::min(nz - 1, pos(2) + 1);
	for (int x = tmp1; x <= tmp2; x++) {
		for (int y = tmp3; y <= tmp4; y++) {
			for (int z = tmp5; z <= tmp6; z++) {
				if (x != pos(0) && y != pos(1) && z != pos(2)) {
					continue;
				}
				int idx = coordinateToIndex(Eigen::Vector3i(x, y, z));
				if (modelNew(idx) % 2 == 1 && visited(idx) == 0) {
					numConnectedComponent++;
					DFS(Eigen::Vector3i(x, y, z), numConnectedComponent, modelNew, visited, queue, tmp1, tmp2, tmp3, tmp4, tmp5, tmp6);
				}
			}
		}
	}

	if (numConnectedComponent == 0) {
		return isStable = true;
	}
	else if (numConnectedComponent == 1 && pos(2) > outerBrim) {
		return isStable = true;
	}

	Eigen::VectorXi connectedBottom(numConnectedComponent);
	connectedBottom.setZero();
	std::vector<std::deque<Eigen::Vector3i>> queueList(numConnectedComponent);
	Eigen::VectorXi componentSize(numConnectedComponent);
	componentSize.setZero();

	for (int i = 0; i < queue.size(); i++) {
		int idx = coordinateToIndex(queue[i]);
		int componentIdx = visited(idx);
		queueList[componentIdx - 1].push_back(queue[i]);
		componentSize(componentIdx - 1)++;
		if (queue[i](2) <= outerBrim) {
			connectedBottom(componentIdx - 1) = 1;
		}
	}
	queue.clear();
	queue.shrink_to_fit();

	if (numConnectedComponent == 1 && connectedBottom(0) == 1) {
		return isStable = true;
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
			for (int j = 0; j < this->neighborDir.size(); j++) {
				Eigen::Vector3i newPos = posTmp + this->neighborDir[j];
				if (newPos(0) < outerBrim || newPos(0) > this->nx - 1 - outerBrim
					|| newPos(1) < outerBrim || newPos(1) > this->ny - 1 - outerBrim
					|| newPos(2) < outerBrim || newPos(2) > this->nz - 1) {
					continue;
				}
				int newIdx = coordinateToIndex(newPos);
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
					if (newPos(2) <= outerBrim) {
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
						visited(coordinateToIndex(posTmp2)) = i + 1;
					}
					queueList[visitedIdx - 1].clear();
					queueList[visitedIdx - 1].shrink_to_fit();
					componentSize(visitedIdx - 1) = 0;

					int tmpConnect = connectedBottom(visitedIdx - 1);
					connectedBottom(visitedIdx - 1) = 1 + i + 1;

					newNumConnected--;
					if (newNumConnected == 1 && pos(2) > outerBrim) {
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
					return false;
				}
			}

		}
	}

	if (connectedBottom.minCoeff() == 0) {
		return false;
	}
	else {
		return true;
	}

}


bool model3D::stillStable(std::vector<Eigen::Vector3i> posList, Eigen::VectorXi model) {
	bool isStable = false;

	Eigen::VectorXi modelNew;//new model after removing the voxel
	if (model.size() == 0) {
		modelNew = currentModelValue;
	}
	else {
		modelNew = model;
	}

	std::vector<int> checkIdxList;
	checkIdxList.reserve(posList.size());
	for (int i = 0; i < posList.size(); i++) {
		int checkIdx = coordinateToIndex(posList[i]);
		checkIdxList.push_back(checkIdx);
		modelNew(checkIdx) = 0;
	}

	Eigen::VectorXi visited(nSpace);//visited elements
	visited.setZero();
	int numConnectedComponent = 0;//number of connected components
	
	std::vector<Eigen::Vector3i> queue;//queue for all non-zero elements

	//search in the neighborhood of pos

	for (int i = 0; i < posList.size(); i++) {
		Eigen::Vector3i pos = posList[i];
		int tmp1 = std::max(outerBrim, pos(0) - 1);
		int tmp2 = std::min(nx - outerBrim - 1, pos(0) + 1);
		int tmp3 = std::max(outerBrim, pos(1) - 1);
		int tmp4 = std::min(ny - outerBrim - 1, pos(1) + 1);
		int tmp5 = std::max(0, pos(2) - 1);
		int tmp6 = std::min(nz - 1, pos(2) + 1);
		for (int x = tmp1; x <= tmp2; x++) {
			for (int y = tmp3; y <= tmp4; y++) {
				for (int z = tmp5; z <= tmp6; z++) {
					if (x != pos(0) && y != pos(1) && z != pos(2)) {
						continue;
					}
					int idx = coordinateToIndex(Eigen::Vector3i(x, y, z));
					if (modelNew(idx) % 2 == 1 && visited(idx) == 0) {
						numConnectedComponent++;
						DFS(Eigen::Vector3i(x, y, z), numConnectedComponent, modelNew, visited, queue, tmp1, tmp2, tmp3, tmp4, tmp5, tmp6);
					}
				}
			}
		}
	}

	if (numConnectedComponent == 0) {
		return isStable = true;
	}
	else if (numConnectedComponent == 1 && posList[0](2) > outerBrim) {
		return isStable = true;
	}

	Eigen::VectorXi connectedBottom(numConnectedComponent);
	connectedBottom.setZero();
	std::vector<std::deque<Eigen::Vector3i>> queueList(numConnectedComponent);
	Eigen::VectorXi componentSize(numConnectedComponent);
	componentSize.setZero();

	for (int i = 0; i < queue.size(); i++) {
		int idx = coordinateToIndex(queue[i]);
		int componentIdx = visited(idx);
		queueList[componentIdx - 1].push_back(queue[i]);
		componentSize(componentIdx - 1)++;
		if (queue[i](2) <= outerBrim) {
			connectedBottom(componentIdx - 1) = 1;
		}
	}
	queue.clear();
	queue.shrink_to_fit();

	if (numConnectedComponent == 1 && connectedBottom(0) == 1) {
		return isStable = true;
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
			for (int j = 0; j < this->neighborDir.size(); j++) {
				Eigen::Vector3i newPos = posTmp + this->neighborDir[j];
				if (newPos(0) < outerBrim || newPos(0) > this->nx - 1 - outerBrim
					|| newPos(1) < outerBrim || newPos(1) > this->ny - 1 - outerBrim
					|| newPos(2) < outerBrim || newPos(2) > this->nz - 1) {
					continue;
				}
				int newIdx = coordinateToIndex(newPos);
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
					if (newPos(2) <= outerBrim) {
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
						visited(coordinateToIndex(posTmp2)) = i + 1;
					}
					queueList[visitedIdx - 1].clear();
					queueList[visitedIdx - 1].shrink_to_fit();
					componentSize(visitedIdx - 1) = 0;

					int tmpConnect = connectedBottom(visitedIdx - 1);
					connectedBottom(visitedIdx - 1) = 1 + i + 1;

					newNumConnected--;
					if (newNumConnected == 1 && posList[0](2) > outerBrim) {
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
					return false;
				}
			}

		}
	}

	if (connectedBottom.minCoeff() == 0) {
		return false;
	}
	else {
		return true;
	}

}





void model3D::DFS(Eigen::Vector3i pos, int componentIdx, const Eigen::VectorXi& modelNew, Eigen::VectorXi& visited,
	std::vector<Eigen::Vector3i>& queue, int xl, int xh, int yl, int yh, int zl, int zh) {

	if (pos(0) < xl || pos(0) > xh || pos(1) < yl || pos(1) > yh || pos(2) < zl || pos(2) > zh) {
		return;
	}
	int idx = coordinateToIndex(pos);
	if (modelNew(idx) % 2 == 0 || visited(idx) != 0) {
		return;
	}
	visited(idx) = componentIdx;
	queue.push_back(pos);

	for (int i = 0; i < this->neighborDir.size(); i++) {
		Eigen::Vector3i newPos = pos + this->neighborDir[i];
		DFS(newPos, componentIdx, modelNew, visited, queue, xl, xh, yl, yh, zl, zh);
	}

}

Eigen::VectorXi model3D::expandModel(Eigen::VectorXi model, Eigen::Vector3i& size, int expansionSize) {
	int nxIni = size(0), nyIni = size(1), nzIni = size(2);
	int nxNew = nxIni + expansionSize * 2;
	int nyNew = nyIni + expansionSize * 2;
	int nzNew = nzIni;
	Eigen::VectorXi modelNew(nxNew * nyNew * nzNew);
	modelNew.setZero();

	for (int x = 0; x < nxIni; x++) {
		for (int y = 0; y < nyIni; y++) {
			for (int z = 0; z < nzIni; z++) {
				int idx = z * nxIni * nyIni + y * nxIni + x;
				if (model(idx) == 1) {
					int xNew = x + expansionSize;
					int yNew = y + expansionSize;
					int zNew = z;
					int idxNew = zNew * nxNew * nyNew + yNew * nxNew + xNew;
					modelNew(idxNew) = 1;
				}
			}
		}
	}

	size << nxNew, nyNew, nzNew;

	return modelNew;
}

Eigen::VectorXi model3D::wrapModel(Eigen::VectorXi model, Eigen::Vector3i& size, std::vector<int>& wrapIdx) {
	int nxIni = size(0), nyIni = size(1), nzIni = size(2);
	int nxNew = nxIni + 2;
	int nyNew = nyIni + 2;
	int nzNew = nzIni + 1;
	Eigen::VectorXi modelNew(nxNew * nyNew * nzNew);
	modelNew.setZero();
	size << nxNew, nyNew, nzNew;

	// get the expanded model
	for (int x = 0; x < nxIni; x++) {
		for (int y = 0; y < nyIni; y++) {
			for (int z = 0; z < nzIni; z++) {
				int idx = z * nxIni * nyIni + y * nxIni + x;
				if (model(idx) == 1) {
					int xNew = x + 1;
					int yNew = y + 1;
					int zNew = z;
					int idxNew = zNew * nxNew * nyNew + yNew * nxNew + xNew;
					modelNew(idxNew) = 1;
				}
			}
		}
	}

	wrapIdx.clear();
	// wrap the expanded model
	for (int z=0;z<nzIni;z++){
		// x direction
		for (int y=0;y<nyNew;y++){
			// -x direction
			if (modelNew(z * nxNew * nyNew + y * nxNew + 1) == 1){
				int idx = z * nxNew * nyNew + y * nxNew;
				modelNew(idx) = 1;
				wrapIdx.push_back(idx);
			}
			else if (modelNew(z * nxNew * nyNew + y * nxNew + 2) == 1){
				int idx = z * nxNew * nyNew + y * nxNew + 1;
				modelNew(idx) = 1;
				wrapIdx.push_back(idx);
			}
			// +x direction
			if (modelNew(z * nxNew * nyNew + y * nxNew + nxNew - 2) == 1){
				int idx = z * nxNew * nyNew + y * nxNew + nxNew - 1;
				modelNew(idx) = 1;
				wrapIdx.push_back(idx);
			}
			else if (modelNew(z * nxNew * nyNew + y * nxNew + nxNew - 3) == 1){
				int idx = z * nxNew * nyNew + y * nxNew + nxNew - 2;
				modelNew(idx) = 1;
				wrapIdx.push_back(idx);
			}
		}

		// y direction
		for (int x=0;x<nxNew;x++){
			// -y direction
			if (modelNew(z * nxNew * nyNew + 1 * nxNew + x) == 1){
				int idx = z * nxNew * nyNew+x;
				modelNew(idx) = 1;
				wrapIdx.push_back(idx);
			}
			else if (modelNew(z * nxNew * nyNew + 2 * nxNew + x) == 1){
				int idx = z * nxNew * nyNew + 1 * nxNew + x;
				modelNew(idx) = 1;
				wrapIdx.push_back(idx);
			}
			// +y direction
			if (modelNew(z * nxNew * nyNew + (nyNew - 2) * nxNew + x) == 1){
				int idx = z * nxNew * nyNew + (nyNew - 1) * nxNew + x;
				modelNew(idx) = 1;
				wrapIdx.push_back(idx);
			}
			else if (modelNew(z * nxNew * nyNew + (nyNew - 3) * nxNew + x) == 1){
				int idx = z * nxNew * nyNew + (nyNew - 2) * nxNew + x;
				modelNew(idx) = 1;
				wrapIdx.push_back(idx);
			}
		}
	}

	return modelNew;
}


Eigen::VectorXi model3D::augmentModel(Eigen::VectorXi model, Eigen::Vector3i size, Eigen::Vector3i startPos, Eigen::Vector3i coreSize, Eigen::Vector3i shellSize) {
	if (startPos(0) < 0 || startPos(0) >= size(0) || startPos(1) < 0 || startPos(1) >= size(1) || startPos(2) < 0 || startPos(2) >= size(2)) {
		std::cout << "Error: start position is out of range" << std::endl;
		return Eigen::VectorXi(0);
	}
	
	int nxAug = coreSize(0) + 2 * shellSize(0) + 2;
	int nyAug = coreSize(1) + 2 * shellSize(1) + 2;
	int nzAug = coreSize(2) + shellSize(2) + 1;

	int shellStartX = startPos(0) - shellSize(0);
	int shellEndX = startPos(0) + coreSize(0) + shellSize(0) - 1;
	int shellStartY = startPos(1) - shellSize(1);
	int shellEndY = startPos(1) + coreSize(1) + shellSize(1) - 1;
	int shellStartZ = startPos(2);
	int shellEndZ = startPos(2) - coreSize(2) - shellSize(2) + 1;

	Eigen::VectorXi augModel(nxAug * nyAug * nzAug);
	augModel.setZero();

	//fill the core region and the shell region
	for (int z = shellStartZ; z >= shellEndZ; z--) {
		for (int x = shellStartX; x <= shellEndX; x++) {
			for (int y = shellStartY; y <= shellEndY; y++) {
				int value;
				if (x < 0 || x >= size(0) || y < 0 || y >= size(1)) {
					value = 0;
					continue;
				}
				else {
					if (z < 0) {
						value = 1;
					}
					else {
						int idx = z * size(0) * size(1) + y * size(0) + x;
						value = model(idx);
					}
				}

				if (value == 1) {
					int xNew = x - shellStartX + 1;
					int yNew = y - shellStartY + 1;
					int zNew = z - shellEndZ + 1;
					int idxNew = zNew * nxAug * nyAug + yNew * nxAug + xNew;
					augModel(idxNew) = 1;
				}
			}
		}
	}

	// fill the boundary region
	for (int z = shellStartZ; z >= shellEndZ; z--) {

		// x direction
		for (int y = shellStartY; y <= shellEndY; y++) {
			int yNew = y - shellStartY + 1;
			int zNew = z - shellEndZ + 1;

			// -x direction
			if (shellStartX > 0) {
				int xNew = 0;
				int idxNew = zNew * nxAug * nyAug + yNew * nxAug + xNew;

				if (y < 0 || y >= size(1)) {
					continue;
				}
				if (z < 0) {
					augModel(idxNew) = 1;
					continue;
				}
				for (int x = 0; x < shellStartX; x++) {
					int idx = z * size(0) * size(1) + y * size(0) + x;
					if (model(idx) == 1) {
						augModel(idxNew) = 1;
						break;
					}
				}
			}

			// +x direction
			if (shellEndX < size(0) - 1) {
				int xNew = nxAug - 1;
				int idxNew = zNew * nxAug * nyAug + yNew * nxAug + xNew;

				if (y < 0 || y >= size(1)) {
					continue;
				}
				if (z < 0) {
					augModel(idxNew) = 1;
					continue;
				}
				for (int x = shellEndX + 1; x < size(0); x++) {
					int idx = z * size(0) * size(1) + y * size(0) + x;
					if (model(idx) == 1) {
						augModel(idxNew) = 1;
						break;
					}
				}
			}
		}

		// y direction
		for (int x = shellStartX; x <= shellEndX; x++) {
			int xNew = x - shellStartX + 1;
			int zNew = z - shellEndZ + 1;

			// -y direction
			if (shellStartY > 0) {
				int yNew = 0;
				int idxNew = zNew * nxAug * nyAug + yNew * nxAug + xNew;

				if (x < 0 || x >= size(0)) {
					continue;
				}
				if (z < 0) {
					augModel(idxNew) = 1;
					continue;
				}
				for (int y = 0; y < shellStartY; y++) {
					int idx = z * size(0) * size(1) + y * size(0) + x;
					if (model(idx) == 1) {
						augModel(idxNew) = 1;
						break;
					}
				}
			}

			// +y direction
			if (shellEndY < size(1) - 1) {
				int yNew = nyAug - 1;
				int idxNew = zNew * nxAug * nyAug + yNew * nxAug + xNew;
				if (x < 0 || x >= size(0)) {
					continue;
				}
				if (z < 0) {
					augModel(idxNew) = 1;
					continue;
				}
				for (int y = size(1) - 1; y > shellEndY; y--) {
					int idx = z * size(0) * size(1) + y * size(0) + x;
					if (model(idx) == 1) {
						augModel(idxNew) = 1;
						break;
					}
				}
			}
		}

		// -x -y direction
		if (shellStartY > 0 && shellStartX > 0) {
			int zNew = z - shellEndZ + 1;
			int idxNew = zNew * nxAug * nyAug;

			if (z < 0) {
				augModel(idxNew) = 1;
			}
			else {
				for (int x = 0; x < shellStartX; x++) {
					for (int y = 0; y < shellStartY; y++) {
						int idx = z * size(0) * size(1) + y * size(0) + x;
						if (model(idx) == 1) {
							augModel(idxNew) = 1;
							break;
						}
					}
					if (augModel(idxNew) == 1) {
						break;
					}
				}
			}
		}

		// -x +y direction
		if (shellStartX > 0 && shellEndY < size(1) - 1) {
			int yNew = nyAug - 1;
			int zNew = z - shellEndZ + 1;
			int idxNew = zNew * nxAug * nyAug + yNew * nxAug;

			if (z < 0) {
				augModel(idxNew) = 1;
			}
			else {
				for (int x = 0; x < shellStartX; x++) {
					for (int y = size(1) - 1; y > shellEndY; y--) {
						int idx = z * size(0) * size(1) + y * size(0) + x;
						if (model(idx) == 1) {
							augModel(idxNew) = 1;
							break;
						}
					}
					if (augModel(idxNew) == 1) {
						break;
					}
				}
			}
		}

		// +x -y direction
		if (shellStartY > 0 && shellEndX < size(0) - 1) {
			int xNew = nxAug - 1;
			int zNew = z - shellEndZ + 1;
			int idxNew = zNew * nxAug * nyAug + xNew;

			if (z < 0) {
				augModel(idxNew) = 1;
			}
			else {
				for (int x = shellEndX + 1; x < size(0); x++) {
					for (int y = 0; y < shellStartY; y++) {
						int idx = z * size(0) * size(1) + y * size(0) + x;
						if (model(idx) == 1) {
							augModel(idxNew) = 1;
							break;
						}
					}
					if (augModel(idxNew) == 1) {
						break;
					}
				}
			}
		}

		// +x +y direction
		if (shellEndX < size(0) - 1 && shellEndY < size(1) - 1) {
			int xNew = nxAug - 1;
			int yNew = nyAug - 1;
			int zNew = z - shellEndZ + 1;
			int idxNew = zNew * nxAug * nyAug + yNew * nxAug + xNew;

			if (z < 0) {
				augModel(idxNew) = 1;
			}
			else {
				for (int x = shellEndX + 1; x < size(0); x++) {
					for (int y = size(1) - 1; y > shellEndY; y--) {
						int idx = z * size(0) * size(1) + y * size(0) + x;
						if (model(idx) == 1) {
							augModel(idxNew) = 1;
							break;
						}
					}
					if (augModel(idxNew) == 1) {
						break;
					}
				}
			}
		}
	}

	// the lowest boundary region
	for (int x = shellStartX - 1; x <= shellEndX + 1; x++) {
		for (int y = shellStartY - 1; y <= shellEndY + 1; y++) {
			if (x < 0 || x >= size(0) || y < 0 || y >= size(1)) {
				continue;
			}

			int xNew = x - shellStartX + 1;
			int yNew = y - shellStartY + 1;
			int idxNew = yNew * nxAug + xNew;
			augModel(idxNew) = 1;
		}
	}

	return augModel;
}