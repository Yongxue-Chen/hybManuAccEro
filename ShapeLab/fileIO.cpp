#include "fileIO.h"

Eigen::MatrixXd fileIO::readTXTfile(std::string filename) {
	std::ifstream file(filename);
	if (!file.is_open()) {
		std::cerr << "Could not open file " << filename << std::endl;
		return Eigen::MatrixXd();
	}

	std::vector<std::vector<double>> data;
	std::string line;

	while (std::getline(file, line)) {
		std::vector<double> row;
		std::stringstream ss(line);
		std::string cell;

		while (std::getline(ss, cell, ',')) {
			row.push_back(std::stod(cell));
		}

		data.push_back(row);
	}

	file.close();

	Eigen::MatrixXd matrix(data.size(), data[0].size());
	for (int i = 0; i < data.size(); i++) {
		for (int j = 0; j < data[0].size(); j++) {
			matrix(i, j) = data[i][j];
		}
	}

	std::cout << "Read file successfully:\n"
		<< "Rows: " << matrix.rows() << "\n"
		<< "Cols: " << matrix.cols() << std::endl;

	//std::cout << "Matrix:\n" << matrix << "\n\n";

	return matrix;
}

Eigen::MatrixXd fileIO::rot90(Eigen::MatrixXd matrix, int k) {
	Eigen::MatrixXd rotatedMatrix = matrix;
	k = ((k % 4) + 4) % 4;// make sure k is between 0 and 3

	std::cout << "Rotating matrix by " << k * 90 << " degrees" << std::endl;

	Eigen::MatrixXd newMatrix;
	for (int i = 0; i < k; i++) {
		newMatrix = rotatedMatrix.transpose();
		//std::cout << "Transposed matrix:\n" << newMatrix << std::endl;
		rotatedMatrix = newMatrix.colwise().reverse();
		//std::cout << "Reversed matrix:\n" << rotatedMatrix << std::endl;
	}

	return rotatedMatrix;
	
}

Eigen::SparseMatrix<int> fileIO::readModelMatrix(std::string filename) {
	Eigen::MatrixXd matrix = readTXTfile(filename);
	matrix = rot90(matrix, -1); // rotate matrix by -90 degrees counter-clockwise
	//std::cout << "Rotated matrix:\n" << matrix << std::endl;

	int extendedRows = 2;

	Eigen::MatrixXi extendedMatrix(matrix.rows() + 2 * extendedRows, matrix.cols());
	extendedMatrix.topRows(extendedRows).setZero();
	extendedMatrix.bottomRows(extendedRows).setZero();
	extendedMatrix.middleRows(extendedRows, matrix.rows()) = matrix.cast<int>();

	//std::cout << "Extended matrix:\n" << extendedMatrix << std::endl;

	Eigen::SparseMatrix<int> sparseMatrix = extendedMatrix.sparseView(1e-3);
	std::cout << "Converted to sparse matrix" << std::endl;

	return sparseMatrix;
}

void fileIO::writeMatrix(Eigen::MatrixXd matrix, std::string filename, int precision) {
	std::ofstream file(filename);
	if (!file.is_open()) {
		std::cerr << "Could not open file " << filename << std::endl;
		return;
	}

	//set the precision of the output
	file << std::fixed << std::setprecision(precision);


	for (int i = 0; i < matrix.rows(); i++) {
		for (int j = 0; j < matrix.cols(); j++) {
			file << matrix(i, j);
			if (j < matrix.cols() - 1) {
				file << ",";
			}
		}
		file << std::endl;
	}

	file.close();
	//std::cout << "Matrix written to file " << filename << std::endl;
}

Eigen::MatrixXd fileIO::readMatrix(std::string filename) {
	Eigen::MatrixXd matrix = readTXTfile(filename);
	return matrix;
}


void fileIO::writeVector(Eigen::VectorXi vector, std::string filename) {
	std::ofstream file(filename);
	if (!file.is_open()) {
		std::cerr << "Could not open file " << filename << std::endl;
		return;
	}

	for (int i = 0; i < vector.size(); i++) {
		file << vector(i);
		if (i < vector.size() - 1) {
			file << ",";
		}
	}

	file.close();
	//std::cout << "Vector written to file " << filename << std::endl;
}

void fileIO::readVector(std::string filename, Eigen::VectorXi& vector) {
	std::ifstream file(filename);
	if (!file.is_open()) {
		std::cerr << "Could not open file " << filename << std::endl;
		return;
	}

	std::string line;
	std::getline(file, line);
	std::stringstream ss(line);
	std::string cell;
	std::vector<int> data;

	while (std::getline(ss, cell, ',')) {
		data.push_back(std::stoi(cell));
	}

	vector = Eigen::Map<Eigen::VectorXi>(data.data(), data.size());

	file.close();
}

void fileIO::writeString(std::string str, std::string filename) {
	std::ofstream file(filename);
	if (!file.is_open()) {
		std::cerr << "Could not open file " << filename << std::endl;
		return;
	}

	file << str;
	file.close();
	//std::cout << "String written to file " << filename << std::endl;
}

void fileIO::writeSparseMatrix(const Eigen::SparseMatrix<int>& matrix, std::string filename, bool size) {
	std::ofstream file(filename);
	if (!file.is_open()) {
		std::cerr << "Could not open file " << filename << std::endl;
		return;
	}

	if (size) {
		file << matrix.rows() << "," << matrix.cols() << std::endl;
	}

	for (int k = 0; k < matrix.outerSize(); ++k) {
		for (Eigen::SparseMatrix<int>::InnerIterator it(matrix, k); it; ++it) {
			file << it.row() << "," << it.col() << "," << it.value() << std::endl;
		}
	}

	file.close();
	//std::cout << "Sparse matrix written to file " << filename << std::endl;
}

Eigen::SparseMatrix<int> fileIO::readSparseMatrix(std::string filename) {
	std::ifstream file(filename);
	if (!file.is_open()) {
		std::cerr << "Could not open file " << filename << std::endl;
		return Eigen::SparseMatrix<int>();
	}

	std::string line;
	std::getline(file, line);
	std::stringstream ss(line);
	std::string cell;
	std::vector<int> size;

	while (std::getline(ss, cell, ',')) {
		size.push_back(std::stoi(cell));
	}

	std::vector<Eigen::Triplet<int>> triplets;
	while (std::getline(file, line)) {
		std::stringstream ss(line);
		std::vector<int> row;
		while (std::getline(ss, cell, ',')) {
			row.push_back(std::stoi(cell));
		}
		triplets.push_back(Eigen::Triplet<int>(row[0], row[1], row[2]));
	}

	Eigen::SparseMatrix<int> matrix(size[0], size[1]);
	matrix.setFromTriplets(triplets.begin(), triplets.end());

	file.close();
	return matrix;
}


void fileIO::writeHMP(std::vector<std::vector<int>> Time, int tLast, std::string filename) {
	std::ofstream file(filename);
	if (!file.is_open()) {
		std::cerr << "Could not open file " << filename << std::endl;
		return;
	}

	file << tLast << "," << Time.size() << std::endl; // write tLast and the number of rows in Time
	for (int i = 0; i < Time.size(); i++) {
		for (int j = 0; j < Time[i].size(); j++) {
			file << Time[i][j];
			if (j < Time[i].size() - 1) {
				file << ",";
			}
		}
		file << std::endl;
	}

	file.close();
	//std::cout << "HMP struct written to file " << filename << std::endl;
}

void fileIO::readHMP(std::string filename, std::vector<std::vector<int>>& Time, int& tLast) {
	std::ifstream file(filename);
	if (!file.is_open()) {
		std::cerr << "Could not open file " << filename << std::endl;
		return;
	}

	std::string line;
	std::getline(file, line);
	std::stringstream ss(line);
	std::string cell;
	std::vector<int> size;

	while (std::getline(ss, cell, ',')) {
		size.push_back(std::stoi(cell));
	}
	tLast = size[0];
	Time.resize(size[1]);

	int i = 0;
	while (std::getline(file, line)) {
		std::stringstream ss(line);
		std::vector<int> row;
		while (std::getline(ss, cell, ',')) {
			row.push_back(std::stoi(cell));
		}
		Time[i] = row;
		i++;
	}

	file.close();
}

void fileIO::writeAccEroElement(std::vector<std::vector<int>> Element, std::string filename) {
	std::ofstream file(filename);
	if (!file.is_open()) {
		std::cerr << "Could not open file " << filename << std::endl;
		return;
	}

	file << Element.size() << std::endl; // write the number of rows in Element

	for (int i = 0; i < Element.size(); i++) {
		for (int j = 0; j < Element[i].size(); j++) {
			file << Element[i][j];
			if (j < Element[i].size() - 1) {
				file << ",";
			}
		}
		file << std::endl;
	}

	file.close();
	//std::cout << "AccEro elements written to file " << filename << std::endl;
}

std::vector<std::vector<int>> fileIO::readAccEroElement(std::string filename) {
	std::ifstream file(filename);
	if (!file.is_open()) {
		std::cerr << "Could not open file " << filename << std::endl;
		return std::vector<std::vector<int>>();
	}

	std::string line;
	std::getline(file, line);
	std::stringstream ss(line);
	std::string cell;
	std::vector<int> size;

	while (std::getline(ss, cell, ',')) {
		size.push_back(std::stoi(cell));
	}

	std::vector<std::vector<int>> Element(size[0]);

	int i = 0;
	while (std::getline(file, line)) {
		std::stringstream ss(line);
		std::vector<int> row;
		while (std::getline(ss, cell, ',')) {
			row.push_back(std::stoi(cell));
		}
		Element[i] = row;
		i++;
	}
	file.close();

	return Element;
}

void fileIO::clearFolder(std::string folder) {
	
	if (!std::filesystem::exists(folder)) {
		if (std::filesystem::create_directory(folder)) {
			std::cout << "Created folder " << folder << std::endl;
		} else {
			std::cerr << "Could not create folder " << folder << std::endl;
		}
	}

	try {
		for (const auto& entry : std::filesystem::directory_iterator(folder)) {
			std::filesystem::remove(entry.path());
		}
		std::cout << "Cleared folder " << folder << std::endl;
	} catch (const std::filesystem::filesystem_error& e) {
		std::cerr << "Error clearing folder " << folder << ": " << e.what() << std::endl;
	}
}

void fileIO::read3DMatrix(std::string filename, Eigen::Vector3i& size, Eigen::VectorXi& matrixValue, Eigen::VectorXi& outerStatus) {
	std::ifstream file(filename);
	if (!file.is_open()) {
		std::cerr << "Could not open file " << filename << std::endl;
		return;
	}

	std::string line;
	std::getline(file, line);
	std::stringstream ss(line);
	std::string cell;

	std::vector<int> sizeData;
	while (std::getline(ss, cell, ',')) {
		sizeData.push_back(std::stoi(cell));
	}
	size = Eigen::Map<Eigen::Vector3i>(sizeData.data(), sizeData.size());

	std::vector<int> outerData;
	std::getline(file, line);
	ss = std::stringstream(line);
	while (std::getline(ss, cell, ',')) {
		outerData.push_back(std::stoi(cell));
	}
	outerStatus = Eigen::Map<Eigen::VectorXi>(outerData.data(), outerData.size());

	std::vector<int> matrixData;
	while (std::getline(file, line)) {
		std::stringstream ss(line);
		while (std::getline(ss, cell, ',')) {
			matrixData.push_back(std::stoi(cell));
		}
	}
	matrixValue = Eigen::Map<Eigen::VectorXi>(matrixData.data(), matrixData.size());

	file.close();

}

void fileIO::writeMatrix(Eigen::MatrixXi matrix, std::string filename) {
	std::ofstream file(filename);
	if (!file.is_open()) {
		std::cerr << "Could not open file " << filename << std::endl;
		return;
	}

	for (int i = 0; i < matrix.rows(); i++) {
		for (int j = 0; j < matrix.cols(); j++) {
			file << matrix(i, j);
			if (j < matrix.cols() - 1) {
				file << ",";
			}
		}
		file << std::endl;
	}

	file.close();
}

Eigen::MatrixXi fileIO::readMatrixInt(std::string filename) {
	Eigen::MatrixXi matrix = readTXTfile(filename).cast<int>();
	return matrix;
}

Eigen::MatrixXi fileIO::read3DTrainingModel(std::string filename, Eigen::Vector3i& size) {
	std::ifstream file(filename);
	if (!file.is_open()) {
		std::cerr << "Could not open file " << filename << std::endl;
		return Eigen::MatrixXi();
	}

	std::string line;
	std::getline(file, line);
	std::stringstream ss(line);
	std::string cell;

	std::vector<int> sizeData;
	while (std::getline(ss, cell, ',')) {
		sizeData.push_back(std::stoi(cell));
	}
	size = Eigen::Map<Eigen::Vector3i>(sizeData.data(), sizeData.size());

	std::vector<std::vector<int>> data;
	while (std::getline(file, line)) {
		std::vector<int> row;
		std::stringstream ss(line);
		while (std::getline(ss, cell, ',')) {
			row.push_back(std::stoi(cell));
		}
		data.push_back(row);
	}

	file.close();

	Eigen::MatrixXi matrix(data.size(), data[0].size());
	for (int i = 0; i < data.size(); i++) {
		for (int j = 0; j < data[0].size(); j++) {
			matrix(i, j) = data[i][j];
		}
	}

	return matrix;
}

void fileIO::readHMP3D(std::string filename, Eigen::MatrixXi& T, int& tLast) {
	std::ifstream file(filename);
	if (!file.is_open()) {
		std::cerr << "Could not open file " << filename << std::endl;
		return;
	}

	std::string line;
	std::getline(file, line);
	std::stringstream ss(line);
	std::string cell;

	std::vector<int> data;
	while (std::getline(ss, cell, ',')) {
		data.push_back(std::stoi(cell));
	}
	tLast = data[0];

	std::vector<std::vector<int>> TData;
	while (std::getline(file, line)) {
		std::vector<int> row;
		std::stringstream ss(line);
		while (std::getline(ss, cell, ',')) {
			row.push_back(std::stoi(cell));
		}
		TData.push_back(row);
	}

	file.close();

	T = Eigen::MatrixXi(TData.size(), TData[0].size());
	for (int i = 0; i < TData.size(); i++) {
		for (int j = 0; j < TData[0].size(); j++) {
			T(i, j) = TData[i][j];
		}
	}

}

void fileIO::readHMP3D(std::string filename, Eigen::MatrixXd& T, double& tLast) {
	std::ifstream file(filename);
	if (!file.is_open()) {
		std::cerr << "Could not open file " << filename << std::endl;
		return;
	}

	std::string line;
	std::getline(file, line);
	std::stringstream ss(line);
	std::string cell;

	std::vector<double> data;
	while (std::getline(ss, cell, ',')) {
		data.push_back(std::stod(cell));
	}
	tLast = data[0];

	std::vector<std::vector<double>> TData;
	while (std::getline(file, line)) {
		std::vector<double> row;
		std::stringstream ss(line);
		while (std::getline(ss, cell, ',')) {
			row.push_back(std::stod(cell));
		}
		TData.push_back(row);
	}

	file.close();

	T = Eigen::MatrixXd(TData.size(), TData[0].size());
	for (int i = 0; i < TData.size(); i++) {
		for (int j = 0; j < TData[0].size(); j++) {
			T(i, j) = TData[i][j];
		}
	}
}

void fileIO::writeHMP3D(Eigen::MatrixXd T, double tLast, std::string filename) {
	std::ofstream file(filename);
	if (!file.is_open()) {
		std::cerr << "Could not open file " << filename << std::endl;
		return;
	}

	file << std::setprecision(16) << std::fixed;

	file << tLast << std::endl;

	for (int i = 0; i < T.rows(); i++) {
		for (int j = 0; j < T.cols(); j++) {
			file << T(i, j);
			if (j < T.cols() - 1) {
				file << ",";
			}
		}
		file << std::endl;
	}

	file.close();
}

void fileIO::writePathGroup(std::vector<Eigen::MatrixXd> path, int opeType, std::string filename) {
	std::ofstream file(filename);
	if (!file.is_open()) {
		std::cerr << "Could not open file " << filename << std::endl;
		return;
	}
	file << opeType << std::endl;
	file << path.size() << std::endl;

	for (const auto& p : path) {
		file << p.rows() << "," << p.cols() << std::endl;
		for (int i = 0; i < p.rows(); i++) {
			for (int j = 0; j < p.cols(); j++) {
				file << p(i, j);
				if (j < p.cols() - 1) {
					file << ",";
				}
			}
			file << std::endl;
		}
	}

	file.close();
}

void fileIO::saveVectorXd(const Eigen::VectorXd& vec, const std::string& filename) {
	std::ofstream out(filename, std::ios::binary);
	if (out.is_open()) {
		int size = vec.size();
		out.write(reinterpret_cast<const char*>(&size), sizeof(int));
		out.write(reinterpret_cast<const char*>(vec.data()), size * sizeof(double));
		out.close();
	}
	else {
		std::cerr << "Error: " << filename << std::endl;
	}
}

void fileIO::saveStdVector(const std::vector<double>& vec, const std::string& filename) {
	std::ofstream out(filename, std::ios::binary);
	if (out.is_open()) {
		size_t size = vec.size();
		out.write(reinterpret_cast<const char*>(&size), sizeof(size_t));
		out.write(reinterpret_cast<const char*>(vec.data()), size * sizeof(double));
		out.close();
	}
	else {
		std::cerr << "Error: " << filename << std::endl;
	}
}

Eigen::VectorXd fileIO::loadVectorXd(const std::string& filename) {
	std::ifstream in(filename, std::ios::binary);
	if (in.is_open()) {
		int size;
		in.read(reinterpret_cast<char*>(&size), sizeof(int));
		Eigen::VectorXd vec(size);
		in.read(reinterpret_cast<char*>(vec.data()), size * sizeof(double));
		in.close();
		return vec;
	}
	else {
		std::cerr << "Error: " << filename << std::endl;
		return Eigen::VectorXd();
	}
}

std::vector<double> fileIO::loadStdVector(const std::string& filename) {
	std::ifstream in(filename, std::ios::binary);
	if (in.is_open()) {
		size_t size;
		in.read(reinterpret_cast<char*>(&size), sizeof(size_t));
		std::vector<double> vec(size);
		in.read(reinterpret_cast<char*>(vec.data()), size * sizeof(double));
		in.close();
		return vec;
	}
	else {
		std::cerr << "Error: " << filename << std::endl;
		return std::vector<double>();
	}
}

#include <iostream>
#include <fstream>
#include <vector>
#include <Eigen/Sparse>

void fileIO::saveSparseMatrixCompressed(const Eigen::SparseMatrix<double>& mat, const std::string& filename) {
	std::ofstream out(filename, std::ios::binary);
	if (!out.is_open()) {
		std::cerr << "Failed to open file for writing: " << filename << std::endl;
		return;
	}

	// Ensure the matrix is compressed
	Eigen::SparseMatrix<double> compressedMat = mat;
	compressedMat.makeCompressed();

	int rows = compressedMat.rows();
	int cols = compressedMat.cols();
	int nonZeros = compressedMat.nonZeros();
	int outerSize = compressedMat.outerSize();

	// Determine storage order: 0 for ColMajor, 1 for RowMajor
	int storageOrder = (mat.IsRowMajor) ? 1 : 0;

	// Write metadata
	out.write(reinterpret_cast<const char*>(&rows), sizeof(int));
	out.write(reinterpret_cast<const char*>(&cols), sizeof(int));
	out.write(reinterpret_cast<const char*>(&nonZeros), sizeof(int));
	out.write(reinterpret_cast<const char*>(&outerSize), sizeof(int));
	out.write(reinterpret_cast<const char*>(&storageOrder), sizeof(int));  // New storage order flag

	// Write matrix data
	out.write(reinterpret_cast<const char*>(compressedMat.valuePtr()), nonZeros * sizeof(double));
	out.write(reinterpret_cast<const char*>(compressedMat.innerIndexPtr()), nonZeros * sizeof(int));
	out.write(reinterpret_cast<const char*>(compressedMat.outerIndexPtr()), (outerSize + 1) * sizeof(int));

	out.close();
}

Eigen::SparseMatrix<double> fileIO::loadSparseMatrixCompressed(const std::string& filename) {
	std::ifstream in(filename, std::ios::binary);
	if (!in.is_open()) {
		std::cerr << "Failed to open file for reading: " << filename << std::endl;
		return Eigen::SparseMatrix<double>();
	}

	int rows, cols, nonZeros, outerIndexSize, storageOrder;
	in.read(reinterpret_cast<char*>(&rows), sizeof(int));
	in.read(reinterpret_cast<char*>(&cols), sizeof(int));
	in.read(reinterpret_cast<char*>(&nonZeros), sizeof(int));
	in.read(reinterpret_cast<char*>(&outerIndexSize), sizeof(int));
	in.read(reinterpret_cast<char*>(&storageOrder), sizeof(int));  // Read storage order flag

	std::vector<double> values(nonZeros);
	std::vector<int> innerIndices(nonZeros);
	std::vector<int> outerIndices(outerIndexSize);

	in.read(reinterpret_cast<char*>(values.data()), nonZeros * sizeof(double));
	in.read(reinterpret_cast<char*>(innerIndices.data()), nonZeros * sizeof(int));
	in.read(reinterpret_cast<char*>(outerIndices.data()), outerIndexSize * sizeof(int));
	in.close();

	// Determine the matrix format and reconstruct accordingly
	if (storageOrder == 1) {  // RowMajor
		Eigen::Map<Eigen::SparseMatrix<double, Eigen::RowMajor, int>> mappedMat(
			rows, cols, nonZeros, outerIndices.data(), innerIndices.data(), values.data()
		);
		return Eigen::SparseMatrix<double>(mappedMat);
	}
	else {  // ColMajor (default)
		Eigen::Map<Eigen::SparseMatrix<double, Eigen::ColMajor, int>> mappedMat(
			rows, cols, nonZeros, outerIndices.data(), innerIndices.data(), values.data()
		);
		return Eigen::SparseMatrix<double>(mappedMat);
	}
}

Eigen::SparseVector<double> fileIO::loadSparseVector(const std::string& filename) {
	std::ifstream in(filename, std::ios::binary);
	if (!in.is_open()) {
		std::cerr << "Failed to open file for reading: " << filename << std::endl;
		return Eigen::SparseVector<double>();
	}

	int size, nonZeros;
	in.read(reinterpret_cast<char*>(&size), sizeof(int));
	in.read(reinterpret_cast<char*>(&nonZeros), sizeof(int));

	std::vector<double> values(nonZeros);
	std::vector<int> indices(nonZeros);
	in.read(reinterpret_cast<char*>(values.data()), nonZeros * sizeof(double));
	in.read(reinterpret_cast<char*>(indices.data()), nonZeros * sizeof(int));
	in.close();

	Eigen::SparseVector<double> vec(size);
	vec.reserve(nonZeros);
	for (int i = 0; i < nonZeros; ++i) {
		vec.insert(indices[i]) = values[i];
	}
	return vec;
}

void fileIO::saveSparseVector(const Eigen::SparseVector<double>& vec, const std::string& filename) {
	std::ofstream out(filename, std::ios::binary);
	if (!out.is_open()) {
		std::cerr << "Failed to open file for writing: " << filename << std::endl;
		return;
	}

	int size = static_cast<int>(vec.size());
	int nonZeros = static_cast<int>(vec.nonZeros());

	// Write metadata
	out.write(reinterpret_cast<const char*>(&size), sizeof(int));
	out.write(reinterpret_cast<const char*>(&nonZeros), sizeof(int));

	// Write vector data
	std::vector<double> values;
	std::vector<int> indices;
	values.reserve(nonZeros);
	indices.reserve(nonZeros);
	for (Eigen::SparseVector<double>::InnerIterator it(vec); it; ++it) {
		values.push_back(it.value());
		indices.push_back(it.index());
	}

	out.write(reinterpret_cast<const char*>(values.data()), nonZeros * sizeof(double));
	out.write(reinterpret_cast<const char*>(indices.data()), nonZeros * sizeof(int));

	out.close();
}


void fileIO::writeBoolToBinary(const std::string& filename, bool value) {
	std::ofstream out(filename, std::ios::binary);
	if (!out.is_open()) {
		std::cerr << "Failed to open file for writing: " << filename << std::endl;
		return;
	}

	char byteValue = value ? 1 : 0;
	out.write(reinterpret_cast<const char*>(&byteValue), sizeof(char));

	out.close();
}

bool fileIO::readBoolFromBinary(const std::string& filename) {
	std::ifstream in(filename, std::ios::binary);
	if (!in.is_open()) {
		std::cerr << "Failed to open file for reading: " << filename << std::endl;
		return false;
	}

	char byteValue;
	in.read(reinterpret_cast<char*>(&byteValue), sizeof(char));

	in.close();

	return byteValue != 0;
}

bool fileIO::directoryExists(const std::string& path) {
	struct stat info;
	if (stat(path.c_str(), &info) != 0) {
		return false;// cannot access the directory
	}
	return (info.st_mode & S_IFDIR) != 0;
}

bool fileIO::createDirectory(const std::string& path) {
	if (directoryExists(path)) {
		return true;
	}
	if (MKDIR(path.c_str()) == 0) {
		return true;
	}
	else {
		std::cerr << "Failed to create directory: " << path << std::endl;
		return false;
	}
}


void fileIO::startCommandFile(std::string filePath) {
	std::ofstream file(filePath);
	if (!file.is_open()) {
		std::cerr << "Failed to create file: " << filePath << std::endl;
		return;
	}

	file << "G21 (Set units to millimeters)\n";
	file << "G40 (Cancel cutter radius compensation)\n";
	file << "G49 (Cancel tool length offset)\n";
	file << "G80 (Cancel canned cycle)\n";
	file << "G90 (Set to absolute positioning mode)\n";
	file << "M5 (Stop spindle)\n";
	file << "G54 (Activate work coordinate system G54)\n";
	file << "G94 (Set feed rate mode to units per minute)\n";
	file << "M9002 (open port)\n\n";

	file.close();
}

void fileIO::endCommandFile(std::string filePath, int lastPos) {
	std::ofstream file(filePath, std::ios::app);
	if (!file.is_open()) {
		std::cerr << "Failed to open file: " << filePath << std::endl;
		return;
	}

	//if (lastPos == 100000) {
	//	file << "M9000 E-100 F600 (pullback filament)\n";
	//}

	file << "M9003 (close port)\n";

	file << "G1 Z200 F1000\n";
	file << "G1 B-42.245 F1000\n";
	file << "M6000\n\n";

	file << "M30 (End of program)\n";

	file.close();
}

void fileIO::changeToolCommand(std::string filePath, int lastPos, int currentPos, commandParas cmParas) {

	std::ofstream file(filePath, std::ios::app);
	if (!file.is_open()) {
		std::cerr << "Failed to open file: " << filePath << std::endl;
		return;
	}


	if (lastPos > 0 && lastPos < 10000 && currentPos>0 && currentPos < 10000) {
		// both SM horizontal

		file << "G1 Y" << cmParas.smHomeHorizontal(1) << " F" << cmParas.fastFeedRate << " (move to SM horizontal tmp home)\n";
		switch (currentPos) {
		case 1000:
			file << "G1 C" << cmParas.zeroC - 90 << " F" << cmParas.fastFeedRateLow << " (ajust C axis)\n";
			break;
		case 100:
			file << "G1 C" << cmParas.zeroC + 90 << " F" << cmParas.fastFeedRateLow << " (ajust C axis)\n";
			break;
		case 10:
			file << "G1 C" << cmParas.zeroC << " F" << cmParas.fastFeedRateLow << " (ajust C axis)\n";
			break;
		case 1:
			file << "G1 C" << cmParas.zeroC + 180 << " F" << cmParas.fastFeedRateLow << " (ajust C axis)\n";
			break;
		default:
			std::cout << "Error: currentPos is not valid" << std::endl;
			return;
		}

		file.close();
		return;
	}


	switch (lastPos) {
	case 0:
		//starting
		file << "M9010 T0 (select AM tool)\n";
		file << "G0 Z" << cmParas.amHome(2)
			<< " B" << cmParas.AM_B << " C" << cmParas.zeroC
			<< " (move to AM home)\n";
		file << "M9000 E101 F600 (push filament)\n";
		file << "G4 P15000 (wait for 15s)\n";
		break;
	case 100000:
		// AM
		//file << "M9000 E-100 F600 (pullback filament)\n";
		file << "G0 Z" << cmParas.amHome(2)
			<< " B" << cmParas.AM_B << " C" << cmParas.zeroC
			<< " (move to AM home)\n\n";
		//file << "G4 P8000 (wait for 8s)\n";
		break;
	case 10000:
		file << "M9009 S0 (turn off spindle)\n";
		file << "G0 Z" << cmParas.smHomeVertical(2)
			<< " B" << cmParas.SMvertical_B << " C" << cmParas.zeroC
			<< " (move to SM vertical home)\n\n";
		break;
	case 1000:
	case 100:
	case 10:
	case 1:
		file << "M9009 S0 (turn off spindle)\n";
		file << "G0 X" << cmParas.smHomeHorizontal(0) << " Y" << cmParas.smHomeHorizontal(1) << " Z" << cmParas.smHomeHorizontal(2)
			<< " B" << cmParas.SMhorizontal_B << " C" << cmParas.zeroC
			<< " (move to SM horizontal home)\n\n";
		break;
	default:
		std::cout << "Error: lastPos is not valid" << std::endl;
		return;
	}

	switch (currentPos) {
	case -1:
		file << "M9010 T0 (select AM tool)\n";
		file << "M9000 E-100 F600 (push filament)\n";
		break;
	case 100000:
		file << "M9010 T0 (select AM tool)\n";
		file << "G0 Z" << cmParas.amHome(2)
			<< " B" << cmParas.AM_B << " C" << cmParas.zeroC
			<< " (move to AM home)\n";
		//file << "\n\n";
		break;
	case 10000:
		file << "M9010 T3 (select SM tool)\n";
		file << "G0 Z" << cmParas.smHomeVertical(2)
			<< " B" << cmParas.SMvertical_B << " C" << cmParas.zeroC
			<< " (move to SM vertical home)\n";
		file << "M9009 S5500 (turn on spindle)\n";
		break;
	case 1000:
		file << "M9010 T3 (select SM tool)\n";
		file << "G0 X" << cmParas.smHomeHorizontal(0) << " Y" << cmParas.smHomeHorizontal(1) << " Z" << cmParas.smHomeHorizontal(2)
			<< " B" << cmParas.SMhorizontal_B << " C" << cmParas.zeroC - 90
			<< " (move to SM horizontal home)\n";
		file << "M9009 S5500 (turn on spindle)\n";
		break;
	case 100:
		file << "M9010 T3 (select SM tool)\n";
		file << "G0 X" << cmParas.smHomeHorizontal(0) << " Y" << cmParas.smHomeHorizontal(1) << " Z" << cmParas.smHomeHorizontal(2)
			<< " B" << cmParas.SMhorizontal_B << " C" << cmParas.zeroC + 90
			<< " (move to SM horizontal home)\n";
		file << "M9009 S5500 (turn on spindle)\n";
		break;
	case 10:
		file << "M9010 T3 (select SM tool)\n";
		file << "G0 X" << cmParas.smHomeHorizontal(0) << " Y" << cmParas.smHomeHorizontal(1) << " Z" << cmParas.smHomeHorizontal(2)
			<< " B" << cmParas.SMhorizontal_B << " C" << cmParas.zeroC
			<< " (move to SM horizontal home)\n";
		file << "M9009 S5500 (turn on spindle)\n";
		break;
	case 1:
		file << "M9010 T3 (select SM tool)\n";
		file << "G0 X" << cmParas.smHomeHorizontal(0) << " Y" << cmParas.smHomeHorizontal(1) << " Z" << cmParas.smHomeHorizontal(2)
			<< " B" << cmParas.SMhorizontal_B << " C" << cmParas.zeroC + 180
			<< " (move to SM horizontal home)\n";
		file << "M9009 S5500 (turn on spindle)\n";
		break;
	default:
		std::cout << "Error: currentPos is not valid" << std::endl;
		return;
	}

	file.close();
}

void fileIO::moveCommand(std::string filePath, std::vector<Eigen::MatrixXd> path, int opeType, commandParas cmParas, double cuttingArea) {
	std::ofstream file(filePath, std::ios::app);
	if (!file.is_open()) {
		std::cerr << "Failed to open file: " << filePath << std::endl;
		return;
	}

	double feedRate = cmParas.SMFeedRate;
	for (int i = 0; i < path.size(); i++) {
		Eigen::MatrixXd p = path[i];
		if (p.rows() == 0) {
			continue;
		}

		switch (opeType) {
		case 100000: {
			feedRate = cmParas.AMFeedRate;

			p.col(0).array() += cmParas.amCenterPosition(0);
			p.col(1).array() += cmParas.amCenterPosition(1);
			p.col(2).array() += cmParas.amCenterPosition(2);

			file << "G1 X" << p(0, 0) << " Y" << p(0, 1) << " Z" << p(0, 2) + cmParas.AMShift
				<< " B" << cmParas.AM_B << " C" << cmParas.zeroC
				<< " F" << cmParas.fastFeedRateLow << "\n";
			file << "G1 Z" << p(0, 2) << " F" << cmParas.AMFeedRate << "\n";
			file << "M9000 E7 F900\n";
			file << "G4 P1000\n";
			double lengthSeg = 0;
			for (int j = 1; j < p.rows(); j++) {
				lengthSeg += (p.row(j) - p.row(j - 1)).norm();
			}
			lengthSeg += 1;
			double ETmp = lengthSeg * cuttingArea;
			double Fe = cuttingArea * cmParas.AMFeedRate;
			file << "M9000 E" << ETmp << " F" << Fe << "\n";
			break;
		}
		case 10000:
			p.col(0).array() += cmParas.smCenterPositionVertical(0);
			p.col(1).array() += cmParas.smCenterPositionVertical(1);
			p.col(2).array() += cmParas.smCenterPositionVertical(2);

			file << "G1 X" << p(0, 0) << " Y" << p(0, 1) << " Z" << p(0, 2) + cmParas.SMShift
				<< " B" << cmParas.SMvertical_B << " C" << cmParas.zeroC
				<< " F" << cmParas.fastFeedRateLow << "\n";
			file << "G1 Z" << p(0, 2) << " F" << cmParas.SMFeedRate << "\n";
			break;
		case 1000: {
			Eigen::MatrixXd pTmp = p;
			pTmp.col(0) = -1 * p.col(1);
			pTmp.col(1) = p.col(0);
			p = pTmp;
			p.col(0).array() += cmParas.smCenterPositionHorizontal(0);
			p.col(1).array() += cmParas.smCenterPositionHorizontal(1);
			p.col(2).array() += cmParas.smCenterPositionHorizontal(2);

			file << "G1 X" << p(0, 0) << " Y" << p(0, 1) - cmParas.SMShift << " Z" << p(0, 2)
				<< " B" << cmParas.SMhorizontal_B << " C" << cmParas.zeroC - 90
				<< " F" << cmParas.fastFeedRateLow << "\n";
			file << "G1 Y" << p(0, 1) << " F" << cmParas.SMFeedRate << "\n";
			break;
		}
		case 100: {
			Eigen::MatrixXd pTmp = p;
			pTmp.col(0) = p.col(1);
			pTmp.col(1) = -1 * p.col(0);
			p = pTmp;
			p.col(0).array() += cmParas.smCenterPositionHorizontal(0);
			p.col(1).array() += cmParas.smCenterPositionHorizontal(1);
			p.col(2).array() += cmParas.smCenterPositionHorizontal(2);

			file << "G1 X" << p(0, 0) << " Y" << p(0, 1) - cmParas.SMShift << " Z" << p(0, 2)
				<< " B" << cmParas.SMhorizontal_B << " C" << cmParas.zeroC + 90
				<< " F" << cmParas.fastFeedRateLow << "\n";
			file << "G1 Y" << p(0, 1) << " F" << cmParas.SMFeedRate << "\n";
			break;
		}
		case 10:
			p.col(0).array() += cmParas.smCenterPositionHorizontal(0);
			p.col(1).array() += cmParas.smCenterPositionHorizontal(1);
			p.col(2).array() += cmParas.smCenterPositionHorizontal(2);

			file << "G1 X" << p(0, 0) << " Y" << p(0, 1) - cmParas.SMShift << " Z" << p(0, 2)
				<< " B" << cmParas.SMhorizontal_B << " C" << cmParas.zeroC
				<< " F" << cmParas.fastFeedRateLow << "\n";
			file << "G1 Y" << p(0, 1) << " F" << cmParas.SMFeedRate << "\n";
			break;
		case 1: {
			Eigen::MatrixXd pTmp = p;
			pTmp.col(0) = -1 * p.col(0);
			pTmp.col(1) = -1 * p.col(1);
			p = pTmp;
			p.col(0).array() += cmParas.smCenterPositionHorizontal(0);
			p.col(1).array() += cmParas.smCenterPositionHorizontal(1);
			p.col(2).array() += cmParas.smCenterPositionHorizontal(2);

			file << "G1 X" << p(0, 0) << " Y" << p(0, 1) - cmParas.SMShift << " Z" << p(0, 2)
				<< " B" << cmParas.SMhorizontal_B << " C" << cmParas.zeroC + 180
				<< " F" << cmParas.fastFeedRateLow << "\n";
			file << "G1 Y" << p(0, 1) << " F" << cmParas.SMFeedRate << "\n";
			break;
		}
		default:
			std::cout << "Error: opeType is not valid" << std::endl;
			return;
		}


		Eigen::Vector3d directionNow = p.row(1) - p.row(0);
		directionNow = directionNow / directionNow.norm();

		for (int j = 1; j < p.rows(); j++) {
			if (j == p.rows() - 1) {
				file << "G1 X" << p(j, 0) << " Y" << p(j, 1) << " Z" << p(j, 2)
					<< " F" << feedRate << "\n";
				break;
			}

			Eigen::Vector3d directionNext = p.row(j + 1) - p.row(j);
			directionNext = directionNext / directionNext.norm();

			if (directionNow.dot(directionNext) < 0.01) {
				file << "G1 X" << p(j, 0) << " Y" << p(j, 1) << " Z" << p(j, 2)
					<< " F" << feedRate << "\n";
			}

			directionNow = directionNext;
		}

		switch (opeType) {
		case 100000:
			file << "M9000 E-7 F900\n";
			file << "G4 P1000\n";
			file << "G1 Z" << p(p.rows() - 1, 2) + cmParas.AMShift << " F" << cmParas.fastFeedRateLow << "\n";
			break;
		case 10000:
			file << "G1 Z" << p(p.rows() - 1, 2) + cmParas.SMShift << " F" << cmParas.fastFeedRateLow << "\n";
			break;
		case 1000:
		case 100:
		case 10:
		case 1:
			file << "G1 Y" << p(p.rows() - 1, 1) - cmParas.SMShift << " F" << cmParas.fastFeedRateLow << "\n";
			break;
		default:
			std::cout << "Error: opeType is not valid" << std::endl;
			return;
		}
		file << "\n";
	}

	file.close();
}

void fileIO::writeVoxelGroup(const std::vector<Eigen::Vector3i>& voxelGroup, int opeType, std::string filename){
	std::ofstream file(filename);
	if (!file.is_open()) {
		std::cerr << "Failed to open file: " << filename << std::endl;
		return;
	}

	file << opeType << std::endl;
	for (int i = 0; i < voxelGroup.size(); i++) {
		file << voxelGroup[i](0) << " " << voxelGroup[i](1) << " " << voxelGroup[i](2) << std::endl;
	}
	file.close();
}

void fileIO::writeAccAll(const string& filename, const vector<vector<pair<int, int>>>& data) {
    ofstream outFile(filename, ios::binary);
    if (!outFile) {
        cerr << "Failed to Open: " << filename << endl;
        return;
    }
    size_t outerSize = data.size();
    outFile.write(reinterpret_cast<const char*>(&outerSize), sizeof(outerSize));
    for (const auto& innerVec : data) {
        size_t innerSize = innerVec.size();
        outFile.write(reinterpret_cast<const char*>(&innerSize), sizeof(innerSize));
        outFile.write(reinterpret_cast<const char*>(innerVec.data()), innerSize * sizeof(pair<int, int>));
    }
    outFile.close();
}

vector<vector<pair<int, int>>> fileIO::readAccAll(const string& filename) {
	ifstream inFile(filename, ios::binary);
	if (!inFile) {
		cerr << "Failed to Open: " << filename << endl;
		return {};
	}
	size_t outerSize;
	inFile.read(reinterpret_cast<char*>(&outerSize), sizeof(outerSize));
	vector<vector<pair<int, int>>> data(outerSize);
	for (auto& innerVec : data) {
		size_t innerSize;
		inFile.read(reinterpret_cast<char*>(&innerSize), sizeof(innerSize));
		innerVec.resize(innerSize);
		inFile.read(reinterpret_cast<char*>(innerVec.data()), innerSize * sizeof(pair<int, int>));
	}
	inFile.close();
	return data;
}

void fileIO::writeEroAll(const string& filename, const vector<vector<int>>& data) {
	ofstream outFile(filename, ios::binary);
	if (!outFile) {
		cerr << "Failed to Open: " << filename << endl;
		return;
	}
	size_t outerSize = data.size();
	outFile.write(reinterpret_cast<const char*>(&outerSize), sizeof(outerSize));
	for (const auto& innerVec : data) {
		size_t innerSize = innerVec.size();
		outFile.write(reinterpret_cast<const char*>(&innerSize), sizeof(innerSize));
		outFile.write(reinterpret_cast<const char*>(innerVec.data()), innerSize * sizeof(int));
	}
	outFile.close();
}

vector<vector<int>> fileIO::readEroAll(const string& filename) {
	ifstream inFile(filename, ios::binary);
	if (!inFile) {
		cerr << "Failed to Open: " << filename << endl;
		return {};
	}
	size_t outerSize;
	inFile.read(reinterpret_cast<char*>(&outerSize), sizeof(outerSize));
	vector<vector<int>> data(outerSize);
	for (auto& innerVec : data) {
		size_t innerSize;
		inFile.read(reinterpret_cast<char*>(&innerSize), sizeof(innerSize));
		innerVec.resize(innerSize);
		inFile.read(reinterpret_cast<char*>(innerVec.data()), innerSize * sizeof(int));
	}
	inFile.close();
	return data;
}

void fileIO::writeModel(std::string filename, Eigen::Vector3i size, Eigen::VectorXi matrixValue) {
	std::ofstream file(filename);
	if (!file.is_open()) {
		std::cerr << "Failed to open file: " << filename << std::endl;
		return;
	}

	file << size(0) << "," << size(1) << "," << size(2) << std::endl;
	file << 0 << "," << 0 << "," << 0 << std::endl;
	for (int i = 0; i < matrixValue.size(); i++) {
		file << matrixValue(i);
		if (i < matrixValue.size() - 1) {
			file << ",";
		}
	}
	file << std::endl;

	file.close();
}