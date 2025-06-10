#pragma once
#include "../QMeshLib/PolygenMesh.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <Eigen/Sparse>
#include <filesystem>
#include <iomanip>
#include <filesystem>

#include <sys/stat.h>
#include <sys/types.h>

#ifdef _WIN32
#include <direct.h>
#define MKDIR(path) _mkdir(path)
#else
#include <unistd.h>
#define MKDIR(path) mkdir(path, 0777)
#endif

using namespace std;

struct commandParas {
	double AMFeedRate = 1500;
	double SMFeedRate = 900;
	double fastFeedRate = 20000;
	double fastFeedRateLow = 20000;

	Eigen::Vector3d amCenterPosition = Eigen::Vector3d(194.9, 224.61, 91.01);
	//Eigen::Vector3d smCenterPositionVertical = Eigen::Vector3d(111.2, 301.15, -40.472);
	Eigen::Vector3d smCenterPositionVertical = Eigen::Vector3d(111.2, 301.15, -40.955);
	//Eigen::Vector3d smCenterPositionHorizontal = Eigen::Vector3d(112.83, 235.032, -109.05);
	Eigen::Vector3d smCenterPositionHorizontal = Eigen::Vector3d(112.83, 235.515, -109.05);

	double zeroC = 21.613;
	double AM_B = 47.755;
	double SMvertical_B = -42.245;
	double SMhorizontal_B = -132.245;
	Eigen::Vector3d amHome = Eigen::Vector3d(156, 216, 170);
	Eigen::Vector3d smHomeVertical = Eigen::Vector3d(106, 304, 40);
	Eigen::Vector3d smHomeHorizontal = Eigen::Vector3d(106, 120, 40);
	double AMShift = 6;
	double SMShift = 16;
};


class fileIO
{
public:
	fileIO(){};
	~fileIO(){};

	Eigen::MatrixXd readTXTfile(std::string filename); // read a txt file
	Eigen::SparseMatrix<int> readModelMatrix(std::string filename); // read a model matrix from a txt file

	void writeMatrix(Eigen::MatrixXd matrix, std::string filename, int precision); // write a matrix to a txt file
	void writeMatrix(Eigen::MatrixXi matrix, std::string filename); // write a matrix to a txt file
	Eigen::MatrixXd readMatrix(std::string filename); // read a matrix from a txt file
	Eigen::MatrixXi readMatrixInt(std::string filename); // read a matrix from a txt file

	void writeVector(Eigen::VectorXi vector, std::string filename); // write a vector to a txt file
	void readVector(std::string filename, Eigen::VectorXi& vector); // read a vector from a txt file

	void writeString(std::string str, std::string filename); // write a string to a txt file

	void writeSparseMatrix(const Eigen::SparseMatrix<int>& matrix, std::string filename, bool size); // write a sparse matrix to a txt file
	Eigen::SparseMatrix<int> readSparseMatrix(std::string filename); // read a sparse matrix from a txt file

	void writeHMP(std::vector<std::vector<int>> Time, int tLast, std::string filename); // write a HMP struct to a txt file
	void readHMP(std::string filename, std::vector<std::vector<int>>& Time, int& tLast); // read a HMP struct from a txt file

	void writeAccEroElement(std::vector<std::vector<int>> Element, std::string filename); // write Acc and Ero elements to a txt file
	std::vector<std::vector<int>> readAccEroElement(std::string filename); // read Acc and Ero elements from a txt file

	void clearFolder(std::string folder); // clear all files in a folder

	void readHMP3D(std::string filename, Eigen::MatrixXi& T, int& tLast); // read a HMP struct from a txt file
	void readHMP3D(std::string filename, Eigen::MatrixXd& T, double& tLast); // read a HMP struct from a txt file
	void writeHMP3D(Eigen::MatrixXd T, double tLast, std::string filename); // write a HMP struct to a txt file

	void writePathGroup(std::vector<Eigen::MatrixXd> path, int opeType, std::string filename); // write a path group to a txt file
	void writeVoxelGroup(const std::vector<Eigen::Vector3i>& voxelGroup, int opeType, std::string filename); // write a voxel group to a txt file

private:
	Eigen::MatrixXd rot90(Eigen::MatrixXd matrix, int k); // rotate a matrix by 90 degrees k times counter-clockwise
	
public:
	void read3DMatrix(std::string filename, Eigen::Vector3i& size, Eigen::VectorXi& matrixValue, Eigen::VectorXi& outerStatus);
	Eigen::MatrixXi read3DTrainingModel(std::string filename, Eigen::Vector3i& size);
	void writeModel(std::string filename, Eigen::Vector3i size, Eigen::VectorXi matrixValue);

public:
	void saveVectorXd(const Eigen::VectorXd& vec, const std::string& filename);
	void saveStdVector(const std::vector<double>& vec, const std::string& filename);
	Eigen::VectorXd loadVectorXd(const std::string& filename);
	std::vector<double> loadStdVector(const std::string& filename);
	void saveSparseMatrixCompressed(const Eigen::SparseMatrix<double>& mat, const std::string& filename);
	Eigen::SparseMatrix<double> loadSparseMatrixCompressed(const std::string& filename);
	void saveSparseVector(const Eigen::SparseVector<double>& vec, const std::string& filename);
	Eigen::SparseVector<double> loadSparseVector(const std::string& filename);
	void writeBoolToBinary(const std::string& filename, bool value);
	bool readBoolFromBinary(const std::string& filename);

public:
	bool directoryExists(const std::string& path);
	bool createDirectory(const std::string& path);

	void startCommandFile(std::string filePath);
	void endCommandFile(std::string filePath, int lastPos);
	void changeToolCommand(std::string filePath, int lastPos, int currentPos, commandParas cmParas);
	void moveCommand(std::string filePath, std::vector<Eigen::MatrixXd> path, int opeType, commandParas cmParas, double cuttingArea);

	void writeAccAll(const string& filename, const vector<vector<pair<int, int>>>& data);
	vector<vector<pair<int, int>>> readAccAll(const string& filename);
	void writeEroAll(const string& filename, const vector<vector<int>>& data);
	vector<vector<int>> readEroAll(const string& filename);
};