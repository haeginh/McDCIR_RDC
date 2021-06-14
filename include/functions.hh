#ifndef FUNCTIONS_HH
#define FUNCTIONS_HH

#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <map>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/Geometry>
#include <igl/bbw.h>

using namespace std;
using namespace Eigen;

MatrixXd GenerateBonePoints(MatrixXd C, MatrixXi BE, double interval);
map<tuple<int,int,int>,vector<int>> GenerateGrid(MatrixXd V, double size=1.);
map<int, map<int, double>> GenerateBarycentricCoord(MatrixXd V_f, MatrixXi T_f, MatrixXd V);
void PrintBaryCoords(string file, map<int, map<int, double>> &baryCoords);
map<int, map<int, double>> ReadBaryFile(string file);
SparseMatrix<double> GenerateBarySparse(map<int, map<int, double>> &baryCoords, int v_size);
bool CalculateScalingWeights(MatrixXd &C, MatrixXd &V,MatrixXi &T,MatrixXd &W);
bool CalculateScalingWeights(MatrixXd &C, MatrixXd &V,MatrixXi &T,MatrixXd &W, vector<int> eyeLV, vector<int> eyeRV);
AngleAxisd GetRotMatrix(Vector3d from, Vector3d to);

typedef
  std::vector<Eigen::Quaterniond,Eigen::aligned_allocator<Eigen::Quaterniond> >
  RotationList;
void myDqs(
  const Eigen::MatrixXd & V,
  const std::vector<std::map<int, double>> & W,
  const RotationList & vQ,
  const std::vector<Vector3d> & vT,
  Eigen::MatrixXd & U);

#endif

