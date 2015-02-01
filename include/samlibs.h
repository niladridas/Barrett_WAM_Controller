/*
 * samlibs.h
 *
 *  Created on: 20-Aug-2014
 *      Author: mobman
 */

#ifndef SAMLIBS_H_
#define SAMLIBS_H_
#include <iostream>
#include <fstream>
#include <cstdio>
#include <algorithm>
#include <iterator>
#include <cstdlib> // for atof()
#include <list>
#include <map>
#include <sstream>
//#define debug
#include <math.h>
#include <cstring> // for strtok()

#include <vector>

#include <string>
#include <boost/accumulators/statistics/variance.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/bind.hpp>
#include <boost/regex.hpp>
#include <sys/types.h>
#include <dirent.h>
#include <errno.h>
#include </usr/include/eigen3/Eigen/Core>
namespace Sam{

  template <typename T>
  void disp(T s);
  typedef std::vector<double> vec_type;
  typedef std::vector<std::vector<double> > mat_type;
  template <typename T>
  void display(const std::vector<std::vector<T> >& dataTowrite,std::string str="" );
  template <typename T>
  void display(const std::vector<T>& dataTowrite, std::string str="");
  template <typename T>
  void display(T* const dataTowrite, size_t rows, size_t cols, std::string str="");
  template <typename T>
  void display(T** const dataTowrite,size_t rows, size_t cols, std::string str="" );
  template <typename T>
  std::vector<T> addAsRows(const std::vector<T> &vec1, const std::vector<T> &vec2);
  template <typename T>
  std::vector<std::vector<T> > addAsRows(const std::vector<T> &vec1, const std::vector<T> &vec2);
  template <typename T>
  std::vector<std::vector<T> > addAsCols(const std::vector<T> &vec1, const std::vector<T> &vec2);
  template <typename T>
  std::vector<std::vector<T> > addAsCols(const std::vector<T> &vec1, const std::vector<std::vector<T> > &mat2);
  template <typename T>
  std::vector<std::vector<T> > addAsCols(const std::vector<std::vector<T> >& mat1, const std::vector<T>& vec2);
  template <typename T>
  std::vector<std::vector<T> > addAsCols(const std::vector<std::vector<T> >& mat1, const std::vector<std::vector<T> >& mat2);
  template <typename T>
  inline std::vector<T> getCol(const std::vector<std::vector<T> >& mat, size_t colIndx);
  template <typename T>
  inline std::vector<T> getRow(const std::vector<std::vector<T> >& mat, size_t rowIndx);
  template <typename T1 >
  inline std::vector<T1> getBlock_vec(const std::vector<std::vector<T1> >& mat, size_t rowIndx, size_t colIndx, size_t blockLength_row, size_t blockLength_col);
  template <typename T>
  inline std::vector<std::vector<T> > getBlock_mat(std::vector<std::vector<T> >& mat, size_t rowIndx, size_t colIndx, size_t blockLength_row, size_t blockLength_col);
  template <typename T>
  void writeToFile(const std::string& filename, std::vector<std::vector<T> > dataTowrite, bool append_mode=false);
  template <typename T>
  bool writeTrainingData_FANNformat(const std::string& filename, const std::vector<std::vector<T> >& dataMat,  size_t num_inp, size_t num_out);
  inline void waitForEnter();
  // list files in a directory
  inline int listFiles (std::string dir, std::vector<std::string>& files, std::string ext);
  inline int listDir (std::string dir, std::vector<std::string>& dirs);
  template <typename T>
  std::vector <std::vector <T> > readFile(const std::string & fileName);
  inline bool HasSpecialCharacters(const char *str);
  /// For reading and retrieving configuration params. Configuration can be found in NN::save()
  inline bool readConfigFile(const std::string& configFile, std::map<std::string, std::string>& allData);
  // find value (value type vector) from the map
  template<typename T>
  bool findValueFromMap(std::map<std::string, std::string> map, std::string varName, std::vector<T>& var);
  // find value (value type vector) from the map
  template<typename T>
  bool findValueFromMap(std::map<std::string, std::string> map, const std::string& varName, T& var);
  inline void rewriteDataFiles(std::string ip_dir_path, std::string ext, size_t starting_indx, std::string fileName_part1, std::string fileName_part2, bool delFlag = false);
  template <typename T>
  void initStdVec (std::vector<T>& vec, T* arr, size_t len);
  template <typename T>
  void initStdMat (std::vector<std::vector<T> >& mat, T** arr, size_t rows, size_t cols);
  template <typename T>
  void initEigenMat (Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>& M, const std::vector<std::vector<T> >& mat);
  template <typename T>
  void initEigenVec (Eigen::Matrix<T, Eigen::Dynamic, 1>& V, const std::vector<T>& v);
  template <typename T>
  void initEigenVec (Eigen::Matrix<T, Eigen::Dynamic, 1>& V, const std::vector<std::vector<T> >& mat);
  template <typename T > int signOf(T val);
}
#include <Detail/samlibs-inl.h>


#endif /* SAMLIBS_H_ */
