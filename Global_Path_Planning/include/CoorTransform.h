//
// Created by SUNX on 2023/9/13.
//

#ifndef GLOBAL_PATH_PLANNING_COORTRANSFORM_H
#define GLOBAL_PATH_PLANNING_COORTRANSFORM_H

#include <vector>
#include <algorithm>
#include <iostream>
#include <math.h>

int GetGeoTrans(const char* tifFileName, double* GeoTrans);
std::pair<double, double> Projection2Geo(const char* tifFileName, int x_idx, int y_idx);
std::pair<int, int> Geo2Projection(const char* tifFileName, double lon, double lat);
bool Geo2ImageRowCol(double* adfGeoTransform, double XGeo, double YGeo, int& iRow, int& iCol);
bool ImageRowCol2Geo(double* adfGeoTransform, int iCol, int iRow, double& XGeo, double& YGeo);



#endif //GLOBAL_PATH_PLANNING_COORTRANSFORM_H
