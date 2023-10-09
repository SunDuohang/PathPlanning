//
// Created by SUNX on 2023/9/13.
//

#include "gdal.h"
#include "gdal_priv.h"
#include <iostream>
#include <fstream>
#include "../include/CoorTransform.h"

using namespace std;

#define FAILURE 0
#define SUCCESS 1

/**
 * @brief 获取tif文件的仿射矩阵
 * @param tifFileName tif文件名
 * @param GeoTrans 保存仿射矩阵信息
 * @return 0-failure 1-success
 */
int GetGeoTrans(const char* tifFileName, double* GeoTrans){
    // 注册所有功能
    GDALAllRegister();
    // 打开文件
    GDALDataset* hDS = (GDALDataset*) GDALOpen(tifFileName, GA_ReadOnly);
    if (hDS == nullptr){
        fprintf(stderr, "Can't Open tifFile");
        return FAILURE;
    }
    if(GDALGetGeoTransform(hDS, GeoTrans) != CE_None){
        fprintf(stderr, "Can't Get GeoTransform");
        return FAILURE;
    }
    //关闭文件
    GDALClose(hDS);
    return SUCCESS;
}

/**
* @brief 投影坐标转地理坐标
* @param tifFileName TIF数据文件名
* @param xCol 列坐标
* @param yRow 行坐标
* @return <double, double> 经纬度
*/
std::pair<double, double> Projection2Geo(const char* tifFileName, int xCol, int yRow)
{
    //注册所有功能
    GDALAllRegister();

    //打开文件
    GDALDataset* hDS = (GDALDataset*)GDALOpen(tifFileName, GA_ReadOnly);
    if (hDS == nullptr){
        fprintf(stderr, "Can't open %s\n", tifFileName);
        return make_pair(0.0, 0.0);
    }

    double GeoTrans[6] = {0, 1, 0, 0, 0, 1};
    if (GetGeoTrans(tifFileName, GeoTrans) == FAILURE) {
        return make_pair(0.0, 0.0);
    }
    double xLon = 0.0, yLat = 0.0;
    int x = xCol, y = yRow;
    ImageRowCol2Geo(GeoTrans, x, y, xLon, yLat);

    //关闭文件
    GDALClose(hDS);
    return make_pair(xLon, yLat);
}

/**
* @brief 投影坐标转地理坐标
* @param tifFileName TIF数据文件名
* @param lon 经度坐标
* @param lat 纬度坐标
*/
std::pair<int, int> Geo2Projection(const char* tifFileName, double lon, double lat)
{
    double GeoTrans[6] = {0,1,0,0,0,1};
    if(GetGeoTrans(tifFileName, GeoTrans) == FAILURE){
        return make_pair(0, 0);
    }

    double dLon = lon, dLat = lat;
    int dx = 0, dy = 0;
    Geo2ImageRowCol(GeoTrans, dLon, dLat, dx, dy);

    return make_pair(dx, dy);
}

/**
* @brief 将地理坐标转为投影坐标，通过地址将值返回
* @param adfGeoTransform 仿射矩阵
* @param iCol 投影坐标x
* @param iRow 投影坐标y
* @param XGeo 地理坐标lon
* @param YGeo 地理坐标lat
*/
bool Geo2ImageRowCol(double* adfGeoTransform, double XGeo, double YGeo, int& iRow, int& iCol){
    try
    {
        double dTemp = adfGeoTransform[1] * adfGeoTransform[5] - adfGeoTransform[2] * adfGeoTransform[4];
        double dCol = 0.0, dRow = 0.0;
        dCol = (adfGeoTransform[5] * (XGeo - adfGeoTransform[0]) -
                adfGeoTransform[2] * (YGeo - adfGeoTransform[3])) / dTemp + 0.5;
        dRow = (adfGeoTransform[1] * (YGeo - adfGeoTransform[3]) -
                adfGeoTransform[4] * (XGeo - adfGeoTransform[0])) / dTemp + 0.5;
        iCol = int(dCol);
        iRow = int(dRow);
        return true;
    }
    catch (...)
    {
        return false;
    }
}

/**
* @brief 将投影坐标转为地理坐标，通过地址将值返回
* @param adfGeoTransform 仿射矩阵
* @param iCol 投影坐标x [列号]
* @param iRow 投影坐标y [行号]
* @param XGeo 地理坐标lon
* @param YGeo 地理坐标lat
*/
bool ImageRowCol2Geo(double* adfGeoTransform, int iCol, int iRow, double& XGeo, double& YGeo){
    try
    {
        XGeo = adfGeoTransform[0] + adfGeoTransform[1] * iCol + adfGeoTransform[2] * iRow;
        YGeo = adfGeoTransform[3] + adfGeoTransform[4] * iCol + adfGeoTransform[5] * iRow;
        return true;
    }
    catch (...)
    {
        return false;
    }
}

