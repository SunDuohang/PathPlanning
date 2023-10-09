#include <iostream>

#include <iostream>
#include "stdio.h"
#include <gdal.h>
#include <gdal_priv.h>
#include "./include/CoorTransform.h"

using namespace std;

int main(void) {
    std::cout << "Hello, World!" << std::endl;

    GDALDataset* poDataset;
    GDALAllRegister();
    poDataset = (GDALDataset*)GDALOpen("D:\\2.5m-tongxing.tif", GA_ReadOnly);
    if (poDataset == NULL)
    {
        cout << "文件打开失败!!!" << endl;
    }
    else {
        cout << "文件打开成功!!!" << endl;
    }
    int nBandCount = poDataset->GetRasterCount();
    int nImgSizeX = poDataset->GetRasterXSize();
    int nImgSizeY = poDataset->GetRasterYSize();

    const char* filename = "D:\\2.5m-tongxing.tif";
    double lon = 121.44437847812613;
    double lat = 25.129042069908703;

    int iCol = 0;
    int iRow = 0;
    std::tie(iCol, iRow)  = Geo2Projection(filename, lon, lat);
    cout << "nBandCount:" << nBandCount << endl << "nImgSizeX:" << nImgSizeX << endl << "nImgSizeY:" << nImgSizeY << endl;
    cout << "Col:" << iCol << "\nRow:" << iRow << endl;

    int yRow = 9979;
    int xCol = 16249;
    lon = 0.0;
    lat = 0.0;
    std::tie(lon, lat) = Projection2Geo(filename, xCol, yRow);
    cout << "lon:" << lon << "\nlat:" << lat << endl;
    GDALClose(poDataset);
    return 0;
}
