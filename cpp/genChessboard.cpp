#include <iostream>
#include <opencv/cv.hpp>

using namespace std;
using namespace cv;

int main(){

    //单位转换
    int dot_per_inch = 96;      //DPI,每英寸上像素个数
    double inch_to_cm = 2.54;   //1inch = 2.54cm

    //自定义标定板
    double blockSize_cm = 2; //方格尺寸：边长1.3cm的正方形
    int blockNum = 8; //8*8个方格


    int blockSize = (int)(blockSize_cm /inch_to_cm *dot_per_inch);
    cout << "Block Reslution:"<<blockSize<<"*"<<blockSize<< endl;

    int imageSize = blockSize * blockNum;
    cout <<"Image Reslution:"<< imageSize<<"*"<<imageSize<< endl;
    cout << "Image Size(cm):"<<blockSize_cm*blockNum<<"*"<<blockSize_cm*blockNum<< endl;
    Mat chessBoard(imageSize, imageSize, CV_8UC3, Scalar::all(0));
    unsigned char color = 0;

    for (int i = 0; i < imageSize; i = i + blockSize){
        color = ~color;
        for (int j = 0; j < imageSize; j = j + blockSize){
            Mat ROI = chessBoard(Rect(i, j, blockSize, blockSize));
            ROI.setTo(Scalar::all(color));
            color = ~color;
        }
    }
    imshow("Chess board", chessBoard);
    imwrite("chessBoard.png",chessBoard);
    waitKey(0);
    return 0;
}
