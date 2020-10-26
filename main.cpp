#include "mainwindow.h"
#include <iostream>

#include <QApplication>
#include <QTest>
#include <QDateTime>
#include <ctime>
#include <cstdlib>

using namespace cv;
using namespace std;

// Найти уравнение прямой
void line_equation(Point src1, Point src2, double& k, int& b) {
    if ((src1.x != src2.x) && (src1.y != src2.y)) {
        k = (double) (src2.y - src1.y) / (double) (src2.x - src1.x);
        b = src1.y - (k * src1.x);
    } else {
        k = 0;
    }
}

void find_perp(Point src, double src_k, int src_b, Point& dst) {
    dst.x = (src.x + src_k * src.y - src_k * src_b) / (src_k * src_k + 1);
    dst.y = src_k * dst.x + src_b;
}

void find_parallel(Point src, vector<Point>& dst, double k, int rows) {
    int b = src.y - k * src.x;
    dst.push_back(Point(-b/k, 0));
    dst.push_back(Point((rows - b) / k, rows));
}

// Найти точки пересечения прямой и контура
void line_interseption(vector<Point> src_line, Point p, Point q, vector<Point>& dst_point) {
    int d = 0;
    bool first_point_flag = false;
    bool second_point_flag = false;
    for(size_t i = 1; i < src_line.size(); i++) {
        d = abs((p.y - q.y)*src_line[i].x - (p.x - q.x)*src_line[i].y + p.x * q.y - p.y * q.x)
                /sqrt((p.y - q.y) * (p.y - q.y) + (p.x - q.x) * (p.x - q.x));
        if(p.x != q.x) {
            if((d == 0) && !first_point_flag && (p.x > src_line[i].x)) {
                dst_point.push_back(src_line[i]);
                first_point_flag = true;
            }
            if((d == 0) && !second_point_flag && (p.x < src_line[i].x)) {
                dst_point.push_back(src_line[i]);
                second_point_flag = true;
            }
        } else {
            if((d == 0) && !first_point_flag && (p.y > src_line[i].y)) {
                dst_point.push_back(src_line[i]);
                first_point_flag = true;
            }
            if((d == 0) && !second_point_flag && (p.y < src_line[i].y)) {
                dst_point.push_back(src_line[i]);
                second_point_flag = true;
            }
        }
    }
}

// Найти точку пересечения прямых
void intersection_points(Point flp1, Point flp2, Point slp1, Point slp2, Point& dst_point) {
    double k1 = (double)(flp2.y - flp1.y) / (flp2.x - flp1.x);
    double k2 = (double)(slp2.y - slp1.y) / (slp2.x - slp1.x);
    double b1 = k1 * flp1.x - flp1.y;
    double b2 = k2 * slp1.x - slp1.y;
    dst_point.x = -(b1 - b2) / (k2 - k1);
    dst_point.y = -(k2 * b1 - k1 * b2) / (k2 - k1);
}


// Найти максимально удаленную точку от прямой
void find_max(vector<Point> sepCont, Point p, Point q, Point &dst) {
    dst = sepCont[0];
    int d = 0;
    int d_max = abs((p.y - q.y)*sepCont[0].x - (p.x - q.x)*sepCont[0].y + p.x * q.y - p.y * q.x)
            /sqrt((p.y - q.y) * (p.y - q.y) + (p.x - q.x) * (p.x - q.x));
    for(size_t i = 1; i < sepCont.size(); i++) {
        d = abs((p.y - q.y)*sepCont[i].x - (p.x - q.x)*sepCont[i].y + p.x * q.y - p.y * q.x)
                /sqrt((p.y - q.y) * (p.y - q.y) + (p.x - q.x) * (p.x - q.x));
        if(d > d_max) {
            dst = sepCont[i];
            d_max = d;
        }
    }
}

// Найти расстаяние между двумя точками
int find_dist(Point p1, Point p2) {
    return sqrt((p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y));
}

// Найти точку на прямой
void point_on_line(Point src1, Point src2, double k, int b, Point& dst) {
    double dif = 0;
    if(src1.y - src2.y != 0) {
        dif = (double)abs(src1.x - src2.x) / (double)abs(src1.y - src2.y);
    } else {
        dif = 2;
    }
    int dist = find_dist(src1, src2);
    if(dif > 1) {
        if(src1.x > src2.x) {
            dst.y = k * (src1.x - dist/10) + b;
            dst.x = src1.x - dist/10;
        } else {
            dst.y = k * (src1.x + dist/10) + b;
            dst.x = src1.x + dist/10;
        }
    } else {
        if(src1.y > src2.y) {
            dst.y = src1.y - dist/10;
            dst.x = (dst.y - b) / k;
        } else {
            dst.y = src1.y + dist/10;
            dst.x = (dst.y - b) / k;
        }
    }
}

void point_on_line(vector<Point> src, double k, int b, vector<Point>& dst) {
    double dif = 0;
    if(src[0].y - src[1].y != 0) {
        dif = (double)abs(src[0].x - src[1].x) / (double)abs(src[0].y - src[1].y);
    } else {
        dif = 2;
    }
    int dist = find_dist(src[0], src[1]);
    if(dif > 1) {
        if(src[0].x > src[1].x) {
            dst[0].y = k * (src[0].x - dist/10) + b;
            dst[0].x = src[0].x - dist/10;
            dst[1].y = k * (src[1].x + dist/10) + b;
            dst[1].x = src[1].x + dist/10;
        } else {
            dst[0].y = k * (src[0].x + dist/10) + b;
            dst[0].x = src[0].x + dist/10;
            dst[1].y = k * (src[1].x - dist/10) + b;
            dst[1].x = src[1].x - dist/10;
        }
    } else {
        if(src[0].y > src[1].y) {
            dst[0].y = src[0].y - dist/10;
            dst[0].x = (dst[0].y - b) / k;
            dst[1].y = src[1].y + dist/10;
            dst[1].x = (dst[1].y - b) / k;
        } else {
            dst[0].y = src[0].y + dist/10;
            dst[0].x = (dst[0].y - b) / k;
            dst[1].y = src[1].y - dist/10;
            dst[1].x = (dst[1].y - b) / k;
        }
    }
}

// Извлечь прямоугольник по 4 точкам
void extractRect(Mat src, Mat dst, Point p1, Point p2, Point p3, Point p4) {
    int x_min = p1.x;
    int x_max = p1.x;
    int y_min = p1.y;
    int y_max = p1.y;


}

void separeteContour(vector<vector<Point>> src, vector<Point>& top, vector<Point>& low, double k, int b) {
    for(size_t i = 0; i < src.size(); i++) {
        for(size_t j = 0; j < src[i].size(); j++) {
            int y_cont = k * src[i][j].x + b;
            if(src[i][j].y < y_cont) {
                low.push_back(src[i][j]);
            } else {
                top.push_back(src[i][j]);
            }
        }
    }
}

string type2str(int type) {
    string r;

    uchar depth = type & CV_MAT_DEPTH_MASK;
    uchar chans = 1 + (type >> CV_CN_SHIFT);

    switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
    }

    r += "C";
    r += (chans+'0');

    return r;
}

int main()
{
    VideoCapture cap("C:/Users/Roman/Desktop/vid1.avi");
    Mat frame;
    Mat bg = imread("C:/Users/Roman/Desktop/bg1.png", IMREAD_UNCHANGED);
    Mat preform;
    Mat bin;
    Mat blured;
    Mat ROI;
    Moments mnt;
    vector< vector<Point> > contours;
    vector<Vec3f> circles;
    Point circle_center, peak;

    double k_main, k_par;
    int b_main, b1, b2, b3, b4;

    cap >> frame;

    resize(frame, frame, Size(), 0.4, 0.4);
    resize(bg, bg, Size(), 0.4, 0.4);

    int first_th_line = frame.rows * 0.3;
    int second_th_line = frame.rows * 0.7;
    int thresh_val = 10;
    int min_thresh_area = 12000;
    int max_thresh_area = 13300;
    int par1 = 10;
    int par2 = 20;
    int my_time = 1000;
    int cont_count = 0;

    namedWindow("Paramiters");
    createTrackbar("Timer", "Paramiters", &my_time, 10000);
    createTrackbar("Threshold", "Paramiters", &thresh_val, 255);
    createTrackbar("first line", "Paramiters", &first_th_line, frame.rows/2-10);
    createTrackbar("second line", "Paramiters", &second_th_line, frame.rows);
    createTrackbar("Min area", "Paramiters", &min_thresh_area, 20000);
    createTrackbar("Max area", "Paramiters", &max_thresh_area, 20000);
    createTrackbar("param1", "Paramiters", &par1, 100);
    createTrackbar("param2", "Paramiters", &par2, 100);

    qDebug("fg size: %d. bg size: %d", frame.size, bg.size);

    QTime t;
    t.start();

    while(true) {
        cap >> frame;
        if(!frame.empty()) {
            ROI = frame;
            resize(ROI, ROI, Size(), 0.4, 0.4);
            absdiff(ROI, bg, preform);

            line(ROI, Point(0, first_th_line), Point(ROI.cols, first_th_line), Scalar(255, 0, 0), 2);
            line(ROI, Point(0, ROI.rows/2), Point(ROI.cols, ROI.rows/2), Scalar(0, 0, 255), 2);
            line(ROI, Point(0, second_th_line), Point(ROI.cols, second_th_line), Scalar(0, 255, 0), 2);

            cvtColor(preform, preform, COLOR_BGR2GRAY);
            threshold(preform, bin, thresh_val, 255, THRESH_BINARY);
            findContours(bin, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);

            if(!contours.empty()) {
                for(int i = 0; i < static_cast<int>(contours.size()); i++) {
                    mnt = moments(contours[i]);
                    Point center_mass = Point(mnt.m10/mnt.m00, mnt.m01/mnt.m00);

                    if((contourArea(contours[i]) > min_thresh_area) & (contourArea(contours[i]) < max_thresh_area)) {
                        if((center_mass.y > first_th_line) & (center_mass.y < second_th_line)) {
                            //QTest::qSleep(my_time);
                            GaussianBlur(bin, blured, Size(5, 5), 9, 9);
                            HoughCircles(blured, circles, HOUGH_GRADIENT, 1, ROI.rows/16, par1, par2, 25, 30);

                            for(size_t j = 0; j < circles.size(); j++) {
                                circle_center = Point(cvRound(circles[j][0]), cvRound(circles[j][1]));

                                // Обработка центральной части преформы

                                /* Проблема
                                 * Самоставление центра масс с его вписанной окружностью?
                                 */

                                line_equation(circle_center, center_mass, k_main, b_main);
                                if(k_main != 0) {
                                    k_par = -1/k_main;

                                    find_max(contours[i], circle_center, center_mass, peak);
                                    b1 = peak.y - k_par * peak.x;

                                    line(ROI, Point(-b_main/k_main, 0), Point((ROI.rows-b_main), ROI.rows), 0, 2);
                                    line(ROI, Point(-b1/k_par, 0), Point((ROI.rows-b1), ROI.rows), 0, 2);

                                }

                                int radius = cvRound(circles[j][2]);
                                circle(ROI, circle_center, 3, Scalar(0, 0, 255), 3);
                                circle(ROI, circle_center, radius, Scalar(0, 255, 255), 3, 8, 0);
                            }

                            circle(ROI, center_mass, 3, Scalar(255, 255, 0), -1);
                            drawContours(ROI, contours, i, Scalar(255, 100, 100), 2);
                        }
                        cont_count += contours.size();
                    }
                }
                circles.clear();
            }
            contours.clear();

            imshow("bin", bin);
            imshow("Frame", ROI);
        }
        else {
            break;
        }

        if(waitKey(25) == 27)
            break;
    }

    qDebug("Время выполнения программы = %d s. Кол-во найденных контуров: %d. Преформ прошло: %d", t.elapsed()/1000, cont_count, t.elapsed()/100); // время работы программы

    cap.release();
    destroyAllWindows();

    return EXIT_SUCCESS;
}
