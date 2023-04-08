#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

#define MIN_AREA 350 //����� �� ��������� ������ �������

double detectAngle(int xB, int xR, int yB, int yR)
{
    if (xB == 0 || yB == 0 || xR == 0 || yR == 0)
        return 0;
    if (xR == xB && yB < yR)
        return 180;
    if (yB - yR == 0)
        return 90;
    //��������� ����, ���� ���� ���� ������
    double t = (xB * xR + yB * yR) / (sqrt((double)xB * xB + yB * yB) * sqrt((double)xR * xR + yR * yR));
    if (t < -1) t = -1;
    else if (t > 1) t = 1;
    return acos(t);
}
Scalar convert_hsv(Scalar hsv)
{
    // �������� H S V ��������
    int h = hsv[0];
    int s = hsv[1];
    int v = hsv[2];
    // ��������� H �� ������� [0, 360] � [2, 180]
    h = static_cast<int>(h * 180 / 360);
    // ��������� S � V �� ������� [0, 100] � [0, 255]
    s = static_cast<int>(s * 255 / 100);
    v = static_cast<int>(v * 255 / 100);
    // ���������� HSV
    return Scalar(h, s, v);
}

int main() {

    //�������� ������ � ����������� � ������
    VideoCapture cap(0);

    //� ������ ������ ��� �������� ����������� � ������ ���������� ������
    if (!cap.isOpened()) {
        cout << "Error opening video stream or file" << endl;
        return -1;
    }


    while (1) {

        Mat frame;
        //��������� ������ � ���������� (������)
        cap >> frame;

        //��������� ������
        if (frame.empty())
            break;

        Mat hsv;
        cvtColor(frame, hsv, COLOR_BGR2HSV);//�������������� ����������� ����������� � �������� ������������ hsv
        Scalar L_Blue = convert_hsv(Scalar(110, 50, 50));//������ ������� ����������� �����
        Scalar U_Blue = convert_hsv(Scalar(260, 100, 100));//������� ������� �����

        Scalar L_Red = convert_hsv(Scalar(300, 30, 30));
        Scalar U_Red = convert_hsv(Scalar(340, 100, 100));

        Scalar L_Green = convert_hsv(Scalar(50, 40, 40));
        Scalar U_Green = convert_hsv(Scalar(180, 100, 100));

        Mat Bmask, Rmask, Gmask;

        inRange(hsv, L_Blue, U_Blue, Bmask);//��������� ������� ����� ����� ��������� ���������
        inRange(hsv, L_Red, U_Red, Rmask);
        inRange(hsv, L_Green, U_Green, Gmask);

        Mat mask = Bmask | Rmask | Gmask;
        Mat result;

        bitwise_and(frame, frame, result, mask);//�������� �������� ����� ���������� �����

        int xB = 0;
        int yB = 0;
        int xR = 0;
        int yR = 0;
        int xG = 0;
        int yG = 0;

        //������� ������ ������� ��������� �����
        vector<vector<Point>> contoursB;
        findContours(Bmask, contoursB, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        //���� ������ ���� �� ���� ������
        if (contoursB.size() > 0)
        {
            auto contourMax = max_element(contoursB.begin(), contoursB.end(), [](auto& a, auto& b)
                {
                    return contourArea(a) < contourArea(b);
                });
            if (contourArea(*contourMax) > MIN_AREA)//���� ������ ������ ����������� �������
            {
                Moments M = moments(*contourMax);//���� ����� ����
                if (M.m00 != 0)
                {
                    xB = static_cast<int>(M.m10 / M.m00);
                    yB = static_cast<int>(M.m01 / M.m00);
                    //drawContours(result, contoursB, -1, (255, 0, 0), 2);//������������ ������
                    circle(result, Point(xB, yB), 7, (0, 255, 255), -1);//������ ���� � ����� ������ ���� (����� ������)
                }
            }
        }

        vector<vector<Point>> contoursR;
        findContours(Rmask, contoursR, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        //���� ������ ���� �� ���� ������
        if (contoursR.size() > 0)
        {
            auto contourMax = max_element(contoursR.begin(), contoursR.end(), [](auto& a, auto& b)
                {
                    return contourArea(a) < contourArea(b);
                });
            if (contourArea(*contourMax) > MIN_AREA)//���� ������ ������ ����������� �������
            {
                Moments M = moments(*contourMax);//���� ����� ����
                if (M.m00 != 0)
                {
                    xR = static_cast<int>(M.m10 / M.m00);
                    yR = static_cast<int>(M.m01 / M.m00);
                    //drawContours(result, contoursR, -1, (0, 0, 255), 2);//������������ ������
                    circle(result, Point(xR, yR), 7, (255, 255, 0), -1);//������ ���� � ����� ������ ���� (������� ������)
                }
            }
        }

        vector<vector<Point>> contoursG;
        findContours(Gmask, contoursG, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        //���� ������ ���� �� ���� ������
        if (contoursG.size() > 0)
        {
            auto contourMax = max_element(contoursG.begin(), contoursG.end(), [](auto& a, auto& b)
                {
                    return contourArea(a) < contourArea(b);
                });
            if (contourArea(*contourMax) > MIN_AREA)//���� ������ ������ ����������� �������
            {
                Moments M = moments(*contourMax);//���� ����� ����
                if (M.m00 != 0)
                {
                    xG = static_cast<int>(M.m10 / M.m00);
                    yG = static_cast<int>(M.m01 / M.m00);
                    //drawContours(result, contoursR, -1, (0, 0, 255), 2);//������������ ������
                    circle(result, Point(xG, yG), 7, (0, 0, 255), -1);//������ ���� � ����� ������ ����
                }
            }
        }

        if (contoursB.size() > 0 && contoursR.size() > 0)
        {
            // �����, �����������  ����� ���� ������ ������ (�����) � ������
            line(result, Point(xB, yB), Point(xG, yG), Scalar(256, 256, 256), 1);

            // �����, ����������� ������ ����
            line(result, Point(xR, yR), Point(xB, yB), Scalar(256, 256, 256), 1);
            //���� ����� ������� 
            double angle = detectAngle(xB, xG, yB, yG);
            //���� �� ����� ��������
            putText(result, to_string(angle), Point(xB, yB), FONT_HERSHEY_SIMPLEX, 1, Scalar(256, 256, 256), 2);
        }

        imshow("Original", frame);//������� ����������� � ������
        imshow("Detector", result);//������� ����������� � ������

        //��� ���������� ������ �������� ESC
        char c = (char)waitKey(25);
        if (c == 27)
            break;
    }
    //��������� ���������
    cap.release();

    //��������� ��� ����
    destroyAllWindows();

    return 0;
}