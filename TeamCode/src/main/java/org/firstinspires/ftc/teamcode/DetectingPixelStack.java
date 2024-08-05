package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
/**
public class DetectingPixelStack implements org.firstinspires.ftc.vision.VisionProcessor {
    Mat submat = new Mat();
    Mat hsvMat = new Mat();


    public void inputToHSV(Mat frame) {
        Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_RGB2HSV);
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    int detectBlackWhite(Mat input) {
        int leftEdge = 0;
        int rightEdge = 0;
        int edgeNum = 0;
        int x1 = 0;
        int x2 = 1000;
        int midy = 750;
        for (int j = x1 + 1; j < x2; j++) {
//            if (input.at(Byte.class, (y2 + y1) / 2, j).getV4c().get_0() - input.at(Byte.class, (y2 + y1) / 2, j - 1).getV4c().get_0() > Constants.changeThresh) {
            if ((input.at(Byte.class, midy, j).getV().byteValue() - input.at(Byte.class, midy, j - 1).getV().byteValue()) > 128) {
                edgeNum++;
                if (edgeNum == 1) {
                    leftEdge = j;
                }
                else if (edgeNum == 2) {
                    rightEdge = j;
                }
            }
        }

        //return leftEdge, rightEdge;
    }

 //   @Override
   // public Object processFrame(Mat frame, long captureTimeNanos) {

  //  }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}**/
