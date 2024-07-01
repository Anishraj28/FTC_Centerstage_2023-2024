package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.*;
import org.opencv.core.Mat;
import org.firstinspires.ftc.robotcore.external.hardware.camera.*;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.robotcore.external.Telemetry;


public class CompetitionRedLeft implements VisionProcessor {
    public volatile int position = -1; // 1 for left, 2 for middle, 3 for right
    //1 for center 2 for right 3 for left maybe
    //private final boolean TUNING = true;

    public Rect leftRect = new Rect(280,200,42,74); // left square to scan
    public Rect middleRect = new Rect(540,244,74,53); // middle square to scan


    private Mat left = new Mat();
    private Mat middle = new Mat();
    private Mat right = new Mat();

    private Mat red = new Mat();

    public int avg1;
    public int avg2;

    private final Scalar RED = new Scalar(255, 0, 0); // for debugging
    private final Scalar GREEN = new Scalar(0, 255, 0); // for debugging

    @Override
    public void init(int width, int height, CameraCalibration calibration) {


    }
    int max = 0;
    @Override
    public Object processFrame(Mat inputMat, long ms) {
        Imgproc.cvtColor(inputMat, red, Imgproc.COLOR_RGB2HSV); // convert RGB to HSV. We read the Saturation value.
        resizeRegions();
        avg1 = (int) Core.mean(left).val[1]; // averages the left region and extract saturation value
        avg2 = (int) Core.mean(middle).val[1]; // averages the left region and extract saturation value

        max = Math.max(avg1, avg2); // finds the biggest between the two

        if(max == avg1) position = 1;
        if(max == avg2) position = 2;
        if(max < 50) position = 3; // if the highest saturation value between left and middle is less than 50, both regions are grey.

        render(inputMat); // draw squares onto screen

        telemetry.addData("position", position);
        telemetry.addData("left region:", avg1);
        telemetry.addData("middle region:", avg2);
        telemetry.update();
        return inputMat;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int width, int height, float canvasPx, float pxDensity, Object userContext){

    }

    public void resizeRegions(){
        left = red.submat(leftRect);
        middle = red.submat(middleRect);

    }

    public void render(Mat input){
        Imgproc.rectangle(
                input, // Buffer to draw on
                leftRect.tl(), // First point which defines the rectangle
                leftRect.br(), // Second point which defines the rectangle
                (position == 1) ? GREEN : RED, // The color the rectangle is drawn in
                (position == 1) ? -1 : 2); // Thickness of the rectangle lines. -1 fills in the whole square
        Imgproc.rectangle(
                input, // Buffer to draw on
                middleRect.tl(), // First point which defines the rectangle
                middleRect.br(), // Second point which defines the rectangle
                (position == 2) ? GREEN : RED, // The color the rectangle is drawn in
                (position == 2) ? -1 : 2); // Thickness of the rectangle lines
    }


    Telemetry telemetry;

    public CompetitionRedLeft(Telemetry telemetry) {
        this.telemetry = telemetry;
        // telemetry.addData("position: ", position);

    }


}