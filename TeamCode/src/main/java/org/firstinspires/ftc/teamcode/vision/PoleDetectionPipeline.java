package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import static org.opencv.core.Core.inRange;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;

public class PoleDetectionPipeline extends OpenCvPipeline {
    private final Mat yCrCb = new Mat();

    private final Size kSize = new Size(5, 5);
    private final Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, kSize);


    public Scalar poleLower = new Scalar(60, 135, 10);
    public Scalar poleHigher = new Scalar(190, 180, 105);

    private final Mat binaryMat = new Mat();

    private final ArrayList<MatOfPoint> contours = new ArrayList<>();

    public double horizon = 5;

    public double CONTOUR_AREA = 250.0;
    private final Scalar CONTOUR_COLOR = new Scalar(255,0,255);
    private final Scalar HORIZON_COLOR = new Scalar(0,255,0);
    private final Scalar TEXT_COLOR = new Scalar(0, 0, 0);

    private Rect poleRect = new Rect();

    public int poleX = 0;
    public int poleY = 0;


    @Override
    //Detect pole and get coordinates of pole rectangle
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, yCrCb, Imgproc.COLOR_RGB2YCrCb);
        Imgproc.erode(yCrCb, yCrCb, kernel);

        inRange(yCrCb, poleLower, poleHigher, binaryMat);

        Imgproc.findContours(binaryMat, contours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        contours.removeIf(c -> Imgproc.boundingRect(c).y + (Imgproc.boundingRect(c).height / 2.0) < horizon);
        Imgproc.drawContours(input, contours, -1, CONTOUR_COLOR);

        if(!contours.isEmpty()) {
            MatOfPoint biggestPole = Collections.max(contours, Comparator.comparingDouble(t0 -> Imgproc.boundingRect(t0).height));
            if(Imgproc.contourArea(biggestPole) > CONTOUR_AREA) {
                poleRect = Imgproc.boundingRect(biggestPole);

                poleX = poleRect.x;
                poleY = poleRect.y;


                Imgproc.rectangle(input, poleRect, CONTOUR_COLOR, 2);
                Imgproc.putText(input, "Pole " + (poleRect.x + (poleRect.width/2.0)) +","+(poleRect.y + (poleRect.height/2.0)), new Point(poleRect.x, poleRect.y < 10 ? (poleRect.y+poleRect.height+20) : (poleRect.y - 8)), Imgproc.FONT_HERSHEY_SIMPLEX, 0.8, TEXT_COLOR, 2);
                Imgproc.circle(input, new Point(poleRect.x + (poleRect.width/2.0), poleRect.y + (poleRect.height/2.0)), 3, HORIZON_COLOR, 3);
            }
        }

        Imgproc.line(input, new Point(0,horizon), new Point(320, horizon), HORIZON_COLOR);
        Imgproc.circle(input, new Point(320, 240), 3, HORIZON_COLOR, 3);

        contours.clear();
        yCrCb.release();
        binaryMat.release();

        return input;


    }
}
