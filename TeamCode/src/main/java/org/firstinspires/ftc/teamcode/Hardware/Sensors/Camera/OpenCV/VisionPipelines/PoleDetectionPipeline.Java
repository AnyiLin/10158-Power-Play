package org.firstinspires.ftc.teamcode.Hardware.Sensors.Camera.OpenCV.VisionPipelines;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.photo.Photo;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class PoleDetectionPipeline extends OpenCvPipeline {
    public ArrayList<ArrayList<List<Point>>> listt = new ArrayList<>();
    @Override
    public Mat processFrame(Mat input){
        listt.add(this.boxes(input));
        return input;
    }
    public ArrayList<ArrayList<List<Point>>> getList(){
        return listt;
    }
    public Mat removeWhiteNoise(int n, Mat img) {
        Mat kernel; kernel = Mat.ones(n, n, CvType.CV_8SC1);
        Mat Mid = new Mat();
        Mat Final = new Mat();
        Imgproc.dilate(img, Mid, kernel);
        Imgproc.erode(Mid, Final, kernel);
        return Final;
    }

    public Mat closeHoles(int n, Mat img) {
        Mat kernel;kernel = Mat.ones(n, n, CvType.CV_8SC1);
        Mat Mid = new Mat();
        Mat Final = new Mat();
        Imgproc.erode(img, Mid, kernel);
        Imgproc.dilate(Mid, Final, kernel);
        return Final;
    }

    public static double calcArea(double w, double h) {
        return w * h;
    }

    public Mat findYellow(Mat img) {
        Mat dest = new Mat();
        Mat mask = new Mat();
        Imgproc.cvtColor(img, dest, Imgproc.COLOR_BGR2HSV);
        Core.inRange(dest, new Scalar(14, 91, 146), new Scalar(60, 230, 255), mask);
        return mask;
    }

    public Mat colorAugmentation(Mat img) {
        Mat dest = new Mat();
        Mat ddest;
        Mat igkm;
        Photo.fastNlMeansDenoisingColored(img, dest, 10, 10, 7, 21);
        igkm = new Mat(3, 3, CvType.CV_8SC1);
        igkm.put(0, 0, 0);
        igkm.put(0, 1, -1);
        igkm.put(0, 2, 0);
        igkm.put(1, 0, -1);
        igkm.put(1, 1, 5);
        igkm.put(1, 2, -1);
        igkm.put(2, 0, 0);
        igkm.put(2, 1, -1);
        igkm.put(2, 2, 0);
        ddest = new Mat();
        Imgproc.filter2D(dest, ddest, -1, igkm);
        return ddest;
    }

    public ArrayList<List<Point>> boxes(Mat img) {
        Mat a = new Mat();
        Mat l = new Mat();
        Mat e = new Mat();
        Mat d = new Mat();
        Mat f,g,h,i,j,b,c;
        List<MatOfPoint> k;
        MatOfPoint2f approxCurve;
        ArrayList<List<Point>> o;
        Imgproc.resize(img, a, new Size(), 448, 252);
        b = colorAugmentation(a);
        c = findYellow(b);
        Core.bitwise_not(c, d);
        Imgproc.threshold(d, e, 0, 255, 1);
        f = removeWhiteNoise(2, e);
        g = closeHoles(4, f);
        h = removeWhiteNoise(6, g);
        i = closeHoles(12, h);
        j = closeHoles(6, i);
        k = new ArrayList<>();
        Imgproc.findContours(j, k, l, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        approxCurve = new MatOfPoint2f();
        o = new ArrayList<>();
        for (int n = 0; n < k.size(); n++) {
            //Convert contours(i) from MatOfPoint to MatOfPoint2f
            MatOfPoint2f contour2f = new MatOfPoint2f(k.get(n).toArray());
            double approxDistance = Imgproc.arcLength(contour2f, true) * 0.02;
            Imgproc.approxPolyDP(contour2f, approxCurve, approxDistance, true);}
        MatOfPoint points;
        points = new MatOfPoint(approxCurve.toArray());
        List<Point> m;
        m = points.toList();//Idk what size this is or what it looks like
        double x1 = m.get(0).x;
        double y1 = m.get(0).y;
        double x2 = m.get(1).x;
        double y2 = m.get(1).y;
        if((calcArea(Math.abs(x1-x2),Math.abs(y1-y2))>100)&&(Math.abs(x1-x2)*.8>Math.abs(y1-y2))){
            o.add(Arrays.asList(new Point((x1+x2)/2,(y1+y2)/2)));
        }
        return o;
    }
}