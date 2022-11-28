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
import java.util.List;

public class PoleDetectionPipeline extends OpenCvPipeline {

    public ArrayList<List<Point>> list = new ArrayList<List<Point>>();

    public Mat removeWhiteNoise(int n, Mat img) {
        Mat kernel; try{kernel = Mat.ones(n, n, CvType.CV_8SC1);} catch(Error E) {throw new RuntimeException(".1");}
        Mat Mid = new Mat();
        Mat Final = new Mat();
        try{Imgproc.dilate(img, Mid, kernel);} catch(Error E) {throw new RuntimeException("111");}
        try{Imgproc.erode(Mid, Final, kernel);} catch(Error E) {throw new RuntimeException("222");}
        return Final;
    }

    public Mat closeHoles(int n, Mat img) {
        Mat kernel;try{kernel = Mat.ones(n, n, CvType.CV_8SC1);} catch(Error E) {throw new RuntimeException("12");}
        Mat Mid = new Mat();
        Mat Final = new Mat();
        try{Imgproc.erode(img, Mid, kernel);} catch(Error E) {throw new RuntimeException("qqq");}
        try{Imgproc.dilate(Mid, Final, kernel);} catch(Error E) {throw new RuntimeException("www");}
        return Final;
    }

    public static double calcArea(double w, double h) {
        return w * h;
    }

    public Mat findYellow(Mat img) {
        Mat dest = new Mat();
        Mat mask = new Mat();
        try{Imgproc.cvtColor(img, dest, Imgproc.COLOR_BGR2HSV);} catch(Error E) {throw new RuntimeException("qq");}
        try{Core.inRange(dest, new Scalar(14, 91, 146), new Scalar(60, 230, 255), mask);} catch(Error E) {throw new RuntimeException("ww");}
        return mask;
    }

    public Mat colorAugmentation(Mat img) {
        Mat dest = new Mat();
        Mat ddest;
        Mat igkm;
        try{Photo.fastNlMeansDenoisingColored(img, dest, 10, 10, 7, 21);} catch(Error E) {throw new RuntimeException("q");}
        try{igkm = new Mat(3, 3, CvType.CV_8SC1);} catch(Error E) {throw new RuntimeException("w");}
        try{igkm.put(0, 0, 0);} catch(Error E) {throw new RuntimeException("e");}
        try{igkm.put(0, 1, -1);} catch(Error E) {throw new RuntimeException("r");}
        try{igkm.put(0, 2, 0);} catch(Error E) {throw new RuntimeException("t");}
        try{igkm.put(1, 0, -1);} catch(Error E) {throw new RuntimeException("y");}
        try{igkm.put(1, 1, 5);} catch(Error E) {throw new RuntimeException("u");}
        try{igkm.put(1, 2, -1);} catch(Error E) {throw new RuntimeException("i");}
        try{igkm.put(2, 0, 0);} catch(Error E) {throw new RuntimeException("o");}
        try{igkm.put(2, 1, -1);} catch(Error E) {throw new RuntimeException("p");}
        try{igkm.put(2, 2, 0);} catch(Error E) {throw new RuntimeException("a");}
        ddest = new Mat();
        try{Imgproc.filter2D(dest, ddest, -1, igkm);} catch(Error E) {throw new RuntimeException("s");}
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
        try{Imgproc.resize(img, a, new Size( 448, 252));} catch(Error E) {throw new RuntimeException("1");}
        try{b = colorAugmentation(a);} catch(Error E) {throw new RuntimeException("2");}
        try{c = findYellow(b);} catch(Error E) {throw new RuntimeException("3");}
        try{Core.bitwise_not(c, d);} catch(Error E) {throw new RuntimeException("4");}
        try{Imgproc.threshold(d, e, 0, 255, 1);} catch(Error E) {throw new RuntimeException("5");}
        try{f = removeWhiteNoise(2, e);} catch(Error E) {throw new RuntimeException("6");}
        try{g = closeHoles(4, f);} catch(Error E) {throw new RuntimeException("7");}
        try{h = removeWhiteNoise(6, g);} catch(Error E) {throw new RuntimeException("8");}
        try{i = closeHoles(12, h);} catch(Error E) {throw new RuntimeException("9");}
        try{j = closeHoles(6, i);} catch(Error E) {throw new RuntimeException("10");}
        k = new ArrayList<>();
        try{Imgproc.findContours(j, k, l, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);} catch(Error E) {throw new RuntimeException("problem");}
        try{approxCurve = new MatOfPoint2f();} catch(Error E) {throw new RuntimeException("problem");}
        o = new ArrayList<List<Point>>();
        for (int n = 0; n < k.size(); n++) {
            //Convert contours(i) from MatOfPoint to MatOfPoint2f
            MatOfPoint2f contour2f;
            double approxDistance;
            double x1,x2,y1,y2;
            try{contour2f = new MatOfPoint2f(k.get(n).toArray());} catch(Error E) {throw new RuntimeException("Hi");}
            //Processing on mMOP2f1 which is in type MatOfPoint2f
            try{approxDistance = Imgproc.arcLength(contour2f, true) * 0.02;} catch(Error E) {throw new RuntimeException("It's");}
            try{Imgproc.approxPolyDP(contour2f, approxCurve, approxDistance, true);} catch(Error E) {throw new RuntimeException("Me");}
            MatOfPoint points;
            try{points = new MatOfPoint(approxCurve.toArray());} catch(Error E) {throw new RuntimeException("I'm");}
            List<Point> m;
            try{m = points.toList();} catch(Error E) {throw new RuntimeException("the");}//Idk what size this is or what it looks like
            try{x1 = m.get(0).x;} catch(Error E) {throw new RuntimeException("problem");}
            try{y1 = m.get(0).y;} catch(Error E) {throw new RuntimeException("IT'S");}
            try{x2 = m.get(1).x;} catch(Error E) {throw new RuntimeException("ME");}
            try{y2 = m.get(1).y;} catch(Error E) {throw new RuntimeException(" At");}
            try{if((calcArea(Math.abs(x1-x2),Math.abs(y1-y2))>100)&&(Math.abs(x1-x2)*.8>Math.abs(y1-y2))){
                o.add(m);
            }} finally {throw new RuntimeException("");}
        }
        return o;
    }

    @Override
    public Mat processFrame(Mat input) {
        list = boxes(input);
        return input;
    }
}