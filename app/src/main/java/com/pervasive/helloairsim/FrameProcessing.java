package com.pervasive.helloairsim;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import org.opencv.android.Utils;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import android.util.Log;

import java.util.ArrayList;

import java.util.List;

public class FrameProcessing {

    private Bitmap frontBitmap;
    private Mat frontImage;

    private final MainActivity AirSim;

    private float current_steering=0;
    private float current_acceleration=0;

    public FrameProcessing(MainActivity mainactivity) {
        frontImage = new Mat();
        AirSim = mainactivity;
        frontBitmap = null;

        //Init bitmap image
        BitmapFactory.Options options = new BitmapFactory.Options();
        options.inJustDecodeBounds = false;
        options.inBitmap = frontBitmap;
        options.inMutable = true;
        int w = 256, h = 144;
        frontBitmap = Bitmap.createBitmap(w, h, Bitmap.Config.ARGB_8888);
    }

    public void getFrontImage() {
        AirSim.GetImage(frontImage.getNativeObjAddr()); //get frame in Mat
        Imgproc.cvtColor(frontImage, frontImage, Imgproc.COLOR_BGR2RGB);
    }

    public Bitmap LaneDetection(){

        // Converto l'immagine da BGR a HSV
        Mat frontImageHSV = new Mat();
        RGBtoHSV(frontImage, frontImageHSV);

        //Individuo solo il colore bianco e applico il filtro di Canny
        Mat edges = new Mat();
        whiteMaskandCannyFilter (frontImageHSV, edges);

        //Mi concentro solo su una determinata ROI, eliminando la parte alta dell'immagine
        edges = croppedEdges (edges);

        //Individuo le linee utilizzando la trasformata di Hough
        int threshold = 5;
        int minLineSize = 25;
        int lineGap = 4;
        Mat lines = new Mat();
        Imgproc.HoughLinesP(edges, lines, 1, Math.PI / 180,threshold,minLineSize,lineGap);

        //Individuo le linee delle due lane
        List<double[]> lane_line= average_slope_intercept(frontImage, lines);

        //Disegno le due lane della corsia
        for (int x = 0; x < lane_line.size(); x++) {
            double[] line = lane_line.get(x);
            Point[] points_= make_points(frontImage, line);
            Imgproc.line(frontImage,points_[0], points_[1], new Scalar(127, 255, 212), 3, Imgproc.LINE_AA, 0);
        }

        //Disegno la linea rossa centrale che seguirà l'agente
        Point p_offset= defineCentralLine(frontImage , lane_line);

        //Definisco steering angle e accelerazione
        if (p_offset!=null)
        {
            Point[] points_= make_points(frontImage, lane_line.get(0));
            Point p2= new Point(frontImage.cols()/2, points_[0].y);

            double angle_to_mid_radian = Math.atan((p_offset.x-p2.x)/(p_offset.y-p2.y));
            angle_to_mid_radian= -angle_to_mid_radian;

            current_acceleration=0.3f;
            current_steering= (float) angle_to_mid_radian;
            AirSim.CarControl(current_steering, current_acceleration);
        }

        //Converto la matrice dell'immagine in Bitmap per visualizzarla
        Utils.matToBitmap(frontImage, frontBitmap);
        return frontBitmap;
    }

    public void RGBtoHSV (Mat frontImage, Mat frontImageHSV) {
        Imgproc.cvtColor(frontImage, frontImageHSV, Imgproc.COLOR_RGB2HSV);
    }

    public void whiteMaskandCannyFilter (Mat frontImageHSV, Mat edges){
        int sensitivity = 15;
        Scalar lower_white = new Scalar (0,0,255-sensitivity);
        Scalar upper_white = new Scalar(255,sensitivity,255);
        Core.inRange(frontImageHSV, lower_white, upper_white, edges);
        Imgproc.Canny(edges, edges, 200, 400);
    }

    public Mat croppedEdges (Mat edges){

        int height = edges.height();
        int width = edges.width();

        Mat mask = Mat.zeros(edges.rows(), edges.cols(),edges.type());
        mask.channels();

        List<Point> points = new ArrayList<>();
        points.add(new Point(0, height * 1 / 2));
        points.add(new Point(width, height * 1 / 2));
        points.add(new Point(width, height));
        points.add(new Point(0, height));

        MatOfPoint mPoints = new MatOfPoint();
        mPoints.fromList(points);
        List<MatOfPoint> polygon = new ArrayList<MatOfPoint>();
        polygon.add(mPoints);

        Imgproc.fillPoly(mask, polygon, new Scalar (255,255,255));

        Core.bitwise_and(edges, mask, edges);
        return edges;
    }

    public List<double[]> average_slope_intercept(Mat frame, Mat line_segments)
    {
        /*
        This function combines line segments into one or two lane lines
        If all line slopes are < 0: then we only have detected left lane
        If all line slopes are > 0: then we only have detected right lane
        */

        List<double[]> lane_lines = new ArrayList<>();

        int lane_ind= 0;

        double width = frame.cols();

        int lef_indx=0;
        int right_indx=0;

        double boundary = 1/3;
        double left_region_boundary = width * (1 - boundary) ; // left lane line segment should be on left 2/3 of the screen
        double right_region_boundary = width * boundary ;// right lane line segment should be on left 2/3 of the screen

        //coeffienti m e p medi per entrambe le strisce
        double m_mean_left= 0;
        double m_mean_right= 0;
        double q_mean_left= 0;
        double q_mean_right= 0;

        for (int x= 0; x<line_segments.rows(); x++)
        {
            double[] l = line_segments.get(x, 0);
            double x1= l[0];
            double y1= l[1];
            double x2= l[2];
            double y2= l[3];

            double m=(y2-y1)/(x2-x1);
            double q= (x2*y1-x1*y2)/(x2-x1);

            //se l'angolo è minore di zero vuol dire che è della striscia sinistra
            if (m <0)
            {
                if (x1 < left_region_boundary && x2 < left_region_boundary)
                {
                    lef_indx++;

                    m_mean_left+= m;
                    q_mean_left+= q;
                }
            }
            else
            {
                if (x1 > right_region_boundary && x2 > right_region_boundary) {
                    right_indx++;

                    m_mean_right += m;
                    q_mean_right += q;
                }
            }
        }

        //faccio a media dei parametri ottenuti per entrambe le strisce
        m_mean_left= m_mean_left/(lef_indx);
        m_mean_right= m_mean_right/(right_indx);
        q_mean_left= q_mean_left/(lef_indx);
        q_mean_right= q_mean_right/(right_indx);

        if( lef_indx > 0){
            lane_lines.add(new double[]{ m_mean_left, q_mean_left});
            lane_ind++;
        }

        if(right_indx> 0){
            lane_lines.add(new double[]{ m_mean_right, q_mean_right});
            lane_ind++;
        }

        return lane_lines;

    }


    public Point[] make_points(Mat frame, double[] line)
    {
        double height = frame.rows();
        double m= line[0];
        double q= line[1];

        double y1 = height;  // bottom of the frame
        double y2 = y1 * 1 / 2 ; // make points from middle of the frame down


        double x1 = (y1 - q) / m;
        double x2 = (y2 - q) / m;

        Point p1= new Point(x1,y1);
        Point p2= new Point(x2,y2);

        Point[] points= {p1,p2};
        return  points;
    }

    public Point defineCentralLine(Mat image , List<double[]> lane_lines)
    {
        if(lane_lines.size()==0)
            return null;

        double height = image.rows();
        double width = image.cols();

        double x_offset;
        double y_offset;

        if(lane_lines.size()==2)
        {
            Point[] pointsLeft= make_points(image, lane_lines.get(0));
            Point[] pointsRight= make_points(image, lane_lines.get(1));

            double left_x2= pointsLeft[1].x;
            double right_x2= pointsRight[1].x;

            x_offset = (left_x2 + right_x2) / 2 ;
            y_offset = (height / 2);

            Imgproc.line(image,new Point(width/2, pointsLeft[0].y), new Point(x_offset, y_offset), new Scalar(255, 0, 0), 10, Imgproc.LINE_AA, 0);
        }
        else
        {
            Point[] points= make_points(image, lane_lines.get(0));

            double x1= points[0].x;
            double x2= points[1].x;

            x_offset = x2 - x1;
            y_offset = (height / 2);

            Imgproc.line(image,new Point(width/2, points[0].y), new Point(x_offset, y_offset), new Scalar(255, 0, 0), 3, Imgproc.LINE_AA, 0);
        }

        return new Point(x_offset, y_offset);


    }

}
