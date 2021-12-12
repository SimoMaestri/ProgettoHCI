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

    private static final int w = 256, h = 144;
    private Bitmap frontBitmap;
    private Mat frontImage;

    private MainActivity AirSim;

    private int sensitivity = 15;
    private Scalar lower_white = new Scalar (0,0,255-sensitivity);
    private Scalar upper_white = new Scalar(255,sensitivity,255);


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
        frontBitmap = Bitmap.createBitmap(w, h, Bitmap.Config.ARGB_8888);
    }

    public void getFrontImage() {
        AirSim.GetImage(frontImage.getNativeObjAddr()); //get frame in Mat
        Imgproc.cvtColor(frontImage, frontImage, Imgproc.COLOR_BGR2RGB);
    }

    public Bitmap LineDetection(){

        // Converto l'immagine da RGB a HSV
        Mat edges = new Mat();
        Mat frontImageHSV = new Mat();
        Imgproc.cvtColor(frontImage, frontImageHSV, Imgproc.COLOR_BGR2HSV);

        //Individuo solo il colore bianco e applico il filtro di Canny
        Core.inRange(frontImageHSV, lower_white, upper_white, edges);
        Imgproc.Canny(edges, edges, 200, 400);

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


        if (p_offset!=null)
        {
            Point[] points_= make_points(frontImage, lane_line.get(0));
            Point p2= new Point(frontImage.cols()/2, points_[0].y);

            double angle_to_mid_radian = Math.atan((p_offset.x-p2.x)/(p_offset.y-p2.y));
            angle_to_mid_radian= -angle_to_mid_radian;

            Log.i("Steering angle", "current steering calculate"+String.valueOf(angle_to_mid_radian));


            if(lane_line.size()==2)
            {


                current_acceleration=0.3f;
            }
            else
            {


                current_acceleration=0.3f;

            }


            current_steering= (float) angle_to_mid_radian;
            AirSim.CarControl(current_steering, current_acceleration);


        }


        Utils.matToBitmap(frontImage, frontBitmap);
        return frontBitmap;
    }


    public Mat croppedEdges (Mat edges){
        //Cropped Edges
        int height = edges.height();
        int width = edges.width();

        Mat mask = Mat.zeros(144, 256,edges.type());
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
        //List<double[]> mean_value= new ArrayList<>();

        int lane_ind= 0;

        if (line_segments.rows()==0)
        {
            Log.i("average_slope", "'No line_segment segments detected'");
        }

        //double height = frame.rows();
        double width = frame.cols();


        int lef_indx=0;
        int right_indx=0;

        double boundary = 1/3;
        double left_region_boundary = width * (1 - boundary) ; // left lane line segment should be on left 2/3 of the screen
        double right_region_boundary = width * boundary ;// right lane line segment should be on left 2/3 of the screen

        //coeffienti m e p medi per entrambe le sctrisc e
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

            Log.i("average_slope", "x1"+String.valueOf(x1)+"x2"+String.valueOf(x2)+"y1"+String.valueOf(y1)+"y2"+String.valueOf(y2));

            if(x1==x2)
            {
                Log.i("average_slope", "skipping vertical line segment (slope=inf): %s"+String.valueOf(l));
                continue;
            }

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

                    //Imgproc.line(frame, new Point(x1,y1), new Point(x2,y2), new Scalar(255, 0, 255), 1, Imgproc.LINE_AA, 0);

                }
            }
            else
            {
                if (x1 > right_region_boundary && x2 > right_region_boundary)
                {
                    right_indx++;

                    m_mean_right+= m;
                    q_mean_right+= q;
                    //Imgproc.line(frame, new Point(x1,y1), new Point(x2,y2), new Scalar(255, 255, 0), 1, Imgproc.LINE_AA, 0);


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
        Log.i("lane line", "number line "+String.valueOf(lane_lines.size()));


        return lane_lines;

    }


    public Point[] make_points(Mat frame, double[] line)
    {
        double height = frame.rows();
        //double width = frame.cols();
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


            //int mid = (int)(width / 2);
            x_offset = (left_x2 + right_x2) / 2 ;
            y_offset = (height / 2);

            //per disegnare la curva uso anche x1
            double left_x1= pointsLeft[0].x;
            double right_x1= pointsRight[0].x;

            //double xmean= (left_x1 + right_x1) / 2 ;

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

        //Point[] points = make_points(image, lane_lines.get(0));

        return new Point(x_offset, y_offset);


    }

}
