package com.pervasive.helloairsim;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.view.View;
import android.widget.Toast;
import androidx.appcompat.app.AppCompatActivity;
import android.os.Bundle;

import android.widget.ImageView;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.opencv.android.Utils;

public class MainActivity extends AppCompatActivity {

    static {
        System.loadLibrary("carclient");
    }

    public native boolean CarConnect();
    public native void CarForward();
    public native void GetImage(long imgAddr);

    private Bitmap bm;
    public Mat myImage = new Mat(480,360, CvType.CV_8UC4);
    private static final int w = 720, h = 1280;
    private ImageView cameraImage;


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        cameraImage = (ImageView) findViewById(R.id.camera_image);
    }

    public void OnButtonForward(View view) {
        Toast.makeText(getApplicationContext(),
                "connection result " + myImage.getNativeObjAddr(), Toast.LENGTH_LONG).show();
        GetImage(myImage.getNativeObjAddr());
        TaskRunner runner = new TaskRunner();
        runner.executeAsync(new BaseTask() {
            @Override
            public Void call() throws Exception {
                //CarForward();
                //GetImage(myImage.getNativeObjAddr());
                /*BitmapFactory.Options options = new BitmapFactory.Options();
                options.inJustDecodeBounds = false;
                options.inBitmap = bm;
                options.inMutable = true;
                bm = Bitmap.createBitmap(myImage.rows(), myImage.cols(), Bitmap.Config.ARGB_8888);
                Imgproc.cvtColor(myImage, myImage, Imgproc.COLOR_BGR2RGBA);
                Utils.matToBitmap(myImage, bm);
                cameraImage.setImageBitmap(bm);*/
                return null;
            }
        });
    }

    public void OnButtonConnect(View view) {
        TaskRunner runner = new TaskRunner();
        runner.executeAsync(new CustomCallable<Boolean>() {
            @Override
            public Boolean call() throws Exception {
               return CarConnect();
            }
            @Override
            public void postExecute(Boolean result) {
                Toast.makeText(getApplicationContext(),
                            "connection result " + result, Toast.LENGTH_LONG).show();
            }

            @Override
            public void preExecute() {
            }
        });
    }
}
