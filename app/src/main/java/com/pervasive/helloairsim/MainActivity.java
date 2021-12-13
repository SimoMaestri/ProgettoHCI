package com.pervasive.helloairsim;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.os.Handler;
import android.util.Log;
import android.view.View;
import android.widget.Toast;
import androidx.appcompat.app.AppCompatActivity;
import android.os.Bundle;

import android.widget.ImageView;

import org.opencv.android.OpenCVLoader;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.android.Utils;

public class MainActivity extends AppCompatActivity {

    static {
        System.loadLibrary("carclient");
    }

    public native boolean CarConnect();
    public native void GetImage(long imgAddr);
    public native void CarControl(float angle, float force);

    private ImageView cameraImage;
    private FrameProcessing frameProc;
    public boolean connection = false;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        cameraImage = (ImageView) findViewById(R.id.camera_image);
        frameProc = new FrameProcessing(this);
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
                GetFrame();
                connection = result;
            }

            @Override
            public void preExecute() {
            }
        });
    }

    private void GetFrame(){
        final Handler handler = new Handler();
        handler.postDelayed(new Runnable() {
            public void run() {
                if (connection) {
                    frameProc.getFrontImage();
                    if (cameraImage != null) {
                        cameraImage.setImageBitmap(frameProc.LaneDetection());
                    }
                    handler.postDelayed(this, 1);
                }
            }

        }, 1);
    }
}