package com.example.stereoreconstruction;

import androidx.appcompat.app.AppCompatActivity;

import android.content.Intent;
import android.graphics.Bitmap;
import android.os.Bundle;
import android.util.DisplayMetrics;
import android.util.Log;
import android.widget.ImageView;

import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

public class ShowImageActivity extends AppCompatActivity {

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_show_image);
        Intent intent = getIntent();
        Mat mat = MainActivity.currentVisibleMat;
        Bitmap bm = Bitmap.createBitmap(mat.cols(), mat.rows(),Bitmap.Config.ARGB_8888);

        Utils.matToBitmap(mat, bm);
        ImageView imageView = findViewById(R.id.Image);
        imageView.setImageBitmap(bm);
    }
}
