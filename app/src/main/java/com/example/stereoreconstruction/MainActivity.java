package com.example.stereoreconstruction;

import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;

import android.Manifest;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.os.Bundle;
import android.util.Log;
import android.view.Surface;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;

import java.io.Serializable;


public class MainActivity extends AppCompatActivity {

	Mat outputImageMat;
	Mat inputImageA;
	Mat inputImageB;
	boolean capturedA = false;
	boolean capturedB = false;
	private BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(this) {
		@Override
		public void onManagerConnected(int status) {
			switch (status) {
				case LoaderCallbackInterface.SUCCESS:
				{
					Log.i("OpenCV", "OpenCV loaded successfully");
					outputImageMat=new Mat();
					inputImageA=new Mat();
					inputImageB=new Mat();
				} break;
				default:
				{
					super.onManagerConnected(status);
				} break;
			}
		}
	};
	// Used to load the 'native-lib' library on application startup.
	static {
		System.loadLibrary("native-lib");
	}
	boolean start = true;

	@Override
	protected void onStart() {
		super.onStart();
	}

	@Override
	protected void onCreate(Bundle savedInstanceState) {
		initMeasurement();
		super.onCreate(savedInstanceState);

		setContentView(R.layout.activity_main);

		if (ContextCompat.checkSelfPermission(this, Manifest.permission.CAMERA) != PackageManager.PERMISSION_GRANTED) {
			ActivityCompat.requestPermissions(
					this,
					new String[] { Manifest.permission.CAMERA },
					1);
			Log.e("3dscanning", "No Camera Permission");
			return;
		}
		SurfaceView surfaceView = (SurfaceView) findViewById(R.id.surfaceView);
		SurfaceHolder surfaceHolder = surfaceView.getHolder();
		surfaceHolder.addCallback(new SurfaceHolder.Callback() {
			@Override
			public void surfaceCreated(SurfaceHolder holder) {

				Log.v("3dscanning", "surface created.");
				holder.setFixedSize(640, 480);
				startCameraPreview(holder.getSurface());
			}
			@Override
			public void surfaceDestroyed(SurfaceHolder holder) {
				stopCameraPreview();
			}
			@Override
			public void surfaceChanged(SurfaceHolder holder, int format, int width, int height) {
			}
		});

		Button measureButton = findViewById(R.id.MeasureButton);
		measureButton.setOnClickListener(new View.OnClickListener() {
			@Override
			public void onClick(View view) {
				if(start)
				{
					startMeasurement();
				}
				else{
					stopMeasurement();
				}
				start = !start;
			}
		});

		// Example of a call to a native method
		//TextView tv = findViewById(R.id.sample_text);
		//tv.setText(stringFromJNI());
	}
	@Override
	protected void onResume() {
		super.onResume();
		if (!OpenCVLoader.initDebug()) {
			Log.d("OpenCV", "Internal OpenCV library not found. Using OpenCV Manager for initialization");
			OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_0_0, this, mLoaderCallback);
		} else {
			Log.d("OpenCV", "OpenCV library found inside package. Using it!");
			mLoaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
		}
	}
	public void onCalculateButton(View view) {

		//Bitmap bmA = BitmapFactory.decodeResource(getResources(), R.drawable.im0);
		//Bitmap bmB = BitmapFactory.decodeResource(getResources(), R.drawable.im1);
		//Utils.bitmapToMat(bmA, inputImageA);
		//Utils.bitmapToMat(bmB, inputImageB)
		//TODO start the image processing here
		if (capturedA && capturedB) {
			processImages(inputImageA.getNativeObjAddr(), inputImageB.getNativeObjAddr(),
					outputImageMat.getNativeObjAddr());
			Intent intent = new Intent(this, ShowImageActivity.class);
			intent.putExtra("Mat", outputImageMat.getNativeObjAddr());
			startActivity(intent);
		} else {
			Toast.makeText(this, "No images captured", Toast.LENGTH_SHORT).show();
		}
	}
	public void onDisplayImage(View view) {
		//TODO same for Picture B
		if (capturedA) {
			processImages(inputImageA.getNativeObjAddr(), inputImageB.getNativeObjAddr(),
					outputImageMat.getNativeObjAddr());
			Intent intent = new Intent(this, ShowImageActivity.class);
			intent.putExtra("Mat", inputImageA.getNativeObjAddr());
			startActivity(intent);
		} else {
			Toast.makeText(this, "No images captured", Toast.LENGTH_SHORT).show();
		}
	}
	public void onTakeImageButton(View view) {
		//TODO redesign this stuff here and combine with calculate
		if(capturedA) {
			capturedB = true;
			//TODO don't do this stuff here
			inputImageB = new Mat(480, 640, CvType.CV_8UC3);
			takePicture(inputImageB.getNativeObjAddr());
			//TODO stop measuring here
		} else {
			capturedA = true;
			//TODO don't do this stuff here
			inputImageA = new Mat(480, 640, CvType.CV_8UC3);
			takePicture(inputImageA.getNativeObjAddr());
			//TODO start measuring here
		}
	}
	/**
	 * A native method that is implemented by the 'native-lib' native library,
	 * which is packaged with this application.
	 */
	public native String stringFromJNI();
	public native void startCameraPreview(Surface surfaceView);
	public native void stopCameraPreview();
	public native void takePicture(long output_mat_addr);
	public native void initMeasurement();
	public native void startMeasurement();
	public native void stopMeasurement();
	public native void processImages(long inputMatA, long inputMatB, long outputMatAddr);
}
