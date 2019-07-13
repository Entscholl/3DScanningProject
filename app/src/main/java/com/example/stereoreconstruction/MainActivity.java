package com.example.stereoreconstruction;

import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;

import android.Manifest;
import android.content.Context;
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
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import static org.opencv.imgproc.Imgproc.*;

import java.io.Serializable;


public class MainActivity extends AppCompatActivity {
	Context context;
	static Mat outputImageMat;
	static Mat inputImageA;
	static Mat inputImageB;
	boolean capturedA = false;
	boolean capturedB = false;
	private BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(this) {
		@Override
		public void onManagerConnected(int status) {
			switch (status) {
				case LoaderCallbackInterface.SUCCESS:
				{
					Log.i("OpenCV", "OpenCV loaded successfully");
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
		context = this;
		initMeasurement();
		super.onCreate(savedInstanceState);

		setContentView(R.layout.activity_main);
		if (ContextCompat.checkSelfPermission(this, Manifest.permission.CAMERA) != PackageManager.PERMISSION_GRANTED) {
			ActivityCompat.requestPermissions(
					this,
					new String[] { Manifest.permission.CAMERA },
					1);
			Toast.makeText(context, "No Camera Permission" ,Toast.LENGTH_SHORT).show();
			Log.e("3dscanning", "No Camera Permission");
			return;
		}
		SurfaceView surfaceView = (SurfaceView) findViewById(R.id.surfaceView);
		SurfaceHolder surfaceHolder = surfaceView.getHolder();
		surfaceHolder.addCallback(new SurfaceHolder.Callback() {
			@Override
			public void surfaceCreated(SurfaceHolder holder) {

				Log.v("3dscanning", "surface created.");

				//Todo currently hardcoded
				holder.setFixedSize(1920, 1080);
				initCamera(holder.getSurface());
			}
			@Override
			public void surfaceDestroyed(SurfaceHolder holder) {
				stopCameraPreview();
			}
			@Override
			public void surfaceChanged(SurfaceHolder holder, int format, int width, int height) {
				Log.i("3dscanning", "New Preview Size: " + width + ", " + height);
				setUpCameraSession(holder.getSurface());
				startCameraPreview();
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

		if (!capturedA || !capturedB) {
			Toast.makeText(context, "No images captured, using defaults" ,
					Toast.LENGTH_SHORT).show();
			Bitmap bmA = BitmapFactory.decodeResource(getResources(), R.drawable.im0);
			Bitmap bmB = BitmapFactory.decodeResource(getResources(), R.drawable.im1);
			Utils.bitmapToMat(bmA, inputImageA);
			Utils.bitmapToMat(bmB, inputImageB);
		}
		Toast.makeText(context, "Calculating..." ,Toast.LENGTH_LONG).show();
		outputImageMat = new Mat();
		int status  = processImages(inputImageA.getNativeObjAddr(), inputImageB.getNativeObjAddr(),
				outputImageMat.getNativeObjAddr());
		if(status == 0) {
			displayCVMatrix(outputImageMat);
		} else {
			if(status == -1) {
				Toast.makeText(context, "Empty images" ,Toast.LENGTH_SHORT).show();
			}
		}
	}
	public void displayCVMatrix(Mat mat) {
		Mat output = new Mat();

		//TODO get actual Display Size

		Imgproc.resize(mat, output, new Size(1100,2000));

		Intent intent = new Intent(this, ShowImageActivity.class);
		intent.putExtra("Mat", output.getNativeObjAddr());
		startActivity(intent);
	}
	public void onDisplayImageA(View view) {
		if (capturedA) {
			displayCVMatrix(inputImageA);
		} else {
			Toast.makeText(context, "No image captured", Toast.LENGTH_SHORT).show();
		}
	}
	public void onDisplayImageB(View view) {
		if (capturedB) {
			displayCVMatrix(inputImageB);
		} else {
			Toast.makeText(context, "No image captured", Toast.LENGTH_SHORT).show();
		}
	}
	public void onRectifyButton(View view) {
		if (!capturedA || !capturedB) {
			Toast.makeText(context, "No images captured, error" ,
					Toast.LENGTH_SHORT).show();
		}
		rectifyImages(inputImageA.getNativeObjAddr(), inputImageB.getNativeObjAddr());
	}
	public void onTakeImageButton(View view) {
		if(capturedA && !capturedB) {
			capturedB = true;
			inputImageB = new Mat();
			takePicture(inputImageB.getNativeObjAddr());
			stopMeasurement();
		} else if (!capturedA) {
			capturedA = true;
			inputImageA = new Mat();
			takePicture(inputImageA.getNativeObjAddr());
			startMeasurement();
		} else {
			capturedB = false;
			capturedA = true;
			inputImageA = new Mat();
			takePicture(inputImageA.getNativeObjAddr());
			startMeasurement();
		}
	}
	/**
	 * A native method that is implemented by the 'native-lib' native library,
	 * which is packaged with this application.
	 */
	public native String stringFromJNI();
	public native void setUpCameraSession(Surface surfaceView);
	public native void startCameraPreview();
	public native void initCamera(Surface surfaceView);
	public native void stopCameraPreview();
	public native void takePicture(long output_mat_addr);
	public native void initMeasurement();
	public native void startMeasurement();
	public native void stopMeasurement();
	public native void rectifyImages(long inputMatA, long inputMatB);

	public native int processImages(long inputMatA, long inputMatB, long outputMatAddr);
}
