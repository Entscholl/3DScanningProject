package com.example.stereoreconstruction;

import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;

import android.Manifest;
import android.content.Context;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.content.res.Configuration;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.os.Bundle;
import android.util.DisplayMetrics;
import android.util.Log;
import android.view.Surface;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.View;
import android.widget.Button;
import android.widget.CheckBox;
import android.widget.EditText;
import android.widget.SeekBar;
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

import static org.opencv.core.Core.ROTATE_90_CLOCKWISE;
import static org.opencv.core.Core.ROTATE_90_COUNTERCLOCKWISE;
import static org.opencv.core.Core.rotate;
import static org.opencv.imgproc.Imgproc.*;

import java.io.Serializable;


public class MainActivity extends AppCompatActivity {
	Context context;
	public static Mat currentVisibleMat;
	static Mat outputImageMat;
	static Mat inputImageA;
	static Mat inputImageB;
	static boolean rotatedA = false;
	static boolean rotatedB = false;
	static boolean capturedA = false;
	static boolean capturedB = false;
	private BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(this) {
		@Override
		public void onManagerConnected(int status) {
			switch (status) {
				case LoaderCallbackInterface.SUCCESS:
				{
					Log.i("OpenCV", "OpenCV loaded successfully");
					if(outputImageMat == null) {
						outputImageMat= new Mat();
					}
					if(inputImageA == null) {
						inputImageA= new Mat();
					}
					if(inputImageB == null) {
						inputImageB= new Mat();
					}
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
		//calibrate();

		if (!capturedA || !capturedB) {
			Toast.makeText(context, "No images captured, using defaults" ,
					Toast.LENGTH_SHORT).show();
			Bitmap bmA = BitmapFactory.decodeResource(getResources(), R.drawable.im0);
			Bitmap bmB = BitmapFactory.decodeResource(getResources(), R.drawable.im1);
			Utils.bitmapToMat(bmA, inputImageA);
			Utils.bitmapToMat(bmB, inputImageB);
		}

		if (getResources().getConfiguration().orientation == Configuration.ORIENTATION_LANDSCAPE) {
			if(!rotatedA) {
				rotatedA = true;
				rotate(inputImageA, inputImageA, ROTATE_90_COUNTERCLOCKWISE);
			}
			if(!rotatedB) {
				rotatedB = true;
				rotate(inputImageB, inputImageB, ROTATE_90_COUNTERCLOCKWISE);
			}
		}
		SeekBar disparitiesBar = findViewById(R.id.disparitiesBar);
		SeekBar blockSizeBar =  findViewById(R.id.blockSizeBar);
		int status  = processImages(inputImageA.getNativeObjAddr(), inputImageB.getNativeObjAddr(),
				outputImageMat.getNativeObjAddr(), disparitiesBar.getProgress()* 16,
				blockSizeBar.getProgress()*2 +1);
		//int status  = processImages(inputImageA.getNativeObjAddr(), inputImageB.getNativeObjAddr(),
		//		outputImageMat.getNativeObjAddr(), 16, 5);
		if(status == 0) {
			displayCVMatrix(outputImageMat);
		} else {
			if(status == -1) {
				Toast.makeText(context, "Empty images" ,Toast.LENGTH_SHORT).show();
			}
		}

	}
	public void displayCVMatrix(Mat mat) {
		if(mat.cols() == 0 || mat.rows() == 0) {
			Toast.makeText(context, "Not a valid image" , Toast.LENGTH_SHORT).show();
			return;
		}
        DisplayMetrics displayMetrics = new DisplayMetrics();
        getWindowManager().getDefaultDisplay().getMetrics(displayMetrics);
        int width = displayMetrics.widthPixels;
        int height = displayMetrics.heightPixels;

		currentVisibleMat = mat.clone();
        Imgproc.resize(currentVisibleMat, currentVisibleMat, new Size(width ,height));

		Intent intent = new Intent(this, ShowImageActivity.class);
		startActivity(intent);
	}
	public void onDisplayImageA(View view) {
		if (capturedA) {
			if (getResources().getConfiguration().orientation ==
					Configuration.ORIENTATION_LANDSCAPE) {
				if(!rotatedA) {
					rotatedA = true;
					rotate(inputImageA, inputImageA, ROTATE_90_COUNTERCLOCKWISE);
				}
			}
			displayCVMatrix(inputImageA);
		} else {
			Toast.makeText(context, "No image captured", Toast.LENGTH_SHORT).show();
		}
	}
	public void onDisplayImageB(View view) {
		if (capturedB) {
			if (getResources().getConfiguration().orientation
					== Configuration.ORIENTATION_LANDSCAPE) {
				if(!rotatedB) {
					rotatedB = true;
					rotate(inputImageB, inputImageB, ROTATE_90_COUNTERCLOCKWISE);
				}
			}
			displayCVMatrix(inputImageB);
		} else {
			Toast.makeText(context, "No image captured", Toast.LENGTH_SHORT).show();
		}
	}
	public void onRectifyButton(View view) {
		if (getResources().getConfiguration().orientation == Configuration.ORIENTATION_LANDSCAPE) {
			if(!rotatedA) {
				rotatedA = true;
				rotate(inputImageA, inputImageA, ROTATE_90_COUNTERCLOCKWISE);
			}
			if(!rotatedB) {
				rotatedB = true;
				rotate(inputImageB, inputImageB, ROTATE_90_COUNTERCLOCKWISE);
			}
		}
		if (!capturedA || !capturedB) {
			Toast.makeText(context, "No images captured, error" ,
					Toast.LENGTH_SHORT).show();
		}
		float x = 0.f,y = 0.f,z = 0.f;
		EditText x_edit = findViewById(R.id.x_val);
		EditText y_edit = findViewById(R.id.y_val);
		EditText z_edit = findViewById(R.id.z_val);
		if(x_edit.getText().toString().isEmpty()) {
			x = 0.f;
		} else {
			x = Float.valueOf(x_edit.getText().toString());
		}
		if(y_edit.getText().toString().isEmpty()) {
			y = 0.f;
		} else {
			y = Float.valueOf(y_edit.getText().toString());
		}
		if(z_edit.getText().toString().isEmpty()) {
			z = 0.f;
		} else {
			z = Float.valueOf(z_edit.getText().toString());
		}
        CheckBox gyro_box = findViewById(R.id.gyroCheck);
        CheckBox accel_box = findViewById(R.id.accelCheck);
        CheckBox uncalibrated_box = findViewById(R.id.uncalibratedCheck);
        rectifyImages(inputImageA.getNativeObjAddr(), inputImageB.getNativeObjAddr(),
				outputImageMat.getNativeObjAddr(), x, y, z, gyro_box.isChecked(),
                accel_box.isChecked(), uncalibrated_box.isChecked());
		displayCVMatrix(outputImageMat);
	}
	public void onTakeImageButton(View view) {
		//addCalibrationImage();

		if(capturedA && !capturedB) {
			capturedB = true;
			rotatedB = false;
			takePicture(inputImageB.getNativeObjAddr());
			stopMeasurement();
		} else if (!capturedA) {
			capturedA = true;
			rotatedA = false;
			takePicture(inputImageA.getNativeObjAddr());
			startMeasurement();
		} else {
			capturedB = false;
			capturedA = true;
			rotatedA = false;
			takePicture(inputImageA.getNativeObjAddr());
			startMeasurement();
		}

	}

	public void onShowBokeh(View view){
		ShowBokehDialog.displayBokehDialog(this,inputImageA,outputImageMat);
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
	public native void rectifyImages(long inputMatA, long inputMatB, long outputMatAddr ,float x
									,float y, float z, boolean use_gyro, boolean use_accel,
									 boolean use_uncalibrated);
	public native void addCalibrationImage();
	public native void calibrate();

	public native int processImages(long inputMatA, long inputMatB, long outputMatAddr,
									int num_disparities, int block_size);
	public native void makeBokehEffect(long rgbImageCV, long disparityImageCV, long outputImage,
	                              double dFocus, double focalLength);

}
