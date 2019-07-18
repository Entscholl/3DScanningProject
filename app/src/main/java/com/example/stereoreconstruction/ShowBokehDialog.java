package com.example.stereoreconstruction;

import android.app.Activity;
import android.content.Context;
import android.content.Intent;
import android.graphics.Bitmap;
import android.os.Bundle;
import android.os.SystemClock;
import android.widget.ImageView;
import android.widget.SeekBar;
import android.widget.Toast;

import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

public class ShowBokehDialog extends Activity {
	private static Mat rgbImg;
	private static Mat disparityImg;
	private static Mat disparityImgS;
	private static Mat rgbImgS;
	private Bitmap bitmapToRender;

	private ImageView imageView;
	private SeekBar focalLenghtSlider;
	//private SeekBar aperturSlider;
	private static MainActivity mainActivity;

	public void process(Context context){
		Toast.makeText(context,"start processing",Toast.LENGTH_LONG).show();
		float farFocus = focalLenghtSlider.getProgress()/500.0f;
		//double nearFocus = aperturSlider.getProgress()/500.0;
		rgbImgS  = new Mat();
		disparityImgS = new Mat();
		Imgproc.resize(rgbImg,rgbImgS,new Size(),0.5,0.5);
		Imgproc.resize(disparityImg,disparityImgS,new Size(),0.5,0.5);

		Mat outputMat = new Mat();
		mainActivity.makeBokehEffect(rgbImgS.nativeObj,disparityImgS.nativeObj,outputMat.nativeObj,farFocus);
		if(outputMat.cols()>0 && outputMat.rows() >0) {
			bitmapToRender = Bitmap.createBitmap(outputMat.cols(), outputMat.rows(), Bitmap.Config.ARGB_8888);
			Utils.matToBitmap(outputMat, bitmapToRender);
			imageView.setImageBitmap(bitmapToRender);
		}
		Toast.makeText(context,"done processing",Toast.LENGTH_SHORT).show();
	}

	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.activity_show_bokeh_image);
		Intent intent = getIntent();
		imageView = findViewById(R.id.bokehImage);
		focalLenghtSlider = findViewById(R.id.focusSlider);
		//aperturSlider = findViewById(R.id.apertureSlider);
		final Context ctx = this;
		SeekBar.OnSeekBarChangeListener listener = new SeekBar.OnSeekBarChangeListener() {
			protected int defaultInterval =1000;//override in child class
			private long lastTimeClicked = 0;
			@Override
			public void onProgressChanged(SeekBar seekBar, int i, boolean b) {
				if ((SystemClock.elapsedRealtime() - lastTimeClicked) < defaultInterval) {
					return;
				}
				lastTimeClicked = SystemClock.elapsedRealtime();
				process(ctx);
			}

			@Override
			public void onStartTrackingTouch(SeekBar seekBar) {

			}

			@Override
			public void onStopTrackingTouch(SeekBar seekBar) {

			}
		};
		focalLenghtSlider.setOnSeekBarChangeListener(listener);
		process(this);
	}

	public static void displayBokehDialog(MainActivity parent,Mat rgbImg, Mat disparityImg){
		Intent intent = new Intent(parent, ShowBokehDialog.class);
		ShowBokehDialog.rgbImg = rgbImg;
		ShowBokehDialog.disparityImg = disparityImg;
		mainActivity = parent;
		parent.startActivity(intent);
	}


}
