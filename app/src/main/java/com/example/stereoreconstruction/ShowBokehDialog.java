package com.example.stereoreconstruction;

import android.app.Activity;
import android.content.Intent;
import android.graphics.Bitmap;
import android.os.Bundle;
import android.widget.ImageView;
import android.widget.SeekBar;

import org.opencv.android.Utils;
import org.opencv.core.Mat;

public class ShowBokehDialog extends Activity {
	private static Mat rgbImg;
	private static Mat disparityImg;
	private Bitmap bitmapToRender;

	private ImageView imageView;
	private SeekBar focalLenghtSlider;
	private SeekBar aperturSlider;
	private static MainActivity mainActivity;

	public void process(){
		double focus = focalLenghtSlider.getProgress()/1000.0;
		double aperture = aperturSlider.getProgress()/1000.0;
		Mat outputMat = new Mat();
		mainActivity.makeBokehEffect(rgbImg.nativeObj,disparityImg.nativeObj,outputMat.nativeObj,focus,aperture);
		Utils.matToBitmap(outputMat, bitmapToRender);
		imageView.setImageBitmap(bitmapToRender);
	}

	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.activity_show_bokeh_image);
		Intent intent = getIntent();
		imageView = findViewById(R.id.bokehImage);
		focalLenghtSlider = findViewById(R.id.focusSlider);
		aperturSlider = findViewById(R.id.apertureSlider);
		SeekBar.OnSeekBarChangeListener listener = new SeekBar.OnSeekBarChangeListener() {
			@Override
			public void onProgressChanged(SeekBar seekBar, int i, boolean b) {
				process();
			}

			@Override
			public void onStartTrackingTouch(SeekBar seekBar) {

			}

			@Override
			public void onStopTrackingTouch(SeekBar seekBar) {

			}
		};
		focalLenghtSlider.setOnSeekBarChangeListener(listener);
		aperturSlider.setOnSeekBarChangeListener(listener);
		process();
	}

	public static void displayBokehDialog(MainActivity parent,Mat rgbImg, Mat disparityImg){
		Intent intent = new Intent(parent, ShowBokehDialog.class);
		ShowBokehDialog.rgbImg = rgbImg;
		ShowBokehDialog.disparityImg = disparityImg;
		mainActivity = parent;
		parent.startActivity(intent);
	}


}
