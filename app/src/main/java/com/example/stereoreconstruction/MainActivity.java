package com.example.stereoreconstruction;

import androidx.appcompat.app.AppCompatActivity;

import android.os.Bundle;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;

public class MainActivity extends AppCompatActivity {

	// Used to load the 'native-lib' library on application startup.
	static {
		System.loadLibrary("native-lib");
	}
	boolean start = true;

	MainActivity(){
		initMeasurement();
	}

	@Override
	protected void onStart() {
		super.onStart();
	}

	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);

		setContentView(R.layout.activity_main);

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

	/**
	 * A native method that is implemented by the 'native-lib' native library,
	 * which is packaged with this application.
	 */
	public native String stringFromJNI();

	public native void initMeasurement();
	public native void startMeasurement();
	public native void stopMeasurement();
}
