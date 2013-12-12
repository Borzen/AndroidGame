package com.example.android.accelerometerplay;

import android.app.IntentService;
import android.content.Intent;
import android.widget.Toast;

public class TimingService extends IntentService{

	public TimingService() {
		super("TimingService");
		// TODO Auto-generated constructor stub
	}

	@Override
	protected void onHandleIntent(Intent intent) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public int onStartCommand(Intent intent, int flags, int startId) {
	    Toast.makeText(this, "service starting", Toast.LENGTH_SHORT).show();
	    return super.onStartCommand(intent,flags,startId);
	}
}
