package com.example.android.accelerometerplay;

import java.util.Timer;
import java.util.TimerTask;

import android.app.Service;
import android.content.Context;
import android.content.Intent;
import android.hardware.SensorManager;
import android.os.Handler;
import android.os.HandlerThread;
import android.os.IBinder;
import android.os.Message;
import android.widget.Toast;

public class TimingService extends Service{

	private static Timer time = new Timer();
	private HandlerThread thread;
	private static boolean _stop;
	private Context context;
	@Override
	public IBinder onBind(Intent intent) {
		// TODO Auto-generated method stub
		return null;
	}
	
	@Override
	public void onCreate() {
	    super.onCreate();
	    thread = new HandlerThread("TimingService",
		            android.os.Process.THREAD_PRIORITY_BACKGROUND);
	    _stop=false;
	    thread.start();
	   
	}
	
	
	@Override
	  public int onStartCommand(Intent intent, int flags, int startId) {     
		time.schedule(new notifier(),10000,5000);
		return START_STICKY;
	  }

	
	
	@Override
	  public void onDestroy() {
	    stopSelf();
	    _stop=true;
	    super.onDestroy();
	  }


	private class notifier extends TimerTask{
		public void run()
		{
			if(!_stop)
			{
				context = getApplicationContext();
				SensorManager sm = (SensorManager) context.getSystemService(context.SENSOR_SERVICE);
				
				handler.sendEmptyMessage(0);
			}
		}
	}
	
	
	private final Handler handler = new Handler()
	{
		public void handleMessage(Message msg){
			System.out.println("YOU AREN'T DOING ANYTHING!");
			Toast.makeText(getApplicationContext(), "DO SOMETHING", Toast.LENGTH_SHORT).show();
		}
	};

}
