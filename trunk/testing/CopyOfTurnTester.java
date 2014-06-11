package testing;

import static testing.rac3Truck.Rac3TruckMovement.*;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.hardware.port.SensorPort;
import lejos.robotics.SampleProvider;
import testing.rac3Truck.Rac3TruckSteering;
import testing.sensors.EV3IRSideSensor;
import testing.sensors.EV3UltrasonicSideSensor;
import testing.sensors.SideSensor;

public class CopyOfTurnTester
{
	private static SideSensor leftSensor = new EV3UltrasonicSideSensor(SensorPort.S2);
	private static SideSensor frontSensor = new EV3IRSideSensor(SensorPort.S4);
	private static ResettableGyroSensor sensor = new ResettableGyroSensor(SensorPort.S1);
	private static SampleProvider gyro = sensor.getAngleMode();
	private static SampleProvider rgyro = sensor.getRateMode();
	private static float leftReading, frontReading;
	private static int power;
	private static float targetReading;
	
	private static float getDataFromSensor()
	{
		float[] data = new float[1];	//an array is necessary to get the data
		gyro.fetchSample(data, 0);
		return data[0];
	}
	
	private static float getRateDataFromSensor()
	{
		float[] data = new float[1];	//an array is necessary to get the data
		rgyro.fetchSample(data, 0);
		return data[0];
	}

//	@SuppressWarnings("deprecation")	//Thread.stop() is deprecated
	public static void main(String[] args) throws Throwable
	{
		Button.LEDPattern(2);	//turn button backlight red

//		sensor.hardReset();
		sensor.reset();
		Thread.sleep(1500);	//reset delay
		getRateDataFromSensor();
		getDataFromSensor();
		Thread.sleep(3000);

		Rac3TruckSteering.reset();
		Button.LEDPattern(1);	//turn button backlight green
		Sound.beep();
		Thread.sleep(1000);
		System.out.println("Program starting.");
		//Button.waitForAnyPress();
		/*Thread first = new Thread(new LatchThread(25, 0));
		first.start();
		Thread.sleep(5000);
		first.stop();
		goLeft(40);	//counterclockwise rotations are positive
		while((getDataFromSensor()) < 75)
		{
			Thread.sleep(10);
		}
		Thread second = new Thread(new LatchThread(25, 90));
		second.start();
		Thread.sleep(5000);
		second.stop();	
		stop();*/
		goStraight(100);
		new Thread(new LeftMonitorThread()).start();
		Thread forwardThread = new Thread(new LatchThread(70, 0));
		forwardThread.start();
		while(leftReading != Float.POSITIVE_INFINITY);
		forwardThread.interrupt();
		new Thread(new FrontMonitorThread()).start();
		goLeft(50);
		while(getDataFromSensor() < 85);
		Thread hallThread = new Thread(new LatchThread(60, 90));
		hallThread.start();
		while(frontReading > 20);
		hallThread.interrupt();
		Thread roomThread = new Thread(new LatchThread(70, 0));
		while(leftReading != Float.POSITIVE_INFINITY);
		goRight(50);
		while(getDataFromSensor() > 5);
		roomThread.interrupt();
		Thread lastThread = new Thread(new LatchThread(70, 90));
		lastThread.start();
		while(frontReading > 10);
		lastThread.interrupt();
	}
	
	private static class LeftMonitorThread implements Runnable
	{
		public void run()
		{
			while(true)
			{
				leftReading = leftSensor.getDistanceInCM();
				try {
					Thread.sleep(10);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			}
		}
	}
	
	private static class FrontMonitorThread implements java.lang.Runnable
	{
		public void run()
		{
			frontReading = frontSensor.getDistanceInCM();
			try {
				Thread.sleep(10);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
	}
	
	private static class LatchThread implements java.lang.Runnable
	{
		
		LatchThread(int _power, float _targetReading)
		{
			power = _power;
			targetReading = _targetReading;
		}
		
		public void run()
		{
			float currentReading = getDataFromSensor();
			float offset = currentReading - targetReading;
			System.out.println("Time\tTacho\tOffset\tLeft distance\tAction");
			for(long currentTime = System.currentTimeMillis(); true;)
			{
				String action;
				if(offset >= 3)	//too much to the left
				{
					tiltRight(power);
					action = "Tilt right";
				}
				else if(offset <= -3)	//too much to the right
				{
					tiltLeft(power);
					action = "Tilt left";
				}
				else
				{
					goStraight(power);
					action = "Straight";
				}
				System.out.println((System.currentTimeMillis() - currentTime) + "\t" + Rac3TruckSteering.getTachoCount()
						+ "\t" + offset + "\t" + leftReading + " cm\t" + action);	
				LCD.clearDisplay();	//we don't want the LCD to be cluttered with output			

				currentReading = getDataFromSensor();
				offset = currentReading - targetReading;
				if(isStalled())	//once the robot has crashed into an obstacle
				{
					System.out.println("Robot stalled; stopping program");
					System.exit(0);
				}
				if(Thread.interrupted())
					return;
			}
		}
	}
}




