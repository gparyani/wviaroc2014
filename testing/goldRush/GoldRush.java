package testing.goldRush;
import static testing.rac3Truck.Rac3TruckMovement.*;

import java.util.ArrayDeque;
import java.util.Deque;
import java.util.Queue;

import testing.ResettableGyroSensor;
import testing.rac3Truck.Rac3TruckMovement;
import testing.rac3Truck.Rac3TruckSteering;
import testing.sensors.EV3IRSideSensor;
import testing.sensors.EV3UltrasonicSideSensor;
import testing.sensors.SideSensor;
import lejos.hardware.Button;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3IRSensor;
import lejos.robotics.SampleProvider;

public class GoldRush {
	private static EV3IRSideSensor leftSensor = new EV3IRSideSensor(SensorPort.S2);
	private static EV3IRSideSensor frontSensor = new EV3IRSideSensor(SensorPort.S4);
	private static EV3IRSideSensor rightSensor = new EV3IRSideSensor(SensorPort.S3);
	private static ResettableGyroSensor sensor = new ResettableGyroSensor(SensorPort.S1);
	private static SampleProvider gyro = sensor.getAngleMode(), rgyro = sensor.getRateMode();
	private static volatile float leftReading, frontReading, rightReading, offset;
	private static int power;
	private static volatile float targetReading, currentReading;
	private static volatile boolean stalled, leftWall, frontWall, rightWall, isBacking, isTurning;
	private static final double X_ORIGINAL_VALUE = 0.0, Y_ORIGINAL_VALUE = 20.0;
	private static volatile double realX = 0, realY = 1;
	private static volatile double x = X_ORIGINAL_VALUE, y = Y_ORIGINAL_VALUE; //Center of all sensors
	private static volatile int rTachoCount, lTachoCount;
	private static Queue<Float> leftValues = new ArrayDeque<Float>(), frontValues = new ArrayDeque<Float>(), rightValues = new ArrayDeque<Float>();
	private static final int ANGLE_ERROR_MARGIN = 5;
	private static final float CELL_WIDTH = 62.5f;
	
	private enum State {
		CALIBRATING, SEEKING_BEACON, MOVING_TOWARDS_BEACON, AVOIDING_OBSTACLE
	}
	
	private static synchronized void updateCurrentLoc()
	{
		int deltaR, deltaL;
		double deltaX, deltaY;
		
		deltaR = getRightTachoCount() - rTachoCount;
		rTachoCount += deltaR;
		
		deltaL = getLeftTachoCount() - lTachoCount;
		lTachoCount += deltaL;
			
		int betterReading;
		if(Math.abs(deltaL) < 2 || Math.abs(deltaR) < 2) 
			betterReading = (deltaL < 0 || deltaR < 0) ? Math.max(deltaL, deltaR) : Math.min(deltaL, deltaR);
		else betterReading = (deltaL + deltaR) /2 ;
		
		//currentReading is with respect to +y-axis; add 90 so it's w.r.t. +x-axis
		//change vector magnitude and direction to x- and y-components
		//deltaX and deltaY are displacements since last iteration
		//they are in units of MOTOR degrees
		deltaX = betterReading * Math.cos(Math.toRadians(currentReading + 90));
		deltaY = betterReading * Math.sin(Math.toRadians(currentReading + 90));
		
		//update current absolute location
		realX += deltaX * 2.5 / 360.0 * 31;	//deltaX * gear ratio / (convert degrees to rotations) * wheel circumference
		realY += deltaY * 2.5 / 360.0 * 31;	//deltaY * gear ratio / (convert degrees to rotations) * wheel circumference
		
		x= realX + 20 * Math.cos(Math.toRadians(currentReading + 90));
		y = realY + 20* Math.sin(Math.toRadians(currentReading + 90));
	}
	
	private static synchronized float getDataFromSensor()
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
	
	/**
	 * Controls the movements of the robot
	 * @author Robert
	 *
	 */
	private static class MovementThread implements Runnable {
			MovementThread(int _power, float _targetReading)
			{
				power = _power;
				targetReading = _targetReading;
			}
			
			private static final float BEARING_TO_OFFSET_RATIO = 0.6f;
			
			public void run()
			{
				while(true)
				{
					currentReading = getDataFromSensor();
					float offset = currentReading - targetReading;
					System.out.println(offset);
					float bearing = offset / BEARING_TO_OFFSET_RATIO;
					
					if(bearing >= 4)
					{
						bearing += 25;
					}
					else if(bearing <= -4)
					{
						bearing -= 25;
					}

					if(Math.abs(offset) < ANGLE_ERROR_MARGIN)
						goStraight(power);
					else
						tiltTo(power, (int)(bearing));
				}
			}
	}
	private static class IRMonitorThread implements Runnable {
		public void run() {
			while (true) {
				currentReading = getDataFromSensor();
				targetReading = frontSensor.getBearingFromBeacon(1);
				
			}
		}
	}
	public static void main(String[] args) {
		Button.LEDPattern(2);	//solid red
		try {
			//Begin gyro reset procedure
			sensor.reset();
			Thread.sleep(1500);	//reset delay
			getRateDataFromSensor();
			getDataFromSensor();
			Thread.sleep(4000);
			//End gyro reset procedure
		} catch (Exception e) {e.printStackTrace();}
		Button.LEDPattern(1);	//solid green
		new Thread(new IRMonitorThread()).start();
		try {
			Thread.sleep(250);
		} catch (InterruptedException e) {
			e.printStackTrace(System.out);
		}
		new Thread(new MovementThread(10, 0)).start();
		while(true)
		{
			Button.waitForAnyPress();
			System.exit(0);
		}
	}
	
}
