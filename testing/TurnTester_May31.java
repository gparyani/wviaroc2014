package testing;

import static testing.rac3Truck.Rac3TruckMovement.*;
import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.hardware.port.SensorPort;
import lejos.robotics.SampleProvider;
import testing.rac3Truck.Rac3TruckSteering;
import testing.sensors.*;
import java.util.*;

public class TurnTester_May31
{
	private static SideSensor leftSensor = new EV3UltrasonicSideSensor(SensorPort.S2);
	private static SideSensor frontSensor = new EV3IRSideSensor(SensorPort.S4);
//	private static SideSensor rightSensor = new NXTUltrasonicSideSensor(SensorPort.S3);
	private static SideSensor rightSensor = new EV3IRSideSensor(SensorPort.S3);
	private static ResettableGyroSensor sensor = new ResettableGyroSensor(SensorPort.S1);
	private static SampleProvider gyro = sensor.getAngleMode();
	private static SampleProvider rgyro = sensor.getRateMode();
	private static volatile float leftReading, frontReading, rightReading, offset;
	private static int power;
	private static volatile float targetReading, currentReading;
	private static Thread forwardThread;
	private static volatile boolean stalled, leftWall, frontWall, rightWall, isBacking;
	private static volatile double x = -40.0, y = 15.0;
	private static volatile int lwLastRead, rwLastRead, tachoCount;
	private static Deque<Cell> coordinateList;
	
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
	
	private static class Cell
	{
		private int x, y;
		private boolean west, north, south, east;
		
		Cell(int x, int y)
		{
			this.x = x;
			this.y = y;
		}

		int getX() {
			return x;
		}

		int getY() {
			return y;
		}

		boolean isWest() {
			return west;
		}

		void setWest(boolean west) {
			this.west = west;
		}

		boolean isNorth() {
			return north;
		}

		void setNorthWall(boolean north) {
			this.north = north;
		}

		boolean southWallExists() {
			return south;
		}

		void setSouthWall(boolean south) {
			this.south = south;
		}
		
		boolean eastWallExists() {
			return east;
		}
		
		void setEastWall(boolean east) {
			this.east = east;
		}
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
		Thread.sleep(4000);
		Rac3TruckSteering.reset();
		Button.LEDPattern(1);	//turn button backlight green

		System.out.println("Program starting.");
		//Button.waitForAnyPress();

		goStraight(20);
		forwardThread = new Thread(new LatchThread(20, 0));
		new Thread(new MonitorThread()).start();

		new Thread(new Runnable() {
			public void run()
			{
				Button.waitForAnyPress();
				System.exit(0);
			}
		}).start();
		forwardThread.start();
		while(true)
		{
			if(stalled = isStalled())	//once the robot has crashed into an obstacle
			{
//				System.out.println("Robot stalled");
			}
		}		
	}


	
	private static class MonitorThread implements Runnable
	{		
		public void run()
		{
			boolean isTurning = false;
			int change;
			double deltaX, deltaY;
			
			tachoCount = getRightTachoCount();

			while(true)
			{
				change = getRightTachoCount() - tachoCount;
				tachoCount += change;
				
				leftReading = leftSensor.getDistanceInCM();
				frontReading = frontSensor.getDistanceInCM();
				rightReading = rightSensor.getDistanceInCM();
				
				leftWall = leftReading < 40;
				frontWall = frontReading < 24;
				rightWall = rightReading < 34;
				
				
				deltaX = -1 * change * Math.sin(Math.toRadians(currentReading));
				deltaY = change * Math.cos(Math.toRadians(currentReading));
				
				x += deltaX * 29.5 * 2.5 / 360.0;
				y += deltaY * 29.5 * 2.5 / 360.0;
				
				if( isBacking )
				{
//					System.out.println( "isBacking");
					if( !rightWall )
					{
						isTurning = true;
						rwLastRead = tachoCount;
						targetReading -= 90;
						isBacking = false;
					}
/*
					else if( !leftWall )
					{
						isTurning = true;
						lwLastRead = tachoCount;
						targetReading += 90;
						isBacking = false;
					}
*/
				}
				else if(!isTurning)
				{
					if(!leftWall)
					{
						isTurning = true;
						lwLastRead = tachoCount;
						targetReading += 90;
					}
					else if(frontWall && !rightWall)
					{
						isTurning = true;
						rwLastRead = tachoCount;
						targetReading -= 90;
					}
					else if(frontWall && rightWall && leftWall)
					{
//						isTurning = true;
						isBacking = true;
//						targetReading -= 180;
					}
					
//					if( isTurning)
//						System.out.println(x + "\t" + y + "\t" + (int)((x)/ 60) + "\t" + (int)((y)/ 60));
				}
				offset = currentReading - targetReading;
				if(offset >= -5 && offset <= 5)
				{
//					System.out.println(x + "\t" + y + "\t" + (int)((x)/ 62.5) + "\t" + (int)((y)/ 62.5));
					isTurning = false;
				}
				System.out.println("Monitor\t" + leftReading + "\t" + frontReading +
						"\t" + rightReading + "\t" + tachoCount +"\t => target " + targetReading);
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
			boolean alreadyWentBackTurn = false,
					alreadyWentBack = false;

//			System.out.println("Time\tTacho\tOffset\tAction\tBearing");
			for(long currentTime = System.currentTimeMillis(); true;)
			{
				String action;
				
				currentReading = getDataFromSensor();
				offset = currentReading - targetReading;

				System.out.print("Latch T = " + (System.currentTimeMillis() - currentTime) + "\t Steer = " + Rac3TruckSteering.getTachoCount()
						+ "\t offset = " + offset );	
				
				if(offset <= 5 || offset >= -5) //When going straight forward/back
				{
					if( rightReading < 12 )
					{
						offset -= (2*(12-rightReading) );
						System.out.print("-R" + rightReading + "\t");
					}
					else if( leftReading < 12 )
					{
						offset += (2*(12-leftReading));
						System.out.print("-L" + leftReading + "\t");
					}
				}
				int bearing = (int) (offset/1.25);
				int turnAngle = -1 * bearing;
				int backOff = 1000;
				
				//Steering needs to be at least 40 to have some effect
				if( turnAngle <= -4 )
				{
					turnAngle  -= 30;
					bearing += 30;
				}
				else if( turnAngle >= 4 )
				{
					turnAngle  += 30;
					bearing -= 30;
				}

				if(stalled)
				{
					System.out.print("Stalled\t");
					if( !isBacking ) {
						System.out.print("Forward&Back\t");
						goBackwards(power - 5, bearing, backOff);
						goForwards( power-5, -bearing, backOff);
					}
					else
						isBacking = false;
				}
				if( isBacking )
				{
					Button.LEDPattern(3);	//button backlight orange
					if(!alreadyWentBack)
					{
						System.out.print("Start Backing\t");
						stopTruck();

						alreadyWentBack = true;
					}
					goBackwards(power - 5, bearing);
					System.out.print("isBacking\n");
					continue;
				}
				else
				{
					Button.LEDPattern(1);	//button backlight green
					alreadyWentBack = false;
				}
				if(offset >= 60 || offset <= -60)	//too much to the left
				{
					if(!alreadyWentBackTurn)
					{
						int temp = (offset >= 60) ? rwLastRead : lwLastRead;

						int now = getRightTachoCount();
						if( !frontWall && ((now - temp) < 5))
							backOff = 500;
						
						System.out.print( "Turn : " + turnAngle + "Backoff\t" + backOff + "now&temp\t" + now + temp );
						goBackwards(power - 5, bearing, backOff);
						alreadyWentBackTurn = true;
					}
					System.out.print("Big Offset \t");
					tiltTo(power, turnAngle);
					action = "Go Turn " + bearing ;
				}
				else if(offset >= 5 || offset <= -5)	//too much to the left
				{
					System.out.print("small Offset\t");
					tiltTo(power, turnAngle);
					action = "Tilt " + bearing;
					alreadyWentBackTurn = false;
				}
				else
				{
					System.out.print("Staright\t");
					goStraight(power);
					action = "Straight";
					alreadyWentBackTurn = false;
				}

				System.out.print("\n");	
//				LCD.clearDisplay();	//we don't want the LCD to be cluttered with output			
				
			}
		}
	}
}