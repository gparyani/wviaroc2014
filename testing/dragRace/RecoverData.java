package testing.dragRace;
import static testing.rac3Truck.Rac3TruckMovement.*;
import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.hardware.port.SensorPort;
import lejos.robotics.SampleProvider;
import testing.rac3Truck.Rac3TruckSteering;
import testing.sensors.*;
import java.util.*;
import java.util.concurrent.*;
public class RecoverData {
	private static ResettableGyroSensor sensor/* = new ResettableGyroSensor(SensorPort.S1)*/;
	private static SampleProvider gyro/* = sensor.getAngleMode()*/;
	private static SampleProvider rgyro/* = sensor.getRateMode()*/;
	private static volatile float offset;
	private static int power;
	private static volatile float targetReading, currentReading;
	private static Thread forwardThread;
	private static volatile boolean stalled;
	private static final double X_ORIGINAL_VALUE = 0.0, Y_ORIGINAL_VALUE = 20.0;
	private static volatile double x = X_ORIGINAL_VALUE, y = Y_ORIGINAL_VALUE; //Center of all sensors
	//End variable declarations
	
	private static float getDataFromSensor()
	{
		float[] data = new float[1];	//an array is necessary to get the data
		gyro.fetchSample(data, 0);
		return -1 * data[0];
	}
	
	private static float getRateDataFromSensor()
	{
		float[] data = new float[1];	//an array is necessary to get the data
		rgyro.fetchSample(data, 0);
		return data[0];
	}
	
	/**
	 * Represents a cell within a maze. A maze is composed of several cells.
	 *
	 */

//		@SuppressWarnings("deprecation")	//Thread.stop() is deprecated
	public static void main(String[] args) throws Throwable
	{
		Button.LEDPattern(2);	//turn button backlight red

//			sensor.hardReset();
		
		//Begin gyro reset procedure
		sensor = new ResettableGyroSensor(SensorPort.S1);
		gyro = sensor.getAngleMode();
		rgyro = sensor.getRateMode();
		Thread.sleep(750);	//allow user enough time to let go of robot
		sensor.reset();
		Thread.sleep(1500);	//reset delay
		getRateDataFromSensor();
		getDataFromSensor();
		Thread.sleep(4000);
		//End gyro reset procedure
		
		Rac3TruckSteering.reset();
		Button.LEDPattern(1);	//turn button backlight green

		System.out.println("Program starting.");
		Button.waitForAnyPress();

		goStraight(100);	//Begin moving robot
		forwardThread = new Thread(new MovementThread(100, 0));
		//new Thread(new MonitorThread()).start();
		
		forwardThread.start();
		//Check for robot crashes into obstacles
		while(true)
		{
			stalled = isStalled();	//handled in movement thread
		}		
	}
	private static class MovementThread implements java.lang.Runnable
	{
		MovementThread(int _power, float _targetReading)
		{
			power = _power;
			targetReading = _targetReading;
		}
		
		public void run()
		{
			for(/*long currentTime = System.currentTimeMillis()*/; true;)
			{
				currentReading = getDataFromSensor();
				offset = currentReading - targetReading;
				int bearing = (int) (offset/1.1);
				int turnAngle = -1 * bearing;
				
				//Steering needs to be at least 40 to have some effect
				if( turnAngle <= -4 )
				{
					turnAngle  -= 25;
					bearing += 25;
				}
				else if( turnAngle >= 4 )
				{
					turnAngle  += 25;
					bearing -= 25;
				}

				if(stalled)
				{
//						System.out.print("Stalled\t");
					sensor.close();
					System.exit(0);
				}
				
				if(Math.abs(offset) >= 45)	//too much to the sides
				{
		//			System.out.print("Big Offset \t");
					tiltTo(power, turnAngle);
//						action = "Go Turn " + bearing ;
				}
				else if(Math.abs(offset) >= 5)	//too much to the left
				{
			//		System.out.print("small Offset\t");
					tiltTo(power, turnAngle);
				}
				else
				{
				//	System.out.print("Straight\t");
					goStraight(power);
				}

//					LCD.clearDisplay();	//we don't want the LCD to be cluttered with output			
				
			}
		}
	}
}
