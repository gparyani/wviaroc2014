package testing.rac3Truck;

import lejos.hardware.Button;
import lejos.hardware.Key;
import lejos.hardware.KeyListener;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.NXTUltrasonicSensor;

public class LeftMazeFollower
{
	private static EV3UltrasonicSensor leftSensor = new EV3UltrasonicSensor(SensorPort.S2);
	private static NXTUltrasonicSensor rightSensor = new NXTUltrasonicSensor(SensorPort.S3);
	
	public static void main(String[] args)
	{
		Button.ESCAPE.addKeyListener(new EscapeListener());
		while(true)
		{
			
		}
	}
	
	private static class EscapeListener implements KeyListener
	{
		public void keyPressed(Key k) {
			System.exit(0);
		}

		public void keyReleased(Key k) {
		}
	}
	
	private static boolean isCloseToLeftWall()
	{
		//TODO Implement method
		return false;
	}
}
