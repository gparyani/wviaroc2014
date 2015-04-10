/**
 * 
 */
package testing.sensors;

import lejos.hardware.port.I2CPort;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.I2CSensor;

/**
 * Interfaces with the Ping))) ultrasonic side sensors connected using an Arduino.
 * @author Gaurav Paryani
 *
 */
public class ArduinoSideSensors {

	private I2CPort port;
	private volatile int leftSensorValue;
	private volatile int rightSensorValue;
	private int address;
	private SideSensor leftSideSensor = new LeftSideSensor(),
			rightSideSensor = new RightSideSensor();

	public ArduinoSideSensors(Port port, int address)
	{
		this.port = (new I2CSensor(port)).getPort();
		this.address = address << 1;
		Thread t = new Thread(new ReadingThread());
		t.setDaemon(true);	//don't hang the program if calling code exits
		t.start();
	}
	
	public SideSensor getLeftSideSensor()
	{
		return leftSideSensor;
	}
	
	public SideSensor getRightSideSensor()
	{
		return rightSideSensor;
	}
	
	private class ReadingThread implements Runnable
	{
		byte[] values = new byte[8],
				emptyArray = {1};
		
		public void run() {
			while(true)
			{
				port.i2cTransaction(address, emptyArray, 0, 1, values, 0, 8);
				System.out.println(java.util.Arrays.toString(values));
				leftSensorValue = toUnsignedInt(values[0]);	//values are returned from Arduino as unsigned values
				rightSensorValue = toUnsignedInt(values[1]);
			}
		}
	}
	
	private static int toUnsignedInt(byte x)
	{
		return ((int) x) & 0xff;
	}
	
	private class LeftSideSensor extends SideSensor
	{
		@Override
		public float getDistanceInCM() {
			return leftSensorValue;
		}
	}
	
	private class RightSideSensor extends SideSensor
	{
		@Override
		public float getDistanceInCM() {
			return rightSensorValue;
		}
	}
}
