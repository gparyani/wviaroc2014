package testing;

import lejos.hardware.Button;
import lejos.hardware.port.SensorPort;
import testing.sensors.ArduinoSideSensors;
import testing.sensors.SideSensor;

public class ArduinoSideSensorsTest {

	public static void main(String[] args) {
		ArduinoSideSensors sensorSet = new ArduinoSideSensors(SensorPort.S2, 4);
		SideSensor left = sensorSet.getLeftSideSensor(), right = sensorSet.getRightSideSensor();
		new Thread(new Runnable()
				{
					public void run()
					{
						Button.waitForAnyPress();
						System.exit(0);
					}
				}).start();
		while(true)
		{
			System.out.println("Left: " + left.getDistanceInCM() + "\tRight: " + right.getDistanceInCM());
//			try {
//				Thread.sleep(10);
//			} catch (InterruptedException e) {
//				// TODO Auto-generated catch block
//				e.printStackTrace();
//			}
		}
	}

}
