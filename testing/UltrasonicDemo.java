package testing;

import lejos.hardware.Sound;
import lejos.hardware.port.SensorPort;
import testing.sensors.ArduinoSideSensors;
import testing.sensors.EV3IRSideSensor;
import testing.sensors.SideSensor;

public class UltrasonicDemo {

	public static void main(String[] args) {
		ArduinoSideSensors sensorSet = new ArduinoSideSensors(SensorPort.S2, 4);
		SideSensor leftSensor = sensorSet.getLeftSideSensor();
		SideSensor rightSensor = sensorSet.getRightSideSensor();
		SideSensor frontSensor = new EV3IRSideSensor(SensorPort.S4);
		Sound.setVolume(30);
		int[] PIANO = {4, 25, 500, 7000, 5};
		while(true)
		{
			Sound.playNote(PIANO, (int)leftSensor.getDistanceInCM() * 3 + 300, (int)frontSensor.getDistanceInCM() * 6);
			Sound.playNote(PIANO, (int)rightSensor.getDistanceInCM() * 3 + 300, (int)frontSensor.getDistanceInCM() * 6);
		}

	}

}
