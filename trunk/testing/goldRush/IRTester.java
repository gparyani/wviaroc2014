package testing.goldRush;

import lejos.hardware.Button;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3IRSensor;
import lejos.robotics.SampleProvider;

public class IRTester {

	public static void main(String[] args) {
		try(EV3IRSensor sensor = new EV3IRSensor(SensorPort.S4))
		{
			SampleProvider provider = sensor.getSeekMode();
			System.out.println("Bearing\tDistance");
			while(!Button.ESCAPE.isDown())
			{
				float[] samples = new float[8];
				provider.fetchSample(samples, 0);
				System.out.println(samples[0] + "\t" + samples[1]);
			}
		}
	}

}
