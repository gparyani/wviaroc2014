package testing.sensors;

import lejos.hardware.port.Port;
import lejos.hardware.sensor.NXTUltrasonicSensor;
import lejos.robotics.SampleProvider;

public class NXTUltrasonicSideSensor extends SideSensor
{
	private NXTUltrasonicSensor sensor;
	private SampleProvider provider;
	
	public NXTUltrasonicSideSensor(Port p)
	{
		sensor = new NXTUltrasonicSensor(p);
		sensor.enable();
		provider = sensor.getContinuousMode();
	}

	@Override
	public float getDistanceInCM()
	{
		float[] data = new float[1];
		provider.fetchSample(data, 0);
		return data[0] * 100;
	}
}
