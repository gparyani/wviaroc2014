package testing.sensors;

import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3IRSensor;
import lejos.robotics.SampleProvider;

public class EV3IRSideSensor extends SideSensor
{
	private EV3IRSensor detector; 
	private SampleProvider distance, seek;
	
	public EV3IRSideSensor(Port port)
	{
		detector = new EV3IRSensor(port);
		distance = detector.getDistanceMode();
		seek = detector.getSeekMode();
	}
	
	public synchronized float getBearingFromBeacon(int channel)
	{
		channel -= 1;
		channel *= 2;
		float[] data = new float[8];
		seek.fetchSample(data, 0);
		return data[channel];
	}
	
	public synchronized float getDistanceFromBeacon(int channel)
	{
		channel -= 1;
		channel *= 2;
		channel += 1;
		float[] data = new float[8];
		seek.fetchSample(data, 0);
		return data[channel];
	}
	
	@Override
	public synchronized float getDistanceInCM()
	{
		float[] data = new float[distance.sampleSize()];
		distance.fetchSample(data, 0);
		return calibrateIRDistance(data[0]);
	}
	
	private float calibrateIRDistance(float percentage)
	{
		if(percentage > 50)
			return Float.POSITIVE_INFINITY;
		else
			return 1.65f * (float) (0.0078 * percentage * percentage + 0.3047 * percentage + 2.9338);
//			return (float)(2E-05 * Math.pow(percentage, 4) - 0.0023 * Math.pow(percentage, 3) + 0.0739 * percentage * percentage - 0.193 * percentage + 2.9876);
	}
}