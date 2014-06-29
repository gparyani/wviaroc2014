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
	
	private float calibrateIRDistance(float x)
	{
		if(x > 80)
			return Float.POSITIVE_INFINITY;
		else
			return ((x < 1) ? 2f : 
					(0.0002f*x*x*x - 0.0116f*x*x + 0.6458f*x + 3.856f));
			
//			return ((percentage == 1) ? 2f : (0.0087f * percentage * percentage + 0.3f * percentage + 4.74f));
//			return 1.25f * (float) (0.0078 * percentage * percentage + 0.3047 * percentage + 2.9338);
//			return (float)(2E-05 * Math.pow(percentage, 4) - 0.0023 * Math.pow(percentage, 3) + 0.0739 * percentage * percentage - 0.193 * percentage + 2.9876);
//			return ((percentage == 1) ? 2.0f : (0.5804f * percentage + 3.6804f));
	}
}