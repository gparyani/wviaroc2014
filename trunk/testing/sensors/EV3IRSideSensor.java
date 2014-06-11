package testing.sensors;

import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3IRSensor;
import lejos.robotics.SampleProvider;

public class EV3IRSideSensor extends SideSensor
{
	private EV3IRSensor detector; 
	private SampleProvider sensor;
	
	public EV3IRSideSensor(Port port)
	{
		detector = new EV3IRSensor(port);
		sensor = detector.getDistanceMode();
	}
	
	@Override
	public float getDistanceInCM()
	{
		float[] data = new float[sensor.sampleSize()];
		sensor.fetchSample(data, 0);
		return calibrateIRDistance(data[0]);
	}
	
	private float calibrateIRDistance(float percentage)
	{
		if(percentage > 50)
			return Float.POSITIVE_INFINITY;
		else
			return (float) (0.0078 * percentage * percentage + 0.3047 * percentage + 2.9338);
	}
}