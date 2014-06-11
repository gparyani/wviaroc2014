package testing.sensors;

import lejos.hardware.port.Port;
import lejos.robotics.SampleProvider;

public class EV3UltrasonicSideSensor extends SideSensor {
	
	private lejos.hardware.sensor.EV3UltrasonicSensor sensor;
	private SampleProvider distanceProvider;
	
	public EV3UltrasonicSideSensor(Port port)
	{
		sensor = new lejos.hardware.sensor.EV3UltrasonicSensor(port);
		sensor.enable();
		distanceProvider = sensor.getDistanceMode();
	}

	@Override
	public float getDistanceInCM() {
		float[] data = new float[distanceProvider.sampleSize()];
		distanceProvider.fetchSample(data, 0);
		return data[0]* 100f;
	}
	
	@Override
	protected void finalize()	//once we are being deleted by the GC or the program ends
	{
		sensor.disable();
	}
}
