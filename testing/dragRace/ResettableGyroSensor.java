package testing.dragRace;

import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3GyroSensor;

public class ResettableGyroSensor extends EV3GyroSensor
{
	public ResettableGyroSensor(Port port)
	{
		super(port);
	}
	
	public void hardReset()
	{
		switchMode(-1, 0);
	}
}