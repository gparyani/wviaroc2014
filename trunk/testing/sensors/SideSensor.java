package testing.sensors;

public abstract class SideSensor
{
	public abstract float getDistanceInCM();
	
	public float getDistanceInInches()
	{
		return getDistanceInCM() / 2.54f;
	}
}
