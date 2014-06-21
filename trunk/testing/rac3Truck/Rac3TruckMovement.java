package testing.rac3Truck;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;

/**
 * Contains methods to move the robot. The {@code reset()} method of class {@code Rac3TruckSteering} will be called during class initialization.
 */
public class Rac3TruckMovement {
	
	private static EV3LargeRegulatedMotor left = new EV3LargeRegulatedMotor(MotorPort.B), right = new EV3LargeRegulatedMotor(MotorPort.C);
	
	static
	{
		if(!Rac3TruckSteering.resetCalled)
			Rac3TruckSteering.reset();
	}
	
	/**
	 * Helper method for checking the range of power values passed into the other methods.
	 * @param power the power value to be checked
	 * @throws java.lang.IllegalArgumentException if {@code power} is not within the range 1-100
	 */
	private static void checkRange(int power)
	{
		if(power < 1 || power > 100)
			throw new IllegalArgumentException("Invalid value for power: " + power);
	}
	
	/**
	 * Helper method to calculate the percentage of a power into the degrees per second velocity of the motor.
	 * @param power the power value
	 * @param percent the percentage to scale on
	 * @return the given power scaled into a given percent
	 */
	private static float percentageScale(float power, float percent)
	{
		return (power / 100) * (percent / 100) * left.getMaxSpeed();
	}
	
	/**
	 * Causes the robot to begin going straight with the specified power value.
	 * @param power a value from 1 to 100 indicating the power value to be used
	 * @throws java.lang.IllegalArgumentException if {@code power} is outside the specified range
	 */
	public static void goStraight(int power)
	{
		checkRange(power);
		Rac3TruckSteering.center();
		left.setSpeed(-1 * percentageScale(power, 100));
		right.setSpeed(percentageScale(power, 100));
		left.forward();
		right.forward();
	}
	
	/**
	 * Causes the robot to begin going backwards with the specified power value.
	 * @param power a value from 1 to 100 indicating the power value to be used
	 * @throws java.lang.IllegalArgumentException if {@code power} is outside the specified range
	 */
	public static void goBackwards(int power)
	{
		goBackwards(power, 0);
	}
	
	public static void goBackwards(int power, int bearing)
	{
		checkRange(power);
		Rac3TruckSteering.turnTo(bearing);
		left.setSpeed(-1 * percentageScale(power, 100));
		right.setSpeed(percentageScale(power, 100));
		left.backward();
		right.backward();

	}
	
	public static void goBackwards(int power, int bearing, int time)
	{
		left.stop();
		right.stop();
		goBackwards(power, bearing);
		try {
			Thread.sleep(time);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
	}
	
	public static void goForwards(int power, int bearing, int time)
	{
		left.stop();
		right.stop();
		tiltTo(power, bearing);
		try {
			Thread.sleep(time);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
	}
	/**
	 * Causes the robot to begin turning left with the specified power value. The steering wheels will be used.
	 * @param power a value from 1 to 100 indicating the power value to be used
	 * @throws java.lang.IllegalArgumentException if {@code power} is outside the specified range
	 */
	public static void goLeft(int power)
	{
		checkRange(power);
		Rac3TruckSteering.turnLeft();
		left.setSpeed(-1 * percentageScale(power, 100));
		right.setSpeed(percentageScale(power, 100));
		left.forward();
		right.forward();
	}
	
	public static void tiltLeft(int power)
	{
		checkRange(power);
		Rac3TruckSteering.tiltLeft();
		left.setSpeed(-1 * percentageScale(power, 100));
		right.setSpeed(percentageScale(power, 100));
		left.forward();
		right.forward();
	}
	
	/**
	 * Causes the robot to begin turning right with the specified power value. The steering wheels will be used.
	 * @param power a value from 1 to 100 indicating the power value to be used
	 * @throws java.lang.IllegalArgumentException if {@code power} is outside the specified range
	 */
	public static void goRight(int power)
	{
		checkRange(power);
/*		left.stop();
		right.stop();	
		Rac3TruckSteering.turnRight();
		while(true);
*/	

		Rac3TruckSteering.turnRight();
		left.setSpeed(-1 * percentageScale(power, 100));
		right.setSpeed(percentageScale(power, 100));
		left.forward();
		right.forward();
	}
	
	public static void tiltRight(int power)
	{
		checkRange(power);
		Rac3TruckSteering.tiltRight();
		left.setSpeed(-1 * percentageScale(power, 100));
		right.setSpeed(percentageScale(power, 100));
		left.forward();
		right.forward();
	}
	
	public static void tiltTo(int power, int bearing)
	{
		checkRange(power);
		Rac3TruckSteering.turnTo(bearing);
		left.setSpeed(-1 * percentageScale(power, 100));
		right.setSpeed(percentageScale(power, 100));
		left.forward();
		right.forward();
	}
	
	/**
	 * Causes the robot to begin turning back left with the specified power value. The steering wheels will be used.
	 * @param power a value from 1 to 100 indicating the power value to be used
	 * @throws java.lang.IllegalArgumentException if {@code power} is outside the specified range
	 */
	public static void backLeft(int power)
	{
		goBackwards(power, 75);
	}
	
	/**
	 * Causes the robot to begin turning back right with the specified power value. The steering wheels will be used.
	 * @param power a value from 1 to 100 indicating the power value to be used
	 * @throws java.lang.IllegalArgumentException if {@code power} is outside the specified range
	 */
	public static void backRight(int power)
	{
		goBackwards(power, -75);
	}
	
	/**
	 * Causes the robot to stop.
	 */
	public static void stopTruck()
	{
		left.stop();
		right.stop();
		Rac3TruckSteering.center();
	}
	
	public static synchronized int getLeftTachoCount()
	{
		return left.getTachoCount();
	}
	
	public static synchronized int getRightTachoCount()
	{
		return right.getTachoCount();
	}
	
	public static synchronized void resetAllTachoCounts()
	{
		right.resetTachoCount();
		left.resetTachoCount();
	}
	
	public static boolean isStalled()
	{
		return left.isStalled() || right.isStalled();
	}
}
