package testing.rac3Truck;

import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.MotorPort;

/**
 * Contains methods to control the steering wheels of the robot. The {@code reset()} method of this class must have been invoked at least once to use the methods of this class.
 */
public class Rac3TruckSteering {

	private static EV3MediumRegulatedMotor steering = new EV3MediumRegulatedMotor(MotorPort.A);
	/*package private*/ static boolean resetCalled = false;
	
	/**
	 * Resets the position of the steering wheels. This method must have been invoked at least once to use any of the other methods.
	 */
	public static void reset()
	{
		//System.out.println("Moving right");	//TODO debug statement
			steering.forward();
			try {
				Thread.sleep(1500);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
			steering.stop();
		//	System.out.println("Should be right");	//TODO debug statement
			
			steering.rotate(-160);
			steering.resetTachoCount();
			resetCalled = true;
		//	System.out.println("Steering reset");	//TODO debug statement
	}
	
	private static void checkResetCalled()
	{
		if(!resetCalled)
			throw new IllegalStateException("You must call reset() before you can use this class.");
	}
	
	/**
	 * Turns the steering wheels left. The {@code reset()} method must have been called at least once.
	 * @throws java.lang.IllegalStateException if {@code reset()} has not yet been called
	 */
	public static void turnLeft()
	{
		checkResetCalled();
		steering.rotateTo(90, true);
	}
	
	//left tilt seems very strong, but...
	private static int leftTiltBearing = 30;
	private static final int LEFT_ORIGINAL_VALUE = leftTiltBearing;
	
	public static void tiltLeft()
	{
		checkResetCalled();
		steering.rotateTo(leftTiltBearing <= (LEFT_ORIGINAL_VALUE + 20) ? leftTiltBearing : leftTiltBearing++, true);
	}
	
	//...right tilt is too weak
	private static int rightTiltBearing = -30;
	private static final int RIGHT_ORIGINAL_VALUE = rightTiltBearing;
	
	public static void tiltRight()
	{
		checkResetCalled();
		steering.rotateTo(rightTiltBearing >= (RIGHT_ORIGINAL_VALUE - 20) ? rightTiltBearing : rightTiltBearing--, true);
	}
	
	/**
	 * Turns the steering wheels right. The {@code reset()} method must have been called at least once.
	 * @throws java.lang.IllegalStateException if {@code reset()} has not yet been called
	 */
	public static void turnRight()
	{
		checkResetCalled();
		steering.rotateTo(-90, true);
	}
	
	public static void turnTo(int turnDegree)
	{
		checkResetCalled();
		if(turnDegree < -180)
			turnDegree = -180;
		else if(turnDegree > 180)
			turnDegree = 180;
		steering.rotateTo(turnDegree, true);
	}
	
	/**
	 * Centers the steering wheels. The {@code reset()} method must have been called at least once.
	 * @throws java.lang.IllegalStateException if {@code reset()} has not yet been called
	 */
	public static void center()
	{
		checkResetCalled();
		steering.rotateTo(0, true);
		leftTiltBearing = LEFT_ORIGINAL_VALUE;
		rightTiltBearing = RIGHT_ORIGINAL_VALUE;
	}
	
	public static int getTachoCount()
	{
		return steering.getTachoCount();
	}
}
