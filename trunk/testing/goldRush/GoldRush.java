package testing.goldRush;
import java.util.ArrayDeque;
import java.util.Deque;
import java.util.Queue;

import testing.ResettableGyroSensor;
import testing.rac3Truck.Rac3TruckMovement;
import testing.rac3Truck.Rac3TruckSteering;
import testing.sensors.EV3IRSideSensor;
import testing.sensors.EV3UltrasonicSideSensor;
import testing.sensors.SideSensor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3IRSensor;
import lejos.robotics.SampleProvider;

public class GoldRush {
	private static SideSensor leftSensor = new EV3UltrasonicSideSensor(SensorPort.S2);
	private static SideSensor frontSensor = new EV3IRSideSensor(SensorPort.S4);
	private static SideSensor rightSensor = new EV3UltrasonicSideSensor(SensorPort.S3);
	private static ResettableGyroSensor sensor = new ResettableGyroSensor(SensorPort.S1);
	private static SampleProvider gyro = sensor.getAngleMode();
	private static volatile float leftReading, frontReading, rightReading, offset;
	private static int power;
	private static volatile float targetReading, currentReading;
	private static volatile boolean stalled, leftWall, frontWall, rightWall, isBacking, isTurning;
	private static final double X_ORIGINAL_VALUE = 0.0, Y_ORIGINAL_VALUE = 20.0;
	private static volatile double realX = 0, realY = 1;
	private static volatile double x = X_ORIGINAL_VALUE, y = Y_ORIGINAL_VALUE; //Center of all sensors
	private static volatile int rTachoCount, lTachoCount;
	private static Queue<Float> leftValues = new ArrayDeque<Float>(), frontValues = new ArrayDeque<Float>(), rightValues = new ArrayDeque<Float>();
	private static final int ANGLE_ERROR_MARGIN = 15;
	private static final float CELL_WIDTH = 62.5f;
	
	private enum State {
		CALIBRATING, SEEKING_BEACON, MOVING_TOWARDS_BEACON, AVOIDING_OBSTACLE
	}
	
	/**
	 * Controls the movements of the robot
	 * @author Robert
	 *
	 */
	public class MovementThread implements Runnable {
		MovementThread(int _power, float _targetReading)
		{
			power = _power;
			targetReading = _targetReading;
		}
		public void run() {
			while(true) {
				
			}
		}
	}
	public class IRMonitorThread implements Runnable {
		public void run() {
			while (true) {
				//frontSensor;
			}
		}
	}
	public static void main(String[] args) {
		
	}
}
