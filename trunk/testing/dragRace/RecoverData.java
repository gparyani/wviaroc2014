package testing.dragRace;
import static testing.rac3Truck.Rac3TruckMovement.*;
import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.hardware.port.SensorPort;
import lejos.robotics.SampleProvider;
import testing.rac3Truck.Rac3TruckSteering;
import testing.sensors.*;
import java.util.*;
import java.util.concurrent.*;
public class RecoverData {
	private static SideSensor leftSensor = new EV3UltrasonicSideSensor(SensorPort.S2);
	private static SideSensor frontSensor = new EV3IRSideSensor(SensorPort.S4);
//	private static SideSensor rightSensor = new NXTUltrasonicSideSensor(SensorPort.S3);
	private static SideSensor rightSensor = new EV3IRSideSensor(SensorPort.S3);
	private static ResettableGyroSensor sensor = new ResettableGyroSensor(SensorPort.S1);
	private static SampleProvider gyro = sensor.getAngleMode();
	private static SampleProvider rgyro = sensor.getRateMode();
	private static Direction front;
	private static volatile float leftReading, frontReading, rightReading, offset;
	private static int power;
	private static volatile float targetReading, currentReading;
	private static Thread forwardThread;
	private static volatile boolean stalled, leftWall, frontWall, rightWall, isBacking, isTurning;
	private static final double X_ORIGINAL_VALUE = 0.0, Y_ORIGINAL_VALUE = 20.0;
	private static volatile double x = X_ORIGINAL_VALUE, y = Y_ORIGINAL_VALUE; //Center of all sensors
	private static volatile int lwLastRead, rwLastRead, tachoCount;
	private static Deque<Cell> maze = new ArrayDeque<Cell>();
	private static final int ANGLE_ERROR_MARGIN = 5;
	private static final double CELL_WIDTH = 62.5;
	private static volatile int xCoordinate, yCoordinate;
	//End variable declarations
	
	private static float getDataFromSensor()
	{
		float[] data = new float[1];	//an array is necessary to get the data
		gyro.fetchSample(data, 0);
		return data[0];
	}
	
	private static float getRateDataFromSensor()
	{
		float[] data = new float[1];	//an array is necessary to get the data
		rgyro.fetchSample(data, 0);
		return data[0];
	}
	
	private static enum Direction
	{
		NORTH, EAST, SOUTH, WEST, IN_BETWEEN;
		
		/**
		 * Gets the Direction at which the robot is facing.
		 * @param reading the reading from the gyro
		 * @return the Direction object
		 */
		static Direction getDirectionFromGyro(int reading)
		{
			if(reading > 0)
				reading %= 360;	//ensure that we are dealing only with values from 0 to 359
			else
			{
				while(reading < 0)
					reading += 360;
			}
			
			if(reading >= (90 - ANGLE_ERROR_MARGIN) && reading <= (90 + ANGLE_ERROR_MARGIN))
				return WEST;
			if(reading >= (180 - ANGLE_ERROR_MARGIN) && reading <= (180 + ANGLE_ERROR_MARGIN))
				return SOUTH;
			if(reading >= (270 - ANGLE_ERROR_MARGIN) && reading <= (270 + ANGLE_ERROR_MARGIN))
				return EAST;
			if(reading >= (360 - ANGLE_ERROR_MARGIN) || reading <= ANGLE_ERROR_MARGIN)
				return NORTH;
			return IN_BETWEEN;
		}
		
		Direction getOppositeDirection()
		{
			switch(this) {
			case NORTH:
				return SOUTH;
			case EAST:
				return WEST;
			case SOUTH:
				return NORTH;
			case WEST:
				return EAST;
			}
			return this;	//IN_BETWEEN's opposite direction is also IN_BETWEEN; compiler throws error if in switch statement itself
		}
	}
	
	/**
	 * Represents a cell within a maze. A maze is composed of several cells.
	 *
	 */
	private static class Cell
	{
		private int xPos, yPos;	//the grid location of this cell
		private boolean west, north, south, east;	//the walls surrounding the cell
		
		Cell(int x, int y)
		{
			this.xPos = x;
			this.yPos = y;
		}

		int getX() {
			return xPos;
		}

		int getY() {
			return yPos;
		}

		boolean westWallExists() {
			return west;
		}

		void setWest(boolean west) {
			if(!this.west)
			{
				this.west = west;
				if (west) 
					System.out.println(this);
			}
		}

		boolean northWallExists() {
			return north;
		}

		void setNorth(boolean north) {
			if(!this.north)
			{
				this.north = north;
				if(north) 				
					System.out.println(this);
			}
		}

		boolean southWallExists() {
			return south;
		}

		void setSouth(boolean south) {
			if(!this.south)
			{
				this.south = south;
				if( south ) 				
					System.out.println( this );
			}
		}
		
		boolean eastWallExists() {
			return east;
		}
		
		void setEast(boolean east) {
			if(!this.east) {
				this.east = east;
				if( east ) 				
					System.out.println( this );
			}
		}
		
		@Override
		public boolean equals(Object c)
		{
			if(super.equals(c))
				return true;
			else if(c instanceof Cell)	//returns false if c == null
			{
				Cell cell = (Cell) c;
				return xPos == cell.xPos && yPos == cell.yPos;
			}
			else
				return false;
		}
		
		@Override
		public int hashCode()
		{
			return xPos * 3737 + yPos;
		}
		
		@Override
		public String toString()
		{
			return Direction.getDirectionFromGyro((int)currentReading) + "\t" + xPos + ", " + yPos + "\t(" + x + ", " + y + ")\tNorth: " +
					north + "\tEast: " + east + "\tSouth: " + south + "\tWest: " + west + "\t" + leftReading + "\t" +
							+ frontReading + "\t" + rightReading + "\n";
		}
	}
	
	private static class ResetCoordinates implements Runnable
	{
		public void run()
		{
			System.out.println("Starting solution run");
			x = X_ORIGINAL_VALUE;
			y = Y_ORIGINAL_VALUE;
		}
	}

//		@SuppressWarnings("deprecation")	//Thread.stop() is deprecated
	public static void main(String[] args) throws Throwable
	{
		Button.LEDPattern(2);	//turn button backlight red

//			sensor.hardReset();
		
		//Begin gyro reset procedure
		sensor.reset();
		Thread.sleep(1500);	//reset delay
		getRateDataFromSensor();
		getDataFromSensor();
		Thread.sleep(4000);
		//End gyro reset procedure
		
		Rac3TruckSteering.reset();
		Button.LEDPattern(1);	//turn button backlight green

		System.out.println("Program starting.");
		//Button.waitForAnyPress();

		goStraight(100);	//Begin moving robot
		forwardThread = new Thread(new MovementThread(100, 0));
		//new Thread(new MonitorThread()).start();

		//Check for button presses
		new Thread(new Runnable() {
			public void run()
			{
				checkForPresses:
				if(Button.waitForAnyPress() == Button.ENTER.getId())
				{
					new Thread(new ResetCoordinates()).start();
					break checkForPresses;
				}
				System.exit(0);
			}
		}).start();
		
		forwardThread.start();
		//Check for robot crashes into obstacles
		while(true)
		{
			stalled = isStalled();	//handled in movement thread
		}		
	}

	static boolean isTooCloseToNSBorder(double yCoordinate)	//Near north and south borders of a cell
	{
		double dist = getDistanceFromBorder(Direction.SOUTH, 0, yCoordinate);	//xCoordinate parameter is ignored if NORTH or SOUTH is passed in
		if(isBacking)
			dist += 8;
		return (dist > 52 || dist < 7); //if too close to boundary, skip update
	}
	
	static boolean isTooCloseToEWBorder(double xCoordinate)	//Near east and west borders of a cell
	{
		double dist = getDistanceFromBorder(Direction.WEST, xCoordinate, 0);	//yCoordinate parameter is ignored if EAST or WEST is passed in
		if(isBacking)
			dist += 8;
		return (dist > 52 || dist < 7); //if too close to boundary, skip update
	}
	
	synchronized static double getDistanceFromBorder(Direction border, double xCoord, double yCoord)
	{
		double dist;
		switch(border) {
		case SOUTH:
			dist = y - (yCoord * CELL_WIDTH);	//how far the robot is from the wall behind it
			break;
		case NORTH:
			dist = y - (yCoord * CELL_WIDTH);	//how far the robot is from the wall behind it
			dist = CELL_WIDTH - dist;
			break;
		case EAST:
			dist = x - (xCoord * CELL_WIDTH);
			break;
		case WEST:
			dist = x - (xCoord * CELL_WIDTH);
			dist = CELL_WIDTH - dist;
			break;
		default:
			throw new IllegalArgumentException("Border can't be IN_BETWEEN");
		}
		return dist;
	}
	
	/**
	 * Returns a reference to the cached Cell object corresponding to a specific set of grid coordinates. Creates a new Cell if required.
	 * @param xCoord the x-coordinate of the grid point
	 * @param yCoord the y-coordinate of the grid point
	 * @return the reference to the Cell object
	 */
	private static Cell getCell(int xCoord, int yCoord)
	{
		Cell currentCell = null;
//			boolean wasAdded = false;
		for(Cell element : maze)
		{
			if(element.getX() == xCoord && element.getY() == yCoord)
			{
				currentCell = element;
				break;
			}
		}
		if(currentCell == null)
		{
			currentCell = new Cell(xCoord, yCoord);
			maze.add(currentCell);
//				wasAdded = true;
		}
		else if(!isTurning)
		{

			Cell oldCell = maze.getLast();	//gets the last cell added
			if(!currentCell.equals(oldCell))
				maze.removeLast();	//this means that we visited a cell that was unnecessary and should be removed
			//set virtual walls so that the robot knows one single path through the maze so that we can get through it without wrong turns
			if(currentCell.getX() == oldCell.getX() + 1)	//moved east
				currentCell.setWest(true);
			else if(currentCell.getX() == oldCell.getX() - 1)	//moved west
				currentCell.setEast(true);
			else if(currentCell.getY() == oldCell.getY() + 1)	//moved north
				currentCell.setSouth(true);
			else if(currentCell.getY() == oldCell.getY() - 1)	//moved south
				currentCell.setNorth(true);
		}
		
//			if(wasAdded)
//				System.out.println(maze);
		return currentCell;
	}
	
	
	private static synchronized void updateCurrentLoc()
	{
		int change;
		double deltaX, deltaY;
		
		change = getRightTachoCount() - tachoCount; //All tacho counts on right wheel
		tachoCount += change;
		
		//currentReading is with respect to +y-axis; add 90 so it's w.r.t. +x-axis
		//change vector magnitude and direction to x- and y-components
		//deltaX and deltaY are displacements since last iteration
		//they are in units of MOTOR degrees
		deltaX = change * Math.cos(Math.toRadians(currentReading + 90));
		deltaY = change * Math.sin(Math.toRadians(currentReading + 90));
		
		//update current absolute location
		x += deltaX * 2.5 / 360.0 * 31;	//deltaX * gear ratio / (convert degrees to rotations) * wheel circumference
		y += deltaY * 2.5 / 360.0 * 31;	//deltaY * gear ratio / (convert degrees to rotations) * wheel circumference
		
		//calculate the grid coordinates of the current cell
		xCoordinate = (x > 0) ? (int)((x + (CELL_WIDTH / 2)) / CELL_WIDTH) : (int)((x - (CELL_WIDTH / 2)) / CELL_WIDTH);
		yCoordinate = (int)(y / CELL_WIDTH);
	}
	
	private static class MonitorThread implements Runnable
	{		
		

		public void run()
		{
			tachoCount = getRightTachoCount();

			while(true)
			{

				leftReading = leftSensor.getDistanceInCM();
				frontReading = frontSensor.getDistanceInCM();
				rightReading = rightSensor.getDistanceInCM();

				updateCurrentLoc();
				
				front = Direction.getDirectionFromGyro((int)currentReading);
				Cell currentCell = getCell(xCoordinate, yCoordinate);
				
				//sensors will face particular absolute direction depending on the direction that the robot is facing
				
				
				//Navigation logic
				System.out.println("Monitor\t" + leftReading + "\t" + frontReading + "\t" + rightReading + "\t" + tachoCount +"\t => target " + targetReading);
			}
		}
	}
		
	private static class MovementThread implements java.lang.Runnable
	{
		MovementThread(int _power, float _targetReading)
		{
			power = _power;
			targetReading = _targetReading;
		}
		
		public void run()
		{
			for(/*long currentTime = System.currentTimeMillis()*/; true;)
			{
				currentReading = getDataFromSensor();
				offset = currentReading - targetReading;
				if(Math.abs(offset) <= 5) //When going straight forward/back
				{
					if( rightReading < 50 )
					{
						offset = (int) (2.2*(rightReading-50) );
					}
					else if( leftReading < 50 )
					{
						offset = (int) (2.2*(50-leftReading));
					}
				}
				int bearing = (int) (offset/1.1);
				int turnAngle = -1 * bearing;
				
				//Steering needs to be at least 40 to have some effect
				if( turnAngle <= -4 )
				{
					turnAngle  -= 25;
					bearing += 25;
				}
				else if( turnAngle >= 4 )
				{
					turnAngle  += 25;
					bearing -= 25;
				}

				if(stalled)
				{
//						System.out.print("Stalled\t");
					System.exit(0);
				}
				
				if(Math.abs(offset) >= 45)	//too much to the sides
				{
		//			System.out.print("Big Offset \t");
					tiltTo(power, turnAngle);
//						action = "Go Turn " + bearing ;
				}
				else if(offset >= 5 || offset <= -5)	//too much to the left
				{
			//		System.out.print("small Offset\t");
					tiltTo(power, turnAngle);
				}
				else
				{
				//	System.out.print("Straight\t");
					goStraight(power);
				}

//					LCD.clearDisplay();	//we don't want the LCD to be cluttered with output			
				
			}
		}
	}
}
