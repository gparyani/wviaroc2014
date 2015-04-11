package testing;
//TODO
/*
 * Robert's ideas
 * -If stalling when entering cell, add real wall
 * -Only consider IR readings when not attempting to realign
 * -Only make turns/backoffs from middle of cell to other middle of cell in order to stay in center
 */

import static testing.rac3Truck.Rac3TruckMovement.*;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.port.SensorPort;
import lejos.robotics.SampleProvider;
import testing.dragRace.ResettableGyroSensor;
import testing.rac3Truck.Rac3TruckSteering;
import testing.sensors.*;

import java.util.*;

public class SurveyRoute
{
	//Begin variable declarations
	private static ArduinoSideSensors sensorSet;	//maintain constant reference to object to prevent it from being GC'd
	private static SideSensor leftSensor;	//initialized in static block
	private static SideSensor frontSensor = new EV3IRSideSensor(SensorPort.S4);
//	private static SideSensor rightSensor = new NXTUltrasonicSideSensor(SensorPort.S3);
	private static SideSensor rightSensor;	//initialized in static block
	private static ResettableGyroSensor sensor/* = new ResettableGyroSensor(SensorPort.S1)*/;
	private static SampleProvider gyro/* = sensor.getAngleMode()*/;
	private static SampleProvider rgyro/* = sensor.getRateMode()*/;
	private static Direction front;
	private static volatile float leftReading, frontReading, rightReading, offset;
	private static int power;
	private static volatile float targetReading, currentReading;
	private static volatile boolean stalled, leftWall, frontWall, rightWall, isBacking, isTurning;
	private static final double X_ORIGINAL_VALUE = 0.0, Y_ORIGINAL_VALUE = 20.0;
	private static volatile double realX = 0, realY = 1;
	private static volatile double x = X_ORIGINAL_VALUE, y = Y_ORIGINAL_VALUE; //Center of all sensors
	private static volatile int /*lwLastRead, rwLastRead, */rTachoCount, lTachoCount;
	private static Deque<Cell> maze = new ArrayDeque<Cell>();
	private static Queue<Float> leftValues = new ArrayDeque<Float>(), frontValues = new ArrayDeque<Float>(), rightValues = new ArrayDeque<Float>();
	private static final int ANGLE_ERROR_MARGIN = 10;
	private static final float CELL_WIDTH = 71;
	private static volatile State currentState = State.CALIBRATING;
//	private static volatile int xCoordinate, yCoordinate;
	//End variable declarations
	
	static
	{
		sensorSet = new ArduinoSideSensors(SensorPort.S2, 4);
		leftSensor = sensorSet.getLeftSideSensor();
		rightSensor = sensorSet.getRightSideSensor();
	}
	
	private static synchronized float getDataFromSensor()
	{
		float[] data = new float[1];	//an array is necessary to get the data
		gyro.fetchSample(data, 0);
		return -1 * data[0];
	}
	
	private static float getRateDataFromSensor()
	{
		float[] data = new float[1];	//an array is necessary to get the data
		rgyro.fetchSample(data, 0);
		return data[0];
	}
	
	private /*static*/ enum State	//nested enums are implicitly static
	{
		CALIBRATING, MAPPING_RUN, WAITING_TO_BEGIN_SOLUTION, SOLUTION_RUN
	}
	
	private enum Direction
	{
		NORTH, EAST, SOUTH, WEST, IN_BETWEEN;
		
		/**
		 * Gets the Direction at which the robot is facing.
		 * @param reading the reading from the gyro
		 * @return the Direction object
		 */
		static Direction getDirectionFromGyro(int reading)
		{
			if(reading >= 360)
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
			default:
				break;
			}
			return this;	//IN_BETWEEN's opposite direction is also IN_BETWEEN; compiler throws error if in switch statement itself
		}
		
		Direction getLeftDirection()
		{
			switch(this) {
			case NORTH:
				return WEST;
			case EAST:
				return NORTH;
			case SOUTH:
				return EAST;
			case WEST:
				return SOUTH;
			default:
				break;
			}
			return this;
		}
		
		Direction getRightDirection()
		{
			switch(this) {
			case NORTH:
				return EAST;
			case EAST:
				return SOUTH;
			case SOUTH:
				return WEST;
			case WEST:
				return NORTH;
			default:
				break;
			}
			return this;
		}
	}
	
	/**
	 * Represents a cell within a maze. A maze is composed of several cells.
	 *
	 */
	private static class Cell
	{
		private int xPos, yPos;	//the grid location of this cell
		private WallState west = WallState.UNKNOWN, north = WallState.UNKNOWN,
				south = WallState.UNKNOWN, east = WallState.UNKNOWN;	//the walls surrounding the cell
		
		enum WallState
		{
			UNKNOWN, NO_WALL, VIRTUAL_WALL, REAL_WALL
		}
		
		
		Cell(int x, int y)
		{
			this.xPos = x;
			this.yPos = y;
		}
		
		Cell getCellFromDirection(Direction dir)
		{
			switch(dir) {
			case NORTH:
				return getCell(xPos, yPos + 1);
			case EAST:
				return getCell(xPos + 1, yPos);
			case SOUTH:
				return getCell(xPos, yPos - 1);
			case WEST:
				return getCell(xPos - 1, yPos);
			default:
				break;
			}
			throw new IllegalArgumentException();
		}

		int getX() {
			return xPos;
		}

		int getY() {
			return yPos;
		}

		WallState westWallState() {
			return west;
		}

		void setWest(WallState west) {
			this.west = west;
			System.out.println(this);
		}

		WallState northWallState() {
			return north;
		}

		void setNorth(WallState north) {
			this.north = north;
		}

		WallState southWallState() {
			return south;
		}

		void setSouth(WallState south) {
			this.south = south;
		}
		
		WallState eastWallState() {
			return east;
		}
		
		void setEast(WallState east) {
			this.east = east;
			System.out.println(this);
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
					north + "\tSouth: " + south + "\tEast: " + east + "\tWest: " + west + "\t" + leftReading + "\t" +
							+ frontReading + "\t" + rightReading + "\n";
		}

		public boolean westWallExists(boolean wall) {
			if(west == WallState.NO_WALL)
				wall = false;
			//return whether a real wall exists, or only if the robot is executing the solution run, if a virtual wall exists
			wall |=  (west == WallState.REAL_WALL);
			wall |= (currentState == State.SOLUTION_RUN && west == WallState.VIRTUAL_WALL);
			return wall;
		}

		public boolean eastWallExists(boolean wall) {
			if(east == WallState.NO_WALL)
				wall = false;
			//return whether a real wall exists, or only if the robot is executing the solution run, if a virtual wall exists
			wall |=  (east == WallState.REAL_WALL);
			wall |= (currentState == State.SOLUTION_RUN && east == WallState.VIRTUAL_WALL);
			return wall;
		}

		public boolean northWallExists(boolean wall) {
			if(north == WallState.NO_WALL)
				wall = false;
			//return whether a real wall exists, or only if the robot is executing the solution run, if a virtual wall exists
			wall |=  (north == WallState.REAL_WALL);
			wall |= (currentState == State.SOLUTION_RUN && north == WallState.VIRTUAL_WALL);
			return wall;
		}

		public boolean southWallExists(boolean wall) {
			if(south == WallState.NO_WALL)
				wall = false;
			//return whether a real wall exists, or only if the robot is executing the solution run, if a virtual wall exists
			wall |=  (south == WallState.REAL_WALL);
			wall |= (currentState == State.SOLUTION_RUN && south == WallState.VIRTUAL_WALL);
			return wall;
		}
	}
	
	private static void calibrateGyro() 
	{
		Button.LEDPattern(2);	//solid red
		try {
			//Begin gyro reset procedure
			Thread.sleep(750);	//allow user enough time to let go of robot
			sensor.reset();
			Thread.sleep(1500);	//reset delay
			getRateDataFromSensor();
			getDataFromSensor();
			Thread.sleep(4000);
			//End gyro reset procedure
		} catch (Exception e) {e.printStackTrace();}
		Button.LEDPattern(1);	//solid green
	}
	
	//	@SuppressWarnings("deprecation")	//Thread.stop() is deprecated
	public static void main(String[] args) throws Throwable
	{
		System.setErr(System.out);
		Button.LEDPattern(4);	//turn button backlight blinking green

//		sensor.hardReset();
		Button.waitForAnyPress();
		Button.LEDPattern(2);	//solid red
		Thread.sleep(750);
		sensor = new ResettableGyroSensor(SensorPort.S1);
		gyro = sensor.getAngleMode();
		rgyro = sensor.getRateMode();
		calibrateGyro();
		
		Rac3TruckSteering.reset();
		Button.LEDPattern(1);	//turn button backlight green

		System.out.println("Program starting.");
		currentState = State.MAPPING_RUN;


		new Thread(new MonitorThread()).start();
		Thread.sleep(300);
		new Thread(new MovementThread(15, 0)).start();

		//Check for button presses
		new Thread(new Runnable() {
			public void run()
			{
				while(true)
				{
					if(Button.waitForAnyPress() == Button.ENTER.getId())
					{	
						if(currentState == State.MAPPING_RUN)
						{
							//initialize values for MonitorThread
							currentState = State.WAITING_TO_BEGIN_SOLUTION;
							Button.LEDPattern(4);	//blinking green
							System.out.println(maze);
						}
						else if(currentState == State.WAITING_TO_BEGIN_SOLUTION)
						{
							System.out.println("Starting solution run");
							
							Button.LEDPattern(1);	//solid green
							resetAllTachoCounts();
							targetReading = 0;
							leftValues.clear();
							frontValues.clear();
							rightValues.clear();
							x = X_ORIGINAL_VALUE;
							y = Y_ORIGINAL_VALUE;
							realX = 0;
							realY = 1;
							front = Direction.NORTH;
							stalled = false;
							try {
								Thread.sleep(1000);
							} catch (InterruptedException e1) {
								e1.printStackTrace();
							}
							calibrateGyro();
							
							currentReading = getDataFromSensor();
							
							currentState = State.SOLUTION_RUN;
							
							new Thread(new MonitorThread()).start();
							try {
								Thread.sleep(500);
							} catch (InterruptedException e) {
								e.printStackTrace();
							}
							new Thread(new MovementThread(15, 0)).start();
						}
					}
					else
					{
						sensor.close();
						System.exit(0);
					}
				}
			}
		}).start();
		//Check for robot crashes into obstacles
		while(true)
		{
			stalled = isStalled();	//handled in movement thread
		}		
	}

	static boolean isTooCloseToNSBorder()	//Near north and south borders of a cell
	{
		double dist = getDistanceFromBorder(Direction.SOUTH);

//		if(isBacking && dist > 7 && dist < 20) dist-= 7; //Slight adjustment for moving back
		return (dist > (CELL_WIDTH - 20) || dist < 20); //if too close to boundary, skip update
	}
	
	static boolean isTooCloseToEWBorder()	//Near east and west borders of a cell
	{
		double dist = getDistanceFromBorder(Direction.WEST);

//		if(isBacking && dist > 7 && dist < 20) dist-= 7; //Slight adjustment for moving back
		return (dist > (CELL_WIDTH - 20) || dist < 20); //if too close to boundary, skip update
	}
	
	synchronized static double getDistanceFromBorder(Direction border)
	{
		//calculate the grid coordinates of the current cell
		int xCoord = (x > 0) ? (int)((x + (CELL_WIDTH / 2)) / CELL_WIDTH) : (int)((x - (CELL_WIDTH / 2)) / CELL_WIDTH),
		yCoord = (int)(y / CELL_WIDTH);
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
			dist = (x > 0) ? (x - ((xCoord * CELL_WIDTH) - CELL_WIDTH/2)) : (((xCoord * CELL_WIDTH) + CELL_WIDTH/2) - x );
			break;
		case WEST:
			dist = (x > 0) ? (x - ((xCoord * CELL_WIDTH) - CELL_WIDTH/2)) : (((xCoord * CELL_WIDTH) + CELL_WIDTH/2) - x);
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
//		boolean wasAdded = false;
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
			leftValues.clear();
			frontValues.clear();
			rightValues.clear();
//			wasAdded = true;
		}
		else if(!isTurning)
		{

			Cell oldCell = maze.getLast();	//gets the last cell added
//			if(!currentCell.equals(oldCell))
//				maze.removeLast();	//this means that we visited a cell that was unnecessary and should be removed
			//set virtual walls so that the robot knows one single path through the maze so that we can get through it without wrong turns
			if(currentState == State.MAPPING_RUN)
			{
				if(currentCell.getX() == oldCell.getX() + 1)	//moved east
				{
					System.out.println("moved east");
					maze.remove( oldCell );
					currentCell.setWest(Cell.WallState.VIRTUAL_WALL);
				}
				else if(currentCell.getX() == oldCell.getX() - 1)	//moved west
				{
					System.out.println("moved west");
					maze.remove( oldCell );
					currentCell.setEast(Cell.WallState.VIRTUAL_WALL);
				}
				else if(currentCell.getY() == oldCell.getY() + 1)	//moved north
				{
					System.out.println("moved north");
					maze.remove( oldCell );
					currentCell.setSouth(Cell.WallState.VIRTUAL_WALL);
				}
				else if(currentCell.getY() == oldCell.getY() - 1)	//moved south
				{
					System.out.println("moved south");
					maze.remove( oldCell );
					currentCell.setNorth(Cell.WallState.VIRTUAL_WALL);
				}
			}
		}
		
//		if(wasAdded)
//			System.out.println(maze);
		return currentCell;
	}
	
	
	private static synchronized void updateCurrentLoc()
	{
		int deltaR, deltaL;
		double deltaX, deltaY;
		
		deltaR = getRightTachoCount() - rTachoCount;
		rTachoCount += deltaR;
		
		deltaL = getLeftTachoCount() - lTachoCount;
		lTachoCount += deltaL;
			
		int betterReading;
		if(Math.abs(deltaL) < 2 || Math.abs(deltaR) < 2) 
			betterReading = (deltaL < 0 || deltaR < 0) ? Math.max(deltaL, deltaR) : Math.min(deltaL, deltaR);
		else betterReading = (deltaL + deltaR) /2 ;
		
		//currentReading is with respect to +y-axis; add 90 so it's w.r.t. +x-axis
		//change vector magnitude and direction to x- and y-components
		//deltaX and deltaY are displacements since last iteration
		//they are in units of MOTOR degrees
		deltaX = betterReading * Math.cos(Math.toRadians(currentReading + 90));
		deltaY = betterReading * Math.sin(Math.toRadians(currentReading + 90));
		
		//update current absolute location
		realX += deltaX * 2.5 / 360.0 * 31;	//deltaX * gear ratio / (convert degrees to rotations) * wheel circumference
		realY += deltaY * 2.5 / 360.0 * 31;	//deltaY * gear ratio / (convert degrees to rotations) * wheel circumference
		
		x= realX + 20 * Math.cos(Math.toRadians(currentReading + 90));
		y = realY + 20* Math.sin(Math.toRadians(currentReading + 90));
	}
	
	private static class MonitorThread implements Runnable
	{
		static float movingAverage(Queue<Float> sampleSet, float newSample)
		{
			if(newSample == Float.POSITIVE_INFINITY)
				newSample = CELL_WIDTH;
			sampleSet.add(newSample);
			if(sampleSet.size() == 4)
				sampleSet.remove();
			float sum = 0;
			for(float element : sampleSet)
				sum += element;
			return sum / sampleSet.size();
		}

		Cell turningFrom = null, turningTo = null;
		public void run()
		{
			try {
				lTachoCount = getLeftTachoCount();
				rTachoCount = getRightTachoCount();
				while(true)
				{
					if(currentState == State.WAITING_TO_BEGIN_SOLUTION) return;
					
					leftReading = leftSensor.getDistanceInCM();
					frontReading = frontSensor.getDistanceInCM();
					rightReading = rightSensor.getDistanceInCM();
					
					front = Direction.getDirectionFromGyro((int)currentReading);
	
					if(front != Direction.IN_BETWEEN)
					{
						//Filter sensor readings
						leftReading = movingAverage(leftValues, leftReading);
						frontReading = movingAverage(frontValues, frontReading);
						rightReading = movingAverage(rightValues, rightReading);
					}
					else
					{
						leftValues.clear();
						frontValues.clear();
						rightValues.clear();
					}
//					leftWall = leftReading < 42;	//Maximum distance from left wall is 36
//					frontWall = frontReading < 22;	//Detect front wall only when close enough
//					rightWall = rightReading < 42;
	
					updateCurrentLoc();
					double wdist = getDistanceFromBorder(Direction.WEST),
						   edist = getDistanceFromBorder(Direction.EAST),
						   ndist = getDistanceFromBorder(Direction.NORTH),
						   sdist = getDistanceFromBorder(Direction.SOUTH);
					
					float leeway = (CELL_WIDTH / 2) + 10;
					
					wdist = wdist < leeway ? leeway : wdist;
					edist = edist < leeway ? leeway : edist;
					ndist = ndist < leeway ? leeway : ndist;
					sdist = sdist < leeway ? leeway : sdist;
					
					System.out.println( wdist + " " + leftValues.size() );
					//calculate the grid coordinates of the current cell
					int xCoordinate = (x > 0) ? (int)((x + (CELL_WIDTH / 2)) / CELL_WIDTH) : (int)((x - (CELL_WIDTH / 2)) / CELL_WIDTH),
						yCoordinate = (int)(y / CELL_WIDTH);
					
					Cell currentCell = getCell(xCoordinate, yCoordinate);
					
					//sensors will face particular absolute direction depending on the direction that the robot is facing
					switch(front)
					{
						case NORTH:

							if((ndist > 20 && sdist > 20)  && currentCell.northWallState() == Cell.WallState.UNKNOWN) {//if too close to boundary, skip update
								if(frontReading < 16)
									currentCell.setNorth(Cell.WallState.REAL_WALL);
								if(leftReading < 28)
									currentCell.setWest(Cell.WallState.REAL_WALL);
								if(rightReading < 28)
									currentCell.setEast(Cell.WallState.REAL_WALL);
							}
							leftWall = currentCell.westWallExists(leftReading < wdist);
							rightWall = currentCell.eastWallExists(rightReading < edist);
							frontWall = currentCell.northWallExists(frontReading < ndist);
							break;
						case EAST:
							if((edist > 20 && wdist > 20) && currentCell.eastWallState() == Cell.WallState.UNKNOWN) {
								if(frontReading < 16)
									currentCell.setEast(Cell.WallState.REAL_WALL);
								if(leftReading < 28)
									currentCell.setNorth(Cell.WallState.REAL_WALL);
								if(rightReading < 28)
									currentCell.setSouth(Cell.WallState.REAL_WALL);
							}
							leftWall = currentCell.northWallExists(leftReading < ndist);
							rightWall = currentCell.southWallExists(rightReading < sdist);
							frontWall = currentCell.eastWallExists(frontReading < edist);
							break;
						case WEST:
							if((edist > 20 && wdist > 20) && currentCell.westWallState() == Cell.WallState.UNKNOWN) {
								if(frontReading < 16)
									currentCell.setWest(Cell.WallState.REAL_WALL);
								if(leftReading < 28)
									currentCell.setSouth(Cell.WallState.REAL_WALL);
								if(rightReading < 28)
									currentCell.setNorth(Cell.WallState.REAL_WALL);
							}
							leftWall = currentCell.southWallExists(leftReading < sdist);
							rightWall = currentCell.northWallExists(rightReading < ndist);
							frontWall = currentCell.westWallExists(frontReading < wdist);
							break;
						case SOUTH:
							if((ndist > 20 && sdist > 20) && currentCell.southWallState() == Cell.WallState.UNKNOWN) {
								if(frontReading < 16)
									currentCell.setSouth(Cell.WallState.REAL_WALL);
								if(leftReading < 28)
									currentCell.setEast(Cell.WallState.REAL_WALL);
								if(rightReading < 28)
									currentCell.setWest(Cell.WallState.REAL_WALL);
							}
							leftWall = currentCell.eastWallExists(leftReading < edist);
							rightWall = currentCell.westWallExists(rightReading < wdist);
							frontWall = currentCell.southWallExists(frontReading <sdist);
							break;
						//case IN_BETWEEN:
						default:	//workaround warning
							break;
					}
					
					
					//Navigation logic
					if(currentState == State.SOLUTION_RUN && maze.getLast().equals(currentCell))
					{
						stopTruck();
						Sound.playTone(1050, 100);
						try {
							Thread.sleep(50);
						} catch (InterruptedException e) {
							e.printStackTrace();
						}
						Sound.playTone(1050, 1000);
						sensor.close();
						System.exit(0);
					}
					if(front != Direction.IN_BETWEEN /* && ( currentState == State.SOLUTION_RUN || leftValues.size() > 1 )  && !isTooCloseToEWBorder() && !isTooCloseToNSBorder()*/)
					{
						if(isBacking) //if at a dead end, back up
						{
		//					System.out.println( "isBacking");
							if(!rightWall)	//since we are following the left wall, look for the opposite side opening when backing up
							{
								System.out.println("BACKRIGHT:\t" + currentCell);
								isTurning = true;
								turningFrom = currentCell;
								turningTo = currentCell.getCellFromDirection(front.getRightDirection());
		//						rwLastRead = tachoCount;
								targetReading -= 90;	//causes movement thread to turn robot right
								isBacking = false;
							}
						}
						else if(!isTurning)	//forward motion
						{
							if(!leftWall)	//look for the left wall
							{
								System.out.println("LEFT:\t" + currentCell);
								isTurning = true;
								turningFrom = currentCell;
								turningTo = currentCell.getCellFromDirection(front.getLeftDirection());
		//						lwLastRead = tachoCount;
								targetReading += 90;	//turn left
							}
							else if(frontWall && !rightWall)	//if front wall is blocked, turn right if possible
							{
								System.out.println("RIGHT:\t" + currentCell);
								isTurning = true;
								turningFrom = currentCell;
								turningTo = currentCell.getCellFromDirection(front.getRightDirection());
		//						rwLastRead = tachoCount;
								targetReading -= 90;
							}
							else if(frontWall && rightWall && leftWall)	//detect dead ends; block above will activate in next iteration
							{
								System.out.println("BACKING:" + currentCell);
		//						isTurning = true;
								isBacking = true;
		//						targetReading -= 180;
							}
						}
					}
					currentReading = getDataFromSensor();
					offset = currentReading - targetReading;	//recalculate all variables for next iteration
					front = Direction.getDirectionFromGyro((int)currentReading);
					if(isTurning && offset >= -ANGLE_ERROR_MARGIN && offset <= ANGLE_ERROR_MARGIN)	//detect when the turn gets completed by the movement thread
					{
						if(stalled || (currentCell.equals(turningTo) 
								&& (getDistanceFromBorder(front.getOppositeDirection()) > 10))
								)
							isTurning = false;
						
						if(stalled )
						{
							switch(front) {
							case NORTH:
								currentCell.setNorth(Cell.WallState.REAL_WALL);
								break;
							case EAST:
								currentCell.setEast(Cell.WallState.REAL_WALL);
								break;
							case WEST:
								currentCell.setWest(Cell.WallState.REAL_WALL);
								break;
							case SOUTH:
								currentCell.setSouth(Cell.WallState.REAL_WALL);
								break;
							}
						}
						
						if( currentState == State.MAPPING_RUN && currentCell != turningFrom && front != Direction.IN_BETWEEN 
								&& currentCell != turningFrom && (getDistanceFromBorder(front.getOppositeDirection()) > 10)  ) {
							switch(front) {
							case NORTH:
								turningFrom.setNorth(Cell.WallState.NO_WALL);
								currentCell.setSouth(Cell.WallState.NO_WALL);
								break;
							case EAST:
								turningFrom.setEast(Cell.WallState.NO_WALL);
								currentCell.setWest(Cell.WallState.NO_WALL);
								break;
							case SOUTH:
								turningFrom.setSouth(Cell.WallState.NO_WALL);
								currentCell.setNorth(Cell.WallState.NO_WALL);
								break;
							case WEST:
								turningFrom.setWest(Cell.WallState.NO_WALL);
								currentCell.setEast(Cell.WallState.NO_WALL);
								break;
							default:	//workaround compiler warning
								break;
							}
						}
					}
					System.out.println("Monitor\t" + "offset\t" + offset + "\t" + leftReading + "\t" + frontReading +
							"\t" + rightReading + "\t" + lTachoCount + "\t" + rTachoCount +"\t => target " + targetReading);
				}
			} catch (Exception e) {e.printStackTrace(System.out);}
		}
	}
		
	private static class MovementThread implements java.lang.Runnable
	{
		MovementThread(int _power, float _targetReading)
		{
			power = _power;
			targetReading = _targetReading;
		}
		
		private int getBackOffFromCoordinates()
		{
			if( front == Direction.IN_BETWEEN )
				return 120;
				
			return (int) (getDistanceFromBorder(front.getOppositeDirection()) *360/(1.5*2.5*31));
					
			//return (25 * (int) getDistanceFromBorder(front.getOppositeDirection()))
					/*+ ((front == Direction.WEST || front == Direction.EAST) ? 250 : 0);*/
			
		}
		
		private static final double BEARING_TO_OFFSET_RATIO = 0.65;
		
		public void run()
		{
			boolean alreadyWentBackTurn = false,
					alreadyWentBack = false;
			
			float effectiveOffset;
			
//			System.out.println("Time\tTacho\tOffset\tAction\tBearing");
			for(/*long currentTime = System.currentTimeMillis()*/; true;)
			{
				if(currentState == State.WAITING_TO_BEGIN_SOLUTION) 
				{
					stopTruck();
					return;
				}
				
//				String action;
				
				currentReading = getDataFromSensor();
				offset = currentReading - targetReading;
				effectiveOffset = offset;

//				System.out.print("Latch T = " + (System.currentTimeMillis() - currentTime) + "\t Steer = " + Rac3TruckSteering.getTachoCount()
//						+ "\t offset = " + offset + "\t");	
				
				if(!isTurning && !stalled && (offset <= ANGLE_ERROR_MARGIN || offset >= -ANGLE_ERROR_MARGIN)) //When going straight forward/back
				{
					if( rightReading < 18)
					{
						effectiveOffset = (int) (1.4*(rightReading-18) );
						if( isBacking ) effectiveOffset *= -1;
//						System.out.println("-R" + rightReading + "\t" + offset);
					}
					else if( leftReading < 18)
					{
						effectiveOffset = (int) (1.4*(18-leftReading));
						if( isBacking ) effectiveOffset *= -1;
//						System.out.println("-L" + leftReading + "\t" + offset);
					}
				}
				int bearing = (int) (effectiveOffset/BEARING_TO_OFFSET_RATIO);
				int turnAngle = -1 * bearing;
				int backOff = 120;
				
				//Steering needs to be at least 25 to have some effect
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
//					System.out.print("Stalled\t");
					if( !isBacking ) {
//						System.out.print("Forward&Back\t");
						bearing = (bearing < 0) ? (bearing - 25) : (bearing+25); //stuck in loop otherwise
						goBackwardsTacho(power , bearing, backOff);	//we may stall either while backing or while proceeding, so...
						goForwards( power, -bearing, backOff);	//...we go both ways
					}
					else
						isBacking = false;
				}
				if( isBacking )
				{
					Button.LEDPattern(3);	//button backlight orange
					if(!alreadyWentBack)
					{
//						System.out.print("Start Backing\t");
						stopTruck();

						alreadyWentBack = true;
					}
					goBackwards(power, bearing);
//					System.out.print("isBacking\n");
					continue;
				}
				else
				{
					Button.LEDPattern(1);	//button backlight green
					alreadyWentBack = false;
				}
				if(offset >= 45 || offset <= -45)	//too much to the sides
				{
					if(!alreadyWentBackTurn)
					{
//						int temp = (offset >= 45) ? rwLastRead : lwLastRead;

						stopTruck();
						updateCurrentLoc();
						backOff = getBackOffFromCoordinates();
						
//						int now = getRightTachoCount();
//						if( !frontWall && ((now - temp) < 5))
//							backOff = 700;
						
						
						bearing = (int) (offset/BEARING_TO_OFFSET_RATIO);
						turnAngle = -1 * bearing;
						System.out.println( "Turn : " + turnAngle + "Backoff\t" + backOff + "\t" + "Bearing: " + bearing);
						goBackwardsTacho(power, bearing, backOff);//TODO
						alreadyWentBackTurn = true;
					}
		//			System.out.print("Big Offset \t");
					tiltTo(power, turnAngle);
//					action = "Go Turn " + bearing ;
				}
				else /*if(offset >= ANGLE_ERROR_MARGIN || offset <= -ANGLE_ERROR_MARGIN) */	//too much to the left
				{
			//		System.out.print("small Offset\t");
					tiltTo(power, turnAngle);
//					action = "Tilt " + bearing;
					alreadyWentBackTurn = false;
				}
/*
				else
				{
				//	System.out.print("Straight\t");
					goStraight(power);
//					action = "Straight";
					alreadyWentBackTurn = false;
				}
*/
//				System.out.println();	
//				LCD.clearDisplay();	//we don't want the LCD to be cluttered with output			
				
			}
		}
	}
}