package testing;

import lejos.hardware.lcd.LCD;
import lejos.utility.Delay;

public class HelloWorld {

	public static void main(String[] args) {
		LCD.drawString("Hello World!", 0, 0);
		Delay.msDelay(5000);
	}

}
