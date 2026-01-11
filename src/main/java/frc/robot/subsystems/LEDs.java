package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.utils.PeriodicRunnable;
import java.util.Random;

public class LEDs extends PeriodicRunnable {
  final AddressableLED leds;
  final AddressableLEDBuffer ledBuff;

  final Random random = new Random();
  final int[] raindrop =
      new int[180]; // Random numbers generated at startup. https://xkcd.com/221/.

  int timespeed;
  int time;
  int value; // COLOR
  int value2;
  int x;

  final boolean mode1;
  final boolean mode2;
  final boolean mode3;

  final boolean solidColor = true;
  Color color = Color.kRed;

  public LEDs() {
    super(); // Super call adds it to the registry, which calls the periodic method every tick
    leds = new AddressableLED(5);
    ledBuff =
        new AddressableLEDBuffer(
            280); // something around 280 length for full LED (only 85 on robot though)
    // TODO: what does that comment above mean?
    leds.setLength(ledBuff.getLength());
    leds.setData(ledBuff);
    leds.start();

    // TODO: Replace this with Enums
    mode1 = false; // lights moving in a line mode
    mode2 = false;
    mode3 = false;
    x = 1;

    if (!solidColor) {
      for (var i = 0; i < 180; i++) {
        raindrop[i] = random.nextInt(255);
      }

      for (var i = 0; i != 85; i++) {
        ledBuff.setHSV(i, 130, 255, 255);
      }
    }
  }

  @Override
  public void periodic() {

    if (!solidColor) {
      x = 85;
      ledBuff.setHSV(value, 180, 1, raindrop[value] - 1);

      timespeed++; // use this to control the speed of other variables around like the color change.
      timespeed = timespeed % 2; // higher value = slower time/speed.

      if (timespeed == 0) { // value controls the color change speed here currently.
        value++;
      }
      value = value % x;
      if (value == x - 1) {
        value2++;
      }
      value2 = value2 % 180;

      if (mode1) {
        x = 95;
        for (var i = value; i < 1 + value; i++) {
          ledBuff.setHSV(i, raindrop[i], 255, 255);
        }

        if (value > 11) { // Cancels the leds 10 indexes behind, giving it a moving effect
          ledBuff.setHSV(value - 10, 130, 255, 50);
        }

        // Turn off the leds at the end of the strip
        for (var i = 85; i < 95; i++) {
          ledBuff.setHSV(i, 0, 0, 0);
          ledBuff.setHSV(0, 0, 0, 0);
        }
      }

      if (mode2) {
        x = 85;
        ledBuff.setHSV(value, raindrop[value2], 255, 255);
      }

      if (mode3) {
        x = 43;
        for (var i = value + 42; i < value + 43; i++) ledBuff.setHSV(i, raindrop[value2], 255, 255);

        for (var i = -value + 42; i < -value + 43; i++)
          ledBuff.setHSV(i, raindrop[value2], 255, 255);
      }

    } else {
      for (int i = 0; i < 85; i++) ledBuff.setLED(i, color);
    }
    leds.setData(ledBuff);
  }

  public void setColor(Color color) {
    this.color = color;
  }
}
