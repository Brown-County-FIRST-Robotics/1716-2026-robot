package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.utils.PeriodicRunnable;
import java.util.Random;

public class LEDs extends PeriodicRunnable {
  public enum LEDMode {
    SOLID,
    OFF
  }

  final AddressableLED leds;
  final AddressableLEDBuffer ledBuff;

  final Random random = new Random();

  int step;

  Color color = Color.kWhite;

  LEDMode mode;

  public LEDs() {
    super(); // Super call adds it to the registry, which calls the periodic method every tick
    leds = new AddressableLED(5); // LEDs on port 5
    ledBuff = new AddressableLEDBuffer(280); // Encode up to 280 of them
    leds.setLength(ledBuff.getLength());
    leds.setData(ledBuff);
    leds.start();

    mode = LEDMode.OFF;

    // Init all LEDs to teal (because why not I guess)
    for (int i = 0; i < ledBuff.getLength(); i++) ledBuff.setHSV(i, 50, 255, 50);
  }

  private static double lerp(double d1, double d2, double fac) {
    assert (fac >= 0);
    assert (fac <= 1);

    return fac * d2 + (1.0 - fac) * d1;
  }

  private static Color lerp(Color c1, Color c2, double fac) {
    final double r = lerp(c1.red, c2.red, fac);
    final double g = lerp(c1.green, c2.green, fac);
    final double b = lerp(c1.blue, c2.blue, fac);
    return new Color(r, g, b);
  }

  @Override
  public void periodic() {
    if (mode == LEDMode.OFF) {
      for (int i = 0; i < ledBuff.getLength(); i++) ledBuff.setLED(i, Color.kBlack);
    } else if (mode == LEDMode.SOLID) {
      for (int i = 0; i < ledBuff.getLength(); i++) ledBuff.setLED(i, color);
    }
    // TODO: More LED modes
    leds.setData(ledBuff);
  }

  public void setColor(Color color) {
    this.color = color;
  }

  public void setMode(LEDMode mode) {
    this.mode = mode;
  }
}