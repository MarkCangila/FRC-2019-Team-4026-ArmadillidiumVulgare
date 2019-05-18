package frc.robot;

public class Portmap {
  // Motor controllers on CAN
  public static final int LEFTDRIVETALON = 1;
  public static final int LEFTDRIVETALON2 = 4;
  public static final int RIGHTDRIVETALON = 2;
  public static final int RIGHTDRIVETALON2 = 3;
  public static final int FRONTINTAKEMOTOR = 5;
  public static final int BACKINTAKEMOTOR = 6;
  public static final int LEFTFLIPPER = 3;
  public static final int RIGHTFLIPPER = 4;

  // Solenoids

  // Digital Inputs
  // Limit switches
  public static final int HATCH_REVERSE_LIMIT = 4;
  public static final int FLIPPER_REVERSE_LIMIT = 5;
  public static final int FLIPPER_FOREWARD_LIMIT = 6;

  public static final int RIGHT_ENCODER_1 = 0;
  public static final int RIGHT_ENCODER_2 = 1;
  public static final int LEFT_ENCODER_1 = 2;
  public static final int LEFT_ENCODER_2 = 3;

  // Analog sensors
  public static final int GYRO = 0;

  public static double clipOneToOne(double val) {
    val = Math.min(1, val);
    val = Math.max(-1, val);
    return val;
  }
}
