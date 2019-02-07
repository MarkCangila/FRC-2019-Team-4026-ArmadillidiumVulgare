package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;

public class GyroSubsystem extends Subsystem {
  public class GyroMaintanceCMD extends Command {
    long lastPrinted_ns = 0;

    public GyroMaintanceCMD() {
      requires(Robot.gyroSubsystem);
    }

    public void execute() {
      // Print every second
      if (System.nanoTime() >= lastPrinted_ns + 1e9) {
        System.out.println(
            String.format(
                "Current angle is %.1f, current pitch is %.1f, current yaw is %.1f",
                navX.getAngle(), navX.getPitch(), navX.getYaw()));
        lastPrinted_ns = System.nanoTime();
      }
    }

    @Override
    protected boolean isFinished() {
      return false;
    }
  }

  public class GyroResetCMD extends Command {
    public GyroResetCMD() {
      requires(Robot.gyroSubsystem);
    }

    @Override
    public void execute() {
      navX.reset();
    }

    @Override
    public boolean isFinished() {
      return navX.getYaw() == 0;
    }
  }

  public float getRotation() {
    return navX.getYaw();
  }

  public double getAngle() {
    return navX.getAngle();
  }

  AHRS navX = new AHRS(SerialPort.Port.kMXP);

  public GyroSubsystem() {
    navX.reset();
    if (!navX.isConnected()) {
      // throw new IllegalStateException("navX not connected");
    }
  }

  @Override
  protected void initDefaultCommand() {
    setDefaultCommand(new GyroMaintanceCMD());
  }
}
