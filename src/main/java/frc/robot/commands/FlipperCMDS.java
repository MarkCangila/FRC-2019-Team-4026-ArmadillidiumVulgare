/*package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class FlipperCMDS {
  public static class ManualFlip extends Command {
    public ManualFlip() {
      requires(Robot.flipperSubsystem);
    }

    @Override
    protected void execute() {
      Robot.flipperSubsystem.setFlipperPower((Robot.oi.stick2.getThrottle() * 0.65));
      Robot.flipperSubsystem.setFlipperPower(Robot.oi.stick2.getThrottle() * 0.65);

    }

    @Override
    protected boolean isFinished() {
      return false;
    }

    @Override
    protected void end() {
      Robot.flipperSubsystem.stopFlipper();
    }
  }
  public static class AutoFlip extends Command{
    int state = 0;

    private final int closeToFlip = 0; //Set this to the encoder value when the motor is close to flipping
    private final double fastPower = .75;
    private final double slowPower = .2;


    public AutoFlip() {
      requires(Robot.flipperSubsystem);
    }

    @Override
    protected void initialize(){

    }

    protected void execute (){
      switch (state){
        case 0:
          Robot.flipperSubsystem.setFlipperPower(fastPower);
        if (Math.abs(Robot.flipperSubsystem.getPosition()) > closeToFlip) {
          state++;
        }
        break;
      case 1:
        Robot.flipperSubsystem.setFlipperPower(slowPower);
          if(Math.abs(Robot.flipperSubsystem.getPosition()) > Robot.flipperSubsystem.softLimitForAutoFlip){
            state++;
            Robot.flipperSubsystem.stopFlipper();
          }
          break;
      case 2:
          Robot.flipperSubsystem.stopFlipper();
          if(Math.abs(Robot.Accelerometer.getZ()) > 5){
            state++;
          }
          break;
      case 3:
          Robot.flipperSubsystem.setFlipperPower(-slowPower);
          if(Robot.flipperSubsystem.getPosition() < 1000){
            Robot.flipperSubsystem.stopFlipper();
            state++;
          }
          break;
      case 4:
        Robot.flipperSubsystem.stopFlipper();

      break;


      }

    }
    @Override
    protected void interrupted(){
      Robot.flipperSubsystem.stopFlipper();
      state = 0;
    }
    protected boolean isFinished(){
      return true;
    }
  }
}set

*/
