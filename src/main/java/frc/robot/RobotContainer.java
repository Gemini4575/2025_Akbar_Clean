// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.JoystickConstants;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.algea.EXO.OzDown;
import frc.robot.commands.algea.EXO.OzUp;
import frc.robot.commands.climb.Climb;
import frc.robot.commands.coral.lili.LIPlaceCoral;
import frc.robot.subsystems.LiliCoralSubystem;
import frc.robot.subsystems.NickClimbingSubsystem;
import frc.robot.subsystems.OzzyGrabberSubsystem;
import frc.robot.subsystems.drive.Drivetrain;

import static frc.robot.Constants.JoystickConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  /* Controllers */
  private final Joystick driver = new Joystick(0);
  private final Joystick operator = new Joystick(1);
  private final Joystick climber = new Joystick(2);
  private final Joystick testing = new Joystick(3);

  /* Driver Buttons */
  private final JoystickButton zeroGyro = new JoystickButton(driver, 4);
  private final Trigger Slow = new Trigger(new JoystickButton(driver, 7)
      .and(new JoystickButton(driver, 12)))
      .or(new JoystickButton(operator, START_BUTTON));

  /* Pathplanner stuff */
  private final SendableChooser<Command> PathplannerautoChoosers;

  /* Subsystems */
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final LiliCoralSubystem c = new LiliCoralSubystem();
  private final NickClimbingSubsystem nc = new NickClimbingSubsystem();
  private final OzzyGrabberSubsystem g = new OzzyGrabberSubsystem();

  public RobotContainer() {

    PathplannerautoChoosers = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("[Robot]Auto Chosers", PathplannerautoChoosers);

    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    /* Driver Controls */
    zeroGyro.onTrue(new InstantCommand(() -> m_drivetrain.ResetGyro()));
    m_drivetrain.setDefaultCommand(
        new TeleopSwerve(
            m_drivetrain,
            () -> -driver.getRawAxis(LEFT_X_AXIS),
            () -> driver.getRawAxis(LEFT_Y_AXIS),
            () -> -driver.getTwist(),
            Slow));
    /* Operator Controls */
    new JoystickButton(operator, JoystickConstants.BLUE_BUTTON)
        .onTrue(new LIPlaceCoral(c));
    new JoystickButton(operator, JoystickConstants.GREEN_BUTTON)
        .whileTrue(new OzUp(g));
    new JoystickButton(operator, YELLOW_BUTTON)
        .whileTrue(new OzDown(g));

    /* Testing */
    new JoystickButton(testing, BLUE_BUTTON)
        .onTrue(new Climb(nc));
    System.out.println("Ended configureBindings()");
  }

  public void teleopPeriodic() {
    if (operator.getRawButton(GREEN_BUTTON)) {
      g.Up();
    } else {
      g.end();
    }
    if (operator.getRawButton(LEFT_BUMPER)) {
      g.intake();
    } else if (operator.getRawButton(RIGHT_BUMPER)) {
      g.outake();
    } else {
      g.stop();
    }
    // c.JoyControll(operator.getRawAxis(JoystickConstants.LEFT_Y_AXIS));
    g.joy(MathUtil.applyDeadband(operator.getRawAxis(JoystickConstants.LEFT_Y_AXIS), 0.5) * 1);
    // g.joy1(MathUtil.applyDeadband(climber.getRawAxis(JoystickConstants.LEFT_Y_AXIS),
    // 0.2));
    if (climber.getRawButton(GREEN_BUTTON)) {
      nc.JoyClimb1(-1, false);
      nc.JoyClimb2(-1, false);
    } else {
      nc.JoyClimb1(MathUtil.applyDeadband(climber.getRawAxis(JoystickConstants.RIGHT_Y_AXIS), 0.5),
          climber.getRawButton(JoystickConstants.START_BUTTON));
      nc.JoyClimb2(MathUtil.applyDeadband(climber.getRawAxis(JoystickConstants.LEFT_Y_AXIS), 0.5),
          climber.getRawButton(JoystickConstants.BACK_BUTTON));
    }
    nc.Flipper(testing.getRawAxis(LEFT_Y_AXIS));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return PathplannerautoChoosers.getSelected();
  }
}
