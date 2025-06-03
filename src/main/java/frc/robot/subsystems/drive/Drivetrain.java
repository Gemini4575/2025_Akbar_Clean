package frc.robot.subsystems.drive;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveConstants.Mod0;
import frc.robot.Constants.SwerveConstants.Mod1;
import frc.robot.Constants.SwerveConstants.Mod2;
import frc.robot.Constants.SwerveConstants.Mod3;

import static frc.robot.Constants.SwerveConstants.*;

import java.io.IOException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.studica.frc.AHRS.NavXUpdateRate;

public class Drivetrain extends SubsystemBase {
  // TODO make the auto spit a percent at us

  private SwerveModule backLeft_0 = new SwerveModule(Mod0.constants);
  private SwerveModule backRight_1 = new SwerveModule(Mod1.constants);
  private SwerveModule frontRight_2 = new SwerveModule(Mod2.constants);
  private SwerveModule frontLeft_3 = new SwerveModule(Mod3.constants);

  private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI, NavXUpdateRate.k100Hz);

  private double xSpeed_cur;
  private double ySpeed_cur;
  private double rot_cur;

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_backLeftLocation,
      m_backRightLocation,
      m_frontRightLocation, m_frontLeftLocation);

  private final SwerveDrivePoseEstimator poseEstimator;

  private final SwerveSetpointGenerator setpointGenerator;
  private SwerveSetpoint previousSetpoint;
  private RobotConfig config;

  public Drivetrain() {
    gyro.reset();

    try {
      config = RobotConfig.fromGUISettings();
    } catch (IOException | org.json.simple.parser.ParseException e) {
      e.printStackTrace();
      throw new RuntimeException(e);
    }

    var stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
    var visionStdDevs = VecBuilder.fill(1, 1, 1);
    poseEstimator = new SwerveDrivePoseEstimator(
        m_kinematics,
        gyro.getRotation2d(),
        getModulePositions(),
        new Pose2d(),
        stateStdDevs,
        visionStdDevs);

    poseEstimator.resetPosition(new Rotation2d(180), getModulePositions(),
        new Pose2d(7.558, 4.010, new Rotation2d(180)));

    configureAutoBuilder();

    setpointGenerator = new SwerveSetpointGenerator(
        config,
        Units.rotationsToRadians(1.0) // The max rotation velocity of a swerve module in radians per second. This
                                      // should probably be stored in your Constants file
    );
    previousSetpoint = new SwerveSetpoint(new ChassisSpeeds(), getModuleStates(),
        DriveFeedforwards.zeros(config.numModules));
  }

  private SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
        backLeft_0.getState(),
        backRight_1.getState(),
        frontRight_2.getState(),
        frontLeft_3.getState(),
    };
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
        backLeft_0.getPosition(),
        backRight_1.getPosition(),
        frontRight_2.getPosition(),
        frontLeft_3.getPosition(),
    };
  }

  // hi
  public void driveRobotRelative(ChassisSpeeds c) {
    drive((c.vxMetersPerSecond / MaxMetersPersecond),
        (c.vyMetersPerSecond / MaxMetersPersecond), (c.omegaRadiansPerSecond / kModuleMaxAngularVelocity), false);
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    SmartDashboard.putNumber("[Drivetrain]drive rot", rot);
    SmartDashboard.putNumber("[Drivetrain]drive xSpeed", xSpeed);
    SmartDashboard.putNumber("[Drivetrain]drive ySpeed", ySpeed);
    xSpeed_cur = xSpeed;
    ySpeed_cur = ySpeed;
    rot_cur = rot;
    SmartDashboard.putNumber("[Drivetrain]Gyro", gyro.getAngle());
    var swerveModuleStates = m_kinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(-1 * xSpeed, ySpeed, rot, gyro.getRotation2d())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, 1.0);

    SmartDashboard.putString("[Drivetrain]gyro", gyro.getRotation2d().toString());
    SmartDashboard.putString("[Drivetrain]module 0", swerveModuleStates[0].toString());
    SmartDashboard.putString("[Drivetrain]module 1", swerveModuleStates[1].toString());
    SmartDashboard.putString("[Drivetrain]module 2", swerveModuleStates[2].toString());
    SmartDashboard.putString("[Drivetrain]module 3", swerveModuleStates[3].toString());

    setModuleStates(swerveModuleStates);
  }

  private void setModuleStates(SwerveModuleState[] swerveModuleStates) {
    backLeft_0.SetDesiredState(swerveModuleStates[0]);
    backRight_1.SetDesiredState(swerveModuleStates[1]);
    frontRight_2.SetDesiredState(swerveModuleStates[2]);
    frontLeft_3.SetDesiredState(swerveModuleStates[3]);

  }

  public void ResetGyro() {
    gyro.reset();

    SmartDashboard.putString("[Drivetrain]Gyro has been reset", java.time.LocalTime.now().toString());
    System.out.println("Gyro has been reset");
  }

  public ChassisSpeeds getSpeed() {
    return new ChassisSpeeds(xSpeed_cur, ySpeed_cur, rot_cur);
  }

  private ChassisSpeeds getRobotRelativeSpeeds() {
    var c = m_kinematics.toChassisSpeeds(getModuleStates());
    SmartDashboard.putString("[Drivetrain]Robot relative speeds", c.toString());
    return c;
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void resetPose(Pose2d aPose2d) {
    // m_odometry.resetPosition(m_gyro.getRotation2d(),
    // getModulePositions(), aPose2d );
    poseEstimator.resetPosition(gyro.getRotation2d(), getModulePositions(), aPose2d);
  }

  public void configureAutoBuilder() {
    // Configure AutoBuilder last
    AutoBuilder.configure(
        this::getPose, // Robot pose supplier
        this::resetPose, // Method to reset odometry (will be called if your auto hasa starting pose)
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT
        // RELATIVE
        // ChassisSpeeds. Also optionally outputs
        // individual
        // module feedforwards
        new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for
            // holonomic
            // drive trains
            new PIDConstants(0.48, 0, 0.0), // Translation PID constants
            new PIDConstants(11.5, 0, 0.0) // Rotation PID constants
        ),
        config, // The robot configuration
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
    );
  }

  /**
   * See {@link SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double)}.
   */
  public void addVisionMeasurement(Pose2d visionMeasurement, double timestampSeconds) {
    poseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds);
  }

  /**
   * See
   * {@link SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double, Matrix)}.
   */
  public void addVisionMeasurement(
      Pose2d visionMeasurement, double timestampSeconds, Matrix<N3, N1> stdDevs) {
    poseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds, stdDevs);
  }

}
