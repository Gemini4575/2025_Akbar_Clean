package frc.robot.subsystems.drive;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
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
import com.pathplanner.lib.config.RobotConfig;
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

    // configureAutoBuilder();

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
        frontLeft_3.getState(),
        frontRight_2.getState(),
        backRight_1.getState()
    };
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
        backLeft_0.getPosition(),
        frontLeft_3.getPosition(),
        frontRight_2.getPosition(),
        backRight_1.getPosition()
    };
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    xSpeed_cur = xSpeed;
    ySpeed_cur = ySpeed;
    rot_cur = rot;
    SmartDashboard.putNumber("[Drivetrain]Gyro", gyro.getAngle());
    var swerveModuleStates = m_kinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(-1 * xSpeed, ySpeed, rot, gyro.getRotation2d())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);

    SmartDashboard.putString("[Drivetrain]gyro", m_gyro.getRotation2d().toString());
    SmartDashboard.putString("[Drivetrain]module 0", swerveModuleStates[0].toString());
    SmartDashboard.putString("[Drivetrain]module 1", swerveModuleStates[1].toString());
    SmartDashboard.putString("[Drivetrain]module 2", swerveModuleStates[2].toString());
    SmartDashboard.putString("[Drivetrain]module 3", swerveModuleStates[3].toString());

    setModuleStates(swerveModuleStates);
  }

  private void setModuleStates(SwerveModuleState[] swerveModuleStates) {
    m_backLeft_0.setDesiredState(swerveModuleStates[0]);
    m_frontLeft_1.setDesiredState(swerveModuleStates[1]);
    m_frontRight_2.setDesiredState(swerveModuleStates[2]);
    m_backRight_3.setDesiredState(swerveModuleStates[3]);
  }

  // public void configureAutoBuilder() {
  // // Configure AutoBuilder last
  // AutoBuilder.configure(
  // this::getPose, // Robot pose supplier
  // this::resetPose, // Method to reset odometry (will be called if your auto has
  // a starting pose)
  // this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT
  // RELATIVE
  // (speeds, feedforwards) -> driveForPathPlanner(speeds), // Method that will
  // drive the robot given ROBOT
  // // RELATIVE
  // // ChassisSpeeds. Also optionally outputs
  // // individual
  // // module feedforwards
  // new PPHolonomicDriveController( // PPHolonomicController is the built in path
  // following controller for
  // // holonomic
  // // drive trains
  // new PIDConstants(15.5, 0, 0.0), // Translation PID constants
  // new PIDConstants(6, 0.0, 0.0) // Rotation PID constants
  // ),
  // config, // The robot configuration
  // () -> {
  // // Boolean supplier that controls when the path will be mirrored for the red
  // // alliance
  // // This will flip the path being followed to the red side of the field.
  // // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

  // // var alliance = DriverStation.getAlliance();
  // // if (alliance.isPresent()) {
  // // return alliance.get() == DriverStation.Alliance.Red;
  // // }
  // return false;
  // },
  // this // Reference to this subsystem to set requirements
  // );
  // }

}
