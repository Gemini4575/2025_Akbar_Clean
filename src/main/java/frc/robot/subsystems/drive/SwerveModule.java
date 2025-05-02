package frc.robot.subsystems.drive;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogInput;
import frc.lib.math.SwerveModuleConstants;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule {

    private ProfiledPIDController turningPidController = new ProfiledPIDController(
            1.0, // Proportional gain
            0.0, // Integral gain
            0.0,
            new TrapezoidProfile.Constraints(
                    SwerveConstants.kModuleMaxAngularVelocity,
                    SwerveConstants.kModuleMaxAngularAcceleration));

    private ProfiledPIDController drivingPidController = new ProfiledPIDController(
            1.0, // Proportional gain
            0.0, // Integral gain
            0.0,
            new TrapezoidProfile.Constraints(
                    SwerveConstants.MaxMetersPersecond,
                    SwerveConstants.kMaxAceceration));

    private SparkMax driveMotor;
    private SparkMax angleMotor;

    private AnalogInput Encoder;

    private double angleOffset;

    public SwerveModule(SwerveModuleConstants s) {
        driveMotor = new SparkMax(s.driveMotorID, MotorType.kBrushless);
        angleMotor = new SparkMax(s.angleMotorID, MotorType.kBrushless);
        SparkBaseConfig driveMotorConfig = new SparkMaxConfig();
        SparkBaseConfig angleMotorConfig = new SparkMaxConfig();
        driveMotorConfig.smartCurrentLimit(40, 40);
        driveMotorConfig.disableFollowerMode();
        driveMotorConfig.inverted(true);
        driveMotorConfig.idleMode(IdleMode.kBrake);
        angleMotorConfig.apply(driveMotorConfig);

        driveMotor.configure(driveMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        angleMotor.configure(angleMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        Encoder = new AnalogInput(s.cancoderID);

        angleOffset = s.angleOffset;
    }

    public void SetDesiredState(SwerveModuleState desiredState) {

    }
}
