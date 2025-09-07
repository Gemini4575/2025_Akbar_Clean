package robot;

import static frc.robot.Constants.SwerveConstants.m_backLeftLocation;
import static frc.robot.Constants.SwerveConstants.m_backRightLocation;
import static frc.robot.Constants.SwerveConstants.m_frontLeftLocation;
import static frc.robot.Constants.SwerveConstants.m_frontRightLocation;
import static org.junit.jupiter.api.Assertions.fail;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class PoseEstimationTest {

    @Test
    public void test() {
        SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_backLeftLocation,
                m_backRightLocation,
                m_frontRightLocation, m_frontLeftLocation);

        var startTime = System.currentTimeMillis() / 1000;

        var stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
        var visionStdDevs = VecBuilder.fill(1, 1, 1);

        var initialGyroPose = Rotation2d.fromDegrees(0);

        var initialModulePositions = new SwerveModulePosition[] {
                new SwerveModulePosition(0.0, Rotation2d.fromDegrees(0)),
                new SwerveModulePosition(0.0, Rotation2d.fromDegrees(0)),
                new SwerveModulePosition(0.0, Rotation2d.fromDegrees(0)),
                new SwerveModulePosition(0.0, Rotation2d.fromDegrees(0))
        };

        var poseEstimator = new SwerveDrivePoseEstimator(
                m_kinematics,
                initialGyroPose,
                initialModulePositions,
                new Pose2d(),
                stateStdDevs,
                visionStdDevs);

        poseEstimator.resetPosition(Rotation2d.fromDegrees(0), initialModulePositions,
                new Pose2d(1, 1, Rotation2d.fromDegrees(0)));

        System.out.println("Initial pose: " + poseEstimator.getEstimatedPosition());

        var modulePosition2 = new SwerveModulePosition[] {
                new SwerveModulePosition(1.0, Rotation2d.fromDegrees(0)),
                new SwerveModulePosition(1.0, Rotation2d.fromDegrees(0)),
                new SwerveModulePosition(1.0, Rotation2d.fromDegrees(0)),
                new SwerveModulePosition(1.0, Rotation2d.fromDegrees(0))
        };

        var gyroPose2 = Rotation2d.fromDegrees(0);

        poseEstimator.updateWithTime(startTime + 1, gyroPose2, modulePosition2);

        System.out.println("Pose2: " + poseEstimator.getEstimatedPosition());
    }

}
