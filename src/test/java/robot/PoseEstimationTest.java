package robot;

import static frc.robot.Constants.SwerveConstants.m_backLeftLocation;
import static frc.robot.Constants.SwerveConstants.m_backRightLocation;
import static frc.robot.Constants.SwerveConstants.m_frontLeftLocation;
import static frc.robot.Constants.SwerveConstants.m_frontRightLocation;
import static org.junit.jupiter.api.Assertions.fail;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N3;

public class PoseEstimationTest {

        private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_backLeftLocation,
                        m_backRightLocation,
                        m_frontRightLocation, m_frontLeftLocation);

        private final Vector<N3> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
        private final Vector<N3> visionStdDevs = VecBuilder.fill(1, 1, 1);

        /**
         * This is the most basic test - every angle is zero. This means the robot front
         * is facing right (positive X) and wheels point right (positive X).
         * We move each wheel 1 meter keeping their angle at zero.
         * The result is that the robot moved right by one meter.
         */
        @Test
        public void test_basicStraight() {
                var startTime = System.currentTimeMillis() / 1000;

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

                System.out.println("New pose: " + poseEstimator.getEstimatedPosition());
        }

        /**
         * In this test, we start with the robot at 90 degrees, which means it's facing
         * up (positive Y).
         * The wheels angle is 0 which means they are aligned with the robot and also
         * point up (positive Y).
         * We move each wheel 1 meter keeping their angle at zero.
         * The result is that the robot moved up by one meter.
         */
        @Test
        public void test_basicStraightUp() {
                var startTime = System.currentTimeMillis() / 1000;

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
                                new Pose2d(1, 1, Rotation2d.fromDegrees(90)));

                System.out.println("Initial pose: " + poseEstimator.getEstimatedPosition());

                var modulePosition2 = new SwerveModulePosition[] {
                                new SwerveModulePosition(1.0, Rotation2d.fromDegrees(0)),
                                new SwerveModulePosition(1.0, Rotation2d.fromDegrees(0)),
                                new SwerveModulePosition(1.0, Rotation2d.fromDegrees(0)),
                                new SwerveModulePosition(1.0, Rotation2d.fromDegrees(0))
                };

                var gyroPose2 = Rotation2d.fromDegrees(0);

                poseEstimator.updateWithTime(startTime + 1, gyroPose2, modulePosition2);

                System.out.println("New pose: " + poseEstimator.getEstimatedPosition());
        }

        /**
         * In this test, we start with the robot at 0 degrees facing right (positive X).
         * The wheels angle is -90 which means they are perpendicular to the robot
         * pointing up (positive Y).
         * We move each wheel 1 meter keeping their angle at -90.
         * The result is that the robot moved up by one meter.
         */
        @Test
        public void test_UpSideways() {
                var startTime = System.currentTimeMillis() / 1000;

                var initialGyroPose = Rotation2d.fromDegrees(0);

                var initialModulePositions = new SwerveModulePosition[] {
                                new SwerveModulePosition(0.0, Rotation2d.fromDegrees(90)),
                                new SwerveModulePosition(0.0, Rotation2d.fromDegrees(90)),
                                new SwerveModulePosition(0.0, Rotation2d.fromDegrees(90)),
                                new SwerveModulePosition(0.0, Rotation2d.fromDegrees(90))
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
                                new SwerveModulePosition(1.0, Rotation2d.fromDegrees(90)),
                                new SwerveModulePosition(1.0, Rotation2d.fromDegrees(90)),
                                new SwerveModulePosition(1.0, Rotation2d.fromDegrees(90)),
                                new SwerveModulePosition(1.0, Rotation2d.fromDegrees(90))
                };

                var gyroPose2 = Rotation2d.fromDegrees(0);

                poseEstimator.updateWithTime(startTime + 1, gyroPose2, modulePosition2);

                System.out.println("New pose: " + poseEstimator.getEstimatedPosition());
        }

        /**
         * In this test, we start robot at 0 facing right, but all wheels are at 45
         * degrees. We drive 1 meter keeping wheels at the same angle. The robot ends up
         * facing right diagonally up on both X and Y axis (moved up by 0.71 on each
         * axis)
         */
        @Test
        public void test_diagonal() {
                var startTime = System.currentTimeMillis() / 1000;

                var initialGyroPose = Rotation2d.fromDegrees(0);

                var initialModulePositions = new SwerveModulePosition[] {
                                new SwerveModulePosition(0.0, Rotation2d.fromDegrees(45)),
                                new SwerveModulePosition(0.0, Rotation2d.fromDegrees(45)),
                                new SwerveModulePosition(0.0, Rotation2d.fromDegrees(45)),
                                new SwerveModulePosition(0.0, Rotation2d.fromDegrees(45))
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
                                new SwerveModulePosition(1.0, Rotation2d.fromDegrees(45)),
                                new SwerveModulePosition(1.0, Rotation2d.fromDegrees(45)),
                                new SwerveModulePosition(1.0, Rotation2d.fromDegrees(45)),
                                new SwerveModulePosition(1.0, Rotation2d.fromDegrees(45))
                };

                var gyroPose2 = Rotation2d.fromDegrees(0);

                poseEstimator.updateWithTime(startTime + 1, gyroPose2, modulePosition2);

                System.out.println("New pose: " + poseEstimator.getEstimatedPosition());
        }

        /**
         * In this test, we see what happens when wheel direction is reversed. We do the
         * same thing as in the diagonal test, but half the wheel point in the opposite
         * direction (45 - 180 degrees) and they also drive in the opposite direction
         * (-1 meter). The result is exactly the same as in the regular diagonal test.
         */
        @Test
        public void test_diagonalMixedDirections() {
                var startTime = System.currentTimeMillis() / 1000;

                var initialGyroPose = Rotation2d.fromDegrees(0);

                var initialModulePositions = new SwerveModulePosition[] {
                                new SwerveModulePosition(0.0, Rotation2d.fromDegrees(45 - 180)),
                                new SwerveModulePosition(0.0, Rotation2d.fromDegrees(45)),
                                new SwerveModulePosition(0.0, Rotation2d.fromDegrees(45 - 180)),
                                new SwerveModulePosition(0.0, Rotation2d.fromDegrees(45))
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
                                new SwerveModulePosition(-1.0, Rotation2d.fromDegrees(45 - 180)),
                                new SwerveModulePosition(1.0, Rotation2d.fromDegrees(45)),
                                new SwerveModulePosition(-1.0, Rotation2d.fromDegrees(45 - 180)),
                                new SwerveModulePosition(1.0, Rotation2d.fromDegrees(45))
                };

                var gyroPose2 = Rotation2d.fromDegrees(0);

                poseEstimator.updateWithTime(startTime + 1, gyroPose2, modulePosition2);

                System.out.println("New pose: " + poseEstimator.getEstimatedPosition());
        }

        /**
         * In this test, we attempt to spin in place but pointing all wheels in a circle
         * and driving the same direction. The pose estimator then reports the same
         * location as the starting point, with robot angle whatever we set the gyro to
         * be (in this example 90)
         */
        @Test
        public void test_spinInPlace() {
                var startTime = System.currentTimeMillis() / 1000;

                var initialGyroPose = Rotation2d.fromDegrees(0);

                var initialModulePositions = new SwerveModulePosition[] {
                                new SwerveModulePosition(0.0, Rotation2d.fromDegrees(45)),
                                new SwerveModulePosition(0.0, Rotation2d.fromDegrees(180 - 45)),
                                new SwerveModulePosition(0.0, Rotation2d.fromDegrees(180 + 45)),
                                new SwerveModulePosition(0.0, Rotation2d.fromDegrees(-45))
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
                                new SwerveModulePosition(1.0, Rotation2d.fromDegrees(45)),
                                new SwerveModulePosition(1.0, Rotation2d.fromDegrees(180 - 45)),
                                new SwerveModulePosition(1.0, Rotation2d.fromDegrees(180 + 45)),
                                new SwerveModulePosition(1.0, Rotation2d.fromDegrees(-45))
                };

                var gyroPose2 = Rotation2d.fromDegrees(90);

                poseEstimator.updateWithTime(startTime + 1, gyroPose2, modulePosition2);

                System.out.println("New pose: " + poseEstimator.getEstimatedPosition());
        }

        /**
         * This test reflects how our robot is actually configured on startup. We set
         * both gyro and initial heading to 180. We also reset gyro to zero. So if we
         * just move straight for one meter, at the end of the movemenet our gyro will
         * be zero. This confuses the estimator. It outputs that we moved up on Y axis
         * by 0.64m which doesn't make sense.
         */
        @Test
        public void test_ourRobot() {
                var startTime = System.currentTimeMillis() / 1000;

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

                poseEstimator.resetPosition(Rotation2d.fromDegrees(180), initialModulePositions,
                                new Pose2d(1, 1, Rotation2d.fromDegrees(180)));

                System.out.println("Initial pose: " + poseEstimator.getEstimatedPosition());

                var modulePosition2 = new SwerveModulePosition[] {
                                new SwerveModulePosition(1.0, Rotation2d.fromDegrees(0)),
                                new SwerveModulePosition(1.0, Rotation2d.fromDegrees(0)),
                                new SwerveModulePosition(1.0, Rotation2d.fromDegrees(0)),
                                new SwerveModulePosition(1.0, Rotation2d.fromDegrees(0))
                };

                var gyroPose2 = Rotation2d.fromDegrees(0);

                poseEstimator.updateWithTime(startTime + 1, gyroPose2, modulePosition2);

                System.out.println("New pose: " + poseEstimator.getEstimatedPosition());
        }

        /**
         * This is what I beliveve is the correct initial robot config. Gyro should be
         * set to zero since we reset it when we start. Initial heading is 90 degrees
         * since the robot faces up (positive Y). If we move straight 1 meter, we
         * correctly end up 1 meter higher (positive Y)
         */
        @Test
        public void test_correctRobotConfig() {
                var startTime = System.currentTimeMillis() / 1000;

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
                                new Pose2d(1, 1, Rotation2d.fromDegrees(90)));

                System.out.println("Initial pose: " + poseEstimator.getEstimatedPosition());

                var modulePosition2 = new SwerveModulePosition[] {
                                new SwerveModulePosition(1.0, Rotation2d.fromDegrees(0)),
                                new SwerveModulePosition(1.0, Rotation2d.fromDegrees(0)),
                                new SwerveModulePosition(1.0, Rotation2d.fromDegrees(0)),
                                new SwerveModulePosition(1.0, Rotation2d.fromDegrees(0))
                };

                var gyroPose2 = Rotation2d.fromDegrees(0);

                poseEstimator.updateWithTime(startTime + 1, gyroPose2, modulePosition2);

                System.out.println("New pose: " + poseEstimator.getEstimatedPosition());
        }

        @Test
        public void test_correctRobotConfig2() {
                var startTime = System.currentTimeMillis() / 1000;

                var initialGyroPose = Rotation2d.fromDegrees(-90);

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

                poseEstimator.resetPosition(Rotation2d.fromDegrees(-90), initialModulePositions,
                                new Pose2d(1, 1, Rotation2d.fromDegrees(90)));

                System.out.println("Initial pose: " + poseEstimator.getEstimatedPosition());

                var modulePosition2 = new SwerveModulePosition[] {
                                new SwerveModulePosition(1.0, Rotation2d.fromDegrees(0)),
                                new SwerveModulePosition(1.0, Rotation2d.fromDegrees(0)),
                                new SwerveModulePosition(1.0, Rotation2d.fromDegrees(0)),
                                new SwerveModulePosition(1.0, Rotation2d.fromDegrees(0))
                };

                var gyroPose2 = Rotation2d.fromDegrees(-90);

                poseEstimator.updateWithTime(startTime + 1, gyroPose2, modulePosition2);

                System.out.println("New pose: " + poseEstimator.getEstimatedPosition());
        }

}
