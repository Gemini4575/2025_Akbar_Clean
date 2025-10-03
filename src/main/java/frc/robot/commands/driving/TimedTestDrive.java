package frc.robot.commands.driving;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lidar;
import frc.robot.subsystems.drivetrainIOLayers.DrivetrainIO;

public class TimedTestDrive extends Command {

    private final long durationMillis;
    private final double speed;
    private final DrivetrainIO driveTrain;
    private final Lidar lidar;

    private long startTime;
    private Pose2d initialPose;
    private double initialLidarDistance;

    public TimedTestDrive(DrivetrainIO driveTrain, Lidar lidar, long durationMillis, double speed) {
        this.durationMillis = durationMillis;
        this.speed = speed;
        this.driveTrain = driveTrain;
        this.lidar = lidar;
        addRequirements(driveTrain, lidar);
    }

    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
        initialPose = driveTrain.getPose();
        initialLidarDistance = lidar.getDistanceMeters();
    }

    @Override
    public void execute() {
        driveTrain.drive(-speed, 0, 0, true);
        SmartDashboard.putNumber("[TimedTestDrive] drive time", System.currentTimeMillis() - startTime);
        SmartDashboard.putNumber("[TimedTestDrive] estimated distance",
                initialPose.getTranslation().getDistance(driveTrain.getPose().getTranslation()));
        SmartDashboard.putNumber("[TimedTestDrive] Lidar distance",
                lidar.getDistanceMeters() - initialLidarDistance);
    }

    @Override
    public boolean isFinished() {
        var done = System.currentTimeMillis() - startTime >= durationMillis;
        if (done) {
            driveTrain.drive(0, 0, 0, false);
        }
        return done;
    }

}
