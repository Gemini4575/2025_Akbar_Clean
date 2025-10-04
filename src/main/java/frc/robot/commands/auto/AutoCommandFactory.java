package frc.robot.commands.auto;

import static frc.robot.LocationData.START_TO_REEF_FRONT;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.coral.lili.LIPlaceCoral;
import frc.robot.commands.driving.DriveToLocation;
import frc.robot.model.PathContainer;
import frc.robot.subsystems.LiliCoralSubystem;
import frc.robot.subsystems.drivetrainIOLayers.DrivetrainIO;

// TODO
public class AutoCommandFactory {

    private final LiliCoralSubystem coralSubystem;
    private final DrivetrainIO drivetrainIO;
    private final LaserCan laserCan;

    public AutoCommandFactory(DrivetrainIO drivetrainIO, LaserCan laserCan, LiliCoralSubystem coralSubystem) {
        this.drivetrainIO = drivetrainIO;
        this.laserCan = laserCan;
        this.coralSubystem = coralSubystem;
    }

    public Command dropOneCenter() {
        return new SequentialCommandGroup(drive(START_TO_REEF_FRONT), placeCoral());
    }

    private Command drive(PathContainer path) {
        return new DriveToLocation(drivetrainIO, laserCan, path);
    }

    private Command placeCoral() {
        return new LIPlaceCoral(coralSubystem);
    }

}
