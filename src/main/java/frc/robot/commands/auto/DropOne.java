package frc.robot.commands.auto;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.coral.lili.LIPlaceCoral;
import frc.robot.commands.driving.DriveToLocation;
import frc.robot.model.PathContainer;
import frc.robot.subsystems.LiliCoralSubystem;
import frc.robot.subsystems.drivetrainIOLayers.DrivetrainIO;

public class DropOne extends SequentialCommandGroup {
    public DropOne(DrivetrainIO D, LaserCan lc, LiliCoralSubystem c, PathContainer pathContainer) {
        addCommands(new DriveToLocation(D, lc,
                pathContainer),
                new LIPlaceCoral(c));
    }

}
