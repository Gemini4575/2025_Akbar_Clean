package frc.robot.model;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public record VisionEstimateContainer(EstimatedRobotPose estimatedPose, Matrix<N3, N1> stdDev) {

}
