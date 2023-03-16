package frc.robot.commands;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.team_8840_lib.input.communication.CommunicationManager;

public class EstimatePosition extends CommandBase {
    private static final double kMaxReliableSpeed = 1.0;
    private static final double kMaxReliableAngularSpeed = 1.0;

    public static enum Mode {
        NONE,
        FOLLOW;
    }

    private VisionSubsystem visionSubsystem;

    public EstimatePosition(VisionSubsystem visionSubsystem) {
        this.visionSubsystem = visionSubsystem;
        addRequirements(visionSubsystem);

        this.visionSubsystem.setRelativeEstimationOn(true);
    }

    @Override
    public void execute() {
        EstimatedRobotPose estimatedRobotPose;

        try {
            estimatedRobotPose = visionSubsystem.getEstimatedRobotPose();
        } catch (Exception e) {
            return;
        }

        boolean hasTarget = visionSubsystem.hasTarget();

        if (hasTarget) {
            // PhotonTrackedTarget[] relativePosition = visionSubsystem.getRelativePositions();

            // for (int i = 0; i < relativePosition.length; i++) {
            //     int id = relativePosition[i].getFiducialId();
            //     Transform3d transform = relativePosition[i].getBestCameraToTarget();

            //     CommunicationManager.getInstance()
            //         .updateInfo("apriltags", "tag" + id + "/x", transform.getTranslation().getX())
            //         .updateInfo("apriltags", "tag" + id + "/y", transform.getTranslation().getY())
            //         .updateInfo("apriltags", "tag" + id + "/z", transform.getTranslation().getZ());
            // }

            try {
                if (CommunicationManager.getInstance().fieldExists()) {
                    CommunicationManager.getInstance().updateFieldObjectPose("EstPose", 
                        new Pose2d(
                            new Translation2d(
                                estimatedRobotPose.estimatedPose.getX(),
                                estimatedRobotPose.estimatedPose.getY()
                            ),
                            estimatedRobotPose.estimatedPose.getRotation().toRotation2d()
                        )
                    );

                }
            } catch (Exception e) { }
        } else if (CommunicationManager.getInstance().fieldExists()) {
            CommunicationManager.getInstance().updateFieldObjectPose("EstPose", 
                    new Pose2d(
                    new Translation2d(
                        0,
                        0
                    ),
                    Rotation2d.fromDegrees(0)
                )
            );
        }

        //ChassisSpeeds currentVelocity = RobotContainer.getInstance().getDriveSubsystem().getSwerveDrive().getVelocity();

        // boolean goingSlowEnough = !(
        //     Math.abs(currentVelocity.vxMetersPerSecond) > kMaxReliableSpeed ||
        //     Math.abs(currentVelocity.vyMetersPerSecond) > kMaxReliableSpeed ||
        //     Math.abs(currentVelocity.omegaRadiansPerSecond) > kMaxReliableAngularSpeed
        // );

        // if (!goingSlowEnough) return;

        //TODO: use estimated position and save it to a variable.
    }
}
