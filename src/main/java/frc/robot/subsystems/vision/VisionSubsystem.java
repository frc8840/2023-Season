package frc.robot.subsystems.vision;

import java.util.List;
import java.util.Optional;

import javax.xml.crypto.dsig.Transform;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.VisionSettings;
import frc.team_8840_lib.info.console.Logger;
import frc.team_8840_lib.input.communication.CommunicationManager;
import frc.team_8840_lib.listeners.Robot;

public class VisionSubsystem extends SubsystemBase {
    private static VisionSubsystem instance;
    public static VisionSubsystem getInstance() {
        return instance;
    }

    private PhotonCamera camera;
    private AprilTagFieldLayout fieldLayout;
    private PhotonPoseEstimator poseEstimator;

    private boolean doRelativeEstimation = false;
    private PhotonTrackedTarget[] relativePositions;

    public void setRelativeEstimationOn(boolean doRelativeEstimation) {
        this.doRelativeEstimation = doRelativeEstimation;
    }

    public VisionSubsystem() {
        instance = this;

        if (Robot.isReal()) {
            camera = new PhotonCamera(VisionSettings.CAMERA_NAME);

            try {
                fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);

                poseEstimator = new PhotonPoseEstimator(
                    fieldLayout,
                    PoseStrategy.LOWEST_AMBIGUITY,
                    camera,
                    VisionSettings.cameraPosition
                );
            } catch (Exception e) {
                Logger.Log("[VisionSubsystem] ERROR: Failed to load field layout.");
            }
        }
    }

    private boolean hasTarget = false;
    private EstimatedRobotPose estimatedRobotPose;

    @Override
    public void periodic() {
        if (Robot.isSimulation() || Robot.isReal()) return;
        if (!Robot.getRealInstance().ready()) return;

        PhotonPipelineResult result;

        try {
            result = camera.getLatestResult();
        } catch (Exception e) {
            //We'll just try again next time.
            return;
        }

        boolean hasTarget = result.hasTargets();

        if (hasTarget) {
            try {
                Optional<EstimatedRobotPose> poseResult = poseEstimator.update(result);
                if (poseResult.isPresent()) {
                    estimatedRobotPose = poseResult.get();
                }
            } catch (Exception e) {
                //return;
            }
        }

        if (hasTarget) {
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

        this.hasTarget = hasTarget;

        if (this.doRelativeEstimation && hasTarget) {
            List<PhotonTrackedTarget> targets = result.getTargets();
            
            relativePositions = new PhotonTrackedTarget[targets.size()];
            for (int i = 0; i < targets.size(); i++) {
                relativePositions[i] = targets.get(i);
            }
        }
    }

    public boolean hasTarget() {
        return hasTarget;
    }

    public EstimatedRobotPose getEstimatedRobotPose() {
        return estimatedRobotPose;
    }

    public PhotonTrackedTarget[] getRelativePositions() {
        return relativePositions;
    }
}
