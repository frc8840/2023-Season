package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.ConfigureSettings;
import frc.robot.utils.ModuleConstants;
import frc.team_8840_lib.controllers.SwerveGroup;
import frc.team_8840_lib.input.communication.CommunicationManager;
import frc.team_8840_lib.utils.controllers.Pigeon;
import frc.team_8840_lib.utils.controllers.swerve.SwerveSettings;

public class DriveSubsystem extends SubsystemBase {
    private SwerveGroup swerveDrive;

    /**
     * Creates a new DriveSubsystem.
     */
    public DriveSubsystem() {
        //Generate the SwerveSettings object.
        SwerveSettings settings = ConfigureSettings.getSwerveDriveSettings();

        int[] drivePorts = new int[4];
        int[] turnPorts = new int[4];
        int[] encoderPorts = new int[4];

        drivePorts[SwerveGroup.ModuleIndex.kTOP_LEFT.getIndex()] = ModuleConstants.TopLeft.DRIVE;
        turnPorts[SwerveGroup.ModuleIndex.kTOP_LEFT.getIndex()] = ModuleConstants.TopLeft.TURN;
        encoderPorts[SwerveGroup.ModuleIndex.kTOP_LEFT.getIndex()] = ModuleConstants.TopLeft.ENCODER;

        drivePorts[SwerveGroup.ModuleIndex.kTOP_RIGHT.getIndex()] = ModuleConstants.TopRight.DRIVE;
        turnPorts[SwerveGroup.ModuleIndex.kTOP_RIGHT.getIndex()] = ModuleConstants.TopRight.TURN;
        encoderPorts[SwerveGroup.ModuleIndex.kTOP_RIGHT.getIndex()] = ModuleConstants.TopRight.ENCODER;

        drivePorts[SwerveGroup.ModuleIndex.kBOTTOM_LEFT.getIndex()] = ModuleConstants.BottomLeft.DRIVE;
        turnPorts[SwerveGroup.ModuleIndex.kBOTTOM_LEFT.getIndex()] = ModuleConstants.BottomLeft.TURN;
        encoderPorts[SwerveGroup.ModuleIndex.kBOTTOM_LEFT.getIndex()] = ModuleConstants.BottomLeft.ENCODER;

        drivePorts[SwerveGroup.ModuleIndex.kBOTTOM_RIGHT.getIndex()] = ModuleConstants.BottomRight.DRIVE;
        turnPorts[SwerveGroup.ModuleIndex.kBOTTOM_RIGHT.getIndex()] = ModuleConstants.BottomRight.TURN;
        encoderPorts[SwerveGroup.ModuleIndex.kBOTTOM_RIGHT.getIndex()] = ModuleConstants.BottomRight.ENCODER;

        //Set the SwerveGroup object to the swerve drive.
        swerveDrive = new SwerveGroup("NEO Swerve Drive", settings,
                drivePorts,
                turnPorts,
                encoderPorts,
                new Pigeon(Pigeon.Type.TWO, ModuleConstants.PIGEON_ID, false)
        );

        swerveDrive.providePower(true);
        swerveDrive.doStateOptimization(true);
        swerveDrive.doSetAngle(true);
        swerveDrive.doSetSpeed(true);
    }
    
    /**
     * Get the SwerveGroup object.
     * @return SwerveGroup object
     */
    public SwerveGroup getSwerveDrive() {
        return swerveDrive;
    }

    /**
     * Check if the swerve drive is ready.
     * @return true if ready, false otherwise
     */
    public boolean ready() {
        return swerveDrive.ready();
    }

    /**
     * Reset the odometry of the swerve drive.
     * @param pos the position to reset to
     */
    public void resetOdometry(Pose2d pos) {
        swerveDrive.resetOdometry(pos);
    }

    /**
     * Reset the odometry of the swerve drive.
     */
    public void resetOdometry() {
        resetOdometry(new Pose2d(7, 4, new Rotation2d(0)));
    }

    public static enum BrakeMode {
        NORMAL, BRAKE, COAST
    }

    /**
     * Set the brake mode of the swerve drive.
     * @param mode the brake mode to set to
     */
    public void setBrakeMode(BrakeMode mode) {
        switch (mode) {
            case BRAKE:
                swerveDrive.setIndividualBrakeModes(true, true);
                break;
            case COAST:
                swerveDrive.setIndividualBrakeModes(false, false);
                break;
            case NORMAL:
            default:
                swerveDrive.setIndividualBrakeModes(true, false);
        }
    }

    @Override
    public void periodic() {
        if (swerveDrive != null) {
            //Update about swerve info
            CommunicationManager.getInstance().updateSwerveInfo(swerveDrive);
            
            //If field exists
            if (CommunicationManager.getInstance().fieldExists()) {
                swerveDrive.updateOdometry(); //Update swerve odometry

                //Update the position of the field robot
                CommunicationManager.getInstance().updateFieldObjectPose("SwerveRobot", swerveDrive.getPose());

                //Update even more stuff
                swerveDrive.updateFieldRobot();
            }
        }
    }

    
}
