package frc.robot.testing;

import com.revrobotics.REVPhysicsSim;

import frc.robot.utils.ConfigureSettings;
import frc.robot.utils.ModuleConstants;
import frc.team_8840_lib.controllers.SwerveGroup;
import frc.team_8840_lib.input.communication.CommunicationManager;
import frc.team_8840_lib.listeners.EventListener;
import frc.team_8840_lib.listeners.Robot;
import frc.team_8840_lib.utils.controllers.Pigeon;
import frc.team_8840_lib.utils.controllers.swerve.SwerveSettings;

public class EmptySwerve extends EventListener {

    public SwerveGroup swerveDrive;

    @Override
    public void robotInit() {
        //Generate the SwerveSettings object.
        SwerveSettings settings = ConfigureSettings.getSwerveDriveSettings();

        //Set the SwerveGroup object to the swerve drive.
        swerveDrive = new SwerveGroup("NEO Swerve Drive", settings,
                new int[] { //Drive IDs
                    ModuleConstants.TopLeft.DRIVE,
                    ModuleConstants.TopRight.DRIVE,
                    ModuleConstants.BottomLeft.DRIVE,
                    ModuleConstants.BottomRight.DRIVE
                }, 

                new int[] { //Turn/Steering IDs
                    ModuleConstants.TopLeft.TURN,
                    ModuleConstants.TopRight.TURN,
                    ModuleConstants.BottomLeft.TURN,
                    ModuleConstants.BottomRight.TURN
                }, 

                new int[] { //Encoder IDs
                    ModuleConstants.TopLeft.ENCODER, 
                    ModuleConstants.TopRight.ENCODER, 
                    ModuleConstants.BottomLeft.ENCODER, 
                    ModuleConstants.BottomRight.ENCODER
                },

                new Pigeon(Pigeon.Type.TWO, ModuleConstants.PIGEON_ID, false)
        );        
    }

    @Override
    public void onAutonomousEnable() {
        
        
    }

    @Override
    public void onAutonomousPeriodic() {
        
        
    }

    @Override
    public void onDisabled() {
        
        
    }

    @Override
    public void onDisabledPeriodic() {
        
        
    }

    @Override
    public void onTeleopEnable() {
        
        
    }

    @Override
    public void onTeleopPeriodic() {
        
        
    }

    @Override
    public void onTestEnable() {
        
        
    }

    @Override
    public void onTestPeriodic() {
        
        
    }

    @Override
    public void robotPeriodic() {
        //Run the REVPhysicsSim if we're in simulation mode.
        if (Robot.isSimulation()) {
            REVPhysicsSim.getInstance().run();
        }

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
