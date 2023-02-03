package frc.robot.testing;

import com.revrobotics.REVPhysicsSim;

import frc.robot.utils.ConfigureSettings;
import frc.robot.utils.Ports;
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
                    Ports.TopLeft.DRIVE,
                    Ports.TopRight.DRIVE,
                    Ports.BottomLeft.DRIVE,
                    Ports.BottomRight.DRIVE
                }, 

                new int[] { //Turn/Steering IDs
                    Ports.TopLeft.TURN,
                    Ports.TopRight.TURN,
                    Ports.BottomLeft.TURN,
                    Ports.BottomRight.TURN
                }, 

                new int[] { //Encoder IDs
                    Ports.TopLeft.ENCODER, 
                    Ports.TopRight.ENCODER, 
                    Ports.BottomLeft.ENCODER, 
                    Ports.BottomRight.ENCODER
                },

                new Pigeon(Pigeon.Type.TWO, Ports.PIGEON_ID, false)
        );        
    }

    @Override
    public void onAutonomousEnable() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void onAutonomousPeriodic() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void onDisabled() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void onDisabledPeriodic() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void onTeleopEnable() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void onTeleopPeriodic() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void onTestEnable() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void onTestPeriodic() {
        // TODO Auto-generated method stub
        
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
