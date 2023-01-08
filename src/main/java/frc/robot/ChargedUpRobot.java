package frc.robot;

import com.revrobotics.REVPhysicsSim;

import frc.robot.utils.ConfigureSettings;
import frc.team_8840_lib.listeners.EventListener;
import frc.team_8840_lib.listeners.Robot;
import frc.team_8840_lib.utils.controllers.swerve.SwerveSettings;

public class ChargedUpRobot extends EventListener {
    //ROBOT INIT

    @Override
    public void robotInit() {
        //Generate the SwerveSettings object.
        SwerveSettings settings = ConfigureSettings.getSwerveDriveSettings();
    }

    //ROBOT PERIODIC

    @Override
    public void robotPeriodic() {
        //Run the REVPhysicsSim if we're in simulation mode.
        if (Robot.isSimulation()) {
            REVPhysicsSim.getInstance().run();
        }
    }
    
    //AUTONOMOUS METHODS

    @Override
    public void onAutonomousEnable() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void onAutonomousPeriodic() {
        // TODO Auto-generated method stub
        
    }

    //TELEOPERATED METHODS

    @Override
    public void onTeleopEnable() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void onTeleopPeriodic() {
        // TODO Auto-generated method stub
        
    }

    // DISABLED METHODS

    @Override
    public void onDisabled() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void onDisabledPeriodic() {
        // TODO Auto-generated method stub
        
    }

    // TESTING METHODS

    @Override
    public void onTestEnable() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void onTestPeriodic() {
        // TODO Auto-generated method stub
        
    }
}
