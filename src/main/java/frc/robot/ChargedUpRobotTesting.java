package frc.robot;

import com.revrobotics.CANSparkMax;

import java.io.FileNotFoundException;
import java.io.IOException;
import java.nio.file.Path;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import frc.team_8840_lib.listeners.EventListener;

public class ChargedUpRobotTesting extends TimedRobot {

    private PWMSparkMax testMotor;
    
    public void robotInit() {

        testMotor = new PWMSparkMax(0);
    }   

    
    public void autonomousInit() {
        // TODO Auto-generated method stub
        
    }

    
    public void autonomousPeriodic() {
        // TODO Auto-generated method stub
        
    }


    
    public void teleopInit() {
        // TODO Auto-generated method stub
        
    }

    
    public void teleopPeriodic() {
        // TODO Auto-generated method stub
        
        testMotor.set(0.5);
    }
}
