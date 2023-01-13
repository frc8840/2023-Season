package frc.robot;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import frc.team_8840_lib.listeners.EventListener;

public class ChargedUpRobotTesting extends EventListener {

    private PWMSparkMax testMotor;

    @Override
    public void robotInit() {
        testMotor = new PWMSparkMax(0);
    }

    @Override
    public void robotPeriodic() {

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
    public void onTeleopEnable() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void onTeleopPeriodic() {
        // TODO Auto-generated method stub
        
        testMotor.set(0.5);
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
    public void onDisabled() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void onDisabledPeriodic() {
        // TODO Auto-generated method stub
        
    }
}
