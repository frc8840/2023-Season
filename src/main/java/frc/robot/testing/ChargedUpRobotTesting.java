package frc.robot.testing;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import frc.team_8840_lib.input.controls.SimulatedController;
import frc.team_8840_lib.utils.controls.Axis;

public class ChargedUpRobotTesting extends TimedRobot {

    private PWMSparkMax testMotor;
    private SimulatedController simControls;
    
    public void robotInit() {
        testMotor = new PWMSparkMax(0);

        simControls = new SimulatedController();
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
        testMotor.set(simControls.getAxis(Axis.Vertical));
    }
}
