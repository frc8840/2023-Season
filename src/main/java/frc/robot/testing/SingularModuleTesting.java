package frc.robot.testing;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.utils.ConfigureSettings;
import frc.team_8840_lib.controllers.SwerveModule;
import frc.team_8840_lib.info.console.Logger;
import frc.team_8840_lib.input.communication.CommunicationManager;
import frc.team_8840_lib.input.controls.SimulatedController;
import frc.team_8840_lib.listeners.EventListener;
import frc.team_8840_lib.utils.controllers.swerve.CTREConfig;
import frc.team_8840_lib.utils.controllers.swerve.SwerveSettings;
import frc.team_8840_lib.utils.controls.Axis;

public class SingularModuleTesting extends EventListener {


    private SwerveModule module;

    private final int drivePort = 0;
    private final int turnPort = 1;
    private final int driveEncoderPort = -1;

    SimulatedController simController;
    
    private SwerveSettings settings;

    //ROBOT INIT

    @Override
    public void robotInit() {
        settings = ConfigureSettings.getSwerveDriveSettings();

        module = new SwerveModule(drivePort, turnPort, 1, 0, new CTREConfig(settings));

        simController = new SimulatedController();
    }

    //ROBOT PERIODIC

    @Override
    public void robotPeriodic() {
        
    }
    
    //AUTONOMOUS METHODS

    @Override
    public void onAutonomousEnable() {

    }

    @Override
    public void onAutonomousPeriodic() {

    }

    //TELEOPERATED METHODS

    @Override
    public void onTeleopEnable() {
        
    }

    @Override
    public void onTeleopPeriodic() {        
        double rotation = ((360 - simController.getAxis(Axis.Twist)) / 360) * 2 * Math.PI;

        double speed = simController.getAxis(Axis.Vertical);

        CommunicationManager.getInstance()
            .updateInfo("feedback", "speed", speed)
            .updateInfo("feedback", "rotation", rotation);

        Logger.Log("speed: " + speed + " rotation: " + rotation);

        module.setDesiredState(new SwerveModuleState(speed * settings.maxSpeed, new Rotation2d(rotation)), false);
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
