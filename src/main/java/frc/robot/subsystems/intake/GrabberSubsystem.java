package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.ArmSettings;
import frc.team_8840_lib.controllers.specifics.SparkMaxEncoderWrapper;

public class GrabberSubsystem extends SubsystemBase {
    private static GrabberSubsystem instance;
    public static GrabberSubsystem getInstance() {
        return instance;
    }

    private static final boolean enabled = false;

    public enum GrabberState {
        OPEN,
        CLOSED,
        ACTIVE;
    }

    public enum GrabberDirection {
        IN,
        OUT;
    }

    public enum LoadedPiece {
        NONE,
        CUBE,
        CONE;
    }

    public enum ConeHoldStrat {
        NONE,
        SLOW_BACKWARDS;
    }

    private GrabberState state;
    private GrabberDirection direction;
    private LoadedPiece loadedPiece;

    private ConeHoldStrat coneHoldStrat;

    private CANSparkMax grabberMotor;
    private SparkMaxEncoderWrapper grabberEncoder;

    private double customSpeed = 0;

    public GrabberSubsystem() {
        instance = this;

        if (!enabled) return;

        state = GrabberState.OPEN;
        direction = GrabberDirection.IN;
        loadedPiece = LoadedPiece.NONE;

        grabberMotor = new CANSparkMax(ArmSettings.Grabber.PORT, MotorType.kBrushless);
        grabberEncoder = new SparkMaxEncoderWrapper(grabberMotor);

        this.setupController();
    }

    public void setupController() {
        grabberMotor.restoreFactoryDefaults();

        grabberMotor.setSmartCurrentLimit(25);
        grabberMotor.setSecondaryCurrentLimit(30);

        grabberMotor.setOpenLoopRampRate(0.5);
        grabberMotor.setClosedLoopRampRate(0.5);

        grabberMotor.setInverted(true);

        grabberMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        grabberMotor.setCANTimeout(10);

        grabberMotor.burnFlash();
    }

    public void intake(LoadedPiece piece) {
        state = GrabberState.ACTIVE;
        direction = GrabberDirection.IN;
        loadedPiece = piece;
    }

    public void stopIntake() {
        state = GrabberState.CLOSED;
    }

    public void outtake() {
        state = GrabberState.ACTIVE;
        direction = GrabberDirection.OUT;
        loadedPiece = LoadedPiece.NONE;
    }

    public void outtake(double speed) {
        customSpeed = speed;
        outtake();
    }

    public void stopOuttake() {
        state = GrabberState.OPEN;
        customSpeed = 0;
    }

    public void stopPickOrPlace() {
        if (direction == GrabberDirection.IN) {
            stopIntake();
        } else {
            stopOuttake();
        }
    }

    public GrabberState getState() {
        return state;
    }

    public GrabberDirection getDirection() {
        return direction;
    }

    public LoadedPiece getLoadedPiece() {
        return loadedPiece;
    }

    public void updateDashboard() {
        SmartDashboard.putString("grabber", toString());
    }

    @Override
    public String toString() {
        return "GrabberSubsystem [direction=" + direction + ", loadedPiece=" + loadedPiece + ", state=" + state + "]";
    }

    @Override
    public void periodic() {
        if (!enabled) return;

        if (coneHoldStrat == ConeHoldStrat.SLOW_BACKWARDS && loadedPiece == LoadedPiece.CONE && state == GrabberState.CLOSED) {
            grabberMotor.set(0.02);
        }

        if (state == GrabberState.ACTIVE) {
            if (direction == GrabberDirection.IN) {
                grabberMotor.set(-0.5);
            } else {
                grabberMotor.set(customSpeed == 0 ? 0.1 : customSpeed);
            }
        } else {
            grabberMotor.set(0);
        }
    }
}
