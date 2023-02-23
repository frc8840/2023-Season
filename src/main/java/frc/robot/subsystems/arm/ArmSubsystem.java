package frc.robot.subsystems.arm;

import static java.lang.Math.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.ArmSettings;
import frc.robot.utils.Measurements;
import frc.team_8840_lib.controllers.specifics.SparkMaxEncoderWrapper;
import frc.team_8840_lib.input.communication.CommunicationManager;
import frc.team_8840_lib.listeners.Robot;
import frc.team_8840_lib.utils.math.MathUtils;
import frc.team_8840_lib.utils.math.units.Cartesian2d;

import static frc.team_8840_lib.utils.math.MathUtils.*;

/**
 * The arm subsystem.
 * 
 * @author Sean Kuwamoto, Jaiden Grimminck
 */
public class ArmSubsystem extends SubsystemBase {
    public static Translation2d[] SIDE_VIEW_ROBOT = new Translation2d[] {
        //Front corners
        new Translation2d(-Measurements.Robot.LENGTH / 2, 0),
        new Translation2d(-Measurements.Robot.LENGTH / 2, Measurements.Robot.BASE_HEIGHT),
        
        //Back corners
        new Translation2d(Measurements.Robot.LENGTH / 2, 0),
        new Translation2d(Measurements.Robot.LENGTH / 2, Measurements.Robot.BASE_HEIGHT)
        
    };

    private static ArmSubsystem instance;

    public static ArmSubsystem getInstance() {
        return instance;
    }

    private CANSparkMax baseMotor;
    private CANSparkMax elbowMotor;

    private SparkMaxEncoderWrapper baseEncoder;
    private SparkMaxEncoderWrapper elbowEncoder;

    private SparkMaxPIDController basePID;
    private SparkMaxPIDController elbowPID;

    private ArmFeedforward baseFeedforward;
    private ArmFeedforward elbowFeedforward;

    private double baseAngleCache;
    private double elbowAngleCache;

    public ArmSubsystem() {
        instance = this;

        baseMotor = new CANSparkMax(ArmSettings.Base.PORT, MotorType.kBrushless);
        elbowMotor = new CANSparkMax(ArmSettings.Elbow.PORT, MotorType.kBrushless);

        baseEncoder = new SparkMaxEncoderWrapper(baseMotor);
        elbowEncoder = new SparkMaxEncoderWrapper(elbowMotor);

        basePID = baseMotor.getPIDController();
        elbowPID = elbowMotor.getPIDController();

        baseFeedforward = new ArmFeedforward(ArmSettings.Base.kS, ArmSettings.Base.kV, ArmSettings.Base.kG);
        elbowFeedforward = new ArmFeedforward(ArmSettings.Elbow.kS, ArmSettings.Elbow.kV, ArmSettings.Elbow.kG);

        if (RobotBase.isSimulation()) {
            REVPhysicsSim.getInstance().addSparkMax(baseMotor, DCMotor.getNEO(1));
            REVPhysicsSim.getInstance().addSparkMax(elbowMotor, DCMotor.getNEO(1));
        }

        configMotors();
    }

    private void configMotors() {
        baseMotor.restoreFactoryDefaults();
        elbowMotor.restoreFactoryDefaults();

        baseMotor.setInverted(ArmSettings.Base.INVERTED);
        elbowMotor.setInverted(ArmSettings.Elbow.INVERTED);

        baseMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        elbowMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        baseMotor.setSmartCurrentLimit(40);
        elbowMotor.setSmartCurrentLimit(40);

        //Set secondary current limits to 30 amps
        baseMotor.setSecondaryCurrentLimit(30);
        elbowMotor.setSecondaryCurrentLimit(30);

        //Enable voltage compensation
        baseMotor.enableVoltageCompensation(12);
        elbowMotor.enableVoltageCompensation(12);

        //TODO: Set position conversion factor
        baseEncoder.setPositionConversionFactor(1);
        elbowEncoder.setPositionConversionFactor(1);

        //Set base PID
        basePID.setP(ArmSettings.Base.PID.kP);
        basePID.setI(ArmSettings.Base.PID.kI);
        basePID.setD(ArmSettings.Base.PID.kD);
        basePID.setFF(ArmSettings.Base.PID.kF);
        basePID.setIZone(ArmSettings.Base.PID.kIZone);

        //Set elbow PID
        elbowPID.setP(ArmSettings.Elbow.PID.kP);
        elbowPID.setI(ArmSettings.Elbow.PID.kI);
        elbowPID.setD(ArmSettings.Elbow.PID.kD);
        elbowPID.setFF(ArmSettings.Elbow.PID.kF);
        elbowPID.setIZone(ArmSettings.Elbow.PID.kIZone);

        //Burn to flash
        baseMotor.burnFlash();
        elbowMotor.burnFlash();

        baseEncoder.doSubtractionOfStart(true);
        elbowEncoder.doSubtractionOfStart(true);
    }

    public void setPosition(Rotation2d basePosition, Rotation2d elbowPosition) {
        setBasePosition(basePosition);
        setElbowPosition(elbowPosition);
    }

    public void setBasePosition(Rotation2d basePositionRot) {
        double basePosition = basePositionRot.getDegrees();

        baseAngleCache = basePosition;

        basePID.setReference(
            baseEncoder.calculatePosition(basePosition), 
            ControlType.kPosition,
            0, 
            baseFeedforward.calculate(basePosition, ArmSettings.Base.kVelocity)
        );
    }

    public void setElbowPosition(Rotation2d elbowPositionRot) {
        double elbowPosition = elbowPositionRot.getDegrees();

        elbowAngleCache = elbowPosition;

        elbowPID.setReference(
            elbowEncoder.calculatePosition(elbowPosition), 
            ControlType.kPosition,
            0,
            elbowFeedforward.calculate(elbowPosition, ArmSettings.Elbow.kVelocity)
        );
    }

    public void reportToNetworkTables() {
        CommunicationManager.getInstance()
            .updateInfo("arm", "elbowAngle", getRelativeElbowAngle())
            .updateInfo("arm", "realElbowAngle", getRealElbowAngle())
            .updateInfo("arm", "baseAngle", getBaseAngle());
    }

    /**
     * Calculates the position the arm should be at based on some position. 
     * This is assuming a sideview of the arm, x is forward/horozontal, y is up.
     * @param position The position of the end effector
     * @return The position of the base and elbow
     */
    public Rotation2d[] calculatePositions(Translation2d position, boolean flipped) {
        /**
         * 0: Base position
         * 1: Elbow position
         */
        Rotation2d[] positions = new Rotation2d[] {
            Rotation2d.fromDegrees(0),
            Rotation2d.fromDegrees(0)
        };

        double x = position.getX();
        double y = position.getY();

        if (flipped) {
            positions = calculatePositions(new Translation2d(-x, y), false);
            positions[0] = Rotation2d.fromRadians(Math.PI - positions[0].getRadians());
            positions[1] = Rotation2d.fromRadians(-positions[1].getRadians());
            return positions;
        }

        if (position.getDistance(new Translation2d(0, 0)) >= (ArmSettings.Base.armLengthMeters + ArmSettings.Elbow.armLengthMeters)) {
            //We'll set it so they're both pointing towards the target
            positions[0] = Rotation2d.fromRadians(atan2(y, x));
            positions[1] = Rotation2d.fromDegrees(0);
            return positions;
        }

        double pointDistance = clamp(
            Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2)),
            abs(ArmSettings.Base.armLengthMeters - ArmSettings.Elbow.armLengthMeters), 
            ArmSettings.Base.armLengthMeters + ArmSettings.Elbow.armLengthMeters
        );

        double pointAngle = atan2(y, x);

        //θ + cos^-1((a_2^2 - a_1^2 - d^2) / (-2 * a_1 + d))
        double baseAngle = pointAngle + acos(
            (pow(ArmSettings.Elbow.armLengthMeters, 2) - pow(ArmSettings.Base.armLengthMeters, 2) - pow(pointDistance, 2)) / 
            (-2 * ArmSettings.Base.armLengthMeters * pointDistance)
        );
 
        //-PI + cos^-1((d^2 - a_1^2 - a_2^2) / (-2 * a_1 * a_2))
        double elbowAngle = -PI + acos(
            (pow(pointDistance, 2) - pow(ArmSettings.Base.armLengthMeters, 2) - pow(ArmSettings.Elbow.armLengthMeters, 2)) / 
            (-2 * ArmSettings.Base.armLengthMeters * ArmSettings.Elbow.armLengthMeters)
        );

        positions[0] = Rotation2d.fromRadians(baseAngle);
        positions[1] = Rotation2d.fromRadians(elbowAngle);

        // if (positions[0].getDegrees() < 0) {
        //     positions[0] = Rotation2d.fromDegrees(360 + positions[0].getDegrees());
        // }

        if (positions[1].getDegrees() < 0) {
            positions[1] = Rotation2d.fromDegrees(360 + positions[1].getDegrees());
        }

        return positions;
    }

    public Rotation2d[] calculatePositions(Translation2d position) {
        return calculatePositions(position, position.getX() > 0);
    }

    public Rotation2d[] translateToRelativeAngles(Rotation2d[] angles) {
        double baseLowerBound = ArmSettings.Base.LOWER_BOUND_DEGREES;
        double baseUpperBound = ArmSettings.Base.UPPER_BOUND_DEGREES;

        // Apply the offset so the lower bound is 0
        double newBaseAngle = angles[0].getDegrees() - baseLowerBound;

        double newElbowAngle = angles[1].getDegrees() + baseLowerBound;

        // // Make sure that lower_bound < a < 360 + lower_bound
        while (newBaseAngle < 0) {
            newBaseAngle += 360;
        }

        newBaseAngle = clamp(newBaseAngle, 0.001, baseUpperBound - baseLowerBound);

        return new Rotation2d[] {
            Rotation2d.fromDegrees(newBaseAngle),
            Rotation2d.fromDegrees(newElbowAngle)
        };
    }

    public Rotation2d[] magicWitchcraft(Rotation2d[] arms, Translation2d endpoints) {
        if (endpoints.getX() < 0) {
            arms = calculatePositions(new Translation2d(-endpoints.getX(), endpoints.getY()));
            arms[0] = Rotation2d.fromRadians(PI - arms[0].getRadians());
            arms[1] = Rotation2d.fromRadians(-arms[1].getRadians());
        }

        arms[0] = Rotation2d.fromRadians(clamp(arms[0].getRadians(), 0, PI));

        if (arms[0].getRadians() == PI) {
            arms[1] = Rotation2d.fromRadians(atan2(-endpoints.getY(), -ArmSettings.Base.armLengthMeters - endpoints.getY()));
        }

        if (arms[0].getRadians() == 0) {
            arms[1] = Rotation2d.fromRadians(atan2(endpoints.getY(), ArmSettings.Base.armLengthMeters - endpoints.getY()));
        }

        Translation2d endOfArm = new Cartesian2d(ArmSettings.Base.armLengthMeters, arms[0]).toTranslation2d();

        //Magic witchcraft
        endOfArm = new Translation2d(endOfArm.getX(), endOfArm.getY() + 0.00001);

        Rotation2d angle1 = Rotation2d.fromRadians(
            2 * PI + min(
                atan2(
                    SIDE_VIEW_ROBOT[0].getY() - endOfArm.getY(),
                    SIDE_VIEW_ROBOT[0].getX() - endOfArm.getX()
                ),
                atan2(
                    SIDE_VIEW_ROBOT[1].getY() - endOfArm.getY(),
                    SIDE_VIEW_ROBOT[1].getX() - endOfArm.getX()
                )
            )
        );

        Rotation2d angle2 = Rotation2d.fromRadians(
            Math.max(
                Math.atan2(
                    SIDE_VIEW_ROBOT[2].getY() - endOfArm.getY(),
                    SIDE_VIEW_ROBOT[2].getX() - endOfArm.getX()
                ),
                Math.atan2(
                    SIDE_VIEW_ROBOT[3].getY() - endOfArm.getY(),
                    SIDE_VIEW_ROBOT[3].getX() - endOfArm.getX()
                )
            )
        );

        if (endpoints.getX() > 0 && endOfArm.getDistance(SIDE_VIEW_ROBOT[3]) < ArmSettings.Elbow.armLengthMeters) {
            arms[1] = Rotation2d.fromRadians(
                max(
                    angle2.getRadians() - arms[0].getRadians(),
                    arms[1].getRadians()
                )
            );
        } else if (endpoints.getX() <= 0 && endOfArm.getDistance(SIDE_VIEW_ROBOT[1]) < ArmSettings.Elbow.armLengthMeters) {
            arms[1] = Rotation2d.fromRadians(
                min(
                    angle1.getRadians() - arms[0].getRadians(),
                    arms[1].getRadians()
                )
            );
        }

        return arms;
    }

    public double getRealElbowAngle() {
        if (Robot.isSimulation()) {
            return elbowAngleCache + getBaseAngle();
        }

        //The real angle is what it actually is in the real world
        return elbowEncoder.getPosition() + getBaseAngle();
    }

    public double getRelativeElbowAngle() {
        if (Robot.isSimulation()) {
            return elbowAngleCache;
        }

        return elbowEncoder.getPosition();
    }

    public double getBaseAngle() {
        if (RobotBase.isSimulation()) {
            return baseAngleCache;
        }

        return baseEncoder.getPosition();
    }
}
