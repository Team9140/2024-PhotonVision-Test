package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    private static Arm instance;

    private CANSparkMax motor;
    private DutyCycleEncoder aps;

    private ArmFeedforward feedforward = new ArmFeedforward(Constants.Arm.kS, Constants.Arm.kG, Constants.Arm.kV,
            Constants.Arm.kA);

    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(Math.PI * 3.0, Math.PI * 1.50);

    private volatile TrapezoidProfile.State armTargetState;

    private volatile TrapezoidProfile.State armCurrentState;

    public static Arm getInstance() {
        return (Arm.instance == null) ? Arm.instance = new Arm() : Arm.instance;
    }

    public enum ArmState {
        STARTUP,
        POSITION,
        FAULT
    }

    private ArmState currentArmState;
    private double startUpTime;

    private TrapezoidProfile profile;

    private Arm() {
        this.motor = new CANSparkMax(10, CANSparkLowLevel.MotorType.kBrushless);
        this.motor.setSmartCurrentLimit(Constants.Arm.CURRENT_LIMIT);
        this.motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        this.motor.setInverted(true);
        this.motor.getEncoder().setPositionConversionFactor(Constants.Arm.POSITION_CONVERSION);
        this.motor.getEncoder().setVelocityConversionFactor(Constants.Arm.VELOCITY_CONVERSION);

        this.motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 3.85f);
        this.motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        this.motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, -1.00f);
        this.motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

        this.motor.getPIDController().setP(Constants.Arm.P);
        this.motor.getPIDController().setD(Constants.Arm.D);

        this.aps = new DutyCycleEncoder(Constants.Ports.ARM_ENCODER_DIO);

        // Minimum: 0.8955
        // 0 Deg: 0.2839
        // Limit: 0.1326

        this.currentArmState = ArmState.STARTUP;
        this.startUpTime = Timer.getFPGATimestamp();

        this.armTargetState = new TrapezoidProfile.State(Constants.Arm.Positions.STOW, 0.0);
        this.armCurrentState = new TrapezoidProfile.State(Constants.Arm.Positions.STOW, 0.0);

        this.profile = new TrapezoidProfile(constraints);
        // this.setRadians(this.getMotorRadians());
    }

    private double getAPSRadians() {
        return Math.PI * 2 * (this.aps.getAbsolutePosition() - 0.2839);
    }

    private double getMotorRadians() {
        return this.motor.getEncoder().getPosition();
    }

    private double getMotorRadiansPerSecond() {
        return this.motor.getEncoder().getVelocity();
    }

    int debugCounter = 0;

    @Override
    public void periodic() {
        double time = Timer.getFPGATimestamp();
        ArmState nextArmState = this.currentArmState;
        switch (this.currentArmState) {
            case STARTUP:
                if (time - this.startUpTime < 2.0) {
                    //
                    // } else if (Math.abs(this.getAPSRadians() - this.getMotorRadians()) >
                    // Constants.Arm.READY_DEADZONE) {
                    // // this.motor.getEncoder().setPosition(this.getAPSRadians());
                    this.motor.getEncoder().setPosition(Constants.Arm.Positions.STOW);
                } else {
                    nextArmState = ArmState.POSITION;
                    this.armTargetState = new TrapezoidProfile.State(Constants.Arm.Positions.STOW, 0.0);
                }
                break;
            case POSITION:
                // if (Math.abs(this.getAPSRadians() - this.getMotorRadians()) >
                // Constants.Arm.FAULT_DEADZONE) {
                // nextArmState = ArmState.FAULT;
                // }

                double FF = feedforward.calculate(armCurrentState.position, armCurrentState.velocity);
                armCurrentState = this.profile.calculate(TimedRobot.kDefaultPeriod, armCurrentState, armTargetState);

                motor.getPIDController().setReference(armCurrentState.position, CANSparkMax.ControlType.kPosition, 0, FF);
                SmartDashboard.putNumber("debug", debugCounter++);
                break;
            case FAULT:
                break;
        }

        this.currentArmState = nextArmState;

        SmartDashboard.putString("Arm State", this.currentArmState.toString());
        SmartDashboard.putNumber("Arm APS (raw)", this.aps.getAbsolutePosition());
        SmartDashboard.putNumber("Arm APS (radians)", this.getAPSRadians());
        SmartDashboard.putNumber("Arm NEO (radians)", this.getMotorRadians());
        SmartDashboard.putNumber("Arm Motor / APS Difference (radians)", this.getAPSRadians() - this.getMotorRadians());
        SmartDashboard.putNumber("Arm Output (V)", this.motor.getAppliedOutput() * 12.0);
        SmartDashboard.putNumber("Arm Current (A)", this.motor.getOutputCurrent());
        SmartDashboard.putNumber("Arm Temperature (C)", this.motor.getMotorTemperature());
        SmartDashboard.putNumber("Arm Final Target (radians)", this.armTargetState.position);
        SmartDashboard.putNumber("Arm Profile Target (radians)", this.armCurrentState.position);
    }

    public boolean isUp() {
        return this.getTargetStatePosition() <= 3.0 || this.getMotorRadians() <= 3.0;
        // return ((Math.sin(this.getTargetStatePosition()) >= 0) || (Math.sin(this.getCurrentStatePosition()) >= 0) || (Math.abs(this.getTargetStatePosition() - this.getCurrentStatePosition()) >= Math.PI));
    }

    public double getTargetStatePosition() {
        return this.armTargetState.position;
    };

    public double getCurrentStatePosition() {
        return this.armCurrentState.position;
    }

    public Command setRadians(double rad) {
        return this.runOnce(() -> this.armTargetState = new TrapezoidProfile.State(rad, 0.0));
    }

    public Command setRadiansAndFinish(double rad) {
        return run(() -> this.armTargetState = new TrapezoidProfile.State(rad, 0.0))
                .until(() -> (Math.abs(rad - this.getMotorRadians()) < Math.toRadians(2.5)));
    }

    public Command setStow() {
        return this.runOnce(() -> this.armTargetState = new TrapezoidProfile.State(Constants.Arm.Positions.STOW, 0.0));
    }

    public Command setHighNode() {
        return this.runOnce(() -> this.armTargetState = new TrapezoidProfile.State(Constants.Arm.Positions.HIGH_NODE, 0.0));

    }

    public Command setMidNode() {
        return this.runOnce(() -> this.armTargetState = new TrapezoidProfile.State(Constants.Arm.Positions.MID_NODE, 0.0));
    }

    public Command setFloor() {
        return this.runOnce(() -> this.armTargetState = new TrapezoidProfile.State(Constants.Arm.Positions.FLOOR, 0.0));
    }
}
