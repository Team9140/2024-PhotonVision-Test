package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private static Intake instance;
  private CANSparkMax motor;

  private double nextThrowVolts = Constants.Intake.OFF;

  private Intake() {
    this.motor = new CANSparkMax(11, CANSparkLowLevel.MotorType.kBrushed);
  }

  public static Intake getInstance() {
    return Intake.instance == null ? Intake.instance = new Intake() : Intake.instance;
  }

  public Command intakeCone() {
    return run(() -> {
      this.motor.setSmartCurrentLimit(Constants.Intake.INTAKE_CURRENT_LIMIT);
      this.motor.set(Constants.Intake.INTAKE_CONE_VOLTS);
    });
  }

  public Command holdCone() {
    return run(() -> {
      this.motor.setSmartCurrentLimit(Constants.Intake.HOLD_CURRENT_LIMIT);
      this.motor.setVoltage(Constants.Intake.HOLD_CONE_VOLTS);
      this.nextThrowVolts = Constants.Intake.THROW_CONE_VOLTS;
    });
  }

  public Command intakeCube() {
    return run(() -> {
      this.motor.set(Constants.Intake.INTAKE_CUBE_VOLTS);
      this.motor.setSmartCurrentLimit(Constants.Intake.INTAKE_CURRENT_LIMIT);
    });
  }

  public Command holdCube() {
    return run(() -> {
      this.motor.setSmartCurrentLimit(Constants.Intake.HOLD_CURRENT_LIMIT);
      this.motor.setVoltage(Constants.Intake.HOLD_CUBE_VOLTS);
      this.nextThrowVolts = Constants.Intake.THROW_CUBE_VOLTS;
    });
  }

  public Command throwItem() {
    return run(() -> {
      this.motor.setVoltage(this.nextThrowVolts);
      this.motor.setSmartCurrentLimit(Constants.Intake.INTAKE_CURRENT_LIMIT);
    });
  }

  public Command off() {
    return runOnce(() -> this.motor.setVoltage(Constants.Intake.OFF));
  }

  @Override
  public void periodic() {}
}
