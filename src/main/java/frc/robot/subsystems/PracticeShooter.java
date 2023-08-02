package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.EnhancedPIDController;
import frc.robot.Utils;

public class PracticeShooter extends SubsystemBase {
    private final int speedDifferenceStartDecelerate = 2000;

    private TalonFX[] shooterMotors;
    private TalonFX kickerMotor;
    private boolean disabled = false;
    private EnhancedPIDController pidController;

    public PracticeShooter(int[] shooterPorts, int kickerPort, boolean[] reverted) {
        shooterMotors = new TalonFX[shooterPorts.length];
        for (int i = 0; i < shooterPorts.length; i++) {
            this.shooterMotors[i] = new TalonFX(shooterPorts[i]);
            this.shooterMotors[i].configFactoryDefault();
            this.shooterMotors[i].configVoltageCompSaturation(12.0);
            this.shooterMotors[i].enableVoltageCompensation(true);
            if (reverted[i])
                this.shooterMotors[i].setInverted(InvertType.InvertMotorOutput);
        }

        this.kickerMotor = new TalonFX(kickerPort);
        kickerMotor.setInverted(reverted[reverted.length-1]);

        this.pidController = new EnhancedPIDController(new EnhancedPIDController.DynamicalPIDProfile(
                0.8,
                0,
                0,
                0,
                0.8 / 3000,
                1500,
                6000
        ));
    }

    public void setShooterSpeed(int speedRPM) {
        this.pidController.startNewTaskKeepIntegration(new EnhancedPIDController.Task(EnhancedPIDController.Task.SET_TO_SPEED, speedRPM));
    }

    public void disableShooter() {
        this.disabled = true;
    }

    public void enableShooter() {
        this.disabled = false;
    }

    @Override
    public void periodic() {
        double feedBackPower;

        if (disabled)
            feedBackPower = 0;
        else
            feedBackPower = pidController.getMotorPower(0, getCurrentSpeedRPM());
        SmartDashboard.putNumber("current speed(rpm): " , getCurrentSpeedRPM());
        SmartDashboard.putNumber("feed-back power:", feedBackPower);

        for (TalonFX shooterMotor : shooterMotors) {
            shooterMotor.set(TalonFXControlMode.PercentOutput, feedBackPower);
        }
    }

    private double getCurrentSpeedRPM() {
        return Utils.falconToRPM(this.shooterMotors[0].getSelectedSensorVelocity(), 1);
    }

    public void setDefaultShooterSpeed() {
        setShooterSpeed(3000);
    }

    public void setHigherShooterSpeed() {
        setShooterSpeed(6000);
    }

    public void setZeroShooterSpeed() { setShooterSpeed(0); }
}
