package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils;

public class PracticeShooter extends SubsystemBase {
    private final int speedDifferenceStartDecelerate = 2000;

    private TalonFX[] shooterMotors;
    private TalonFX kickerMotor;

    private SpeedChangeProcess currentProcess;
    private boolean disabled = false;
    private double percentErrorIntegration;

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

        this.percentErrorIntegration = 0;
        this.kickerMotor = new TalonFX(kickerPort);
        this.currentProcess = new SpeedChangeProcess(0, 0);
        kickerMotor.setInverted(reverted[reverted.length-1]);
    }

    public void setShooterSpeed(int speedRPM) {
        this.currentProcess = new SpeedChangeProcess((int) getCurrentSpeedRPM(), speedRPM);
        this.percentErrorIntegration = 0;
    }

    public void disableShooter() {
        this.disabled = true;
    }

    public void enableShooter() {
        this.disabled = false;
    }

    @Override
    public void periodic() {
        double targetedSpeed = currentProcess.sampleCurrentVelocity();
        double speedDifference = targetedSpeed - getCurrentSpeedRPM();

        double feedBackPower = speedDifference * currentProcess.shooterMaxMotorPower / this.speedDifferenceStartDecelerate;
        feedBackPower = MathUtil.clamp(feedBackPower, -currentProcess.shooterMaxMotorPower, currentProcess.shooterMaxMotorPower);

        if (disabled) feedBackPower = 0;

        for (TalonFX shooterMotor : shooterMotors) {
            shooterMotor.set(TalonFXControlMode.PercentOutput, feedBackPower);
        }
        
        System.out.println(getCurrentSpeedRPM() + "," + speedDifference * currentProcess.shooterMaxMotorPower / this.speedDifferenceStartDecelerate);
    }

    private double getCurrentSpeedRPM() {
        return Utils.falconToRPM(this.shooterMotors[0].getSelectedSensorVelocity(), 1);
    }

    public void setDefaultShooterSpeed() {
        setShooterSpeed(3000);
    }

    static class SpeedChangeProcess {
        private final int shooterMaxAccelerationRPMPerSec = 1000;
        private final double shooterMaxMotorPower = 0.3;

        private final Timer taskTimer;
        private final int startingRPM;
        private final int targetedRPM;

        public SpeedChangeProcess(int startingRPM, int targetedRPM) {
            taskTimer = new Timer();
            taskTimer.start();
            taskTimer.reset();
            this.startingRPM = startingRPM;
            this.targetedRPM = targetedRPM;
        }

        public double sampleCurrentVelocity() {
            if (targetedRPM - startingRPM > 0)
                return Math.min(
                        startingRPM + taskTimer.get() * this.shooterMaxAccelerationRPMPerSec,
                        targetedRPM
                );
            return Math.max(
                    startingRPM - taskTimer.get() * this.shooterMaxAccelerationRPMPerSec,
                    targetedRPM
            );
        }
    }
}
