package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Utils;

public class PracticeShooter extends SubsystemBase {
    private final int speedDifferenceStartDecelerate = 2000;

    private TalonFX[] shooterMotors;
    private TalonFX kickerMotor;

    private SpeedChangeProcess currentProcess;
    private boolean disabled = false;

    public PracticeShooter(int[] shooterPorts, int kickerPort) {
        shooterMotors = new TalonFX[shooterPorts.length];
        for (int i = 0; i < shooterPorts.length; i++)
            this.shooterMotors[i] = new TalonFX(shooterPorts[i]);

        this.kickerMotor = new TalonFX(kickerPort);
    }

    public void setShooterSpeed(int speedRPM) {
        this.currentProcess = new SpeedChangeProcess((int) shooterMotors[0].getSelectedSensorVelocity(), speedRPM);
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
        double speedDifference = targetedSpeed - shooterMotors[0].getSelectedSensorVelocity();

        double feedBackPower = getCurrentSpeedRPM() / this.speedDifferenceStartDecelerate;

        if (disabled) feedBackPower = 0;

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

    static class SpeedChangeProcess {
        private final int shooterMaxAccelerationRPMPerSec = 1000;
        private final double shooterMaxMotorPower = 0.3;

        private Timer taskTimer;
        private int startingRPM;
        private int targetedRPM;

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
