package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.EnhancedPIDController;
import frc.robot.Utils;

/**
 * enhanced shooter, applied profiled-PID algorithm to fly wheel control
 * @Author Sam
 * @Version 0.1
 */
public class EnhancedShooter extends Shooter {
    private EnhancedPIDController pidController;
    private boolean disabled;
    private boolean onWaitAndShootTask;
    private double desiredShooterRPM;
    private final double shooterSpeedToleranceRPM = 500;

    public EnhancedShooter() {
        super();
        disabled = false;
        onWaitAndShootTask = false;
        desiredShooterRPM = 0;
        pidController =  new EnhancedPIDController(new EnhancedPIDController.DynamicalPIDProfile(
                0.8,
                0,
                0,
                0,
                0.8 / 3000,
                1500,
                6000
        ));
    }

    /**
     * update the status of the shooter
     */
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Recorded RPM Left", Utils.falconToRPM(super.m_left_follower.getSelectedSensorVelocity(), 1.0));
        SmartDashboard.putNumber("Recorded RPM Right", Utils.falconToRPM(super.m_right_master.getSelectedSensorVelocity(), 1.0));

        double feedBackPower = 0;
        if (!disabled)
            feedBackPower = pidController.getMotorPower(0, super.get_shooter_rpm());

        m_right_master.set(TalonFXControlMode.PercentOutput, feedBackPower);

        if (Math.abs(get_shooter_rpm() - desiredShooterRPM) < shooterSpeedToleranceRPM && onWaitAndShootTask) {
            onWaitAndShootTask = false;
            super.runKicker();
        }
    }

    /**
     * set the targeted speed of the fly wheel, called once during each operation, and the updating process will be proceeded in each following period
     * @param desiredShooterRPM the desired speed of the fly wheel, in rpm
     */
    public void setDesiredFlyWheelSpeed(double desiredShooterRPM) {
        this.desiredShooterRPM = desiredShooterRPM;
        this.pidController.startNewTaskKeepIntegration(new EnhancedPIDController.Task(
                EnhancedPIDController.Task.SET_TO_SPEED, desiredShooterRPM
        ));
    }

    /**
     * sets the shooter speed into the speed matching the target distance, according to the check-table
     * (does not include shooting)
     * @param distance the distance to the target, units should match the check-table
     * */
    @Override
    public void shoot_from_anywhere(double distance){
        double rpm = get_target_rpm(distance);
        setDesiredFlyWheelSpeed(rpm);
    }

    @Override
    public void stop() {
        setDesiredFlyWheelSpeed(0);
        super.stop();
    }

    /**
     * add a task to wait for the shooter to accelerate to its current desired speed(which is not set in this scope) and initiate kicker
     * the task will be proceeded in the following periods
     *  */
    public void setWaitAndShootTask() {
        onWaitAndShootTask = true;
    }


    /** disable the shooter(should be called whenever robot is set to disabled */
    public void setDisabled() {
        this.disabled = true;
    }

    /** enable the shooter */
    public void setEnabled() {
        this.disabled = false;
    }

    /**
     * whether there is a shooting task in progress
     * @return (true or false)
     * */
    public boolean isOnWaitAndShootTask() {
        return onWaitAndShootTask;
    }
}
