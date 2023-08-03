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
    private boolean wasEnabled = false;
    private boolean onWaitAndShootTask;
    private double desiredShooterRPM;
    private final double shooterSpeedToleranceRPM = 200;

    public EnhancedShooter() {
        super();
        disabled = false;
        onWaitAndShootTask = false;
        desiredShooterRPM = 0;
        pidController =  new EnhancedPIDController(new EnhancedPIDController.DynamicalPIDProfile(
                0.85,
                0,
                0,
                0,
                1.6 / 3000,
                3000,
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

        // System.out.println("desired RPM" + desiredShooterRPM);
        double feedBackPower = 0;
        if (!disabled)
            feedBackPower = pidController.getMotorPower(0, super.get_shooter_rpm());
        // System.out.println("disabled:" + disabled);

        m_right_master.set(TalonFXControlMode.PercentOutput, feedBackPower);

        // System.out.println("error: " + Math.abs(get_shooter_rpm() - desiredShooterRPM));
        if (Math.abs(get_shooter_rpm() - desiredShooterRPM) < shooterSpeedToleranceRPM && onWaitAndShootTask) {
            System.out.println("running kicker...");
            onWaitAndShootTask = false;
            super.runKicker();
        }
    }

    /**
     * set the targeted speed of the fly wheel, called once during each operation, and the updating process will be proceeded in each following period
     * repeated operations(the new task is the same as old one) will be ignored
     * @param desiredShooterRPM the desired speed of the fly wheel, in rpm
     */
    public void setDesiredFlyWheelRPM(double desiredShooterRPM) {
        if (desiredShooterRPM == this.desiredShooterRPM)
            return;
        this.desiredShooterRPM = desiredShooterRPM;
        this.pidController.startNewTaskKeepIntegration(new EnhancedPIDController.Task(
                EnhancedPIDController.Task.SET_TO_SPEED, desiredShooterRPM
        ));

        // System.out.println("started new task...");
    }

    public void setDesiredFlyWheelRPM() {
        setDesiredFlyWheelRPM(super.defaultSpeed);
    }

    /**
     * sets the shooter speed into the speed matching the target distance, according to the check-table
     * (does not include shooting)
     * @param distance the distance to the target, units should match the check-table
     * */
    @Override
    public void shoot_from_anywhere(double distance){
        double rpm = get_target_rpm(distance);
        setDesiredFlyWheelRPM(rpm);
    }

    @Override
    public void stop() {
        setDesiredFlyWheelRPM(0);
        super.stop();
    }

    /**
     * add a task to wait for the shooter to accelerate to its current desired speed(which is not set in this scope) and initiate kicker
     * the task will be proceeded in the following periods
     *  */
    public void setWaitAndShootTask() {
        System.out.println("set task");
        onWaitAndShootTask = true;
    }


    /** disable the shooter(should be called whenever robot is set to disabled */
    public void onDisabled() {
        this.disabled = true;
        this.wasEnabled = false;
        setDesiredFlyWheelRPM(0);
        stop();
    }

    /** enable the shooter */
    public void onEnabled() { // fix logic here: fly wheel started once after enabling
        this.disabled = false;
        if (!wasEnabled) {
            setDesiredFlyWheelRPM(this.desiredShooterRPM);
            wasEnabled = true;
        }
    }

    /**
     * whether there is a shooting task in progress
     * @return (true or false)
     * */
    public boolean isOnWaitAndShootTask() {
        return onWaitAndShootTask;
    }
}
