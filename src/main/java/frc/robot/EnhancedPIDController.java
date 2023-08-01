package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;

/**
 * An enhanced version of the math utility: edu.wpi.first.math.controller.ProfiledPIDController
 *
 * the major features are:
 * - multiple controlling modes (go to position, hold still, maintain speed)
 * - adjusts the coefficients according to the feedbacks from the motors (senses oscillation, undershooting, windup and so on
 * - though the fundamental of the PID algorithm stays the same, we set the variables in a different, so it's easier for programmers to understand
 * - a special method of holding still the mechanism around a position, more specifically it calculates how much a force is needed to maintain at the position and use it for better control
 *
 * @author Sam
 * @version 0.0
 *
 * TODO dynamic adjusting of coefficients
 *
 */
public class EnhancedPIDController {
    /** the settings of the PID */
    private final PIDProfile pidProfile;
    /** the current mission of the controller */
    private Task task;
    /** the current schedule for the movement */
    private TrapezoidPathSchedule pathSchedule;
    /** the current shedule for the set to speed process */
    private SpeedChangingProcess speedChangingProcess;

    /** to calculate the time stamp between two calls, reset in the end of every call of getMotorPower() */
    private final Timer dt;
    /** the position at the last call */
    private double previousPosition;
    /** the velocity during the last call */
    private double previousVelocity;
    /** the acceleration (or motor power) in the at the last call */
    private double previousMotorPower;
    /** the integrated value */
    private double integralValue;
    /** the estimated gravity force on the mechanism, assuming that the force generated by full motor (not max power limit) power is 1  */
    private double gravitationalForce;

    public EnhancedPIDController(PIDProfile config) {
        this(config, 0);
    }

    public EnhancedPIDController(PIDProfile config, double initialPosition) {
        this.pidProfile = config;
        dt = new Timer();

        reset(initialPosition);
    }

    /** set all variables to initial state */
    public void reset(double initialValue) {
        dt.start();
        dt.reset();
        task.resetTaskTime();
        previousPosition = initialValue;
        previousVelocity = 0;
        previousMotorPower = 0;
        integralValue = 0;
        pathSchedule = null;
        speedChangingProcess = null;
    }

    public void startNewTask(Task newTask, double initialPosition) {
        this.task = newTask;
        reset(initialPosition);
    }

    public void startNewTask(Task newTask) {
        startNewTask(newTask, previousPosition);
    }

    public double getMotorPower(double currentPosition) {
        double velocity = (currentPosition - previousPosition) / dt.get();
        return getMotorPower(currentPosition, velocity);
    }

    /**
     * get the required motor power in order to complete the task
     * note that if the mechanism is a closed shape, like steers, you must give the actual velocity, otherwise it will fail whenever going through the origin
     * @param currentPosition the current position of the mechanism
     * @param velocity the instance velocity of the mechanism at the moment
     * @return the amount of motor power, in percent output
     */
    public double getMotorPower(double currentPosition, double velocity) {
        double motorPower;
        switch (task.taskType) {
            case Task.MAINTAIN_POSITION: {
                if (!pidProfile.dynamicallyAdjusting)
                    throw new IllegalStateException("Cannot use this control mode since the PID profile offered is static");
                motorPower = getMotorPowerMaintainPosition(currentPosition, velocity);
                break;
            }
            case Task.GO_TO_POSITION: {
                if (pidProfile.dynamicallyAdjusting)
                    return getMotorPowerGoToPositionDynamic(currentPosition, velocity);
                motorPower = getMotorPowerGoToPositionClassic(currentPosition, velocity);
                break;
            }
            case Task.MAINTAIN_SPEED: {
                // TODO write this part
            }
            case Task.SET_TO_SPEED: {

            }
            default: {
                throw new IllegalArgumentException("Unknown task type given to the PID controller");
            }
        }
        dt.reset();
        return motorPower;
    }

    /**
     * gets the amount of motor power needed, if the task is to maintain in a certain position
     * in order to hold the mechanism still it automatically estimates the gravitational force
     * @param currentPosition the current position of the mechanism
     * @param velocity the instance velocity of the mechanism at the moment
     * @return the amount of motor power, in percent output
     */
    private double getMotorPowerMaintainPosition(double currentPosition, double velocity) { // TODO add explanations
        DynamicalPIDProfile dynamicalPIDProfile = (DynamicalPIDProfile) pidProfile;

        double acceleration = (velocity - previousVelocity) / dt.get();
        double workloadMass = (pidProfile.getMaxPowerAllowed() - pidProfile.getMinPowerToMove()) / dynamicalPIDProfile.maxAcceleration; // m = deltaF / deltaA
        double estimatedGravity = acceleration * workloadMass - previousMotorPower; // G = AM - F
        gravitationalForce = (estimatedGravity - gravitationalForce) * dt.get() / pidProfile.getFeedForwardTime(); // tune it little by little to the estimated value at each call, after a time of feed forward, the newer value will complete replace the older

        double predictedFuturePosition = currentPosition + velocity * pidProfile.getFeedForwardTime();
        double error = task.value - predictedFuturePosition;
        double motorPower = error * pidProfile.getProportion() - gravitationalForce;

        previousVelocity = velocity;
        previousMotorPower = motorPower;

        return motorPower;
    }


    public double getMotorPowerGoToPositionDynamic(double currentPosition, double velocity) {
        if (pathSchedule == null)
            pathSchedule = new TrapezoidPathSchedule((DynamicalPIDProfile) this.pidProfile, this.task, currentPosition);

        return getMotorPowerGoToPositionClassic(
                currentPosition,
                velocity,
                new Task(Task.GO_TO_POSITION, pathSchedule.getCurrentPathPosition())
        );
    }

    /**
     * classic PID control algorithm, use the current task of the controller as default
     * @param currentPosition
     * @param velocity
     * @return
     */
    private double getMotorPowerGoToPositionClassic(double currentPosition, double velocity) {
        return getMotorPowerGoToPositionClassic(currentPosition, velocity, this.task);
    }

    /**
     * classic PID control algorithm
     * @param currentPosition
     * @param velocity
     * @param task the task to execute
     * @return
     */
    private double getMotorPowerGoToPositionClassic(double currentPosition, double velocity, Task task) {
        double predictedFuturePosition = currentPosition + velocity * pidProfile.getFeedForwardTime();
        double error = task.value - predictedFuturePosition;
        this.integralValue += (task.value - currentPosition) * dt.get();

        if (Math.abs(error) < pidProfile.getErrorTolerance())
            return 0;

        double correctionPower = error * pidProfile.getProportion() + integralValue * pidProfile.getErrorIntegralCoefficient();

        double correctionPowerMagnitude = Math.abs(correctionPower);
        if (correctionPowerMagnitude < pidProfile.getMinPowerToMove())
            correctionPowerMagnitude = pidProfile.getMinPowerToMove();
        if (correctionPowerMagnitude > pidProfile.getMaxPowerAllowed())
            correctionPowerMagnitude = pidProfile.getMaxPowerAllowed();
        correctionPower = Math.copySign(correctionPowerMagnitude, correctionPower);

        return correctionPower;
    }

    /**
     * gets the power required in order to go to the desired velocity
     * automatically schedules the acceleration/deceleration process
     * @param currentVelocity the current velocity of the mechanism, in whatever unit / second
     * @return motor power, in percentage output
     */
    public double getMotorPowerSetToVelocityDynamic(double currentVelocity) {
        if (speedChangingProcess == null)
            speedChangingProcess = new SpeedChangingProcess(this.task, (DynamicalPIDProfile) pidProfile, currentVelocity);
        return getMotorPowerSetToVelocityClassic(
                currentVelocity,
                new Task(Task.SET_TO_SPEED, speedChangingProcess.sampleCurrentVelocity())
        );
    }

    /**
     * get the power required in order to go to the desired velocity of the current task
     * uses a traditional feed-forward system
     *
     * @param currentVelocity the current velocity of the mechanism, in whatever unit / second
     * @return motor power, in percentage output
     */
    private double getMotorPowerSetToVelocityClassic(double currentVelocity) {
        return getMotorPowerSetToVelocityClassic(currentVelocity, this.task);
    }

    /**
     * get the power required in order to go to the desired velocity of a given task
     * uses a traditional feed-forward system
     *
     * @param currentVelocity the current velocity of the mechanism, in whatever unit / second
     * @param task the given task to achive, overrides the current task of the controller
     * @return motor power, in percentage output
     */
    private double getMotorPowerSetToVelocityClassic(double currentVelocity, Task task) {
        double targetedVelocity = task.value;
        if (pidProfile.dynamicallyAdjusting) {
            double velocityRestriction = ((DynamicalPIDProfile)pidProfile).maxVelocity;
            targetedVelocity = MathUtil.clamp(targetedVelocity, -velocityRestriction, velocityRestriction);
        }

        double velocityDifference = task.value - currentVelocity;

        double feedBackPower = velocityDifference * pidProfile.getProportion();
        double feedBackPowerMagnitude = Math.abs(feedBackPower);
        if (feedBackPowerMagnitude > pidProfile.getMaxPowerAllowed())
            feedBackPowerMagnitude = pidProfile.getMaxPowerAllowed();
        else if (feedBackPowerMagnitude < pidProfile.getMinPowerToMove())
            feedBackPowerMagnitude = pidProfile.getMinPowerToMove();
        feedBackPower = Math.copySign(feedBackPowerMagnitude, feedBackPower);

        /* integral */
        feedBackPower += integralValue;
        integralValue += velocityDifference * dt.get();

        return feedBackPower;
    }


    // <-- subclasses -->
    /**
     * stores the settings for the PID algorithm
     */
    public static abstract class PIDProfile {
        /** whether this profile will update according to feedbacks of the robot */
        private final boolean dynamicallyAdjusting;

        /** the restriction on power */
        private double maxPowerAllowed;
        /** the amount of motor power required to make the mechanism moving */
        private double minPowerToMove;
        /** the distance to target where the mechanism should start decelerate, the mechanism will otherwise move with full power */
        private double errorStartDecelerate;
        /** the amount of error to ignore */
        private double errorTolerance;
        /** the amount of time to think forward for the mechanism, the feedback power will be calculated according to the predicted position of the mechanism after an amount of time instead of its current position */
        private double feedForwardTime;
        /** the coefficient of the cumulated error, also known as kD */
        private double errorIntegralCoefficient;
        /** the coefficient of cumulated velocity difference */
        private double velocityDifferenceIntegralCoefficient;

        /**
         * Creates a new profile with the all settings given and fixed all the time
         * @param dynamicallyAdjusting whether this profile will update according to feedbacks of the robot
         * @param maxPowerAllowed the restriction on power
         * @param minPowerToMove the amount of motor power required to make the mechanism moving
         * @param errorStartDecelerate the distance to target where the mechanism should start decelerate, the mechanism will otherwise move with full power
         * @param errorTolerance the amount of error to ignore
         * @param feedForwardTime the amount of time to think forward for the mechanism, the feedback power will be calculated according to the predicted position of the mechanism after an amount of time instead of its current position
         * @param errorIntegralCoefficient the coefficient of the cumulated error, also known as kI
         * @param velocityDifferenceIntegralCoefficient
         */
        protected PIDProfile(boolean dynamicallyAdjusting, double maxPowerAllowed, double minPowerToMove, double errorStartDecelerate, double errorTolerance, double feedForwardTime, double errorIntegralCoefficient, double velocityDifferenceIntegralCoefficient) {
            this.dynamicallyAdjusting = dynamicallyAdjusting;

            this.maxPowerAllowed = maxPowerAllowed;
            this.minPowerToMove = minPowerToMove;
            this.errorStartDecelerate = errorStartDecelerate;
            this.errorTolerance = errorTolerance;
            this.feedForwardTime = feedForwardTime;
            this.errorIntegralCoefficient = errorIntegralCoefficient;
            this.velocityDifferenceIntegralCoefficient = velocityDifferenceIntegralCoefficient;
        }

        // Getters for all the private variables
        public double getMaxPowerAllowed() {
            return maxPowerAllowed;
        }

        public double getMinPowerToMove() {
            return minPowerToMove;
        }

        public double getErrorStartDecelerate() {
            return errorStartDecelerate;
        }

        public double getErrorTolerance() {
            return errorTolerance;
        }

        public double getFeedForwardTime() {
            return feedForwardTime;
        }

        public double getErrorIntegralCoefficient() {
            return errorIntegralCoefficient;
        }

        public double getVelocityDifferenceIntegralCoefficient() { return velocityDifferenceIntegralCoefficient; }

        public double getProportion() {
            return (maxPowerAllowed - minPowerToMove) /
                    (errorStartDecelerate - errorTolerance);
        }
    }

    /**
     *  PID coefficients that adjusts to the feedbacks from the machinist, senses oscillation and
     *
     */
    public static class DynamicalPIDProfile extends PIDProfile {
        /** the maximum instant acceleration that the mechanism can achieve with the max power */
        public final double maxAcceleration;
        /** the restriction on the velocity of the mechanism */
        public final double maxVelocity;

        /**
         * Creates a dynamically adjusting PID profile with the following params, and estimates other params automatically
         * @param maxPowerAllowed the restriction on power
         * @param minPowerToMove the amount of motor power required to make the mechanism moving
         * @param errorTolerance the amount of error to ignore
         * @param integralCoefficientError the coefficient of the cumulated error, also known as kI
         * @param integralCoefficientVelocityDifference
         * @param maxAcceleration the maximum instant acceleration that the mechanism can achieve with the max power
         * @param maxVelocity the restriction on the velocity of the mechanism
         */
        public DynamicalPIDProfile(double maxPowerAllowed, double minPowerToMove, double errorTolerance, double integralCoefficientError, double integralCoefficientVelocityDifference, double maxAcceleration, double maxVelocity) {
            super(
                    true,
                    maxPowerAllowed,
                    minPowerToMove,
                    calculateErrorStartDecelerate(errorTolerance, maxAcceleration, maxVelocity),
                    errorTolerance,
                    maxVelocity / maxAcceleration,
                    integralCoefficientError,
                    integralCoefficientVelocityDifference
            );
            this.maxAcceleration = maxAcceleration;
            this.maxVelocity = maxVelocity;
        }

        /**
         * calculate in what distance should the mechanism start decelerate, according the max acceleration and maxVelocity
         */
        private static double calculateErrorStartDecelerate(double errorTolerance, double maxAcceleration, double maxVelocity) {
            /** the amount of time for the arm to fully stop */
            double decelerateTime = maxVelocity / maxAcceleration;

            /** the amount of distance travelled during this process */
            double distanceDecelerate = decelerateTime * maxVelocity / 2;

            return  Math.max(distanceDecelerate - errorTolerance, 0);
        }

        public void setMaxPowerAllowed(double maxPowerAllowed) {
            super.maxPowerAllowed = maxPowerAllowed;
        }

        public void setMinPowerToMove(double minPowerToMove) {
            super.minPowerToMove = minPowerToMove;
        }

        public void setErrorStartDecelerate(double errorStartDecelerate) {
            super.errorStartDecelerate = errorStartDecelerate;
        }

        public void setErrorTolerance(double errorTolerance) {
            super.errorTolerance = errorTolerance;
        }

        public void setFeedForwardTime(double feedForwardTime) {
            super.feedForwardTime = feedForwardTime;
        }

        public void setIntegralCoefficient(double integralCoefficient) {
            super.errorIntegralCoefficient = integralCoefficient;
        }
    }

    /**
     * stores the task that this controller will have to complete
     */
    public static class Task {
        public final int taskType;
        public final double value;
        /** whether the expected outcome is achieved */
        public boolean completed;
        /** the time elapsed since the task is created */
        private Timer taskTime = new Timer();

        public static final int MAINTAIN_POSITION = 0;
        public static final int GO_TO_POSITION = 1;
        public static final int MAINTAIN_SPEED = 2;
        public static final int SET_TO_SPEED = 3;
        public static final int GO_TO_AND_MAINTAIN_POSITION = 4;

        public Task(int taskType, double value) {
            this.taskType = taskType;
            this.value = value;
            taskTime.start();
        }

        /** reset the time of initiation */
        public void resetTaskTime() {
            this.taskTime.reset();
        }

        /**
         * gets the time elapsed since the task is initiated
         * @return the time, in seconds
         */
        private double getTaskTime() {
            return this.taskTime.get();
        }
    }

    static class SpeedChangingProcess {
        private final DynamicalPIDProfile profile;
        private final Task task;
        private final double startingVelocity;

        public SpeedChangingProcess(Task task, DynamicalPIDProfile profile, double startingVelocity) {
            if (task.taskType != Task.SET_TO_SPEED)
                throw new IllegalStateException("the current task type:" + task.taskType + "does not support speed changing schedule");
            this.startingVelocity = startingVelocity;
            this.task = task;
            this.profile = profile;
        }

        public double sampleCurrentVelocity() {
            if (task.value - startingVelocity > 0)
                return Math.min(
                        startingVelocity + task.getTaskTime() * profile.maxAcceleration,
                        task.value
                );
            return Math.max(
                    startingVelocity - task.getTaskTime() * profile.maxAcceleration,
                    task.value
            );
        }

    }

    /**
     * stores the plan of going to a position using trapezoid algorithm
     * */
    static class TrapezoidPathSchedule {
        private DynamicalPIDProfile profile;
        private Task task;
        private double startingPosition;
        private double currentPosition;
        private double previousTime;
        private CheckPoint[] checkPoints;
        private double expectedTimeOfArrival;

        public TrapezoidPathSchedule(DynamicalPIDProfile profile, Task task, double startingPosition) {
            if (task.taskType != Task.GO_TO_POSITION)
                throw new IllegalStateException("the current task type:" + task.taskType + "does not support trapezoid path schedule");

            this.profile = profile;
            this.startingPosition = startingPosition;
            this.task = task;

            scheduleCheckPoints();
        }

        private void scheduleCheckPoints() {
            double timeToFullyAccelerate = profile.maxVelocity / profile.maxAcceleration;
            double distanceTravelledDuringAccelerating = timeToFullyAccelerate * profile.maxVelocity / 2;
            double totalDistance = Math.abs(task.value - startingPosition);

            /** whether the trapezoid is a triangle */
            boolean isTriangle = distanceTravelledDuringAccelerating * 2 > totalDistance; // it is a triangle whenever there is no enough distance to fully accelerate and decelerate
            /* if it is triangle */
            if (isTriangle) {
                this.checkPoints = new CheckPoint[3];

                this.checkPoints[0] = new CheckPoint(0, 0);

                double timeToAccelerate = Math.sqrt(totalDistance / 2 / this.profile.maxAcceleration); // 2AT^2 = D
                double fullSpeed = timeToAccelerate * profile.maxAcceleration;
                this.checkPoints[1] = new CheckPoint(timeToAccelerate, fullSpeed);

                this.checkPoints[2] = new CheckPoint(timeToAccelerate * 2, 0);

                expectedTimeOfArrival = checkPoints[2].time;

                return;
            }

            /* if it is a classic trapezoid */
            this.checkPoints = new CheckPoint[4];

            this.checkPoints[0] = new CheckPoint(0, 0);

            this.checkPoints[1] = new CheckPoint(checkPoints[0].time + timeToFullyAccelerate, profile.maxVelocity);

            double timeFullSpeed = (totalDistance - distanceTravelledDuringAccelerating * 2) / profile.maxVelocity;
            this.checkPoints[2] = new CheckPoint(checkPoints[1].time + timeFullSpeed, profile.maxVelocity);

            this.checkPoints[3] = new CheckPoint(checkPoints[2].time + timeToFullyAccelerate, 0);

            expectedTimeOfArrival = checkPoints[3].time;

            return;
        }

        public double getCurrentPathPosition() {
            double dt = task.getTaskTime() - this.previousTime;
            currentPosition += dt * getCurrentVelocity();

            this.previousTime = task.getTaskTime();

            return currentPosition;
        }

        private double getCurrentVelocity() {
            /* if the task isn't started yet */
            if (task.getTaskTime() < 0)
                return 0;
            for (int currentCheckPoint = 1; currentCheckPoint <= checkPoints.length; currentCheckPoint++) {
                if (task.getTaskTime() < checkPoints[currentCheckPoint].time) {
                    CheckPoint previousCheckPoint = checkPoints[currentCheckPoint-1];
                    CheckPoint nextCheckPoint = checkPoints[currentCheckPoint];

                    double currentVelocity = previousCheckPoint.velocity + (task.getTaskTime() - previousCheckPoint.time) / (nextCheckPoint.time - previousCheckPoint.time);

                    return currentVelocity;
                }
            }
            /* if the task is already over */
            return 0;
        }

        /** store a checkpoint inside the path, it records the  */
        class CheckPoint {
            public double time;
            public double velocity;
            public CheckPoint(double time, double velocityAtTheTime) {
                this.time = time;
                this.velocity = velocityAtTheTime;
            }
        }
    }
}