package team3647.lib.team254.swerve;

import team3647.lib.team254.geometry.Rotation2d;
import team3647.lib.team254.geometry.Translation2d;
import team3647.lib.team254.geometry.Twist2d;
import team3647.lib.team254.util.Util;

/**
 * Takes a prior setpoint (ChassisSpeeds), a desired setpoint (from a driver, or from a path
 * follower), and outputs a new setpoint that respects all of the kinematic constraints on module
 * rotation speed and wheel velocity/acceleration. By generating a new setpoint every iteration, the
 * robot will converge to the desired setpoint quickly while avoiding any intermediate state that is
 * kinematically infeasible (and can result in wheel slip or robot heading drift as a result).
 */
public class SwerveSetpointGenerator {
    private final SwerveDriveKinematics mKinematics;

    // Pre-allocated to avoid per-loop heap allocation
    private final SwerveModuleState[] m_desiredModuleState;
    private final double[] m_prevVx;
    private final double[] m_prevVy;
    private final Rotation2d[] m_prevHeading;
    private final double[] m_desiredVx;
    private final double[] m_desiredVy;
    private final Rotation2d[] m_desiredHeading;
    private final Rotation2d[] m_overrideSteering;
    private final boolean[] m_hasOverrideSteering;

    public SwerveSetpointGenerator(final SwerveDriveKinematics kinematics) {
        this.mKinematics = kinematics;
        int n = kinematics.getNumModules();
        m_desiredModuleState = new SwerveModuleState[n];
        m_prevVx = new double[n];
        m_prevVy = new double[n];
        m_prevHeading = new Rotation2d[n];
        m_desiredVx = new double[n];
        m_desiredVy = new double[n];
        m_desiredHeading = new Rotation2d[n];
        m_overrideSteering = new Rotation2d[n];
        m_hasOverrideSteering = new boolean[n];
        for (int i = 0; i < n; i++) {
            m_desiredModuleState[i] = new SwerveModuleState();
        }
    }

    /**
     * Check if it would be faster to go to the opposite of the goal heading (and reverse drive
     * direction).
     *
     * @param prevToGoal The rotation from the previous state to the goal state (i.e.
     *     prev.inverse().rotateBy(goal)).
     * @return True if the shortest path to achieve this rotation involves flipping the drive
     *     direction.
     */
    private boolean flipHeading(Rotation2d prevToGoal) {
        return Math.abs(prevToGoal.getRadians()) > Math.PI / 2.0;
    }

    private double unwrapAngle(double ref, double angle) {
        double diff = angle - ref;
        if (diff > Math.PI) {
            return angle - 2.0 * Math.PI;
        } else if (diff < -Math.PI) {
            return angle + 2.0 * Math.PI;
        } else {
            return angle;
        }
    }

    @FunctionalInterface
    private interface Function2d {
        public double f(double x, double y);
    }

    /**
     * Find the root of the generic 2D parametric function 'func' using the regula falsi technique.
     * This is a pretty naive way to do root finding, but it's usually faster than simple bisection
     * while being robust in ways that e.g. the Newton-Raphson method isn't.
     *
     * @param func The Function2d to take the root of.
     * @param x_0 x value of the lower bracket.
     * @param y_0 y value of the lower bracket.
     * @param f_0 value of 'func' at x_0, y_0 (passed in by caller to save a call to 'func' during
     *     recursion)
     * @param x_1 x value of the upper bracket.
     * @param y_1 y value of the upper bracket.
     * @param f_1 value of 'func' at x_1, y_1 (passed in by caller to save a call to 'func' during
     *     recursion)
     * @param iterations_left Number of iterations of root finding left.
     * @return The parameter value 's' that interpolating between 0 and 1 that corresponds to the
     *     (approximate) root.
     */
    private double findRoot(
            Function2d func,
            double x_0,
            double y_0,
            double f_0,
            double x_1,
            double y_1,
            double f_1,
            int iterations_left) {
        if (iterations_left < 0 || Util.epsilonEquals(f_0, f_1)) {
            return 1.0;
        }
        var s_guess = Math.max(0.0, Math.min(1.0, -f_0 / (f_1 - f_0)));
        var x_guess = (x_1 - x_0) * s_guess + x_0;
        var y_guess = (y_1 - y_0) * s_guess + y_0;
        var f_guess = func.f(x_guess, y_guess);
        if (Math.signum(f_0) == Math.signum(f_guess)) {
            // 0 and guess on same side of root, so use upper bracket.
            return s_guess
                    + (1.0 - s_guess)
                            * findRoot(
                                    func,
                                    x_guess,
                                    y_guess,
                                    f_guess,
                                    x_1,
                                    y_1,
                                    f_1,
                                    iterations_left - 1);
        } else {
            // Use lower bracket.
            return s_guess
                    * findRoot(func, x_0, y_0, f_0, x_guess, y_guess, f_guess, iterations_left - 1);
        }
    }

    protected double findSteeringMaxS(
            double x_0,
            double y_0,
            double f_0,
            double x_1,
            double y_1,
            double f_1,
            double max_deviation,
            int max_iterations) {
        f_1 = unwrapAngle(f_0, f_1);
        double diff = f_1 - f_0;
        if (Math.abs(diff) <= max_deviation) {
            // Can go all the way to s=1.
            return 1.0;
        }
        double offset = f_0 + Math.signum(diff) * max_deviation;
        Function2d func =
                (x, y) -> {
                    return unwrapAngle(f_0, Math.atan2(y, x)) - offset;
                };
        return findRoot(func, x_0, y_0, f_0 - offset, x_1, y_1, f_1 - offset, max_iterations);
    }

    protected double findDriveMaxS(
            double x_0,
            double y_0,
            double f_0,
            double x_1,
            double y_1,
            double f_1,
            double max_vel_step,
            int max_iterations) {
        double diff = f_1 - f_0;
        if (Math.abs(diff) <= max_vel_step) {
            // Can go all the way to s=1.
            return 1.0;
        }
        double offset = f_0 + Math.signum(diff) * max_vel_step;
        Function2d func =
                (x, y) -> {
                    return Math.hypot(x, y) - offset;
                };
        return findRoot(func, x_0, y_0, f_0 - offset, x_1, y_1, f_1 - offset, max_iterations);
    }

    /**
     * Generate a new setpoint.
     *
     * @param limits The kinematic limits to respect for this setpoint.
     * @param prevSetpoint The previous setpoint motion. Normally, you'd pass in the previous
     *     iteration setpoint instead of the actual measured/estimated kinematic state.
     * @param desiredState The desired state of motion, such as from the driver sticks or a path
     *     following algorithm.
     * @param dt The loop time.
     * @return A Setpoint object that satisfies all of the KinematicLimits while converging to
     *     desiredState quickly.
     */
    public SwerveSetpoint generateSetpoint(
            final SwerveKinematicLimits limits,
            final SwerveSetpoint prevSetpoint,
            ChassisSpeeds desiredState,
            double dt) {
        final Translation2d[] modules = mKinematics.getModuleLocations();
        final int n = modules.length;

        mKinematics.fillSwerveModuleStates(desiredState, m_desiredModuleState);
        if (limits.kMaxDriveVelocity > 0.0) {
            SwerveDriveKinematics.desaturateWheelSpeeds(m_desiredModuleState, limits.kMaxDriveVelocity);
            desiredState = mKinematics.toChassisSpeeds(m_desiredModuleState);
        }

        // Special case: desiredState is a complete stop. In this case, module angle is arbitrary,
        // so just use the previous angle.
        boolean need_to_steer = true;
        if (desiredState.toTwist2d().epsilonEquals(Twist2d.identity(), Util.kEpsilon)) {
            need_to_steer = false;
            for (int i = 0; i < n; ++i) {
                m_desiredModuleState[i].angle = prevSetpoint.mModuleStates[i].angle;
                m_desiredModuleState[i].speedMetersPerSecond = 0.0;
            }
        }

        // For each module, compute local Vx and Vy vectors.
        boolean all_modules_should_flip = true;
        for (int i = 0; i < n; ++i) {
            m_prevVx[i] =
                    prevSetpoint.mModuleStates[i].angle.cos()
                            * prevSetpoint.mModuleStates[i].speedMetersPerSecond;
            m_prevVy[i] =
                    prevSetpoint.mModuleStates[i].angle.sin()
                            * prevSetpoint.mModuleStates[i].speedMetersPerSecond;
            m_prevHeading[i] = prevSetpoint.mModuleStates[i].angle;
            if (prevSetpoint.mModuleStates[i].speedMetersPerSecond < 0.0) {
                m_prevHeading[i] = m_prevHeading[i].flip();
            }
            m_desiredVx[i] =
                    m_desiredModuleState[i].angle.cos()
                            * m_desiredModuleState[i].speedMetersPerSecond;
            m_desiredVy[i] =
                    m_desiredModuleState[i].angle.sin()
                            * m_desiredModuleState[i].speedMetersPerSecond;
            m_desiredHeading[i] = m_desiredModuleState[i].angle;
            if (m_desiredModuleState[i].speedMetersPerSecond < 0.0) {
                m_desiredHeading[i] = m_desiredHeading[i].flip();
            }
            if (all_modules_should_flip) {
                double required_rotation_rad =
                        Math.abs(
                                m_prevHeading[i]
                                        .inverse()
                                        .rotateBy(m_desiredHeading[i])
                                        .getRadians());
                if (required_rotation_rad < Math.PI / 2.0) {
                    all_modules_should_flip = false;
                }
            }
        }
        if (all_modules_should_flip
                && !prevSetpoint
                        .mChassisSpeeds
                        .toTwist2d()
                        .epsilonEquals(Twist2d.identity(), Util.kEpsilon)
                && !desiredState.toTwist2d().epsilonEquals(Twist2d.identity(), Util.kEpsilon)) {
            return generateSetpoint(limits, prevSetpoint, new ChassisSpeeds(), dt);
        }

        double dx = desiredState.vxMetersPerSecond - prevSetpoint.mChassisSpeeds.vxMetersPerSecond;
        double dy = desiredState.vyMetersPerSecond - prevSetpoint.mChassisSpeeds.vyMetersPerSecond;
        double dtheta =
                desiredState.omegaRadiansPerSecond
                        - prevSetpoint.mChassisSpeeds.omegaRadiansPerSecond;

        double min_s = 1.0;

        for (int i = 0; i < n; i++) {
            m_hasOverrideSteering[i] = false;
        }

        final double max_theta_step = dt * limits.kMaxSteeringVelocity;
        for (int i = 0; i < n; ++i) {
            if (!need_to_steer) {
                m_overrideSteering[i] = prevSetpoint.mModuleStates[i].angle;
                m_hasOverrideSteering[i] = true;
                continue;
            }
            m_hasOverrideSteering[i] = false;
            if (Util.epsilonEquals(prevSetpoint.mModuleStates[i].speedMetersPerSecond, 0.0)) {
                if (Util.epsilonEquals(m_desiredModuleState[i].speedMetersPerSecond, 0.0)) {
                    m_overrideSteering[i] = prevSetpoint.mModuleStates[i].angle;
                    m_hasOverrideSteering[i] = true;
                    continue;
                }
                var necessaryRotation =
                        prevSetpoint
                                .mModuleStates[i]
                                .angle
                                .inverse()
                                .rotateBy(m_desiredModuleState[i].angle);
                if (flipHeading(necessaryRotation)) {
                    necessaryRotation = necessaryRotation.rotateBy(Rotation2d.kPi);
                }
                final double numStepsNeeded =
                        Math.abs(necessaryRotation.getRadians()) / max_theta_step;
                if (numStepsNeeded <= 1.0) {
                    m_overrideSteering[i] = m_desiredModuleState[i].angle;
                    m_hasOverrideSteering[i] = true;
                    continue;
                } else {
                    m_overrideSteering[i] =
                            prevSetpoint.mModuleStates[i].angle.rotateBy(
                                    Rotation2d.fromRadians(
                                            Math.signum(necessaryRotation.getRadians())
                                                    * max_theta_step));
                    m_hasOverrideSteering[i] = true;
                    min_s = 0.0;
                    continue;
                }
            }
            if (min_s == 0.0) {
                // s can't get any lower. Save some CPU.
                continue;
            }

            final int kMaxIterations = 8;
            double s =
                    findSteeringMaxS(
                            m_prevVx[i],
                            m_prevVy[i],
                            m_prevHeading[i].getRadians(),
                            m_desiredVx[i],
                            m_desiredVy[i],
                            m_desiredHeading[i].getRadians(),
                            max_theta_step,
                            kMaxIterations);
            min_s = Math.min(min_s, s);
        }

        // Enforce drive wheel acceleration limits.
        final double max_vel_step = dt * limits.kMaxDriveAcceleration;
        for (int i = 0; i < n; ++i) {
            if (min_s == 0.0) {
                // No need to carry on.
                break;
            }
            double vx_min_s =
                    min_s == 1.0
                            ? m_desiredVx[i]
                            : (m_desiredVx[i] - m_prevVx[i]) * min_s + m_prevVx[i];
            double vy_min_s =
                    min_s == 1.0
                            ? m_desiredVy[i]
                            : (m_desiredVy[i] - m_prevVy[i]) * min_s + m_prevVy[i];
            final int kMaxIterations = 10;
            double s =
                    min_s
                            * findDriveMaxS(
                                    m_prevVx[i],
                                    m_prevVy[i],
                                    Math.hypot(m_prevVx[i], m_prevVy[i]),
                                    vx_min_s,
                                    vy_min_s,
                                    Math.hypot(vx_min_s, vy_min_s),
                                    max_vel_step,
                                    kMaxIterations);
            min_s = Math.min(min_s, s);
        }

        ChassisSpeeds retSpeeds =
                new ChassisSpeeds(
                        prevSetpoint.mChassisSpeeds.vxMetersPerSecond + min_s * dx,
                        prevSetpoint.mChassisSpeeds.vyMetersPerSecond + min_s * dy,
                        prevSetpoint.mChassisSpeeds.omegaRadiansPerSecond + min_s * dtheta);
        var retStates = mKinematics.toSwerveModuleStates(retSpeeds);
        for (int i = 0; i < n; ++i) {
            if (m_hasOverrideSteering[i]) {
                var override = m_overrideSteering[i];
                if (flipHeading(retStates[i].angle.inverse().rotateBy(override))) {
                    retStates[i].speedMetersPerSecond *= -1.0;
                }
                retStates[i].angle = override;
            }
            final var deltaRotation =
                    prevSetpoint.mModuleStates[i].angle.inverse().rotateBy(retStates[i].angle);
            if (flipHeading(deltaRotation)) {
                retStates[i].angle = retStates[i].angle.flip();
                retStates[i].speedMetersPerSecond *= -1.0;
            }
        }
        return new SwerveSetpoint(retSpeeds, retStates);
    }
}
