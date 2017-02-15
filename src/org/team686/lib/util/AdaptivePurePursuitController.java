package org.team686.lib.util;

import java.util.Optional;
import java.util.Set;

import org.team686.simsbot.DataLogger;

/**
 * Implements an adaptive pure pursuit controller. See:
 * https://www.ri.cmu.edu/pub_files/pub1/kelly_alonzo_1994_4/kelly_alonzo_1994_4.pdf
 * 
 * Basically, we find a spot on the path we'd like to follow and calculate the
 * wheel speeds necessary to make us land on that spot. The target spot is a
 * specified distance ahead of us, and we look further ahead the greater our
 * tracking error.
 */
public class AdaptivePurePursuitController {
	private static final double kEpsilon = 1E-9;

	double mFixedLookahead;
	Path mPath;
	RigidTransform2d.Delta mLastCommand;
	double mLastTime;
	double mMaxAccel;
	double mDt;
	boolean mReversed;
	double mPathCompletionTolerance;

	static double distanceFromPath;
	static PathSegment.Sample lookaheadPoint;
	static double remainingLength;
	static double speed;
	Optional<Circle> circle;
	static RigidTransform2d.Delta cmd;

	public AdaptivePurePursuitController(double fixed_lookahead, double max_accel, double nominal_dt, Path path,
			boolean reversed, double path_completion_tolerance) {
		mFixedLookahead = fixed_lookahead;
		mMaxAccel = max_accel;
		mPath = path;
		mDt = nominal_dt;
		mLastCommand = null;
		mReversed = reversed;
		mPathCompletionTolerance = path_completion_tolerance;
	}

	public boolean isDone() {
		remainingLength = mPath.getRemainingLength();
		return remainingLength <= mPathCompletionTolerance;
	}

	public RigidTransform2d.Delta update(RigidTransform2d robot_pose, double now) {
		RigidTransform2d pose = robot_pose;
		if (mReversed) {
			pose = new RigidTransform2d(robot_pose.getTranslation(),
					robot_pose.getRotation().rotateBy(Rotation2d.fromRadians(Math.PI)));
		}

		double distanceFromPath = mPath.update(robot_pose.getTranslation());
		if (this.isDone()) {
			return new RigidTransform2d.Delta(0, 0, 0);
		}

		lookaheadPoint = mPath.getLookaheadPoint(robot_pose.getTranslation(), distanceFromPath + mFixedLookahead);
		circle = joinPath(pose, lookaheadPoint.translation);

		double speed = lookaheadPoint.speed;
		if (mReversed) {
			speed *= -1;
		}
		// Ensure we don't accelerate too fast from the previous command
		double dt = now - mLastTime;
		if (mLastCommand == null) {
			mLastCommand = new RigidTransform2d.Delta(0, 0, 0);
			dt = mDt;
		}
		double accel = (speed - mLastCommand.dx) / dt;
		if (accel < -mMaxAccel) {
			speed = mLastCommand.dx - mMaxAccel * dt;
		} else if (accel > mMaxAccel) {
			speed = mLastCommand.dx + mMaxAccel * dt;
		}

		// Ensure we slow down in time to stop
		// vf^2 = v^2 + 2*a*d
		// 0 = v^2 + 2*a*d
		double remaining_distance = mPath.getRemainingLength();
		double max_allowed_speed = Math.sqrt(2 * mMaxAccel * remaining_distance);
		if (Math.abs(speed) > max_allowed_speed) {
			speed = max_allowed_speed * Math.signum(speed);
		}
		final double kMinSpeed = 4.0;
		if (Math.abs(speed) < kMinSpeed) {
			// Hack for dealing with problems tracking very low speeds with
			// Talons
			speed = kMinSpeed * Math.signum(speed);
		}

		if (circle.isPresent()) {
			cmd = new RigidTransform2d.Delta(speed, 0,
					(circle.get().turn_right ? -1 : 1) * Math.abs(speed) / circle.get().radius);
		} else {
			cmd = new RigidTransform2d.Delta(speed, 0, 0);
		}
		mLastTime = now;
		mLastCommand = cmd;

		// DEBUG
		System.out.printf("Loc=(%.1f,%.1f), ", robot_pose.getTranslation().getX(), robot_pose.getTranslation().getY());
		System.out.printf("DistFromPath=%.1f, ", distanceFromPath);
		System.out.printf("Lookahead=(%.1f,%.1f), ", lookaheadPoint.translation.getX(),
				lookaheadPoint.translation.getY());
		System.out.printf("Speed=%.2f, ", speed);
		System.out.printf("Cmd=(%.2f,%.2f)\n", cmd.dx, cmd.dtheta);

		return cmd;
	}

	public Set<String> getMarkersCrossed() {
		return mPath.getMarkersCrossed();
	}

	public static class Circle {
		public final Translation2d center;
		public final double radius;
		public final boolean turn_right;

		public Circle(Translation2d center, double radius, boolean turn_right) {
			this.center = center;
			this.radius = radius;
			this.turn_right = turn_right;
		}
	}

	public static Optional<Circle> joinPath(RigidTransform2d robot_pose, Translation2d lookaheadPoint) {
		double x1 = robot_pose.getTranslation().getX();
		double y1 = robot_pose.getTranslation().getY();
		double x2 = lookaheadPoint.getX();
		double y2 = lookaheadPoint.getY();

		Translation2d pose_to_lookahead = robot_pose.getTranslation().inverse().translateBy(lookaheadPoint);
		double cross_product = pose_to_lookahead.getX() * robot_pose.getRotation().sin()
				- pose_to_lookahead.getY() * robot_pose.getRotation().cos();
		if (Math.abs(cross_product) < kEpsilon) {
			return Optional.empty();
		}

		double dx = x1 - x2;
		double dy = y1 - y2;
		double mx = Math.signum(cross_product) * robot_pose.getRotation().sin();
		double my = -Math.signum(cross_product) * robot_pose.getRotation().cos();

		double cross_term = mx * dx + my * dy;

		if (Math.abs(cross_term) < kEpsilon) {
			// Points are colinear
			return Optional.empty();
		}

		return Optional.of(new Circle(
				new Translation2d((mx * (x1 * x1 - x2 * x2 - dy * dy) + 2 * my * x1 * dy) / (2 * cross_term),
						(-my * (-y1 * y1 + y2 * y2 + dx * dx) + 2 * mx * y1 * dx) / (2 * cross_term)),
				(0.5 * Math.abs((dx * dx + dy * dy) / cross_term)), (cross_product > 0)));
	}


	
	
	
	private final DataLogger logger = new DataLogger()
    {
        @Override
        public void log()
        {
    		put("DistFromPath", distanceFromPath);
    		put("xLookahead", lookaheadPoint.translation.getX());
    		put("yLookahead", lookaheadPoint.translation.getY());
    		put("RemainingLength", remainingLength);
    		put("Speed", speed);
    		// if (circle.isPresent())
    		// put("PathRadius", circle.get().radius);
    		// else
    		// put("PathRadius", 999);
    		put("Cmd.X", cmd.dx);
    		put("Cmd.Theta", cmd.dtheta);
        }
    };
    
    public DataLogger getLogger() { return logger; }
	
}
