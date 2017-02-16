package org.team686.lib.util;

import java.util.Optional;
import java.util.Set;

import org.mini2Dx.gdx.math.Vector2;
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
public class AdaptivePurePursuitController 
{
	private static final double kEpsilon = 1E-9;

	double mFixedLookahead;
	Path mPath;
	Pose.Delta mLastCommand;
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
	static Pose.Delta cmd;

	public AdaptivePurePursuitController(double fixed_lookahead, double max_accel, double nominal_dt, Path path,
			boolean reversed, double path_completion_tolerance) 
	{
		mFixedLookahead = fixed_lookahead;
		mMaxAccel = max_accel;
		mPath = path;
		mDt = nominal_dt;
		mLastCommand = null;
		mReversed = reversed;
		mPathCompletionTolerance = path_completion_tolerance;
	}

	public boolean isDone() 
	{
		remainingLength = mPath.getRemainingLength();
		return remainingLength <= mPathCompletionTolerance;
	}

	public Pose.Delta update(Pose robot_pose, double now)
	{
		Pose pose = new Pose(robot_pose);
		if (mReversed)
		{
			pose.turnRad(Math.PI);
		}

		double distanceFromPath = mPath.update(robot_pose.getPosition());
		if (this.isDone()) 
		{
			return new Pose.Delta(0, 0);
		}

		lookaheadPoint = mPath.getLookaheadPoint(robot_pose.getPosition(), distanceFromPath + mFixedLookahead);
		circle = joinPath(pose, lookaheadPoint.position);

		double speed = lookaheadPoint.speed;
		if (mReversed) 
			speed = -speed;
		
		// Ensure we don't accelerate too fast from the previous command
		double dt = now - mLastTime;
		if (mLastCommand == null) 
		{
			mLastCommand = new Pose.Delta(0, 0);
			dt = mDt;
		}
		double accel = (speed - mLastCommand.dDistance) / dt;
		if (accel < -mMaxAccel) 
		{
			speed = mLastCommand.dDistance - mMaxAccel * dt;
		} 
		else if (accel > mMaxAccel) 
		{
			speed = mLastCommand.dDistance + mMaxAccel * dt;
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

		if (circle.isPresent()) 
		{
			cmd = new Pose.Delta(speed, (circle.get().turn_right ? -1 : 1) * Math.abs(speed) / circle.get().radius);
		} else {
			cmd = new Pose.Delta(speed, 0);
		}
		mLastTime = now;
		mLastCommand = cmd;

		// DEBUG
		System.out.printf("Loc=(%.1f,%.1f), ", robot_pose.getX(), robot_pose.getY());
		System.out.printf("DistFromPath=%.1f, ", distanceFromPath);
		System.out.printf("Lookahead=(%.1f,%.1f), ", lookaheadPoint.position.x,
				lookaheadPoint.position.y);
		System.out.printf("Speed=%.2f, ", speed);
		System.out.printf("Cmd=(%.2f,%.2f)\n", cmd.dDistance, cmd.dHeading);

		return cmd;
	}

	public Set<String> getMarkersCrossed()
	{
		return mPath.getMarkersCrossed();
	}

	public static class Circle
	{
		public final Vector center;
		public final double radius;
		public final boolean turn_right;

		public Circle(Vector center, double radius, boolean turn_right) 
		{
			this.center = center;
			this.radius = radius;
			this.turn_right = turn_right;
		}
	}

	public static Optional<Circle> joinPath(Pose robot_pose, Vector lookaheadPoint)
	{
		double x1 = robot_pose.getX();
		double y1 = robot_pose.getY();
		double x2 = lookaheadPoint.x;
		double y2 = lookaheadPoint.y;

		Vector pose_to_lookahead = lookaheadPoint.sub(robot_pose.getPosition());
		double cross_product = pose_to_lookahead.cross(robot_pose.getHeadingUnitVector());
		if (Math.abs(cross_product) < kEpsilon) 
		{
			return Optional.empty();
		}

		double dx = x1 - x2;
		double dy = y1 - y2;
		double mx = Math.signum(cross_product) * Math.sin(robot_pose.getHeadingRad());
		double my = -Math.signum(cross_product) * Math.cos(robot_pose.getHeadingRad());

		double cross_term = mx * dx + my * dy;

		if (Math.abs(cross_term) < kEpsilon) {
			// Points are colinear
			return Optional.empty();
		}

		return Optional.of(new Circle(
				new Vector(( mx * ( x1 * x1 - x2 * x2 - dy * dy) + 2 * my * x1 * dy) / (2 * cross_term),
						   (-my * (-y1 * y1 + y2 * y2 + dx * dx) + 2 * mx * y1 * dx) / (2 * cross_term)),
				(0.5 * Math.abs((dx * dx + dy * dy) / cross_term)), (cross_product > 0)));
	}


	
	
	
	private final DataLogger logger = new DataLogger()
    {
        @Override
        public void log()
        {
    		put("DistFromPath", distanceFromPath);
    		put("xLookahead", lookaheadPoint.position.x);
    		put("yLookahead", lookaheadPoint.position.y);
    		put("RemainingLength", remainingLength);
    		put("Speed", speed);
    		// if (circle.isPresent())
    		// put("PathRadius", circle.get().radius);
    		// else
    		// put("PathRadius", 999);
    		put("Cmd.X", cmd.dDistance);
    		put("Cmd.Theta", cmd.dHeading);
        }
    };
    
    public DataLogger getLogger() { return logger; }
	
}
