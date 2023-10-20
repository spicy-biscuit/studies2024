package frc.robot.org.team100.plotter;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

/**
 * A collection of plot operations made from SVG path operators.
 */
public class SVGToPlotOperations {
    private static final double ABS_MAX = 1000;

    private final double xScale;
    private final double yScale;
    private final List<Operation> operations;

    private double currentX = 0;
    private double currentY = 0;
    private Double initialX = null;
    private Double initialY = null;

    /**
     * Note that WPILib spline generation makes a scale assumption, and fails if the
     * step size is larger than some absolute scale, don't give it
     * anything more than like 1000?
     */
    public SVGToPlotOperations(double xScale, double yScale) {
        this.xScale = xScale;
        this.yScale = yScale;
        operations = new ArrayList<>();
    }

    public void move(double rawX, double rawY) {
        double x = xScale * rawX;
        double y = yScale * rawY;
        if (Math.abs(x) > ABS_MAX || Math.abs(y) > ABS_MAX)
            throw new IllegalArgumentException(String.format("args too big %f %f", rawX, rawY));
        moveScaled(x, y);
    }

    private void moveScaled(double x, double y) {
        lineScaled(x, y, false);
        this.initialX = x;
        this.initialY = y;
    }

    public void line(double rawX, double rawY) {
        double x = xScale * rawX;
        double y = yScale * rawY;
        if (Math.abs(x) > ABS_MAX || Math.abs(y) > ABS_MAX)
            throw new IllegalArgumentException(String.format("args too big %f %f", rawX, rawY));

        lineScaled(x, y, true);
    }

    /** Make a trajectory with just a start and end. */
    private Trajectory shortTrajectory(Pose2d startPose, Pose2d endPose) {
        // TODO: use nonzero velocities
        Trajectory.State start = new Trajectory.State(0, 0, 0, startPose, 0);
        Trajectory.State end = new Trajectory.State(0.1, 0, 0, endPose, 0);
        return new Trajectory(List.of(start, end));
    }

    private void lineScaled(double x, double y, boolean penDown) {
        Rotation2d rot = new Rotation2d(x - this.currentX, y - this.currentY);
        Pose2d start = new Pose2d(this.currentX, this.currentY, rot);
        Pose2d end = new Pose2d(x, y, rot);

        Trajectory trajectory = null;

        if (end.minus(start).getTranslation().getNorm() < 0.1) {
            // too short, trajectory generator barfs
            System.out.println("using short trajectory");
            trajectory = shortTrajectory(start, end);
        } else {
            trajectory = TrajectoryGenerator.generateTrajectory(
                    start, List.of(), end, new TrajectoryConfig(10, 0.1));
        }
        operations.add(new Operation(penDown, trajectory));

        this.currentX = x;
        this.currentY = y;
    }

    public void curve(double rawX, double rawY, double rawX1, double rawY1, double rawX2, double rawY2) {
        double x = xScale * rawX;
        double y = yScale * rawY;
        double x1 = xScale * rawX1;
        double y1 = yScale * rawY1;
        double x2 = xScale * rawX2;
        double y2 = yScale * rawY2;
        if (Math.abs(x) > ABS_MAX || Math.abs(y) > ABS_MAX ||
                Math.abs(x1) > ABS_MAX || Math.abs(y1) > ABS_MAX ||
                Math.abs(x2) > ABS_MAX || Math.abs(y2) > ABS_MAX)
            throw new IllegalArgumentException(String.format(
                    "args too big %f %f %f %f %f %f", rawX, rawY, rawX1, rawY1, rawX2, rawY2));
        curveScaled(x, y, x1, y1, x2, y2);
    }

    private void curveScaled(double x, double y, double x1, double y1, double x2, double y2) {
        // convert the control points into Hermite derivatives
        double x0dot = (x1 - this.currentX) / 3;
        double y0dot = (y1 - this.currentY) / 3;
        double x1dot = (x - x2) / 3;
        double y1dot = (y - y2) / 3;
        Rotation2d r0 = new Rotation2d(x0dot, y0dot);
        Rotation2d r1 = new Rotation2d(x1dot, y1dot);
        Pose2d start = new Pose2d(this.currentX, this.currentY, r0);
        Pose2d end = new Pose2d(x, y, r1);
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                start, List.of(), end, new TrajectoryConfig(10, 0.1));

        operations.add(new Operation(true, trajectory));

        this.currentX = x;
        this.currentY = y;
    }

    public void close() {
        if (initialX == null || initialY == null)
            throw new IllegalStateException("can't close a path before starting one");
        lineScaled(initialX, initialY, true);
    }

    public List<Operation> getOperations() {
        return operations;
    }

}
