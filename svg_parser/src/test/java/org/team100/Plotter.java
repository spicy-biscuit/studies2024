package org.team100;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.geom.Ellipse2D;
import java.util.List;

import javax.swing.JFrame;
import javax.swing.WindowConstants;

import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.plot.XYPlot;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

/**
 * Show cartesian trajectories on a chart.
 * 
 * Note that WPILib spline generation makes a scale assumption, don't give it
 * anything more than like 1000?
 */
public class Plotter {
    private static final double ABS_MAX = 1000;

    private final XYSeriesCollection dataset = new XYSeriesCollection();
    private final XYPlot xy;

    private Double x = null;
    private Double y = null;
    private Double initialX = null;
    private Double initialY = null;
    private int seriesIdx = 0;

    public Plotter() {
        JFrame frame = new JFrame("Chart Collection");
        JFreeChart chart = ChartFactory.createScatterPlot("Plotter", "X", "Y", dataset);
        chart.removeLegend();
        xy = (XYPlot) chart.getPlot();
        xy.setBackgroundPaint(Color.WHITE);
        ChartPanel panel = new ChartPanel(chart) {
            @Override
            public Dimension getPreferredSize() {
                return new Dimension(500, 500);
            }
        };
        frame.add(panel, BorderLayout.EAST);
        frame.setLocationRelativeTo(null);
        frame.setDefaultCloseOperation(WindowConstants.EXIT_ON_CLOSE);
        frame.pack();
        frame.setVisible(true);
    }

    /**
     * Pen-up, move to x, y.
     * 
     * Establishes new "initial point" used later in "close path".
     * 
     * x and y need to be smaller than like 1000
     */
    public void move(double x, double y) {
        if (Math.abs(x) > ABS_MAX || Math.abs(y) > ABS_MAX)
            throw new IllegalArgumentException(String.format("args too big %f %f", x, y));
        penUp();
        this.x = x;
        this.y = y;
        this.initialX = x;
        this.initialY = y;
    }

    /**
     * Pen-down, move to x, y
     * 
     * x and y need to be smaller than like 1000
     */
    public void line(double x, double y) {
        if (Math.abs(x) > ABS_MAX || Math.abs(y) > ABS_MAX)
            throw new IllegalArgumentException(String.format("args too big %f %f", x, y));
        penDown();

        if (this.x == null || this.y == null)
            throw new IllegalStateException("can't make a line before starting a path");

        // line is too short, trajectory will be confused.
        if (Math.abs(this.x - x) < 0.1 && Math.abs(this.y - y) < 0.1) {
            // update the current position without actually moving
            this.x = x;
            this.y = y;
            System.out.println("line too short");
            return;
        }

        // rot is the same at the start and the end, it's a straight line.
        Rotation2d rot = new Rotation2d(x - this.x, y - this.y);
        Pose2d start = new Pose2d(this.x, this.y, rot);
        Pose2d end = new Pose2d(x, y, rot);

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                start, List.of(), end, new TrajectoryConfig(10, 0.1));

        XYSeries series1 = new XYSeries(String.format("line %d", seriesIdx));
        for (double t = 0; t < trajectory.getTotalTimeSeconds(); t += 3) {
            Trajectory.State s = trajectory.sample(t);
            series1.add(s.poseMeters.getX(), s.poseMeters.getY());
        }

        dataset.addSeries(series1);
        xy.getRenderer().setSeriesShape(seriesIdx, new Ellipse2D.Double(0, 0, 5, 5));
        xy.getRenderer().setSeriesPaint(seriesIdx, Color.BLACK);
        this.x = x;
        this.y = y;
        seriesIdx++;
        fixScales();
    }

    /**
     * @param x  end
     * @param y  end
     * @param x1 start ctl
     * @param y1 start ctl
     * @param x2 end ctl
     * @param y2 end ctl
     */
    public void curve(double x, double y, double x1, double y1, double x2, double y2) {
        if (Math.abs(x) > ABS_MAX || Math.abs(y) > ABS_MAX ||
                Math.abs(x1) > ABS_MAX || Math.abs(y1) > ABS_MAX ||
                Math.abs(x2) > ABS_MAX || Math.abs(y2) > ABS_MAX)
            throw new IllegalArgumentException(String.format(
                    "args too big %f %f %f %f %f %f", x, y, x1, y1, x2, y2));
        penDown();

        if (this.x == null || this.y == null)
            throw new IllegalStateException("can't make a curve before starting a path");

        // convert the control points into Hermite derivatives
        double x0dot = (x1 - this.x) / 3;
        double y0dot = (y1 - this.y) / 3;
        double x1dot = (x - x2) / 3;
        double y1dot = (y - y2) / 3;
        Rotation2d r0 = new Rotation2d(x0dot, y0dot);
        Rotation2d r1 = new Rotation2d(x1dot, y1dot);
        Pose2d start = new Pose2d(this.x, this.y, r0);
        Pose2d end = new Pose2d(x, y, r1);
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                start, List.of(), end, new TrajectoryConfig(10, 0.1));
        XYSeries series1 = new XYSeries(String.format("curve %d", seriesIdx));
        for (double t = 0; t < trajectory.getTotalTimeSeconds(); t += 3) {
            Trajectory.State s = trajectory.sample(t);
            series1.add(s.poseMeters.getX(), s.poseMeters.getY());
        }
        dataset.addSeries(series1);
        xy.getRenderer().setSeriesShape(seriesIdx, new Ellipse2D.Double(0, 0, 5, 5));
        xy.getRenderer().setSeriesPaint(seriesIdx, Color.BLACK);
        this.x = x;
        this.y = y;
        seriesIdx++;
        fixScales();
    }

    private void fixScales() {
        xy.getDomainAxis().setAutoRange(true);
        xy.getRangeAxis().setAutoRange(true);
        double domainLength = xy.getDomainAxis().getRange().getLength();
        double rangeLength = xy.getRangeAxis().getRange().getLength();
        if (domainLength > rangeLength) {
            double rangeCenter = xy.getRangeAxis().getRange().getCentralValue();
            xy.getRangeAxis().setRange(rangeCenter - domainLength / 2, rangeCenter + domainLength / 2);
        } else {
            double domainCenter = xy.getDomainAxis().getRange().getCentralValue();
            xy.getDomainAxis().setRange(domainCenter - rangeLength / 2, domainCenter + rangeLength / 2);
        }
    }

    public void close() {
        if (initialX == null || initialY == null)
            throw new IllegalStateException("can't close a path before starting one");

        // path is already closed.
        if (Math.abs(this.x - initialX) < 0.1 && Math.abs(this.y - initialY) < 0.1) {
            System.out.println("close short");
            return;
        }
        line(initialX, initialY);
    }

    private void penUp() {
        // control a pen actuator
    }

    private void penDown() {
        // control a pen actuator
    }
}
