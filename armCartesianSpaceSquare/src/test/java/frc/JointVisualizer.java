package frc;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Dimension;

import javax.swing.JFrame;
import javax.swing.SwingUtilities;
import javax.swing.WindowConstants;

import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.plot.XYPlot;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import frc.robot.arm.ArmTrajectories;
import frc.robot.armMotion.ArmAngles;
import frc.robot.armMotion.ArmKinematics;

/**
 * Visualize trajectories in joint space. Click "Run" below in vscode to see it.
 */
public class JointVisualizer {

    /** Return a dataset with one series with proximal in x and distal in y */
    private static XYSeriesCollection joints() {
        XYSeriesCollection dataset = new XYSeriesCollection();
        TrajectoryConfig config = new TrajectoryConfig(1, 1);
        ArmTrajectories trajectories = new ArmTrajectories(config);
        ArmAngles t0 = new ArmAngles(0.089803, 1.681915);
        ArmAngles t1 = new ArmAngles(0.316365, 1.147321);
        Trajectory trajectory = trajectories.onePoint(t0, t1, 90);

        XYSeries series1 = new XYSeries("Joints");
        for (double t = 0; t < trajectory.getTotalTimeSeconds(); t += 0.1) {
            Trajectory.State s = trajectory.sample(t);
            // note distal is X here
            // TODO: reverse these
            double x1 = s.poseMeters.getY(); // proximal
            double y1 = s.poseMeters.getX(); // distal
            series1.add(x1, y1);
        }
        dataset.addSeries(series1);
        return dataset;
    }

    /**
     * Add end and elbow series.
     * 
     * X represents "z" i.e. height
     * Y represents "x" i.e. forward
     * 
     * @param data has proximal in x and distal in y
     * 
     */
    private static XYSeriesCollection jointToCartesian(XYSeriesCollection data) {
        ArmKinematics k = new ArmKinematics(1, 1);
        XYSeries joints = data.getSeries(0);
        XYSeries end = new XYSeries("Cartesian End");
        XYSeries elbow = new XYSeries("Cartesian Elbow");
        int ct = joints.getItemCount();
        for (int i = 0; i < ct; ++i) {
            Number nx = joints.getX(i); // proximal
            Number ny = joints.getY(i); // distal
            double proximal = nx.doubleValue();
            double distal = ny.doubleValue();
            ArmAngles a = new ArmAngles(proximal, distal);
            Translation2d c = k.forward(a);
            end.add(c.getY(), c.getX());
            Translation2d el = k.elbow(a);
            elbow.add(el.getY(), el.getX());
        }
        XYSeriesCollection result = new XYSeriesCollection();
        result.addSeries(end);
        result.addSeries(elbow);
        XYSeries base = new XYSeries("Base");
        base.add(0, 0);
        result.addSeries(base);
        return result;

    }

    /** Click "Run" below in vscode. */
    public static void main(String[] args) {
        SwingUtilities.invokeLater(() -> {

            JFrame frame = new JFrame("Chart Collection");

            XYSeriesCollection joints = joints();
            JFreeChart jointChart = ChartFactory.createScatterPlot(
                    "Trajectory in Joint Space",
                    "Proximal (Lower)",
                    "Distal (Upper)",
                    joints);

            XYPlot jointXY = (XYPlot) jointChart.getPlot();
            jointXY.setBackgroundPaint(Color.WHITE);
            jointXY.getDomainAxis().setRange(0, 1.5); // 1.5
            jointXY.getRangeAxis().setRange(1.0, 2.5); // 1.5

            ChartPanel jointPanel = new ChartPanel(jointChart) {
                @Override
                public Dimension getPreferredSize() {
                    return new Dimension(400, 400);
                }
            };
            frame.add(jointPanel, BorderLayout.EAST);

            XYSeriesCollection cartesian = jointToCartesian(joints);
            JFreeChart cartesianChart = ChartFactory.createScatterPlot(
                    "Trajectory in Cartesian Space",
                    "X (forward)",
                    "Z (up)",
                    cartesian);

            XYPlot cartesianXY = (XYPlot) cartesianChart.getPlot();
            cartesianXY.setBackgroundPaint(Color.WHITE);
            cartesianXY.getDomainAxis().setRange(-0.5, 1.5);
            cartesianXY.getRangeAxis().setRange(-0.5, 1.5);

            ChartPanel cartesianPanel = new ChartPanel(cartesianChart) {
                @Override
                public Dimension getPreferredSize() {
                    return new Dimension(400, 400);
                }
            };
            frame.add(cartesianPanel, BorderLayout.WEST);
            frame.setLocationRelativeTo(null);
            frame.setDefaultCloseOperation(WindowConstants.EXIT_ON_CLOSE);
            frame.pack();
            frame.setVisible(true);
        });
    }
}