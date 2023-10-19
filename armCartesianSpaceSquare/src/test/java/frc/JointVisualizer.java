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
            double x1 = s.poseMeters.getX();
            double y1 = s.poseMeters.getY();
            series1.add(x1, y1);
        }
        dataset.addSeries(series1);
        return dataset;
    }

    private static XYSeriesCollection jointToCartesian(XYSeriesCollection data) {
        ArmKinematics k = new ArmKinematics(1, 1);
        XYSeries joints = data.getSeries(0);
        XYSeries cartesian = new XYSeries("Cartesian");
        int ct = joints.getItemCount();
        for (int i = 0; i < ct; ++i) {
            Number nx = joints.getX(i);
            Number ny = joints.getY(i);
            double proximal = nx.doubleValue();
            double distal = ny.doubleValue();
            Translation2d c = k.forward(new ArmAngles(proximal, distal));
            cartesian.add(c.getX(), c.getY());
        }
        XYSeriesCollection result = new XYSeriesCollection();
        result.addSeries(cartesian);
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

            ((XYPlot) jointChart.getPlot()).setBackgroundPaint(Color.WHITE);

            ChartPanel jointPanel = new ChartPanel(jointChart) {
                @Override
                public Dimension getPreferredSize() {
                    return new Dimension(400,400);
                }
            };
            frame.add(jointPanel, BorderLayout.EAST);

            XYSeriesCollection cartesian = jointToCartesian(joints);
            JFreeChart cartesianChart = ChartFactory.createScatterPlot(
                    "Trajectory in Cartesian Space",
                    "X",
                    "Y",
                    cartesian);

            ((XYPlot) cartesianChart.getPlot()).setBackgroundPaint(Color.WHITE);

            ChartPanel cartesianPanel = new ChartPanel(cartesianChart) {
                @Override
                public Dimension getPreferredSize() {
                    return new Dimension(400,400);
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