package frc;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Line2D;

import javax.swing.JFrame;
import javax.swing.JPanel;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import frc.robot.arm.ArmTrajectories;
import frc.robot.armMotion.ArmAngles;

/** Visualize some paths. */
public class VisualTest {
    public static class Plot extends JPanel {
        protected void paintComponent(Graphics grf) {

            TrajectoryConfig config = new TrajectoryConfig(1, 1);
            ArmTrajectories trajectories = new ArmTrajectories(config);
            ArmAngles t0 = new ArmAngles(0.089803, 1.681915);
            ArmAngles t1 = new ArmAngles(0.316365, 1.147321);
            Trajectory trajectory = trajectories.onePoint(t0, t1, 90);

            super.paintComponent(grf);
            Graphics2D graph = (Graphics2D) grf;

            // get width and height
            int width = getWidth();
            int height = getHeight();

            double cx = width / 2;
            double cy = height / 2;

            // draw axes
            graph.draw(new Line2D.Double(cx, 0, cx, height));
            graph.draw(new Line2D.Double(0, cy, width, cy));

            double scale = Math.min(width, height) / Math.PI;
            System.out.printf("scale %5.3f\n", scale);

            graph.drawString("distal (upper)", (int) cx - 100, 25);
            graph.drawString("proximal(lower)", (int) width - 100, (int) cy + 25);

            graph.setPaint(Color.RED);

            for (double t = 0; t < trajectory.getTotalTimeSeconds(); t += 0.1) {
                Trajectory.State s = trajectory.sample(t);
                double x1 = s.poseMeters.getX();
                double y1 = s.poseMeters.getY();
                System.out.printf("x1 %5.3f y1 %5.3f\n", x1, y1);
                // note minus sign below; Y in jframe is downward positive.
                graph.fill(new Ellipse2D.Double(cx + scale * x1 - 2, cy - scale * y1 - 2, 4, 4));
            }
        }
    }

    @Test
    void testSimple() {
        JFrame frame = new JFrame();
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.add(new Plot());
        frame.setSize(400, 400);
        frame.setVisible(true);
        try {
            // let the user see it
            Thread.sleep(10000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

}
