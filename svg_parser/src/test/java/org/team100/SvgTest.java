package org.team100;

import java.io.IOException;
import java.io.InputStream;
import java.nio.file.Files;
import java.nio.file.Path;

import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.org.team100.planning.OperationExecutor;
import frc.robot.org.team100.planning.SvgReader;
import frc.robot.org.team100.plotter.Operation;
import frc.robot.org.team100.plotter.SVGToPlotOperations;

public class SvgTest {

    private static InputStream stream(String filename) throws IOException {
        Path deployPath = Filesystem.getDeployDirectory().toPath();
        Path path = deployPath.resolve(filename);
        return Files.newInputStream(path);
    }

    @Test
    void testSvg1() throws InterruptedException, IOException {
        System.out.println("start");
        double xScale = 1;
        double yScale = -1;

        SVGToPlotOperations ops = new SVGToPlotOperations(xScale, yScale);
        SvgReader reader = new SvgReader(stream("center_mark.svg"), ops);

        reader.run();

        // this would be in the command initialize() method
        OperationExecutor exec = new OperationExecutor();

        // this would be in the command execute() method.

        for (Operation op: ops.getOperations()) {
            exec.executeTrajectory(op.isPenDown(), op.getTrajectory());
        }

        // let the user look at the picture.
        Thread.sleep(1000);
        System.out.println("done");
    }

    @Test
    void testSvg2() throws InterruptedException, IOException {
        System.out.println("start");
        double xScale = 0.0005;
        double yScale = -0.0005;

        SVGToPlotOperations ops = new SVGToPlotOperations(xScale, yScale);
        SvgReader reader = new SvgReader(stream("hat.svg"), ops);

        reader.run();

        // this would be in the command initialize() method
        OperationExecutor exec = new OperationExecutor();

        // this would be in the command execute() method.

        for (Operation op: ops.getOperations()) {
            exec.executeTrajectory(op.isPenDown(), op.getTrajectory());
        }

        // let the user look at the picture.
        Thread.sleep(1000);
        System.out.println("done");
    }

    @Test
    void testSvg3() throws InterruptedException, IOException {
        System.out.println("start");
        double xScale = 1;
        double yScale = -1;

        SVGToPlotOperations ops = new SVGToPlotOperations(xScale, yScale);
        SvgReader reader = new SvgReader(stream("circle.svg"), ops);

        reader.run();

        // this would be in the command initialize() method
        OperationExecutor exec = new OperationExecutor();

        // this would be in the command execute() method.

        for (Operation op: ops.getOperations()) {
            exec.executeTrajectory(op.isPenDown(), op.getTrajectory());
        }

        // let the user look at the picture.
        Thread.sleep(1000);
        System.out.println("done");
    }

    @Test
    void testSvg4() throws InterruptedException, IOException {
        System.out.println("start");
        double xScale = 1;
        double yScale = -1;

        SVGToPlotOperations ops = new SVGToPlotOperations(xScale, yScale);
        SvgReader reader = new SvgReader(stream("penrose_triangle.svg"), ops);

        reader.run();

        // this would be in the command initialize() method
        OperationExecutor exec = new OperationExecutor();

        // this would be in the command execute() method.

        for (Operation op: ops.getOperations()) {
            exec.executeTrajectory(op.isPenDown(), op.getTrajectory());
        }

        // let the user look at the picture.
        Thread.sleep(1000);
        System.out.println("done");
    }

    @Test
    void testSvg5() throws InterruptedException, IOException {
        System.out.println("start");
        double xScale = 1;
        double yScale = -1;

        SVGToPlotOperations ops = new SVGToPlotOperations(xScale, yScale);
        SvgReader reader = new SvgReader(stream("subpop.svg"), ops);
        reader.run();


        // this would be in the command initialize() method
        OperationExecutor exec = new OperationExecutor();

        // this would be in the command execute() method.

        for (Operation op: ops.getOperations()) {
            exec.executeTrajectory(op.isPenDown(), op.getTrajectory());
        }

        // let the user look at the picture.
        Thread.sleep(1000);
        System.out.println("done");
    }
}
