package org.team100;

import java.io.IOException;
import java.io.InputStream;
import java.nio.file.Files;
import java.nio.file.Path;

import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.org.team100.planning.Plotter;
import frc.robot.org.team100.planning.SvgReader;

public class SvgTest {

    private static InputStream stream(String filename) throws IOException {
        Path deployPath = Filesystem.getDeployDirectory().toPath();
        Path path = deployPath.resolve(filename);
        return Files.newInputStream(path);
    }

    @Test
    void testSvg() throws InterruptedException, IOException {
        System.out.println("start");

        Plotter plotter = new Plotter();

        // SvgReader reader = new SvgReader(stream("center_mark.svg"), 1, -1, plotter);
        // SvgReader reader = new SvgReader(stream("hat.svg"), 0.0005, -0.0005, plotter);
        SvgReader reader = new SvgReader(stream("circle.svg"), 1, -1, plotter);
        // SvgReader reader = new SvgReader(stream("penrose_triangle.svg"), 1, -1, plotter);
        // SvgReader reader = new SvgReader(stream("subpop.svg"), 1, -1, plotter);

        reader.run();

        // let the user look at the picture.
        Thread.sleep(10000);
        System.out.println("done");
    }

}
