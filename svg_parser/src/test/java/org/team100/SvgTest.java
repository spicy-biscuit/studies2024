package org.team100;

import java.io.IOException;
import java.io.InputStream;
import java.nio.file.Files;
import java.nio.file.Path;

import org.junit.jupiter.api.Test;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;
import org.w3c.dom.svg.SVGDocument;
import org.w3c.dom.svg.SVGPathSeg;
import org.w3c.dom.svg.SVGPathSegList;

import edu.wpi.first.wpilibj.Filesystem;
import io.sf.carte.echosvg.anim.dom.SAXSVGDocumentFactory;
import io.sf.carte.echosvg.anim.dom.SVGDOMImplementation;
import io.sf.carte.echosvg.anim.dom.SVGOMPathElement;
import io.sf.carte.echosvg.dom.svg.AbstractSVGPathSegList;
import io.sf.carte.echosvg.dom.svg.SVGPathSegItem;

public class SvgTest {
    @Test
    void testSvg() throws IOException, InterruptedException {
        System.out.println("start");

        Plotter plotter = new Plotter();

        Path deployPath = Filesystem.getDeployDirectory().toPath();

        // Path path = deployPath.resolve("center_mark.svg");
        // double xScale = 1;
        // double yScale = -1;

        Path path = deployPath.resolve("hat.svg");
        double xScale = 0.0005;
        double yScale = -0.0005;

        // Path path = deployPath.resolve("circle.svg");
        // double xScale = 1;
        // double yScale = -1;

        // Path path = deployPath.resolve("penrose_triangle.svg");
        // double xScale = 1;
        // double yScale = -1;

        // Path path = deployPath.resolve("subpop.svg");
        // double xScale = 1;
        // double yScale = -1;

        String svgNS = SVGDOMImplementation.SVG_NAMESPACE_URI;
        SAXSVGDocumentFactory factory = new SAXSVGDocumentFactory(null);
        InputStream is = Files.newInputStream(path);
        SVGDocument doc = factory.createDocument(svgNS, is);
        Element e = doc.getDocumentElement();
        NodeList nodeList = e.getElementsByTagName("path");
        int nodeListLength = nodeList.getLength();
        for (int i = 0; i < nodeListLength; ++i) {
            System.out.println("path " + i);
            SVGOMPathElement node = (SVGOMPathElement) nodeList.item(i);
            SVGPathSegList pathList = node.getNormalizedPathSegList();
            int pathListLength = pathList.getNumberOfItems();
            for (int j = 0; j < pathListLength; ++j) {
                SVGPathSeg item = pathList.getItem(j);
                if (item instanceof AbstractSVGPathSegList.SVGPathSegMovetoLinetoItem) {
                    AbstractSVGPathSegList.SVGPathSegMovetoLinetoItem m = (AbstractSVGPathSegList.SVGPathSegMovetoLinetoItem) item;
                    switch (m.getPathSegType()) {
                        case SVGPathSeg.PATHSEG_LINETO_ABS:
                            System.out.printf("Line Abs: %5.3f %5.3f\n", m.getX(), m.getY());
                            plotter.line(xScale * m.getX(), yScale * m.getY());
                            break;
                        case SVGPathSeg.PATHSEG_MOVETO_ABS:
                            System.out.printf("Move Abs: %5.3f %5.3f\n", m.getX(), m.getY());
                            plotter.move(xScale * m.getX(), yScale * m.getY());
                            break;
                        default:
                            System.out.printf("Unknown: %s %5.3f %5.3f\n", m.getPathSegTypeAsLetter(), m.getX(),
                                    m.getY());
                    }
                } else if (item instanceof AbstractSVGPathSegList.SVGPathSegCurvetoCubicItem) {
                    AbstractSVGPathSegList.SVGPathSegCurvetoCubicItem m = (AbstractSVGPathSegList.SVGPathSegCurvetoCubicItem) item;
                    switch (m.getPathSegType()) {
                        case SVGPathSeg.PATHSEG_CURVETO_CUBIC_ABS:
                            System.out.printf("Curve Abs: %5.3f %5.3f %5.3f %5.3f %5.3f %5.3f\n",
                                    m.getX(), m.getY(), m.getX1(), m.getY1(), m.getX2(), m.getY2());
                            plotter.curve(
                                    xScale * m.getX(), yScale * m.getY(),
                                    xScale * m.getX1(), yScale * m.getY1(),
                                    xScale * m.getX2(), yScale * m.getY2());
                            break;
                        default:
                            System.out.printf("Unknown: %s %5.3f %5.3f %5.3f %5.3f %5.3f %5.3f\n",
                                    m.getPathSegTypeAsLetter(),
                                    m.getX(), m.getY(),
                                    m.getX1(), m.getY1(),
                                    m.getX2(), m.getY2());

                    }
                } else if (item instanceof SVGPathSegItem) {
                    SVGPathSegItem m = (SVGPathSegItem) item;
                    switch (m.getPathSegType()) {
                        case SVGPathSeg.PATHSEG_CLOSEPATH:
                            System.out.println("close path");
                            plotter.close();
                            break;
                        default:
                            System.out.println("Unknown item letter: " + item.getPathSegTypeAsLetter());
                    }
                } else {
                    System.out.println("unknown item type: " + item.getClass().getName());
                }
            }
        }
        // let the user look at the picture.
        Thread.sleep(10000);
        System.out.println("done");
    }

}
