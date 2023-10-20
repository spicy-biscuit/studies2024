package frc.robot.org.team100.planning;

import java.io.IOException;
import java.io.InputStream;

import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;
import org.w3c.dom.svg.SVGDocument;
import org.w3c.dom.svg.SVGPathSeg;
import org.w3c.dom.svg.SVGPathSegList;

import io.sf.carte.echosvg.anim.dom.SAXSVGDocumentFactory;
import io.sf.carte.echosvg.anim.dom.SVGDOMImplementation;
import io.sf.carte.echosvg.anim.dom.SVGOMPathElement;
import io.sf.carte.echosvg.dom.svg.AbstractSVGPathSegList;
import io.sf.carte.echosvg.dom.svg.SVGPathSegItem;

public class SvgReader {
    private final InputStream input;
    private final double xScale;
    private final double yScale;
    private final Plotter plotter;

    public SvgReader(InputStream input, double xScale, double yScale, Plotter plotter) {
        this.input = input;
        this.xScale = xScale;
        this.yScale = yScale;
        this.plotter = plotter;
    }

    private NodeList getPaths() {
        try {
            String svgNS = SVGDOMImplementation.SVG_NAMESPACE_URI;
            SAXSVGDocumentFactory factory = new SAXSVGDocumentFactory(null);
            SVGDocument doc = factory.createDocument(svgNS, input);
            Element documentElement = doc.getDocumentElement();
            return documentElement.getElementsByTagName("path");
        } catch (IOException e) {
            e.printStackTrace();
            return new NodeList() {
                @Override
                public int getLength() {
                    return 0;
                }

                @Override
                public Node item(int arg0) {
                    return null;
                }
            };
        }
    }

    public void run() {
        NodeList nodeList = getPaths();
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
    }

}