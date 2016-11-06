package aim4.util;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Point;
import java.awt.Polygon;
import java.awt.Rectangle;
import java.awt.Shape;
import java.awt.Stroke;
import java.awt.event.ActionEvent;
import java.awt.geom.*;

import java.awt.geom.Point2D;
import java.awt.geom.Line2D;
import java.awt.geom.GeneralPath;
import java.awt.geom.PathIterator;
import java.awt.geom.AffineTransform;
import java.awt.geom.IllegalPathStateException;
import java.awt.geom.NoninvertibleTransformException;

import java.util.ArrayList;
import java.util.List;

import javax.swing.AbstractAction;
import javax.swing.Action;
import javax.swing.JPanel;

public class ShapeUtils
{
	synchronized public static Area[] shapeSplit(Area a, Point2D p0, Point2D p1)
	{
		// Compute the angle of the line to the x-axis
		double dx = p1.getX() - p0.getX();
		double dy = p1.getY() - p0.getY();
		double angleRadToX = Math.atan2(dy, dx);

		// Align the area so that the line matches the x-axis
		AffineTransform at = new AffineTransform();
		at.rotate(-angleRadToX);
		at.translate(-p0.getX(), -p0.getY());
		Area aa = a.createTransformedArea(at);

		// Compute the upper and lower halves that the area
		// has to be intersected with
		Rectangle2D bounds = aa.getBounds2D();

		double half0minY = Math.min(0, bounds.getMinY());
		double half0maxY = Math.min(0, bounds.getMaxY());
		Rectangle2D half0 = new Rectangle2D.Double(
				bounds.getX(), half0minY,
				bounds.getWidth(), half0maxY-half0minY);

		double half1minY = Math.max(0, bounds.getMinY());
		double half1maxY = Math.max(0, bounds.getMaxY());
		Rectangle2D half1 = new Rectangle2D.Double(
				bounds.getX(), half1minY,
				bounds.getWidth(), half1maxY-half1minY);

		// Compute the resulting areas by intersecting
		// the original area with both halves, and
		// transform them back to their initial position
		Area a0 = new Area(aa);
		a0.intersect(new Area(half0));

		Area a1 = new Area(aa);
		a1.intersect(new Area(half1));

		try
		{
			at.invert();
		}
		catch (NoninvertibleTransformException e)
		{
			// Always invertible
		}
		a0 = a0.createTransformedArea(at);
		a1 = a1.createTransformedArea(at);

		return new Area[] { a0, a1 };
	}

	synchronized public static void drawArc(Graphics2D g, int x, int y, int width, int height, double start, double end, int innerXOffset, int innerYOffset)
	{
		Area a = createArc(x, y, width, height, start, end, innerXOffset, innerYOffset);
		g.draw(a);
	}

	synchronized public static void fillArc(Graphics2D g, int x, int y, int width, int height, double start, double end, int innerXOffset, int innerYOffset)
	{
		Area a = createArc(x, y, width, height, start, end, innerXOffset, innerYOffset);
		g.fill(a);
	}

	synchronized public static Area createArc(int x, int y, int width, int height, double start, double end, int innerXOffset, int innerYOffset)
	{
		Shape s = new Ellipse2D.Double(x,y, width, height);
		Area a = new Area(s);
		int center_x = x + width / 2;
		int center_y = y + height / 2;
		int xs[] = new int[6];
		int ys[] = new int[6];
		xs[0] = center_x;
		ys[0] = center_y;
		double middle = start + (end -start) / 2;
		double quarter1 =start + (middle - start)/2; //new point in the polygon between start and middle
		double quarter2 =middle + (end - middle)/2; //new point in the polygon between middle and end

		int pt1_x = (int) (center_x + width * Math.cos(start));
		int pt1_y = (int) (center_y + height * Math.sin(start));
		int pt2_x = (int) (center_x + width * Math.cos(end));
		int pt2_y = (int) (center_y + height * Math.sin(end));
		int mid_x = (int) (center_x + width * Math.cos(middle)); //now there is no need to *2 because with a polygon with 6 points the worst case (360 degrees) is guaranteed
		int mid_y = (int) (center_y + height * Math.sin(middle));
		int quar1_x= (int) (center_x + height * Math.cos(quarter1)); //calculates the x and y for the new points
		int quar1_y= (int) (center_y + height * Math.sin(quarter1));
		int quar2_x= (int) (center_x + height * Math.cos(quarter2));
		int quar2_y= (int) (center_y + height * Math.sin(quarter2));
		//inserts the new points in the polygon' array in the rigth order
		xs[1] = pt1_x;
		ys[1] = pt1_y;
		xs[2] = quar1_x;
		ys[2] = quar1_y;
		xs[3] = mid_x;
		ys[3] = mid_y;
		xs[4] = quar2_x;
		ys[4] = quar2_y;
		xs[5] = pt2_x;
		ys[5] = pt2_y;

		Polygon p = new Polygon(xs, ys, 6); // create the new polygon with the 6 points
		Area clip = new Area(p);
		a.intersect(clip);
		Ellipse2D.Double inner = new Ellipse2D.Double(x + innerXOffset, y + innerYOffset, width - innerXOffset * 2, height - innerYOffset * 2);
		a.subtract(new Area(inner));
		return a;
	}


	synchronized public static Shape getLineShape(Line2D line, float lineWidth) {
		Shape myStrokeShape;
		Stroke s = new BasicStroke(lineWidth);
		myStrokeShape = s.createStrokedShape(line);
		return myStrokeShape;
	}

	synchronized public static Polygon createStandardStar(double width, double height, int points, double centerRatio, double angleOffset) {
		int pts = points * 2;
		int xs[] = new int[pts];
		int ys[] = new int[pts];
		double xrad = width / 2;
		double yrad = height / 2;
		int innerx = (int) (xrad * centerRatio);
		int innery = (int) (yrad * centerRatio);
		double startangle = 0 + angleOffset;
		double anglePer = 2 * Math.PI / points;
		for (int i = 0; i < points ; i++) {
			double angle = startangle + anglePer * i;
			xs[i * 2] = (int) (xrad + xrad * Math.sin(angle));
			ys[i * 2] = (int) (yrad - yrad * Math.cos(angle));
			xs[i * 2 + 1] = (int) (xrad + innerx * Math.sin(angle + anglePer / 2));
			ys[i * 2 + 1] = (int) (yrad - innery * Math.cos(angle + anglePer / 2));
		}
		Polygon p = new Polygon(xs, ys, pts);
		return p;
	}

	synchronized public static void rotateArea(Area a, double rotation, Point2D rotateAround)
	{
		AffineTransform at1 = AffineTransform.getTranslateInstance(rotateAround.getX(), rotateAround.getY());
		at1.rotate(rotation);
		at1.translate(-rotateAround.getX(), -rotateAround.getY());
		a.transform(at1);
	}

	synchronized public static void rotateArea(Area a, double rotation)
	{
		Rectangle2D bounds = a.getBounds2D();
		rotateArea(a, rotation, new Point2D.Double(bounds.getX() + bounds.getWidth() / 2, bounds.getY() + bounds.getHeight() /2));
	}

	synchronized public static Point2D.Double getCenter(Shape s)
	{
		Rectangle r = s.getBounds();
		double totalX =0, totalY = 0;
		int pts = 0;
		for (int i = 0; i < r.width; i++) {
			for (int j = 0; j < r.height; j++) {
				if (s.contains(i,j)) {
					totalX += i;
					totalY += j;
					pts++;
				}
			}
		}
		totalX /= pts;
		totalY /= pts;
		return new Point2D.Double((int) Math.round(totalX), (int)Math.round(totalY));
	}

	@SuppressWarnings("serial")
	synchronized public static void rotateTest() {
		final Polygon p = createStandardStar(100,100,5, 0.2, 0);
		p.translate(50, 50);
		final Area myArea = new Area(p);
		final JPanel jp = new JPanel() {
			public void paintComponent(Graphics g) {
				super.paintComponent(g);
				g.setColor(Color.RED);
				((Graphics2D)g).fill(myArea);
			}
		};
		jp.setPreferredSize(new Dimension(200,200));
		Action a = new AbstractAction("Rotate The Star!") {
			public void actionPerformed(ActionEvent ae) {
				rotateArea(myArea, Math.PI / 6);
				//Arrays.printArray(m)
				jp.repaint();
			}
		};
	}


}
