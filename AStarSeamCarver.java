package seamcarving;

import astar.AStarGraph;
import astar.WeightedEdge;
import edu.princeton.cs.algs4.Picture;

import astar.AStarSolver;
import java.awt.Color;
import java.awt.Point;
import java.util.ArrayList;
import java.util.List;
import java.util.ListIterator;

public class AStarSeamCarver implements SeamCarver {
    private Picture picture;
    private Point top;
    private Point horizFinish;
    private Point vertFinish;

    public AStarSeamCarver(Picture picture) {
        if (picture == null) {
            throw new NullPointerException("Picture cannot be null.");
        }
        this.picture = new Picture(picture);
        this.top = new Point(-1, -1);
        this.horizFinish = new Point(width() - 1, -1);
        this.vertFinish = new Point(-1, height() - 1);
    }

    public Picture picture() {
        return new Picture(picture);
    }

    public void setPicture(Picture picture) {
        this.picture = picture;
    }

    public int width() {
        return picture.width();
    }

    public int height() {
        return picture.height();
    }

    public Color get(int x, int y) {
        return picture.get(x, y);
    }

    public double energy(int x, int y) {
        int w = width();
        int h = height();

        if (x < 0 || x >= w) {
            throw new IndexOutOfBoundsException("x is out of bounds");
        } else if (y < 0 || y >= h) {
            throw new IndexOutOfBoundsException("y is out of bounds");
        }

        Color left = get(Math.floorMod(x - 1, w), y);
        Color right = get(Math.floorMod(x + 1, w), y);
        Color above = get(x, Math.floorMod(y - 1, h));
        Color below = get(x, Math.floorMod(y + 1, h));

        double deltaXR = (Math.abs(left.getRed() - right.getRed()));
        double deltaXG = (Math.abs(left.getGreen() - right.getGreen()));
        double deltaXB = (Math.abs(left.getBlue() - right.getBlue()));
        double deltaYR = (Math.abs(above.getRed() - below.getRed()));
        double deltaYG = (Math.abs(above.getGreen() - below.getGreen()));
        double deltaYB = (Math.abs(above.getBlue() - below.getBlue()));

        double deltaX = Math.pow(deltaXR, 2) + Math.pow(deltaXG, 2) + Math.pow(deltaXB, 2);
        double deltaY = Math.pow(deltaYR, 2) + Math.pow(deltaYG, 2) + Math.pow(deltaYB, 2);

        return  Math.sqrt(deltaX + deltaY);
    }

    public int[] findHorizontalSeam() {
        SeamGraph g = new SeamGraph(true);
        int[] hSeam = new int[width()];
        AStarSolver solver = new AStarSolver(g, top, horizFinish, 10000);
        ListIterator<Point> iter = solver.solution().listIterator();
        iter.next();
        while (iter.hasNext()) {
            int index = iter.nextIndex() - 1;
            Point p = iter.next();
            if (iter.hasNext()) {
                hSeam[index] = p.y;
            }
        }
        return hSeam;
    }

    public int[] findVerticalSeam() {
        SeamGraph g = new SeamGraph(false);
        int[] vSeam = new int[height()];
        AStarSolver solver = new AStarSolver(g, top, vertFinish, 10000);
        ListIterator<Point> iter = solver.solution().listIterator();
        iter.next();
        while (iter.hasNext()) {
            int index = iter.nextIndex() - 1;
            Point p = iter.next();
            if (iter.hasNext()) {
                vSeam[index] = p.x;
            }
        }
        return vSeam;
    }

    final class SeamGraph implements AStarGraph<Point> {
        private boolean horizSeam;

        private SeamGraph(boolean hSeam) {
            this.horizSeam = hSeam;
        }

        @Override
        public List<WeightedEdge<Point>> neighbors(Point p) {
            List<WeightedEdge<Point>> neighbors = new ArrayList<>();
            int w = width();
            int h = height();

            if (p.equals(top)) {
                if (this.horizSeam) {
                    for (int i = 0; i < h; i++) {
                        Point p2 = new Point(0, i);
                        neighbors.add(new WeightedEdge<>(top, p2, energy(0, i)));
                    }
                } else {
                    for (int i = 0; i < width(); i++){
                        Point n = new Point(i, 0);
                        neighbors.add(new WeightedEdge<>(top, n, energy(i, 0)));
                    }
                }
            } else {
                if (this.horizSeam) {
                    if (p.x == w - 1) {
                        neighbors.add(new WeightedEdge<>(p, horizFinish, 0));
                        return neighbors;
                    } else {
                        Point mid = new Point(p.x + 1, p.y);
                        neighbors.add(new WeightedEdge<>(p, mid, energy(p.x + 1, p.y)));
                        if (p.y != 0) {
                            Point up = new Point(p.x + 1, p.y - 1);
                            neighbors.add(new WeightedEdge<>(p, up, energy(p.x + 1, p.y - 1)));
                        }
                        if (p.y != h - 1) {
                            Point down = new Point(p.x + 1, p.y + 1);
                            neighbors.add(new WeightedEdge<>(p, down, energy(p.x + 1, p.y + 1)));
                        }
                    }
                } else {
                    if (p.y == h - 1) {
                        neighbors.add(new WeightedEdge<>(p, vertFinish, 0));
                        return neighbors;
                    } else {
                        Point mid = new Point(p.x, p.y + 1);
                        neighbors.add(new WeightedEdge<>(p, mid, energy(p.x, p.y + 1)));
                        if (p.x != 0) {
                            Point left = new Point(p.x - 1, p.y + 1);
                            neighbors.add(new WeightedEdge<>(p, left, energy(p.x - 1, p.y + 1)));
                        }
                        if (p.x != w - 1) {
                            Point right = new Point(p.x + 1, p.y + 1);
                            neighbors.add(new WeightedEdge<>(p, right, energy(p.x + 1, p.y + 1)));
                        }
                    }
                }
            }

            return neighbors;
        }

        @Override
        public double estimatedDistanceToGoal(Point p, Point goal) {
            return 0;
        }
    }

}