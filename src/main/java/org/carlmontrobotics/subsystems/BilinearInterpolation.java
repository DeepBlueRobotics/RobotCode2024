package org.carlmontrobotics.subsystems;

public class BilinearInterpolation {

    private static final int size = 3;

    double[] dataX = new double[size];
    double[] dataY = new double[size];

    double[][] values = new double[size][size];

    public double interpolate(double x, double y) {
        double[] xIndices = getClosestIndices(dataX, x);
        double[] yIndices = getClosestIndices(dataY, y);

        double x1 = dataX[(int) xIndices[0]];
        double x2 = dataX[(int) xIndices[1]];
        double y1 = dataY[(int) yIndices[0]];
        double y2 = dataY[(int) yIndices[1]];

        double q11 = values[(int) xIndices[0]][(int) yIndices[0]];
        double q12 = values[(int) xIndices[0]][(int) yIndices[1]];
        double q21 = values[(int) xIndices[1]][(int) yIndices[0]];
        double q22 = values[(int) xIndices[1]][(int) yIndices[1]];

        double xDirection =
                (((x2 - x) / (x2 - x1)) * q11) + (((x - x1) / (x2 - x1)) * q21);
        double xDirection2 =
                (((x2 - x) / (x2 - x1)) * q12) + (((x - x1) / (x2 - x1)) * q22);

        return (((y2 - y) / (y2 - y1)) * xDirection)
                + (((y - y1) / (y2 - y1)) * xDirection2);
    }

    public double[] getClosestIndices(double[] array, double value) {
        if (value <= array[0]) {
            return new double[] {0, 1};
        }

        if (value >= array[size - 1]) {
            return new double[] {size - 2, size - 1};
        }

        for (int i = 1; i < size; i++) {
            if (value < array[i]) {
                return new double[] {i - 1, i};
            }
        }

        return new double[] {size - 2, size - 1}; // will never return this unless the array isn't
                                                  // sorted
    }

    public double[][] getClosestPoints(double x, double y) {
        double[] xIndices = getClosestIndices(dataX, x);
        double[] yIndices = getClosestIndices(dataY, y);

        double[][] points = new double[4][3]; // index 0 is bottom left, 1 is bottom right, 2 is top
                                              // left, 3 is top right
                                              // the point is (x, y, value)
        int index = 0;

        for (double xIndex : xIndices) {
            for (double yIndex : yIndices) {
                points[index][0] = dataX[(int) xIndex];
                points[index][1] = dataY[(int) yIndex];
                points[index][2] = values[(int) xIndex][(int) yIndex];
                index++;
            }
        }

        return points;
    }


}
