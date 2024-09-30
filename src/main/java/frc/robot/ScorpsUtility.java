package frc.robot;

public final class ScorpsUtility {
    public static double getAbsoluteDifference(double numOne, double numTwo) {
        double difference = Math.abs(numOne - numTwo);
        return difference;
    }

    public static double getAverage(double numOne, double numTwo) {
        double average = (numOne + numTwo) / 2;
        return average;
    }
}
