package org.firstinspires.ftc.teamcode;

public class ExampleFunctionTest {

    // this is the main function that runs when you run this program
    // right-click in this window and choose "Run 'ExampleFunctionTest.main()' with Coverage"
    public static void main(String[] args) {
        double mySlope = 0.0235;
        double myOffset = 0.05;
        double ty = 1.124;

        double myPower = getTurretPower(ty, mySlope, myOffset);

        // this is what will print out to the screen
        System.out.println("ty=" + ty);
        System.out.println("Power=" + myPower);
    }

    // this is the function we're testing
    public static double getTurretPower (double ty, double slope, double offset) {
        if (ty < 0) {
            return (((ty * slope) - offset) * -1);
        } else if (ty > 0) {
            return (((ty * slope) + offset) * -1);
        } else {
            return 0;
        }
    }
}

