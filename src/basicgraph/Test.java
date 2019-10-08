package basicgraph;

import java.util.Random;

public class Test {

    public static void main(String [] args) {
        //demo for how 2D arrays work
        Random random = new Random();
        int [] [] twoD = new int[4][3];
        for (int index = 0 ; index < twoD.length; index ++) {
            for (int innerIndex = 0; innerIndex < 3; innerIndex ++){
                twoD[index] [innerIndex] = random.nextInt(10);
            }
        }
        for (int[] eachArray : twoD){
            for (int eachIntInEachArray: eachArray) {
                System.out.print(eachIntInEachArray + ",");
            }
            System.out.println("\n");
        }
    }
}
