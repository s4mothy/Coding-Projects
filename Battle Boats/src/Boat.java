public class Boat {
    private char dir;
    private Point cenPt;
    private Point[] boatPts;
    private int hitCt;

    public Boat(char dir, int cenPtI, int cenPtJ) {

        this.dir = dir;
        cenPt = new Point(cenPtI, cenPtJ);
        boatPts = new Point[3];

        for(int ii=0; ii<boatPts.length; ii++) {
            this.boatPts[ii] = new Point(cenPt);
        }
        if(this.dir == 'h') {
            this.boatPts[0].i--;
            this.boatPts[2].i++;
        } else {
            this.boatPts[0].j--; //points around center point are adjusted based on dir
            this.boatPts[2].j++;
        }
        hitCt = 0;
    }

    public Point[] getBoatPts() { //basic getter method
        return boatPts;
    }

    public void incHitCt() { //increments hitCt
        hitCt++;
    }

    public boolean isSunk() { //checks if hitCt >= 3
        if(hitCt >= 3) {
            return true;
        } else {
            return false;
        }
    }

    public String toString() { //for testing purposes, prints the direction and point values of the boat
        String retStr = "(" + dir + ", (" + boatPts[0].i + ", " + boatPts[0].j + "), (";
        retStr += boatPts[1].i + ", " + boatPts[1].j + "), (" + boatPts[2].i + ", " + boatPts[2].j +"))";
        return retStr;
    }

}
