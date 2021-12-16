public class Drone {
    private Point centPt;
    private Point[] dronePts;

    public Drone(Point pt) {
        dronePts = new Point[5];
        centPt = new Point(pt.i,pt.j);
        for(int i=0; i<dronePts.length; i++){
            dronePts[i] = new Point(centPt);
        }
        dronePts[1].i++;
        dronePts[2].j++;
        dronePts[3].i--;
        dronePts[4].j--; //adjusting pts for the "+" pattern
    }

    public Point[] getDronePts() {
        return dronePts;
    }

}
