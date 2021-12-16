public class Point { //I used a Point class, just to allow me to store 'i' and 'j' values as point arrays
                     //made it easier to makes sense of, in my opinion
    public int i;
    public int j; //public, because this class primarily exists so I say things like newBoat.getBoatPts[0].i

    public Point(int i, int j) { //creates point from input i and j values
        this.i = i;
        this.j = j;
    }

    public Point(Point ref) { //copies point from reference point
        this.i = ref.i;
        this.j = ref.j;
    }

    public boolean equals(Point other) { //two points are equal if their i and j values are equal
        if(this.i == other.i && this.j == other.j) {
            return true;
        } else {
            return false;
        }
    }

}
