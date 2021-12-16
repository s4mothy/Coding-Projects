import java.util.*;

public class BattleBoatsBoard {
    private int width;
    private int height;
    private char[][] gameBoard;
    private int boatCt;
    private int boatsMade;
    private Boat[] fleetArr;

    public BattleBoatsBoard() {

        inputWidth();
        inputHeight(); //each requests inputs for width and height from user and inputs values into variables
                       //see methods for further details

        gameBoard = new char[this.width][this.height]; //game board is sized according to input height and width
        for(int i=0; i<gameBoard.length; i++) {
            for(int j=0; j<gameBoard[0].length; j++) {
                gameBoard[i][j] = '-'; //defaults to '-' for boatless locs in hidden grid ('-' for better visibility of boats)
            }
        }
        setBoatCt(); //sets Boat count based on height and width (see method for details)
    }

    public BattleBoatsBoard(BattleBoatsBoard other) { //essentially copies a Board to a new Board
        this.width = other.width;
        this.height = other.height;
        gameBoard = new char[other.width][other.height]; //game board is sized according to other board's height and width
        for(int i=0; i<other.gameBoard.length; i++) {
            for(int j=0; j<other.gameBoard[0].length; j++) {
                this.gameBoard[i][j] = other.gameBoard[i][j]; //sets game board values to the same
            }
        }
        setBoatCt(); //sets Boat count based on height and width (see method for details)

    }

    public void inputWidth() {
        Scanner s = new Scanner(System.in);
        System.out.println("Enter board width (3-12):");
        width = s.nextInt(); //request width
        while(width < 3 || width > 12) {
            System.out.println("Entered value is outside of range. Try again!");
            width = s.nextInt(); //re-requests width if not in range
        }
    }

    public void inputHeight() {
        Scanner s = new Scanner(System.in);
        System.out.println("Enter board height (3-12):");
        height = s.nextInt(); //requests height
        while(height < 3 || height > 12) {
            System.out.println("Entered value is outside of range. Try again!");
            height = s.nextInt(); //re-requests height if not in range
        }
    }

    public int getWidth() {
        return width; //basic 'getter' method
    }

    public int getHeight() {
        return height; //basic 'getter' method
    }

    public char[][] getGameBoard() {
        return gameBoard; //basic 'getter' method
    }

    public Boat[] getFleetArr() {
        return fleetArr; //basic 'getter' method
    }

    public char getCharAtPt(Point pt) { //returns a character at a given point on the board
        return gameBoard[pt.i][pt.j];
    }

    public void replaceCharAtPt(char newChar, Point pt) { //modifies character at given Point
        gameBoard[pt.i][pt.j] = newChar;
    }


    private void setBoatCt() { //sets boat count based on table provided, then sizes boat array accordingly
        if(width == 3 || height == 3) {
            boatCt = 1;
        } else if((width > 3 && width <= 6) || (height > 3 && height <= 6)) {
            boatCt = 2;
        } else if((width > 6 && width <= 8) || (height > 6 && height <= 8)) {
            boatCt = 3;
        } else if((width > 8 && width <= 10) || (height > 8 && height <= 10)) {
            boatCt = 4;
        } else {
            boatCt = 6;
        }

        fleetArr = new Boat[boatCt];
        boatsMade = 0;
    }


    private void addBoat() { //generates random boats, then checks if they would collide with current boats, generates until no collision
        boolean boatAdded = false;
        while(!boatAdded) {
            char dir;
            switch((int)Math.floor(2*Math.random())) { //generates a random orientation
                case 0: dir = 'h';                     //'h' for horizontal, 'v' for vertical
                    break;
                default: dir = 'v';
                    break;
            }

            int posI;
            int posJ;
            if(dir == 'h') {
                posI = 1 + (int)Math.floor(Math.random()*(width-2));  //boats are positioned based on center point
                posJ = (int)Math.floor(Math.random()*height);         //therefore, one direction cannot use points on the edge of the board

            } else {
                posI = (int)Math.floor(Math.random()*width);          //these calculations create random numbers in ranges
                posJ = 1 + (int)Math.floor(Math.random()*(height-2)); //that keep this limitation in mind
            }

            Boat newBoat = new Boat(dir, posI, posJ);

            boolean flag = false; //flags for newBoat hitting existing boat
            for(int ii=0; ii<newBoat.getBoatPts().length; ii++) {
                int testI = newBoat.getBoatPts()[ii].i;
                int testJ = newBoat.getBoatPts()[ii].j;
                if(gameBoard[testI][testJ] == 'B') {
                    flag = true; //if a boat already exists at this point, further steps are ignored and while loop continues
                }
            }

            if(!flag) {

                fleetArr[boatsMade] = newBoat; //adds boat to fleetArr
                boatsMade++;

                for(int ii=0; ii<newBoat.getBoatPts().length; ii++) { //adds boat to gameBoard
                    int newPosI = newBoat.getBoatPts()[ii].i;
                    int newPosJ = newBoat.getBoatPts()[ii].j;
                    gameBoard[newPosI][newPosJ] = 'B';
                }

                boatAdded = true;
            }
        }
    }

    public void populateBoats() { //creates boats based on boatCt
        for(int i=0; i<boatCt; i++) {
            addBoat();
        }
    }

    public boolean isFleetSunk() { //checks if all boats in fleetArr have hitCt >= 3 (game ends on this cond.)
        boolean fleetSunk = true;
        for(int i=0; i<fleetArr.length; i++) {
            if(!(fleetArr[i].isSunk())) {
                fleetSunk = false;
            }
        }
        return fleetSunk;
    }

    public void hideBoardChars() {
        for(int i=0; i<gameBoard.length; i++) {
            for(int j=0; j<gameBoard[0].length; j++) {
                gameBoard[i][j] = ' '; //sets to ' ' to make grid appear empty, creates default visible map
            }
        }
    }

    public String toString() { //prints board with marked axis for coordinates

        String ret = "   ";
        for(int i=0; i<gameBoard.length; i++) { //prints i-axis, needs to check for nums>9 to adjust spacing
            String gap;
            if(i<10) {
                gap = " ";
            } else {
                gap = "";
            }
            ret += gap + i + " ";
        }
        ret += "(i)\n";

        for(int j=0; j<gameBoard[0].length; j++) {

            String gap;
            if(j<10) {
                gap = " ";
            } else {
                gap = "";
            }
            ret += (gap + j + " "); //prints row value before printing spaces

            for(int i=0; i<gameBoard.length; i++) {
                ret += ("[" + gameBoard[i][j] + "]"); //prints row of game spaces at index i
            }
            ret += "\n";
        }
        ret+="(j)";

        return ret;
    }
}
