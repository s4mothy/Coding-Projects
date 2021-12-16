import java.util.*;

public class BattleBoatsGame {
    public int turnCt;
    private boolean debugMode;
    public boolean gameWin;
    public BattleBoatsBoard hiddenBoard;
    public BattleBoatsBoard visibleBoard;


    public BattleBoatsGame() {
        this.turnCt = 0;
        this.debugMode = false;
        this.gameWin = false;

    }

    public static String inputFromMainMenu() { //creates main menu, then requests input, waits until valid option is received before returning it
        String mainMenu = "";
        mainMenu += "\n";
        mainMenu += "~~ Main Menu (type phrase to continue) ~~\n";
        mainMenu += "      rules - prints the rules of the game\n";
        mainMenu += "    newGame - start a new game of BattleBoats\n";
        mainMenu += "       quit - exit the program\n"; //main menu text

        System.out.println(mainMenu); //prints main menu text
        Scanner s = new Scanner(System.in);
        String input = s.nextLine(); //user inputs text
        while(!input.equals("rules") && !input.equals("newGame") && !input.equals("quit")) {
            System.out.println("You have entered an incorrect option. Try again!");
            input = s.nextLine();
        } //cycles until valid option is picked
        return input; //returns input string
    }

    public void printRules() { //prints simple game rules, complex ones are in README
        String rules = "";
        rules += "Basic rules:\n";
        rules += "Win as fast as possible by entering either of the two options below to destroy all the ships!\n";
        rules += "({i},{j}) - fire at space @ position {i} across, {j} down\n";
        rules += "drone ({i},{j}) - deploy a drone to scan area around position at ({i},{j})\n";
        rules += "               (but you will lose 4 turns waiting on its return!)\n";
        rules += "penalty #1 - you lose 1 turn if you fire at or deploy a drone to a space out of bounds!\n";
        rules += "penalty #2 - you lose 1 turn if you fire at the same space twice!\n";
        rules += "For further rules and information on the mechanics, check out the README!";

        System.out.println(rules);
    }

    public void checkDebugMode() { //asks the user if they want to use debug mode, run in initGame()
        System.out.println("Run in debug mode? (Y/N)");
        Scanner s = new Scanner(System.in);
        String ans = s.nextLine();
        while(!ans.equalsIgnoreCase("Y") && !ans.equalsIgnoreCase("N")) { //only valid options are "y", "Y", "n" and "N"
            System.out.println("Not valid option. Try again!");
            ans = s.nextLine();
        }

        if(ans.equalsIgnoreCase("Y")) { //toggles debugMode based on answer
            this.debugMode = true;
        } else {
            this.debugMode = false;
        }

    }

    public void initGame() { //creates a new instance of the game, generating both boards, and resetting relevant variables
        checkDebugMode(); //asks if debug mode should be active
        turnCt = 0; //resets turn count
        gameWin = false; //resets gameWin status

        hiddenBoard = new BattleBoatsBoard(); //generates new hidden board
        hiddenBoard.populateBoats(); //creates and places boats (method is outlined in BattleBoatsBoard method)

        /*Text below was from testing a bug where the fleetArr was adding the last newBoat to every space in the array
        Just thought I'd leave it in, because this was my method for checking the points for each boat in the array*/
        //Boat[] fleetArr = hiddenBoard.getFleetArr();
        //for(int i=0; i<fleetArr.length; i++){
        //    System.out.println(fleetArr[i].toString());
        //}

        visibleBoard = new BattleBoatsBoard(hiddenBoard); //visibleBoard is a copy of hiddenBoard (to steal size variables mostly)
        visibleBoard.hideBoardChars(); //this replaces all characters in the board array with ' '

    }

    public void playGame() { //how we run the game, most of the steps are assigned to methods, for clarity
        Scanner s = new Scanner(System.in);
        while(!gameWin) {//this will loop until the fleet is sunk
            turnCt++;//turn count is incremented
            System.out.println("\nTurn " + turnCt + ":");//the turn number is printed
            printBoard();//the board is printed, which board is dependent on if debugMode is true or false
            reqNextMove();//this requests a move from the player, the result is processed in this method

            if(hiddenBoard.isFleetSunk()) { //checks if fleet is sunk, if true game ends
                gameWin = true;
            }
        }
        System.out.println("\nThe fleet has been destroy! You won in " + turnCt + " turns!"); //returns the # of turns it took to win

    }

    public void printBoard(){ //like I said, returns hiddenBoard if debug is true, visibleBoard otherwise
        if(debugMode == true) {
            System.out.println(hiddenBoard.toString());
        } else {
            System.out.println(visibleBoard.toString());
        }
    }

    public void reqNextMove() {//requests move from player, converts into point if valid, then processes the points with evalMovePt() and deployDrone()
        boolean nextMoveMade = false;
        Scanner s = new Scanner(System.in);
        while (!nextMoveMade) { //loops until a move is made, which only occurs if an input enters either method above
            System.out.print("Next Move: ");
            String move = s.nextLine(); //user enters next move

            try { //catches errors below and sends the user back to start

                if(move.substring(0,1).equals("(")) { //"(" dictates a point, will error if not valid

                    String temp = move.substring(1,move.length()-1); //creates point from input "(i,j)"
                    int inI = Integer.parseInt(temp.split(",")[0]);
                    int inJ = Integer.parseInt(temp.split(",")[1]);
                    Point nextMovePt = new Point(inI, inJ);

                    char resultType = evalMovePt(nextMovePt); //evalMovePt method is called, returns a char based on result
                    String moveOutcomeStr = getMoveOutcome(resultType); //char is read and correct String is called
                    System.out.println(moveOutcomeStr); //string is printed to let player know results

                    nextMoveMade = true;

                } else if(move.substring(0,5).equals("drone")) {//"drone" dictates a drone deployment, will error if points after are not valid

                    String temp = move.split(" ")[1]; //creates point from input "drone (i,j)"
                    temp = temp.substring(1, temp.length() - 1);
                    int inI = Integer.parseInt(temp.split(",")[0]);
                    int inJ = Integer.parseInt(temp.split(",")[1]);
                    Point nextMovePt = new Point(inI, inJ);

                    char resultType = deployDrone(nextMovePt); //deployDrone is called, returns a char based on result
                    String moveOutcomeStr = getMoveOutcome(resultType); //char is read and correct String is called
                    System.out.println(moveOutcomeStr); //string is printed to let player know results

                    nextMoveMade = true;

                } else {
                    System.out.println("Invalid entry. Try again!");
                    continue; //somehow getting here will cause it to send back to start
                }

            } catch (Exception e) {
                System.out.println("Invalid entry. Try again!");
                continue; //all errors will return to start of while loop
            }

        }

    }

    private char evalMovePt(Point nextMovePt) { //takes in user pt, checks 1) if it's out of bounds, 2) if it's been done, 3) if it's a hit, sets retVal accordingly
        Point pt = nextMovePt; //shortened for brevity
        char locChar; //used later, but cannot be initialized, or will cause error (which will eat the rest of the command; see 'try' above)
        char retVal = 'e'; //retVal is the returned character, defaulted to 'e' for error checking
        boolean penalty1 = pt.i < 0 || pt.j < 0 || pt.i > (hiddenBoard.getWidth()-1) || pt.j > (hiddenBoard.getHeight()-1); //checks if points are out of range
        if(penalty1) { //if out of range, increments turn count and sets retVal to 'p' for penalty
            turnCt++;
            retVal = 'p';
        } else if((locChar = hiddenBoard.getGameBoard()[pt.i][pt.j]) == 'H' || locChar == 'm' || locChar == 'S') { //checks if the space has been hit before
            turnCt++; //if hit before, increments turn count and sets retVal to 'p' for penalty
            retVal = 'p';
        } else if(locChar == 'B') { //checks if space is a boat piece
            hiddenBoard.replaceCharAtPt('H', pt); //if so, replaces the char in hiddenBoard and visibleBoard with 'H' for hit
            visibleBoard.replaceCharAtPt('H', pt);
            Boat[] fleetArr = hiddenBoard.getFleetArr();
            for (int i = 0; i < fleetArr.length; i++) {
                for (int j = 0; j < fleetArr[0].getBoatPts().length; j++) { //cycles through boat points of all boats in hiddenBoard
                    if (pt.equals(fleetArr[i].getBoatPts()[j])) { //checks if the current pt is the hit boat pt
                        fleetArr[i].incHitCt(); //if so, increments the boats hit counter
                        if (fleetArr[i].isSunk()) { //checks if boat is sunk (method in boat, checks if hitCt >= 3
                            Point[] boatPts = fleetArr[i].getBoatPts();
                            for (int k = 0; k < boatPts.length; k++) {
                                hiddenBoard.replaceCharAtPt('S', boatPts[k]); //if so, replaces 'H' with 'S', to help avoid confusion
                                visibleBoard.replaceCharAtPt('S', boatPts[k]);
                            }
                            retVal = 's'; //if sunk, retVal is set to 's' for sunk
                        } else {
                            retVal = 'h'; //if hit only, retVal is set to 'h' for hit
                        }
                    }
                }
            }
        } else if(locChar == '-') {  //checks if pt is a blank space
            hiddenBoard.replaceCharAtPt('m', pt);
            visibleBoard.replaceCharAtPt('m', pt); //if so, replaces the char in hiddenBoard and visibleBoard with 'm' for miss
            retVal = 'm'; //retVal is set to 'm' for miss
        } else {
            retVal = 'e'; //if somehow reaches here, retVal is set to 'e' for error
        }
        return retVal;
    }

    public char deployDrone(Point nextMovePt) { //drone is deployed and copies values from hiddenBoard to visibleBoard
        System.out.println("drone deployed..."); //lets them know the drone is deployed
        turnCt += 4; //turn count is incremented
        Point pt = nextMovePt;
        char retVal = 'e';
        boolean penalty = pt.i < 0 || pt.j < 0 || pt.i > (hiddenBoard.getWidth() - 1) || pt.j > (hiddenBoard.getHeight() - 1); //checks if point is in bounds
        if (penalty) { //if not, turn count incremented and retVal = 'p' for penalty
            turnCt++;
            retVal = 'p';
        } else {
            Drone newDrone = new Drone(nextMovePt); //new drone is created centered at nextMovePt
            Point[] dronePtArr = newDrone.getDronePts();
            for (int i = 0; i < dronePtArr.length; i++) {
                Point pt1 = dronePtArr[i];
                try { //this looks really odd, but this was a work around for points around the centPt being checked that are off the board.
                    char hidCharAtPt = hiddenBoard.getCharAtPt(pt1);
                    visibleBoard.replaceCharAtPt(hidCharAtPt, pt1); //for none-error points, char in hiddenBoard is copied to visibleBoard
                } catch (Exception e) {
                    ; //no return, just needs to be here
                }
            }
            retVal = 'd'; //retVal is set to 'd' for drone
        }
        return retVal;
    }

    public String getMoveOutcome(char resultType) { //retVal characters are interpretted and returned as their corresponding String
        switch(resultType) {
            case 'p': return "Penalty! Lose a turn!";
            case 's': return "You sunk my battle boat!";
            case 'h': return "A direct hit!";
            case 'm': return "Missed it by that much!";
            case 'd': return "Drone was successful! (4 turns expended)";
            default: return "//error//";

        }
    }

    public static void main(String[] args) {

        BattleBoatsGame newBattleBoats = new BattleBoatsGame();

        String titleStr = "";
        titleStr += "                   Welcome to BattleBoats!\n";
        titleStr += "The game where you blow up all enemy ships as fast as possible!";
        System.out.println(titleStr);

        String inputStr = newBattleBoats.inputFromMainMenu();

        while(!inputStr.equals("quit")) { //repeats until quit is selected

            if(inputStr.equals("rules")) { //prints rules if entered
                newBattleBoats.printRules();
            }

            if(inputStr.equals("newGame")) { //initializes, then plays a game
                newBattleBoats.initGame();
                newBattleBoats.playGame();

            }

            inputStr = newBattleBoats.inputFromMainMenu();

        }

        System.out.println("Thanks for playing!"); //little farewell text

    }

}
