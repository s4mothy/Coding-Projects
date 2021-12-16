Created By Sam Williams <br>
Purpose: CSci 1933 Project 2, BattleBoats! :-O

# Battle Boats

## Required Setup

All .java files contained in the src folder MUST be kept in the same folder as 'BattleBoatsGame.java'.
BattleBoatsGame.java is the main file, so it needs to be compiled and ran to play the BattleBoats.


## Instructions

When run, BattleBoatsGame.java will prompt you to enter "rules", "newGame" or "quit".
"rules" gives a basic overview of the rules of BattleBoats.
"newGame" starts a new game of BattleBoats.
"quit" exits the program.


## Rules

BattleBoats is pretty simple, you give the computer a size for a board, and it generates locations for boats on the board.
You need to guess where the boats are at, and blow them up! You can enter "({i},{j})", replacing {i} and {j} with the coordinates
you want to dry blowing up. The game will let you know if you hit a boat! A turn counter is constantly increasing on each hit,
but that's not the only way it goes up! If you try to blow up a space out of bounds or that you already blew up, you lose a turn!
You also have a drone that you can deploy (expends 4 turns waiting for it to come back) by using the command "drone ({i},{j})"
similar to the points before. This drone with reveal an area in a "+" shape around the point you enter. Edges can be picked, but
points out of bounds will cause a turn penalty (and the 4 turns waiting for the return).
Try to get the lowest score you can! I recommend 4x4 as easy mode, 6x6 for medium difficulty, 9x9 for hard, and 12x12 for extra hard.
<br>
One the grid:
- [  ] represents an unknown space.
- [\-] represents a blank space (gotten from drone)
- [B] represents an unhit boat space (gotten from drone)
- [m] represents a space that was fired at and was a miss
- [H] represents a space that was fired at and hit
- [S] represents a space that was a sunk ship occupies

## Debug Mode

If you want to just test how the code works, you are prompted UPON STARTING A NEW GAME if you want to utilize DEBUG MODE.
This shows you the entire board, but is great if you want to see how the code works.
If you are testing the drone, however, it's really only visible with debug mode disabled. Otherwise, it's basically just reveiling an already visible area.


## Notes on Placing Boats

The hardest part of this, besides setting up the actual game, had to be generating the boats correctly.
Essentially, board creates an instance of the Boat class, assigning it a random direction, and a random center-point based on the direction.
Basically, if you have a horizontal boat, your horizontal coordinates are one less on either side, so the boat doesnt run of the edge.

Then it was a matter of checking if a new boat was colliding with an existing one.
If it was, the process was repeated, otherwise the boat was added to an array, and the points of the boat were used to change the values on the board.


## Ideas for Improvements

A couple things I think could be easily implemented are:
- PvP, just add another board and create a bunch of white space so they can't see it on each other's turn
- PvComp, have the computer randomly guess spots, but when it finds one, checks around it, could be challenging
- Scoreboard, really would be neat, like an arcade, needs a prompt for the name at the start, could print at the end
- Bomb, uses 2-3 turns to deploy, but creates a hit in a 2x2 area or something along those lines, maybe have a limited number
