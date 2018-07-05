# push
Use expanding and contracting lightfields to direct/recharge robots and move boxes into a 2D shape.

## Installation
```
cd push
make
```

You will need to edit your Makefile to export your Box2D installation to your CPATH. See the line:

```export CPATH=/home/adam/Documents/packages/Box2D_v2.3.0/Box2D```

I had to build an older version of Box2D (namely 2.3.0) to circumvent the frustrating new (as of May 2018) Box2D build methods.

If you are not running Linux, you can un-comment out the earlier section for MacOS.

## Running Push

By default (running via ```./push``` alone), the program will use a circular lightfield, square robots and square boxes.

Otherwise, you may specify the following options:

| Option        | Description   | Argument |
| ------------- |:-------------:| :-------------:
| -r      | Number of Robots | Integer |
| -b      | Number of Boxes      | Integer |
| -z | Robot size     | Float, meters |
| -s | Box size | Float, meters |
| -t | Robot shape | R = Rectangular, C = Circular |
| -y | Box shape | R = Rectangular, C = Circular, H = Hexagonal |
| -p | Path to polygon file | String |
| -g | How often to render the arena | Integer|
| -o | Output file name, saves a replay | String, no .txt needed|
| -i | Input file name, loads a replay | String, specify exact name|
| -x | Run without GUI | No Argument |
| -f | Set flare of corners | Positive Float |
| -d | Set drag speed | 0 <= Float <= 1 |

A typical run command:

```./push -r 60 -b 200 -s 0.6 -z 0.7 -t C -y H -f 1.75 -d 0.25 -p shapes/square.txt```

A typical run command with output:

```./push -r 60 -b 200 -s 0.6 -z 0.7 -t C -y H -g 50 -o myReplay```

A typical load command:

```./push -i myReplay.txt```

Note that (all other command-line arguments are overwritten by the input file header if an input file is specified).

If an output filename doesn't exist yet, it is created. Otherwise, it is overwritten. While the output file is human-readable, it should not be edited by hand. The header information is assumed to be reliable, and the positions/quantity of the robots and boxes are taken to be valid.

It is highly recommended that replays are saved with `-g >= 50` or so. Saving *every* state of the world (e.g. `-g = 1`) will result in a very large textfile. The intention is that replays will capture the most important information of a costly run: Although an expensive set-up (say, thousands of robots and thousands of boxes) may run very slowly, the replay will run comparatively much faster, as the only computations are the loads from the file, and not e.g. the physics of the world. As well, with sparse GUI rendering (the `-g >> 1` case), the world will jump from state to state, explicitly placing the objects wherever they need to be, saving all of the in-between calculations of the physics.

The flare option refers to scaling of corner vertices. This accounts for the rounded corners often exhibited in squares and rectangles. By extending the corners out, we can achieve far sharper corners. The float value corresponds to the scaling factor if the corner is a 90 degree angle. Other corners will have a less dramatic scale if the angle is more than 90 degrees, and more dramatic scale if it is less. The calculation is: `scale = 1/(angle/(90 * flare))`

## Polygon Files
A description of a [simple polygon](https://en.wikipedia.org/wiki/Simple_polygon)'s vertices.

Each line is a new vertex, and each pair of coordinates are separated by a whitespace character. The polygon will be constructed by connecting vertices that are adjacent lines in the polygon file, and connecting the vertex on the last line back to that on the first. The file should contain ordered vertices and nothing else.

Simple polygon file example (a square):
```
4 4
-4 4
-4 -4
4 -4
```
