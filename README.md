# push
Use expanding and contracting lightfields to direct/recharge robots and move boxes into a 2D shape.

## Installation
```cd push```

``` make```

You may need to change your Makefile to export your Box2D installation to your CPATH. See the line:

```export CPATH=/home/adam/Documents/packages/Box2D_v2.3.0/Box2D```

I had to build an older version of Box2D (namely 2.3.0) to circumvent the frustrating new build methods.

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

A typical run command:

```./push -r 60 -b 200 -s 0.6 -z 0.7 -t C -y H -p shapes/square.txt```

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
