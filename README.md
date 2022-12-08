# Lowest Cost Path Finder for Robot
This project was an assignment for an introductory computing course in sem 1 2022, where a robot vehicle resides in a matrix world with obstacles placed throughout the grid. The goal is to calculate the lowest cost path to every cell in the matrix from the robots home base, where rectilinear moves cost 2 and diagonal moves cost 3.

This is acheived by sweeping through the grid and updating the cost of each cell as necessary and keep repeating until no more changes are made.

Zones that are unreachable are marked by a different letter for each zone, starting at 'a' --> 'z'. For reachable cells if the last digit of the lowest cost is between 0 and 3, the second last digit is printed, and if between 4 and 9 a '.' is printed.

## Using the Program
The details of the robots matrix are inputted from stdin with the first line specifying the world dimensions and the following lines specifying the rectangular regions of all obstacles. For example the input:
```
7 3
4 4 0 1
5 5 1 2
```
means that the world has dimensions 7x3 with one obstacle from (4, 0) to (4, 1), and the other from (5, 1) to (5, 2).