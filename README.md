# min-annulus

Finding the smallest-width annulus enclosing a given point set.

#### Example output
![example](images/all-border.png)
Left to right:
* Point set
* Voronoi diagram in a bounding box
* Farthest-point Voronoi diagram
* Diagram overlay
* Smallest-width annulus

#### Algorithm steps
1. Compute a Voronoi diagram with Fortune's algorithm
2. Compute a farthest-point Voronoi diagram with an incremental algorithm
3. Generate a set of annulus candidates by overlaying two diagrams
4. Choose the best candidate (the one with the smallest width)
