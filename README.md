# Boids Simulation

Recreation of Craig Reynold's boid simulation in Golang.

Attempt at recreating the boids simulation by Craig Reynolds in Golang.

## Rules

The three rules that determine the boid's movements are 

**ALIGNMENT** -> boids try to move in the same direction

**SEPARATION** -> boids try to stay away from each other and not collide

**COHESION** -> boids try to remain in a compact flock

## Directory structure

```
boids
+-- cmd           // contains the main entry file to run (main.go)
|
+-- pkg
|  |
|  +-- boid       // contains complete boid logic
|  +-- draw       // contains ebiten drawing functions
|  +-- vector     // contains vector math
|  +-- constants  // contains constants
```