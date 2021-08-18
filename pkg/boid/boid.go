package boid

import (
	"github.com/strobosco/boids/pkg/vector"
)

type Boids interface {
	Logic(boids []*Boid)
}

type Boid struct {
	ImageWidth  int           // * boid image width
	ImageHeight int           // * boid image height
	V           vector.Vector // * boid velocity vector
	O           vector.Vector // * boid center vector
}

const (
	screenWidth      = 400   // * width of sim screen
	screenHeight     = 400   // * height of sim screen
	maxSpeed         = 4.0   // * max boid speed
	maxForce         = 1.0   // * constant for boid movement
	perceptionRadius = 100.0 // * range at which boids can sense other boids
	separationIndex  = 50.0  // * range at which separation is taken into account
	alignmentIndex   = 75.0  // * range at which alignment is taken into account
	cohesionIndex    = 100.0 // * range at which cohesion is taken into account
)

/*
* Function that checks if the boid is within the edges of the screen.
* Could change and include mx = difference between screen width/height
* and the image width/height.
 */
func (s *Boid) CheckEdges() {
	if s.O.X < 0 {
		s.O.X = screenWidth
	} else if mx := screenWidth; float64(mx) <= s.O.X {
		s.O.X = 0
	}
	if s.O.Y < 0 {
		s.O.Y = screenHeight
	} else if my := screenHeight; float64(my) <= s.O.Y {
		s.O.Y = 0
	}
}

/*
* Need to find the neighbors close to the boids. This makes it so that
* the boid only references other boids it can see. However, this also
* takes into account the boids behind it so need to incorporate viewing angle.
 */
func (s *Boid) FindNeighbors(boids []*Boid) []*Boid {

	neighbors := []*Boid{}

	for _, neighbor := range boids {
		if s.O.Distance(neighbor.O) < perceptionRadius && s != neighbor {
			neighbors = append(neighbors, neighbor)
		}
	}

	return neighbors
}

/*
* Boids follow a common alignment so this function calculates the
* average alignment vector. After calculating, it subtracts the boids
* own movement vector and sets the magnitude (normalizes result).
 */
func (s *Boid) alignment(boids []*Boid) vector.Vector {

	count := 0.0
	alignSteering := vector.Vector{} // vector that will update alignment

	// Check if each neighbor is within alignment radius
	for _, neighbor := range boids {
		if s.O.Distance(neighbor.O) < alignmentIndex && s != neighbor {
			alignSteering.X += neighbor.V.X
			alignSteering.Y += neighbor.V.Y
			count++
		}
	}

	// Initially steering vectors are the sum of all alignments, which are then:
	if count > 0 {
		// Averaged over the entire number of boids within alignment radius
		alignSteering.X /= count
		alignSteering.Y /= count

		alignSteering.SetMagnitude(maxSpeed)

		// Remove the boids own direction as it would bias the end result
		alignSteering.X -= s.V.X
		alignSteering.Y -= s.V.Y

		alignSteering.Limit(maxForce) // limited to the max force
	}

	return alignSteering
}

/*
* Boids remain near one another, this function helps
* maintain the boids at a safe distance
 */
func (s *Boid) cohesion(boids []*Boid) vector.Vector {

	count := 0.0
	cohesionSteering := vector.Vector{} // vector with the mean position of neighboring boids

	// Check if neighbors are withing cohesion radius
	for _, neighbor := range boids {
		if s.O.Distance(neighbor.O) < cohesionIndex && s != neighbor {
			cohesionSteering.X += neighbor.O.X
			cohesionSteering.Y += neighbor.O.Y
			count++
		}
	}

	if count > 0 {
		cohesionSteering.X /= count
		cohesionSteering.Y /= count

		cohesionSteering.X -= s.O.X
		cohesionSteering.Y -= s.O.Y

		cohesionSteering.SetMagnitude(maxSpeed)

		cohesionSteering.X -= s.V.X
		cohesionSteering.Y -= s.V.Y

		cohesionSteering.SetMagnitude(maxForce * 0.9)
	}

	return cohesionSteering
}

/*
* Boids stay separated from one another as to not collide.
* This function checks the distance between the boids and
* steers the boid away from its neighbors.
 */
func (s *Boid) separation(boids []*Boid) vector.Vector {

	count := 0.0
	separationSteering := vector.Vector{}

	for _, neighbor := range boids {
		if d := s.O.Distance(neighbor.O); d < separationIndex && s != neighbor {
			diff := vector.Vector{}
			diff = s.O

			diff.X = diff.X - neighbor.O.X
			diff.Y = diff.Y - neighbor.O.Y

			diff.X /= d
			diff.Y /= d

			separationSteering.X += diff.X
			separationSteering.Y += diff.Y
			count++
		}
	}

	if count > 0 {
		separationSteering.X /= count
		separationSteering.Y /= count

		separationSteering.SetMagnitude(maxSpeed)

		separationSteering.X -= s.V.X
		separationSteering.Y -= s.V.Y

		separationSteering.SetMagnitude(maxForce * 1.2)
	}

	return separationSteering
}

/*
* The previous functions calculate the vectors that will
* influence the boids movement. This function applies the movement
* after calculating the accelertion vector resulting from applying
* the rules.
 */
// After applying rules with previous functions, move the boid
func (s *Boid) move(alignment, separation, cohesion vector.Vector) {

	// Calculate new vector
	acceleration := vector.Vector{}

	acceleration.X = (alignment.X + separation.X + cohesion.X) / 3
	acceleration.Y = (alignment.Y + separation.Y + cohesion.Y) / 3

	// * Move boid from previous position
	s.O.X += s.V.X
	s.O.Y += s.V.Y

	// * Change velocity vector (both intensity and direction)
	s.V.X += acceleration.X
	s.V.Y += acceleration.Y

	s.V.Limit(maxSpeed)

}

/*
* This function helps abstract the logic behind the boids movement.
 */
func (s *Boid) Logic(boids []*Boid) {
	// Check boundaries
	s.CheckEdges()

	// Find neighboring boids
	neighbors := s.FindNeighbors(boids)

	// Apply alignment, separation, and cohesion
	alignment := s.alignment(neighbors)

	cohesion := s.cohesion(neighbors)

	separation := s.separation(neighbors)

	// Move boid
	s.move(alignment, separation, cohesion)
}
