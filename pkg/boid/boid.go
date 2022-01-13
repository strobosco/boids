// + Viewing angle seems to be working even though I don't know how to check it

package boid

import (
	"math"

	"github.com/strobosco/boids/pkg/constants"
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
	screenWidth      = constants.ScreenWidth      // * width of sim screen
	screenHeight     = constants.ScreenHeight     // * height of sim screen
	maxSpeed         = constants.MaxSpeed         // * max boid speed
	maxForce         = constants.MaxForce         // * constant for boid movement
	perceptionRadius = constants.PerceptionRadius // * range at which boids can sense other boids
	separationIndex  = constants.SeparationIndex  // * range at which separation is taken into account
	alignmentIndex   = constants.AlignmentIndex   // * range at which alignment is taken into account
	cohesionIndex    = constants.CohesionIndex    // * range at which cohesion is taken into account
)

func popNeighbor(slice []*Boid, i int) []*Boid {
	new := []*Boid{}
	for id, element := range slice {
		if id != i {
			new = append(new, element)
		}
	}
	return new
}

/*
* DISCLAIMER:
	The first function (currently commented out) was devised by myself but
	unfortunately I don't remember the implementation anymore (its been a few months).
	The second function comes from here (https://stackoverflow.com/questions/65265444/flocking-boids-algorithm-field-of-view-specified-by-angle-in-3d)
	and seems to work.

	Now, although the second function seems more mathematically correct it seems to be much slower. To
	get similar performance I had to halve the number of boids in the screen, from 400 to 200.
*/

func (s *Boid) inView(neighbors []*Boid) []*Boid {

	/* FUNCTION 1

	// boidAlfa := math.Atan((s.V.Y * math.Pi) / (s.V.X * 180))
	// boidAlfa = (180 * boidAlfa) / math.Pi
	// m1, m2 := boidAlfa-45, boidAlfa+45

	// for id, neighbor := range neighbors {
	// 	neighborM := (neighbor.O.Y - s.O.Y) / (neighbor.O.X - s.O.X)
	// 	if neighborM*neighbor.O.X < m1*neighbor.O.X && neighborM*neighbor.O.X > m2*neighbor.O.X {
	// 		neighbors = popNeighbor(neighbors, id)
	// 	}
	// 	if neighbor.O.Y/neighborM < neighbor.O.Y/m1 && neighbor.O.Y/neighborM > neighbor.O.Y/m2 {
	// 		neighbors = popNeighbor(neighbors, id)
	// 	}
	// }

	*/

	// FUNCTION 2
	
	const viewingAngle = 45
	for id, neighbor := range neighbors {
		cos1 := (s.V.X*(neighbor.O.X-s.O.X) + s.V.Y*(neighbor.O.Y-s.O.Y)) / (math.Sqrt(s.V.X*s.V.X+s.V.Y*s.V.Y) * math.Sqrt(math.Pow((neighbor.O.X-s.O.X), 2)+math.Pow((neighbor.O.Y-s.O.Y), 2)))
		
		if cos1 < math.Cos(viewingAngle * math.Pi / 180) {
			neighbors = popNeighbor(neighbors, id)
		}
	}

	return neighbors
}

/*
* Function that checks if the boid is within the edges of the screen.
* (could change and include mx = difference between screen width/height
* and the image width/height)
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

	neighbors = s.inView(neighbors)

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
