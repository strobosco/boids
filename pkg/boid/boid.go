package boid

import (
	"github.com/strobosco/boids/pkg/vector"
)


type Boids interface {
	Logic(boids []*Boid)
}

type Boid struct {
	ImageWidth int
	ImageHeight int
	V vector.Vector
	O vector.Vector
}

const (
	screenWidth          = 800
	screenHeight         = 800
	maxSpeed     = 4.0
	maxForce	= 1.0
	perceptionRadius = 200.0
	separationIndex = 50.0
	alignmentIndex = 75.0
	cohesionIndex = 100.0
)

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

func (s *Boid) FindNeighbors(boids []*Boid) ([]*Boid) {

	neighbors := []*Boid{}

	for _, neighbor := range boids {
		if s.O.Distance(neighbor.O) <= perceptionRadius && s != neighbor {
			neighbors = append(neighbors, neighbor)
		}
	}

	return neighbors

}

func (s *Boid) alignment(boids []*Boid) (vector.Vector) {

	count := 0.0
	alignSteering := vector.Vector{}

	for _, neighbor := range boids {
		if s.O.Distance(neighbor.O) < alignmentIndex {
			alignSteering.X += neighbor.V.X
			alignSteering.Y += neighbor.V.Y
			count++
		}
	}

	alignSteering.X /= count
	alignSteering.Y /= count

	alignSteering.SetMagnitude(maxSpeed)

	alignSteering.X -= s.V.X
	alignSteering.Y -= s.V.Y

	alignSteering.Limit(maxForce)

	return alignSteering
}

func (s *Boid) cohesion(boids []*Boid) (vector.Vector) {

	count := 0.0
	cohesionSteering := vector.Vector{}

	for _, neighbor := range boids {
		if s.O.Distance(neighbor.O) < cohesionIndex {
			cohesionSteering.X += neighbor.O.X
			cohesionSteering.Y += neighbor.O.Y
			count++
		}
	}

	cohesionSteering.X /= count
	cohesionSteering.Y /= count

	cohesionSteering.X -= s.O.X
	cohesionSteering.Y -= s.O.Y

	cohesionSteering.SetMagnitude(maxSpeed)

	cohesionSteering.X -= s.V.X
	cohesionSteering.Y -= s.V.Y

	cohesionSteering.SetMagnitude(maxForce * 0.9)

	return cohesionSteering
}

func (s *Boid) separation(boids []*Boid) (vector.Vector) {

	count := 0.0
	separationSteering := vector.Vector{}

	for _, neighbor := range boids {
		if d := s.O.Distance(neighbor.O); d < separationIndex {
			diff := vector.Vector{}
			diff = s.O

			diff.X -= neighbor.O.X
			diff.Y -= neighbor.O.Y

			diff.X /= d
			diff.Y /= d

			separationSteering.X += diff.X
			separationSteering.Y += diff.Y
			count++
		}
	}

	separationSteering.X /= count
	separationSteering.Y /= count

	separationSteering.SetMagnitude(maxSpeed)

	separationSteering.X -= s.V.X
	separationSteering.Y -= s.V.Y

	separationSteering.SetMagnitude(maxForce * 1.2)

	return separationSteering
}

func (s *Boid) move(alignment, separation, cohesion vector.Vector) {
	
	// Calculate new vector
	acceleration := vector.Vector{}
	
	acceleration.X = (alignment.X + separation.X + cohesion.X ) / 3
	acceleration.Y = (alignment.Y + separation.Y + cohesion.Y ) / 3

	// Move boid from previous position
	s.O.X += s.V.X
	s.O.Y += s.V.Y
	
	// Apply new vector
	s.V.X += acceleration.X
	s.V.Y += acceleration.Y

	s.V.Limit(maxSpeed)

}

func (s *Boid) Logic(boids []*Boid) {
	// fmt.Println(boids[0], boids[10], boids[50])

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
	// fmt.Println(boids[0], boids[10], boids[50])

}