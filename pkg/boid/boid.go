package boid

import (
	"boids/pkg/center"
	"math"
)

type Boid struct {
	Vx int
	Vy int
	V int
	O center.Center
	Alfa int
}

func (boid *Boid) getVelocity() {
	argument := math.Hypot(float64(boid.Vx), float64(boid.Vy))
	boid.V = int(argument)
}

func (boid *Boid) getAlfa() {
	argument := math.Hypot(float64(boid.Vx), float64(boid.Vy))
	result := math.Acos(argument)
	boid.Alfa = int(result)
}

// func (boid Boid) Orientation() int {
// 	argument := math.Hypot(float64(boid.V.X), float64(boid.V.Y))
// 	result := math.Acos(argument)
// 	return int(result * 180 / math.Pi)
// }

// func (boid Boid) New(Vx, Vy, alfa int, O Center) {
// 	boid.Vx = Vx
// 	boid.Vy = Vy
// 	boid.V = int(math.Hypot(float64(Vx), float64(Vy)))
// 	boid.O = O
// 	boid.Alfa = alfa
// }
