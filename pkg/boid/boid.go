package boid

import (
	"github.com/strobosco/boids/pkg/vector"
)


type Boids interface {
	Update()
}

type Boid struct {
	ImageWidth int
	ImageHeight int
	V vector.Vector
	O vector.Vector
	Alfa int
}

const (
	screenWidth  = 320
	screenHeight = 240
	maxAngle     = 256
)

func (s *Boid) Update() {
	// fmt.Println("Update")
	s.O.X += s.V.X
	s.O.Y += s.V.Y
	if s.O.X < 0 {
		s.O.X = -s.O.X
		s.V.X = -s.V.X
	} else if mx := screenWidth - s.ImageWidth; mx <= s.O.X {
		s.O.X = 2*mx - s.O.X
		s.V.X = -s.V.X
	}
	if s.O.Y < 0 {
		s.O.Y = -s.O.Y
		s.V.Y = -s.V.Y
	} else if my := screenHeight - s.ImageHeight; my <= s.O.Y {
		s.O.Y = 2*my - s.O.Y
		s.V.Y = -s.V.Y
	}
	s.Alfa++
	if s.Alfa == maxAngle {
		s.Alfa = 0
	}
}