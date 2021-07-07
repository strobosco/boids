package vector

import (
	"math"
)

type Vector struct {
	X float64
	Y float64
}

func (v *Vector) Limit(max float64) {
	magSq := v.MagnitudeSquared()
	if float64(magSq) > max*max {
		v.Divide(math.Sqrt(float64(magSq)))
		v.Multiply(max)
	}
}

func (v *Vector) Divide(z float64) {
	v.X /= z
	v.Y /= z
}

func (v *Vector) Multiply(z float64) {
	v.X *= z
	v.Y *= z
}

func (v *Vector) MagnitudeSquared() float64 {
	return v.X*v.X + v.Y*v.Y
}

func (v *Vector) SetMagnitude(z float64) {
	v.Normalize()
	v.X *= z
	v.Y *= z
}

func (v *Vector) Normalize() {
	mag := math.Sqrt(v.X * v.X + v.Y * v.Y)
	v.X /= mag
	v.Y /= mag
}

func (v *Vector) Distance(v2 Vector) float64 {
	return math.Sqrt(math.Pow(float64(v2.X)-float64(v.X), 2) + math.Pow(float64(v2.Y)-float64(v.Y), 2))
}