package constants

const (
	ScreenWidth      = 400   // * width of sim screen
	ScreenHeight     = 400   // * height of sim screen
	MaxSpeed         = 4.0   // * max boid speed
	MaxForce         = 1.0   // * constant for boid movement
	PerceptionRadius = 100.0 // * range at which boids can sense other boids
	SeparationIndex  = 50.0  // * range at which separation is taken into account
	AlignmentIndex   = 75.0  // * range at which alignment is taken into account
	CohesionIndex    = 100.0 // * range at which cohesion is taken into account
	NumBoids         = 400   // * number of boids
)
