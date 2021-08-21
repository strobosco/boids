package draw

import (
	"fmt"
	"image/color"
	_ "image/png"
	"log"
	"math"
	"math/rand"
	"time"

	"github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/ebitenutil"
	"github.com/strobosco/boids/pkg/boid"
	"github.com/strobosco/boids/pkg/constants"
	"github.com/strobosco/boids/pkg/vector"
)

var (
	boidImage *ebiten.Image
)

const (
	screenWidth  = constants.ScreenWidth  // * width of sim screen
	screenHeight = constants.ScreenHeight // * height of sim screen
	maxForce     = constants.MaxForce     // * constant for boid movement
	numBoids     = constants.NumBoids     // * number of boids
)

type Game struct {
	boids  []*boid.Boid // collection of boidss
	op     ebiten.DrawImageOptions
	inited bool // has the game initialized?
}

// general inizialiation that create the boids image
func init() {
	fmt.Println("Loading image")
	boid, _, err := ebitenutil.NewImageFromFile("../pkg/draw/chevron.png")
	if err != nil {
		log.Fatal(err)
	}
	fmt.Println("Image loaded")

	w, h := boid.Size()
	boidImage = ebiten.NewImage(w/2, h/2)
	op := &ebiten.DrawImageOptions{}
	op.ColorM.Scale(1, 1, 1, 1)
	boidImage.DrawImage(boid, op)
}

// Game inizialitaion func
func (g *Game) init() {
	defer func() {
		g.inited = true // init game
	}()

	// create slice of boids
	rand.Seed(time.Hour.Milliseconds())
	g.boids = make([]*boid.Boid, numBoids)

	// give boids random V (x; y), O (x; y) and assign image width and height
	for i := range g.boids {
		w, h := boidImage.Size()
		x, y := rand.Float64()*float64(screenWidth-w/2), rand.Float64()*float64(screenWidth-h/2)
		min, max := -maxForce, maxForce
		vx, vy := rand.Float64()*(max-min)+min, rand.Float64()*(max-min)+min
		g.boids[i] = &boid.Boid{
			ImageWidth:  w / 2,
			ImageHeight: h / 2,
			V:           vector.Vector{X: vx, Y: vy},
			O:           vector.Vector{X: x, Y: y},
		}
	}

}

// Update the boids position
func (g *Game) Update() error {
	if !g.inited {
		g.init()
	}
	for i := range g.boids {
		boid.Boids.Logic(g.boids[i], g.boids)
	}
	return nil
}

// draw the boids on screen
func (g *Game) Draw(screen *ebiten.Image) {
	screen.Fill(color.White)
	w, h := boidImage.Size()
	for i := 0; i < len(g.boids); i++ {
		s := g.boids[i]
		g.op.GeoM.Reset()
		g.op.GeoM.Translate(-float64(w/2)/2, -float64(h/2)/2)
		g.op.GeoM.Rotate(-1*math.Atan2(s.V.Y*-1, s.V.X) + math.Pi/2)
		g.op.GeoM.Translate(s.O.X, s.O.Y)
		screen.DrawImage(boidImage, &g.op)
	}
}

func (g *Game) Layout(outsideWidth, outsideHeight int) (int, int) {
	return screenWidth, screenHeight
}

func Main() {
	ebiten.SetWindowSize(screenWidth*2, screenHeight*2)
	ebiten.SetWindowTitle("Boids")
	if err := ebiten.RunGame(&Game{}); err != nil {
		log.Fatal(err)
	}
}
