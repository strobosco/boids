package draw

import (
	"fmt"
	"image/color"
	_ "image/png"
	"log"
	"math"
	"math/rand"

	"github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/ebitenutil"
	"github.com/strobosco/boids/pkg/boid"
	"github.com/strobosco/boids/pkg/vector"
)

var (
	boidImage *ebiten.Image
)

const (
	screenWidth = 320
	screenHeight = 240
	maxAngle     = 256
)

type Game struct{
	boids []*boid.Boid
	op ebiten.DrawImageOptions
	inited  bool
}

func init() {
	fmt.Println("Loading image")
	boid, _, err := ebitenutil.NewImageFromFile("../pkg/draw/chevron-up.png")
	if err != nil {
		log.Fatal(err)
	}
	fmt.Println("Image loaded")

	// img, _, err := image.Decode(bytes.NewReader(images.Ebiten_png))
	// if err != nil {
	// 	log.Fatal(err)
	// }
	// boid := ebiten.NewImageFromImage(img)

	w, h := boid.Size()
	boidImage = ebiten.NewImage(w, h)
	op := &ebiten.DrawImageOptions{}
	op.ColorM.Scale(1, 1, 1, 1)
	boidImage.DrawImage(boid, op)
}

func (g *Game) init() {
	defer func ()  {
		g.inited = true
	}()

	g.boids = make([]*boid.Boid, 100)
	for i := range g.boids {
		w, h := boidImage.Size()
		x, y := rand.Intn(screenWidth-w), rand.Intn(screenHeight-h)
		vx, vy := 2*rand.Intn(2)-1, 2*rand.Intn(2)-1	
		a := rand.Intn(maxAngle)	
		g.boids[i] = &boid.Boid{
			V: vector.Vector{X: int(vx), Y: int(vy)},
			O: vector.Vector{X: int(x), Y: int(y)},
			Alfa: a,
		}
	}
	
}

func (g *Game) Update() error {
	if !g.inited {
		g.init()
	}

	for i := range g.boids {
		boid.Boids.Update(g.boids[i])
	}
	return nil
}

func (g *Game) Draw(screen *ebiten.Image) {
	screen.Fill(color.White)
	w, h := boidImage.Size()
	for i := 0; i < len(g.boids); i++ {
		s := g.boids[i]
		g.op.GeoM.Reset()
		g.op.GeoM.Translate(-float64(w)/2, -float64(h)/2)
		g.op.GeoM.Rotate(2 * math.Pi * float64(s.Alfa) / maxAngle)
		g.op.GeoM.Translate(float64(w)/2, float64(h)/2)
		g.op.GeoM.Translate(float64(s.O.X), float64(s.O.Y))
		screen.DrawImage(boidImage, &g.op)
	}
}

func (g *Game) Layout(outsideWidth, outsideHeight int) (int, int) {
	return screenWidth, screenHeight
}

func Main() {
	ebiten.SetWindowSize(screenWidth * 2, screenHeight * 2)
	ebiten.SetWindowTitle("Boids")
	if err := ebiten.RunGame(&Game{}); err != nil {
		log.Fatal(err)
	}
}