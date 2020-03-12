package hexgrid

import (
	"fmt"
	"math"
)

type Direction int

const (
	DirectionSE Direction = iota
	DirectionNE
	DirectionN
	DirectionNW
	DirectionSW
	DirectionS
)

var directions = []Hex{
	NewHex(1, 0),
	NewHex(1, -1),
	NewHex(0, -1),
	NewHex(-1, 0),
	NewHex(-1, +1),
	NewHex(0, +1),
}

func (this Direction) ToString() string {
	switch this {
	case DirectionSE:
		return "SE";
	case DirectionNE:
		return "NE";
	case DirectionN:
		return "N";
	case DirectionNW:
		return "NW";
	case DirectionSW:
		return "SW";
	case DirectionS:
		return "S";
	default:
		panic("Invalid direction")
	}
}

// Hex describes a regular hexagon with Cube Coordinates (although the S coordinate is computed on the constructor)
// It's also easy to reference them as axial (trapezoidal coordinates):
// - R represents the vertical axis
// - Q the diagonal one
// - S can be ignored
// For additional reference on these coordinate systems: http://www.redblobgames.com/grids/hexagons/#coordinates
//           _ _
//         /     \
//    _ _ /(0,-1) \ _ _
//  /     \  -R   /     \
// /(-1,0) \ _ _ /(1,-1) \
// \  -Q   /     \       /
//  \ _ _ / (0,0) \ _ _ /
//  /     \       /     \
// /(-1,1) \ _ _ / (1,0) \
// \       /     \  +Q   /
//  \ _ _ / (0,1) \ _ _ /
//        \  +R   /
//         \ _ _ /
type Hex struct {
	Q int // x axis
	R int // y axis
	S int // z axis
}

func NewHex(q, r int) Hex {

	h := Hex{Q: q, R: r, S: -q - r}
	return h

}

func (h Hex) String() string {
	return fmt.Sprintf("(%d,%d)", h.Q, h.R)
}

// fractionHex provides a more precise representation for hexagons when precision is required.
// It's also represented in Cube Coordinates
type fractionalHex struct {
	q float64
	r float64
	s float64
}

func NewFractionalHex(q, r float64) fractionalHex {

	h := fractionalHex{q: q, r: r, s: -q - r}
	return h

}

// Rounds a FractionalHex to a Regular Hex
func (h fractionalHex) Round() Hex {

	roundToInt := func(a float64) int {
		if a < 0 {
			return int(a - 0.5)
		}
		return int(a + 0.5)
	}

	q := roundToInt(h.q)
	r := roundToInt(h.r)
	s := roundToInt(h.s)

	q_diff := math.Abs(float64(q) - h.q)
	r_diff := math.Abs(float64(r) - h.r)
	s_diff := math.Abs(float64(s) - h.s)

	if q_diff > r_diff && q_diff > s_diff {
		q = -r - s
	} else if r_diff > s_diff {
		r = -q - s
	} else {
		s = -q - r
	}
	return Hex{q, r, s}

}

// Adds two hexagons
func HexAdd(a, b Hex) Hex {
	return NewHex(a.Q+b.Q, a.R+b.R)
}

// Subtracts two hexagons
func HexSubtract(a, b Hex) Hex {
	return NewHex(a.Q-b.Q, a.R-b.R)
}

// Scales an hexagon by a k factor. If factor k is 1 there's no change
func HexScale(a Hex, k int) Hex {
	return NewHex(a.Q*k, a.R*k)
}

func HexLength(Hex Hex) int {
	return int((math.Abs(float64(Hex.Q)) + math.Abs(float64(Hex.R)) + math.Abs(float64(Hex.S))) / 2.)
}

func HexDistance(a, b Hex) int {
	sub := HexSubtract(a, b)
	return HexLength(sub)
}

// Pass 0 for qOffset if you want the actual axis, or any other number to get the distance to that Q value
func GetDistanceToQAxis(h Hex, direction Direction, qOffset int) int {
	hexDir := directions[direction]
	if h.Q == qOffset {
		return 0
	}
	if hexDir.Q == 0 {
		panic("This direction is along the Q axis already, it will never intersect it")
	}
	return (qOffset - h.Q) / hexDir.Q
}

// Pass 0 for rOffset if you want the actual axis, or any other number to get the distance to that Q value
func GetDistanceToRAxis(h Hex, direction Direction, rOffset int) int {
	hexDir := directions[direction]
	if h.R == rOffset {
		return 0
	}
	if hexDir.R == 0 {
		panic("This direction is along the Q axis already, it will never intersect it")
	}
	return (rOffset - h.R) / hexDir.R
}

// Pass 0 for rOffset if you want the actual axis, or any other number to get the distance to that Q value
func GetDistanceToSAxis(h Hex, direction Direction, sOffset int) int {
	hexDir := directions[direction]
	if h.S == sOffset {
		return 0
	}
	if hexDir.S == 0 {
		panic("This direction is along the Q axis already, it will never intersect it")
	}
	return (sOffset - h.S) / hexDir.S
}

// Returns the neighbor hexagon at a certain Direction
func HexNeighbor(h Hex, direction Direction) Hex {
	directionOffset := directions[direction]
	return NewHex(h.Q+directionOffset.Q, h.R+directionOffset.R)
}

// Returns the neighbor hexagon a certain distance in a certain Direction
func HexNeighborOffset(h Hex, direction Direction, distance int) Hex {
	directionOffset := HexScale(directions[direction], distance)
	return NewHex(h.Q+directionOffset.Q, h.R+directionOffset.R)
}

// Returns the slice of hexagons that exist on a line that goes from hexagon a to hexagon b
func HexLineDraw(a, b Hex) []Hex {

	hexLerp := func(a fractionalHex, b fractionalHex, t float64) fractionalHex {
		return NewFractionalHex(a.q*(1-t)+b.q*t, a.r*(1-t)+b.r*t)
	}

	N := HexDistance(a, b)

	// Sometimes the hexLerp will output a Point that’s on an edge.
	// On some systems, the rounding code will push that to one side or the other,
	// somewhat unpredictably and inconsistently.
	// To make it always push these points in the same Direction, add an “epsilon” value to a.
	// This will “nudge” things in the same Direction when it’s on an edge, and leave other points unaffected.

	a_nudge := NewFractionalHex(float64(a.Q)+0.000001, float64(a.R)+0.000001)
	b_nudge := NewFractionalHex(float64(b.Q)+0.000001, float64(b.R)+0.000001)

	results := make([]Hex, 0)
	step := 1. / math.Max(float64(N), 1)

	for i := 0; i <= N; i++ {
		results = append(results, hexLerp(a_nudge, b_nudge, step*float64(i)).Round())
	}
	return results
}

// Returns the set of hexagons around a certain center for a given radius
func HexRange(center Hex, radius int) []Hex {

	var results = make([]Hex, 0)

	if radius >= 0 {
		for dx := -radius; dx <= radius; dx++ {

			for dy := math.Max(float64(-radius), float64(-dx-radius)); dy <= math.Min(float64(radius), float64(-dx+radius)); dy++ {
				results = append(results, HexAdd(center, NewHex(int(dx), int(dy))))
			}
		}
	}

	return results

}

// Returns the set of hexagons that form a rectangle with the specified width and height
func HexRectangleGrid(width, height int) []Hex {

	results := make([]Hex, 0)

	for q := 0; q < width; q++ {
		qOffset := int(math.Floor(float64(q) / 2.))

		for r := -qOffset; r < height-qOffset; r++ {

			results = append(results, NewHex(q, r))
		}
	}

	return results
}

// Determines if a given hexagon is visible from another hexagon, taking into consideration a set of blocking hexagons
func HexHasLineOfSight(center Hex, target Hex, blocking []Hex) bool {

	contains := func(s []Hex, e Hex) bool {
		for _, a := range s {
			if a == e {
				return true
			}
		}
		return false
	}

	for _, h := range HexLineDraw(center, target) {

		if contains(blocking, h) {
			return false
		}
	}

	return true
}

// Returns the list of hexagons that are visible from a given hexagon
func HexFieldOfView(source Hex, candidates []Hex, blocking []Hex) []Hex {

	results := make([]Hex, 0)

	for _, h := range candidates {

		distance := HexDistance(source, h)

		if len(blocking) == 0 || distance <= 1 || HexHasLineOfSight(source, h, blocking) {
			results = append(results, h)
		}
	}

	return results
}
