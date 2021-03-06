package hexgrid

import (
	"fmt"
	"testing"
)

func TestHexAdd(t *testing.T) {

	var testCases = []struct {
		hexA     Hex
		hexB     Hex
		expected Hex
	}{
		{NewHex(1, -3), NewHex(3, -7), NewHex(4, -10)},
	}

	for _, tt := range testCases {

		actual := HexAdd(tt.hexA, tt.hexB)

		if actual != tt.expected {
			t.Error("Expected:", tt.expected, "got:", actual)
		}
	}
}

func TestHexSubtract(t *testing.T) {

	var testCases = []struct {
		hexA     Hex
		hexB     Hex
		expected Hex
	}{
		{NewHex(1, -3), NewHex(3, -7), NewHex(-2, 4)},
	}

	for _, tt := range testCases {

		actual := HexSubtract(tt.hexA, tt.hexB)

		if actual != tt.expected {
			t.Error("Expected:", tt.expected, "got:", actual)
		}
	}
}

func TestHexScale(t *testing.T) {

	var testCases = []struct {
		hexA     Hex
		factor   int
		expected Hex
	}{
		{NewHex(1, -3), 2, NewHex(2, -6)},
		{NewHex(-2, 3), 2, NewHex(-4, 6)},
	}

	for _, tt := range testCases {

		actual := HexScale(tt.hexA, tt.factor)

		if actual != tt.expected {
			t.Error("Expected:", tt.expected, "got:", actual)
		}
	}

}

func TestDirection_ToString(t *testing.T) {
	var testCases = []struct {
		direction Direction
		expected  string
	}{
		{DirectionN, "N"},
		{DirectionNE, "NE"},
		{DirectionSE, "SE"},
		{DirectionS, "S"},
		{DirectionSW, "SW"},
		{DirectionNW, "NW"},
	}

	for _, tt := range testCases {

		actual := tt.direction.ToString()
		if actual != tt.expected {
			t.Error("Expected:", tt.expected, "got:", actual)
		}
	}
}

//           _ _
//         /     \
//    _ _ /(0,-2) \ _ _
//  /     \       /     \
// /(-1,-1)\ _ _ /(1,-2) \
// \       /     \       /
//  \ _ _ /(0,-1) \ _ _ /
//  /     \       /     \
// /(-1,0) \ _ _ /(1,-1) \
// \       /     \       /
//  \ _ _ / (0,0) \ _ _ /
//        \       /
//         \ _ _ /
// Tests that the neighbors of a certain hexagon are properly computed for all Directions
func TestHexNeighbor(t *testing.T) {

	var testCases = []struct {
		origin    Hex
		direction Direction
		expected  Hex
	}{

		{NewHex(0, -1), DirectionSE, NewHex(1, -1)},
		{NewHex(0, -1), DirectionNE, NewHex(1, -2)},
		{NewHex(0, -1), DirectionN, NewHex(0, -2)},
		{NewHex(0, -1), DirectionNW, NewHex(-1, -1)},
		{NewHex(0, -1), DirectionSW, NewHex(-1, 0)},
		{NewHex(0, -1), DirectionS, NewHex(0, 0)},
	}

	for _, tt := range testCases {

		actual := HexNeighbor(tt.origin, tt.direction)

		if actual != tt.expected {
			t.Error("Expected:", tt.expected, "got:", actual)
		}
	}
}

// Tests that the offset neighbors of a certain hexagon are properly computed for all Directions
func TestHexNeighborOffset(t *testing.T) {

	var testCases = []struct {
		origin    Hex
		direction Direction
		distance  int
		expected  Hex
	}{

		{NewHex(0, -1), DirectionSE, 0, NewHex(0, -1)},
		{NewHex(0, -1), DirectionSE, 1, NewHex(1, -1)},
		{NewHex(0, -1), DirectionNE, 2, NewHex(2, -3)},
		{NewHex(0, -1), DirectionN, 3, NewHex(0, -4)},
		{NewHex(0, -1), DirectionNW, 4, NewHex(-4, -1)},
		{NewHex(0, -1), DirectionSW, 5, NewHex(-5, 4)},
		{NewHex(0, -1), DirectionS, 200, NewHex(0, 199)},
	}

	for _, tt := range testCases {

		actual := HexNeighborOffset(tt.origin, tt.direction, tt.distance)

		if actual != tt.expected {
			t.Error("Expected:", tt.expected, "got:", actual)
		}
	}
}

// DISTANCE TESTS

//           _ _
//         /     \
//    _ _ /(0,-2) \ _ _
//  /     \       /     \
// /(-1,-1)\ _ _ /(1,-2) \
// \       /     \       /
//  \ _ _ /(0,-1) \ _ _ /
//  /     \       /     \
// /(-1,0) \ _ _ /(1,-1) \
// \       /     \       /
//  \ _ _ / (0,0) \ _ _ /
//  /     \       /     \
// /(-1,1) \ _ _ / (1,0) \
// \       /     \       /
//  \ _ _ / (0,1) \ _ _ /
//        \       /
//         \ _ _ /

func TestHexDistance(t *testing.T) {

	var testCases = []struct {
		origin      Hex
		destination Hex
		expected    int
	}{
		{NewHex(-1, -1), NewHex(1, -1), 2},
		{NewHex(-1, -1), NewHex(0, 0), 2},
		{NewHex(0, -1), NewHex(0, -2), 1},
		{NewHex(-1, -1), NewHex(0, 1), 3},
		{NewHex(1, 0), NewHex(-1, -1), 3},
	}

	for _, tt := range testCases {

		actual := HexDistance(tt.origin, tt.destination)

		if actual != tt.expected {
			t.Error("Expected:", tt.expected, "got:", actual)
		}
	}
}

func TestGetDistanceToQAxis(t *testing.T) {

	var testCases = []struct {
		origin      Hex
		direction	Direction
		qOffset		int
		expectedDist    int
		expectedDoesTouch bool
	}{
		{NewHex(-3, +1), DirectionN, 0, 0, false},
		{NewHex(-3, +1), DirectionNE, 0, 3, true},
		{NewHex(-3, +1), DirectionSE, 0, 3, true},
		{NewHex(-3, +1), DirectionS, 0, 0, false},
		{NewHex(-3, +1), DirectionSW, 0, -3, true},
		{NewHex(-3, +1), DirectionNW, 0, -3, true},
		// would be false but it's already on the axis
		{NewHex(0, +1), DirectionS, 0, 0, true},
	}

	for _, tt := range testCases {

		actualDist, actualDoesTouch := GetDistanceToQAxis(tt.origin, tt.direction, tt.qOffset)

		if actualDist != tt.expectedDist {
			t.Error("Expected:", tt.expectedDist, "got:", actualDist)
		}
		if actualDoesTouch != tt.expectedDoesTouch {
			t.Error("Expected:", tt.expectedDoesTouch, "got:", actualDoesTouch)
		}
	}
}

func TestGetDistanceToRAxis(t *testing.T) {

	var testCases = []struct {
		origin      Hex
		direction	Direction
		rOffset		int
		expectedDist    int
		expectedDoesTouch bool
	}{
		{NewHex(-2, -1), DirectionN, 3, -4, true},
		{NewHex(-2, -1), DirectionNE, 3, -4, true},
		{NewHex(-2, -1), DirectionSE, 3, 0, false},
		{NewHex(-2, -1), DirectionS, 3, 4, true},
		{NewHex(-2, -1), DirectionSW, 3, 4, true},
		{NewHex(-2, -1), DirectionNW, 3, 0, false},
		// would be false but it's already on the axis
		{NewHex(-2, 3), DirectionNW, 3, 0, true},
	}

	for _, tt := range testCases {

		actualDist, actualDoesTouch := GetDistanceToRAxis(tt.origin, tt.direction, tt.rOffset)

		if actualDist != tt.expectedDist {
			t.Error("Expected:", tt.expectedDist, "got:", actualDist)
		}
		if actualDoesTouch != tt.expectedDoesTouch {
			t.Error("Expected:", tt.expectedDoesTouch, "got:", actualDoesTouch)
		}
	}
}

func TestGetDistanceToSAxis(t *testing.T) {

	var testCases = []struct {
		origin      Hex
		direction	Direction
		sOffset		int
		expectedDist    int
		expectedDoesTouch bool
	}{
		{NewHex(1, -1), DirectionN, -2, -2, true},
		{NewHex(1, -1), DirectionNE, -2, 0, false},
		{NewHex(1, -1), DirectionSE, -2, 2, true},
		{NewHex(1, -1), DirectionS, -2, 2, true},
		{NewHex(1, -1), DirectionSW, -2, 0, false},
		{NewHex(1, -1), DirectionNW, -2, -2, true},
		// would be false but it's already on the axis
		{NewHex(3, -1), DirectionSW, -2, 0, true},
	}

	for _, tt := range testCases {

		actualDist, actualDoesTouch := GetDistanceToSAxis(tt.origin, tt.direction, tt.sOffset)

		if actualDist != tt.expectedDist {
			t.Error("Expected:", tt.expectedDist, "got:", actualDist)
		}
		if actualDoesTouch != tt.expectedDoesTouch {
			t.Error("Expected:", tt.expectedDoesTouch, "got:", actualDoesTouch)
		}
	}
}

//          _____         _____         _____
//         /     \       /     \       /     \
//   _____/ -2,-2 \_____/  0,-3 \_____/  2,-4 \_____
//  /     \       /     \       /     \       /     \
// / -3,-1 \_____/ -1,-2 \_____/  1,-3 \_____/  3,-4 \
// \       /     \       /     \       /     \       /
//  \_____/ -2,-1 \_____/  0,-2 \_____/  2,-3 \_____/
//  /     \       /     \       /     \       /     \
// / -3,0  \_____/ -1,-1 \_____/  1,-2 \_____/  3,-3 \
// \       /     \       /     \       /     \       /
//  \_____/ -2,0  \_____/  0,-1 \_____/  2,-2 \_____/
//  /     \       /     \       /     \       /     \
// / -3,1  \_____/ -1,0  \_____/  1,-1 \_____/  3,-2 \
// \       /     \       /     \       /     \       /
//  \_____/       \_____/       \_____/       \_____/
func TestHexLineDraw(t *testing.T) {

	var testCases = []struct {
		origin      Hex
		destination Hex
		expected    string // the expected path serialized to string
	}{
		{NewHex(-3, -1), NewHex(3, -3), "[(-3,-1) (-2,-1) (-1,-2) (0,-2) (1,-2) (2,-3) (3,-3)]"},
		{NewHex(-2, 0), NewHex(2, -2), "[(-2,0) (-1,0) (0,-1) (1,-1) (2,-2)]"},
		{NewHex(1, -1), NewHex(1, -3), "[(1,-1) (1,-2) (1,-3)]"},
	}

	for _, tt := range testCases {

		actual := fmt.Sprint(HexLineDraw(tt.origin, tt.destination))

		if actual != tt.expected {
			t.Error("Expected:", tt.expected, "got:", actual)
		}
	}
}

// Tests that the range includes the correct number of hexagons with a certain radius from the center
//                 _____
//                /     \
//          _____/ -1,-2 \_____
//         /     \       /     \
//   _____/ -2,-1 \_____/  0,-2 \_____
//  /     \       /     \       /     \
// / -3,-1 \_____/ -1,-2 \_____/  1,-3 \
// \       /     \       /     \       /
//  \_____/ -2,-2 \_____/  0,-3 \_____/
//  /     \       /     \       /     \
// / -3,-1 \_____/ -1,-2 \_____/  1,-3 \
// \       /     \ CENTR /     \       /
//  \_____/ -2,-1 \_____/  0,-2 \_____/
//  /     \       /     \       /     \
// / -3,0  \_____/ -1,-1 \_____/  1,-2 \
// \       /     \       /     \       /
//  \_____/ -2,0  \_____/  0,-1 \_____/
//        \       /     \       /
//         \_____/ -1,0  \_____/
//               \       /
//                \_____/
func TestHexRange(t *testing.T) {

	var testCases = []struct {
		radius                   int
		expectedNumberOfHexagons int
	}{
		{0, 1},
		{1, 7},
		{2, 19},
	}

	for _, tt := range testCases {

		actual := HexRange(NewHex(1, -2), tt.radius)

		if len(actual) != tt.expectedNumberOfHexagons {
			t.Error("Expected:", tt.expectedNumberOfHexagons, "got:", len(actual))
		}
	}
}

//    _ _           _ _
//  /     \       /     \
// /( 0,0) \ _ _ /(2,-1) \
// \       /     \       /
//  \ _ _ / (1,0) \ _ _ /
//  /     \       /     \
// / (0,1) \ _ _ / (2,0) \
// \       /     \       /
//  \ _ _ / (1,1) \ _ _ /
//        \       /
//         \ _ _ /
func TestHexRectangle(t *testing.T) {

	hexgrid := HexRectangleGrid(3, 2)

	if len(hexgrid) != 6 {
		t.Error("Expected: 6 got:", len(hexgrid))
	}

}

//    _ _           _ _           _ _
//  /     \       /     \       /     \
// /  0 0  \ _ _ /  2-1  \ _ _ /  4-2  \ _ _
// \       /     \   X   /     \   X   /     \
//  \ _ _ /  1 0  \ _ _ /  3-1  \ _ _ /  5-2  \
//  /     \       /# # #\   X   /     \   X   /
// /  0 1  \ _ _ /# 2 0 #\ _ _ /  4-1  \ _ _ /
// \       /     \#     #/# # #\   X   /     \
//  \ _ _ /  1 1  \#_#_#/# 3 0 #\ _ _ /  5-1  \
//  /     \  |P|  /     \#  X  #/     \   X   /
// /  0 2  \ _ _ /  2 1  \#_#_#/  4 0  \ _ _ /
// \       /     \       /     \   X   /     \
//  \ _ _ /  1 2  \ _ _ /  3 1  \ _ _ /  5 0  \
//  /     \       /     \       /     \       /
// /  0 3  \ _ _ /  2 2  \ _ _ /  4 1  \ _ _ /
// \       /     \       /     \       /     \
//  \ _ _ /  1 3  \ _ _ /  3 2  \ _ _ /  5 1  \
//        \       /     \       /     \       /
//         \ _ _ /       \ _ _ /       \ _ _ /
//
// The FOV measured from the central Hex at 1,1, assuming blocking hexagons at 2,0 and 3,0.
// The hexagons marked with an X are non-visible. The remaining 16 are visible.
func TestHexFieldOfView(t *testing.T) {

	universe := []Hex{
		NewHex(0, 0),
		NewHex(0, 1),
		NewHex(0, 2),
		NewHex(0, 3),
		NewHex(1, 0),
		NewHex(1, 1),
		NewHex(1, 2),
		NewHex(1, 3),
		NewHex(2, -1),
		NewHex(2, 0),
		NewHex(2, 1),
		NewHex(2, 2),
		NewHex(3, -1),
		NewHex(3, 0),
		NewHex(3, 1),
		NewHex(3, 2),
		NewHex(4, -2),
		NewHex(4, -1),
		NewHex(4, 0),
		NewHex(4, 1),
		NewHex(5, -2),
		NewHex(5, -1),
		NewHex(5, 0),
		NewHex(5, 1),
	}

	losBlockers := []Hex{NewHex(2, 0), NewHex(3, 0)}

	actual := HexFieldOfView(NewHex(1, 1), universe, losBlockers)

	if len(actual) != 16 {
		t.Error("Expected: 16 got:", len(actual))
	}
}

////////////////
// Benchmarks //
////////////////

func BenchmarkHexDistance(b *testing.B) {

	var testCases = []struct {
		destination Hex
	}{
		{NewHex(0, 0)},
		{NewHex(100, 100)},
		{NewHex(10000, 10000)},
	}

	for _, bm := range testCases {

		origin := NewHex(0, 0)

		b.Run(fmt.Sprint(origin, ":", bm.destination), func(b *testing.B) {
			for i := 0; i < b.N; i++ {
				HexDistance(origin, bm.destination)
			}
		})
	}
}

func BenchmarkHexLineDraw(b *testing.B) {

	var testCases = []struct {
		destination Hex
	}{
		{NewHex(0, 0)},
		{NewHex(100, 100)},
		{NewHex(10000, 10000)},
	}

	for _, bm := range testCases {

		origin := NewHex(0, 0)

		b.Run(fmt.Sprint(origin, ":", bm.destination), func(b *testing.B) {
			for i := 0; i < b.N; i++ {
				HexLineDraw(origin, bm.destination)
			}
		})
	}
}

func BenchmarkHexRange(b *testing.B) {

	var testCases = []struct {
		radius int
	}{
		{0},
		{10},
		{100},
	}

	for _, bm := range testCases {

		origin := NewHex(0, 0)

		b.Run(fmt.Sprint(origin, ":", bm.radius), func(b *testing.B) {
			for i := 0; i < b.N; i++ {
				HexRange(NewHex(1, -2), bm.radius)
			}
		})
	}
}

func BenchmarkHexHasLineOfSight(b *testing.B) {

	for i := 0; i < b.N; i++ {
		HexHasLineOfSight(NewHex(1, 1), NewHex(4, -1), []Hex{NewHex(2, 0), NewHex(3, 0)})
	}
}
