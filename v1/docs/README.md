# Walkthrough

Six documents, one per stage of a run, in the order the arm does them.

They exist because most of the work in this project is invisible.
`find_cuboids()` turns 25,000 points into three measured boxes in a handful of
lines, and reading those lines tells you which functions were called — not why
1.2 cm is the right size of cube to group points with, or how a rotated
footprint falls out of the points that make it up. These documents take one
stage at a time and show the numbers going through it, with the project's own
values, so that the code afterwards reads as a record of a decision rather than
a puzzle.

The pictures are drawn by a script rather than captured from Gazebo — a real
frame has too much in it to point at — but the numbers beside them come from
the project's own functions run on those pictures.

- [**Step 1 — taking pictures**](step1-taking-pictures.md). Where the three
  viewpoints are, how the arm has to be posed to get the camera to them, and
  why one picture from above is not enough.
- [**Step 2 — pictures to points**](step2-pictures-to-points.md). The test that
  separates a coloured box from a grey table, and the arithmetic that takes a
  pixel with a depth reading to a point in the room.
- [**Step 3 — points to boxes**](step3-points-to-boxes.md). Splitting one pile
  of points into one lump per box without comparing every point with every
  other, then fitting a size and an angle to a lump.
- [**Step 4 — pick and place**](step4-pick-and-place.md). Which box goes first,
  where it goes, which way the gripper has to be turned to fit round it, and
  what MoveIt is handed at each move.
- [**Step 5 — measure again**](step5-measure-again.md). The close-up second
  look: steps 1 to 3 over again with different numbers, and how the arm decides
  which of the boxes it can see is the one it just put down.
- [**Step 6 — choose a face and touch it**](step6-choose-face-and-touch.md).
  Six faces from the five numbers step 5 measured, which of them wins, and the
  approach that ends with the fingertip against it.

For the reasoning behind the choices these steps make, read
[`../IMPLEMENTATION_NOTES.md`](../IMPLEMENTATION_NOTES.md). For which file and
function each stage lives in, read [`../PSEUDOCODE.md`](../PSEUDOCODE.md).
