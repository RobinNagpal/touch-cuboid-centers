# v3: the table top starts leaning on a wall

This folder is plans only. There is no code in it yet. The working build is
[`../v2`](../v2), where the top starts lying flat on two stands.

## The problem

The room is the same as in v2:

- a UR5e arm standing on the floor, with a camera on its wrist and a
  two-finger gripper;
- four table legs;
- one table top, a thin board.

One thing changes. **The table top starts leaning against a low wall**, almost
upright. It stands on one long edge on the floor, and leans back 15 to 22
degrees onto the wall's top corner.

The arm has to:

1. find everything with its camera, and measure the top and the legs;
2. work out where the legs go for a top that size;
3. stand the four legs there (as v2 already does);
4. pick the top up off the wall;
5. get it from upright to flat, without dropping it and without knocking a
   leg over;
6. lay it on the legs, and check the table.

Step 5 is the hard part, and it is why v2 starts the top flat on two stands.

The rule from v2 still holds. The robot is told nothing about the room. It
knows only itself: where its base is and how its gripper and camera are
built. Everything else, including how heavy the top is, it has to measure.

## What each file answers

| File | Question |
| --- | --- |
| [`TOP_FROM_WALL.md`](TOP_FROM_WALL.md) | What are the ways to turn the top flat? Way A in detail: grip the upper edge and swing it flat in the air. |
| [`SWING_PHYSICS.md`](SWING_PHYSICS.md) | During that swing, does only the wrist move or the whole arm? How heavy a top can it handle, and how is the weight set in Gazebo? |
| [`TILT_ON_LEGS.md`](TILT_ON_LEGS.md) | The way a person would do it: rest one edge on two legs, then tilt the top down. How does it work, and what can go wrong? |

Short answer: the swing is simple but only works for light tops, about
0.8 kg with v2's gripper. Resting on the legs handles tops several times
heavier, but the legs can be knocked over, so it needs much more care.

## The pictures

The pictures are in [`figures/`](figures). Each script there draws the
pictures for one file:

| Script | Draws the pictures for |
| --- | --- |
| `figures/top_from_wall.py` | `TOP_FROM_WALL.md` (and holds the drawing helpers the other two use) |
| `figures/swing_physics.py` | `SWING_PHYSICS.md` |
| `figures/tilt_on_legs.py` | `TILT_ON_LEGS.md` |

To redraw them, use any Python with matplotlib and numpy. v2's environment
has both:

```
cd v3/figures
../../v2/.pixi/envs/default/bin/python top_from_wall.py
../../v2/.pixi/envs/default/bin/python swing_physics.py
../../v2/.pixi/envs/default/bin/python tilt_on_legs.py
```

They are sketches to explain the idea, not drawings to scale. The two charts
use v2's real numbers.
