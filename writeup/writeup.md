## Part 1

De Casteljau's algorithm is a simple recursive method for evaluating Bezier
curves. Given a Bezier curve with $N$ control points and some desired parameter
$t \in [0, 1]$ on the curve, we can map each "segment" (i.e. pair of consecutive
control points) into a new point by traveling $t$ of the way from the first
control point to the second. This can be easily computed as a weighted average,
and leaves us with a set of $N-1$ points. It turns out that these points form a
new Bezier curve that coincides exactly with the old one, so we can simply
repeat the process until we have only one point left. This point is the value of
the Bezier curve at the parameter $t$.

We picked a Bezier curve of six points. Shown is a progression of the algorithm:


| a | b | c |
| --- | ------------------- | ------------------- |
| ![](images/task1-0.png) | ![](images/task1-1.png) | ![](images/task1-2.png) |
| ![](images/task1-3.png) | ![](images/task1-4.png) | ![](images/task1-5.png) |

![](images/task1-c.png)

Here's a slightly different curve evaluated at a different parameter:

![](images/task1-other.png)

## Part 2

Bezier surfaces are specified by a grid of N by M control points. The de
Casteljau algorithm can be extended to evaluate not only Bezier curves, but
Bezier surfaces as well. By applying the algorithm to each row, we obtain a new
set of control points. We can then apply the algorithm to these new control
points to obtain the value of the Bezier surface at the desired parameter. Turns
out, the algorithm works the same whether we evaluate rows first or columns
first.

Here is a picture of the teapot being rendered:

![](images/task2-empty.png)

![](images/task2-filled.png)

## Part 3

Implementing the area-weighted vertex normals was fairly straightforward, since
`Face::normal()` was already provided to us. The most complex thing we had to do
was to figure out how to compute the area of a triangle. We used the formula
$A = \frac{\lVert \mathbf{AB} \times \mathbf{AC} \rVert}{2}$, which we
implemented in C++ using the provided `cross` and `norm` utilities.

From there, we iterated through each edge connected to the vertex by repeatedly
taking `twin()->next()`, multiplying the result of our area utility function as
well as the provided `Face::normal()` function, and summing the results. We then
normalized the sum and returned it.

Here is a picture of the teapot without the normals used:

![](images/task3-without.png)

And here is a picture of the teapot with the normals used:

![](images/task3-with.png)

## Part 4

Implementing the edge flip was a bit more challenging. At first, it seemed like
it would be a lot of work to keep track of all the pointers and make sure
everything was consistent. However, after rethinking about the problem in terms
of "what are the new triangles", we settled on the following:

![](images/task4-diagram.jpg)

For a given edge `ha` and its twin `hb` shown above (edges labeled at their
starting vertex), if we shorten `->next()` to `.N`, the new triangles should be
the following:

- `ha` → `hb.NN` → `ha.N`
- `hb` → `ha.NN` → `hb.N`

We can set the `next` and `face` pointers accordingly. Furthermore, the vertex
of `ha` should be updated to that of `ha.N.N` and the vertex of `hb` should be
updated to that of `hb.N.N`.

Lastly, we had to make sure that all the pointers pointing back to each halfedge
were updated as well. This was a bit tricky to approach at first, but we avoided
having to think too hard about exactly what changed by simply updating every
single pointer.

Here's some screenshots of edge flips!

| 0 | 1 | 2 |
| --- | --- | --- |
| ![](images/task4-0.png) | ![](images/task4-1.png) | ![](images/task4-2.png) |

With careful planning, a potentially frustrating debugging journey was
thankfully avoided 🙂

## Part 5

Once again, by drawing out a diagram and breaking down the steps beforehand,
implementing edge split was fairly straightforward.
We call the halfedges associated with the edge to be split `ha` and `hb`. A new vertex is
created at the midpoint of the edge being split, and the vertices and new faces created by this
operation are assigned.

EC: We also implemented edge split for boundary edges.
If both half-edges of the edge are boundary edges, then there is no splitting to be done.
In the case that one of `ha` or `hb` is a boundary, then we add a new vertex and create only
one new triangle face instead of two additional faces from having vertices on either side.

|      | Before split | After split | 
| ---- | ------------------- | ------------------- |
| Sketch | ![](images/task5-before-sketch.png) | ![](images/task5-after-sketch.png) | 
| Simple mesh| ![](images/task5-before.png) | ![](images/task5-after.png) | 
| Splits and flips | ![](images/task5-split-flip-before.png) | ![](images/task5-split-flip-after.png) | 
| EC: Boundary split | ![](images/task5-ec-before.png) | ![](images/task5-ec-after.png) | 


## Part 6

We followed the suggested steps of first computing all the new positions for
vertices and edges and storing them in the `newPosition` variable. Next to split
the edges, we create reference to all the original edges to ensure we don't loop
indefinitely, and `splitEdge()` these edges. We had to modify `splitEdge()` slightly
since the original implementation was correct but inconsistent since it moved the
original edges, so we had to swap the original edges back to their positions.
This was a tricky one to debug but we were able to observe what was happening by
viewing the teapot mesh which has more regular/flat edges to begin with.
Then, we flip the edges connecting an old and new vertex, and finally copy the
vertices' stored `newPosition`s into `position`

The loop subdivision on its own can still result in some "sharp" corners if the original mesh had
a sharp corner or vertices with a high index. The provided `cube.dae` for example does not have
regular vertex degrees, resulting in a slightly pointy pillow-like cuboid (left).
If we first manually preprocess the cube and regularize vertex degrees by flipping edges, then
we can obtain a rounder shape.

EC: We also implemented boundary checks such that boundary edges are preserved and not moved or smoothed.
See beetle below. Boundary vertices do not move, and boundary edges are split in the middle instead of using
the Loop formula.

| Cube | Preprocessed cube | Beetle boundary case|
| --- | ------------------- | ------------------- |
| ![](images/task6-pillow1.png) | ![](images/task6-reg1.png) | ![](images/task6-beetle1.png) |
| ![](images/task6-pillow2.png) | ![](images/task6-reg2.png) | ![](images/task6-beetle2.png) |
| ![](images/task6-pillow3.png) | ![](images/task6-reg3.png) | ![](images/task6-beetle3.png) |
| ![](images/task6-pillow4.png) | ![](images/task6-reg4.png) | ![](images/task6-beetle4.png) |
| ![](images/task6-pillow5.png) | ![](images/task6-reg5.png) | |
| ![](images/task6-pillow6.png) | ![](images/task6-reg6.png) | |
