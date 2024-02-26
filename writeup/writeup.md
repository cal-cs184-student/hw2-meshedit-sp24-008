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

![](images/task1-0.png)

![](images/task1-1.png)

![](images/task1-2.png)

![](images/task1-3.png)

![](images/task1-4.png)

![](images/task1-5.png)

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

![](images/task4-0.png)

![](images/task4-1.png)

![](images/task4-2.png)

With careful planning, a potentially frustrating debugging journey was
thankfully avoided 🙂
