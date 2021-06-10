import numpy as np
import pydrake.all
from matplotlib.colors import Colormap

import meshcat.geometry as g
import meshcat.transformations as tf

from pydrake.all import RigidTransform


# TODO: Remove this once https://github.com/rdeits/meshcat-python/pull/91 makes
# its way through to drake.
class TriangularMeshGeometry(g.Geometry):
    """
    A mesh consisting of an arbitrary collection of triangular faces. To
    construct one, you need to pass in a collection of vertices as an Nx3 array
    and a collection of faces as an Mx3 array. Each element of `faces` should
    be a collection of 3 indices into the `vertices` array.

    For example, to create a square made out of two adjacent triangles, we
    could do:

    vertices = np.array([
        [0, 0, 0],  # the first vertex is at [0, 0, 0]
        [1, 0, 0],
        [1, 0, 1],
        [0, 0, 1]
    ])
    faces = np.array([
        [0, 1, 2],  # The first face consists of vertices 0, 1, and 2
        [3, 0, 2]
    ])

    mesh = TriangularMeshGeometry(vertices, faces)

    To set the color of the mesh by vertex, pass an Nx3 array containing the
    RGB values (in range [0,1]) of the vertices to the optional `color`
    argument, and set `vertexColors=True` in the Material.
    """
    __slots__ = ["vertices", "faces"]

    def __init__(self, vertices, faces, color=None):
        super(TriangularMeshGeometry, self).__init__()

        vertices = np.asarray(vertices, dtype=np.float32)
        faces = np.asarray(faces, dtype=np.uint32)
        assert vertices.shape[1] == 3, "`vertices` must be an Nx3 array"
        assert faces.shape[1] == 3, "`faces` must be an Mx3 array"
        self.vertices = vertices
        self.faces = faces
        if color is not None:
            color = np.asarray(color, dtype=np.float32)
            assert np.array_equal(vertices.shape, color.shape),\
                "`color` must be the same shape as vertices"
        self.color = color

    def lower(self, object_data):
        attrs = {u"position": g.pack_numpy_array(self.vertices.T)}
        if self.color is not None:
            attrs[u"color"] = g.pack_numpy_array(self.color.T)
        return {
            u"uuid": self.uuid,
            u"type": u"BufferGeometry",
            u"data": {
                u"attributes": attrs,
                u"index": g.pack_numpy_array(self.faces.T)
            }
        }


def plot_surface(meshcat, X, Y, Z, color=0xdd9999, wireframe=False):
    (rows, cols) = Z.shape

    vertices = np.empty((rows * cols, 3), dtype=np.float32)
    vertices[:, 0] = X.reshape((-1))
    vertices[:, 1] = Y.reshape((-1))
    vertices[:, 2] = Z.reshape((-1))

    # Vectorized faces code from https://stackoverflow.com/questions/44934631/making-grid-triangular-mesh-quickly-with-numpy  # noqa
    faces = np.empty((rows - 1, cols - 1, 2, 3), dtype=np.uint32)
    r = np.arange(rows * cols).reshape(rows, cols)
    faces[:, :, 0, 0] = r[:-1, :-1]
    faces[:, :, 1, 0] = r[:-1, 1:]
    faces[:, :, 0, 1] = r[:-1, 1:]
    faces[:, :, 1, 1] = r[1:, 1:]
    faces[:, :, :, 2] = r[1:, :-1, None]
    faces.shape = (-1, 3)

    if isinstance(color, Colormap):
        z = vertices[:, 2]
        rgba = color((vertices[:, 2] - np.min(z)) / np.ptp(z))
        color = rgba[:, :3]

    if isinstance(color, int):
        meshcat.set_object(
            g.TriangularMeshGeometry(vertices, faces),
            g.MeshLambertMaterial(color=color, wireframe=wireframe))
    else:
        meshcat.set_object(
            TriangularMeshGeometry(vertices, faces, color),
            g.MeshLambertMaterial(vertexColors=True, wireframe=wireframe))


def plot_mathematical_program(meshcat,
                              prog,
                              X,
                              Y,
                              result=None,
                              point_size=0.05):
    assert prog.num_vars() == 2
    assert X.size == Y.size

    N = X.size
    values = np.vstack((X.reshape(-1), Y.reshape(-1)))
    costs = prog.GetAllCosts()

    # Vectorized multiply for the quadratic form.
    # Z = (D*np.matmul(Q,D)).sum(0).reshape(nx, ny)

    if costs:
        Z = prog.EvalBindingVectorized(costs[0], values)
        for b in costs[1:]:
            Z = Z + prog.EvalBindingVectorized(b, values)

    cv = meshcat["constraints"]
    for binding in prog.GetAllConstraints():
        if isinstance(
                binding.evaluator(),
                pydrake.solvers.mathematicalprogram.BoundingBoxConstraint):
            c = binding.evaluator()
            var_indices = [
                int(prog.decision_variable_index()[v.get_id()])
                for v in binding.variables()
            ]
            satisfied = np.array(
                c.CheckSatisfiedVectorized(values[var_indices, :],
                                           0.001)).reshape(1, -1)
            if costs:
                Z[~satisfied] = np.nan

            v = cv[type(c).__name__]
            Zc = np.zeros(Z.shape)
            Zc[satisfied] = np.nan
            plot_surface(v,
                         X,
                         Y,
                         Zc.reshape((X.shape[1], X.shape[0])),
                         color=0xff3333,
                         wireframe=True)
        else:
            Zc = prog.EvalBindingVectorized(binding, values)
            evaluator = binding.evaluator()
            low = evaluator.lower_bound()
            up = evaluator.upper_bound()
            cvb = cv[type(evaluator).__name__]
            for index in range(Zc.shape[0]):
                color = np.repeat([[0.3, 0.3, 1.0]], N, axis=0)
                infeasible = np.logical_or(Zc[index, :] < low[index],
                                           Zc[index, :] > up[index])
                color[infeasible, 0] = 1.0
                color[infeasible, 2] = 0.3
                plot_surface(cvb[str(index)],
                             X,
                             Y,
                             Zc[index, :].reshape(X.shape[1], X.shape[0]),
                             color=color,
                             wireframe=True)

    if costs:
        plot_surface(meshcat["objective"],
                     X,
                     Y,
                     Z.reshape(X.shape[1], X.shape[0]),
                     color=0x77cc77,
                     wireframe=True)

    if result:
        v = meshcat["solution"]
        v.set_object(g.Sphere(point_size),
                     g.MeshLambertMaterial(color=0x55ff55))
        x_solution = result.get_x_val()
        v.set_transform(
            tf.translation_matrix(
                [x_solution[0], x_solution[1],
                 result.get_optimal_cost()]))


# pulled from inside MeshcatVisualizer.
def set_planar_viewpoint(vis,
                         camera_position=[0, -1, 0],
                         camera_focus=[0, 0, 0],
                         xmin=-1,
                         xmax=1,
                         ymin=-1,
                         ymax=1):
    # TODO(russt): Figure out the proper set of camera transformations to
    # implement camera_focus.
    if np.any(camera_focus):
        warnings.warn("Non-zero camera_focus is not supported yet")

    # Set up orthographic camera.
    camera = g.OrthographicCamera(left=xmin,
                                  right=xmax,
                                  top=ymax,
                                  bottom=ymin,
                                  near=-1000,
                                  far=1000)
    vis['/Cameras/default/rotated'].set_object(camera)
    vis['/Cameras/default'].set_transform(
        RigidTransform(camera_position).GetAsMatrix4())

    # Lock the orbit controls.
    vis['/Cameras/default/rotated/<object>'].set_property("position", [0, 0, 0])

    # Turn off background, axes, and grid.
    vis['/Background'].set_property("visible", False)
    vis['/Grid'].set_property("visible", False)
    vis['/Axes'].set_property("visible", False)


def draw_points(meshcat, points, color, **kwargs):
    """Helper for sending a 3xN points of a single color to MeshCat"""
    points = np.asarray(points)
    assert points.shape[0] == 3
    if points.size == 3:
        points.shape = (3, 1)
    colors = np.tile(np.asarray(color).reshape(3, 1), (1, points.shape[1]))
    meshcat.set_object(g.PointCloud(points, colors, **kwargs))
