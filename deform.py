from pyadjoint import Block, annotate_tape
from pyadjoint.overloaded_type import create_overloaded_object
from firedrake import *

class DeformMeshBlock(Block):
    def __init__(self, mesh, control, result):
        super().__init__()
        self.mesh = mesh
        self.control = control
        self.result = result
        self.add_dependency(control)

    def recompute_component(self, inputs, index):
        mesh, control = self.mesh, inputs[0]
        return self._deform_mesh(mesh, control)

    def _deform_mesh(self, mesh, control):
        X = SpatialCoordinate(mesh)
        V = control.function_space()
        coords = Function(V)
        coords.interpolate(as_vector(X + control))
        return Mesh(coords)

    def evaluate_adj_component(self, inputs, adj_output, index):
        V = self.control.function_space()
        zero = Function(V)
        zero.assign(0)
        return [zero]

def deform(mesh, control):
    V = control.function_space()
    X = SpatialCoordinate(V.mesh())  # ← Usamos el dominio de control
    coords = Function(V)
    coords.interpolate(as_vector(X + control))
    new_mesh = Mesh(coords)

    if annotate_tape():
        block = DeformMeshBlock(mesh, control, new_mesh)
        overloaded = create_overloaded_object(new_mesh)
        overloaded.block = block
        return overloaded
    else:
        return new_mesh