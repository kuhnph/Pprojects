# app.py
import sys
import os
src_path = os.path.join(os.path.dirname(__file__), "src")
sys.path.append(src_path)
import numpy as np
from sim.dynamics import dynamics
from _viewer.mesh import aircraft_model_mesh
from _viewer.renderer import Renderer
from _viewer.window import SimWindow


def main():
    dyn = dynamics()

    # Static mesh once
    V, idx = aircraft_model_mesh(scale=5.0)

    def sim_step(dt):
        # Your dynamics uses fixed Ts internally; easiest is:
        # run N substeps if your Ts != dt
        # For now: assume you set dyn.Ts to dt or keep dyn.Ts and step once per tick.
        dyn.update(u=dyn_u())

    def dyn_u():
        # simplest: hold trim input from params
        # replace with controller later
        from sim.params import params
        P = params()
        return P.u_star

    def get_pose():
        s = dyn.state
        return (
            float(s[0,0]), float(s[1,0]), float(s[2,0]),
            float(s[6,0]), float(s[7,0]), float(s[8,0]),
        )

    def renderer_factory(ctx):
        return Renderer(ctx, V, idx)

    win = SimWindow(sim_step, get_pose, renderer_factory)
    import pyglet
    pyglet.app.run()

if __name__ == "__main__":
    main()
