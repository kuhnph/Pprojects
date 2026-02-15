# app.py 2/15/2026
import sys
import os
src_path = os.path.join(os.path.dirname(__file__), "src")
sys.path.append(src_path)
import numpy as np
from sim.dynamics import dynamics
from sim.params import params
from viewer.mesh import aircraft_model_mesh
from viewer.renderer import Renderer
from viewer.window import SimWindow

def main():
    dyn = dynamics()
    P = params()

    # Static mesh once
    V, idx = aircraft_model_mesh(scale=5.0)

    def sim_step(dt):
        #assume you set dyn.Ts to dt or keep dyn.Ts and step once per tick.
        dyn.update(u=dyn_u())
        P.T += P.Ts

    def dyn_u():
        # simplest: hold trim input from params (replace with controller later)
        return P.u_star

    def get_pose():
        s = dyn.state
        return (
            float(s[0,0]), float(s[1,0]), float(s[2,0]),
            float(s[6,0]), float(s[7,0]), float(s[8,0]),
        )

    def renderer_factory(ctx):
        return Renderer(ctx, V, idx)

    win = SimWindow(sim_step, get_pose, renderer_factory, record=True, record_path="out/mav_view.mp4")

    import pyglet
    try:
        pyglet.app.run()
    finally:
        # force finalize ffmpeg even if the app exits without closing the window
        try:
            if getattr(win, "writer", None) is not None:
                win.writer.close()
                win.writer = None
        except Exception as e:
            print("Error closing writer:", e)

if __name__ == "__main__":
    main()
