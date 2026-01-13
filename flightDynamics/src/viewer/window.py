# viewer/window.py
import pyglet
import moderngl
import time
import numpy as np

class SimWindow(pyglet.window.Window):
    def __init__(self, sim_step_func, get_pose_func, renderer_factory,
                 width=1000, height=800, render_hz=60, sim_hz=200):
        super().__init__(width=width, height=height, caption="MAV Viewer (GPU)", resizable=True)

        self.ctx = moderngl.create_context()
        self.renderer = renderer_factory(self.ctx)

        self.sim_step = sim_step_func
        self.get_pose = get_pose_func

        self.sim_dt = 1.0 / sim_hz
        self.accum = 0.0
        self.last = time.perf_counter()

        # One callback that advances sim time (variable dt -> fixed sim steps)
        pyglet.clock.schedule(self._tick)

        # Separate callback that forces redraw at a steady rate
        pyglet.clock.schedule_interval(self._render, 1.0 / render_hz)

    def _tick(self, _dt):
        now = time.perf_counter()
        frame_dt = now - self.last
        self.last = now

        self.accum += frame_dt
        # prevent spiral of death if you pause/debug
        if self.accum > 0.25:
            self.accum = 0.25

        while self.accum >= self.sim_dt:
            self.sim_step(self.sim_dt)
            self.accum -= self.sim_dt

    def _render(self, _dt):
        # force a draw now
        self.dispatch_event("on_draw")
        self.flip()

    def on_draw(self):
        self.clear()
        pn, pe, pd, phi, theta, psi = self.get_pose()
        model = self._model_matrix(pn, pe, pd, phi, theta, psi)
        self.renderer.draw(self.width, self.height, model)

    @staticmethod
    def _model_matrix(pn, pe, pd, phi, theta, psi):
        c, s = np.cos, np.sin
        R = np.array([
            [c(theta)*c(psi), s(phi)*s(theta)*c(psi)-c(phi)*s(psi), c(phi)*s(theta)*c(psi)+s(phi)*s(psi)],
            [c(theta)*s(psi), s(phi)*s(theta)*s(psi)+c(phi)*c(psi), c(phi)*s(theta)*s(psi)-s(phi)*c(psi)],
            [-s(theta),       s(phi)*c(theta),                      c(phi)*c(theta)]
        ], dtype=np.float32)

        M = np.eye(4, dtype=np.float32)
        M[:3, :3] = R
        M[:3, 3] = np.array([pn, pe, pd], dtype=np.float32)
        return M
