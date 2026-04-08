import pyglet
import moderngl
import time
import numpy as np
from sim.rotations import model_matrix_from_pose
from viewer.videoCapture import FFmpegVideoWriter


class SimWindow(pyglet.window.Window):
    def __init__(self, sim_step_func, get_pose_func, get_sim_time_func, renderer_factory,
                 width=1000, height=800, render_hz=60, sim_hz=200,
                 record=False, record_mode=False, record_fps=20,
                 record_path="out/mav_view.mp4"):
        super().__init__(width=width, height=height, caption="MAV Viewer (GPU)", resizable=True)

        self.ctx = moderngl.create_context()
        self.renderer = renderer_factory(self.ctx)

        self.sim_step = sim_step_func
        self.get_pose = get_pose_func
        self.get_sim_time = get_sim_time_func

        self.sim_dt = 1.0 / sim_hz
        self.accum = 0.0
        self.last = time.perf_counter()

        self.record = record
        self.record_mode = record_mode
        self.record_path = record_path
        self.record_fps = int(record_fps)
        self.writer = None
        self._writer_size = None

        # Deterministic frame-to-sim relationship for record mode
        self.frame_dt = 1.0 / self.record_fps
        self.steps_per_frame = max(1, int(round(self.frame_dt / self.sim_dt)))
        self.frame_dt = self.steps_per_frame * self.sim_dt

        self.time_label = pyglet.text.Label(
            "t = 0.00 s",
            font_name="Arial",
            font_size=14,
            x=10,
            y=self.height - 10,
            anchor_x="left",
            anchor_y="top",
            color=(255, 255, 255, 255),
        )

        if not self.record_mode:
            pyglet.clock.schedule(self._tick)
            pyglet.clock.schedule_interval(self._render, 1.0 / render_hz)
        else:
            pyglet.clock.schedule_interval(self._render_record_mode, 1.0 / self.record_fps)

    def _ensure_writer(self):
        if not self.record:
            return

        fb_w, fb_h = self.ctx.screen.size

        if self.writer is None:
            self.writer = FFmpegVideoWriter(
                width=fb_w,
                height=fb_h,
                fps=self.record_fps,
                outfile=self.record_path,
                log_path="out/ffmpeg_capture.log",
            )
            self._writer_size = (fb_w, fb_h)
            return

        if (fb_w, fb_h) != self._writer_size:
            print("[record] Framebuffer resized; closing writer to avoid corrupt output.")
            self.writer.close()
            self.writer = None
            self.record = False

    def _tick(self, _dt):
        now = time.perf_counter()
        frame_dt = now - self.last
        self.last = now

        self.accum += frame_dt
        if self.accum > 0.25:
            self.accum = 0.25

        while self.accum >= self.sim_dt:
            self.sim_step(self.sim_dt)
            self.accum -= self.sim_dt

    def _render(self, _dt):
        if self.record:
            self._ensure_writer()

        self.dispatch_event("on_draw")

        if self.record and self.writer is not None:
            self._capture_window_frame()

        self.flip()

    def _render_record_mode(self, _dt):
        if self.record:
            self._ensure_writer()

        # Advance simulation by fixed amount per output frame
        for _ in range(self.steps_per_frame):
            self.sim_step(self.sim_dt)

        self.dispatch_event("on_draw")

        if self.record and self.writer is not None:
            self._capture_window_frame()

        self.flip()

    def on_close(self):
        if self.writer is not None:
            self.writer.close()
            self.writer = None
        super().on_close()

    def on_draw(self):
        self.clear()

        pn, pe, pd, phi, theta, psi = self.get_pose()
        model = model_matrix_from_pose(pn, pe, pd, phi, theta, psi)

        self.renderer.draw(self.width, self.height, model)
        self._draw_overlay()

    def _draw_overlay(self):
        sim_t = self.get_sim_time()
        self.time_label.text = f"t = {sim_t:8.2f} s"
        self.time_label.y = self.height - 10

        try:
            self.ctx.screen.use()
            self.ctx.disable(moderngl.DEPTH_TEST)
        except Exception:
            pass

        self.time_label.draw()

        try:
            self.ctx.enable(moderngl.DEPTH_TEST)
        except Exception:
            pass

    def _capture_window_frame(self):
        buffer = pyglet.image.get_buffer_manager().get_color_buffer()
        image_data = buffer.get_image_data()

        width = image_data.width
        height = image_data.height

        raw = image_data.get_data("RGBA", width * 4)
        rgba = np.frombuffer(raw, dtype=np.uint8).reshape((height, width, 4))
        rgb = rgba[:, :, :3]

        # rgb = np.flipud(rgb)

        self.writer.write(rgb.tobytes())

    @staticmethod
    def _model_matrix(pn, pe, pd, phi, theta, psi):
        return model_matrix_from_pose(pn, pe, pd, phi, theta, psi)