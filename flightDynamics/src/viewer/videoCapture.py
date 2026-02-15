# src/viewer/videoCapture.py - 2/15/2026
"""viewer.videoCapture

FFmpeg-based raw-frame video writer.

- Accepts RGB24 frames of a fixed width/height
- Pipes frames to ffmpeg stdin to produce an H.264 MP4

""" 
import os
import subprocess
from dataclasses import dataclass

@dataclass
class FFmpegVideoWriter:
    width: int
    height: int
    fps: int
    outfile: str
    crf: int = 18
    preset: str = "veryfast"
    log_path: str = "results/ffmpeg_capture.log"

    def __post_init__(self):
        self.frames = 0

        # Ensure output folder exists
        out_dir = os.path.dirname(self.outfile)
        if out_dir:
            os.makedirs(out_dir, exist_ok=True)
        log_dir = os.path.dirname(self.log_path)
        if log_dir:
            os.makedirs(log_dir, exist_ok=True)

        # yuv420p requires even dims; enforce via ffmpeg scaling
        vf = "vflip,scale=trunc(iw/2)*2:trunc(ih/2)*2"

        cmd = [
            "ffmpeg", "-y",
            "-f", "rawvideo",
            "-pix_fmt", "rgb24",
            "-s", f"{self.width}x{self.height}",
            "-r", str(self.fps),
            "-i", "-",
            "-vf", vf,
            "-an",
            "-c:v", "libopenh264",
            "-b:v", "10M",          # bitrate control (openh264 uses bitrate more reliably than crf)
            "-maxrate", "10M",
            "-bufsize", "20M",
            "-pix_fmt", "yuv420p",
            self.outfile,
        ]

        # Write ffmpeg stderr to a log so we can see why it exits
        self._log_f = open(self.log_path, "wb")
        self.proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stderr=self._log_f)

        self.expected_bytes = self.width * self.height * 3

    def write(self, frame_bytes: bytes):
        if self.proc.poll() is not None:
            raise RuntimeError(
                f"ffmpeg exited early with code {self.proc.returncode}. "
                f"See log: {self.log_path}"
            )

        if len(frame_bytes) != self.expected_bytes:
            raise ValueError(
                f"Frame byte count mismatch: got {len(frame_bytes)}, expected {self.expected_bytes} "
                f"({self.width}x{self.height} rgb24)."
            )

        try:
            self.proc.stdin.write(frame_bytes)
            self.frames += 1
            if self.frames % 60 == 0:
                print(f"[record] wrote {self.frames} frames")
        except BrokenPipeError:
            raise RuntimeError(f"Broken pipe: ffmpeg died. See log: {self.log_path}")
        

    def close(self):
        try:
            if self.proc and self.proc.stdin:
                self.proc.stdin.close()
            if self.proc:
                self.proc.wait()
        finally:
            if getattr(self, "_log_f", None):
                self._log_f.close()
