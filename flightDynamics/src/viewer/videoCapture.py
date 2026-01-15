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

    def __post_init__(self):
        # We use vflip because OpenGL's (0,0) is bottom-left.
        cmd = [
            "ffmpeg", "-y",
            "-f", "rawvideo",
            "-pix_fmt", "rgb24",
            "-s", f"{self.width}x{self.height}",
            "-r", str(self.fps),
            "-i", "-",                # stdin
            "-vf", "vflip",
            "-an",
            "-c:v", "libx264",
            "-preset", self.preset,
            "-crf", str(self.crf),
            "-pix_fmt", "yuv420p",
            self.outfile,
        ]
        self.proc = subprocess.Popen(cmd, stdin=subprocess.PIPE)

    def write(self, frame_bytes: bytes):
        # Must be exactly width * height * 3 bytes for rgb24
        self.proc.stdin.write(frame_bytes)

    def close(self):
        if self.proc and self.proc.stdin:
            self.proc.stdin.close()
        if self.proc:
            self.proc.wait()
