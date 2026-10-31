"""Web versions of the rendered animations: results/anim/<log>_<cam>_drake.mp4 -> docs/anim/<name>.mp4 (960 px wide,
H.264) with a poster frame, and GIFs for the README and the page. Needs ffmpeg.

    python3 web/anim_web.py
"""
import os, subprocess, json, shutil
HERE = os.path.dirname(os.path.abspath(__file__)); SIM = os.path.abspath(os.path.join(HERE, '..'))
ANIM = os.path.join(SIM, 'results', 'anim'); DOCS = os.path.abspath(os.path.join(SIM, '..', 'docs', 'anim'))
os.makedirs(DOCS, exist_ok=True); os.makedirs(os.path.join(SIM, 'results', 'web'), exist_ok=True)

# page name, rendered file, poster time in s
CLIPS = [('follow', 'drake_run_follow_drake.mp4', 9.0), ('side', 'drake_run_side_drake.mp4', 9.0), ('orbit', 'drake_run_orbit_drake.mp4', 12.0),
         ('onpath_follow', 'drake_run_onpath_follow_drake.mp4', 6.0)]
GIFS = [('follow', 'drake_run_follow_drake.mp4', 0.0, 12.0), ('side', 'drake_run_side_drake.mp4', 0.0, 12.0)]
STILLS = [('snapshots', 'drake_run_snapshots.png')]


def run(cmd):
    subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def main():
    sizes = {}
    for name, src, t_poster in CLIPS:
        src = os.path.join(ANIM, src)
        if not os.path.exists(src):
            print('missing', src); continue
        out = os.path.join(DOCS, name + '.mp4'); jpg = os.path.join(DOCS, name + '.jpg')
        run(['ffmpeg', '-y', '-i', src, '-vf', 'scale=960:-2', '-c:v', 'libx264', '-crf', '26', '-preset', 'slow',
             '-pix_fmt', 'yuv420p', '-movflags', '+faststart', '-an', out])
        run(['ffmpeg', '-y', '-ss', str(t_poster), '-i', out, '-frames:v', '1', '-q:v', '4', jpg])
        sizes[name + '.mp4'] = round(os.path.getsize(out) / 1e6, 2); print(name, sizes[name + '.mp4'], 'MB')
    for name, src, t0, dur in GIFS:
        src = os.path.join(ANIM, src)
        if not os.path.exists(src): continue
        out = os.path.join(DOCS, name + '.gif')
        flt = 'fps=10,scale=480:-1:flags=lanczos,split[s0][s1];[s0]palettegen=max_colors=128[p];[s1][p]paletteuse=dither=bayer:bayer_scale=4'
        run(['ffmpeg', '-y', '-ss', str(t0), '-t', str(dur), '-i', src, '-vf', flt, '-loop', '0', out])
        sizes[name + '.gif'] = round(os.path.getsize(out) / 1e6, 2); print(name + '.gif', sizes[name + '.gif'], 'MB')
    for name, src in STILLS:
        src = os.path.join(ANIM, src)
        if os.path.exists(src):
            shutil.copy(src, os.path.join(DOCS, name + '.png'))
    json.dump(sizes, open(os.path.join(SIM, 'results', 'web', 'anim_sizes.json'), 'w'), indent=1)


if __name__ == '__main__':
    main()
