"""Isolated offline 3D timing; never loads user settings or connects to robots."""
import argparse
import cProfile
import io
import json
from pathlib import Path
import pstats
import statistics
import sys
import tempfile
import time
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path)
    parser.add_argument('--frames', type=int, default=6)
    args = parser.parse_args()
    with tempfile.TemporaryDirectory() as folder, patch.object(Path, 'home', return_value=Path(folder)):
        from seer_control.app import Console
        from seer_control.smooth_renderer import renderer_for
        app = Console()
        app.withdraw()
        try:
            assert not app.real
            app.view_mode.set('3D')
            app.world3d.winfo_width = lambda: 1100
            app.world3d.winfo_height = lambda: 700
            start = time.perf_counter()
            app._spatial_render()
            first_ms = (time.perf_counter() - start) * 1000
            renderer = renderer_for(app.world3d)
            samples = []
            profiler = cProfile.Profile()
            for frame in range(args.frames):
                app.sim.state.x += .01
                start = time.perf_counter()
                app._spatial_render()
                assert app.world3d.scene_pose['x'] == app.sim.state.x
                samples.append((time.perf_counter() - start) * 1000)
            if renderer:
                assert renderer.size == (1100, 700)
                assert (app.world3d.render_image.width(), app.world3d.render_image.height()) == (1100, 700)
            profiler.enable()
            app._spatial_render()
            profiler.disable()
            result = dict(size=[1100, 700], backend=getattr(renderer, 'backend', 'Tk fallback'),
                          first_frame_ms=round(first_ms, 2),
                          median_frame_ms=round(statistics.median(samples), 2),
                          max_frame_ms=round(max(samples), 2),
                          samples_ms=[round(v, 2) for v in samples])
            stream = io.StringIO()
            pstats.Stats(profiler, stream=stream).sort_stats('cumulative').print_stats(12)
            print(json.dumps(result, ensure_ascii=False, indent=2))
            print(stream.getvalue())
            # Run the actual Tk timers with both AMR and arm animation. The
            # window stays withdrawn and dimensions stay fixed for repeatability.
            errors = []
            app.report_callback_exception = lambda kind, value, trace: errors.append(str(value))
            app.tabs.select(app.operation_page)
            app._sync_navigation()
            app._spatial_switch()
            origin = app.map.nodes[app.sim.state.last_node]
            app.sim.state.x, app.sim.state.y, app.sim.state.theta = origin['x'], origin['y'], 0.
            app.sim.navigate('LM2')
            app.arm_dev_sim.start(app.arm_dev_config, operation='demo_a')
            poses = []
            render = app._spatial_render
            def observed_render():
                render()
                assert app.world3d.scene_joint_values == app.arm_dev_sim.kin.positions(app.arm_dev_sim.q)
                poses.append((app.sim.state.x, tuple(app.arm_dev_sim.q)))
            app._spatial_render = observed_render
            start = time.perf_counter()
            app.after(2000, app.quit)
            app.mainloop()
            elapsed = time.perf_counter() - start
            assert not errors, errors
            assert len(poses) >= 10, len(poses)
            assert poses[-1][0] > poses[0][0], (poses[0], poses[-1], app.sim.status(), app.sim.block_reason, app.sim.route)
            assert len({q for x, q in poses}) > 5
            result['animated_frames_per_second'] = round(len(poses) / elapsed, 2)
            result['animation_callbacks_ok'] = True
            print('Animated SIM:', json.dumps(result, ensure_ascii=False))
            if args.output:
                args.output.parent.mkdir(parents=True, exist_ok=True)
                args.output.write_text(json.dumps(result, indent=2), encoding='utf-8')
        finally:
            app.close()


if __name__ == '__main__':
    main()
