"""
TODO: Set up figure for appropriate target video size (eg. 720p).
TODO: Decide which additional user options should be available.
"""

from datetime import datetime
from pathlib import Path

import numpy as np
from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation

from flightsim.axes3ds import Axes3Ds
from flightsim.shapes import Quadrotor

class ClosingFuncAnimation(FuncAnimation):
    def __init__(self, fig, func, *args, **kwargs):
        self._close_on_finish = kwargs.pop('close_on_finish')
        FuncAnimation.__init__(self, fig, func, *args, **kwargs)

    # def _stop(self, *args):
    #     super()._stop(self, *args)
    #     if self._close_on_finish:
    #         plt.close(self._fig)

    def _step(self, *args):
        still_going = FuncAnimation._step(self, *args)
        if self._close_on_finish and not still_going:
            plt.close(self._fig)

def _decimate_index(time, sample_time):
    """
    Given sorted lists of source times and sample times, return indices of
    source time closest to each sample time.
    """
    index = np.arange(time.size)
    sample_index = np.round(np.interp(sample_time, time, index)).astype(int)
    return sample_index

def animate(time, position, ideal_pos, rotation, world, filename=None, blit=False, show_axes=True, close_on_finish=False):
    """
    Animate a completed simulation result based on the time, position, and
    rotation history. The animation may be viewed live or saved to a .mp4 video
    (slower, requires additional libraries).

    For a live view, it is absolutely critical to retain a reference to the
    returned object in order to prevent garbage collection before the animation
    has completed displaying.

    Parameters
        time, (N,) with uniform intervals
        position, (N,3)
        rotation, (N,3,3)
        world, a World object
        filename, for saved video, or live view if None
        blit, if True use blit for faster animation, default is False
        show_axes, if True plot axes, default is True
        close_on_finish, if True close figure at end of live animation or save, default is False
    """

    # Temporal style.
    rtf = 1.0 # real time factor > 1.0 is faster than real time playback
    render_fps = 30

    # Decimate data to render interval; always include t=0.
    if time[-1] != 0:
        sample_time = np.arange(0, time[-1], 1/render_fps * rtf)
    else:
        sample_time = np.zeros((1,))
    index = _decimate_index(time, sample_time)
    time = time[index]
    position = position[index,:]
    rotation = rotation[index,:,:]

    # Set up axes.
    if filename is not None:
        if isinstance(filename, Path):
            fig = plt.figure(filename.name)
        else:
            fig = plt.figure(filename)
    else:
        fig = plt.figure('Animation')
    fig.clear()
    ax = Axes3Ds(fig)
    if not show_axes:
        ax.set_axis_off()
    ax.set_xlim(-1,1)
    ax.set_ylim(-1,1)
    ax.set_zlim(-1,1)

    quad = Quadrotor(ax)

    world_artists = world.draw(ax)

    title_artist = ax.set_title('t = {}'.format(time[0]))

    def init():
        ax.draw(fig.canvas.get_renderer())
        
        ax.plot3D(position[:,0], position[:,1], position[:,2], 'b.', ms=0.5)
        ax.plot3D(ideal_pos[:,0], ideal_pos[:,1], ideal_pos[:,2], 'k', linewidth=0.75)
        return world_artists + list(quad.artists) + [title_artist]

    def update(frame):
        title_artist.set_text('t = {:.2f}'.format(time[frame]))
        quad.transform(position=position[frame,:], rotation=rotation[frame,:,:])
        [a.do_3d_projection(fig.canvas.get_renderer()) for a in quad.artists]
        
        ax.plot3D(position[:frame+1, 0], position[:frame+1, 1], position[:frame+1, 2], 'g.', ms=2)
        
        return world_artists + list(quad.artists) + [title_artist]

    ani = ClosingFuncAnimation(fig=fig,
                        func=update,
                        frames=time.size,
                        init_func=init,
                        interval=1000.0/render_fps,
                        repeat=False,
                        blit=blit,
                        close_on_finish=close_on_finish)

    if filename is not None:
        print('Saving Animation')
        ani.save(filename,
                 writer='ffmpeg',
                 fps=render_fps,
                 dpi=100)
        if close_on_finish:
            plt.close(fig)
            ani = None

    return ani
