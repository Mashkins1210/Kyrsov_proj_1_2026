import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class RobotPlotter:
    def __init__(self):
        plt.rcParams['toolbar'] = 'None'
        self.fig = plt.figure("Robot Vision & Control", figsize=(8, 8))
        self.ax = plt.subplot(111, projection='polar')
        self.scatter = self.ax.scatter(
            [], [], s=3, c=[], cmap=plt.cm.Greys_r,
            lw=0, vmin=0, vmax=50
        )
        self.ax.set_rmax(500)
        self.ax.grid(True)
        self.ax.set_theta_offset(np.pi)
        self.ax.set_rticks([100, 200, 300, 400, 500])
        self.ax.set_rlabel_position(22.5)

        self.state_text = self.ax.text(
            0.05, 0.95, 'State: MOVING', transform=self.ax.transAxes,
            fontsize=12, color='green', fontweight='bold',
            verticalalignment='top'
        )
        self.info_text = self.ax.text(
            0.05, 0.88, '', transform=self.ax.transAxes,
            fontsize=9, color='blue', verticalalignment='top'
        )
        self.ani = None
        self.update_counter = 0
        self.update_interval = 5

    def update_visualization(self, scan):
        self.update_counter += 1
        if self.update_counter < self.update_interval:
            return
        self.update_counter = 0

        if scan is not None and len(scan) > 0:
            scan_array = np.array(scan)
            valid_mask = scan_array[:, 2] < 5000
            if np.any(valid_mask):
                valid_scan = scan_array[valid_mask]
                offsets = np.column_stack([
                    np.radians(valid_scan[:, 1]),
                    valid_scan[:, 2]
                ])
                self.scatter.set_offsets(offsets)
                self.scatter.set_array(valid_scan[:, 0])

    def update_state_display(self, state_text, color='green', info_text=''):
        self.state_text.set_text(state_text)
        self.state_text.set_color(color)
        self.info_text.set_text(info_text)

    def create_animation(self, update_func, interval=100):
        self.ani = FuncAnimation(
            self.fig, update_func, interval=interval,
            blit=False, cache_frame_data=False
        )

    def show(self):
        plt.show(block=False)
        plt.pause(0.1)

    def close(self):
        plt.close('all')