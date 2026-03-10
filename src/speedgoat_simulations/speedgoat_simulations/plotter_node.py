import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
import matplotlib.pyplot as plt

class TrajectoryPlotterNode(Node):
    def __init__(self):
        super().__init__('trajectory_plotter')

        self.odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)
        self.path_sub = self.create_subscription(Path, '/reference_path', self.path_callback, 10)

        self.odom_x = []
        self.odom_y = []
        self.path_x = []
        self.path_y = []

        plt.ion()  # Modalità interattiva on
        self.fig, self.ax = plt.subplots()
        self.odom_line, = self.ax.plot([], [], 'b-', label='Traiettoria reale')
        self.path_line, = self.ax.plot([], [], 'r--', label='Percorso riferimento')
        self.ax.legend()
        self.ax.set_xlabel('X [m]')
        self.ax.set_ylabel('Y [m]')
        self.ax.set_title('Traiettoria veicolo vs riferimento')
        self.ax.grid()

        # Timer ROS: aggiorna il plot ogni 0.1 secondi
        self.create_timer(0.1, self.timer_callback) # 10 Hz

    def odom_callback(self, msg):
        self.odom_x.append(msg.pose.pose.position.x)
        self.odom_y.append(msg.pose.pose.position.y)

    def path_callback(self, msg):
        self.path_x = [pose.pose.position.x for pose in msg.poses]
        self.path_y = [pose.pose.position.y for pose in msg.poses]

    def timer_callback(self):
        if self.odom_x and self.path_x:
            self.odom_line.set_data(self.odom_x, self.odom_y)
            self.path_line.set_data(self.path_x, self.path_y)

            self.ax.relim()
            self.ax.autoscale_view()

            self.fig.canvas.draw()
            self.fig.canvas.flush_events()


    def plot_loop(self):
        while rclpy.ok():
            if self.odom_x and self.path_x:
                self.odom_line.set_data(self.odom_x, self.odom_y)
                self.path_line.set_data(self.path_x, self.path_y)

                self.ax.relim()
                self.ax.autoscale_view()

                self.fig.canvas.draw()
                self.fig.canvas.flush_events()
            plt.pause(0.1)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlotterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
