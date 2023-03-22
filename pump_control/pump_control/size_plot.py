import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import matplotlib.pyplot as plt


class DropletSizeSubscriber(Node):

    def __init__(self):
        super().__init__('droplet_size_subscriber')

        self.subscription = self.create_subscription(
            Float64,
            'droplet_size',
            self.droplet_size_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.droplet_sizes = []
        self.times = []
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlabel('Time (s)')
        self.ax.set_ylabel('Droplet Size (pixels)')
        self.ax.set_title('Droplet Size over Time')

    def droplet_size_callback(self, msg):
        droplet_size = msg.data
        now = self.get_clock().now().to_msg().sec
        self.droplet_sizes.append(droplet_size)
        self.times.append(now)

        # Update the plot
        self.ax.plot(self.times, self.droplet_sizes, '-o')
        self.ax.relim()
        self.ax.autoscale_view()

        self.fig.canvas.draw()
        plt.pause(0.001)

def main(args=None):
    rclpy.init(args=args)

    droplet_subscriber = DropletSizeSubscriber()

    rclpy.spin(droplet_subscriber)

    droplet_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()