import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import matplotlib.pyplot as plt
import matplotlib.animation as animation



class DropletSizeSubscriber(Node):

    def __init__(self):
        super().__init__('droplet_size_subscriber')
        print('Node initialized')

        self.droplet_sizes = []
        self.times = []
        fig, ax = plt.subplots()

        self.subscription = self.create_subscription(
            Float64,
            'droplet_size',
            lambda msg: self.droplet_size_callback(msg, fig, ax),
            10)
        self.subscription  # prevent unused variable warning

        

        
    def droplet_size_callback(self, msg,fig,ax):
        droplet_size = msg.data
        now = self.get_clock().now().to_msg().sec
        self.droplet_sizes.append(droplet_size)
        self.times.append(now)
        print(self.droplet_sizes[-1])
        

        # Update the plot
        ax.clear()
        
        ax.plot(self.times,self.droplet_sizes)
        ax.set_title('Droplet size vs Time')
        ax.set_ylabel('Droplet size in pixels')

        ax.relim()
        ax.autoscale_view()

        fig.canvas.draw()

def main(args=None):
    rclpy.init(args=args)

    droplet_subscriber = DropletSizeSubscriber()

    rclpy.spin(droplet_subscriber)

    droplet_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()