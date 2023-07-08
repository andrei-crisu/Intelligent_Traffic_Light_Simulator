import socket
import re
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ServerNode(Node):

    def __init__(self):
        super().__init__('server_node')

        # data settings
        self.data_size = 16 # sending 16 bytes = 128 bits (binary touch states, for example)

        # server settings
        self.server_name = str([l for l in ([ip for ip in socket.gethostbyname_ex(socket.gethostname())[2] if not ip.startswith("127.")][:1], [[(s.connect(('8.8.8.8', 53)), s.getsockname()[0], s.close()) for s in [socket.socket(socket.AF_INET, socket.SOCK_DGRAM)]][0][1]]) if l][0][0]) # https://stackoverflow.com/a/1267524
        #self.server_name='192.168.56.1'
        self.server_port = 6000
        self.server_address = (self.server_name, self.server_port)

        # ROS 2 settings
        self.publisher = self.create_publisher(String, 'topic007', 10)

        # start up server
        self.get_logger().info('Setting up server on: '+str(self.server_address))
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind(self.server_address)
        self.server_socket.listen(1)

        # wait for connection
        self.get_logger().info('Waiting for a client connection...')
        self.connection, self.client_address = self.server_socket.accept()
        self.get_logger().info('Connected to: '+str(self.client_address))

        # flag to indicate whether to keep running the loop
        self.should_run = True

    def run_server(self):
        try:
            while True:
                data = self.connection.recv(self.data_size)


                message = String()
                #remove all symbols that are not digits
                data=str(data)
                data=re.sub("[^0-9]", "",data)
                message.data = data
                if len(data)>0:
                    self.get_logger().info('Received: '+data) 
                    self.publisher.publish(message)
        except KeyboardInterrupt:
            self.get_logger().info('Keyboard interrupt received. Shutting down server...')
            self.connection.close()


    def stop_server(self):
        self.should_run = False

        # close the connection
        self.connection.close()


def main(args=None):
    rclpy.init(args=args)

    server = ServerNode()
    server.run_server()
    server.stop_server()

    rclpy.shutdown()


if __name__ == '__main__':
    main()

   
