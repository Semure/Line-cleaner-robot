#!/usr/bin/env python3

import rclpy # Python library for ROS 2
from rclpy.node import Node # Maneja la creación de nodos
from sensor_msgs.msg import Image # La imagen es el tipo de mensaje
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from cv_bridge import CvBridge # Paquete para convertir entre imágenes ROS y OpenCV
import cv2 # Biblioteca OpenCV
import matplotlib.pyplot as plt
import numpy
 
class seguidor(Node):
  """
  Cree una clase seguidor, que es una subclase de la clase Node.  """
  def __init__(self):
    """
    Constructor de clase para configurar el nodo.


    """
    # Inicia el constructor de la clase Node y dale un nombre
    super().__init__('Seguidor')
     

    cv2.namedWindow("window", 1)
     # Crea el suscriptor.
    self.subscription = self.create_subscription(Image,'/camera1/image_raw',self.listener_callback,rclpy.qos.qos_profile_sensor_data)
    self.subscription # evitar advertencia de variable no utilizada
    
    # Crea el publicador.
    self.publisher= self.create_publisher(Twist, '/cmd_vel', rclpy.qos.qos_profile_system_default)  
    self.publisher
  
    # Se utiliza para convertir entre imágenes ROS y OpenCV    
    self.br = CvBridge()

    self.twist = Twist()
   
  def listener_callback(self, data): #el callback se encarga de procesar la imagen
    """
    Callback function.
    """

    image = self.br.imgmsg_to_cv2(data,desired_encoding='bgr8')

    hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
    lower_yellow = numpy.array([0, 0, 200])
    upper_yellow = numpy.array([145, 60, 255])
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)


    h, w, d = image.shape
    search_top = 2*h//5
    search_bot = 4*h//6
    mask[0:search_top, 0:w] = 0
    mask[search_bot:h, 0:w] = 0

    M = cv2.moments(mask)
    if M['m00'] > 0:
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        cv2.circle(image, (cx, cy), 15, (0,0,255), -1)
        err = cx - w/2 - 332
        self.twist.linear.x = 0.2
        self.twist.angular.z = -float(err) / 100
        s = numpy.sum(mask, axis=0)
        print(s)
        self.publisher.publish(self.twist)
    cv2.imshow("window", image)
    cv2.imshow("mascara", mask)
    cv2.waitKey(3) 
     
    
def main(args=None):
  
  rclpy.init(args=args)
  Seguidor = seguidor()
  rclpy.spin(Seguidor)
  Seguidor.destroy_node()
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()
