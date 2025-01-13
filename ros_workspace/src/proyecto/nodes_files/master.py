import rospy
from std_msgs.msg import String, Bool  

def game_finished_callback(msg):
    if msg.data:
        rospy.loginfo(f"El juego ha terminado. Puedes elegir otro juego.")
        choose_game()

rospy.init_node('master_node', anonymous=True)

rospy.Subscriber('game_status', Bool, game_finished_callback)

hanoi_orders = rospy.Publisher('/hanoi_orders', Bool, queue_size=10)

pieces_orders = rospy.Publisher('/pieces_orders', Bool, queue_size=10)

def choose_game():
    print("Selecciona un juego para iniciar:")
    print("1. Hanoi Tower")
    print("2. Shapes")
    choice = input("Ingresa el número del juego: ")
    
    if choice == "1":
        rospy.loginfo("Iniciando Hanoi Tower...")
        hanoi_orders.publish(True) 
        rospy.loginfo("Hanoi Tower ha empezado.")
    elif choice == "2":
        rospy.loginfo("Iniciando Shapes...")
        pieces_orders.publish(True)
        rospy.loginfo("Shapes ha terminado.")
    else:
        print("Opción no válida. Por favor, elige 1 o 2.")


rate = rospy.Rate(10)  
try:
    rospy.sleep(2)
    choose_game()  
    while not rospy.is_shutdown(): 
        rate.sleep()
except rospy.ROSInterruptException:
    rospy.loginfo("Nodo maestro detenido.")