import json
import paho.mqtt.client as mqtt
import time
import random
import threading

# Initial JSON data
initial_data = {
    "id": 0,
    "estado_operativo": "Espera",
    "consumo_corriente": 0,
    "porcentaje_bateria": 0,
    "carga_colocada": "No",
    "posicion": {
        "posicionX": 0,
        "posicionY": 0,
        "posicionDir": 0,
    }
}
class SensorData:
    def __init__(self,id):
        self.id = id
        self.estado_operativo = "Espera"
        self.consumo_corriente = 0
        self.porcentaje_bateria = 0
        self.carga_colocada = "No"
        self.posicionX = 0
        self.posicionY = 0
        self.posicionDir = 0
        self.f_work=True
        self.f_Stop=False
        self.f_carga=False
        self.f_EO=0
        self.f_debug=False
        self.f_servo=False
        self.f_seguidor=False

    def get_json_data(self):
        data = {
            "id": self.id,
            "estado_operativo": self.estado_operativo,
            "consumo_corriente": self.consumo_corriente,
            "porcentaje_bateria": self.porcentaje_bateria,
            "carga_colocada": self.carga_colocada,
            "posicion": {
                "posicionX": self.posicionX,
                "posicionY": self.posicionY,
                "posicionDir": self.posicionDir,
            }
        }
        return data
    
class MqttPublisher:
    def __init__(self, client_id, broker_address, topic, answer_topic,subscribe_topic, get_new_values_callback,id):
        self.broker_address = broker_address
        self.topic = topic
        self.topic_resp = answer_topic
        self.subscribe_topic = subscribe_topic
        self.client = mqtt.Client(client_id)
        self.data = initial_data
        self.lock = threading.Lock()
        self.task_callbacks = {}
        self.id=id
        self.get_new_values_callback = get_new_values_callback

    def connect_to_broker(self):
        self.client.connect(self.broker_address, 1883, 60)

    def publish_data(self,topic,data):
        payload = json.dumps(data)
        self.client.publish(topic, payload)
#        print(f"Published to {topic}: {payload}")

    def update_json_values(self, new_data):
        with self.lock:
            # Update JSON values dynamically
            for key, value in new_data.items():
                if key in self.data:
                    self.data[key] = value
                elif key in self.data["posicion"]:
                    self.data["posicion"][key] = value
            self.publish_data(self.topic,self.data)

    def on_message(self, client, userdata, msg):
            payload = json.loads(msg.payload.decode())
            print(payload)
            id = payload.get("UR")
            if id == self.id:
                task_i = payload.get("Task")
                if task_i is not None:
                    # Call the callback function for the task if it's defined
                    callback = self.task_callbacks.get(task_i)
                    if callback:
                        callback()
                    else:
                        print(f"No callback defined for Task {task_i}")

    def set_task_callback(self, task_i, callback_function):
        self.task_callbacks[task_i] = callback_function
        
    def run_publisher(self):
        try:
            while True:
                new_values = self.get_new_values_callback()
                self.update_json_values(new_values)
                time.sleep(1)
        except KeyboardInterrupt:
            print("Publishing stopped.")
            self.client.disconnect()

    def run_subscriber(self):
        self.client.subscribe(self.subscribe_topic)
        self.client.on_message = self.on_message
        print("Okay")
        try:
            while True:
                self.client.loop()
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("Subscription stopped.")
            self.client.disconnect()

    def run(self):
        self.connect_to_broker()
        #publisher_thread = threading.Thread(target=self.run_publisher,daemon=True)
        subscriber_thread = threading.Thread(target=self.run_subscriber,daemon=True)
        #publisher_thread.start()
        subscriber_thread.start()
        #publisher_thread.join()

if __name__ == "__main__":
    def get_new_values():
        return {
            "id": 1,
            "estado_operativo": "F",
            "porcentaje_bateria": random.uniform(0, 100),
            "consumo_corriente": random.uniform(0, 300),
            "posicion": {
                "posicionX": random.uniform(-2000, 2000),
                "posicionY": random.uniform(-2000, 2000),
                "posicionDir": int(random.uniform(-180, 180)),
            }
        }

    # Create an instance of MqttPublisher with the callback function
    mqtt_publisher = MqttPublisher(client_id="UR1",broker_address="localhost",topic="UR",answer_topic="Task/resp",subscribe_topic="Task/1",get_new_values_callback=get_new_values)
    mqtt_publisher.run()
