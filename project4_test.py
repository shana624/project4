import paho.mqtt.client as mqtt
import tensorflow as tf
import numpy as np
import base64
import threading
import time
from io import BytesIO
from PIL import Image

# MQTT Settings
BROKER_ADDRESS = "192.168.0.188"
PORT = 1883
TOPIC_SUBSCRIBE = "/vision/data"
TOPIC_PUBLISH = "/vision/result"
MODEL_PATH = "/home/pi/Desktop/model.tflite"

# Define class labels
LABELS = ["normal", "defect", "none"]

# Load the TensorFlow Lite model
interpreter = tf.lite.Interpreter(model_path=MODEL_PATH)
interpreter.allocate_tensors()
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

# Event flag to control debug messages
debug_stop_event = threading.Event()

def process_image(base64_data):
    """Decode and preprocess image for model inference."""
    decoded_data = base64.b64decode(base64_data)
    image = Image.open(BytesIO(decoded_data)).resize((224, 224))

    # Convert image to numpy array (uint8)
    input_data = np.array(image, dtype=np.uint8)  # ✅ uint8 유지
    input_data = np.expand_dims(input_data, axis=0)  # Add batch dimension

    return input_data

def on_message(client, userdata, msg):
    """Callback when an MQTT message is received."""
    try:
        debug_stop_event.set()  # Stop debug messages
        print("Received MQTT message, processing...")
        input_data = process_image(msg.payload)
        interpreter.set_tensor(input_details[0]['index'], input_data)
        interpreter.invoke()
        output_data = interpreter.get_tensor(output_details[0]['index'])

        # Find the index of the highest probability
        predicted_index = np.argmax(output_data)

        # Get the corresponding label
        result_label = LABELS[predicted_index]

        # Publish the result
        client.publish(TOPIC_PUBLISH, result_label)
        print(f"Published result: {result_label}")
    except Exception as e:
        print(f"Error processing image: {e}")
    finally:
        time.sleep(2)  # Small delay to avoid rapid toggling
        debug_stop_event.clear()  # Restart debug messages

def on_connect(client, userdata, flags, rc):
    """Callback when the MQTT client connects."""
    if rc == 0:
        print("Connected to MQTT Broker!")
        client.subscribe(TOPIC_SUBSCRIBE)
        client.publish(TOPIC_PUBLISH, "connected")  # Publish connection status
    else:
        print(f"Failed to connect, return code {rc}")

def debug_message():
    """Function to print debug message every 10 seconds."""
    while True:
        if not debug_stop_event.is_set():
            print("Waiting for MQTT messages...")
        time.sleep(10)

# Start debug message thread
debug_thread = threading.Thread(target=debug_message, daemon=True)
debug_thread.start()

# Initialize MQTT client
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect(BROKER_ADDRESS, PORT, 60)

# Start the loop to process MQTT messages
client.loop_forever()
