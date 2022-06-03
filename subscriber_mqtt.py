import paho.mqtt.client as mqtt
from datetime import datetime

BROKER = 'test.mosquitto.org'
TOPIC = 'bot/fsm-state'

def on_connect(client, userdata, flags, rc):
	print('MQTT client connecting...', end = ' ')
	print('MQTT client subscribing...', end = ' ')
	client.subscribe(TOPIC)

def on_subscribe(client, userdata, mid, granted_qos):
	print(f'subscribed {TOPIC} with QoS: {granted_qos[0]}\n')



def on_message(client, userdata, msg):
	msg = msg.payload.decode("utf-8")
	with open("pub_vel_ws/file.txt", "w") as f:
		f.write(msg)



def main():

	client = mqtt.Client()
	#events --> callback association
	client.on_connect = on_connect
	client.on_subscribe = on_subscribe
	client.on_message = on_message

	client.connect(BROKER)

	#wait and listen for events (ctrl-c to quit)
	try:
		client.loop_forever()
	except KeyboardInterrupt:
		print('\nMQTT client disconnecting...bye')
	finally:
		client.disconnect()

if __name__ == '__main__':
	main()