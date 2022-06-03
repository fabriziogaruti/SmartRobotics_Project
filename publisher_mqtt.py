import paho.mqtt.client as mqtt
from datetime import datetime

BROKER = 'test.mosquitto.org'
TOPIC = 'bot/fsm-state'


def on_connect(client, userdata, flags, rc):
    print('MQTT client connecting...', end=' ')
    print(f'{mqtt.connack_string(rc)}')
    client.subscribe(TOPIC, 0)


# event_flag = True

def on_message(client, userdata, message):
    data = "{'" + str(message.payload) + "', " + str(message.topic) + "}"
    print(data)
    return data


def main():
    client = mqtt.Client(clean_session=True)

    # events --> callback association
    client.on_connect = on_connect
    client.on_message = on_message

    # client --> broker connection

    client.connect(BROKER)
    client.loop_start()

    try:
        while True:
            msg = input('\ninsert msg to publish (ctrl-c to quit)--> ')
            client.publish(TOPIC, msg)

    except KeyboardInterrupt:
        print('\nMQTT client disconnecting...bye')
    finally:
        client.disconnect()
        client.loop_stop()


if __name__ == '__main__':
    main()