import paho.mqtt.client as mqtt
import serial
import platform

lightTopicName = f'homeassistant/light/{platform.node()}/light'
rgbButtonTopicName = f'homeassistant/device_automation/{platform.node()}/rgbButton'
rotaryEncoderButtonTopicName = f'homeassistant/device_automation/{platform.node()}/rotaryEncoderButton'
rotaryEncoderCWTopicName = f'homeassistant/device_automation/{platform.node()}/rotaryEncoderCW'
rotaryEncoderCCWTopicName = f'homeassistant/device_automation/{platform.node()}/rotaryEncoderCCW'

lightState = False
lightColor = b"255,255,255"

ser = serial.Serial('/dev/ttyACM0', timeout=1)  # open serial port


# The callback for when the client receives a CONNACK response from the server.
def on_connect(client: mqtt.Client, userdata, flags, rc):
    print("Connected with result code " + str(rc))

    client.publish(lightTopicName + '/config',
                   f'{{"name": "{platform.node()} light", "unique_id": "{platform.node()}_rgb_light" ,"command_topic": "{lightTopicName}/set", "rgb_command_topic": "{lightTopicName}/rgb_set", "device": {{"identifiers":["{platform.node()}"], "name": "{platform.node()}"}}}}',
                   retain=True)

    client.publish(rgbButtonTopicName + '/config',
                   f'{{"automation_type": "trigger","payload":"rgb_button_pressed" , "topic": "{rgbButtonTopicName}/events", "type": "button_short_press", "subtype": "rgb_button", "device": {{"identifiers":["{platform.node()}"], "name": "{platform.node()}"}}}}',
                   retain=True)

    client.publish(rotaryEncoderButtonTopicName + '/config',
                   f'{{"automation_type": "trigger","payload":"rotary_encoder_button_pressed" , "topic": "{rotaryEncoderButtonTopicName}/events", "type": "button_short_press", "subtype": "rotary_encoder_button", "device": {{"identifiers":["{platform.node()}"], "name": "{platform.node()}"}}}}',
                   retain=True)

    client.publish(rotaryEncoderCWTopicName + '/config',
                   f'{{"automation_type": "trigger","payload":"rotary_encoder_clockwise_turn" , "topic": "{rotaryEncoderCWTopicName}/events", "type": "button_short_press", "subtype": "rotary_encoder_clockwise", "device": {{"identifiers":["{platform.node()}"], "name": "{platform.node()}"}}}}',
                   retain=True)

    client.publish(rotaryEncoderCCWTopicName + '/config',
                   f'{{"automation_type": "trigger","payload":"rotary_encoder_counter_clockwise_turn" , "topic": "{rotaryEncoderCCWTopicName}/events", "type": "button_short_press", "subtype": "rotary_encoder_counter_clockwise", "device": {{"identifiers":["{platform.node()}"], "name": "{platform.node()}"}}}}',
                   retain=True)

    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe(f'{lightTopicName}/#')


# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    global lightState
    global lightColor

    print(msg.topic + " " + str(msg.payload))
    print(lightState)

    if msg.topic == f'{lightTopicName}/set':
        lightState = True if msg.payload == b'ON' else False

    if lightState:
        ser.write(lightColor + b'\n')
    else:
        ser.write(b'0,0,0\n')

    if msg.topic == f'{lightTopicName}/rgb_set' and lightState:
        lightColor = msg.payload



client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.username_pw_set("controlPanel", "password")

client.connect("192.168.10.5", 1883, 60)

# Blocking call that processes network traffic, dispatches callbacks and
# handles reconnecting.
# Other loop*() functions are available that give a threaded interface and a
# manual interface.
client.loop_start()

while True:
    serialLine = ser.readline()

    serialLine = serialLine.replace(b'\r\n', b'')

    serialLineList = serialLine.split(b" ")

    print(serialLine)
    print(serialLineList)


    match serialLineList[0]:
        case b'RGB_BUTTON_PUSHED':
            client.publish(rgbButtonTopicName + '/events', 'rgb_button_pressed')
        case b'ROTARY_BUTTON_PUSHED':
            client.publish(rotaryEncoderButtonTopicName + '/events', 'rotary_encoder_button_pressed')
        case b'POS':
            if int(serialLineList[1]) > 0:
                for num in range(abs(int(serialLineList[1]))):
                    print(num)
                    client.publish(rotaryEncoderCCWTopicName + '/events', 'rotary_encoder_counter_clockwise_turn')
            if int(serialLineList[1]) < 0:
                for num in range(abs(int(serialLineList[1]))):
                    client.publish(rotaryEncoderCWTopicName + '/events', 'rotary_encoder_clockwise_turn')
                    print(num)




