#!/usr/bin/env python3

import sys
import paho.mqtt.client as mqtt

#    ********** Danfoss VLT 2000 **********
#    Bytes  Function                Comment
#    1      Start Byte              '<'
#    2,3     Address                '00'
#    4       Control Char           'U' 'R' 'C' 'I'
#    5,8     Control / Status word  Only last 4 bits from each byte are used
#    9,12    Parameter Number
#    13      Sign                   '-' or anything else is '+'
#    14,18   Data
#    19      Comma                  number of decimals
#    20,21   Checksum               '??' for no control
#    22      Stop Byte              '>'

parameterText = {
  '000': {'name':'Language', 0:'English', 1: 'German', 2:'French', 3: 'Danish'},
  '001': {'name':'Menu Setup Select', 1:'Setup 1', 2: 'Setup 2', 5:'Multi-Setup'},
  '002': {'name':'Setup Copy', 0:'No copying', 6:'Copy setup 1 to 2', 7:'Copy to 2 to 1', 8:'Copy from fact. setting to 1', 9:'Copy from fact. setting to 2'},
  '003': {'name':'Operation Site', 0:'Remote', 1: 'Local with external stop', 2: 'Local', 3:'Local and remote'},
  '004': {'name':'Local Reference', 'unit':'Hz'},
  '005': {'name':'Display Value'},
  '006': {'name':'Local Reset', 0:'Disable', 1: 'Enable'},
  '007': {'name':'Local Start/Stop', 0:'Disable', 1: 'Enable'},
  '008': {'name':'Local Reversing', 0:'Not Possible', 1: 'Possible'},
  '009': {'name':'Local Jog', 0:'Disable', 1: 'Enable'},
  '013': {'name':'Data Change Lock', 0:'Not Locked', 1: 'Locked'},
  '010': {'name':'Local Reference', 0:'Disable', 1: 'Enable' , 2:'Enable and Save'},
  '101': {'name':'Speed Control', 0:'Open Loop', 1: 'Slip compensation', 2: 'Closed Loop'},
  '102': {'name':'Setting of Current Limit', 0:'Pre-programmed value', 1: 'Voltage Signal', 2: 'Current signal'},
  '103': {'name':'Motor Power', 0:'Under size', 1: 'Nominal size', 2: 'Over size'},
  '104': {'name':'Motor Voltage', 0:'200V', 1: '208V', 2: '220V', 3: '230V', 4: '240V'},
  '105': {'name':'Motor Frequency', 0:'50Hz', 1: '60Hz', 2: '87Hz', 3: '100Hz'},
  '107': {'name':'Motor Current', 'unit':'A'},
  '108': {'name':'Motor Magnetizing Current', 'unit':'A'},
  '109': {'name':'Start Voltage', 'unit':'V'},
  '110': {'name':'Start Compensation', 'unit':'V/A'},
  '111': {'name':'V/f Ratio', 'unit':'V/Hz'},
  '112': {'name':'Slip Compensation', 'unit':'Hz'},
  '114': {'name':'Feedback Signal', 0:'Voltage', 1: 'Current', 2: 'Pulses'},
  '119': {'name':'Feed Forward Factor', 'unit':'%'},
  '120': {'name':'Control Range', 'unit':'%'},
  '121': {'name':'Proportional Gain'},
  '122': {'name':'Integral Time', 'unit':'s'},
  '125': {'name':'Feedback Factor', 'unit':'%'},
  '200': {'name':'Frequency Range', 0:'0 to 120 Hz', 1:'0 to 500Hz'},
  '201': {'name':'Minimum Frequency', 'unit':'Hz'},
  '202': {'name':'Maximum Frequency', 'unit':'Hz'},
  '203': {'name':'Jog Frequency', 'unit':'Hz'},
  '204': {'name':'Digital Reference Type', 0:'Sum', 1:'Relative'},
  '205': {'name':'Digital Reference 1', 'unit':'% of f-max - f-min'},
  '206': {'name':'Digital Reference 2', 'unit':'% of f-max - f-min'},
  '207': {'name':'Digital Reference 3', 'unit':'% of f-max - f-min'},
  '208': {'name':'Digital Reference 4', 'unit':'% of f-max - f-min'},
  '209': {'name':'Current Limit', 'unit':'A'},
  '210': {'name':'Warning: Low Frequency', 'unit':'Hz'},
  '211': {'name':'Warning: High Frequency', 'unit':'Hz'},
  '213': {'name':'Warning: High Current', 'unit':'A'},
  '215': {'name':'Accelleration (Ramp-up) Time', 'unit':'s'},
  '216': {'name':'Decelleration (Ramp-down) Time', 'unit':'s'},
  '218': {'name':'Quick Stop Ramp', 'unit':'s'},
  '224': {'name':'Carrier Frequency', 'unit':'kHz'},
  '230': {'name':'Digital Speed Up/Down', 0:'Disable', 1: 'Enable', 2: 'Enable and Save'},
  '300': {'name':'Brake Function', 0:'Not Applied', 1:'Applied'},
  '306': {'name':'DC Braking Time', 'unit':'s'},
  '307': {'name':'DC Brake Cut-in Frequency', 'unit':'Hz'},
  '308': {'name':'DC Brake Voltage', 'unit':'V'},
  '309': {'name':'Reset Function', 0:'Manual Reset', 1:'Automatic Reset 1', 2: 'Automatic reset 5'},
  '310': {'name':'Trip Delay at Current Limit (61 = infinite)', 'unit':'s'},
  '315': {'name':'Motor Thermal Protection', 0:'Off', 1:'Only Warning', 2:'Trip'},
  '402': {'name':'Terminal 18 Start', 0: 'Start', 1: 'Pulse Start', 2: 'No Function', 3: 'Speed Up', 4: 'Digital Reference Select', 5: 'Reversing', 6: 'Reset and Start', 7: 'Motor Coasting & Start'},
  '403': {'name':'Terminal 19 Reversing', 0: 'Reversing', 1: 'Start Reversing', 2: 'No Function', 3: 'Speed Down', 4: 'Digital Reference Select', 5: 'Reset'},
  '404': {'name':'Terminal 27 Stop', 0:'Motor Coasting Stop', 1:'Quick-Stop', 2:'DC Braking', 3: 'Reset and Motor Coasting', 4: 'Stop', 5: 'Reset and Start', 6: 'Speed Down', 7: 'Digital Reference Select'},
  '405': {'name':'Terminal 29 Jog', 0:'Jog', 1:'Start', 2:'Digital Reference', 3: 'Pulse Input, 100 Hz', 4: 'Pulse Input, 1 KHz', 5: 'Pulse Input, 10 KHz', 6: 'Setup Select', 7: 'Reset', 8:'Reversing', 9:'Speed Down'},
  '408': {'name':'Terminal 46 Output', 0:'Unit Ready', 1:'Unit Ready Remote Control', 2:'Enabled no Warning', 3: 'Running', 4: 'Running, no warning', 5: 'Running in range, no warning', 6: 'Speed = reference, no warning', 7: 'Alarm', 8:'Alarm or warning', 9:'Current limit', 10:'Out of frequency range', 11:'Out of current range', 12:'Reversing', 13:'Pulse output 15 Hz - 1.5 kHz', 14:'Pulse output 15 Hz - 3.0 kHz', 15:'Pulse output 15 Hz - value par. 005', 18:'Send/receive RS485', 19:'Receive/send RS485'},
  '409': {'name':'Terminal 01 Relay Output', 0:'Unit Ready', 1:'Unit Ready Remote Control', 2:'Enabled no Warning', 3: 'Running', 4: 'Running, no warning', 5: 'Running in range, no warning', 6: 'Running on reference, no warning', 7: 'Alarm', 8:'Alarm or warning', 9:'Current limit', 10:'Out of frequency range', 11:'Out of current range', 12:'Reversing'},
  '411': {'name':'Analog Input Current', 0:'Linear between minimum and maximum', 1:'Proportional with lower limit'},
  '412': {'name':'Terminal 53 Analog Input Voltage', 0:'No Function', 1:'0 to 10 V', 2:'10 to 0 V'},
  '413': {'name':'Terminal 60 Analog Input Current', 0:'No Function', 1:'0 to 20 mA', 2:'4 to 20 mA', 3:'20 to 0 mA', 4:'20 to 4 mA'},
  '500': {'name':'Address'},
  '501': {'name':'Baud Rate', 'unit':'bits/s'},
  '502': {'name':'Data Readout', 0:'Reference', 1:'Frequency', 2:'Display/Feedback', 3:'Current', 4:'Torque', 5:'Power', 8:'Motor voltage', 9:'DC voltage', 10:'Motor thermal load', 11:'Thermal inverter load', 12:'Digital input', 13:'Analog input 1', 14:'Analog input 2', 15:'Warning parameter', 16:'Control word', 17:'Status word', 18:'Alarm parameter', 19:'Software version no. 4 digits'},
  '503': {'name':'Coasting', 0:'Digital', 1:'Bus', 2: 'Logical and', 3:'Logical or'},
  '504': {'name':'Quick-Stop', 0:'Digital', 1:'Bus', 2: 'Logical and', 3:'Logical or'},
  '505': {'name':'DC Brake', 0:'Digital', 1:'Bus', 2: 'Logical and', 3:'Logical or'},
  '506': {'name':'Start', 0:'Digital', 1:'Bus', 2: 'Logical and', 3:'Logical or'},
  '507': {'name':'Direction of Rotation', 0:'Digital', 1:'Bus', 2: 'Logical and', 3:'Logical or'},
  '508': {'name':'Reset', 0:'Digital', 1:'Bus', 2: 'Logical and', 3:'Logical or'},
  '509': {'name':'Selection of Setup', 0:'Digital', 1:'Bus', 2: 'Logical and', 3:'Logical or'},
  '510': {'name':'Selection of Speed', 0:'Digital', 1:'Bus', 2: 'Logical and', 3:'Logical or'},
  '511': {'name':'Bus Jogging 1', 'unit':'Hz'},
  '514': {'name':'Bus Bit 4', 0:'Quick-Stop', 1:'DC Braking'},
  '516': {'name':'Bus Reference', 'unit':'%'},
  '517': {'name':'Save Data Values', 0:'Off', 1:'On'},
  '606': {'name':'Total Operation Hours'},
  '607': {'name':'Running Hours', 'unit':'hour'},
  '608': {'name':'Number of Power-ups'},
  '609': {'name':'Number of Over-temperature'},
  '610': {'name':'Number of Over-Voltage'}
}

statuswordtext = [
    [["Controls Not Ready", "Controls Ready"],
    [ "VLT Not Ready",    "VLT Ready"],
    [ "Coast",        "Motor Coasting"],
    [ "No Fault",       "Trip"]],

    [["On 2",         "Off 2"],
    [ "On 3",         "Off 3"],
    [ "Start Enabled",    "Start Not Enabled"],
    [ "No Warning",     "Warning"]],

    [["Speed != Ref",     "Speed == Ref"],
    [ "Local Operation",  "Bus Control"],
    [ "Out Of Operating Range", "Freq Limit OK"],
    [ "Not Running",    "Runing"]],

    [["VLT OK",       "VLT Stalling Autostart"],
    [ "Voltage OK",     "Voltage Limit"],
    [ "Current OK",     "Current Limit"],
    [ "Timers OK",      "Timers Limit"]]
]

controlwordtext = [
     [["Off 1", "On 1"],
     ["Off 2", "On 2"],
     ["Off 3", "On 3"],
     ["Coasting", "Enabled"]],

     [["Quick Stop", "Ramp"],
     ["Hold Frequency Output", "Use Ramp"],
     ["Ramp Stop", "Start"],
     ["No Function", "Reset"]],

     [["Jog 1 Off", "Jog 1 On"],
     ["Jog 2 Off", "Jog 2 On"],
     ["Data not Valid","Data Valid"],
     ["No Function", "Slow Down"]],

     [["No Function", "Catch Up"],
     ["Param setup", "Select Setup 1"],
     ["Param Setup", "Select Setup 2"],
     ["No Function", "Reversing"]]
 ]

received = False
startControlWord = b'OGD@'


VLTdata = {
    'startbyte': b'<',
    'address': b'01',
    'controlchar': b'R',
    'ctrlstatus': b'0000',
    'parameter': b'0502',
    'sign': b'+',
    'data': b'00000',
    'comma': b'0',
    'checksum': b'??',
    'stopbyte': b'>'
}

def decodeVLTdata( rawdata ):
    #check if start and stop byte are correct
    if (rawdata[0:1] != b'<' and rawdata[21:] != b'>'):
        print("found startbyte:{:c} and stopbyte:{:c}".format(rawdata[0],rawdata[21]))
        return -1
    else:
      VLTdata['address'] = rawdata[1:3]
      VLTdata['controlchar'] = rawdata[3:4]
      VLTdata['ctrlstatus'] = rawdata[4:8]
      VLTdata['parameter'] = rawdata[8:12]
      VLTdata['sign'] = rawdata[12:13]
      VLTdata['data'] = rawdata[13:18]
      VLTdata['comma'] = rawdata[18:19]
      VLTdata['checksum'] = rawdata[19:21]


def buildTelegram():
    rawdata = bytearray()
    rawdata += VLTdata['startbyte']
    rawdata += VLTdata['address']
    rawdata += VLTdata['controlchar']
    rawdata += VLTdata['ctrlstatus']
    rawdata += VLTdata['parameter']
    rawdata += VLTdata['sign']
    rawdata += VLTdata['data']
    rawdata += VLTdata['comma']
    rawdata += VLTdata['checksum']
    rawdata += VLTdata['stopbyte']

    return rawdata

#U (update)
#Means that the data value, bytes (13-17), must be read into the drive.
def updateVLT(client, parameter, data):
  print('Writing to parameter:{} data:{}'.format(parameter, data))
  VLTdata['controlchar'] = b'U'
  VLTdata['ctrlstatus'] = b'0000'
  VLTdata['parameter'] = '{:>04d}'.format(parameter).encode(encoding="utf-8")
  VLTdata['data'] = '{:>05d}'.format(data).encode(encoding="utf-8")
  VLTdata['checksum'] = b'??'
  print('Sending: {}'.format(buildTelegram()))
  client.publish('mqtt_serial/tx', payload=buildTelegram(), qos=0, retain=False)

#R (read)
#Means that the master wishes to read the data value of the parameter in bytes 8 through 11.
def readVLT(client, parameter):
  print('Reading parameter:{}'.format(parameter))
  VLTdata['controlchar'] = b'R'
  VLTdata['ctrlstatus'] = b'0000'
  VLTdata['parameter'] = '{:>04d}'.format(parameter).encode(encoding="utf-8")
  VLTdata['data'] = b'00000'
  VLTdata['checksum'] = b'??'
#  print('Sending: {}'.format(buildTelegram()))
  client.publish('mqtt_serial/tx', payload=buildTelegram(), qos=0, retain=False)

#C (control)
#Means that the drive reads only the four command bytes, 4 through 7, and returns with status.
#Parameter number and data value are ignored.
def controlVLT(client, controlword):
    print('controlVLT');
    VLTdata['controlchar'] = b'C'
    VLTdata['ctrlstatus'] = controlword
    VLTdata['parameter'] = b'0000'
    VLTdata['data'] = b'00000'
    VLTdata['checksum'] = b'??'
    #print('Sending: {}'.format(buildTelegram()))
    client.publish('mqtt_serial/tx', payload=buildTelegram(), qos=0, retain=False)

#I (read index)
#Means that the drive reads the index and parameter and returns with status.
#The parameter is stated in bytes 8 through 11 and index is stated in bytes 12 through 17.
#Parameters with indices are read-only parameters. Action will be taken on the control word.
def readIndexVLT(client, parameter, index):
  VLTdata['controlchar'] = b'I'
  VLTdata['ctrlstatus'] = b'0000'
  VLTdata['parameter'] = '{:>04d}'.format(parameter).encode(encoding="utf-8")
  VLTdata['data'] = '{:>05d}'.format(index).encode(encoding="utf-8")
  VLTdata['checksum'] = b'??'
  #print('Sending: {}'.format(buildTelegram()))
  client.publish('mqtt_serial/tx', payload=buildTelegram(), qos=0, retain=False)

def printControlWord( controlword ):
  for x in range(4):
    for y in range(4):
      enabled = 1 if controlword[x] & 1<<y else 0
      print("{}.{}={}: {}".format(x, y, enabled, controlwordtext[x][y][enabled]))

def printStatusWord( statusword ):
  print("### Status Word: {} ###".format(VLTdata['ctrlstatus']))
  bitnr = 0
  for x in range(4):
    for y in range(4):
      enabled = 1 if statusword[x] & 1<<y else 0
      print("{:02d}={}: {}".format(bitnr, enabled, statuswordtext[x][y][enabled]))
      bitnr += 1

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    #print("Connected with result code "+str(rc))
    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe("mqtt_serial/rx")

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
  global received
  #print("MQTT: "+msg.topic+" "+str(msg.payload))
  decodeVLTdata( msg.payload )
  received = True

def neatPrinting( ):
    para = VLTdata['parameter'][1:].decode("utf-8")
    dat = int(VLTdata['data'])
    if VLTdata['comma'] == b'9':
      print("VLT returned: Invalid Parameter")
      return
    elif para == '502':
      #special feature, data readout
      print('Parameter {}: {}'.format(para, dat))
      return
    elif para in parameterText:
      print("Parameter {}: {}".format(para, parameterText[para]['name']))
      if 'unit' in parameterText[para]:
        print("Value: {}{}".format( float(VLTdata['data'])/10**int(VLTdata['comma']), parameterText[para]['unit'] ))
      elif 0 in parameterText[para]:
        print("Value {}: {}".format(dat, parameterText[para][dat]))
      else:
        print("Value {} unknown".format(dat))
    else:
     print("Parameter {} unknown".format(para))

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.connect("raspi", 1883, 60)
client.loop_start()

#printControlWord(startControlWord)

if (sys.argv[1] == 'read'):
    readVLT(client, int(sys.argv[2]))
elif (sys.argv[1] == 'control'):
    controlVLT(client, sys.argv[2])
elif (sys.argv[1] == 'update'):
    updateVLT(client, int(sys.argv[2]), int(sys.argv[3]))
elif (sys.argv[1] == 'index'):
    readIndexVLT(client, int(sys.argv[2]), int(sys.argv[3]))
else:
    print("Invalid arguments")
    sys.exit()

# wait for response on mqtt
while not received:
    pass

neatPrinting()
printStatusWord(VLTdata['ctrlstatus'])


client.disconnect()