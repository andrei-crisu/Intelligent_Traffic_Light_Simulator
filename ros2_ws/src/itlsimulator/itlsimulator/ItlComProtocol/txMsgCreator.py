import datetime
import random
import string

from comConstants import *
from EncryptionEngine import *
class MsgCreator:
    def __init__(self):
        pass
    def get_formatted_datetime(self, year, month, day, hour, minute, second):
        dt = datetime.datetime(year, month, day, hour, minute, second)
        formatted_dt = dt.strftime("%y%m%d%H%M%S")
        return formatted_dt

    def generate_random_string(self, length):
        letters = string.ascii_lowercase
        random_string = ''.join(random.choice(letters) for i in range(length))
        return random_string

    def get_formatted_current_datetime(self):
        now = datetime.datetime.now()
        formatted_dt = now.strftime("%y%m%d%H%M%S")
        return formatted_dt

    def buildItlMessage(self,dataString='EMPTY_MESSAGE_NOTHING_IMPORTANT',msgType=ItlMessageTypes.MSGT_EMPTY,
                        securityType=ItlEncryptionOptions.NO_ENCRYPTION):
        
        if "#" in dataString:
            dataString="ERR:[ __POUND_SIGN__ USED IN DATA STRING].IT IS A RESERVED SYMBOL! CANNOT BE USED"

        #this var will store the string that will be send
        msgToSend=''
        msgID=self.get_formatted_current_datetime()
        msgDataLen=str(len(dataString)).rjust(ItlFildsLengths.DATA_SIZE_FIELD_LEN," ")

        #encrypt data
        encryptionFactory=EncryptionEngine(key=ItlEncryptionKeys.SHIFT_KEY_NO_1,
                                           multiplier=ItlEncryptionKeys.MULTIPLY_KEY_NO_3)
        
        if(securityType==ItlEncryptionOptions.STANDARD_ENCRYPTION_METHOD):
            msgData=encryptionFactory.encrypt(dataString)
        elif(securityType==ItlEncryptionOptions.NO_ENCRYPTION):
            msgData=dataString
        else:
            msgData="ERR: [ON SEND UNKNOWN ENCRYPTION]:"+msgData

        msgToSend=msgID+'#'+msgType+"#"+msgDataLen+"#"+msgData+"#"+securityType

        return msgToSend




if __name__ == '__main__':
    # create an instance of the MsgCreator class
    msg_creator = MsgCreator()
    print("This is the txMsgCreator.py")

   
