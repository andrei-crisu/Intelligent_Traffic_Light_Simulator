from comConstants import*
from  EncryptionEngine import *

class MsgExtract:

    def __init__(self):
        self.data=""
        self.id=""
        self.data_type=ItlMessageTypes.MSGT_INFORMATION
    
    def clearExtractor(self):
        self.data=""
        self.id=""
        self.data_type=ItlMessageTypes.MSGT_INFORMATION
    

    def __checkMessageType(self,msg_type):
        if len(msg_type)!=ItlFildsLengths.TYPE_FIELD_LEN:
                return False
        if (msg_type==ItlMessageTypes.MSGT_COMMAND or
            msg_type==ItlMessageTypes.MSGT_DIAGNOSTIC or
            msg_type==ItlMessageTypes.MSGT_EMPTY or
            msg_type==ItlMessageTypes.MSGT_INFORMATION or
            msg_type==ItlMessageTypes.MSGT_RAND):
            return True
        else:
            return False


    def extractItlMessage(self,message_to_extract=""):
        #remove first encryption
        secondEncryption=EncryptionEngine(key=ItlEncryptionKeys.SHIFT_KEY_NO_2,
                                          multiplier=ItlEncryptionKeys.MULTIPLY_KEY_NO_1)
        message_to_extract=secondEncryption.decrypt(message_to_extract)

        mesg_components=message_to_extract.split("#")
        if len(mesg_components)!=ItlFildsLengths.NUMBER_OF_FIELDS:
            return False
        else:
            msgID=mesg_components[0]
            if len(msgID)!= ItlFildsLengths.ID_FIELD_LEN:
                return False
            
            msgType=mesg_components[1]
            if(self.__checkMessageType(msgType)==False):
                return False
            
            msgDataLen=mesg_components[2]
            if len(msgDataLen)!=ItlFildsLengths.DATA_SIZE_FIELD_LEN:
                return False
            int_msgDataLen=int(msgDataLen)
            if int_msgDataLen<ItlDataBounds.LOWER_BOUND or int_msgDataLen>ItlDataBounds.UPPER_BOUND:
                return False
            
            msgData=mesg_components[3]
            if len(msgData)!=int_msgDataLen:
                return False
            
            securityType=mesg_components[4]
            if len(securityType)!=ItlFildsLengths.SECURITY_FIELD_LEN:
                return False
        #decrypt data
        encryptionFactory=EncryptionEngine(key=ItlEncryptionKeys.SHIFT_KEY_NO_1,
                                           multiplier=ItlEncryptionKeys.MULTIPLY_KEY_NO_3)
        
        if(securityType==ItlEncryptionOptions.STANDARD_ENCRYPTION_METHOD):
            decrypted_msgData=encryptionFactory.decrypt(msgData)
            self.data=decrypted_msgData
            self.data_type=msgType
            self.id=msgID
        elif(securityType==ItlEncryptionOptions.NO_ENCRYPTION):
            self.data=msgData
            self.data_type=msgType
            self.id=msgID
        else:
            self.data="ERR:[ON RECEPTION UNABLE TO DECRYPT]: "+msgData
            return False
        
        return True


