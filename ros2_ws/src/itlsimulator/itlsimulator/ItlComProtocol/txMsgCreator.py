import datetime
import random
import string

class MsgCreator:
    data=''
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
    
    def type_selector(self):
        return 'I0F'
    
    def componenetFactory(self,dataString):
        msgString=''
        msgID=self.get_formatted_current_datetime()
        msgType=self.type_selector()
        msgDataLen=len(dataString)
        msgData=dataString
        msgSecurityString='$-0-$'
        msgString=msgID+'#'+msgType+"#"+str(msgDataLen)+"#"+msgData+"#"+msgSecurityString
        return msgString
    
    def getItlMessage(self,dataString='0xDEF'):
        msgString=''
        if dataString=='0xDEF':
            msgDataLen=random.randint(40,50)
            msgString=self.componenetFactory(self.generate_random_string(msgDataLen))
        else:
            msgString=self.componenetFactory(dataString)
        
        #here the data field is set 
        self.data=msgString
        return self.data




if __name__ == '__main__':
    # create an instance of the MsgCreator class
    msg_creator = MsgCreator()

    # call the get_formatted_datetime method
    current_year = 2023
    current_month = 4
    current_day = 11
    current_hour = 10
    current_minute = 30
    current_second = 15
    formatted_datetime = msg_creator.get_formatted_datetime(current_year, current_month, current_day, current_hour, current_minute, current_second)
    print(f"Formatted datetime: {formatted_datetime}")

    # call the generate_random_string method
    random_string_length = 10
    random_string = msg_creator.generate_random_string(random_string_length)
    print(f"Random string: {random_string}")

    # call the get_formatted_current_datetime method
    formatted_current_datetime = msg_creator.get_formatted_current_datetime()
    print(f"Formatted current datetime: {formatted_current_datetime}")
