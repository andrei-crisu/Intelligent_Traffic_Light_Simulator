
#this are the allowed commands
class ItlCmdMessage:
    RED_STATE='COLOR_R'
    YELLOW_STATE='COLOR_Y'
    GREEN_STATE='COLOR_G'
    OFF_STATE='STATE_OFF'
    AUTO_STATE='STATE_AUTO'

#this are the security options
# are represented on 10 characters 
class ItlEncryptionOptions:

    #no encryption provided, system is vulnerable
    NO_ENCRYPTION='ITL_EY_404'

    #for this moment there is no other option provided
    STANDARD_ENCRYPTION_METHOD='ITL_EY_STD'

#encryption keys
class ItlEncryptionKeys:
    SHIFT_KEY_NO_1=17
    SHIFT_KEY_NO_2=33
    SHIFT_KEY_NO_3=47
    MULTIPLY_KEY_NO_1=3
    MULTIPLY_KEY_NO_2=5
    MULTIPLY_KEY_NO_3=7


#in this class are provided the message types
#are represented on 4 characters
class ItlMessageTypes:

    #information
    MSGT_INFORMATION='INFO'
    #command
    MSGT_COMMAND='CMDT'
    #diagnostic
    MSGT_DIAGNOSTIC='DIAG'
    #Empty message
    MSGT_EMPTY='M404'
    #Random data
    MSGT_RAND='RAND'

#provide some length constants for the field 
#that compose the message
class ItlFildsLengths:
    #length for the ID field
    ID_FIELD_LEN=12
    #length for the message type
    TYPE_FIELD_LEN=4
    #length for the field that specifies how many characters should be expected in the data field
    DATA_SIZE_FIELD_LEN=4
    #length for the security type field
    SECURITY_FIELD_LEN=10

    #number of fields that build the message
    NUMBER_OF_FIELDS=5



#provide constants for the data length bounds
class ItlDataBounds:
    LOWER_BOUND=0
    UPPER_BOUND=1000