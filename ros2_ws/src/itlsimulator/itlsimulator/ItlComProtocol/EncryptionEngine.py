from comConstants import ItlEncryptionOptions
from comConstants import ItlEncryptionKeys

class EncryptionEngine:
    def __init__(self, key=ItlEncryptionKeys.SHIFT_KEY_NO_1,
                  multiplier=ItlEncryptionKeys.MULTIPLY_KEY_NO_3):
        self.key = key
        self.multiplier = multiplier

    def encrypt(self, message):
        encrypted_message = ""
        for char in message:
            encrypted_char = chr((ord(char) + self.key) * self.multiplier)
            encrypted_message += encrypted_char
        return encrypted_message

    def decrypt(self, encrypted_message):
        decrypted_message = ""
        for char in encrypted_message:
            decrypted_char = chr((ord(char) // self.multiplier) - self.key)
            decrypted_message += decrypted_char
        return decrypted_message
