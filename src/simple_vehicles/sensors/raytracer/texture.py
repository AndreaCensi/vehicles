
class ConstantTexture:
    def __init__(self, value):
        self.value = value
    def __call__(self, t):
        return self.value
