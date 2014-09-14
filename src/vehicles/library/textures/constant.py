from vehicles import Texture

__all__ = ['ConstantTexture', 'constant_texture']

# TODO: remove this
def constant_texture(color):
    return ['vehicles.library.textures.ConstantTexture', dict(value=color)]


class ConstantTexture(Texture):
    def __init__(self, value):
        self.value = value

    def __call__(self, t):
        return t * 0 + self.value
