
# TODO(russt): Delete this as soon as it lands in drake
#  https://github.com/RobotLocomotion/drake/issues/11191

import numpy as np

def _item_property(cls, index):
    return property(
        fget=lambda self: cls.__getitem__(self, index),
        fset=lambda self, value: cls.__setitem__(self, index, value))


def namedview(name, fields):
    # Prevent mutation.
    fields = tuple(fields)

    class NamedView(object):
        def __init__(self, obj):
            assert len(obj) == len(fields)
            object.__setattr__(self, '_obj', obj)

        @staticmethod
        def get_fields():
            return fields

        def __getitem__(self, index):
            return self._obj.__getitem__(index)

        def __setitem__(self, index, value):
            self._obj.__setitem__(index, value)

        def __setattr__(self, name, value):
            if not hasattr(self, name):
                raise AttributeError("Cannot add attributes!")
            object.__setattr__(self, name, value)

        def __len__(self):
            return self._obj.__len__

        def __iter__(self):
            return self._obj.__iter__()

        def __array__(self):
            return np.asarray(self._obj)

        def __repr__(self):
            values = [
                "{}={}".format(field, repr(self[i]))
                for i, field in enumerate(fields)]
            return "{}({})".format(name, ", ".join(values))

    NamedView.__name__ = NamedView.__qualname__ = name
    for i, field in enumerate(fields):
        setattr(NamedView, field, _item_property(NamedView, i))
    return NamedView