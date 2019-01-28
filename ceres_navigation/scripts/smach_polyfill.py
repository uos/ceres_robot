from smach import CBInterface as SmachCBInterface
from smach import cb_interface as smach_cb_interface


class CBInterface(SmachCBInterface):
    """
    Support for decorated methods (instead of pure functions) as callbacks. Takes care of binding the instance
    to this object and passing it to the method as the self parameter.
    """
    def __init__(self, cb, outcomes=[], input_keys=[], output_keys=[],
                 io_keys=[], _instance=None):
        """Save ths isntance"""
        SmachCBInterface.__init__(self, cb, outcomes, input_keys, output_keys, io_keys)
        self._instance = _instance

    def __get__(self, obj, type=None):
        """Caputes the instance. Returns a new object with the instance assigned."""
        return CBInterface(self._cb, outcomes=self._outcomes, input_keys=self._input_keys, output_keys=self._output_keys, _instance=obj)

    def __call__(self, *args, **kwargs):
        """Executes the callback. If an instance is given, it will be passed for the this parameter"""
        if self._instance is not None:
            return self._cb(self._instance, *args, **kwargs)
        else:
            return self._cb(*args, **kwargs)

class cb_interface(smach_cb_interface):
    """Modified interface, so the new CBInterface from this polyfill will be used"""
    def __init__(self, *args, **kwargs):
        smach_cb_interface.__init__(self, *args, **kwargs)

    def __call__(self, cb):
        return CBInterface(cb, self._outcomes, self._input_keys, self._output_keys)