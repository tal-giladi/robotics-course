"""CPython stand-in for MicroPython's ``micropython`` module (tests only)."""


def const(value):
    return value


def native(function):
    return function


def viper(function):
    return function


def schedule(function, argument):
    function(argument)


def alloc_emergency_exception_buf(size):
    pass


def kbd_intr(char):
    pass
