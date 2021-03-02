from .when_secs_since_surface import WhenSecsSinceSurfaceHandler


EVENT_HANDLER_CLASSES = [WhenSecsSinceSurfaceHandler]

EVENT_HANDLER_CLASS_MAP = {cls.EVENT_NAME: cls
                           for cls in EVENT_HANDLER_CLASSES}


__all__ = (['EVENT_HANDLER_CLASSES', 'event_handler_class_for_name']
           + [cls.__name__ for cls in EVENT_HANDLER_CLASSES])


def event_handler_class_for_name(name):
    return EVENT_HANDLER_CLASS_MAP[name]
