class magic(object):
    the_gathering = dict()

    def __new__(cls, imported, function_name="filter_image", msg_generator_name="generate_msg"):
        try:
            recall = magic.the_gathering[imported.__name__]
            return recall
        except KeyError:
            methods = [method for method in dir(imported) if callable(getattr(imported, method))]
            instance = object.__new__(cls)
            instance.filter_function = getattr(imported, function_name) if function_name in methods else lambda (x,y) : x
            instance.msg_generator = getattr(imported,msg_generator_name) if msg_generator_name in methods else lambda : ""
            magic.the_gathering[imported.__name__] = instance
            return instance

