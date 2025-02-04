import logging

# set up logging
carlog = logging.getLogger('carlog')
carlog.setLevel(logging.INFO)
carlog.propagate = False

handler = logging.StreamHandler()
handler.setFormatter(logging.Formatter('%(message)s'))
carlog.addHandler(handler)
