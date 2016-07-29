from txros import util


@util.cancellableInlineCallbacks
def run(sub):
    yield sub.move.forward(3).go()