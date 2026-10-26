"""Small, method-local descriptions of observable coverage and host residuals."""


def coverage(claim, *, host=(), relation='narrower', uses=()):
  """Attach metadata without wrapping tests or treating CAN traffic as proof."""
  supplements = tuple(dict.fromkeys((*host, *(item for method in uses for item in method.safety_coverage['host_supplements']))))

  def decorate(method):
    method.safety_coverage = {'claim': claim, 'relation': relation, 'host_supplements': supplements}
    return method
  return decorate


def method_coverage(test):
  method = getattr(test, test._testMethodName, None)
  # add_regen_tests uses wraps to preserve attributes. Resolve the delegated
  # brake method again so subclass overrides do not inherit a stale contract.
  if 'regen' in test._testMethodName and hasattr(method, '__wrapped__'):
    method = getattr(test, method.__wrapped__.__name__, None)
  metadata = getattr(method, 'safety_coverage', None)
  if metadata is None:
    return {'claim': None, 'relation': 'unclassified', 'host_supplements': []}
  return {**metadata, 'host_supplements': list(metadata['host_supplements'])}
