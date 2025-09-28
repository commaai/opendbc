AddOption('--minimal',
          action='store_false',
          dest='extras',
          default=True,
          help='the minimum build. no tests, tools, etc.')

AddOption('--ubsan',
          action='store_true',
          help='turn on UBSan')

AddOption('--mutation',
          action='store_true',
          help='generate mutation-ready code')

SConscript(['SConscript'])
