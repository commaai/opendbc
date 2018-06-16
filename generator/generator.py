#!/usr/bin/env python2
import os
import re

def read_dbc(filename):
    with open(os.path.join(dir_name, filename), 'r') as file_in:
        return file_in.read()
  
def create_dbc(filename):
    dbc_file_in = read_dbc(filename) 

    includes = include_pattern.findall(dbc_file_in)
 
    output_filename = filename.replace('.dbc', '_generated.dbc')
    output_file_location = os.path.join(generator_path, output_filename)
    
    if os.path.isfile(output_file_location):
        os.remove(output_file_location)
      
    with open(output_file_location, 'a') as dbc_file_out:
        dbc_file_out.write('CM_ "AUTOGENERATED FILE, DO NOT EDIT"\n')

        for include_filename in reversed(includes):

            include_file_header = '\n\nCM_ "Imported file %s starts here"\n' % include_filename
            dbc_file_out.write(include_file_header)

            include_file = read_dbc(include_filename)
            dbc_file_out.write(include_file)

        dbc_file_out.write('\nCM_ "%s starts here"\n' % filename)
      
        core_dbc = include_pattern.sub('', dbc_file_in)
        dbc_file_out.write(core_dbc)
   
cur_path = os.path.dirname(os.path.realpath(__file__))
generator_path = os.path.join(cur_path, '../')

include_pattern = re.compile(r'CM_ "IMPORT (.*?)"')

for dir_name, _, filenames in os.walk(cur_path):
    if dir_name == cur_path:
        continue

    print dir_name

    for filename in filenames:
        if filename.startswith('_'):
            continue

        print filename
        create_dbc(filename)
