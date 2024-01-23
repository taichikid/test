import configparser
import sys

args = sys.argv
program = args[1]

config = configparser.ConfigParser()
config.read('params.ini')

if args[1] == 'show':
    print('takeoff', config['INPUT']['takeoff'])
    print('takeofftime', config['INPUT']['takeofftime'])
    print('hov', config['INPUT']['hov'])
    print('hovtime', config['INPUT']['hovtime'])
    print('forward', config['INPUT']['forward'])
    print('forwardtime', config['INPUT']['forwardtime'])
else:
    config['INPUT'][program] = args[2]
    with open('params.ini', 'w') as f:
        config.write(f)