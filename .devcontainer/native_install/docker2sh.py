#!/usr/bin/env python3
"""
Dockerfile parser module based on https://github.com/dyne/docker2sh
"""
from argparse import ArgumentParser
from base64 import b64encode
from bz2 import compress
from os.path import dirname, join
from sys import stdin
import json
import re


def rstrip_backslash(line):
    """
    Strip backslashes from end of line
    """
    line = line.rstrip()
    if line.endswith('\\'):
        return line[:-1]
    return line


def get_stages_to_target(data, target, stagere):
    """
    Find which steps we need to build the target
    """
    required_stages = [target]
    MAX_DEPTH = 100

    for _ in range(MAX_DEPTH):
        parent_stage_found = False
        for line in data:
            if stages := stagere.match(line):
                if stages.group("child") == required_stages[-1]:
                    required_stages.append(stages["parent"])
                    parent_stage_found = True
                    break
        if not parent_stage_found:
            return required_stages[:-1]


def parse_instruction(inst, dfile=None):
    """
    Method for translating Dockerfile instructions to shell script
    """
    ins = inst['instruction'].upper()
    val = inst['value']

    # Valid Dockerfile instructions
    cmds = ['ADD', 'ARG', 'CMD', 'COPY', 'ENTRYPOINT', 'ENV', 'EXPOSE', 'FROM',
            'HEALTHCHECK', 'LABEL', 'MAINTAINER', 'ONBUILD', 'RUN', 'SHELL',
            'STOPSIGNAL', 'USER', 'VOLUME', 'WORKDIR']

    if ins == 'ADD':
        val = val.replace('$', '\\$')
        args = val.split(' ')
        return 'wget -O %s %s\n' % (args[1], args[0])

    if ins == 'ARG':
        return '%s\n' % val

    if ins == 'ENV':
        if '=' not in val:
            val = val.replace(' ', '=', 1)
        val = val.replace('$', '\\$')
        return 'export %s\n' % val

    if ins == 'RUN':
        # Replace `` with $()
        while '`' in val:
            val = val.replace('`', '"$(', 1)
            val = val.replace('`', ')"', 1)
        return val + '\n'

    if ins == 'WORKDIR':
        return 'mkdir -p %s && cd %s\n' % (val, val)

    if ins in cmds:
        # TODO: Look at CMD being added to /etc/rc.local
        return '#\n# %s not implemented\n# Instruction: %s %s\n#\n' % \
            (ins, ins, val)

    # Silently ignore unknown instructions
    return ''


def main():
    """
    Main parsing routine
    """
    parser = ArgumentParser()
    parser.add_argument('-j', '--json', action='store_true',
                        help='output the data as a JSON structure')
    parser.add_argument('-s', '--shell', action='store_true',
                        help='output the data as a shell script (default)')
    parser.add_argument('--keeptabs', action='store_true',
                        help='do not replace \\t (tabs) in the strings')
    parser.add_argument('--target', action='store',
                        help='which target to build')
    parser.add_argument('Dockerfile')
    args = parser.parse_args()

    if args.Dockerfile != '-':
        with open(args.Dockerfile) as file:
            data = file.read().splitlines()
    else:
        data = stdin.read().splitlines()

    instre = re.compile(r'^\s*(\w+)\s+(.*)$')
    contre = re.compile(r'^.*\\\s*$')
    commentre = re.compile(r'^\s*#')
    stagere = re.compile(r'^\s*FROM\s*(?P<parent>\S+)\s*AS\s*(?P<child>\S+)')

    required_stages = get_stages_to_target(data, args.target, stagere)
    

    instructions = []
    lineno = -1
    in_continuation = False
    cur_inst = {}
    current_stage = ""

    for line in data:
        lineno += 1
        if line_stages:=stagere.match(line):
            current_stage = line_stages.group("child")
        if commentre.match(line):
            continue
        if not in_continuation:
            rematch = instre.match(line)
            if not rematch:
                continue
            cur_inst = {
                'instruction': rematch.groups()[0].upper(),
                'value': rstrip_backslash(rematch.groups()[1]),
            }
        else:
            if cur_inst['value']:
                cur_inst['value'] += rstrip_backslash(line)
            else:
                cur_inst['value'] = rstrip_backslash(line.lstrip())

        in_continuation = contre.match(line)
        if not in_continuation and cur_inst is not None and (current_stage in required_stages or args.target is None):
            if not args.keeptabs:
                cur_inst['value'] = cur_inst['value'].replace('\t', '')
            instructions.append(cur_inst)

    if args.json:
        print(json.dumps(instructions))
        return

    # Default to shell script output
    script = '#!/bin/sh\nset -e\n'
    for i in instructions:
        script += parse_instruction(i, dfile=args.Dockerfile)
    print(script)


if __name__ == '__main__':
    main()
