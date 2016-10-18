#!/usr/bin/python
# -*- coding: utf-8 -*-

import re

class Parser(object):
    # Expression for the eat_whitespace function
    white_regx  = re.compile(r"\s+")
    # Expressions for the input language
    vertex_regx = re.compile(r"(?P<name>\w+_\w+):\s*")
    never_regx  = re.compile(r"never \{ /\*(?P<formula>.+)\*/")
    if_regex    = re.compile(r"if")
    edge_regex  = re.compile(r":: (?P<cond>\(.*\)) -> goto (?P<dest>\w+_\w+)")
    fi_regex    = re.compile(r"fi;")
    skip_regex  = re.compile(r"skip")
    end_regex   = re.compile(r"\}")

    def __init__(self, instring):
        self.instring = instring
        self.pos = 0

    def eat_whitespace(self):
        match = Parser.white_regx.match(self.instring, self.pos)
        while (match != None):
            self.pos += len(match.group(0))
            match = Parser.white_regx.match(self.instring, self.pos)

    def accept(self, expr, strip_whitespace=True):
        if strip_whitespace:
            self.eat_whitespace()
        match = expr.match(self.instring, self.pos)
        if (match == None):
            return None
        self.pos += len(match.group(0))
        return match.groupdict()

    def parse(self):
        edges = {}
        self.formula = self.accept(Parser.never_regx)["formula"]
        vertex = self.accept(Parser.vertex_regx)
        while (vertex != None):
            vertex_name = vertex["name"]
            if (self.accept(Parser.if_regex) != None):
                edge = self.accept(Parser.edge_regex)
                while (edge != None):
                    edges[(vertex_name, edge["dest"])] = edge["cond"]
                    edge = self.accept(Parser.edge_regex)
                self.accept(Parser.fi_regex)
            elif (self.accept(Parser.skip_regex) != None):
                # self-loop with "skip"
                edges[(vertex_name, vertex_name)] = '1'
            else:
                raise ParseException("Expected 'if' or 'skip' but got %s" % self.instring[self.pos])
            vertex = self.accept(Parser.vertex_regx)
        self.accept(Parser.end_regex)
        self.eat_whitespace()
        if (self.pos != len(self.instring)):
            raise ParseException("Input not fully parsed. Remainder: %s" % self.instring[self.pos:])
        return edges

class ParseException(Exception):
    pass

def parse(promela):
    parser = Parser(promela)
    return parser.parse()

def find_states(edges):
    states = set()
    initial = set()
    accept = set()
    for (f,t) in edges.keys():
        states.add(f)
        states.add(t)
    for state in states:
        if state.startswith("accept"):
            accept.add(state)
        if state.endswith("init"):
            initial.add(state)
    return (list(states), list(initial), list(accept))

def find_symbols(formula):
    regex = re.compile(r"[a-z]+[a-z0-9]*")
    matches = regex.findall(formula)
    symbols = list()
    for match in matches:
        symbols += [match]
    symbols = list(set(symbols))
    symbols.sort()
    return symbols
