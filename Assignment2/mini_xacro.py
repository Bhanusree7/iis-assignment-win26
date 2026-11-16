"""Minimal xacro expander (property / macro / include / ${expr}).
Fallback for machines without the official ROS `xacro` package.
process(path, overrides={"link1_length": 0.5}) -> URDF string."""
import math, os
import re
import xml.etree.ElementTree as ET

NS = "{http://www.ros.org/wiki/xacro}"
_EXPR = re.compile(r"\$\{([^}]*)\}")


def _fmt(v):
    if isinstance(v, bool): return str(v)
    if isinstance(v, (int, float)):
        return str(int(v)) if float(v).is_integer() else repr(round(float(v), 10))
    return str(v)


def _ev(s, scope):
    env = {k: getattr(math, k) for k in dir(math) if not k.startswith("_")}
    env.update(scope)
    return _EXPR.sub(lambda m: _fmt(eval(m.group(1), {"__builtins__": {}}, env)), s)


def _val(s):
    try: return float(s)
    except ValueError: return s


class _Ctx:
    def __init__(self, base_dir, overrides):
        self.base_dir, self.overrides = base_dir, overrides or {}
        self.macros = {}

    def load(self, path):
        root = ET.parse(path).getroot()
        for e in root.iter(NS + "property"):
            if e.get("name") in self.overrides:          # parameter override (like xacro arg)
                e.set("value", str(self.overrides[e.get("name")]))
        return root

    def expand(self, elem, scope):
        out = []
        for ch in list(elem):
            t = ch.tag
            if t == NS + "property":
                scope[ch.get("name")] = _val(_ev(ch.get("value"), scope))
            elif t == NS + "include":
                out += self.expand(self.load(os.path.join(self.base_dir, ch.get("filename"))), scope)
            elif t == NS + "macro":
                self.macros[ch.get("name")] = ch
            elif t.startswith(NS):                       # macro call
                m = self.macros[t[len(NS):]]
                local = dict(scope)
                for p in m.get("params").split():
                    local[p] = _val(_ev(ch.get(p), scope))
                out += self.expand(m, local)
            else:
                new = ET.Element(t, {k: _ev(v, scope) for k, v in ch.attrib.items()})
                if ch.text and ch.text.strip(): new.text = _ev(ch.text, scope)
                new.extend(self.expand(ch, scope))
                out.append(new)
        return out


def process(path, overrides=None):
    ctx = _Ctx(os.path.dirname(os.path.abspath(path)), overrides)
    root = ctx.load(path)
    robot = ET.Element("robot", {"name": root.get("name", "robot")})
    robot.extend(ctx.expand(root, {}))
    ET.indent(robot)
    return ET.tostring(robot, encoding="unicode")
