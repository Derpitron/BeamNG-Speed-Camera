#!/usr/bin/env python3
"""
generate_beamng_stubs_ast.py

BeamNG -> Emmy-style stub generator for Sumneko/Lua language server.

Features:
- Robust imports (works with several luaparser versions)
- Detects ffi.cdef / ffi.metatype mappings (e.g. __luaVec3_t -> LuaVec3)
- Extracts functions, metamethods, table constructors
- Infers returns when function returns newLuaVec3xyz(...) or newLuaQuatxyzw(...) or `self`
- Emits `---@class`, `---@operator`, `---@param`, `---@return` and `function` stubs
- Writes stubs into `.vscode/beamng_stubs` and updates `.vscode/settings.json`
"""
from __future__ import annotations

import argparse
import json
import os
import re
import sys
from collections import defaultdict
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

# ---------------- Compatibility import for luaparser ----------------
# Try multiple import shapes and provide fallbacks. Do NOT require AST base class.
try:
    # prefer new-style import
    from luaparser import ast as luaparser_ast

    try:
        # try to import node classes
        from luaparser.astnodes import (
            Assign,
            Call,
            Function,
            Index,
            LocalAssign,
            Name,
            Return,
            String,
            Table,
            Vararg,
        )
    except Exception:
        # fallback: some luaparser versions expose node classes under same ast module
        Assign = getattr(luaparser_ast, "Assign", None)
        Call = getattr(luaparser_ast, "Call", None)
        Function = getattr(luaparser_ast, "Function", None)
        Index = getattr(luaparser_ast, "Index", None)
        LocalAssign = getattr(luaparser_ast, "LocalAssign", None)
        Name = getattr(luaparser_ast, "Name", None)
        Return = getattr(luaparser_ast, "Return", None)
        String = getattr(luaparser_ast, "String", None)
        Table = getattr(luaparser_ast, "Table", None)
        Vararg = getattr(luaparser_ast, "Vararg", None)
except Exception as e:
    raise ImportError(
        "luaparser import failed. Install with: python3 -m pip install luaparser"
    ) from e

# Parser entry (some versions have parse at module root)
_parse = getattr(luaparser_ast, "parse", None)
if _parse is None:
    # older versions expose 'parser' with 'parse' function
    _parse = getattr(luaparser_ast, "parser", None)
    if _parse is not None and hasattr(_parse, "parse"):
        _parse = getattr(_parse, "parse")
if _parse is None:
    raise ImportError("Couldn't find parse() in luaparser package. Update luaparser.")


# Helper: safe isinstance (since node class variables may be None)
def is_instance(node: Any, cls: Any) -> bool:
    if cls is None:
        return False
    try:
        return isinstance(node, cls)
    except Exception:
        return False

# ---------------- robust AST helpers (replace existing is_ast_node / iter_fields_safe / walk) ----------------

def is_ast_node(obj: Any) -> bool:
    """
    Heuristic test for whether obj is an AST node coming from luaparser.
    """
    if obj is None:
        return False
    # Typical AST nodes have __dict__ and location info (lineno or start/finish)
    if hasattr(obj, "__dict__") and (hasattr(obj, "lineno") or (hasattr(obj, "start") and hasattr(obj, "finish"))):
        return True
    # sometimes node classes don't have lineno; accept objects with __dict__ but exclude plain containers
    return hasattr(obj, "__dict__") and not isinstance(obj, (str, bytes, bytearray, list, tuple, dict, set))

# fields to skip because they commonly contain back-references that create cycles
_CYCLE_SKIP_FIELDS = {
    "parent", "_parent", "env", "_env", "scope", "_scope", "prev", "next", "upvalue",
    "chunk", "function", "block", "module"
}

def iter_fields_safe(node: Any):
    """
    Iterate over meaningful fields of a node in a defensive manner.
    Skips common cycle-causing attribute names and catches attribute access errors.
    """
    if node is None:
        return
    # prefer __dict__ when available (faster, predictable)
    try:
        for name, val in vars(node).items():
            if name in _CYCLE_SKIP_FIELDS:
                continue
            yield name, val
        return
    except Exception:
        pass

    # fallback: dir() with try/except
    for name in dir(node):
        if name.startswith("__") or name in _CYCLE_SKIP_FIELDS:
            continue
        try:
            val = getattr(node, name)
        except Exception:
            continue
        yield name, val

def walk(root: Any):
    """
    Iterative preorder traversal over AST nodes that is cycle-safe.
    Yields (node, parent).
    Uses an explicit stack and a visited set of node ids to avoid infinite recursion.
    """
    if root is None:
        return
    stack = [(root, None)]
    visited = set()
    while stack:
        node, parent = stack.pop()
        nid = id(node)
        if nid in visited:
            continue
        visited.add(nid)
        yield node, parent

        # collect children to push onto the stack (we iterate in reverse so that order resembles recursive preorder)
        children = []
        for _, value in iter_fields_safe(node):
            # lists of nodes
            if isinstance(value, list):
                for item in value:
                    if is_ast_node(item):
                        children.append((item, node))
            # single node-like object
            elif is_ast_node(value):
                children.append((value, node))
            # ignore other types
        # push children (reverse to preserve original order)
        for ch in reversed(children):
            stack.append(ch)


# ---------------- utilities ----------------
def log(*a, **k):
    print(*a, file=sys.stderr, **k)


def safe_read_text(p: Path) -> str:
    try:
        return p.read_text(encoding="utf-8", errors="ignore")
    except Exception:
        return ""


def ensure_dir(p: Path):
    p.mkdir(parents=True, exist_ok=True)


def write_if_changed(path: Path, content: str) -> bool:
    if path.exists():
        try:
            old = path.read_text(encoding="utf-8")
            if old == content:
                return False
        except Exception:
            pass
    path.write_text(content, encoding="utf-8")
    return True


# ---------------- name helpers ----------------
def name_from_node(node: Optional[Any]) -> Optional[str]:
    if node is None:
        return None
    if is_instance(node, Name):
        return getattr(node, "id", None)
    if is_instance(node, Index):
        left = name_from_node(getattr(node, "value", None))
        right = None
        idx = getattr(node, "idx", None)
        if is_instance(idx, String):
            right = getattr(idx, "s", None)
            if right:
                right = right.strip('"').strip("'")
        elif is_instance(idx, Name):
            right = getattr(idx, "id", None)
        if left and right:
            return f"{left}.{right}"
        return left or right
    # fallback: dotted attributes e.g., node.value.attr in some ASTs
    if hasattr(node, "attr") and hasattr(node, "value"):
        left = name_from_node(getattr(node, "value", None))
        attr = getattr(node, "attr", None)
        if left and attr:
            return f"{left}.{attr}"
        return attr or left
    return None


def params_from_argnode(argnode) -> List[str]:
    if argnode is None:
        return []
    seq = None
    if hasattr(argnode, "args"):
        seq = getattr(argnode, "args")
    elif isinstance(argnode, list):
        seq = argnode
    else:
        seq = [argnode]
    params: List[str] = []
    for a in seq:
        if is_instance(a, Name):
            params.append(getattr(a, "id", "arg"))
        elif is_instance(a, Vararg):
            params.append("...")
        else:
            try:
                if hasattr(a, "id"):
                    params.append(str(getattr(a, "id")))
                else:
                    params.append("arg")
            except Exception:
                params.append("arg")
    return params


# ---------------- Extractor ----------------
class BeamNGExtractor:
    def __init__(self, code: str):
        self.code = code
        try:
            self.tree = _parse(code)
        except Exception as e:
            self.tree = None
            self._parse_error = str(e)
        # collected data
        # functions: list of (fullname, params, is_method, is_local)
        self.functions: List[Tuple[str, List[str], bool, bool]] = []
        self.tables = set()
        # ffi maps
        self.ffi_structs: Dict[str, str] = {}  # cstruct -> alias (from cdef)
        self.ffi_metatypes: Dict[str, str] = {}  # cstruct -> alias (from metatype)
        # metamethods: alias -> list of (metaname, params)
        self.metamethods: Dict[str, List[Tuple[str, List[str]]]] = defaultdict(list)
        # func_returns: fullname -> return alias (e.g. "LuaVec3")
        self.func_returns: Dict[str, str] = {}

    def _guess_alias_for_struct(self, sname: str) -> str:
        s = sname.lower()
        if "vec3" in s:
            return "LuaVec3"
        if "quat" in s:
            return "LuaQuat"
        parts = [p for p in re.split(r"[_\W]+", sname) if p]
        return "".join(p.capitalize() for p in parts) or sname

    def extract(self):
        if self.tree is None:
            return
        # pass1: collect table assignments, ffi.cdef/metatype
        for node, parent in walk(self.tree):
            if is_instance(node, Assign):
                targets = getattr(node, "targets", None)
                values = getattr(node, "values", None)
                if (
                    targets
                    and values
                    and len(values) > 0
                    and is_instance(values[0], Table)
                ):
                    for t in targets:
                        tname = name_from_node(t)
                        if tname:
                            self.tables.add(tname)
            if is_instance(node, LocalAssign):
                targets = getattr(node, "targets", None)
                values = getattr(node, "values", None)
                if (
                    targets
                    and values
                    and len(values) > 0
                    and is_instance(values[0], Table)
                ):
                    for t in targets:
                        if hasattr(t, "id"):
                            self.tables.add(getattr(t, "id"))
            if is_instance(node, Call):
                fname = name_from_node(getattr(node, "func", None))
                # ffi.cdef("struct NAME { ... }")
                if fname == "ffi.cdef":
                    args = getattr(node, "args", None) or []
                    if len(args) > 0 and is_instance(args[0], String):
                        txt = getattr(args[0], "s", "")
                        for m in re.finditer(r"struct\s+([A-Za-z0-9_]+)\s*{", txt):
                            sname = m.group(1)
                            alias = self._guess_alias_for_struct(sname)
                            self.ffi_structs[sname] = alias
                # ffi.metatype("struct NAME", LuaVec3)
                if fname == "ffi.metatype":
                    args = getattr(node, "args", None) or []
                    if len(args) >= 2 and is_instance(args[0], String):
                        sname = getattr(args[0], "s", "").strip('"').strip("'")
                        if sname.startswith("struct "):
                            sname = sname.split(" ", 1)[1]
                        a1 = args[1]
                        if is_instance(a1, Name):
                            alias = getattr(a1, "id", None)
                            if alias:
                                self.ffi_metatypes[sname] = alias

        # pass2: functions, metamethods, returns
        for node, parent in walk(self.tree):
            # Function definitions (function foo.bar(...) end)
            if is_instance(node, Function):
                fname = None
                if hasattr(node, "name") and getattr(node, "name", None):
                    fname = name_from_node(getattr(node, "name", None))
                params = params_from_argnode(getattr(node, "args", None))
                is_method = getattr(node, "is_method", False) or (
                    len(params) > 0 and params[0] == "self"
                )
                if fname:
                    self.functions.append((fname, params, is_method, False))
                    ret_alias = self._infer_return_from_function(node)
                    if ret_alias:
                        self.func_returns[fname] = ret_alias
                continue

            # Assignments: foo = function(...) end or foo.bar = function(...)
            if is_instance(node, Assign) and getattr(node, "values", None):
                vals = getattr(node, "values", None)
                if vals and len(vals) > 0:
                    val0 = vals[0]
                    if is_instance(val0, Function):
                        params = params_from_argnode(getattr(val0, "args", None))
                        targets = getattr(node, "targets", None) or []
                        for t in targets:
                            tname = name_from_node(t)
                            if not tname:
                                continue
                            # metamethod on class: LuaVec3.__add = function(a,b) end
                            if "." in tname:
                                left, right = tname.rsplit(".", 1)
                                if right.startswith("__"):
                                    cls = left.split(".")[-1]
                                    self.metamethods[cls].append((right, params))
                                    self.functions.append((tname, params, False, False))
                                    ret_alias = self._infer_return_from_function(val0)
                                    if ret_alias:
                                        self.func_returns[tname] = ret_alias
                                    continue
                            # normal function assignment
                            is_method = params and params[0] == "self"
                            self.functions.append((tname, params, is_method, False))
                            ret_alias = self._infer_return_from_function(val0)
                            if ret_alias:
                                self.func_returns[tname] = ret_alias
                            continue
                    # Table constructor assigned: mt = { __add = function(...) end }
                    if is_instance(val0, Table):
                        fields = getattr(val0, "fields", []) or []
                        for field in fields:
                            try:
                                key = None
                                valnode = None
                                if hasattr(field, "key") and hasattr(field, "value"):
                                    k = field.key
                                    if is_instance(k, String):
                                        key = getattr(k, "s", "").strip('"').strip("'")
                                    elif is_instance(k, Name):
                                        key = getattr(k, "id", None)
                                    valnode = field.value
                                if (
                                    key
                                    and key.startswith("__")
                                    and is_instance(valnode, Function)
                                ):
                                    params = params_from_argnode(
                                        getattr(valnode, "args", None)
                                    )
                                    targets = getattr(node, "targets", None) or []
                                    for t in targets:
                                        tname = name_from_node(t)
                                        if tname:
                                            cls = tname.split(".")[-1]
                                            self.metamethods[cls].append((key, params))
                            except Exception:
                                pass

            # LocalAssign that assigns a function: local foo = function(...) end
            if is_instance(node, LocalAssign) and getattr(node, "values", None):
                vals = getattr(node, "values", None)
                if vals and len(vals) > 0 and is_instance(vals[0], Function):
                    val0 = vals[0]
                    params = params_from_argnode(getattr(val0, "args", None))
                    for t in getattr(node, "targets", []) or []:
                        if hasattr(t, "id"):
                            self.functions.append(
                                (getattr(t, "id"), params, False, True)
                            )
                            ret_alias = self._infer_return_from_function(val0)
                            if ret_alias:
                                self.func_returns[getattr(t, "id")] = ret_alias

    def _infer_return_from_function(self, func_node: Any) -> Optional[str]:
        """
        Walk the function body and look for return expressions that reveal the return type:
         - return newLuaVec3xyz(...) -> LuaVec3
         - return newLuaQuatxyzw(...) -> LuaQuat
         - return self -> (assume LuaVec3 as common)
        """
        vec_ctor = "newLuaVec3xyz"
        quat_ctor = "newLuaQuatxyzw"
        for n, _ in walk(func_node):
            if is_instance(n, Return):
                vals = getattr(n, "values", []) or []
                for v in vals:
                    if is_instance(v, Call):
                        f = name_from_node(getattr(v, "func", None))
                        if f == vec_ctor:
                            return "LuaVec3"
                        if f == quat_ctor:
                            return "LuaQuat"
                    if is_instance(v, Name) and getattr(v, "id", None) == "self":
                        # best-effort assume methods returning self are LuaVec3
                        return "LuaVec3"
        return None

    def summary(self) -> Dict[str, Any]:
        merged = dict(self.ffi_structs)
        merged.update(self.ffi_metatypes)
        return {
            "functions": list(self.functions),
            "tables": sorted(self.tables),
            "ffi_map": merged,
            "metamethods": dict(self.metamethods),
            "func_returns": dict(self.func_returns),
        }


# ---------------- stub rendering ----------------
def class_name_from_leaf(leaf: str) -> str:
    s = Path(leaf).stem
    parts = [p for p in re.split(r"[_\W]+", s) if p]
    return "".join(p.capitalize() for p in parts) or "Tbl"


def render_stub_for_file(
    src: Path, src_root: Path, out_root: Path, extractor: BeamNGExtractor
) -> str:
    header = f"-- AUTO-GENERATED STUB for {src}\n-- Regenerate with tools/generate_beamng_stubs_ast.py\n\n"
    s = extractor.summary()
    lines: List[str] = [header]

    ffi_map = s["ffi_map"]
    metamethods = s.get("metamethods", {})
    func_returns = s.get("func_returns", {})

    # Emit classes for ffi structs/metatypes
    for cstruct, alias in ffi_map.items():
        if alias == "LuaVec3":
            lines.append(f"---@class {alias}")
            lines.append(f"---@field x number")
            lines.append(f"---@field y number")
            lines.append(f"---@field z number")
            lines.append(f"local {alias} = {{}}\n")
            lines.append(f"---@alias {cstruct} {alias}\n")
        elif alias == "LuaQuat":
            lines.append(f"---@class {alias}")
            lines.append(f"---@field x number")
            lines.append(f"---@field y number")
            lines.append(f"---@field z number")
            lines.append(f"---@field w number")
            lines.append(f"local {alias} = {{}}\n")
            lines.append(f"---@alias {cstruct} {alias}\n")
        else:
            lines.append(f"---@class {alias}")
            lines.append(f"local {alias} = {{}}\n")
            lines.append(f"---@alias {cstruct} {alias}\n")

    # Emit table classes (generic)
    for t in s.get("tables", []):
        cname = class_name_from_leaf(t)
        lines.append(f"---@class {cname}")
        lines.append(f"---@field __unknown any")
        lines.append(f"local {cname} = {{}}\n")
        lines.append(f"---@alias {t} {cname}\n")

    # Emit metamethods detected for aliases
    for alias, mm_list in metamethods.items():
        for mname, mparams in mm_list:
            # operator annotations (helpful)
            if mname == "__add":
                lines.append(f"---@operator add({alias}, {alias}): {alias}")
            if mname == "__sub":
                lines.append(f"---@operator sub({alias}, {alias}): {alias}")
            if mname == "__mul":
                lines.append(f"---@operator mul({alias}, number): {alias}")
                lines.append(f"---@operator mul(number, {alias}): {alias}")
            if mname == "__div":
                lines.append(f"---@operator div({alias}, number): {alias}")
            if mname == "__unm":
                lines.append(f"---@operator unm({alias}): {alias}")
            if mname == "__eq":
                lines.append(f"---@operator eq({alias}, {alias}): boolean")
            # function stub for metamethod; ensure return annotation if inferred
            ret = "any"
            lookup_name = f"{alias}.__{mname.strip('_')}"
            if lookup_name in func_returns:
                ret = func_returns[lookup_name]
            else:
                if mname in ("__add", "__sub", "__mul", "__div", "__unm"):
                    ret = alias
            params_str = ", ".join(mparams) if mparams else ""
            for p in mparams:
                lines.append(f"---@param {p} any")
            lines.append(f"---@return {ret}")
            # write a function stub using alias.metaname (e.g., LuaVec3.__add)
            # note: use name with two underscores as in source
            lines.append(f"function {alias}.{mname}({params_str}) end\n")

    # Emit functions with inferred returns
    for fname, params, is_method, is_local in s.get("functions", []):
        ret = "any"
        if fname in func_returns:
            ret = func_returns[fname]
        else:
            # heuristic: if name contains vec3 -> LuaVec3
            if "vec3" in (fname or "").lower():
                for cstruct, alias in ffi_map.items():
                    if "vec3" in cstruct.lower() or "vec3" in alias.lower():
                        ret = alias
                        break
        for p in params:
            lines.append(f"---@param {p} any")
        lines.append(f"---@return {ret}")
        params_str = ", ".join(params)
        lines.append(f"function {fname}({params_str}) end\n")

    if len(lines) == 1:
        lines.append("-- (no defs found)\n")
    return "\n".join(lines)


# ---------------- vscode settings update ----------------
def update_vscode_settings(
    workspace_root: Path, stubs_dir: Path, beamng_dir: Path
) -> Path:
    settings_path = workspace_root.joinpath(".vscode", "settings.json")
    settings_path.parent.mkdir(parents=True, exist_ok=True)
    data: Dict[str, Any] = {}
    if settings_path.exists():
        try:
            data = json.loads(settings_path.read_text(encoding="utf-8"))
        except Exception:
            data = {}
    libs = data.get("Lua.workspace.library", [])
    s_stubs = str(stubs_dir)
    s_beamng = str(beamng_dir)
    if s_stubs not in libs:
        libs.insert(0, s_stubs)
    if s_beamng not in libs:
        libs.append(s_beamng)
    data["Lua.workspace.library"] = libs
    pri = data.get("Lua.workspace.libraryPriority", {})
    pri[s_beamng] = max(pri.get(s_beamng, 20), 20)
    pri[s_stubs] = min(pri.get(s_stubs, 5), 5)
    wk_lua = str(Path(workspace_root).joinpath("lua").resolve())
    pri[wk_lua] = min(pri.get(wk_lua, 1), 1)
    data["Lua.workspace.libraryPriority"] = pri
    data.setdefault("Lua.diagnostics.libraryFiles", "Disable")
    settings_path.write_text(json.dumps(data, indent=2), encoding="utf-8")
    return settings_path


# ---------------- manifest utils ----------------
def load_manifest(out_root: Path) -> Dict[str, str]:
    mf = out_root.joinpath(".manifest.json")
    if not mf.exists():
        return {}
    try:
        return json.loads(mf.read_text(encoding="utf-8"))
    except Exception:
        return {}


def save_manifest(out_root: Path, manifest: Dict[str, str]):
    mf = out_root.joinpath(".manifest.json")
    mf.write_text(json.dumps(manifest, indent=2), encoding="utf-8")


# ---------------- process ----------------
def process(
    beamng_root: Path,
    out_root: Path,
    workspace_root: Path,
    skip_settings_update: bool = False,
) -> Tuple[int, int]:
    beamng_root = beamng_root.resolve()
    out_root = out_root.resolve()
    workspace_root = workspace_root.resolve()
    ensure_dir(out_root)
    manifest = load_manifest(out_root)
    new_manifest: Dict[str, str] = {}
    total = 0
    changed = 0
    for src in beamng_root.rglob("*.lua"):
        total += 1
        rel = str(src.relative_to(beamng_root))
        try:
            code = safe_read_text(src)
            extractor = BeamNGExtractor(code)
            if extractor.tree is None:
                log(
                    f"[WARN] parse error {src}: {getattr(extractor, '_parse_error', '')}"
                )
                out_file = out_root.joinpath(rel)
                ensure_dir(out_file.parent)
                stub = f"-- parse error for {src}\n-- generator skipped\n"
                if write_if_changed(out_file, stub):
                    changed += 1
                new_manifest[rel] = str(src.stat().st_mtime)
                continue
            extractor.extract()
            stub_text = render_stub_for_file(src, beamng_root, out_root, extractor)
            out_file = out_root.joinpath(rel)
            ensure_dir(out_file.parent)
            if write_if_changed(out_file, stub_text):
                changed += 1
            new_manifest[rel] = str(src.stat().st_mtime)
        except Exception as e:
            log(f"[ERROR] processing {src}: {e}")
            continue
    save_manifest(out_root, new_manifest)

    # add gitignore entry
    gi = workspace_root.joinpath(".gitignore")
    relout = os.path.relpath(out_root, workspace_root)
    if gi.exists():
        txt = gi.read_text(encoding="utf-8")
        if relout not in txt:
            with gi.open("a", encoding="utf-8") as f:
                f.write(f"\n# autogenerated beamng stubs\n{relout}\n")
    else:
        gi.write_text(f"# autogenerated beamng stubs\n{relout}\n", encoding="utf-8")

    if not skip_settings_update:
        sp = update_vscode_settings(workspace_root, out_root, beamng_root)
        log(f"Updated settings: {sp}")
    return total, changed


# ---------------- main ----------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--beamng",
        "-b",
        default=os.path.expanduser(
            "~/ .local/share/Steam/steamapps/common/BeamNG.drive/lua".replace(" ", "")
        ),
        help="Path to BeamNG.lua root (default: ~/.local/share/Steam/steamapps/common/BeamNG.drive/lua)",
    )
    ap.add_argument(
        "--out", "-o", default=".vscode/beamng_stubs", help="Output stubs directory"
    )
    ap.add_argument(
        "--workspace",
        "-w",
        default=".",
        help="Workspace root (for settings.json updates)",
    )
    ap.add_argument(
        "--skip-update-settings",
        action="store_true",
        help="Don't edit .vscode/settings.json",
    )
    args = ap.parse_args()
    beamng_root = Path(args.beamng).expanduser().resolve()
    out_root = Path(args.out).resolve()
    workspace_root = Path(args.workspace).resolve()
    if not beamng_root.exists():
        log(f"ERROR: beamng path not found: {beamng_root}")
        sys.exit(2)
    total, changed = process(
        beamng_root,
        out_root,
        workspace_root,
        skip_settings_update=args.skip_update_settings,
    )
    log(f"Processed {total} files; wrote/updated {changed} stubs into: {out_root}")


if __name__ == "__main__":
    main()
