#!/usr/bin/env python3
"""Rewrite absolute symlinks in a sysroot so they resolve inside it.

A link like usr/lib/aarch64-linux-gnu/libblas.so.3 -> /etc/alternatives/libblas.so.3-aarch64-linux-gnu
would point at the build host's /etc. Each absolute target is resolved inside the sysroot (following
chains of absolute links, e.g. through /etc/alternatives) and replaced by a relative link.
"""
import os
import sys


def resolve_in_root(root, target, depth=0):
    """Return the final in-sysroot path for an absolute link target (follows absolute links only)."""
    path = os.path.join(root, target.lstrip("/"))
    if depth < 20 and os.path.islink(path):
        nxt = os.readlink(path)
        if nxt.startswith("/"):
            return resolve_in_root(root, nxt, depth + 1)
    return path


def main():
    root = os.path.abspath(sys.argv[1])
    fixed = dangling = 0
    for dirpath, dirnames, filenames in os.walk(root):
        # don't descend into pseudo filesystems
        if dirpath == root:
            dirnames[:] = [d for d in dirnames if d not in ("proc", "sys", "dev", "run")]
        for name in dirnames + filenames:
            link = os.path.join(dirpath, name)
            if not os.path.islink(link):
                continue
            target = os.readlink(link)
            if not target.startswith("/"):
                continue
            real = resolve_in_root(root, target)
            rel = os.path.relpath(real, dirpath)
            os.remove(link)
            os.symlink(rel, link)
            fixed += 1
            if not os.path.lexists(real):
                dangling += 1
    print(f"rewrote {fixed} absolute symlinks ({dangling} point at files missing in the sysroot)")


if __name__ == "__main__":
    main()
